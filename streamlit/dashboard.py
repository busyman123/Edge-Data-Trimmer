import base64
import glob
import io
import json
import os
import re
import struct
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import streamlit as st

st.set_page_config(page_title="ADAS Cloud Ingestion & OEM ROI Console", layout="wide")


def sanitize_json_numbers(raw_str: str) -> str:
    """Replaces non-standard C++ nan/inf literals with valid JSON null."""
    return re.sub(r'(?<=[:\[,])\s*-?(?:nan|inf|infinity)\b', "null", raw_str, flags=re.IGNORECASE)


def parse_native_mcap(filepath: str):
    """Parses records written by NativeMcapWriter safely, including telemetry, 3D scene, logs, and camera."""
    channels = {}
    telemetry_rows = []
    object_rows = []
    log_rows = []
    camera_frames = {}

    with open(filepath, "rb") as f:
        magic = f.read(8)
        if magic != b"\x89MCAP0\r\n":
            raise ValueError(f"Invalid MCAP magic header in {filepath}")

        while True:
            opcode_byte = f.read(1)
            if not opcode_byte:
                break
            opcode = opcode_byte[0]
            len_bytes = f.read(8)
            if len(len_bytes) < 8:
                break
            body_len = struct.unpack("<Q", len_bytes)[0]
            body = f.read(body_len)

            if opcode == 0x04:  # Channel record
                ch_id, _schema_id, topic_len = struct.unpack("<HHI", body[:8])
                topic = body[8 : 8 + topic_len].decode("utf-8")
                channels[ch_id] = topic

            elif opcode == 0x05:  # Message record
                ch_id, seq, log_time_ns, _pub_time_ns = struct.unpack("<HIQQ", body[:22])
                raw_text = body[22:].decode("utf-8")
                payload = json.loads(sanitize_json_numbers(raw_text))
                topic = channels.get(ch_id, "")
                ts_sec = log_time_ns / 1e9

                if topic == "/vehicle/telemetry":
                    telemetry_rows.append(
                        {
                            "seq": seq,
                            "time_sec": ts_sec,
                            "speed_mps": payload.get("speed_mps") or 0.0,
                            "accel_x_mps2": payload.get("accel_x_mps2") or 0.0,
                            "active_tracks": payload.get("active_tracks") or 0,
                            "anomaly_triggered": payload.get("anomaly_triggered") or 0,
                        }
                    )
                elif topic == "/perception/tracked_objects":
                    object_rows.append(
                        {
                            "time_sec": ts_sec,
                            "dropped_ids": payload.get("dropped_ids") or [],
                            "objects": payload.get("objects") or [],
                        }
                    )
                elif topic == "/perception/scene_3d" and not object_rows:
                    # Fallback if only scene_3d was written to MCAP
                    entities = [
                        e for e in (payload.get("entities") or []) if e.get("id") != "ego_vehicle"
                    ]
                    objs = []
                    for ent in entities:
                        cubes = ent.get("cubes") or []
                        pos = cubes[0].get("pose", {}).get("position", {}) if cubes else {}
                        objs.append(
                            {
                                "id": ent.get("id", "unknown"),
                                "x": pos.get("x", 0.0),
                                "y": pos.get("y", 0.0),
                            }
                        )
                    object_rows.append({"time_sec": ts_sec, "dropped_ids": [], "objects": objs})

                elif topic == "/diagnostics/anomaly_log":
                    log_rows.append(
                        {
                            "time_sec": ts_sec,
                            "level": "ERROR" if payload.get("level") == 4 else "INFO",
                            "message": payload.get("message", ""),
                        }
                    )
                elif topic == "/camera/front/compressed":
                    b64_data = payload.get("data", "")
                    if b64_data:
                        camera_frames[round(ts_sec, 2)] = base64.b64decode(b64_data)

            elif opcode == 0x02:  # Footer
                break

    df_telem = pd.DataFrame(telemetry_rows)
    if not df_telem.empty:
        df_telem["rel_time_s"] = df_telem["time_sec"] - df_telem["time_sec"].iloc[0]
    return df_telem, object_rows, log_rows, camera_frames


# ==========================================
# SIDEBAR: OEM FLEET ROI SIMULATOR CONTROLS
# ==========================================
st.sidebar.header("🚗 OEM Fleet ROI Parameters")
fleet_size = st.sidebar.slider("Active Test / L2+ Fleet Size (Vehicles)", 10, 500, 100, step=10)
raw_gb_per_day = st.sidebar.number_input("Raw Data per Vehicle / Day (GB)", value=450.0)
reduction_pct = st.sidebar.slider("Edge Smart-Trimmer Data Reduction (%)", 50, 99, 98)
cloud_cost_per_gb = st.sidebar.number_input("Cloud Ingestion + Storage + Labeling ($/GB)", value=0.18)

annual_raw_tb = (fleet_size * raw_gb_per_day * 365) / 1024.0
annual_trimmed_tb = annual_raw_tb * (1.0 - reduction_pct / 100.0)
annual_gross_savings = (annual_raw_tb - annual_trimmed_tb) * 1024.0 * cloud_cost_per_gb

st.title("☁️ ADAS Edge-to-Cloud Ingestion & OEM ROI Console")
st.caption("Connected directly to `./cloud_bucket/` • Live Edge-Trimmed `.mcap` Telemetry")

# ==========================================
# TOP KPI BANNER (EXECUTIVE ROI)
# ==========================================
k1, k2, k3, k4 = st.columns(4)
k1.metric("Annual Raw Fleet Data", f"{annual_raw_tb:,.1f} TB", "Baseline Unfiltered")
k2.metric("Cloud-Ingested Edge Slices", f"{annual_trimmed_tb:,.1f} TB", f"-{reduction_pct}% Bandwidth")
k3.metric("Projected Annual OEM Savings", f"${annual_gross_savings:,.0f}", "Cloud & Annotation ROI")
k4.metric("Edge Compute Footprint", "< 3.2% CPU", "512 MB NVMe Ring Cache")

st.divider()

# ==========================================
# CLOUD BUCKET SELECTOR & REFRESH
# ==========================================
col_sel, col_btn = st.columns([4, 1])
with col_btn:
    st.write("")
    if st.button("🔄 Refresh Bucket", use_container_width=True):
        st.rerun()

# Search ./cloud_bucket/ first (also check ../cloud_bucket/ if launched from build/streamlit/)
bucket_patterns = [
    "./cloud_bucket/*.mcap",
    "../cloud_bucket/*.mcap",
    "../../cloud_bucket/*.mcap",
]
mcap_files = sorted(
    [p for pat in bucket_patterns for p in glob.glob(pat)],
    key=os.path.getmtime,
    reverse=True,  # Show newest cloud-ingested event at top
)

if not mcap_files:
    st.warning(
        "No `.mcap` event slices found in `./cloud_bucket/`. "
        "Start `python3 cloud_receiver.py` and run `./build/waymo_edge_trimmer --inject-fault`."
    )
    st.stop()

with col_sel:
    selected_file = st.selectbox(
        f"☁️ Select Cloud-Ingested Anomaly Slice ({len(mcap_files)} event(s) in bucket)",
        mcap_files,
        format_func=lambda p: f"{os.path.basename(p)}  ({os.path.getsize(p)/1024.0:.1f} KB)",
    )

df_telem, object_frames, log_rows, camera_frames = parse_native_mcap(selected_file)

# Display Anomaly Diagnostic Banner if ERROR logs exist
error_logs = [r for r in log_rows if r["level"] == "ERROR"]
if error_logs:
    st.error(f"🚨 **Trigger Root Cause:** `{error_logs[0]['message']}`")

if df_telem.empty:
    st.error("Selected MCAP file contains no telemetry frames.")
    st.stop()

# ==========================================
# SYNCHRONIZED REPLAY: TELEMETRY + 3D BEV + CAMERA
# ==========================================
col_left, col_right = st.columns([1.15, 1.0])

with col_left:
    st.subheader("📉 Ego Vehicle Kinematics & Trigger Pulse")
    plot_cols = [c for c in ["speed_mps", "accel_x_mps2", "anomaly_triggered"] if c in df_telem.columns]
    fig_accel = px.line(
        df_telem,
        x="rel_time_s",
        y=plot_cols,
        labels={"rel_time_s": "Slice Window Time (s)", "value": "Value", "variable": "Signal"},
    )
    fig_accel.add_hline(y=-4.5, line_dash="dash", line_color="red", annotation_text="Braking Threshold (-4.5 m/s²)")
    st.plotly_chart(fig_accel, use_container_width=True)

    # Show Front Camera Frame if embedded in MCAP
    if camera_frames and object_frames:
        st.subheader("📷 Front Dashcam Frame at Scrubbed Timestamp")
        frame_idx_cam = st.session_state.get("frame_scrubber", len(object_frames) // 2)
        target_ts = round(object_frames[frame_idx_cam]["time_sec"], 2)
        nearest_ts = min(camera_frames.keys(), key=lambda t: abs(t - target_ts))
        st.image(io.BytesIO(camera_frames[nearest_ts]), caption=f"Front Camera @ {nearest_ts:.2f}s", use_container_width=True)

with col_right:
    st.subheader("📡 3D LiDAR Track Continuity Replay (Bird's-Eye View)")
    if object_frames:
        frame_idx = st.slider(
            "Scrub Frame Index",
            0,
            len(object_frames) - 1,
            len(object_frames) // 2,
            key="frame_scrubber",
        )
        current_obj_frame = object_frames[frame_idx]
        dropped = current_obj_frame["dropped_ids"]

        if dropped:
            st.error(f"🚨 PERCEPTION TRACK DROPOUT: IDs `{dropped}` vanished inside 35m safety zone!")
        else:
            st.success(f"✅ Active 3D Tracks: {len(current_obj_frame['objects'])} | Nominal Tracking")

        fig_bev = go.Figure()
        fig_bev.add_trace(
            go.Scatter(
                x=[0],
                y=[0],
                mode="markers+text",
                marker=dict(size=16, color="blue", symbol="triangle-up"),
                text=["EGO"],
                textposition="bottom center",
                name="Ego Vehicle",
            )
        )
        fig_bev.add_shape(
            type="circle",
            xref="x",
            yref="y",
            x0=-35,
            y0=-35,
            x1=35,
            y1=35,
        line=dict(color="red", dash="dot"),
)
objs = current_obj_frame["objects"]
if objs:
    fig_bev.add_trace(
        go.Scatter(
            x=[-o["y"] for o in objs],
            y=[o["x"] for o in objs],
            mode="markers+text",
            marker=dict(size=11, color="orange"),
            text=[o["id"][:6] for o in objs],
            textposition="top center",
            name="3D LiDAR Tracks",
        )
    )
fig_bev.update_layout(
    xaxis=dict(title="Lateral Distance (m)", range=[-50, 50]),
    yaxis=dict(title="Longitudinal Distance (m)", range=[-20, 70], scaleanchor="x", scaleratio=1),
    height=460,
    margin=dict(l=20, r=20, t=30, b=20),
)
st.plotly_chart(fig_bev, use_container_width=True)
