
# Edge-Native ADAS Smart-Trimmer & Cloud Replay Pipeline

A zero-copy C++20 edge middleware prototype designed to eliminate cloud ingestion bottlenecks for ADAS and Autonomous Driving fleets.

1. Executive Summary & Problem Statement

Automotive test and L2+ customer fleets generate terabytes of multi-sensor data daily across cameras, LiDAR, radar, and vehicle CAN buses. Uploading, storing, and annotating 100% of raw highway miles is financially unsustainable and buries engineering teams in uninformative logs.
The Edge-Native ADAS Smart-Trimmer runs directly on the vehicle's compute platform. It buffers multi-modal sensor streams inside a strict byte-bounded memory ring, continuously monitors kinematics and 3D perception track continuity, and extracts only high-value anomaly windows (e.g., hard braking, phantom braking, or sudden 3D track dropouts) into compact, Foxglove-ready .mcap containers for edge-to-cloud uplink.

Key Value Metrics

• Bandwidth & Cloud Storage Reduction: Cuts cloud data ingestion and annotation costs by 80% to 98%+ by discarding nominal driving at the edge.
• Deterministic Edge Footprint: Operates inside a fixed memory cap (e.g., 512 MB ring cache) using zero-copy std::shared_ptr semantics and streaming Apache Parquet cursors.
• Industry-Standard Visualization: Outputs spec-compliant .mcap containers compatible with Foxglove Studio and the included Streamlit OEM ROI & Replay Console.

2. System Architecture
```text
┌────────────────────────────────────────────────────────────────────────────┐
│                        VEHICLE EDGE NODE (C++20)                           │
│                                                                            │
│  [Waymo v2 Parquet Streams]                                                │
│   ├── vehicle_pose  ──┐                                                    │
│   ├── lidar_box     ──┼──► [Lockstep Multi-Table Joiner]                   │
│   ├── camera_image  ──┤             │                                      │
│   └── lidar         ──┘             ▼                                      │
│                           [Anomaly Trigger Engine]                         │
│                            ├── Kinematic Deceleration (< -4.5 m/s²)        │
│                            └── 3D Track Continuity (< 35m safety zone)     │
│                                     │                                      │
│                                     ▼                                      │
│                        [CircularFlashStorage Ring]                         │
│                        (Pre-Trigger + Post-Trigger)                        │
│                                     │ (On Trigger Complete)                │
│                                     ▼                                      │
│                       [NativeMcapWriter (.mcap)]                           │
│                                     │                                      │
└─────────────────────────────────────┼──────────────────────────────────────┘
                                      │ POSIX TCP Uplink (127.0.0.1:7447)
                                      ▼
┌────────────────────────────────────────────────────────────────────────────┐
│                       CLOUD INGESTION & ANALYTICS                          │
│                                                                            │
│   [cloud_receiver.py] ──► [./cloud_bucket/*.mcap]                          │
│                                     │                                      │
│                        ┌────────────┴────────────┐                         │
│                        ▼                         ▼                         │
│          [Streamlit ROI & Replay]       [Foxglove Studio 3D]               │
│          (dashboard.py)                 (adas_trimmer_layout.json)         │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```
# Core Software Components


Component	File	Responsibility
Multi-Table Joiner	src/main.cpp	Streams vehicle_pose, lidar_box, camera_image, and lidar Parquet files in chronological lockstep with flat RAM usage.
Track Continuity Monitor	src/main.cpp	Compares 3D LiDAR object IDs across consecutive frames (t-1 vs. t) to detect sudden perception dropouts within a 35-meter radius.
Circular Flash Storage	src/main.cpp	Enforces strict byte and frame ceilings (max_flash_mb, window_sec), automatically evicting oldest frames until a trigger fires.
Native MCAP Writer	src/main.cpp	Zero-dependency MCAP v0 binary writer serializing telemetry, foxglove.SceneUpdate 3D cubes, foxglove.CompressedImage JPEGs, and foxglove.Log diagnostics.
Cloud Uplink Receiver	cloud_receiver.py	Lightweight TCP ingestion daemon receiving trimmed .mcap slices from the edge node into ./cloud_bucket/.
OEM ROI & Replay Console	dashboard.py	Interactive Streamlit web application calculating annual fleet cost savings and replaying 3D bird's-eye-view trajectories and dashcam frames.


3. Directory Structure

```text
Native-Edge-Data-Trimmer/
├── CMakeLists.txt               # C++20 build configuration linking Arrow & Parquet
├── src/
│   └── main.cpp                 # Core edge trimmer, Parquet joiners, MCAP writer, socket uplink
├── data/
│   └── waymo_v2/                # Modular Waymo Open Dataset v2 Parquet inputs
│       ├── vehicle_pose/
│       ├── lidar_box/
│       ├── camera_image/
│       └── lidar/
├── trimmed_events/              # Local edge cache of triggered .mcap slices
├── cloud_bucket/                # Cloud-ingested .mcap slices received by cloud_receiver.py
├── cloud_receiver.py            # Cloud ingestion service (listens on 127.0.0.1:7447)
├── dashboard.py                 # Streamlit executive ROI calculator & event replay viewer
└── adas_trimmer_layout.json     # Pre-configured 3-panel Foxglove Studio layout template
```

4. Prerequisites & Installation


System Dependencies (Ubuntu 22.04 LTS)

1. Apache Arrow & Parquet C++ Libraries:bash
wget https://apache.org
sudo apt install -y ./apache-arrow-apt-source-latest-jammy.deb
sudo apt update
sudo apt install -y libarrow-dev libparquet-dev cmake g++-13

2. Python Dashboard Dependencies:bash
python3 -m venv .venv
source .venv/bin/activate
pip install streamlit pandas plotly

3. Dataset Preparation:
Place matching Waymo Open Dataset v2 segment .parquet files (identical filenames) into ./data/waymo_v2/vehicle_pose/ and ./data/waymo_v2/lidar_box/ (and optionally camera_image/ and lidar/).

5. Build & Execution Guide


Step 1: Compile the C++20 Edge Binary

bash
mkdir -p build && cd build
cmake -DCMAKE_CXX_COMPILER=g++-13 ..
make -j$(nproc)
cd ..
Use code with caution.

Step 2: Start the Cloud Ingestion Receiver (Terminal 1)

bash
python3 cloud_receiver.py
Use code with caution.

Step 3: Launch the Streamlit ROI & Replay Console (Terminal 2)

bash
streamlit run dashboard.py
Use code with caution.

Step 4: Run the Edge Smart-Trimmer (Terminal 3)

• Standard Pass-Through Mode (scans real logs for organic hard braking or 3D track dropouts):bash
./build/waymo_edge_trimmer
Use code with caution.
• Synthetic Fault Injection Mode (deterministically injects a -6.8 m/s² phantom brake and drops a tracked 3D vehicle ID at frame 100 for live demos):bash
./build/waymo_edge_trimmer --inject-fault
Use code with caution.

6. MCAP Telemetry Channels & Visualization

Each exported .mcap event slice contains five synchronized topics:
```text
# Topic Name	            |     Schema Type	            |       Content Description
/vehicle/telemetry	        | waymo.Telemetry (JSON)	    |  Ego longitudinal speed (m/s), acceleration (m/s²)      
                            |                               |    , active track count, and anomaly_triggered pulse (0/1).
/perception/scene_3d        |   foxglove.SceneUpdate        |	Oriented 3D bounding boxes for the blue ego vehicle and surrounding 
                            |                               |    traffic (flashes red on trigger frames).
/perception/tracked_objects	| waymo.TrackedObjects (JSON)	| Raw object coordinates, velocities, and dropped_ids list for Python/
                            |                               |  Streamlit replay.
/camera/front/compressed    | foxglove.CompressedImage	    | Base64-encoded front-facing camera JPEG stream synchronized to each 
                            |                               |    timestamp.
/diagnostics/anomaly_log    |	foxglove.Log	            | Human-readable trigger root-cause logs (level: 4 ERROR on anomaly 
                            |                               |    frames).
```                          


Visualizing in Foxglove Studio

1. Open Foxglove Studio and drag any .mcap file from ./cloud_bucket/ or ./trimmed_events/ onto the window.
2. Click Layout -> Import from file… and select adas_trimmer_layout.json.
3. Press Play (Spacebar) to inspect the synchronized dashcam video, 3D LiDAR bounding boxes (base_link), and kinematic deceleration spike.

7. Configuration & Tuning Parameters

Adjust these parameters inside src/main.cpp to match target hardware or demo scenarios:
• max_flash_mb (Default: 512): Maximum RAM/NVMe flash footprint allocated to the circular buffer.
• window_sec (Default: 30 or 6): Total context duration captured per anomaly (window_sec / 2 pre-trigger + window_sec / 2 post-trigger). Set to 6 seconds when benchmarking single 20-second Waymo clips.
• decel_threshold_ (Default: -4.5f m/s²): Longitudinal deceleration threshold for flagging emergency or phantom braking events.
• critical_radius_meters_ (Default: 35.0 m): Safety bubble radius around the ego vehicle used by TrackContinuityMonitor to filter out benign far-field track exits.


