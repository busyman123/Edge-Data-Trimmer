import os
import socket
import struct

os.makedirs("./cloud_bucket", exist_ok=True)


def recv_exact(conn: socket.socket, num_bytes: int) -> bytes:
    buf = bytearray()
    while len(buf) < num_bytes:
        chunk = conn.recv(num_bytes - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed before receiving expected bytes")
        buf.extend(chunk)
    return bytes(buf)


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 7447))
        srv.listen(5)
        print("☁️  Cloud Ingestion Receiver listening on 127.0.0.1:7447...")
        print("📂 Saving incoming ADAS anomaly slices to ./cloud_bucket/\n")

        while True:
            conn, addr = srv.accept()
            with conn:
                try:
                    (name_len,) = struct.unpack("!I", recv_exact(conn, 4))
                    filename = os.path.basename(recv_exact(conn, name_len).decode("utf-8"))
                    (data_len,) = struct.unpack("<Q", recv_exact(conn, 8))
                    payload = recv_exact(conn, data_len)

                    out_path = os.path.join("./cloud_bucket", filename)
                    with open(out_path, "wb") as f:
                        f.write(payload)

                    print(f"✅ [CLOUD INGEST] Received {len(payload)/1024:.1f} KB from {addr} -> {out_path}")
                except Exception as exc:
                    print(f"⚠️ [CLOUD INGEST ERROR] {exc}")


if __name__ == "__main__":
    main()
