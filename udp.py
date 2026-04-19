import socket
import json

UDP_IP = "0.0.0.0"
UDP_PORT = 65000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(2)   # wait 2 seconds, then continue

print(f"Listening on UDP {UDP_PORT}...", flush=True)

while True:
    try:
        data, addr = sock.recvfrom(65535)
        print(f"\nPacket from {addr}", flush=True)

        try:
            text = data.decode("utf-8", errors="ignore")
            print("Raw:", text, flush=True)

            msg = json.loads(text)
            print("seq:", msg.get("seq"), flush=True)
            print("roll:", msg.get("roll"), flush=True)
            print("pitch:", msg.get("pitch"), flush=True)
            print("yaw:", msg.get("yaw"), flush=True)
            print("accel:", msg.get("accelX"), msg.get("accelY"), msg.get("accelZ"), flush=True)
            print("gyro:", msg.get("gyroX"), msg.get("gyroY"), msg.get("gyroZ"), flush=True)
            print("mag:", msg.get("magX"), msg.get("magY"), msg.get("magZ"), flush=True)
            print("location:", msg.get("latitude"), msg.get("longitude"), flush=True)

        except Exception as e:
            print("JSON parse error:", e, flush=True)

    except socket.timeout:
        print("Still waiting for data...", flush=True)