import json
import sys
import time
import urllib.error
import urllib.request

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing pyserial. Install it with:")
    print("  py -m pip install pyserial")
    sys.exit(1)


DEFAULT_URL = "http://127.0.0.1:65000/bno055"
DEFAULT_BAUD = 115200
POST_LOG_INTERVAL_SEC = 1.0


def available_ports():
    return [f"{port.device} - {port.description}" for port in list_ports.comports()]


def post_json(url, payload):
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=2) as response:
        response.read()


def extract_json(line):
    start = line.find("{")
    end = line.rfind("}")
    if start < 0 or end <= start:
        return None
    return json.loads(line[start : end + 1])


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  py scripts\\bno055_serial_bridge.py COM5 [http://127.0.0.1:65000/bno055]")
        print("")
        print("Available serial ports:")
        ports = available_ports()
        print("\n".join(f"  {port}" for port in ports) if ports else "  none")
        return 2

    port = sys.argv[1]
    url = sys.argv[2] if len(sys.argv) >= 3 else DEFAULT_URL

    print(f"Reading BNO055 JSON from {port} at {DEFAULT_BAUD} baud", flush=True)
    print(f"Posting to {url}", flush=True)

    while True:
        try:
            with serial.Serial(port, DEFAULT_BAUD, timeout=1) as ser:
                ser.dtr = True
                ser.rts = True
                last_heartbeat = time.monotonic()
                last_post_log = 0.0

                while True:
                    raw = ser.readline()
                    if not raw:
                        now = time.monotonic()
                        if now - last_heartbeat >= 5:
                            print("waiting for BNO055 serial JSON...", flush=True)
                            last_heartbeat = now
                        continue

                    line = raw.decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    try:
                        payload = extract_json(line)
                        if payload is None:
                            print(f"skip: {line}", flush=True)
                            continue

                        payload["source"] = "bno055"
                        post_json(url, payload)
                        now = time.monotonic()
                        if now - last_post_log >= POST_LOG_INTERVAL_SEC:
                            last_post_log = now
                            seq = payload.get("seq", "?")
                            euler = payload.get("euler", {})
                            print(
                                f"posted seq={seq} "
                                f"roll={euler.get('roll')} "
                                f"pitch={euler.get('pitch')} "
                                f"yaw={euler.get('yaw')}",
                                flush=True,
                            )
                    except (json.JSONDecodeError, urllib.error.URLError, TimeoutError) as error:
                        print(f"error: {error}", flush=True)
                        time.sleep(0.25)
        except serial.SerialException as error:
            print(f"serial error: {error}", flush=True)
            print("reconnecting in 2 seconds...", flush=True)
            time.sleep(2)


if __name__ == "__main__":
    raise SystemExit(main())
