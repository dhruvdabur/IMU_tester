# IMU Sensor Suite

Expo Go app and local Node receiver for comparing phone IMU data with the STM32/BNO055 stream. The app keeps the original JSON forwarding flow and adds dashboard, 3D orientation, stream controls, settings/export preview, CSV packet mode, and a JSON inspector.

## Run

1. Start the JSON host on the computer:

   ```powershell
   cd C:\Users\bront\Downloads\BNO_IMU
   npm run server
   ```

   The receiver samples the latest phone and BNO055 packets onto one synchronized comparison clock. The default is `50 Hz`. To use another shared output rate:

   ```powershell
   $env:SYNC_RATE_HZ=100
   npm run server
   ```

2. Note the LAN address printed by the server, for example:

   ```text
   Expo app POST URL: http://192.168.4.117:65000/imu
   Visualizer URL:     http://192.168.4.117:65000/visualizer
   Drift JSON URL:     http://192.168.4.117:65000/drift.json
   ```

3. Start Expo:

   ```powershell
   cd C:\Users\bront\Downloads\BNO_IMU
   npm start
   ```

4. Open the QR code in Expo Go. Enter the computer URL in the app, then press **Start**.

5. Flash the STM32 firmware, then bridge the BNO055 USB serial JSON into the host:

   ```powershell
   py -m pip install pyserial
   py scripts\bno055_serial_bridge.py COM5
   ```

   Replace `COM5` with the STM32 virtual COM port shown in Device Manager.

6. Open the visualizer in a browser:

   ```text
   http://192.168.4.117:65000/visualizer
   ```

   Open the research page for recording statistics:

   ```text
   http://192.168.4.117:65000/analysis
   ```

The phone and computer must be on the same network. If the app cannot POST to the server, allow Node.js through Windows Firewall for private networks.

## App features

- **Home**: live yaw/pitch/roll banner, accelerometer/gyroscope/magnetometer cards, rolling mini charts, magnitude, min/max/mean statistics, and orientation preview.
- **3D**: cube-style orientation view, yaw/pitch/roll gauges, quaternion, and rotation matrix.
- **Stream**: host URL, sample rate up to 200 Hz, JSON or CSV packet format, packets sent, errors, and current endpoint links.
- **Settings**: sensor availability, current rate, CSV snapshot, compact JSON snapshot, and receiver notes.
- **JSON**: full raw payload preview.

## Endpoints

- `POST /imu` accepts JSON from the Expo app.
- `POST /imu` also accepts 17-field CSV packets:
  `timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,yaw,pitch,roll`.
- `POST /bno055` accepts JSON from the STM32/BNO055 serial bridge.
- `GET /visualizer` opens the live drift visualizer.
- `GET /analysis` opens the research/statistics dashboard.
- `GET /drift.json` returns phone vs BNO055 roll/pitch/yaw drift.
- `GET /phone.json` returns the latest phone packet.
- `GET /bno055.json` returns the latest BNO055 packet.
- `GET /imu.json` returns the latest packet from any source.
- `GET /history.json?limit=50` returns recent packets.
- `GET /analysis.json` returns current recording statistics.
- `POST /analysis/start`, `POST /analysis/stop`, and `POST /analysis/clear` control the recording buffer.
- `GET /` opens the visualizer.
- UDP JSON and CSV packets are also accepted on port `65000`, so an existing UDP IMU sender can feed the same `/imu.json` endpoint.

## Drift

The STM32 firmware now prints one BNO055 JSON object per line over USB CDC. The bridge script forwards those lines to the Node host. The drift page compares:

```text
drift = phone Euler angle - BNO055 Euler angle
```

Yaw drift wraps at `+/-180` degrees so crossing north does not create a false 360-degree spike.

The drift visualizer does not plot packets at their raw arrival rate. It samples both sources together from their latest values at `SYNC_RATE_HZ`, so the phone and BNO055 traces have the same number of points and the same time spacing.

## Expo Go limitation

Expo Go cannot host its own HTTP or UDP server on the phone, and UDP sending requires native modules that are not available in Expo Go. This app samples the phone sensors in Expo Go, then sends JSON or CSV over HTTP to the included Node host. The host accepts UDP JSON/CSV from other native senders and makes the data available through browser dashboards and JSON endpoints.
