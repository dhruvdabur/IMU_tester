import { StatusBar } from 'expo-status-bar';
import {
  Accelerometer,
  DeviceMotion,
  Gyroscope,
  Magnetometer,
  type AccelerometerMeasurement,
  type DeviceMotionMeasurement,
  type GyroscopeMeasurement,
  type MagnetometerMeasurement,
} from 'expo-sensors';
import { useEffect, useMemo, useRef, useState } from 'react';
import {
  Pressable,
  SafeAreaView,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native';

type Vector3 = {
  x: number | null;
  y: number | null;
  z: number | null;
};

type Quaternion = {
  w: number;
  x: number;
  y: number;
  z: number;
};

type ImuPayload = {
  version: 1;
  source: 'expo-go';
  seq: number;
  generatedAt: string;
  sampleHz: number;
  euler: {
    roll: number;
    pitch: number;
    yaw: number;
    units: 'deg';
    convention: string;
  };
  quaternion: Quaternion;
  rotationMatrix: number[][];
  accelerometer: Vector3 & { units: 'g'; timestamp: number | null };
  acceleration: Vector3 & { units: 'm/s^2'; timestamp: number | null };
  gyroscope: Vector3 & { units: 'rad/s'; timestamp: number | null };
  rotationRate: Vector3 & { units: 'deg/s'; timestamp: number | null };
  magnetometer: Vector3 & { units: 'uT'; timestamp: number | null };
  screenOrientation: number | null;
};

type SensorStatus = 'checking' | 'available' | 'missing';
type TabKey = 'dashboard' | 'orientation' | 'stream' | 'settings' | 'json';
type StreamFormat = 'json' | 'csv';
type SensorKey = 'accelerometer' | 'gyroscope' | 'magnetometer';
type SamplePoint = { t: number; x: number | null; y: number | null; z: number | null };

const DEFAULT_SERVER = 'http://172.20.10.4:65000';
const DEFAULT_RATE_HZ = 20;
const MAX_RATE_HZ = 60;
const UI_UPDATE_MS = 1000;
const POST_STATUS_UPDATE_MS = 1000;
const HISTORY_LIMIT = 140;
const VECTOR_DECIMALS = 6;
const EULER_DECIMALS = 4;
const QUATERNION_DECIMALS = 8;
const MATRIX_DECIMALS = 6;
const METRIC_DISPLAY_DECIMALS = 4;

const emptyVector: Vector3 = { x: null, y: null, z: null };
const emptyHistory: Record<SensorKey, SamplePoint[]> = {
  accelerometer: [],
  gyroscope: [],
  magnetometer: [],
};

function degToRad(value: number) {
  return (value * Math.PI) / 180;
}

function radToDeg(value: number) {
  return (value * 180) / Math.PI;
}

function round(value: number | null | undefined, decimals = VECTOR_DECIMALS) {
  if (typeof value !== 'number' || Number.isNaN(value)) {
    return null;
  }

  const scale = 10 ** decimals;
  return Math.round(value * scale) / scale;
}

function formatValue(value: number | null | undefined, decimals = 2) {
  if (typeof value !== 'number' || Number.isNaN(value)) {
    return '--';
  }

  return value.toFixed(decimals);
}

function vectorFromMeasurement(
  measurement: AccelerometerMeasurement | GyroscopeMeasurement | MagnetometerMeasurement | null
): Vector3 & { timestamp: number | null } {
  if (!measurement) {
    return { ...emptyVector, timestamp: null };
  }

  return {
    x: round(measurement.x),
    y: round(measurement.y),
    z: round(measurement.z),
    timestamp: measurement.timestamp ?? null,
  };
}

function vectorFromDeviceMotion(
  measurement: { x: number; y: number; z: number; timestamp?: number } | null | undefined
): Vector3 & { timestamp: number | null } {
  if (!measurement) {
    return { ...emptyVector, timestamp: null };
  }

  return {
    x: round(measurement.x),
    y: round(measurement.y),
    z: round(measurement.z),
    timestamp: measurement.timestamp ?? null,
  };
}

function quaternionFromEuler(rollDeg: number, pitchDeg: number, yawDeg: number): Quaternion {
  const roll = degToRad(rollDeg);
  const pitch = degToRad(pitchDeg);
  const yaw = degToRad(yawDeg);

  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);

  return {
    w: round(cr * cp * cy + sr * sp * sy, QUATERNION_DECIMALS) ?? 0,
    x: round(sr * cp * cy - cr * sp * sy, QUATERNION_DECIMALS) ?? 0,
    y: round(cr * sp * cy + sr * cp * sy, QUATERNION_DECIMALS) ?? 0,
    z: round(cr * cp * sy - sr * sp * cy, QUATERNION_DECIMALS) ?? 0,
  };
}

function rotationMatrixFromEuler(rollDeg: number, pitchDeg: number, yawDeg: number) {
  const roll = degToRad(rollDeg);
  const pitch = degToRad(pitchDeg);
  const yaw = degToRad(yawDeg);

  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);

  return [
    [
      round(cy * cp, MATRIX_DECIMALS) ?? 0,
      round(cy * sp * sr - sy * cr, MATRIX_DECIMALS) ?? 0,
      round(cy * sp * cr + sy * sr, MATRIX_DECIMALS) ?? 0,
    ],
    [
      round(sy * cp, MATRIX_DECIMALS) ?? 0,
      round(sy * sp * sr + cy * cr, MATRIX_DECIMALS) ?? 0,
      round(sy * sp * cr - cy * sr, MATRIX_DECIMALS) ?? 0,
    ],
    [
      round(-sp, MATRIX_DECIMALS) ?? 0,
      round(cp * sr, MATRIX_DECIMALS) ?? 0,
      round(cp * cr, MATRIX_DECIMALS) ?? 0,
    ],
  ];
}

function normalizeEndpoint(value: string) {
  const trimmed = value.trim();
  if (!trimmed) {
    return `${DEFAULT_SERVER}/imu`;
  }

  const withProtocol = /^https?:\/\//i.test(trimmed) ? trimmed : `http://${trimmed}`;
  const withoutTrailingSlash = withProtocol.replace(/\/+$/, '');

  if (withoutTrailingSlash.endsWith('/imu') || withoutTrailingSlash.endsWith('/phone')) {
    return withoutTrailingSlash;
  }

  if (withoutTrailingSlash.endsWith('/imu.json')) {
    return withoutTrailingSlash.replace(/\/imu\.json$/, '/imu');
  }

  return `${withoutTrailingSlash}/imu`;
}

function jsonUrlFromEndpoint(endpoint: string) {
  return endpoint.replace(/\/imu$/, '/imu.json').replace(/\/phone$/, '/phone.json');
}

function buildPayload(
  seq: number,
  sampleHz: number,
  motion: DeviceMotionMeasurement | null,
  accelerometer: AccelerometerMeasurement | null,
  gyroscope: GyroscopeMeasurement | null,
  magnetometer: MagnetometerMeasurement | null
): ImuPayload {
  const deviceRoll = round(radToDeg(motion?.rotation?.gamma ?? 0), EULER_DECIMALS) ?? 0;
  const devicePitch = round(radToDeg(motion?.rotation?.beta ?? 0), EULER_DECIMALS) ?? 0;
  const deviceYaw = round(radToDeg(motion?.rotation?.alpha ?? 0), EULER_DECIMALS) ?? 0;
  const roll = -devicePitch;
  const pitch = deviceRoll;
  const yaw = -deviceYaw;
  const gyroscopeVector = gyroscope
    ? vectorFromMeasurement(gyroscope)
    : {
        x: round(motion?.rotationRate?.alpha == null ? null : degToRad(motion.rotationRate.alpha)),
        y: round(motion?.rotationRate?.beta == null ? null : degToRad(motion.rotationRate.beta)),
        z: round(motion?.rotationRate?.gamma == null ? null : degToRad(motion.rotationRate.gamma)),
        timestamp: motion?.rotationRate?.timestamp ?? null,
      };

  return {
    version: 1,
    source: 'expo-go',
    seq,
    generatedAt: new Date().toISOString(),
    sampleHz,
    euler: {
      roll,
      pitch,
      yaw,
      units: 'deg',
      convention: 'roll=-radToDeg(DeviceMotion.beta), pitch=radToDeg(DeviceMotion.gamma), yaw=-radToDeg(DeviceMotion.alpha)',
    },
    quaternion: quaternionFromEuler(roll, pitch, yaw),
    rotationMatrix: rotationMatrixFromEuler(roll, pitch, yaw),
    accelerometer: {
      ...vectorFromMeasurement(accelerometer),
      units: 'g',
    },
    acceleration: {
      ...vectorFromDeviceMotion(motion?.acceleration),
      units: 'm/s^2',
    },
    gyroscope: {
      ...gyroscopeVector,
      units: 'rad/s',
    },
    rotationRate: {
      x: round(motion?.rotationRate?.alpha),
      y: round(motion?.rotationRate?.beta),
      z: round(motion?.rotationRate?.gamma),
      timestamp: motion?.rotationRate?.timestamp ?? null,
      units: 'deg/s',
    },
    magnetometer: {
      ...vectorFromMeasurement(magnetometer),
      units: 'uT',
    },
    screenOrientation: motion?.orientation ?? null,
  };
}

function toCompactPayload(payload: ImuPayload) {
  return {
    t: Date.parse(payload.generatedAt),
    source: payload.source,
    accel: [payload.acceleration.x ?? payload.accelerometer.x ?? 0, payload.acceleration.y ?? payload.accelerometer.y ?? 0, payload.acceleration.z ?? payload.accelerometer.z ?? 0],
    gyro: [payload.gyroscope.x ?? 0, payload.gyroscope.y ?? 0, payload.gyroscope.z ?? 0],
    mag: [payload.magnetometer.x ?? 0, payload.magnetometer.y ?? 0, payload.magnetometer.z ?? 0],
    quat: [payload.quaternion.w, payload.quaternion.x, payload.quaternion.y, payload.quaternion.z],
    euler: {
      yaw: payload.euler.yaw,
      pitch: payload.euler.pitch,
      roll: payload.euler.roll,
    },
  };
}

function toCsv(payload: ImuPayload) {
  const compact = toCompactPayload(payload);
  return [
    compact.t,
    ...compact.accel,
    ...compact.gyro,
    ...compact.mag,
    ...compact.quat,
    compact.euler.yaw,
    compact.euler.pitch,
    compact.euler.roll,
  ].join(',');
}

function motionEuler(motion: DeviceMotionMeasurement | null) {
  const deviceRoll = round(radToDeg(motion?.rotation?.gamma ?? 0), EULER_DECIMALS) ?? 0;
  const devicePitch = round(radToDeg(motion?.rotation?.beta ?? 0), EULER_DECIMALS) ?? 0;
  const deviceYaw = round(radToDeg(motion?.rotation?.alpha ?? 0), EULER_DECIMALS) ?? 0;

  return {
    roll: -devicePitch,
    pitch: deviceRoll,
    yaw: -deviceYaw,
  };
}

function buildCsvPacket(
  motion: DeviceMotionMeasurement | null,
  accelerometer: AccelerometerMeasurement | null,
  gyroscope: GyroscopeMeasurement | null,
  magnetometer: MagnetometerMeasurement | null
) {
  const euler = motionEuler(motion);
  const quat = quaternionFromEuler(euler.roll, euler.pitch, euler.yaw);
  const accel = vectorFromDeviceMotion(motion?.acceleration);
  const fallbackAccel = vectorFromMeasurement(accelerometer);
  const gyro = gyroscope
    ? vectorFromMeasurement(gyroscope)
    : {
        x: round(motion?.rotationRate?.alpha == null ? null : degToRad(motion.rotationRate.alpha)),
        y: round(motion?.rotationRate?.beta == null ? null : degToRad(motion.rotationRate.beta)),
        z: round(motion?.rotationRate?.gamma == null ? null : degToRad(motion.rotationRate.gamma)),
      };
  const mag = vectorFromMeasurement(magnetometer);

  return [
    Date.now(),
    accel.x ?? fallbackAccel.x ?? 0,
    accel.y ?? fallbackAccel.y ?? 0,
    accel.z ?? fallbackAccel.z ?? 0,
    gyro.x ?? 0,
    gyro.y ?? 0,
    gyro.z ?? 0,
    mag.x ?? 0,
    mag.y ?? 0,
    mag.z ?? 0,
    quat.w,
    quat.x,
    quat.y,
    quat.z,
    euler.yaw,
    euler.pitch,
    euler.roll,
  ].join(',');
}

function magnitude(vector: Vector3) {
  const values = [vector.x, vector.y, vector.z].filter((value): value is number => typeof value === 'number');
  if (values.length !== 3) {
    return null;
  }

  return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0));
}

function stats(points: SamplePoint[]) {
  const values = points.flatMap((point) => [point.x, point.y, point.z]).filter((value): value is number => typeof value === 'number');
  if (!values.length) {
    return { min: null, max: null, mean: null };
  }

  return {
    min: Math.min(...values),
    max: Math.max(...values),
    mean: values.reduce((sum, value) => sum + value, 0) / values.length,
  };
}

function appendHistory(history: Record<SensorKey, SamplePoint[]>, payload: ImuPayload) {
  const now = Date.now();
  const nextPoint = (vector: Vector3) => ({ t: now, x: vector.x, y: vector.y, z: vector.z });

  return {
    accelerometer: [...history.accelerometer, nextPoint(payload.acceleration)].slice(-HISTORY_LIMIT),
    gyroscope: [...history.gyroscope, nextPoint(payload.gyroscope)].slice(-HISTORY_LIMIT),
    magnetometer: [...history.magnetometer, nextPoint(payload.magnetometer)].slice(-HISTORY_LIMIT),
  };
}

export default function App() {
  const [tab, setTab] = useState<TabKey>('stream');
  const [server, setServer] = useState(DEFAULT_SERVER);
  const [rateHzText, setRateHzText] = useState(String(DEFAULT_RATE_HZ));
  const [format, setFormat] = useState<StreamFormat>('csv');
  const [running, setRunning] = useState(false);
  const [status, setStatus] = useState('Idle');
  const [lastPost, setLastPost] = useState('Not connected');
  const [packetsSent, setPacketsSent] = useState(0);
  const [postErrors, setPostErrors] = useState(0);
  const [sensorStatus, setSensorStatus] = useState<Record<string, SensorStatus>>({
    deviceMotion: 'checking',
    accelerometer: 'checking',
    gyroscope: 'checking',
    magnetometer: 'checking',
  });
  const [payload, setPayload] = useState<ImuPayload>(() =>
    buildPayload(0, DEFAULT_RATE_HZ, null, null, null, null)
  );
  const [history, setHistory] = useState<Record<SensorKey, SamplePoint[]>>(emptyHistory);

  const motionRef = useRef<DeviceMotionMeasurement | null>(null);
  const accelerometerRef = useRef<AccelerometerMeasurement | null>(null);
  const gyroscopeRef = useRef<GyroscopeMeasurement | null>(null);
  const magnetometerRef = useRef<MagnetometerMeasurement | null>(null);
  const subscriptionsRef = useRef<Array<{ remove: () => void }>>([]);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const seqRef = useRef(0);
  const postingRef = useRef(false);
  const serverRef = useRef(server);
  const formatRef = useRef(format);
  const tabRef = useRef(tab);
  const rateRef = useRef(DEFAULT_RATE_HZ);
  const packetsSentRef = useRef(0);
  const postErrorsRef = useRef(0);
  const lastUiSensorUpdateRef = useRef(0);
  const lastUiPostUpdateRef = useRef(0);

  useEffect(() => {
    serverRef.current = server;
  }, [server]);

  useEffect(() => {
    formatRef.current = format;
  }, [format]);

  useEffect(() => {
    tabRef.current = tab;
  }, [tab]);

  useEffect(() => {
    let mounted = true;

    async function checkAvailability() {
      const entries = await Promise.all([
        DeviceMotion.isAvailableAsync().then((ok) => ['deviceMotion', ok] as const),
        Accelerometer.isAvailableAsync().then((ok) => ['accelerometer', ok] as const),
        Gyroscope.isAvailableAsync().then((ok) => ['gyroscope', ok] as const),
        Magnetometer.isAvailableAsync().then((ok) => ['magnetometer', ok] as const),
      ]);

      if (!mounted) {
        return;
      }

      setSensorStatus(
        entries.reduce<Record<string, SensorStatus>>((next, [key, ok]) => {
          next[key] = ok ? 'available' : 'missing';
          return next;
        }, {})
      );
    }

    checkAvailability().catch((error) => {
      setStatus(`Sensor check failed: ${String(error)}`);
    });

    return () => {
      mounted = false;
      clearSubscriptions();
    };
  }, []);

  function parsedRateHz() {
    const parsed = Number(rateHzText);
    if (!Number.isFinite(parsed)) {
      return DEFAULT_RATE_HZ;
    }

    return Math.min(MAX_RATE_HZ, Math.max(1, parsed));
  }

  function snapshot(nextSeq = seqRef.current, sampleHz = rateRef.current) {
    return buildPayload(
      nextSeq,
      sampleHz,
      motionRef.current,
      accelerometerRef.current,
      gyroscopeRef.current,
      magnetometerRef.current
    );
  }

  async function requestPermission(
    name: string,
    sensor: { requestPermissionsAsync?: () => Promise<{ status: string }> },
    available: boolean
  ) {
    if (!available) {
      return;
    }

    if (!sensor.requestPermissionsAsync) {
      return;
    }

    const permission = await sensor.requestPermissionsAsync();
    if (permission.status !== 'granted') {
      throw new Error(`${name} permission was ${permission.status}`);
    }
  }

  function clearSubscriptions() {
    subscriptionsRef.current.forEach((subscription) => subscription.remove());
    subscriptionsRef.current = [];

    if (timerRef.current) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }
  }

  function stop() {
    clearSubscriptions();
    setRunning(false);
    setStatus('Stopped');
  }

  async function postLatest() {
    if (postingRef.current) {
      return;
    }

    const endpoint = normalizeEndpoint(serverRef.current);
    const selectedFormat = formatRef.current;
    const activeTab = tabRef.current;
    const nextSeq = seqRef.current + 1;
    const now = Date.now();
    const shouldUpdatePayload =
      activeTab === 'dashboard' || activeTab === 'orientation' || activeTab === 'settings' || activeTab === 'json';
    const shouldUpdateUi = shouldUpdatePayload && now - lastUiSensorUpdateRef.current >= UI_UPDATE_MS;
    const next =
      selectedFormat === 'json' || shouldUpdateUi
        ? snapshot(nextSeq)
        : null;
    const body =
      selectedFormat === 'json'
        ? JSON.stringify(next)
        : next
          ? toCsv(next)
          : buildCsvPacket(motionRef.current, accelerometerRef.current, gyroscopeRef.current, magnetometerRef.current);

    seqRef.current = nextSeq;

    if (next && shouldUpdateUi) {
      lastUiSensorUpdateRef.current = now;
      setPayload(next);
      if (activeTab === 'dashboard') {
        setHistory((current) => appendHistory(current, next));
      }
    }

    postingRef.current = true;
    try {
      const response = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': selectedFormat === 'json' ? 'application/json' : 'text/csv' },
        body,
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }

      packetsSentRef.current += 1;
      if (now - lastUiPostUpdateRef.current >= POST_STATUS_UPDATE_MS) {
        lastUiPostUpdateRef.current = now;
        setPacketsSent(packetsSentRef.current);
        setPostErrors(postErrorsRef.current);
        setLastPost(`POST ok ${new Date().toLocaleTimeString()}`);
      }
    } catch (error) {
      postErrorsRef.current += 1;
      if (now - lastUiPostUpdateRef.current >= POST_STATUS_UPDATE_MS) {
        lastUiPostUpdateRef.current = now;
        setPacketsSent(packetsSentRef.current);
        setPostErrors(postErrorsRef.current);
        setLastPost(`POST failed: ${error instanceof Error ? error.message : String(error)}`);
      }
    } finally {
      postingRef.current = false;
    }
  }

  async function start() {
    if (running) {
      return;
    }

    setStatus('Requesting sensor permission...');
    setLastPost('Waiting for first sample');

    try {
      const availability = {
        deviceMotion: await DeviceMotion.isAvailableAsync(),
        accelerometer: await Accelerometer.isAvailableAsync(),
        gyroscope: await Gyroscope.isAvailableAsync(),
        magnetometer: await Magnetometer.isAvailableAsync(),
      };

      setSensorStatus({
        deviceMotion: availability.deviceMotion ? 'available' : 'missing',
        accelerometer: availability.accelerometer ? 'available' : 'missing',
        gyroscope: availability.gyroscope ? 'available' : 'missing',
        magnetometer: availability.magnetometer ? 'available' : 'missing',
      });

      if (!Object.values(availability).some(Boolean)) {
        throw new Error('No Android motion sensors are available on this device.');
      }

      await Promise.all([
        requestPermission('Device motion', DeviceMotion, availability.deviceMotion),
        requestPermission('Accelerometer', Accelerometer, availability.accelerometer),
        requestPermission('Gyroscope', Gyroscope, availability.gyroscope),
        requestPermission('Magnetometer', Magnetometer, availability.magnetometer),
      ]);

      const sampleHz = parsedRateHz();
      const intervalMs = Math.round(1000 / sampleHz);
      const magnetometerIntervalMs = Math.max(250, intervalMs);
      rateRef.current = sampleHz;
      if (availability.deviceMotion) {
        DeviceMotion.setUpdateInterval(intervalMs);
      }
      if (availability.accelerometer) {
        Accelerometer.setUpdateInterval(intervalMs);
      }
      if (availability.gyroscope) {
        Gyroscope.setUpdateInterval(intervalMs);
      }
      if (availability.magnetometer) {
        Magnetometer.setUpdateInterval(magnetometerIntervalMs);
      }

      clearSubscriptions();
      const subscriptions: Array<{ remove: () => void }> = [];

      if (availability.deviceMotion) {
        subscriptions.push(DeviceMotion.addListener((measurement) => {
          motionRef.current = measurement;
        }));
      }

      if (!availability.deviceMotion && availability.accelerometer) {
        subscriptions.push(Accelerometer.addListener((measurement) => {
          accelerometerRef.current = measurement;
        }));
      }
      if (!availability.deviceMotion && availability.gyroscope) {
        subscriptions.push(Gyroscope.addListener((measurement) => {
          gyroscopeRef.current = measurement;
        }));
      }
      if (availability.magnetometer) {
        subscriptions.push(Magnetometer.addListener((measurement) => {
          magnetometerRef.current = measurement;
        }));
      }

      subscriptionsRef.current = subscriptions;

      timerRef.current = setInterval(postLatest, intervalMs);
      setRunning(true);
      const missing = Object.entries(availability)
        .filter(([, ok]) => !ok)
        .map(([name]) => name)
        .join(', ');
      setStatus(missing ? `Streaming at ${sampleHz} Hz. Missing: ${missing}` : `Streaming at ${sampleHz} Hz`);
    } catch (error) {
      clearSubscriptions();
      setRunning(false);
      setStatus(error instanceof Error ? error.message : String(error));
    }
  }

  const endpoint = normalizeEndpoint(server);
  const hostedJson = jsonUrlFromEndpoint(endpoint);
  const exportCsv = useMemo(() => {
    const header = 't,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,yaw,pitch,roll';
    return [header, toCsv(payload)].join('\n');
  }, [payload]);

  return (
    <SafeAreaView style={styles.safeArea}>
      <StatusBar style="light" />
      <ScrollView contentContainerStyle={styles.container}>
        <Header payload={payload} status={status} />

        {tab === 'dashboard' && <DashboardScreen payload={payload} history={history} />}
        {tab === 'orientation' && <OrientationScreen payload={payload} />}
        {tab === 'stream' && (
          <StreamScreen
            endpoint={endpoint}
            format={format}
            hostedJson={hostedJson}
            lastPost={lastPost}
            packetsSent={packetsSent}
            postErrors={postErrors}
            rateHzText={rateHzText}
            running={running}
            server={server}
            setFormat={setFormat}
            setRateHzText={setRateHzText}
            setServer={setServer}
            start={start}
            status={status}
            stop={stop}
          />
        )}
        {tab === 'settings' && (
          <SettingsScreen
            exportCsv={exportCsv}
            payload={payload}
            rateHz={rateRef.current}
            sensorStatus={sensorStatus}
          />
        )}
        {tab === 'json' && <JsonScreen payload={payload} />}

        <TabBar active={tab} onChange={setTab} />
      </ScrollView>
    </SafeAreaView>
  );
}

function Header({ payload, status }: { payload: ImuPayload; status: string }) {
  return (
    <View style={styles.header}>
      <View>
        <Text style={styles.title}>IMU Sensor Suite</Text>
        <Text style={styles.caption}>{status}</Text>
      </View>
      <View style={styles.ratePill}>
        <Text style={styles.rateText}>{payload.sampleHz} Hz</Text>
      </View>
    </View>
  );
}

function DashboardScreen({ payload, history }: { payload: ImuPayload; history: Record<SensorKey, SamplePoint[]> }) {
  return (
    <View style={styles.screen}>
      <View style={styles.yprBanner}>
        <YprChip label="Yaw" value={payload.euler.yaw} color="#56c7ff" />
        <YprChip label="Pitch" value={payload.euler.pitch} color="#d0e357" />
        <YprChip label="Roll" value={payload.euler.roll} color="#ff7f6f" />
      </View>

      <SensorCard
        accent="#56c7ff"
        title="Accelerometer"
        unit="m/s2"
        vector={payload.acceleration}
      />
      <SensorCard
        accent="#d0e357"
        title="Gyroscope"
        unit="rad/s"
        vector={payload.gyroscope}
      />
      <SensorCard
        accent="#ff7f6f"
        title="Magnetometer"
        unit="uT"
        vector={payload.magnetometer}
      />
      <View style={styles.panel}>
        <Text style={styles.sectionTitle}>Orientation</Text>
        <CubePreview payload={payload} />
      </View>
    </View>
  );
}

function OrientationScreen({ payload }: { payload: ImuPayload }) {
  return (
    <View style={styles.screen}>
      <View style={styles.panel}>
        <Text style={styles.sectionTitle}>3D Orientation</Text>
        <CubePreview payload={payload} large />
      </View>

      <View style={styles.gaugeGrid}>
        <AngleGauge label="Yaw" value={payload.euler.yaw} limit={180} color="#56c7ff" />
        <AngleGauge label="Pitch" value={payload.euler.pitch} limit={90} color="#d0e357" />
        <AngleGauge label="Roll" value={payload.euler.roll} limit={180} color="#ff7f6f" />
      </View>

      <View style={styles.panel}>
        <Text style={styles.sectionTitle}>Quaternion</Text>
        <View style={styles.rowGrid}>
          <MetricValue label="w" value={payload.quaternion.w} decimals={6} />
          <MetricValue label="x" value={payload.quaternion.x} decimals={6} />
          <MetricValue label="y" value={payload.quaternion.y} decimals={6} />
          <MetricValue label="z" value={payload.quaternion.z} decimals={6} />
        </View>
      </View>

      <View style={styles.panel}>
        <Text style={styles.sectionTitle}>Rotation Matrix</Text>
        {payload.rotationMatrix.map((row, index) => (
          <Text key={index} style={styles.matrixText}>
            {row.map((value) => formatValue(value, 6).padStart(11, ' ')).join(' ')}
          </Text>
        ))}
      </View>
    </View>
  );
}

function StreamScreen({
  endpoint,
  format,
  hostedJson,
  lastPost,
  packetsSent,
  postErrors,
  rateHzText,
  running,
  server,
  setFormat,
  setRateHzText,
  setServer,
  start,
  status,
  stop,
}: {
  endpoint: string;
  format: StreamFormat;
  hostedJson: string;
  lastPost: string;
  packetsSent: number;
  postErrors: number;
  rateHzText: string;
  running: boolean;
  server: string;
  setFormat: (format: StreamFormat) => void;
  setRateHzText: (value: string) => void;
  setServer: (value: string) => void;
  start: () => void;
  status: string;
  stop: () => void;
}) {
  return (
    <View style={styles.screen}>
      <View style={styles.controls}>
        <Text style={styles.label}>Host/IP address</Text>
        <TextInput
          autoCapitalize="none"
          autoCorrect={false}
          keyboardType="url"
          onChangeText={setServer}
          placeholder="http://192.168.4.117:65000"
          placeholderTextColor="#6f7378"
          style={styles.input}
          value={server}
        />

        <Text style={styles.label}>Data forwarding rate (Hz)</Text>
        <TextInput
          keyboardType="numeric"
          onChangeText={setRateHzText}
          placeholder="60"
          placeholderTextColor="#6f7378"
          style={styles.input}
          value={rateHzText}
        />

        <Text style={styles.label}>Packet format</Text>
        <View style={styles.buttonRow}>
          <FormatButton active={format === 'json'} label="JSON" onPress={() => setFormat('json')} />
          <FormatButton active={format === 'csv'} label="CSV" onPress={() => setFormat('csv')} />
        </View>

        <View style={styles.buttonRow}>
          <Pressable
            disabled={running}
            onPress={start}
            style={({ pressed }) => [styles.button, running && styles.buttonDisabled, pressed && styles.pressed]}
          >
            <Text style={styles.buttonText}>Start</Text>
          </Pressable>
          <Pressable
            onPress={stop}
            style={({ pressed }) => [styles.button, styles.stopButton, pressed && styles.pressed]}
          >
            <Text style={styles.buttonText}>Stop</Text>
          </Pressable>
        </View>
      </View>

      <View style={styles.statusPanel}>
        <Text style={styles.statusText}>{status}</Text>
        <Text style={styles.mutedText}>{lastPost}</Text>
        <Text style={styles.urlText}>POST {endpoint}</Text>
        <Text style={styles.urlText}>GET {hostedJson}</Text>
      </View>

      <View style={styles.rowGrid}>
        <MetricValue label="Packets" value={packetsSent} decimals={0} />
        <MetricValue label="Errors" value={postErrors} decimals={0} />
      </View>
    </View>
  );
}

function SettingsScreen({
  exportCsv,
  payload,
  rateHz,
  sensorStatus,
}: {
  exportCsv: string;
  payload: ImuPayload;
  rateHz: number;
  sensorStatus: Record<string, SensorStatus>;
}) {
  return (
    <View style={styles.screen}>
      <View style={styles.panel}>
        <Text style={styles.sectionTitle}>Sensors</Text>
        <Text style={styles.mutedText}>Current stream rate: {rateHz} Hz</Text>
        <View style={styles.sensorStrip}>
          {Object.entries(sensorStatus).map(([name, sensor]) => (
            <View key={name} style={styles.sensorPill}>
              <Text style={styles.sensorName}>{name}</Text>
              <Text style={[styles.sensorState, sensor === 'missing' && styles.sensorMissing]}>{sensor}</Text>
            </View>
          ))}
        </View>
      </View>

      <View style={styles.panel}>
        <Text style={styles.sectionTitle}>Export Snapshot</Text>
        <Text style={styles.mutedText}>CSV row</Text>
        <Text selectable style={styles.codeText}>{exportCsv}</Text>
        <Text style={styles.mutedText}>JSON object</Text>
        <Text selectable style={styles.codeText}>{JSON.stringify(toCompactPayload(payload), null, 2)}</Text>
      </View>

      <View style={styles.panel}>
        <Text style={styles.sectionTitle}>PC Dashboard</Text>
        <Text style={styles.mutedText}>
          The Node receiver accepts phone HTTP posts, phone UDP CSV or JSON packets, STM32/BNO055 bridge posts, drift
          reset, and browser JSON endpoints.
        </Text>
      </View>
    </View>
  );
}

function JsonScreen({ payload }: { payload: ImuPayload }) {
  return (
    <View style={styles.screen}>
      <View style={styles.jsonPanel}>
        <Text style={styles.sectionTitle}>JSON Preview</Text>
        <Text selectable style={styles.codeText}>{JSON.stringify(payload, null, 2)}</Text>
      </View>
    </View>
  );
}

function SensorCard({
  accent,
  title,
  unit,
  vector,
}: {
  accent: string;
  title: string;
  unit: string;
  vector: Vector3;
}) {
  const vectorMagnitude = magnitude(vector);

  return (
    <View style={[styles.panel, { borderColor: accent }]}>
      <Text style={[styles.metricTitle, { color: accent }]}>{title}</Text>
      <View style={styles.rowGrid}>
        <MetricValue label="X" value={vector.x} />
        <MetricValue label="Y" value={vector.y} />
        <MetricValue label="Z" value={vector.z} />
        <MetricValue label="Mag" value={vectorMagnitude} />
      </View>
      <View style={styles.statsRow}>
        <Text style={styles.mutedText}>Live {unit}</Text>
        <Text style={styles.mutedText}>simplified view</Text>
      </View>
    </View>
  );
}

function CubePreview({ large, payload }: { large?: boolean; payload: ImuPayload }) {
  const yaw = Math.max(-35, Math.min(35, payload.euler.yaw));
  const pitch = Math.max(-35, Math.min(35, payload.euler.pitch));
  const roll = Math.max(-45, Math.min(45, payload.euler.roll));

  return (
    <View style={[styles.cubeStage, large && styles.largeCubeStage]}>
      <View
        style={[
          styles.cubeFace,
          large && styles.largeCubeFace,
          {
            transform: [
              { rotateZ: `${roll}deg` },
              { rotateY: `${yaw}deg` },
              { rotateX: `${pitch}deg` },
            ],
          },
        ]}
      >
        <View style={styles.cubeLineHorizontal} />
        <View style={styles.cubeLineVertical} />
      </View>
      <View style={styles.axisLegend}>
        <Text style={[styles.axisText, { color: '#ff7f6f' }]}>X</Text>
        <Text style={[styles.axisText, { color: '#d0e357' }]}>Y</Text>
        <Text style={[styles.axisText, { color: '#56c7ff' }]}>Z</Text>
      </View>
    </View>
  );
}

function AngleGauge({ color, label, limit, value }: { color: string; label: string; limit: number; value: number }) {
  const pct = Math.min(100, Math.max(0, ((value + limit) / (limit * 2)) * 100));

  return (
    <View style={styles.gauge}>
      <Text style={styles.metricTitle}>{label}</Text>
      <Text style={styles.gaugeValue}>{formatValue(value, 2)} deg</Text>
      <View style={styles.gaugeTrack}>
        <View style={[styles.gaugeFill, { width: `${pct}%`, backgroundColor: color }]} />
      </View>
    </View>
  );
}

function YprChip({ color, label, value }: { color: string; label: string; value: number }) {
  return (
    <View style={styles.yprChip}>
      <Text style={[styles.sensorName, { color }]}>{label}</Text>
      <Text style={styles.yprValue}>{formatValue(value, 1)} deg</Text>
    </View>
  );
}

function MetricValue({ decimals = METRIC_DISPLAY_DECIMALS, label, value }: { decimals?: number; label: string; value: number | null }) {
  return (
    <View style={styles.metricValueBox}>
      <Text style={styles.metricLabel}>{label}</Text>
      <Text style={styles.metricValue}>{formatValue(value, decimals)}</Text>
    </View>
  );
}

function FormatButton({ active, label, onPress }: { active: boolean; label: string; onPress: () => void }) {
  return (
    <Pressable onPress={onPress} style={[styles.formatButton, active && styles.activeFormatButton]}>
      <Text style={[styles.formatButtonText, active && styles.activeFormatButtonText]}>{label}</Text>
    </Pressable>
  );
}

function TabBar({ active, onChange }: { active: TabKey; onChange: (tab: TabKey) => void }) {
  const tabs: Array<[TabKey, string]> = [
    ['dashboard', 'Home'],
    ['orientation', '3D'],
    ['stream', 'Stream'],
    ['settings', 'Settings'],
    ['json', 'JSON'],
  ];

  return (
    <View style={styles.tabBar}>
      {tabs.map(([key, label]) => (
        <Pressable key={key} onPress={() => onChange(key)} style={[styles.tabButton, active === key && styles.activeTabButton]}>
          <Text style={active === key ? styles.activeTab : styles.tab}>{label}</Text>
        </Pressable>
      ))}
    </View>
  );
}

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#050607',
  },
  container: {
    paddingHorizontal: 16,
    paddingTop: 18,
    paddingBottom: 28,
    gap: 16,
  },
  header: {
    alignItems: 'center',
    flexDirection: 'row',
    gap: 12,
    justifyContent: 'space-between',
  },
  title: {
    color: '#f3f5f7',
    fontSize: 28,
    fontWeight: '700',
  },
  caption: {
    color: '#a8afb5',
    fontSize: 14,
    lineHeight: 20,
  },
  ratePill: {
    backgroundColor: '#16232a',
    borderColor: '#56c7ff',
    borderRadius: 8,
    borderWidth: 1,
    paddingHorizontal: 10,
    paddingVertical: 7,
  },
  rateText: {
    color: '#56c7ff',
    fontSize: 13,
    fontWeight: '700',
  },
  screen: {
    gap: 14,
  },
  controls: {
    gap: 10,
  },
  label: {
    color: '#d5d9dd',
    fontSize: 16,
    fontWeight: '600',
  },
  input: {
    minHeight: 46,
    borderColor: '#30343a',
    borderRadius: 8,
    borderWidth: 1,
    color: '#f3f5f7',
    fontSize: 16,
    paddingHorizontal: 12,
    backgroundColor: '#101214',
  },
  buttonRow: {
    flexDirection: 'row',
    gap: 12,
    marginTop: 4,
  },
  button: {
    flex: 1,
    minHeight: 46,
    alignItems: 'center',
    justifyContent: 'center',
    borderRadius: 8,
    backgroundColor: '#0b8ee8',
  },
  stopButton: {
    backgroundColor: '#565b60',
  },
  buttonDisabled: {
    backgroundColor: '#335d78',
  },
  pressed: {
    opacity: 0.75,
  },
  buttonText: {
    color: '#ffffff',
    fontSize: 16,
    fontWeight: '700',
  },
  formatButton: {
    flex: 1,
    borderColor: '#30343a',
    borderRadius: 8,
    borderWidth: 1,
    minHeight: 42,
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#101214',
  },
  activeFormatButton: {
    borderColor: '#56c7ff',
    backgroundColor: '#122631',
  },
  formatButtonText: {
    color: '#a8afb5',
    fontWeight: '700',
  },
  activeFormatButtonText: {
    color: '#56c7ff',
  },
  statusPanel: {
    borderColor: '#30343a',
    borderRadius: 8,
    borderWidth: 1,
    padding: 12,
    gap: 6,
    backgroundColor: '#101214',
  },
  statusText: {
    color: '#f3f5f7',
    fontSize: 16,
    fontWeight: '700',
  },
  mutedText: {
    color: '#a8afb5',
    fontSize: 13,
    lineHeight: 19,
  },
  urlText: {
    color: '#8ecdf8',
    fontSize: 13,
  },
  panel: {
    borderColor: '#30343a',
    borderRadius: 8,
    borderWidth: 1,
    gap: 10,
    padding: 12,
    backgroundColor: '#101214',
  },
  yprBanner: {
    borderColor: '#30343a',
    borderRadius: 8,
    borderWidth: 1,
    flexDirection: 'row',
    justifyContent: 'space-between',
    padding: 12,
    backgroundColor: '#101214',
  },
  yprChip: {
    alignItems: 'center',
    flex: 1,
    gap: 4,
  },
  yprValue: {
    color: '#f3f5f7',
    fontSize: 17,
    fontWeight: '700',
    fontVariant: ['tabular-nums'],
  },
  sectionTitle: {
    color: '#f3f5f7',
    fontSize: 19,
    fontWeight: '700',
  },
  metricTitle: {
    color: '#f3f5f7',
    fontSize: 16,
    fontWeight: '700',
  },
  metricLabel: {
    color: '#a8afb5',
    fontSize: 12,
  },
  metricValue: {
    color: '#f3f5f7',
    fontSize: 15,
    fontVariant: ['tabular-nums'],
    fontWeight: '700',
  },
  metricValueBox: {
    minWidth: 66,
    gap: 3,
  },
  rowGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 12,
    justifyContent: 'space-between',
  },
  statsRow: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 10,
    justifyContent: 'space-between',
  },
  chart: {
    alignItems: 'flex-end',
    borderBottomColor: '#30343a',
    borderBottomWidth: 1,
    flexDirection: 'row',
    gap: 2,
    height: 78,
    overflow: 'hidden',
    paddingTop: 8,
  },
  chartColumn: {
    alignItems: 'flex-end',
    flex: 1,
    flexDirection: 'row',
    gap: 1,
    height: '100%',
  },
  chartBar: {
    borderTopLeftRadius: 2,
    borderTopRightRadius: 2,
    flex: 1,
    minHeight: 3,
  },
  axisX: {
    backgroundColor: '#ff7f6f',
  },
  axisY: {
    backgroundColor: '#d0e357',
  },
  axisZ: {
    backgroundColor: '#56c7ff',
  },
  cubeStage: {
    alignItems: 'center',
    height: 160,
    justifyContent: 'center',
  },
  largeCubeStage: {
    height: 260,
  },
  cubeFace: {
    alignItems: 'center',
    aspectRatio: 1,
    borderColor: '#56c7ff',
    borderRadius: 8,
    borderWidth: 3,
    justifyContent: 'center',
    width: 118,
    backgroundColor: '#14252d',
  },
  largeCubeFace: {
    width: 170,
  },
  cubeLineHorizontal: {
    backgroundColor: '#d0e357',
    height: 3,
    width: '100%',
  },
  cubeLineVertical: {
    backgroundColor: '#ff7f6f',
    height: '100%',
    position: 'absolute',
    width: 3,
  },
  axisLegend: {
    bottom: 8,
    flexDirection: 'row',
    gap: 14,
    position: 'absolute',
  },
  axisText: {
    fontWeight: '700',
  },
  gaugeGrid: {
    gap: 10,
  },
  gauge: {
    borderColor: '#30343a',
    borderRadius: 8,
    borderWidth: 1,
    gap: 8,
    padding: 12,
    backgroundColor: '#101214',
  },
  gaugeValue: {
    color: '#f3f5f7',
    fontSize: 22,
    fontVariant: ['tabular-nums'],
    fontWeight: '700',
  },
  gaugeTrack: {
    backgroundColor: '#262a2e',
    borderRadius: 8,
    height: 12,
    overflow: 'hidden',
  },
  gaugeFill: {
    height: '100%',
  },
  matrixText: {
    color: '#f3f5f7',
    fontFamily: 'monospace',
    fontSize: 14,
    textAlign: 'center',
  },
  jsonPanel: {
    borderColor: '#30343a',
    borderRadius: 8,
    borderWidth: 1,
    padding: 12,
    backgroundColor: '#101214',
  },
  codeText: {
    color: '#d5d9dd',
    fontFamily: 'monospace',
    fontSize: 12,
  },
  sensorStrip: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 8,
  },
  sensorPill: {
    borderRadius: 8,
    borderColor: '#30343a',
    borderWidth: 1,
    paddingHorizontal: 10,
    paddingVertical: 8,
    backgroundColor: '#15181a',
  },
  sensorName: {
    color: '#a8afb5',
    fontSize: 12,
  },
  sensorState: {
    color: '#9ad66f',
    fontSize: 13,
    fontWeight: '700',
  },
  sensorMissing: {
    color: '#f07464',
  },
  tabBar: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    borderTopColor: '#25282d',
    borderTopWidth: 1,
    paddingTop: 14,
  },
  tabButton: {
    alignItems: 'center',
    borderRadius: 8,
    flex: 1,
    paddingVertical: 10,
  },
  activeTabButton: {
    backgroundColor: '#122631',
  },
  tab: {
    color: '#8d9399',
    fontSize: 12,
  },
  activeTab: {
    color: '#56c7ff',
    fontSize: 12,
    fontWeight: '700',
  },
});
