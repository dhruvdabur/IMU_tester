const dgram = require('node:dgram');
const http = require('node:http');
const os = require('node:os');

const HTTP_PORT = Number(process.env.HTTP_PORT || process.env.PORT || 65000);
const UDP_PORT = Number(process.env.UDP_PORT || 65000);
const HOST = process.env.HOST || '0.0.0.0';
const HISTORY_LIMIT = Number(process.env.HISTORY_LIMIT || 500);
const SYNC_RATE_HZ = Math.max(1, Math.min(200, Number(process.env.SYNC_RATE_HZ || 50)));
const MAX_BODY_BYTES = 1024 * 1024;
const PACKET_LOG_INTERVAL_MS = Number(process.env.PACKET_LOG_INTERVAL_MS || 1000);

function waitingEnvelope(source) {
  return {
    status: 'waiting',
    source,
    message: `No ${source} packets received yet.`,
    receivedAt: null,
    transport: null,
    remote: null,
    data: null,
  };
}

let latest = waitingEnvelope('any');
const latestBySource = {
  phone: waitingEnvelope('phone'),
  bno055: waitingEnvelope('bno055'),
};
const history = [];
const driftHistory = [];
const imuHistory = {
  phone: [],
  bno055: [],
};
const analysisBuffers = {
  phone: [],
  bno055: [],
};
const headingTrace = [];
let driftBaseline = null;
let lastPacketLogAt = 0;
let recording = false;
let recordStartMs = 0;
let lastSyncedFrameAt = 0;

function localIpv4Addresses() {
  return Object.values(os.networkInterfaces())
    .flat()
    .filter((entry) => entry && entry.family === 'IPv4' && !entry.internal)
    .map((entry) => entry.address);
}

function round(value, decimals = 4) {
  if (typeof value !== 'number' || Number.isNaN(value)) {
    return null;
  }

  const scale = 10 ** decimals;
  return Math.round(value * scale) / scale;
}

function radiansToDegrees(value) {
  return (value * 180) / Math.PI;
}

function toNumber(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function vectorArray(vector, fallback = [0, 0, 0]) {
  if (Array.isArray(vector)) {
    return fallback.map((_, index) => toNumber(vector[index], fallback[index]));
  }

  if (vector && typeof vector === 'object') {
    return [toNumber(vector.x), toNumber(vector.y), toNumber(vector.z)];
  }

  return fallback.slice();
}

function compactFromCsv(text) {
  const parts = text.trim().split(',').map((part) => part.trim());
  if (parts.length < 17) {
    throw new Error('CSV packets must contain 17 fields: timestamp, accel xyz, gyro xyz, mag xyz, quat wxyz, yaw pitch roll');
  }

  const values = parts.map(Number);
  if (values.some((value) => !Number.isFinite(value))) {
    throw new Error('CSV packet contains a non-numeric value');
  }

  return {
    t: values[0],
    source: 'expo-go',
    accel: values.slice(1, 4),
    gyro: values.slice(4, 7),
    mag: values.slice(7, 10),
    quat: values.slice(10, 14),
    euler: {
      yaw: values[14],
      pitch: values[15],
      roll: values[16],
      units: 'deg',
      convention: 'csv:yaw,pitch,roll',
    },
  };
}

function parseIncomingText(text) {
  const trimmed = text.trim();
  if (!trimmed) {
    return {};
  }

  if (trimmed.startsWith('{') || trimmed.startsWith('[')) {
    return JSON.parse(trimmed);
  }

  return compactFromCsv(trimmed);
}

function sourceKeyFor(data, forcedSource) {
  if (forcedSource) {
    return forcedSource;
  }

  const source = String(data?.source || '').toLowerCase();
  if (source.includes('bno') || source.includes('stm32')) {
    return 'bno055';
  }

  return 'phone';
}

function normalizeIncomingData(data, forcedSource) {
  const source = sourceKeyFor(data, forcedSource);
  const compact = data && (Array.isArray(data.accel) || Array.isArray(data.gyro) || Array.isArray(data.mag) || Array.isArray(data.quat));
  if (compact) {
    return {
      ...data,
      source: data.source || (source === 'phone' ? 'expo-go' : source),
      accelerometer: {
        x: vectorArray(data.accel)[0],
        y: vectorArray(data.accel)[1],
        z: vectorArray(data.accel)[2],
        units: 'm/s^2',
      },
      acceleration: {
        x: vectorArray(data.accel)[0],
        y: vectorArray(data.accel)[1],
        z: vectorArray(data.accel)[2],
        units: 'm/s^2',
      },
      gyroscope: {
        x: vectorArray(data.gyro)[0],
        y: vectorArray(data.gyro)[1],
        z: vectorArray(data.gyro)[2],
        units: 'rad/s',
      },
      magnetometer: {
        x: vectorArray(data.mag)[0],
        y: vectorArray(data.mag)[1],
        z: vectorArray(data.mag)[2],
        units: 'uT',
      },
      quaternion: {
        w: vectorArray(data.quat, [1, 0, 0, 0])[0],
        x: vectorArray(data.quat, [1, 0, 0, 0])[1],
        y: vectorArray(data.quat, [1, 0, 0, 0])[2],
        z: vectorArray(data.quat, [1, 0, 0, 0])[3],
      },
    };
  }

  if (source !== 'phone' || !data?.euler) {
    return data;
  }

  const convention = String(data.euler.convention || '');
  const alreadyConverted = convention.includes('radToDeg');
  if (alreadyConverted) {
    return data;
  }

  const roll = Number(data.euler.roll);
  const pitch = Number(data.euler.pitch);
  const yaw = Number(data.euler.yaw);
  if (![roll, pitch, yaw].every(Number.isFinite)) {
    return data;
  }

  return {
    ...data,
    euler: {
      ...data.euler,
      roll: round(radiansToDegrees(roll)),
      pitch: round(radiansToDegrees(pitch)),
      yaw: round(radiansToDegrees(yaw)),
      units: 'deg',
      convention: 'serverConvertedFromRadians(DeviceMotion.gamma/beta/alpha)',
    },
  };
}

function extractEuler(envelope) {
  const data = envelope?.data;
  if (!data) {
    return null;
  }

  const euler = data.euler || data.motion || data;
  const roll = Number(euler.roll);
  const pitch = Number(euler.pitch);
  const yaw = Number(euler.yaw);

  if (![roll, pitch, yaw].every(Number.isFinite)) {
    return null;
  }

  return {
    roll,
    pitch,
    yaw,
  };
}

function extractVector(data, keys) {
  for (const key of keys) {
    const value = data?.[key];
    if (value) {
      return vectorArray(value);
    }
  }

  return [0, 0, 0];
}

function missingEulerReason(label, envelope) {
  if (!envelope?.receivedAt) {
    return `No ${label} packets received yet.`;
  }

  const data = envelope.data;
  if (data?.status === 'error') {
    const detail = data.message || 'reported an error';
    const code = data.errorCode === undefined ? '' : ` code=${data.errorCode}`;
    return `${label} packets are arriving, but ${label} has no Euler sample: ${detail}${code}.`;
  }

  return `${label} packets are arriving, but roll/pitch/yaw were not found.`;
}

function extractBnoCalibration() {
  const calibration = latestBySource.bno055?.data?.calibration;
  if (!calibration) {
    return {
      system: null,
      gyro: null,
      accelerometer: null,
      magnetometer: null,
      ready: false,
      message: 'Waiting for BNO055 calibration data.',
    };
  }

  const status = {
    system: Number(calibration.system),
    gyro: Number(calibration.gyro),
    accelerometer: Number(calibration.accelerometer),
    magnetometer: Number(calibration.magnetometer),
  };
  const requiredReady = [status.gyro, status.accelerometer].every((value) => value === 3);

  return {
    ...status,
    ready: requiredReady,
    message: requiredReady
      ? 'BNO055 gyro and accelerometer are calibrated. Magnetometer is informational.'
      : 'Calibrate BNO055 gyro and accelerometer before drift testing.',
  };
}

function angleDeltaDegrees(phoneValue, bnoValue) {
  let delta = phoneValue - bnoValue;
  while (delta > 180) {
    delta -= 360;
  }
  while (delta < -180) {
    delta += 360;
  }
  return round(delta);
}

function subtractAngleBaseline(value, baseline) {
  if (typeof baseline !== 'number') {
    return value;
  }

  let delta = value - baseline;
  while (delta > 180) {
    delta -= 360;
  }
  while (delta < -180) {
    delta += 360;
  }
  return round(delta);
}

function sourceAgeMs(envelope) {
  if (!envelope?.receivedAt) {
    return null;
  }

  return Date.now() - new Date(envelope.receivedAt).getTime();
}

function buildDrift() {
  const phoneEuler = extractEuler(latestBySource.phone);
  const bnoEuler = extractEuler(latestBySource.bno055);
  const bnoCalibration = extractBnoCalibration();

  const base = {
    generatedAt: new Date().toISOString(),
    phone: latestBySource.phone,
    bno055: latestBySource.bno055,
    phoneAgeMs: sourceAgeMs(latestBySource.phone),
    bno055AgeMs: sourceAgeMs(latestBySource.bno055),
    bnoCalibration,
    history: driftHistory.slice(-240),
  };

  if (!phoneEuler || !bnoEuler) {
    return {
      status: 'waiting',
      message: [
        phoneEuler ? null : missingEulerReason('phone', latestBySource.phone),
        bnoEuler ? null : missingEulerReason('BNO055', latestBySource.bno055),
      ].filter(Boolean).join(' '),
      ...base,
      drift: null,
    };
  }

  if (!bnoCalibration.ready) {
    return {
      status: 'calibrating',
      message: bnoCalibration.message,
      ...base,
      phoneEuler,
      bnoEuler,
      drift: null,
    };
  }

  const drift = {
    roll: subtractAngleBaseline(angleDeltaDegrees(phoneEuler.roll, bnoEuler.roll), driftBaseline?.roll),
    pitch: subtractAngleBaseline(angleDeltaDegrees(phoneEuler.pitch, bnoEuler.pitch), driftBaseline?.pitch),
    yaw: subtractAngleBaseline(angleDeltaDegrees(phoneEuler.yaw, bnoEuler.yaw), driftBaseline?.yaw),
    units: 'deg',
    sign: 'phone_minus_bno055',
    baseline: driftBaseline,
  };

  return {
    status: 'ok',
    ...base,
    phoneEuler,
    bnoEuler,
    drift,
    maxAbsDrift: round(Math.max(Math.abs(drift.roll), Math.abs(drift.pitch), Math.abs(drift.yaw))),
  };
}

function buildFastDrift() {
  const drift = buildDrift();
  return {
    status: drift.status,
    message: drift.message,
    generatedAt: drift.generatedAt,
    phoneAgeMs: drift.phoneAgeMs,
    bno055AgeMs: drift.bno055AgeMs,
    bnoCalibration: drift.bnoCalibration,
    syncRateHz: SYNC_RATE_HZ,
    phoneEuler: drift.phoneEuler || null,
    bnoEuler: drift.bnoEuler || null,
    drift: drift.drift,
    maxAbsDrift: drift.maxAbsDrift ?? null,
    history: drift.history,
    imuHistory: {
      phone: imuHistory.phone.slice(-240),
      bno055: imuHistory.bno055.slice(-240),
    },
    map: headingTrace.slice(-240),
  };
}

function resetDriftBaseline() {
  const phoneEuler = extractEuler(latestBySource.phone);
  const bnoEuler = extractEuler(latestBySource.bno055);
  const bnoCalibration = extractBnoCalibration();

  if (!phoneEuler || !bnoEuler) {
    return {
      ok: false,
      message: [
        phoneEuler ? null : missingEulerReason('phone', latestBySource.phone),
        bnoEuler ? null : missingEulerReason('BNO055', latestBySource.bno055),
      ].filter(Boolean).join(' '),
    };
  }

  if (!bnoCalibration.ready) {
    return {
      ok: false,
      message: bnoCalibration.message,
      bnoCalibration,
    };
  }

  driftBaseline = {
    roll: angleDeltaDegrees(phoneEuler.roll, bnoEuler.roll),
    pitch: angleDeltaDegrees(phoneEuler.pitch, bnoEuler.pitch),
    yaw: angleDeltaDegrees(phoneEuler.yaw, bnoEuler.yaw),
    units: 'deg',
    sign: 'phone_minus_bno055',
    resetAt: new Date().toISOString(),
  };
  driftHistory.length = 0;

  return {
    ok: true,
    baseline: driftBaseline,
    drift: buildDrift(),
  };
}

function numericStats(values) {
  const clean = values.filter((value) => Number.isFinite(value));
  if (!clean.length) {
    return { n: 0, min: null, max: null, mean: null, stdDev: null, rms: null };
  }

  const mean = clean.reduce((sum, value) => sum + value, 0) / clean.length;
  const variance = clean.reduce((sum, value) => sum + (value - mean) ** 2, 0) / clean.length;
  const rms = Math.sqrt(clean.reduce((sum, value) => sum + value * value, 0) / clean.length);

  return {
    n: clean.length,
    min: round(Math.min(...clean), 6),
    max: round(Math.max(...clean), 6),
    mean: round(mean, 6),
    stdDev: round(Math.sqrt(variance), 6),
    rms: round(rms, 6),
  };
}

function buildAnalysis() {
  const sourceSummary = (source) => {
    const samples = analysisBuffers[source];
    const axisStats = (prefix) => ({
      x: numericStats(samples.map((sample) => sample[`${prefix}x`])),
      y: numericStats(samples.map((sample) => sample[`${prefix}y`])),
      z: numericStats(samples.map((sample) => sample[`${prefix}z`])),
    });

    return {
      n: samples.length,
      durationSec: samples.length ? round(samples[samples.length - 1].t - samples[0].t, 3) : 0,
      gyro: axisStats('g'),
      accel: axisStats('a'),
      mag: axisStats('m'),
      euler: {
        roll: numericStats(samples.map((sample) => sample.roll)),
        pitch: numericStats(samples.map((sample) => sample.pitch)),
        yaw: numericStats(samples.map((sample) => sample.yaw)),
      },
      latest: samples[samples.length - 1] || null,
    };
  };

  return {
    recording,
    recordStart: recordStartMs ? new Date(recordStartMs).toISOString() : null,
    phone: sourceSummary('phone'),
    bno055: sourceSummary('bno055'),
  };
}

function setRecording(nextRecording) {
  recording = nextRecording;
  if (nextRecording) {
    recordStartMs = Date.now();
    analysisBuffers.phone.length = 0;
    analysisBuffers.bno055.length = 0;
  }

  return buildAnalysis();
}

function clearAnalysis() {
  recording = false;
  recordStartMs = 0;
  analysisBuffers.phone.length = 0;
  analysisBuffers.bno055.length = 0;
  return buildAnalysis();
}

function pushLimited(list, value) {
  list.push(value);
  if (list.length > HISTORY_LIMIT) {
    list.shift();
  }
}

function appendSyncedFrame() {
  const now = Date.now();
  const drift = buildDrift();

  if (drift.phoneEuler) {
    pushLimited(imuHistory.phone, {
      t: now,
      roll: drift.phoneEuler.roll,
      pitch: drift.phoneEuler.pitch,
      yaw: drift.phoneEuler.yaw,
    });
  }

  if (drift.bnoEuler) {
    pushLimited(imuHistory.bno055, {
      t: now,
      roll: drift.bnoEuler.roll,
      pitch: drift.bnoEuler.pitch,
      yaw: drift.bnoEuler.yaw,
    });
  }

  if (drift.status !== 'ok') {
    return;
  }

  pushLimited(driftHistory, {
    t: now,
    roll: drift.drift.roll,
    pitch: drift.drift.pitch,
    yaw: drift.drift.yaw,
  });

  const yawRad = ((drift.drift.yaw || 0) * Math.PI) / 180;
  const previous = headingTrace[headingTrace.length - 1] || { x: 0, y: 0 };
  pushLimited(headingTrace, {
    t: now,
    x: round(previous.x + Math.sin(yawRad), 3),
    y: round(previous.y - Math.cos(yawRad), 3),
    yaw: drift.drift.yaw,
  });

  lastSyncedFrameAt = now;
}

function remember(data, transport, remote, forcedSource = null) {
  data = normalizeIncomingData(data, forcedSource);
  const source = sourceKeyFor(data, forcedSource);
  const envelope = {
    status: 'ok',
    source,
    receivedAt: new Date().toISOString(),
    transport,
    remote,
    data,
  };

  latest = envelope;
  latestBySource[source] = envelope;

  const euler = extractEuler(envelope);
  if (recording && euler && analysisBuffers[source]) {
    const accel = extractVector(data, ['acceleration', 'accelerometer', 'accel']);
    const gyro = extractVector(data, ['gyroscope', 'gyro']);
    const mag = extractVector(data, ['magnetometer', 'mag']);
    analysisBuffers[source].push({
      t: round((Date.now() - recordStartMs) / 1000, 4),
      ax: accel[0],
      ay: accel[1],
      az: accel[2],
      gx: gyro[0],
      gy: gyro[1],
      gz: gyro[2],
      mx: mag[0],
      my: mag[1],
      mz: mag[2],
      roll: euler.roll,
      pitch: euler.pitch,
      yaw: euler.yaw,
    });
    if (analysisBuffers[source].length > 12000) {
      analysisBuffers[source].shift();
    }
  }

  history.unshift(envelope);
  if (history.length > HISTORY_LIMIT) {
    history.length = HISTORY_LIMIT;
  }

  const now = Date.now();
  if (PACKET_LOG_INTERVAL_MS >= 0 && now - lastPacketLogAt >= PACKET_LOG_INTERVAL_MS) {
    lastPacketLogAt = now;
    const seq = typeof data?.seq === 'number' ? ` seq=${data.seq}` : '';
    console.log(`[${envelope.receivedAt}] ${source} ${transport} ${remote}${seq}`);
  }
}

function sendJson(response, statusCode, body) {
  const text = JSON.stringify(body, null, 2);
  response.writeHead(statusCode, {
    'Access-Control-Allow-Origin': '*',
    'Access-Control-Allow-Methods': 'GET,POST,OPTIONS',
    'Access-Control-Allow-Headers': 'Content-Type',
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'no-store',
  });
  response.end(text);
}

function sendVisualizer(response) {
  const body = `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>IMU Drift Visualizer</title>
  <style>
    * { box-sizing: border-box; }
    body { margin: 0; background: #060708; color: #f5f7f8; font-family: Arial, sans-serif; }
    main { max-width: 1120px; margin: 0 auto; padding: 22px; }
    h1 { margin: 0 0 8px; font-size: 28px; }
    a { color: #65c8ff; }
    canvas { width: 100%; height: 260px; border: 1px solid #2f3438; border-radius: 8px; background: #101214; }
    .muted { color: #a7afb5; }
    .top { display: flex; justify-content: space-between; gap: 16px; align-items: flex-start; flex-wrap: wrap; margin-bottom: 18px; }
    .status { border: 1px solid #2f3438; border-radius: 8px; padding: 10px 12px; background: #101214; }
    button { border: 1px solid #6a737b; border-radius: 8px; background: #f5f7f8; color: #101214; cursor: pointer; font: inherit; font-weight: 700; padding: 10px 14px; }
    button:disabled { cursor: wait; opacity: 0.7; }
    .controls { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; margin-top: 10px; }
    .grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; margin: 18px 0; }
    .panel { border: 1px solid #2f3438; border-radius: 8px; padding: 14px; background: #101214; }
    .axis { font-size: 15px; color: #a7afb5; margin-bottom: 6px; }
    .value { font-size: 34px; font-weight: 700; font-variant-numeric: tabular-nums; }
    .bar { height: 12px; border-radius: 6px; background: #22272b; overflow: hidden; margin-top: 10px; }
    .fill { height: 100%; width: 0%; background: #e9cd37; }
    .source-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; margin: 18px 0; }
    .plot-grid { display: grid; grid-template-columns: minmax(0, 1.35fr) minmax(280px, 0.65fr); gap: 12px; margin: 18px 0; }
    .plot-title { color: #f5f7f8; font-size: 18px; font-weight: 700; margin: 0 0 8px; }
    .calibration { border: 1px solid #2f3438; border-radius: 8px; padding: 14px; background: #101214; margin: 18px 0; }
    .calibration-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; margin-top: 12px; }
    .calibration-item { border: 1px solid #2f3438; border-radius: 8px; padding: 10px; }
    .calibration-name { color: #a7afb5; font-size: 14px; margin-bottom: 6px; }
    .calibration-value { font-size: 24px; font-weight: 700; font-variant-numeric: tabular-nums; }
    .calibration-steps { color: #d9dee2; line-height: 1.45; margin: 10px 0 0; padding-left: 20px; }
    table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
    td { padding: 5px 0; border-bottom: 1px solid #24282c; }
    td:last-child { text-align: right; }
    pre { overflow: auto; max-height: 260px; padding: 12px; border: 1px solid #2f3438; border-radius: 8px; background: #101214; color: #d9dee2; }
    @media (max-width: 900px) { .plot-grid { grid-template-columns: 1fr; } }
    @media (max-width: 760px) { .grid, .source-grid, .calibration-grid { grid-template-columns: 1fr; } .value { font-size: 28px; } }
  </style>
</head>
<body>
  <main>
    <div class="top">
      <div>
        <h1>IMU Drift Visualizer</h1>
        <div class="muted">Drift = phone Euler angle minus BNO055 Euler angle. Yaw wraps at +/-180 deg.</div>
      </div>
      <div class="status">
        <div id="state">Waiting</div>
        <div class="muted" id="ages">Phone -- ms | BNO055 -- ms</div>
        <div class="muted" id="baseline">Baseline: none</div>
        <div class="controls"><button id="resetDrift" type="button">Reset Drift</button><span class="muted" id="resetState">Sets current drift to zero</span></div>
        <div><a href="/drift-fast.json">drift-fast.json</a> | <a href="/drift.json">drift.json</a> | <a href="/phone.json">phone.json</a> | <a href="/bno055.json">bno055.json</a></div>
      </div>
    </div>

    <div class="grid">
      <div class="panel"><div class="axis">Roll drift</div><div class="value" id="roll">--</div><div class="bar"><div class="fill" id="rollBar"></div></div></div>
      <div class="panel"><div class="axis">Pitch drift</div><div class="value" id="pitch">--</div><div class="bar"><div class="fill" id="pitchBar"></div></div></div>
      <div class="panel"><div class="axis">Yaw drift</div><div class="value" id="yaw">--</div><div class="bar"><div class="fill" id="yawBar"></div></div></div>
    </div>

    <div class="calibration">
      <h2>BNO055 Calibration</h2>
      <div id="calibrationState" class="muted">Waiting for BNO055 calibration data.</div>
      <div class="calibration-grid">
        <div class="calibration-item"><div class="calibration-name">System</div><div class="calibration-value" id="calSystem">--/3</div></div>
        <div class="calibration-item"><div class="calibration-name">Gyroscope</div><div class="calibration-value" id="calGyro">--/3</div></div>
        <div class="calibration-item"><div class="calibration-name">Accelerometer</div><div class="calibration-value" id="calAccel">--/3</div></div>
        <div class="calibration-item"><div class="calibration-name">Magnetometer</div><div class="calibration-value" id="calMag">--/3</div></div>
      </div>
      <ol class="calibration-steps">
        <li>Gyroscope: keep the board completely still for a few seconds.</li>
        <li>Accelerometer: place the board still on six different faces. Hold each face for 2-3 seconds. Do not shake it.</li>
        <li>Magnetometer: optional for this drift test. Its value is shown but not required.</li>
      </ol>
    </div>

    <div class="plot-grid">
      <div>
        <h2 class="plot-title">Drift</h2>
        <canvas id="chart" width="1000" height="260"></canvas>
      </div>
      <div>
        <h2 class="plot-title">Heading Map</h2>
        <canvas id="mapCanvas" width="420" height="260"></canvas>
      </div>
    </div>

    <div>
      <h2 class="plot-title">Phone IMU Euler</h2>
      <canvas id="phonePlot" width="1000" height="240"></canvas>
    </div>

    <div>
      <h2 class="plot-title">BNO055 IMU Euler</h2>
      <canvas id="bnoPlot" width="1000" height="240"></canvas>
    </div>

    <div class="source-grid">
      <div class="panel">
        <h2>Phone</h2>
        <table><tbody id="phoneTable"></tbody></table>
      </div>
      <div class="panel">
        <h2>BNO055</h2>
        <table><tbody id="bnoTable"></tbody></table>
      </div>
    </div>

    <pre id="raw">{}</pre>
  </main>

  <script>
    const ids = ['roll', 'pitch', 'yaw'];
    const colors = { roll: '#e9cd37', pitch: '#65c8ff', yaw: '#f06f61' };

    function fmt(value, digits) {
      return typeof value === 'number' ? value.toFixed(digits) : '--';
    }

    function setText(id, value) {
      document.getElementById(id).textContent = value;
    }

    function setCalibrationValue(id, value, ready) {
      const element = document.getElementById(id);
      element.textContent = typeof value === 'number' ? value + '/3' : '--/3';
      element.style.color = ready ? '#9ad66f' : '#f06f61';
    }

    function tableRows(euler) {
      if (!euler) {
        return '<tr><td>Status</td><td>waiting</td></tr>';
      }
      return '<tr><td>Roll</td><td>' + fmt(euler.roll, 4) + ' deg</td></tr>' +
        '<tr><td>Pitch</td><td>' + fmt(euler.pitch, 4) + ' deg</td></tr>' +
        '<tr><td>Yaw</td><td>' + fmt(euler.yaw, 4) + ' deg</td></tr>';
    }

    function drawSeries(canvasId, history, title) {
      const canvas = document.getElementById(canvasId);
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = '#2f3438';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y = 20 + i * 55;
        ctx.beginPath();
        ctx.moveTo(30, y);
        ctx.lineTo(canvas.width - 10, y);
        ctx.stroke();
      }

      if (!history || history.length < 2) {
        ctx.fillStyle = '#a7afb5';
        ctx.fillText('Waiting for ' + title + '...', 34, 36);
        return;
      }

      const maxAbs = Math.max(1, ...history.flatMap((point) => ids.map((id) => Math.abs(point[id] || 0))));
      const left = 34;
      const right = canvas.width - 12;
      const top = 14;
      const bottom = canvas.height - 18;
      const mid = (top + bottom) / 2;
      const xFor = (index) => left + (index / (history.length - 1)) * (right - left);
      const yFor = (value) => mid - (value / maxAbs) * ((bottom - top) / 2);

      ids.forEach((id) => {
        ctx.strokeStyle = colors[id];
        ctx.lineWidth = 2;
        ctx.beginPath();
        history.forEach((point, index) => {
          const x = xFor(index);
          const y = yFor(point[id] || 0);
          if (index === 0) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        ctx.stroke();
      });

      ctx.fillStyle = '#d9dee2';
      ctx.fillText('roll', 34, 18);
      ctx.fillStyle = colors.pitch;
      ctx.fillText('pitch', 82, 18);
      ctx.fillStyle = colors.yaw;
      ctx.fillText('yaw', 140, 18);
    }

    function drawChart(history) {
      drawSeries('chart', history, 'both streams');
    }

    function drawMap(points) {
      const canvas = document.getElementById('mapCanvas');
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = '#2f3438';
      ctx.lineWidth = 1;
      for (let x = 20; x < canvas.width; x += 40) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
      }
      for (let y = 20; y < canvas.height; y += 40) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }

      if (!points || points.length < 2) {
        ctx.fillStyle = '#a7afb5';
        ctx.fillText('Waiting for drift heading...', 24, 34);
        return;
      }

      const xs = points.map((point) => point.x || 0);
      const ys = points.map((point) => point.y || 0);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const span = Math.max(1, maxX - minX, maxY - minY);
      const pad = 24;
      const xFor = (x) => pad + ((x - minX) / span) * (canvas.width - pad * 2);
      const yFor = (y) => pad + ((y - minY) / span) * (canvas.height - pad * 2);

      ctx.strokeStyle = '#65c8ff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      points.forEach((point, index) => {
        const x = xFor(point.x || 0);
        const y = yFor(point.y || 0);
        if (index === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();

      const latest = points[points.length - 1];
      const latestX = xFor(latest.x || 0);
      const latestY = yFor(latest.y || 0);
      const yawRad = ((latest.yaw || 0) * Math.PI) / 180;
      ctx.fillStyle = '#f06f61';
      ctx.beginPath();
      ctx.arc(latestX, latestY, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#f06f61';
      ctx.beginPath();
      ctx.moveTo(latestX, latestY);
      ctx.lineTo(latestX + Math.sin(yawRad) * 24, latestY - Math.cos(yawRad) * 24);
      ctx.stroke();
    }

    async function resetDrift() {
      const button = document.getElementById('resetDrift');
      const resetState = document.getElementById('resetState');
      button.disabled = true;
      resetState.textContent = 'Resetting...';

      try {
        const response = await fetch('/drift/reset', { method: 'POST', cache: 'no-store' });
        const result = await response.json();
        if (!result.ok) {
          resetState.textContent = result.message || 'Reset unavailable';
          return;
        }
        resetState.textContent = 'Reset';
        await refresh();
      } catch (error) {
        resetState.textContent = String(error);
      } finally {
        button.disabled = false;
      }
    }

    async function refresh() {
      try {
        const response = await fetch('/drift-fast.json', { cache: 'no-store' });
        const drift = await response.json();
        setText('state', drift.status === 'ok' ? 'Streaming drift' : drift.message);
        setText('ages', 'Phone ' + fmt(drift.phoneAgeMs, 0) + ' ms | BNO055 ' + fmt(drift.bno055AgeMs, 0) + ' ms');
        const calibration = drift.bnoCalibration || {};
        setText('calibrationState', calibration.message || 'Waiting for BNO055 calibration data.');
        setCalibrationValue('calSystem', calibration.system, calibration.system === 3);
        setCalibrationValue('calGyro', calibration.gyro, calibration.gyro === 3);
        setCalibrationValue('calAccel', calibration.accelerometer, calibration.accelerometer === 3);
        setCalibrationValue('calMag', calibration.magnetometer, calibration.magnetometer === 3);
        const baseline = drift.drift && drift.drift.baseline;
        setText('baseline', baseline
          ? 'Baseline roll ' + fmt(baseline.roll, 4) + ' deg | pitch ' + fmt(baseline.pitch, 4) + ' deg | yaw ' + fmt(baseline.yaw, 4) + ' deg'
          : 'Baseline: none');
        document.getElementById('resetDrift').disabled = false;

        ids.forEach((id) => {
          const value = drift.drift && drift.drift[id];
          setText(id, fmt(value, 4) + ' deg');
          const width = typeof value === 'number' ? Math.min(100, Math.abs(value) / 45 * 100) : 0;
          const bar = document.getElementById(id + 'Bar');
          bar.style.width = width + '%';
          bar.style.background = colors[id];
        });

        document.getElementById('phoneTable').innerHTML = tableRows(drift.phoneEuler);
        document.getElementById('bnoTable').innerHTML = tableRows(drift.bnoEuler);
        document.getElementById('raw').textContent = JSON.stringify(drift, null, 2);
        drawChart(drift.history || []);
        drawSeries('phonePlot', drift.imuHistory && drift.imuHistory.phone, 'phone IMU');
        drawSeries('bnoPlot', drift.imuHistory && drift.imuHistory.bno055, 'BNO055 IMU');
        drawMap(drift.map || []);
      } catch (error) {
        setText('state', String(error));
      }
    }

    document.getElementById('resetDrift').addEventListener('click', resetDrift);
    refresh();
    setInterval(refresh, 50);
  </script>
</body>
</html>`;

  response.writeHead(200, {
    'Access-Control-Allow-Origin': '*',
    'Content-Type': 'text/html; charset=utf-8',
    'Cache-Control': 'no-store',
  });
  response.end(body);
}

function sendAnalysisVisualizer(response) {
  const body = `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>IMU Research Analysis</title>
  <style>
    * { box-sizing: border-box; }
    body { margin: 0; background: #060708; color: #f5f7f8; font-family: Arial, sans-serif; }
    main { max-width: 1120px; margin: 0 auto; padding: 22px; }
    h1 { margin: 0 0 8px; font-size: 28px; }
    h2 { margin: 0 0 10px; font-size: 18px; }
    button { border: 1px solid #6a737b; border-radius: 8px; background: #f5f7f8; color: #101214; cursor: pointer; font: inherit; font-weight: 700; padding: 10px 14px; }
    .muted { color: #a7afb5; }
    .top { display: flex; justify-content: space-between; gap: 16px; align-items: flex-start; flex-wrap: wrap; margin-bottom: 18px; }
    .controls { display: flex; gap: 10px; flex-wrap: wrap; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
    .panel { border: 1px solid #2f3438; border-radius: 8px; padding: 14px; background: #101214; }
    .metric-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    .metric { border: 1px solid #24282c; border-radius: 8px; padding: 10px; }
    .label { color: #a7afb5; font-size: 12px; }
    .value { color: #f5f7f8; font-size: 20px; font-weight: 700; font-variant-numeric: tabular-nums; }
    table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
    td { padding: 6px 0; border-bottom: 1px solid #24282c; }
    td:last-child { text-align: right; }
    pre { max-height: 280px; overflow: auto; border: 1px solid #2f3438; border-radius: 8px; padding: 12px; background: #101214; color: #d9dee2; }
    @media (max-width: 760px) { .grid, .metric-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <main>
    <div class="top">
      <div>
        <h1>IMU Research Analysis</h1>
        <div class="muted">Record stationary samples, then compare gyro noise, bias, RMS, and orientation spread.</div>
      </div>
      <div class="controls">
        <button id="start" type="button">Start Recording</button>
        <button id="stop" type="button">Stop</button>
        <button id="clear" type="button">Clear</button>
        <a class="muted" href="/visualizer">Live visualizer</a>
      </div>
    </div>
    <div id="state" class="panel muted">Waiting</div>
    <div class="grid" id="sources"></div>
    <h2 style="margin-top:18px">Raw summary</h2>
    <pre id="raw">{}</pre>
  </main>
  <script>
    function fmt(value, digits = 4) {
      return typeof value === 'number' ? value.toFixed(digits) : '--';
    }

    function statRows(title, stats) {
      return '<h2>' + title + '</h2><table><tbody>' +
        Object.entries(stats).map(([axis, s]) =>
          '<tr><td>' + axis.toUpperCase() + ' RMS</td><td>' + fmt(s.rms, 6) + '</td></tr>' +
          '<tr><td>' + axis.toUpperCase() + ' Std Dev</td><td>' + fmt(s.stdDev, 6) + '</td></tr>' +
          '<tr><td>' + axis.toUpperCase() + ' Bias</td><td>' + fmt(s.mean, 6) + '</td></tr>'
        ).join('') +
        '</tbody></table>';
    }

    function sourcePanel(name, source) {
      return '<section class="panel">' +
        '<h2>' + name + '</h2>' +
        '<div class="metric-grid">' +
          '<div class="metric"><div class="label">Samples</div><div class="value">' + source.n + '</div></div>' +
          '<div class="metric"><div class="label">Duration</div><div class="value">' + fmt(source.durationSec, 2) + ' s</div></div>' +
          '<div class="metric"><div class="label">Yaw Std Dev</div><div class="value">' + fmt(source.euler.yaw.stdDev, 3) + '</div></div>' +
        '</div>' +
        statRows('Gyro rad/s', source.gyro) +
        statRows('Accel m/s2', source.accel) +
      '</section>';
    }

    async function post(path) {
      await fetch(path, { method: 'POST', cache: 'no-store' });
      await refresh();
    }

    async function refresh() {
      const response = await fetch('/analysis.json', { cache: 'no-store' });
      const data = await response.json();
      document.getElementById('state').textContent = data.recording ? 'Recording samples...' : 'Recording stopped';
      document.getElementById('sources').innerHTML = sourcePanel('Phone', data.phone) + sourcePanel('BNO055', data.bno055);
      document.getElementById('raw').textContent = JSON.stringify(data, null, 2);
    }

    document.getElementById('start').addEventListener('click', () => post('/analysis/start'));
    document.getElementById('stop').addEventListener('click', () => post('/analysis/stop'));
    document.getElementById('clear').addEventListener('click', () => post('/analysis/clear'));
    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>`;

  response.writeHead(200, {
    'Access-Control-Allow-Origin': '*',
    'Content-Type': 'text/html; charset=utf-8',
    'Cache-Control': 'no-store',
  });
  response.end(body);
}

function readBodyText(request) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    let size = 0;

    request.on('data', (chunk) => {
      size += chunk.length;
      if (size > MAX_BODY_BYTES) {
        reject(new Error('Request body is too large'));
        request.destroy();
        return;
      }
      chunks.push(chunk);
    });

    request.on('end', () => {
      resolve(Buffer.concat(chunks).toString('utf8'));
    });

    request.on('error', reject);
  });
}

async function receivePost(request, response, forcedSource) {
  try {
    const text = await readBodyText(request);
    const data = parseIncomingText(text);
    remember(data, 'http', request.socket.remoteAddress || 'unknown', forcedSource);
    sendJson(response, 200, {
      ok: true,
      source: sourceKeyFor(data, forcedSource),
      receivedAt: latest.receivedAt,
      driftJson: `http://${request.headers.host || `localhost:${HTTP_PORT}`}/drift.json`,
    });
  } catch (error) {
    sendJson(response, 400, {
      ok: false,
      error: error instanceof Error ? error.message : String(error),
    });
  }
}

const server = http.createServer(async (request, response) => {
  const url = new URL(request.url || '/', `http://${request.headers.host || `localhost:${HTTP_PORT}`}`);

  if (request.method === 'OPTIONS') {
    sendJson(response, 200, { ok: true });
    return;
  }

  if (request.method === 'GET' && (url.pathname === '/' || url.pathname === '/visualizer')) {
    sendVisualizer(response);
    return;
  }

  if (request.method === 'GET' && url.pathname === '/analysis') {
    sendAnalysisVisualizer(response);
    return;
  }

  if (request.method === 'GET' && url.pathname === '/health') {
    sendJson(response, 200, {
      ok: true,
      httpPort: HTTP_PORT,
      udpPort: UDP_PORT,
      syncRateHz: SYNC_RATE_HZ,
      lastSyncedFrameAt,
      addresses: localIpv4Addresses(),
    });
    return;
  }

  if (request.method === 'GET' && url.pathname === '/imu.json') {
    sendJson(response, 200, latest);
    return;
  }

  if (request.method === 'GET' && url.pathname === '/phone.json') {
    sendJson(response, 200, latestBySource.phone);
    return;
  }

  if (request.method === 'GET' && url.pathname === '/bno055.json') {
    sendJson(response, 200, latestBySource.bno055);
    return;
  }

  if (request.method === 'GET' && url.pathname === '/drift.json') {
    sendJson(response, 200, buildDrift());
    return;
  }

  if (request.method === 'GET' && url.pathname === '/drift-fast.json') {
    sendJson(response, 200, buildFastDrift());
    return;
  }

  if (request.method === 'POST' && url.pathname === '/drift/reset') {
    const result = resetDriftBaseline();
    sendJson(response, result.ok ? 200 : 409, result);
    return;
  }

  if (request.method === 'GET' && url.pathname === '/history.json') {
    const limit = Math.max(1, Math.min(HISTORY_LIMIT, Number(url.searchParams.get('limit') || 50)));
    sendJson(response, 200, history.slice(0, limit));
    return;
  }

  if (request.method === 'GET' && url.pathname === '/analysis.json') {
    sendJson(response, 200, buildAnalysis());
    return;
  }

  if (request.method === 'POST' && url.pathname === '/analysis/start') {
    sendJson(response, 200, setRecording(true));
    return;
  }

  if (request.method === 'POST' && url.pathname === '/analysis/stop') {
    sendJson(response, 200, setRecording(false));
    return;
  }

  if (request.method === 'POST' && url.pathname === '/analysis/clear') {
    sendJson(response, 200, clearAnalysis());
    return;
  }

  if (request.method === 'POST' && url.pathname === '/imu') {
    await receivePost(request, response, null);
    return;
  }

  if (request.method === 'POST' && url.pathname === '/phone') {
    await receivePost(request, response, 'phone');
    return;
  }

  if (request.method === 'POST' && url.pathname === '/bno055') {
    await receivePost(request, response, 'bno055');
    return;
  }

  sendJson(response, 404, {
    ok: false,
    error: 'Not found',
    routes: [
      'GET /visualizer',
      'GET /analysis',
      'GET /drift.json',
      'GET /drift-fast.json',
      'GET /analysis.json',
      'GET /phone.json',
      'GET /bno055.json',
      'GET /history.json',
      'POST /imu',
      'POST /phone',
      'POST /bno055',
      'POST /drift/reset',
      'POST /analysis/start',
      'POST /analysis/stop',
      'POST /analysis/clear',
    ],
  });
});

const udp = dgram.createSocket('udp4');

udp.on('message', (message, remote) => {
  const text = message.toString('utf8');

  try {
    remember(parseIncomingText(text), 'udp', `${remote.address}:${remote.port}`);
  } catch (error) {
    remember(
      {
        parseError: error instanceof Error ? error.message : String(error),
        raw: text,
      },
      'udp',
      `${remote.address}:${remote.port}`
    );
  }
});

udp.on('error', (error) => {
  console.error(`UDP error: ${error.message}`);
});

server.listen(HTTP_PORT, HOST, () => {
  console.log(`HTTP drift visualizer listening on http://${HOST}:${HTTP_PORT}`);
  console.log(`Synchronized comparison output rate: ${SYNC_RATE_HZ} Hz`);
  for (const address of localIpv4Addresses()) {
    console.log(`  Phone app POST URL: http://${address}:${HTTP_PORT}/imu`);
    console.log(`  BNO055 POST URL:    http://${address}:${HTTP_PORT}/bno055`);
    console.log(`  Visualizer URL:     http://${address}:${HTTP_PORT}/visualizer`);
    console.log(`  Analysis URL:       http://${address}:${HTTP_PORT}/analysis`);
    console.log(`  Drift JSON URL:     http://${address}:${HTTP_PORT}/drift.json`);
  }
});

udp.bind(UDP_PORT, HOST, () => {
  console.log(`UDP JSON receiver listening on ${HOST}:${UDP_PORT}`);
});

setInterval(appendSyncedFrame, Math.round(1000 / SYNC_RATE_HZ));
