const MAX_SAMPLES = 300;
const $ = (id) => document.getElementById(id);
const state = {
  lastPacketAt: 0,
  received: 0,
  rateStart: performance.now(),
  rate: 0,
  reconnectAttempts: 0,
  q: [[], [], []],
  target: [[], [], []],
  inference: []
};

function text(id, value) { $(id).textContent = value; }
function number(value, digits = 3) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}
function push(list, value) {
  list.push(value);
  if (list.length > MAX_SAMPLES) list.shift();
}
function setConnection(connected) {
  text("ws-state", connected ? "Connected" : "Disconnected");
  const badge = $("connection-badge");
  badge.textContent = connected ? "CONNECTED" : "DISCONNECTED";
  badge.className = `badge ${connected ? "ok" : "warn"}`;
}
function applyPacket(packet) {
  setConnection(true);
  state.lastPacketAt = performance.now();
  state.received++;
  const now = performance.now();
  if (now - state.rateStart >= 1000) {
    state.rate = state.received * 1000 / (now - state.rateStart);
    state.received = 0;
    state.rateStart = now;
  }
  text("rx-rate", `${number(state.rate, 1)} Hz`);
  text("uptime", `${(packet.uptime_ms / 1000).toFixed(1)} s`);
  text("heap", `${packet.heap_internal_free} B`);
  text("heap-min", `${packet.heap_internal_min} B`);
  text("rssi", packet.rssi === null ? "Unavailable" : `${packet.rssi} dBm`);
  text("robot-state", packet.state_name || "--");
  text("fault", `${packet.fault_code} ${packet.fault_name || ""}`);
  text("command-age", `${packet.command_age_ms} ms`);
  text("telemetry-drops", packet.telemetry_drops);
  text("ws-failures", packet.ws_send_failures);
  const servo = $("servo-state");
  servo.textContent = packet.enabled ? "SERVOS ON" : "SERVOS OFF";
  servo.className = `pill ${packet.enabled ? "ok" : "warn"}`;

  for (let i = 0; i < 3; i++) {
    text(`q${i}`, number(packet.q_rad[i]));
    text(`qd${i}`, number(packet.qd_rad_s[i]));
    text(`target${i}`, number(packet.target_rad[i]));
    text(`action${i}`, number(packet.action_norm[i]));
    text(`acc${i}`, number(packet.acc_mps2[i]));
    text(`gyro${i}`, number(packet.gyro_rad_s[i]));
    push(state.q[i], packet.q_rad[i]);
    push(state.target[i], packet.target_rad[i]);
  }

  text("imu-rate", packet.imu_configured_hz);
  text("joint-rate", packet.joint_configured_hz);
  text("policy-rate", `${number(packet.policy_hz, 1)} Hz`);
  text("inference", `${packet.inference_us} us`);
  text("inference-max", `${packet.inference_max_us} us`);
  text("deadline-misses", packet.deadline_misses);
  text("cmd0", `${number(packet.cmd_mps[0])} m/s`);
  text("cmd1", `${number(packet.cmd_mps[1])} m/s`);
  push(state.inference, packet.inference_us);

  const positionColors = ["#4dd9e8", "#69d391", "#f4cf65"];
  const targetColors = ["#ffad5c", "#ff7b9c", "#b99cff"];
  drawChart($("joint-chart"), [
    ...state.q.map((values, i) => ({ values, color: positionColors[i] })),
    ...state.target.map((values, i) => ({ values, color: targetColors[i] }))
  ]);
  drawChart($("inference-chart"), [{ values: state.inference, color: "#b99cff" }]);
}

function drawChart(canvas, series) {
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "#293444";
  ctx.lineWidth = 1;
  for (let i = 1; i < 5; i++) {
    const y = i * height / 5;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
  }
  const values = series.flatMap((item) => item.values).filter(Number.isFinite);
  if (!values.length) return;
  let minimum = Math.min(...values);
  let maximum = Math.max(...values);
  if (minimum === maximum) { minimum -= 1; maximum += 1; }
  series.forEach((item) => {
    if (item.values.length < 2) return;
    ctx.strokeStyle = item.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    item.values.forEach((value, index) => {
      const x = index * width / (MAX_SAMPLES - 1);
      const y = height - (value - minimum) * height / (maximum - minimum);
      index ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  });
}

function connect() {
  const socketUrl = `ws://${location.hostname}:81/`;
  const socket = new WebSocket(socketUrl);
  socket.onopen = () => { state.reconnectAttempts = 0; setConnection(true); };
  socket.onclose = () => {
    setConnection(false);
    if (state.reconnectAttempts++ < 10) setTimeout(connect, 1000);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = (event) => {
    try { applyPacket(JSON.parse(event.data)); } catch (_) { /* ignore */ }
  };
}

setInterval(() => {
  const age = state.lastPacketAt ? performance.now() - state.lastPacketAt : Infinity;
  text("packet-age", Number.isFinite(age) ? `${Math.round(age)} ms` : "--");
  if (age > 500) setConnection(false);
}, 100);

setConnection(false);
connect();
