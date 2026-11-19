"use strict";

const $ = id => document.getElementById(id);
const sides = ["left", "right"];
const armName = {left: "左臂", right: "右臂"};
const endpoints = {wrist: "手腕", thumb: "拇指", index: "食指", middle: "中指", ring: "无名指", little: "小指"};
const states = {
  QUEUED: "执行准备中", PREFLIGHT: "复核起点与保护参数", HOLDING: "正在使能",
  EXECUTING: "执行中", STOP_REQUESTED: "正在停止", STOPPED: "正在核对失能",
  HOLDING_TARGET: "已到位 · 使能保持中",
  COMPLETED: "已到位", CANCELLED: "已取消", FAULT: "执行失败", STOP_UNCONFIRMED: "停止未确认",
};
const labels = {
  "drive.enable_all": "双臂全部使能", "drive.mode": "电机模式切换", "drive.holding": "使能保持中",
  "drive.mode_failed": "模式切换未确认", "drive.mode_axis": "逐轴切换模式",
  "service.ready": "控制服务就绪", "service.fault": "控制服务故障",
  "feedback.online": "反馈已连接", "feedback.offline": "反馈断开", "feedback.fault": "电机反馈故障",
  "motion.preview": "目标已通过预检", "motion.accepted": "指令已受理",
  "motion.preflight": "执行前校验", "motion.fault": "控制器故障",
  "motion.advisory": "阈值已记录，未中断执行", "motion.notice": "执行提示",
  "motion.completed": "运动完成", "motion.cancelled": "执行已停止",
  "motion.stop_unconfirmed": "失能未确认", "motion.rejected": "指令被拒绝",
  "motion.after": "完成失能回读", "motion.disable_evidence": "失能记录",
};
const state = {
  mode: "joint", arm: "left", status: null, run: null, target: null, pending: false,
  stopping: false, instance: null, cursor: 0, logs: [], filter: "ALL", notice: null,
};
const client = location.protocol === "file:" ? "" : crypto.randomUUID().replaceAll("-", "");
const jointInputs = {};
const ikInputs = {};
const jointCells = {};
const endpointCells = {};

function create(tag, text, className) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
}

function selectedArms() {
  return state.arm === "both" ? sides : [state.arm];
}

function buildFields() {
  for (const side of sides) {
    jointInputs[side] = [];
    jointCells[side] = [];
    endpointCells[side] = {};
    const group = create("div", undefined, "ik-group");
    group.dataset.ikArm = side;
    group.append(create("strong", armName[side]));
    const xyz = create("div", undefined, "xyz");
    ikInputs[side] = ["X", "Y", "Z"].map(axis => {
      const label = create("label", axis);
      const unit = create("div", undefined, "input-unit");
      const input = create("input");
      input.type = "number";
      input.step = "any";
      input.required = true;
      input.setAttribute("aria-label", armName[side] + " " + axis);
      unit.append(input, create("span", "mm"));
      label.append(unit);
      xyz.append(label);
      return input;
    });
    group.append(xyz);
    $("ik-inputs").append(group);
  }
  for (let i = 1; i <= 7; i++) {
    const inputRow = create("div", undefined, "joint-input-row");
    inputRow.append(create("span", "J" + i));
    const feedbackRow = create("tr");
    feedbackRow.append(create("th", "J" + i));
    for (const side of sides) {
      const input = create("input");
      input.type = "number";
      input.step = "any";
      input.setAttribute("aria-label", armName[side] + " J" + i + " 目标角度");
      jointInputs[side].push(input);
      inputRow.append(input);
      const cell = create("td", "—");
      jointCells[side].push(cell);
      feedbackRow.append(cell);
    }
    $("joint-inputs").append(inputRow);
    $("joint-status").append(feedbackRow);
  }
  for (const [endpoint, label] of Object.entries(endpoints)) {
    const row = create("tr");
    row.dataset.endpoint = endpoint;
    row.append(create("th", label));
    for (const side of sides) {
      endpointCells[side][endpoint] = create("td", "—");
      row.append(endpointCells[side][endpoint]);
    }
    $("endpoint-status").append(row);
  }
}

function invalidate(message = "填写目标后，先生成预览。") {
  state.target = null;
  $("target-angles").replaceChildren();
  $("target-state").textContent = "尚未生成";
  $("target-note").textContent = message;
  $("execute").disabled = true;
}

function choose(attribute, value) {
  document.querySelectorAll(`[data-${attribute}]`).forEach(button => {
    const selected = button.dataset[attribute] === value;
    button.classList.toggle("active", selected);
    button.setAttribute("aria-pressed", selected);
  });
}

function updateControls() {
  const status = state.status;
  const holding = state.run?.active && state.run.state === "HOLDING_TARGET" && status?.can_prepare;
  const locked = state.pending || (state.run?.active && !holding);
  const drive = $("drive-mode");
  drive.disabled = state.pending || !status?.can_switch_mode;
  drive.title = status?.drive_modes?.length === 1 ? "实机执行协议为 MIT，其他模式待固件核验" :
    state.run?.active ? "停止并确认失能后可切换模式" : "切换双臂全部 14 轴的电机模式";
  drive.value = status?.drive_mode || "";
  for (const option of drive.options) option.disabled = !status?.drive_modes?.includes(option.value);
  $("enable-all").disabled = state.pending || !status?.can_enable_all || Boolean(state.run?.active);
  const joints = status && sides.flatMap(side => status.arms[side].joints);
  $("enabled-count").textContent = "使能 " + (joints?.every(Boolean) ?
    joints.filter(joint => joint.state === 1).length : "—") + "/14";
  const ik = state.mode === "ik";
  const selected = selectedArms();
  $("control-fields").disabled = !status || locked;
  document.querySelectorAll("[data-mode]").forEach(button => { button.disabled = locked; });
  $("ik-fields").hidden = !ik;
  $("joint-fields").hidden = ik;
  $("prepare").textContent = ik ? "求解与预检" : "预览关节目标";
  $("prepare").disabled = !status?.can_prepare || locked;
  $("use-current").disabled = !selected.every(side => status?.arms[side].endpoints_mm[$("endpoint").value]);
  for (const side of sides) {
    document.querySelector(`[data-ik-arm="${side}"]`).hidden = !selected.includes(side);
    ikInputs[side].forEach(input => { input.disabled = !ik || !selected.includes(side); });
    jointInputs[side].forEach(input => { input.disabled = ik || !selected.includes(side); });
  }
  if (state.target && performance.now() >= state.target.expiresAt) {
    invalidate("预览已过期，请重新生成。");
  }
  $("execute").textContent = status?.mode === "hardware" ? "执行实机目标" : "执行离线目标";
  $("execute").disabled = !status?.can_prepare || locked || !state.target;
  $("stop").disabled = !state.run?.active || state.stopping || Boolean(state.run?.stop_reason);
}

function setConnection(type, title) {
  $("connection").dataset.status = type;
  $("connection").lastElementChild.textContent = title;
}

function renderSensors() {
  const status = state.status;
  const virtual = status?.mode === "virtual";
  const metric = $("joint-metric").value;
  $("endpoint-label").textContent = virtual ? "手腕与 Aero 模型指尖" : "手腕位置";
  $("endpoint-note").textContent = status?.mode === "hardware" ? "手腕坐标由关节回读换算。" : "";
  document.querySelectorAll("[data-endpoint]").forEach(row => {
    row.hidden = row.dataset.endpoint !== "wrist" && !virtual;
  });
  Array.from($("endpoint").options).slice(1).forEach(option => { option.disabled = !virtual; });
  if (!virtual && $("endpoint").value !== "wrist") $("endpoint").value = "wrist";
  for (const side of sides) {
    const arm = status?.arms[side];
    const aid = $(side + "-status");
    aid.textContent = arm?.reason ? " · " + (arm.fresh ? "异常" : "无反馈") :
      arm?.fresh ? ` · ${arm.age_ms} ms` : " · —";
    aid.title = arm?.reason || "";
    aid.dataset.status = arm?.fresh && !arm.reason ? "online" : "offline";
    for (const [index, cell] of jointCells[side].entries()) {
      const row = arm?.joints[index];
      const value = row?.[metric];
      cell.textContent = value == null ? "—" : metric === "state" ?
        ({0: "失能", 1: "使能"}[value] ?? "故障 " + value) : value.toFixed(1);
      const input = jointInputs[side][index];
      input.placeholder = row ? row.position_deg.toFixed(2) : "—";
      if (status) [input.min, input.max] = status.limits_deg[side][index];
    }
    for (const [endpoint, cell] of Object.entries(endpointCells[side])) {
      const coords = arm?.endpoints_mm[endpoint];
      cell.textContent = coords ? coords.map(value => value.toFixed(1)).join(" / ") : "—";
    }
  }
}

function angleStrip(side, values, errors = []) {
  const row = create("div", undefined, "angle-row");
  row.append(create("span", armName[side], "angle-side"));
  const strip = create("div", undefined, "angle-strip");
  values.forEach((angle, index) => {
    const cell = create("div", undefined, "angle");
    cell.append(create("span", "J" + (index + 1)),
                create("strong", angle == null ? "—" : angle.toFixed(2) + "°"));
    if (errors[index] != null) cell.title = "跟踪偏差 " + errors[index].toFixed(2) + "°";
    strip.append(cell);
  });
  row.append(strip);
  return row;
}

function showTarget(target, arms) {
  $("target-angles").replaceChildren(...arms.map(side => angleStrip(side, target.target_deg[side])));
  $("target-state").textContent = "预检通过";
  const joints = target.command_joints.map(label => label.replace(/^left/, "左臂").replace(/^right/, "右臂"));
  $("target-note").textContent = `${joints.join("、")} · ${target.duration_s} s · 峰值 ${target.peak_speed_deg_s.toFixed(2)} °/s · 60 秒内有效`;
}

function kindOf(level) {
  return level === "ERROR" ? "error" : level === "WARN" ? "warn" : "info";
}

const localTime = timestamp => new Date(timestamp).toLocaleTimeString("zh-CN", {hour12: false});
function eventTitle(row) {
  if (row.event === "motion.state") return states[row.message] || row.message;
  return labels[row.event] || row.event;
}

function renderFeedback() {
  const run = state.run;
  const sample = state.status ? run?.sample : null;
  $("tracking").hidden = !sample;
  if (sample) {
    const arms = sides.filter(side => sample.measured_deg[side].some(value => value != null));
    $("feedback-angles").replaceChildren(...arms.map(side => angleStrip(side, sample.measured_deg[side],
                                                                        sample.error_deg[side])));
    $("tracking-error").textContent = `t = ${sample.elapsed_s.toFixed(2)} s · 最大轴误差 ${sample.max_error_deg.toFixed(2)}°`;
  }
  const events = state.logs.filter(row => row.important);
  const latest = events.at(-1);
  const disconnected = !state.status && {title: "本机服务未连接", message: client ?
    "等待本机控制服务恢复。" : "运行同目录的「启动监视控制台.cmd」，再打开本机服务。", kind: "warn"};
  const current = disconnected || state.notice || (latest && {title: eventTitle(latest),
    message: latest.message, kind: kindOf(latest.level)}) ||
    {title: "等待反馈", message: "控制服务正在读取两臂状态。", kind: "info"};
  const summary = JSON.stringify(current);
  const area = $("feedback-current");
  if (area.dataset.summary !== summary) {
    const box = create("div", undefined, "feedback-box " + current.kind);
    const content = create("div");
    content.append(create("strong", current.title), create("p", current.message));
    box.append(content);
    area.replaceChildren(box);
    area.dataset.summary = summary;
  }
  $("feedback-count").textContent = events.length ? events.length + " 条事件" : "";
  $("feedback-history").replaceChildren(...events.slice(-9, -1).reverse().map(row => {
    const item = create("div", undefined, "history-item");
    item.append(create("time", localTime(row.timestamp)), create("span", row.message || eventTitle(row)));
    return item;
  }));
}

function renderLogs() {
  const query = $("search").value.trim().toLowerCase();
  const matching = state.logs.filter(row => (state.filter === "ALL" || row.level === state.filter) &&
    (!query || JSON.stringify(row).toLowerCase().includes(query) || eventTitle(row).includes(query)));
  $("log-body").replaceChildren(...matching.map(row => {
    const tr = create("tr");
    tr.append(create("td", localTime(row.timestamp)), create("td", row.level, "level-" + row.level));
    const detail = create("td");
    const description = [row.request_id, row.message].filter(Boolean).join("\n");
    detail.append(create("span", eventTitle(row), "log-event"),
                  create("span", description, "log-detail"));
    if (row.data) {
      const params = create("details", undefined, "log-params");
      params.append(create("summary", "数据"), create("pre", JSON.stringify(row.data, null, 2)));
      detail.append(params);
    }
    tr.append(detail);
    return tr;
  }));
  $("empty-logs").hidden = matching.length > 0;
  $("log-count").textContent = `${matching.length} / ${state.logs.length} 条事件`;
  if ($("autoscroll").checked) $("log-view").scrollTop = $("log-view").scrollHeight;
}

function renderRun() {
  const run = state.run;
  const id = state.target?.request_id || run?.request_id;
  $("request-label").textContent = id ? id.slice(0, 12) : "本机服务";
  $("request-label").title = id || "";
  const activeLabel = run?.stop_reason || run?.state === "STOP_REQUESTED" ? "正在停止" : states[run?.state];
  $("execution-state").textContent = !state.status ?
    (run?.active ? "连接中断 · 等待停止回报" : "等待本机服务") :
    state.status.pending_mode ? "正在切换并读回 " + state.status.pending_mode :
    run ? (run.active ? activeLabel : run.message || states[run.state]) :
      state.status.log_error ? "日志写入失败，运动已锁定" :
        state.status.can_switch_mode && !state.status.drive_mode ? "请选择电机模式，核对 14 轴回读" :
        state.status.can_prepare ? "就绪" : "等待双臂静止与失能回读";
  const progress = $("progress");
  progress.hidden = !state.status || !run?.active || !run?.sample;
  if (!progress.hidden) progress.value = Math.min(run.sample.elapsed_s / run.duration_s, 1);
}

async function api(path, payload, timeout = 1500) {
  const headers = {"X-OpenArm-Client": client};
  const options = {headers, signal: AbortSignal.timeout(timeout), cache: "no-store"};
  if (payload !== undefined) {
    options.method = "POST";
    headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(payload);
  }
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || "服务返回错误");
  return data;
}

async function refresh() {
  try {
    const requestedAfter = state.cursor;
    const previousRun = state.run;
    const status = await api("/api/status?after=" + requestedAfter);
    const restarted = state.instance !== status.instance;
    if (restarted) {
      state.instance = status.instance;
      state.logs = [];
      state.cursor = 0;
      state.run = null;
      invalidate();
    }
    if (state.target && status.revision > state.target.revision) invalidate("控制状态已变化，请重新预览。");
    if (status.events.length) {
      state.logs = state.logs.concat(status.events).slice(-400);
      state.cursor = status.events.at(-1).sequence;
      if (status.events.some(row => row.important)) state.notice = null;
      renderLogs();
    }
    state.status = status;
    if (restarted || state.run === previousRun || status.run?.request_id === state.run?.request_id) {
      state.run = status.run;
    }
    const online = sides.filter(side => status.arms[side].fresh && !status.arms[side].reason).length;
    setConnection(status.mode === "virtual" ? "virtual" : online ? "online" : "offline",
                  status.mode === "virtual" ? "离线仿真" : `实机 · ${online}/2 臂在线`);
    $("connection").title = status.log_error || status.mapping_version;
    $("export").setAttribute("aria-disabled", "false");
  } catch {
    state.status = null;
    setConnection("offline", "本机服务未连接");
    $("export").setAttribute("aria-disabled", "true");
  }
  renderSensors();
  updateControls();
  renderRun();
  renderFeedback();
  setTimeout(refresh, 220);
}

function actionError(title, error) {
  state.notice = {title, message: error.message || "请检查本机控制服务。", kind: "error"};
  renderFeedback();
}

function controlsChanged() {
  if ($("target-angles").childElementCount) invalidate("参数已变化，请重新预览。");
  updateControls();
}

function targetPayload() {
  const isIK = state.mode === "ik";
  const inputs = isIK ? ikInputs : jointInputs;
  const targets = Object.fromEntries(selectedArms().map(side => [side, inputs[side].map(input =>
    input.value === "" ? null : input.valueAsNumber)]));
  return {mode: state.mode, arm: state.arm, targets, endpoint: $("endpoint").value,
          duration_s: $("duration").valueAsNumber, speed_percent: $("speed").valueAsNumber};
}

buildFields();
$("drive-mode").addEventListener("change", async event => {
  const mode = event.target.value;
  state.pending = true;
  invalidate("模式切换后请重新预览。");
  updateControls();
  try {
    const result = await api("/api/drive-mode", {drive_mode: mode});
    if (state.status) Object.assign(state.status, result);
  } catch (error) {
    actionError("电机模式切换失败", error);
  } finally {
    state.pending = false;
    updateControls();
  }
});
$("enable-all").addEventListener("click", async () => {
  state.pending = true;
  invalidate("全部使能后可继续预览目标。");
  updateControls();
  try {
    state.run = await api("/api/enable-all", {}, 10000);
  } catch (error) {
    actionError("全部使能未受理", error);
  } finally {
    state.pending = false;
    updateControls();
    renderRun();
  }
});
for (const attribute of ["mode", "arm", "filter"]) {
  document.querySelectorAll(`[data-${attribute}]`).forEach(button => {
    button.addEventListener("click", () => {
      state[attribute] = button.dataset[attribute];
      choose(attribute, state[attribute]);
      if (attribute === "filter") renderLogs();
      else controlsChanged();
    });
  });
}
$("joint-metric").addEventListener("change", renderSensors);
$("control-form").addEventListener("input", controlsChanged);
$("control-form").addEventListener("invalid", event => {
  actionError("请检查输入", new Error(event.target.validationMessage));
}, true);
$("endpoint").addEventListener("change", () => {
  sides.forEach(side => ikInputs[side].forEach(input => { input.value = ""; }));
  controlsChanged();
});
$("reset").addEventListener("click", () => { $("control-form").reset(); controlsChanged(); });
$("use-current").addEventListener("click", () => {
  for (const side of selectedArms()) {
    const point = state.status?.arms[side].endpoints_mm[$("endpoint").value];
    if (point) ikInputs[side].forEach((input, index) => { input.value = point[index].toFixed(2); });
  }
  controlsChanged();
});
$("control-form").addEventListener("submit", async event => {
  event.preventDefault();
  if ($("prepare").disabled) return;
  state.notice = null;
  invalidate("正在计算目标与检查路径。");
  state.pending = true;
  updateControls();
  try {
    const arms = selectedArms();
    const target = await api("/api/prepare", targetPayload(), 10000);
    state.target = {...target, expiresAt: performance.now() + 1000 * target.expires_in_s};
    showTarget(target, arms);
  } catch (error) {
    invalidate("目标未通过预检。");
    actionError("目标未通过预检", error);
  } finally {
    state.pending = false;
    updateControls();
  }
});
$("execute").addEventListener("click", async () => {
  if ($("execute").disabled) return;
  const requestId = state.target.request_id;
  state.notice = null;
  state.pending = true;
  updateControls();
  try {
    state.run = await api("/api/execute", {request_id: requestId});
    // Retain the preview for reference while it runs. The ID has been consumed.
    state.target = null;
    $("target-state").textContent = "指令已提交";
    $("target-note").textContent = $("target-note").textContent.replace(" · 60 秒内有效", "");
  } catch (error) {
    invalidate("请重新生成预览。");
    actionError("执行未受理", error);
  } finally {
    state.pending = false;
    updateControls();
    renderRun();
  }
});
$("stop").addEventListener("click", async () => {
  if (!state.run?.active) return;
  state.stopping = true;
  updateControls();
  try {
    await api("/api/stop", {request_id: state.run.session_id || state.run.request_id});
  } catch (error) {
    actionError("停止请求未确认", error);
  } finally {
    state.stopping = false;
    updateControls();
  }
});
$("search").addEventListener("input", renderLogs);
$("autoscroll").addEventListener("change", renderLogs);
$("export").addEventListener("click", event => {
  if ($("export").getAttribute("aria-disabled") === "true") event.preventDefault();
});

const renderClock = () => { $("clock").textContent = localTime(Date.now()); };
renderClock();
setInterval(renderClock, 1000);
renderSensors();
updateControls();
renderFeedback();
if (client) refresh();
else $("open-service").hidden = false;
