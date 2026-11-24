"use strict";

const $ = id => document.getElementById(id);
const sides = ["left", "right"];
const armName = {left: "左臂", right: "右臂"};
const pianoFingers = {pinky: "小指", ring: "无名指", middle: "中指", index: "食指"};
let trialKeySet = "", trialInstance = "";
const whiteNote = key => ["A","B","C","D","E","F","G"][(key - 1) % 7] + Math.floor((key + 4) / 7);
const originSelected = () => $("trial-anchor").value === "3";
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
  "motion.zero_pose": "0位", "motion.start_return": "回0—起始位", "motion.start_pose": "起始位",
  "motion.piano_origin": "钢琴原位",
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
  renderHand();
  const status = state.status;
  const holding = state.run?.active && state.run.state === "HOLDING_TARGET" && status?.can_prepare;
  const locked = state.pending || (state.run?.active && !holding);
  const drive = $("drive-mode");
  drive.disabled = state.pending || !status?.can_switch_mode;
  drive.title = status?.mode === "hardware" && !status.drive_modes?.length ? "切换模式需要加载现场 C02 盘点" :
    state.run?.active ? "停止并确认失能后可切换模式" : "切换双臂全部 14 轴的电机模式";
  drive.value = status?.pending_mode || status?.drive_mode || "";
  const modes = Object.values(status?.axis_modes || {}).flat();
  drive.options[0].textContent = status?.mode_error ? "切换失败，请重新确认" :
    modes.length === 14 && modes.every(Number.isInteger) && new Set(modes).size > 1 ? "双臂模式不一致" : "等待模式回读";
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
  $("piano-air-preview").disabled = !status?.piano_air_available || !status.can_prepare || locked;
  $("piano-air-preview").title = status && !status.piano_air_available ? "请在实机空闲后加载新版控制台服务" : "";
  $("piano-air-position-score").disabled = $("piano-air-preview").disabled || !status?.piano_score_position_available;
  $("piano-air-position-score").title = status && !status.piano_score_position_available ?
    "请在实机空闲后加载新版控制台服务" : "";
  const score = $("piano-air-score");
  score.disabled = state.pending;
  const availableScores = status?.piano_air_scores ?? ["jin"];
  for (const option of score.options) option.disabled = !availableScores.includes(option.value);
  $("piano-air-scope").disabled = state.pending;
  $("piano-air-speed").disabled = state.pending;
  const trial = state.target?.piano_trial;
  const needsHand = trial && trial.scope !== "start";
  const rightHand = status?.hands?.right;
  const runningTrial = state.run?.piano_trial;
  const fingers = status?.hand_score;
  $("trial-anchor").options[0].disabled = !status?.piano_origin;
  if (status && trialInstance !== status.instance) {
    trialInstance = status.instance;
    $("trial-anchor").value = status.piano_origin ? "3" : "1";
  }
  const atOrigin = originSelected();
  $("trial-side-label").hidden = !atOrigin;
  $("trial-speed-label").hidden = atOrigin;
  $("trial-score-label").hidden = !atOrigin;
  const fingeringScores = status?.piano_fingering_scores ?? ["jin-chords"];
  for (const option of $("trial-score").options) option.disabled = !fingeringScores.includes(option.value);
  const selection = atOrigin ? status?.piano_origin?.[$("trial-side").value] : null;
  const keys = selection?.keys ?? Array.from({length:18}, (_, i) => i + 21);
  const signature = [atOrigin, $("trial-side").value, ...keys].join("/");
  if (signature !== trialKeySet) {
    trialKeySet = signature;
    $("trial-key").replaceChildren(...keys.map(k => new Option(
      `${whiteNote(k)} · 实琴 ${k - 9} / 谱 ${k}`, String(k))));
    $("trial-key").value = String(selection?.anchor ?? 24);
  }
  $("trial-position").disabled = $("trial-preview").disabled = !status?.piano_trial_available || !status.can_prepare || locked;
  if (atOrigin) $("trial-preview").disabled = !status?.piano_origin || state.pending;
  $("trial-preview").textContent = atOrigin ? "预览双手指法" : "预览右手试弹";
  for (const id of ["trial-anchor", "trial-key", "trial-side", "trial-score", "trial-scope", "trial-speed"]) $(id).disabled = state.pending;
  let trialStatus = (rightHand?.connected ? `右手${rightHand.demo ? "模拟" : rightHand.port}在线` :
    "未连接右手：仅执行机械臂，指法不发送") + " · C4 点位参考，实琴 15 / 曲谱 24";
  if (atOrigin) trialStatus = "实琴 C2–C7，白键距 22.5 mm。食指原位相邻，左 B3（14），右 C4（15）。两手从原位向外定位。";
  if (state.run?.active && runningTrial) {
    const group = `第 ${runningTrial.anchor_run} 组`;
    if (runningTrial.scope === "start") {
      const note = runningTrial.note ?? whiteNote(runningTrial.target_key);
      const physicalKey = runningTrial.physical_key ?? runningTrial.target_key - 9;
      const where = `${armName[runningTrial.side ?? "right"]} ${note} · 实琴白键 ${physicalKey}`;
      trialStatus = `${group} · ${where} · ${state.run.state === "HOLDING_TARGET" ? "定位保持中" : "腕部定位中"}`;
    } else if (fingers?.id === state.run.request_id) {
      trialStatus = fingers.active ? `${group} · 右手试弹中 · ${Math.floor(fingers.elapsed_s)} / ${Math.ceil(fingers.duration_s)} 秒` :
        `${group} · 右手 · ${fingers.message}`;
    }
  }
  $("trial-status").textContent = status?.piano_trial_available ? trialStatus : "等待加载支持实机点位的控制台服务";
  $("use-current").disabled = !selected.every(side => status?.arms[side].endpoints_mm[$("endpoint").value]);
  $("use-current-joints").disabled = ik || !selected.every(side =>
    status?.arms[side].joints?.every(row => row && Number.isFinite(row.position_deg)));
  for (const side of sides) {
    document.querySelector(`[data-ik-arm="${side}"]`).hidden = !selected.includes(side);
    ikInputs[side].forEach(input => { input.disabled = !ik || !selected.includes(side); });
    jointInputs[side].forEach(input => { input.disabled = ik || !selected.includes(side); });
  }
  $("execute").textContent = trial ? (needsHand ? "开始右手试弹" : `执行 ${whiteNote(trial.target_key)} 定位`) : state.target?.piano_air ? "开始空弹" :
    status?.mode === "hardware" ? "执行实机目标" : "执行离线目标";
  if (trial?.side) $("execute").textContent = `执行${armName[trial.side]} ${trial.note} 定位`;
  $("execute").disabled = !status?.can_prepare || locked || !state.target;
  const startReady = Boolean(status?.can_start_pose) && !state.pending && !state.stopping;
  for (const id of ["start-zero", "start-return", "start-pose"]) $(id).disabled = !startReady;
  $("start-original").disabled = !startReady || !status?.piano_origin;
  const startReason = !status ? "等待本机服务" :
    !status.motion_supported ? "等待 14 轴电机模式就绪" :
    status.run?.active && status.run.state !== "HOLDING_TARGET" ? "机械臂正在运动，请等待到位" :
    "等待 14 轴反馈就绪";
  $("start-zero").title = `双臂直接回到 J7 +15°、其余 0°。${startReady ? "当前可用。" : startReason + "。"}`;
  $("start-return").title = `双臂：0位（J7+15°）→ J1+15°/J2-45° → J1-30°/J4+90° → J2回0/J4+120°。${startReady ? "当前可用。" : startReason + "。"}`;
  $("start-pose").title = `双臂直接回到起始位 J1-30°/J2 0°/J4+120°/J7+15°。${startReady ? "当前可用。" : startReason + "。"}`;
  $("start-original").title = `实琴 C2–C7。左食指 B3（白键 14），右食指 C4（白键 15），3 秒定位。${startReady ? "" : startReason}`;
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
  const air = target.piano_air;
  if (target.piano_trial) {
    const trial = target.piano_trial;
    if (trial.side) {
      $("target-note").textContent = `${armName[trial.side]}食指 ${trial.note} · 实琴白键 ${trial.physical_key} / 曲谱白键 ${trial.target_key} · ${target.duration_s} 秒`;
      return;
    }
    const startNote = whiteNote(trial.start_key);
    const approach = trial.held_key ? `从已保持的 ${whiteNote(trial.held_key)} 接续` : `粗定位 → ${startNote}`;
    $("target-note").textContent = `第 ${trial.anchor_run} 组 · ${approach} · 食指腕位 ${trial.wrist_keys.map(whiteNote).join("、")}` +
      (trial.scope === "start" ? "" : ` · 右手 ${trial.note_count} 音 · ${trial.speed}×`) +
      ` · ${target.duration_s.toFixed(1)} 秒`;
    return;
  }
  $("target-note").textContent = air
    ? `${air.calibrated ? "现场标定" : "CSV 仿真坐标"}${air.scope === "start" ? "" : ` · ${air.speed}× · ${air.note_count} 音`} · ${target.duration_s.toFixed(1)} s · 峰值 ${target.peak_speed_deg_s.toFixed(2)} °/s${air.from_score_start ? " · 从保持的起位接续" : ""}`
    : `${joints.join("、")} · ${target.duration_s} s · 峰值 ${target.peak_speed_deg_s.toFixed(2)} °/s`;
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
    state.status.mode_error ? state.status.mode_error :
    run ? (run.active ? activeLabel : run.message || states[run.state]) :
      state.status.log_error ? "日志写入失败，运动已锁定" :
        state.status.can_switch_mode && !state.status.drive_mode ? "请选择电机模式，核对 14 轴回读" :
        state.status.can_prepare ? "就绪" : "等待双臂静止与失能回读";
  const progress = $("progress");
  progress.hidden = !state.status || !run?.active || !run?.sample;
  if (!progress.hidden) progress.value = Math.min(run.sample.elapsed_s / run.duration_s, 1);
  const air = run?.active && run.sample?.piano_air;
  const notes = air?.notes ?? (air?.note ? [air.note] : []);
  let pianoStatus = state.status && !state.status.piano_score_position_available
    ? (run?.active ? "旧版服务仍在执行或保持，结束后可加载「琴谱起位」" : "「琴谱起位」需要加载新版控制台服务")
    : "";
  if (air?.scope === "start" && air.anchors) {
    const {left, right} = air.anchors;
    const phase = run.state === "HOLDING_TARGET" ? "起位保持中" : "正在前往琴谱起位";
    pianoStatus = `${phase} · 左小指 ${left.pinky}／食指 ${left.index} · 右食指 ${right.index}`;
  } else if (notes.length) {
    const targets = notes.map(note => `${armName[note.side]}${pianoFingers[note.finger]} ${note.key}`);
    pianoStatus = `${air.score_elapsed_s.toFixed(2)} s · 曲谱白键 ${targets.join("、")}`;
  } else if (air && !air.positioning && air.scope !== "start") {
    pianoStatus = run.state === "HOLDING_TARGET" ? "空弹结束 · 保持最后手位" : `${air.score_elapsed_s.toFixed(2)} s · 休止`;
  } else if (air?.positioning) {
    pianoStatus = "正在前往琴谱起位";
  }
  $("piano-air-position").textContent = pianoStatus;
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

const handActions = ["move", "home", "press", "save_home", "save_press"];
const handNames = ["拇指外展", "拇指屈曲", "拇指腱", "食指", "中指", "无名指", "小指"];
let handBusy = false, handView = "", handMessage = "";
let scorePending = false, scoreError = "";

function renderHand() {
  const side = $("hand-side").value;
  const status = state.status;
  const row = status?.hands?.[side];
  const key = [status?.instance, side, row?.session].join("/");
  if (row && key !== handView) {
    handView = key;
    $("hand-mode").value = row.demo ? "demo" : "real";
    $("hand-port").value = row.port;
  }
  const demo = $("hand-mode").value === "demo";
  const connected = Boolean(row?.connected);
  const score = status?.hand_score;
  const handsReady = sides.every(s => status?.hands?.[s]?.connected && !status.hands[s].fault);
  const ready = connected && !row.fault && !handBusy && !score?.active;
  $("hand-score-start").disabled = !score || !handsReady || handBusy || scorePending || score.active;
  $("hand-score-stop").disabled = !score?.active || scorePending;
  $("hand-score-status").textContent = scoreError || (score?.active ?
    `排练中 · ${Math.floor(score.elapsed_s)} / ${score.duration_s} 秒` :
    [score?.message, !handsReady && "请分别连接左手和右手，可选择模拟"].filter(Boolean).join(" · "));
  const ch = $("hand-finger").value;
  const cal = row?.calibration?.[ch] || {};
  const hasValue = value => value != null && value !== "";
  $("hand-mode").disabled = !row || connected || handBusy || status.mode === "virtual";
  $("hand-port").disabled = !row || connected || handBusy || demo;
  $("hand-connect").disabled = !row || connected || handBusy || (!demo && !$("hand-port").value.trim());
  $("hand-disconnect").disabled = !connected;
  $("hand-side").disabled = $("hand-finger").disabled = handBusy;
  for (const action of handActions) {
    $("hand-" + action).disabled = !ready ||
      (["press", "home"].includes(action) && !hasValue(cal[action])) ||
      (action === "move" && (!$("hand-degrees").value || !$("hand-degrees").validity.valid));
  }
  $("hand-summary").textContent = !status ? "等待本机服务" : sides.map(s => {
    const h = status.hands?.[s];
    return (s === "left" ? "左" : "右") + " · " +
      (h?.fault ? "连接故障" : h?.connected ? (h.demo ? "模拟在线" : h.port) : "未连接");
  }).join("　");
  const display = value => hasValue(value) ? Number(value).toFixed(1) + "°" : "未记录";
  const q = row?.feedback?.q?.[ch];
  const temperatures = row?.feedback?.temperatures;
  const zeroTemperatures = connected && !row.demo && temperatures?.length === 7 &&
    temperatures.every(value => value === 0);
  $("hand-result").textContent = [row?.fault || handMessage, connected ?
    `当前 ${display(q)}　抬起位 ${display(cal.home)}　按下位 ${display(cal.press)}` : "",
    zeroTemperatures && "串口已回包，七路舵机温度均为 0°C。请检查手部独立供电和舵机总线。"
  ].filter(Boolean).join(" · ");
  $("hand-time").textContent = row?.feedback ? new Date(row.received_at * 1000).toLocaleTimeString() : "";
  if (!$("hand-panel").open) return;
  $("hand-feedback").replaceChildren(...handNames.map((name, i) => {
    const tr = create("tr");
    tr.append(create("td", name), ...["q", "actuations", "currents", "temperatures", "speeds"].map(field => {
      const value = row?.feedback?.[field]?.[i];
      return create("td", Number.isFinite(value) ? value.toFixed(1) : "—");
    }));
    return tr;
  }));
}

for (const id of ["side", "finger", "mode", "degrees", "port"]) {
  $("hand-" + id).addEventListener("input", () => {
    handMessage = "";
    renderHand();
  });
}
$("hand-panel").addEventListener("toggle", renderHand);
for (const action of ["start", "stop"]) {
  $("hand-score-" + action).addEventListener("click", async () => {
    scorePending = true;
    scoreError = "";
    renderHand();
    try {
      const payload = action === "stop" ? {request_id: state.status.hand_score.id} :
        {sessions: Object.fromEntries(sides.map(s => [s, state.status.hands[s].session]))};
      const result = await api("/api/hand-score/" + action, payload);
      if (state.status) state.status.hand_score = result;
    } catch (error) {
      scoreError = error.message;
    } finally {
      scorePending = false;
      renderHand();
    }
  });
}
for (const action of ["connect", "disconnect", ...handActions]) {
  $("hand-" + action).addEventListener("click", async () => {
    const side = $("hand-side").value;
    const payload = {side, action, session: state.status?.hands?.[side]?.session,
      channel: Number($("hand-finger").value), degrees: Number($("hand-degrees").value),
      demo: $("hand-mode").value === "demo", port: $("hand-port").value.trim()};
    handBusy = true;
    handMessage = "正在处理…";
    renderHand();
    try {
      const result = await api("/api/hand", payload, 10000);
      if (state.status?.hands) state.status.hands[side] = result;
      handMessage = {connect: "已读取七路位置", disconnect: "已停止发送并断开，舵机可保持末目标",
        save_home: "抬起位已保存", save_press: "按下位已保存"}[action] || "位置目标已写入串口";
    } catch (error) {
      handMessage = error.message;
    } finally {
      handBusy = false;
      renderHand();
    }
  });
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
  const original = state.status?.axis_modes;
  if (state.status?.mode === "hardware") {
    const names = ["", "MIT", "POS_VEL", "VEL", "POS_FORCE"];
    const rows = sides.map(s => (s === "left" ? "左臂" : "右臂") + " J1–J7：" +
      original[s].map(code => names[code] || String(code)).join(" / "));
    if (!window.confirm([`将双臂 14 轴切换到 ${mode}？`, ...rows,
        "失败时按逐轴原值回退。本次仅切换运行模式，保持失能，不保存 Flash。"].join("\n"))) {
      updateControls();
      return;
    }
  }
  state.pending = true;
  invalidate("模式切换后请重新预览。");
  updateControls();
  try {
    const result = await api("/api/drive-mode", {drive_mode: mode, original_modes: original});
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
$("use-current-joints").addEventListener("click", () => {
  for (const side of selectedArms()) {
    const joints = state.status?.arms[side].joints || [];
    jointInputs[side].forEach((input, index) => {
      const row = joints[index];
      input.value = row ? row.position_deg.toFixed(2) : "";
    });
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
    state.target = target;
    showTarget(target, arms);
  } catch (error) {
    invalidate("目标未通过预检。");
    actionError("目标未通过预检", error);
  } finally {
    state.pending = false;
    updateControls();
  }
});
for (const id of ["piano-air-score", "piano-air-scope", "piano-air-speed"]) {
  $(id).addEventListener("input", () => {
    if (state.target?.piano_air) invalidate("空弹设置已变化，请重新预览。");
  });
}
async function submitPianoAir(positionOnly) {
  const scope = positionOnly ? "start" : $("piano-air-scope").value;
  const input = $("piano-air-speed");
  if (!positionOnly && !input.reportValidity()) return;
  state.notice = null;
  invalidate(positionOnly ? "正在读取琴谱起位。" : "正在计算双臂腕位与换位。");
  state.pending = true;
  updateControls();
  try {
    const target = await api("/api/prepare", {mode: "piano-air", scope,
      score_id: $("piano-air-score").value, speed: positionOnly ? .35 : input.valueAsNumber}, 15000);
    showTarget(target, sides);
    if (positionOnly) {
      state.run = await api("/api/execute", {request_id: target.request_id});
      $("target-state").textContent = "琴谱起位已提交";
    } else {
      state.target = target;
    }
  } catch (error) {
    invalidate("请重新生成空弹预览。");
    actionError(positionOnly ? "琴谱起位未受理" : "空弹预览未就绪", error);
  } finally {
    state.pending = false;
    updateControls();
    renderRun();
  }
}
$("piano-air-position-score").addEventListener("click", () => submitPianoAir(true));
$("piano-air-preview").addEventListener("click", () => submitPianoAir(false));
for (const id of ["trial-anchor", "trial-side", "trial-key", "trial-score", "trial-scope", "trial-speed"]) {
  $(id).addEventListener("input", () => {
    if (state.target?.piano_trial) invalidate("试弹设置已变化，请重新预览。");
    $("trial-fingering").hidden = true;
    updateControls();
  });
}
async function previewTrial(positionOnly) {
  const speed = $("trial-speed");
  const diagram = originSelected() && !positionOnly;
  if (!positionOnly && !diagram && !speed.reportValidity()) return;
  state.notice = null;
  invalidate("正在读取钢琴点位与曲谱。");
  state.pending = true;
  updateControls();
  try {
    if (diagram) {
      const result = await api("/api/piano/fingering", {
        scope: $("trial-scope").value, score_id: $("trial-score").value,
      }, 15000);
      showFingering(result);
      $("target-state").textContent = "双手分指预览";
      return;
    }
    const target = await api("/api/prepare", {mode: "piano-trial", anchor_run: Number($("trial-anchor").value),
      side: $("trial-side").value,
      key: Number($("trial-key").value), scope: positionOnly ? "start" : $("trial-scope").value,
      speed: positionOnly ? .35 : speed.valueAsNumber}, 15000);
    state.target = target;
    showTarget(target, [target.piano_trial.side ?? "right"]);
  } catch (error) {
    actionError("试弹预览未就绪", error);
  } finally {
    state.pending = false;
    updateControls();
    renderRun();
  }
}
function showFingering(result) {
  const fullScore = result.score_id === "jin-full";
  $("fingering-rows").replaceChildren(...result.rows.map((row, index) => {
    const tr = create("tr");
    tr.append(create("td", (row.time_us/1e6).toFixed(3)), create("td", row.notes.map(n => whiteNote(n.key)).join(" ")));
    if (row.fingers) {
      for (const side of sides) {
        const key = row.wrists[side];
        const fingers = row.fingers[side].map(n => (pianoFingers[n.finger] ?? "用指待标定") + " " + whiteNote(n.key));
        const cell = create("td", fingers.join("、") || (key == null ? "休止" : "延音"));
        if (key != null) {
          const pending = row.pending_wrists?.includes(side) ? " · 腕位待标定" : "";
          cell.append(create("div", `食指对位 ${whiteNote(key)} · 实琴 ${key - 9}${pending}`, "muted"));
        } else if (row.fingers[side].length) {
          cell.append(create("div", fullScore ? "当前指距无共同腕位" : "腕位待标定", "muted"));
        }
        const target = index === 0 && result.first_wrist_targets?.[side];
        if (target) {
          cell.append(create("div", `XYZ（mm） ${target.xyz_mm.map(mm => mm.toFixed(1)).join("，")}`, "small muted"));
          const joints = create("div", undefined, "fingering-wrist-joints small muted");
          joints.append(...target.joints_deg.map((deg, j) => create("span", `J${j + 1} ${deg.toFixed(2)}°`)));
          cell.append(joints);
        }
        tr.append(cell);
      }
    } else {
      const cell = create("td", "未找到完整分指");
      cell.colSpan = 2;
      tr.append(cell);
    }
    tr.append(create("td", row.wrist_distance_mm == null ? "—" : row.wrist_distance_mm.toFixed(2)+" mm"));
    return tr;
  }));
  const firstDistance = result.rows[0]?.wrist_distance_mm;
  $("fingering-summary").textContent = `原谱 ${result.rows.length} 组、${result.note_count} 音，${result.unassigned} 组待补指法。` +
    (fullScore || result.pending_wrist_groups == null ? "" : `${result.pending_wrist_groups} 组腕位待标定。`) +
    (firstDistance == null ? "" : `首和弦两腕相距 ${firstDistance.toFixed(2)} mm。`);
  $("trial-fingering").hidden = false;
}
$("trial-position").addEventListener("click", () => previewTrial(true));
$("trial-preview").addEventListener("click", () => previewTrial(false));
$("execute").addEventListener("click", async () => {
  if ($("execute").disabled) return;
  const requestId = state.target.request_id;
  state.notice = null;
  state.pending = true;
  updateControls();
  try {
    const request = {request_id: requestId};
    if (state.target?.piano_trial) request.sessions = {right: state.status?.hands?.right?.session};
    state.run = await api("/api/execute", request);
    // Retain the preview for reference while it runs. The ID has been consumed.
    state.target = null;
    $("target-state").textContent = "指令已提交";
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
for (const [id, path, label] of [
  ["start-zero", "/api/start/zero", "0位"],
  ["start-return", "/api/start/return", "回0—起始位"],
  ["start-pose", "/api/start/pose", "起始位"],
  ["start-original", "/api/start/piano-origin", "钢琴原位"],
]) {
  $(id).addEventListener("click", async () => {
    if ($(id).disabled) return;
    state.notice = null;
    state.pending = true;
    invalidate(`正在执行${label}…`);
    updateControls();
    try {
      state.run = await api(path, {}, 10000);
    } catch (error) {
      actionError(`${label}未受理`, error);
    } finally {
      state.pending = false;
      updateControls();
      renderRun();
    }
  });
}
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
