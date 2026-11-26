"use strict";

const $ = id => document.getElementById(id);
const sides = ["left", "right"];
const armName = {left: "左臂", right: "右臂"};
const handName = {left: "左手", right: "右手"};
const pianoFingers = {pinky: "小指", ring: "无名指", middle: "中指", index: "食指"};
let trialKeySet = "";
const whiteNote = key => ["A","B","C","D","E","F","G"][(key - 1) % 7] + Math.floor((key + 4) / 7);
const endpoints = {wrist: "手腕", thumb: "拇指", index: "食指", middle: "中指", ring: "无名指", little: "小指"};
const wristAxes = ["Roll", "Pitch", "Yaw"];
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
  "motion.piano_origin": "钢琴原位", "motion.zero_return": "逆方向返回",
  "motion.piano_exit": "退出演奏并保持",
  "motion.after": "完成失能回读", "motion.disable_evidence": "失能记录",
};
const state = {
  mode: "joint", arm: "left", status: null, run: null, target: null, pending: false,
  stopping: false, exitingPerformance: false, instance: null, cursor: 0, logs: [], filter: "ALL", notice: null,
};
const client = location.protocol === "file:" ? "" : crypto.randomUUID().replaceAll("-", "");
const jointInputs = {};
const ikInputs = {};
const wristInputs = {};
const jointCells = {};
const endpointCells = {};
const wristCells = {};
const fingerNoteCells = document.querySelectorAll("[data-hand-note]");

function create(tag, text, className) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
}

function selectedArms() {
  return state.arm === "both" ? sides : [state.arm];
}

function buildIkInputs(container, side, axes, suffix) {
  const group = create("div", undefined, "ik-group");
  group.dataset.ikArm = side;
  group.append(create("strong", armName[side]));
  const fields = create("div", undefined, "xyz");
  const inputs = axes.map(axis => {
    const label = create("label", axis);
    const unit = create("div", undefined, "input-unit");
    const input = create("input");
    input.type = "number";
    input.step = "any";
    input.setAttribute("aria-label", armName[side] + " " + axis);
    unit.append(input, create("span", suffix));
    label.append(unit);
    fields.append(label);
    return input;
  });
  group.append(fields);
  $(container).append(group);
  return inputs;
}

function buildFields() {
  for (const side of sides) {
    jointInputs[side] = [];
    jointCells[side] = [];
    endpointCells[side] = {};
    ikInputs[side] = buildIkInputs("ik-inputs", side, ["X", "Y", "Z"], "mm");
    wristInputs[side] = buildIkInputs("wrist-inputs", side, wristAxes, "°");
    wristCells[side] = wristAxes.map(() => create("td", "—"));
    const row = create("tr");
    const header = create("th", armName[side]);
    header.scope = "row";
    row.append(header, ...wristCells[side]);
    $("wrist-pose-status").append(row);
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

function wristPoseEnabled() {
  return $("endpoint").value === "wrist" && Object.hasOwn(state.status?.arms.left ?? {}, "wrist_rpy_deg");
}

function hasWristTarget(side) {
  return wristPoseEnabled() && wristInputs[side].some(input => input.value !== "");
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
  renderPerformance(locked);
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
  const wristPose = ik && wristPoseEnabled();
  const selected = selectedArms();
  $("control-fields").disabled = !status || locked;
  document.querySelectorAll("[data-mode]").forEach(button => { button.disabled = locked; });
  $("ik-fields").hidden = !ik;
  $("wrist-fields").hidden = !wristPose;
  $("ik-position-note").hidden = wristPose;
  $("joint-fields").hidden = ik;
  $("prepare").textContent = ik ? "求解与预检" : "预览关节目标";
  $("prepare").disabled = !status?.can_prepare || locked;
  const trial = state.target?.piano_trial;
  const runningTrial = state.run?.piano_trial;
  const fingers = status?.hand_score;
  const pianoOrigin = status?.piano_origin;
  const originLabel = pianoOrigin ? sides.map(side => {
    const point = pianoOrigin[side];
    return armName[side] + "食指 " + point.note + "（白键 " + point.physical_key + "）";
  }).join("，") : "";
  syncScoreOptions($("trial-score"), status?.piano_performance_scores ?? {});
  const selection = pianoOrigin?.[$("trial-side").value];
  const keys = selection?.keys ?? [];
  const signature = [$("trial-side").value, selection?.anchor, ...keys].join("/");
  if (signature !== trialKeySet) {
    trialKeySet = signature;
    $("trial-key").replaceChildren(...keys.map(k => new Option(
      whiteNote(k) + " · 实琴 " + (k - 9) + " / 谱 " + k, String(k))));
    if (selection) $("trial-key").value = String(selection.anchor);
  }
  $("trial-position").disabled = !selection || !status?.can_prepare || locked;
  $("trial-preview").disabled = !pianoOrigin || !$("trial-score").value || state.pending;
  $("trial-dual-preview").disabled = $("trial-preview").disabled || !status?.can_prepare || locked;
  for (const id of ["trial-key", "trial-side", "trial-score", "trial-scope", "trial-hand", "trial-speed"]) $(id).disabled = state.pending;
  let trialStatus = pianoOrigin ? "实琴 C2–C7 · 白键距 22.5 mm · 原位 " + originLabel : "等待加载实琴点表";
  if (state.run?.active && runningTrial && !runningTrial.performance) {
    if (runningTrial.scope === "start") {
      const where = armName[runningTrial.side] + " " + runningTrial.note + " · 实琴白键 " + runningTrial.physical_key;
      trialStatus = where + " · " + (state.run.state === "HOLDING_TARGET" ? "定位保持中" : "腕部定位中");
    } else if (runningTrial.finger_notice) {
      trialStatus = "指法未启动：" + runningTrial.finger_notice + " · 执行腕位轨迹";
    } else if (fingers?.id === state.run.request_id) {
      const label = runningTrial.dual ? "双手试弹" : "右手试弹";
      const phase = fingers.elapsed_s < runningTrial.prepare_s ? "首拍就位" : "试弹中";
      trialStatus = label + " · " + (fingers.active ?
        phase + " · " + Math.floor(fingers.elapsed_s) + " / " + Math.ceil(fingers.duration_s) + " 秒" : fingers.message);
    }
    if (runningTrial.skip_notice) trialStatus += " · " + runningTrial.skip_notice;
  }
  $("trial-status").textContent = trialStatus;
  $("use-current").disabled = !selected.every(side => status?.arms[side].endpoints_mm[$("endpoint").value]);
  $("use-current-wrist").disabled = !wristPose || !selected.every(side => status.arms[side].wrist_rpy_deg);
  $("use-current-joints").disabled = ik || !selected.every(side =>
    status?.arms[side].joints?.every(row => row && Number.isFinite(row.position_deg)));
  for (const side of sides) {
    const selectedSide = selected.includes(side);
    document.querySelectorAll(`[data-ik-arm="${side}"]`).forEach(group => { group.hidden = !selectedSide; });
    const positionRequired = !hasWristTarget(side);
    ikInputs[side].forEach(input => {
      input.disabled = !ik || !selectedSide;
      input.required = positionRequired;
    });
    wristInputs[side].forEach(input => { input.disabled = !wristPose || !selectedSide; });
    jointInputs[side].forEach(input => { input.disabled = ik || !selectedSide; });
  }
  $("execute").textContent = status?.mode === "hardware" ? "执行实机目标" : "执行离线目标";
  if (trial?.scope === "start") $("execute").textContent = "执行" + armName[trial.side] + " " + trial.note + " 定位";
  else if (trial) $("execute").textContent = trial.dual ? "开始双手试弹" : "开始右手试弹";
  $("execute").disabled = !status?.can_prepare || locked || !state.target;
  const startReady = Boolean(status?.can_start_pose) && !state.pending && !state.stopping;
  for (const id of ["start-zero", "start-return", "start-pose", "start-zero-return"]) $(id).disabled = !startReady;
  $("start-original").disabled = !startReady || !status?.piano_origin;
  const startReason = !status ? "等待本机服务" :
    !status.motion_supported ? "等待 14 轴电机模式就绪" :
    status.run?.active && status.run.state !== "HOLDING_TARGET" ? "机械臂正在运动，请等待到位" :
    "等待 14 轴反馈就绪";
  $("start-zero").title = `双臂直接回到 J7 +15°、其余 0°。${startReady ? "当前可用。" : startReason + "。"}`;
  $("start-return").title = `双臂：0位（J7+15°）→ J1+15°/J2-45° → J1-30°/J4+90° → J2回0/J4+120°。${startReady ? "当前可用。" : startReason + "。"}`;
  $("start-pose").title = `双臂直接回到起始位 J1-30°/J2 0°/J4+120°/J7+15°。${startReady ? "当前可用。" : startReason + "。"}`;
  $("start-original").title = pianoOrigin ? originLabel + "，3 秒定位。" + (startReady ? "" : startReason) : "等待加载实琴点表";
  $("start-zero-return").title = `逆向执行回0—起始位：当前位置 → 起始位 → J1-30°/J2-45°/J4+90° → J1+15°/J2-45° → 0位，到位后保持使能。${startReady ? "当前可用。" : startReason + "。"}`;
  $("stop").disabled = !state.run?.active || state.stopping || Boolean(state.run?.stop_reason);
}

function syncScoreOptions(select, scores) {
  const entries = Object.entries(scores);
  if (entries.length !== select.options.length || entries.some(([id, score], index) =>
    select.options[index].value !== id || select.options[index].text !== score.title)) {
    const selected = select.value;
    select.replaceChildren(...entries.map(([id, score]) => new Option(score.title, id)));
    if (Object.hasOwn(scores, selected)) select.value = selected;
    else if (select.id === "trial-score" && Object.hasOwn(scores, "jin-chords")) select.value = "jin-chords";
  }
}

const calibChannels = ["3", "4", "5", "6"];

// Fingering readiness beside 弹奏. Paints a hint only; the service still decides
// whether fingers run, and nothing here disables or rejects the button.
function fingerReadiness(choice, status, run) {
  const trial = run?.piano_trial;
  if (run?.active && trial?.performance && trial.finger_notice) {
    return {level: "bad", text: `指法未启动：${trial.finger_notice} · 本次只跑腕位轨迹`};
  }
  const required = choice?.sides?.length ? choice.sides : sides;
  const blocked = [], cautious = [];
  for (const side of required) {
    const name = handName[side];
    const hand = status?.hands?.[side];
    if (!hand) { blocked.push(name + "状态未知"); continue; }
    if (!hand.connected) blocked.push(name + "未连接");
    else if (hand.fault) blocked.push(name + "反馈异常，需断开重连");
    else if (hand.demo && status.mode === "hardware") cautious.push(name + "使用模拟手，不会驱动真手");
    else {
      const missing = calibChannels.filter(ch => {
        const cal = hand.calibration?.[ch];
        return [cal?.home, cal?.press].some(value => {
          if (value == null || String(value).trim() === "") return true;
          const degrees = Number(value);
          return !Number.isFinite(degrees) || degrees < 0 || degrees > 90;
        });
      });
      if (missing.length) cautious.push(`${name} ${missing.join("/")} 号指的按抬位需核对`);
    }
  }
  if (blocked.length) return {level: "bad", text: "指法会跳过：" + blocked.join(" · ")};
  if (cautious.length) return {level: "warn", text: "指法待确认：" + cautious.join(" · ")};
  const detail = required.map(side => {
    const hand = status.hands[side];
    return `${handName[side]} ${hand.demo ? "模拟" : hand.port || "已连接"}`;
  }).join(" · ");
  return {level: "ok", text: "指法就绪 · " + detail};
}

function renderFingerHint(choice, status, run) {
  const box = $("performance-fingers");
  if (!box) return;
  const ready = fingerReadiness(choice, status, run);
  if (box.textContent !== ready.text) box.textContent = ready.text;
  if (box.dataset.state !== ready.level) box.dataset.state = ready.level;
}

function renderPerformance(locked) {
  const status = state.status;
  const scores = status?.piano_performance_scores;
  $("piano-performance").hidden = !scores;
  if (!scores) return;
  const select = $("performance-score");
  syncScoreOptions(select, scores);
  const choice = scores[select.value];
  const run = state.run;
  const trial = run?.piano_trial;
  const playing = run?.active && trial?.performance;
  const fingers = status.hand_score?.id === run?.request_id ? status.hand_score : null;
  select.disabled = state.pending;
  $("performance-play").disabled = !choice?.available || !status.piano_origin || !status.can_prepare || locked ||
    state.exitingPerformance || Boolean(run?.piano_exiting || (run?.piano_exited && fingers?.active));
  $("performance-exit").disabled = !playing || Boolean(run?.piano_exiting || run?.piano_exited || run?.stop_reason) ||
    state.exitingPerformance || state.stopping;
  const count = sides.flatMap(side => status.arms[side].fresh ? status.arms[side].joints : []).filter(Boolean).length;
  const hands = sides.map(side => {
    const hand = status.hands[side];
    const connection = hand.connected ? (hand.demo ? "模拟" : hand.port) : "未连接";
    return `${handName[side]} ${connection}${hand.fault ? " · 反馈异常" : ""}`;
  });
  $("performance-links").textContent = `臂反馈 ${count}/14 · ${hands.join(" · ")}`;
  renderFingerHint(choice, status, run);

  let message = choice?.notice || (choice ?
    `${choice.title} · 定位至首个手位 · 入位后等待 ${choice.wait_s ?? 2} 秒 · ${choice.speed ?? 1}×` : "");
  if (playing) {
    const elapsed = run.sample?.elapsed_s || 0;
    let phase = "演奏中";
    if (elapsed < trial.prepare_s - trial.wait_s) phase = "正在定位首个手位";
    else if (elapsed < trial.prepare_s) phase = "保持首腕位目标 · 等待起奏";
    const sync = run.sample?.piano_sync;
    if (sync?.waiting) {
      if (sync.fingers.length) {
        phase = `等待${sync.fingers.map(side => handName[side]).join("、")}抬指`;
      } else {
        const wrists = Object.entries(sync.wrist_errors).map(([side, error]) =>
          `${handName[side]} ${error.note} 偏差 ${error.distance_mm.toFixed(1)} mm、J7 ${Math.abs(error.j7_deg).toFixed(1)}°`);
        phase = `等待腕位 · ${wrists.join(" / ")}`;
      }
    }
    if (run.state === "HOLDING_TARGET") phase = "演奏结束 · 双臂保持";
    if (run.piano_exiting || state.exitingPerformance) phase = "正在退出演奏";
    if (run.piano_exited) phase = "已退出演奏 · 双臂保持使能" + (fingers?.active ? " · 正在抬指" : "");
    message = `${trial.score_name} · ${trial.speed}× · ${phase}`;
    if (trial.pause_times_us.length) message += ` · 谱面延时 ${trial.pause_times_us.length} 秒`;
    const feedbackDelay = run.sample?.sync_delay_s ?? 0;
    if (feedbackDelay > 0) message += ` · 反馈等待 ${feedbackDelay.toFixed(1)} 秒`;
    if (trial.skip_notice) message += " · " + trial.skip_notice;
    if (trial.finger_notice) message += ` · 指法未启动：${trial.finger_notice}`;
    if (run.piano_exited && fingers?.message?.includes("抬起未确认")) message += " · " + fingers.message;
  }
  $("performance-status").textContent = message;
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
      cell.textContent = coords ? coords.map(value => value.toFixed(2)).join(" / ") : "—";
    }
    wristCells[side].forEach((cell, index) => {
      const angle = arm?.wrist_rpy_deg?.[index];
      cell.textContent = wristInputs[side][index].placeholder = angle?.toFixed(2) ?? "—";
    });
  }
  for (const cell of fingerNoteCells) {
    const {handNote, finger} = cell.dataset;
    const key = status?.arms[handNote].piano_finger_keys?.[finger];
    cell.textContent = key == null ? "—" : whiteNote(key);
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
  if (target.piano_trial) {
    const trial = target.piano_trial;
    if (trial.scope === "start") {
      $("target-note").textContent = `${armName[trial.side]}食指 ${trial.note} · 实琴白键 ${trial.physical_key} / 曲谱白键 ${trial.target_key} · ${target.duration_s} 秒`;
      return;
    }
    const wrists = sides.filter(side => trial.first_wrist_targets[side]).map(side =>
      armName[side] + " " + trial.first_wrist_targets[side].note).join(" / ");
    const approach = trial.from_score_start ? "从已保持的首腕位接续" : "首腕位 " + wrists;
    $("target-note").textContent = `${approach} · ${trial.note_count} 音 · ${trial.speed}× · ` +
      `入位准备 ${trial.prepare_s.toFixed(2)} 秒 · 共 ${target.duration_s.toFixed(1)} 秒` +
      (trial.skip_notice ? " · " + trial.skip_notice : "");
    return;
  }
  $("target-note").textContent = `${joints.join("、")} · ${target.duration_s} s · 峰值 ${target.peak_speed_deg_s.toFixed(2)} °/s`;
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
  let activeLabel = states[run?.state];
  if (run?.piano_exited) activeLabel = "已退出演奏 · 使能保持中";
  if (run?.piano_exiting) activeLabel = "正在退出演奏";
  if (run?.stop_reason || run?.state === "STOP_REQUESTED") activeLabel = "正在停止";
  $("execution-state").textContent = !state.status ?
    (run?.active ? "连接中断 · 等待停止回报" : "等待本机服务") :
    state.status.pending_mode ? "正在切换并读回 " + state.status.pending_mode :
    state.status.mode_error ? state.status.mode_error :
    run ? (run.active ? activeLabel : run.message || states[run.state]) :
      state.status.log_error ? "日志写入失败，运动已锁定" :
        state.status.can_switch_mode && !state.status.drive_mode ? "请选择电机模式，核对 14 轴回读" :
        state.status.can_prepare ? "就绪" : "等待双臂静止与失能回读";
  const progress = $("progress");
  progress.hidden = !state.status || !run?.active || !run?.sample || run.piano_exited;
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
  const pianoPoseAvailable = row?.piano_pose_available ?? (side === "left");
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
  $("hand-piano_pose").disabled = !pianoPoseAvailable || !ready;
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
    row && !pianoPoseAvailable && "重启控制台后可使用右手弹琴指位",
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
for (const action of ["connect", "disconnect", "piano_pose", ...handActions]) {
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
        piano_pose: `已发送${side === "left" ? "左手" : "右手"}弹琴指位`,
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
  const targets = Object.fromEntries(selectedArms().map(side => {
    const fields = isIK && hasWristTarget(side) ? [...inputs[side], ...wristInputs[side]] : inputs[side];
    return [side, fields.map(input => input.value === "" ? null : input.valueAsNumber)];
  }));
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
  sides.forEach(side => [...ikInputs[side], ...wristInputs[side]].forEach(input => { input.value = ""; }));
  controlsChanged();
});
$("reset").addEventListener("click", () => {
  HTMLFormElement.prototype.reset.call($("control-form"));
  controlsChanged();
});
$("use-current").addEventListener("click", () => {
  for (const side of selectedArms()) {
    const point = state.status?.arms[side].endpoints_mm[$("endpoint").value];
    if (point) ikInputs[side].forEach((input, index) => { input.value = point[index].toFixed(2); });
  }
  controlsChanged();
});
$("use-current-wrist").addEventListener("click", () => {
  for (const side of selectedArms()) {
    const rpy = state.status?.arms[side].wrist_rpy_deg;
    if (rpy) wristInputs[side].forEach((input, index) => { input.value = rpy[index].toFixed(2); });
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
for (const id of ["trial-side", "trial-key", "trial-score", "trial-scope", "trial-hand", "trial-speed"]) {
  $(id).addEventListener("input", () => {
    if (state.target?.piano_trial) invalidate("试弹设置已变化，请重新预览。");
    $("trial-fingering").hidden = true;
    updateControls();
  });
}
async function previewTrial(action) {
  const speed = $("trial-speed");
  const positionOnly = action === "position";
  const scope = positionOnly ? "start" : $("trial-scope").value;
  const scoreId = $("trial-score").value;
  if (action === "play" && !speed.reportValidity()) return;
  state.notice = null;
  invalidate("正在读取钢琴点位与曲谱。");
  state.pending = true;
  updateControls();
  try {
    if (action === "fingering") {
      const result = await api("/api/piano/fingering", {scope, score_id: scoreId}, 15000);
      showFingering(result);
      $("target-state").textContent = "双手分指预览";
      return;
    }
    const target = await api("/api/prepare", {mode: "piano-trial", side: $("trial-side").value,
      key: Number($("trial-key").value), scope, score_id: scoreId, trial_hand: $("trial-hand").value,
      speed: positionOnly ? .35 : speed.valueAsNumber}, 15000);
    state.target = target;
    showTarget(target, target.piano_trial.dual ? sides : [target.piano_trial.side ?? "right"]);
  } catch (error) {
    actionError("试弹预览未就绪", error);
  } finally {
    state.pending = false;
    updateControls();
    renderRun();
  }
}
function showFingering(result) {
  const skipReason = {outside_physical_keyboard: "超出实琴", no_common_wrist: "无共同腕位",
    no_finger_at_solved_wrist: "无可用手指"};
  const firstShown = new Set();
  $("fingering-rows").replaceChildren(...result.rows.map(row => {
    const tr = create("tr");
    tr.append(create("td", (row.time_us/1e6).toFixed(3)), create("td", row.notes.map(n => whiteNote(n.key)).join(" ")));
    if (row.fingers) {
      for (const side of sides) {
        const key = row.wrists[side];
        const fingers = row.fingers[side].map(n =>
          (pianoFingers[n.finger] ?? skipReason[n.skip_reason] ?? "跳过") + " " + whiteNote(n.key));
        const cell = create("td", fingers.join("、") || (key == null ? "休止" : "延音"));
        if (key != null) {
          cell.append(create("div", `食指对位 ${whiteNote(key)} · 实琴 ${key - 9}`, "muted"));
        }
        const target = key != null && !firstShown.has(side) && result.first_wrist_targets?.[side];
        if (target) {
          firstShown.add(side);
          cell.append(create("div", `XYZ（mm） ${target.xyz_mm.map(mm => mm.toFixed(2)).join("，")}`, "small muted"));
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
  $("fingering-summary").textContent = `原谱 ${result.rows.length} 组、${result.note_count} 音，${result.unassigned} 组含跳过音。` +
    (firstDistance == null ? "" : `首和弦两腕相距 ${firstDistance.toFixed(2)} mm。`);
  $("trial-fingering").hidden = false;
}
$("trial-position").addEventListener("click", () => previewTrial("position"));
$("trial-preview").addEventListener("click", () => previewTrial("fingering"));
$("trial-dual-preview").addEventListener("click", () => previewTrial("play"));
$("performance-score").addEventListener("input", updateControls);
$("performance-play").addEventListener("click", async () => {
  if ($("performance-play").disabled) return;
  const scoreId = $("performance-score").value;
  const sessions = Object.fromEntries(sides.map(side => [side, state.status.hands[side].session]));
  state.notice = null;
  state.pending = true;
  invalidate("正在准备双臂与双手演奏…");
  updateControls();
  try {
    state.run = await api("/api/piano/play", {score_id: scoreId, sessions}, 15000);
    $("target-state").textContent = "双臂与双手演奏已提交";
  } catch (error) {
    actionError("演奏未受理", error);
  } finally {
    state.pending = false;
    updateControls();
    renderRun();
  }
});
$("performance-exit").addEventListener("click", async () => {
  if ($("performance-exit").disabled) return;
  const requestId = state.run.request_id;
  state.notice = null;
  state.exitingPerformance = true;
  updateControls();
  try {
    await api("/api/piano/exit", {request_id: requestId});
    if (state.run?.request_id === requestId && state.run.active && !state.run.piano_exited) {
      state.run = {...state.run, piano_exiting: true};
    }
  } catch (error) {
    actionError("退出演奏未受理", error);
  } finally {
    state.exitingPerformance = false;
    updateControls();
    renderRun();
  }
});
$("execute").addEventListener("click", async () => {
  if ($("execute").disabled) return;
  const requestId = state.target.request_id;
  state.notice = null;
  state.pending = true;
  updateControls();
  try {
    const request = {request_id: requestId};
    const trial = state.target?.piano_trial;
    if (trial && trial.scope !== "start") {
      const trialSides = trial.dual ? sides : ["right"];
      request.sessions = Object.fromEntries(trialSides.map(side => [side, state.status?.hands?.[side]?.session]));
    }
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
  ["start-zero-return", "/api/start/zero-return", "逆方向返回"],
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
