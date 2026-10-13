const $ = (id) => document.getElementById(id);

const GROUPS = [
  {key: "torso", label: "躯干", count: 4, unit: "deg"},
  {key: "left_arm", label: "左臂", count: 7, unit: "deg"},
  {key: "right_arm", label: "右臂", count: 7, unit: "deg"},
  {key: "left_gripper", label: "左夹爪", count: 1, unit: "open_0_100"},
  {key: "right_gripper", label: "右夹爪", count: 1, unit: "open_0_100"}
];

const ACTION_TYPES = [
  "joint_targets", "switch_camera", "yolo_open", "latch_yolo", "read_yolo_detections",
  "yolo_close", "read_ee_pose", "ee_plan", "ee_plan_relative", "ee_axial_motion", "grasp",
  "chassis_velocity", "navigate_to_pose", "foundation_pose_open", "foundation_pose_close", "read_foundation_pose",
  "aruco_pose_open", "aruco_pose_close", "aruco_fine_enable", "aruco_fine_disable",
  "code_workflow"
];

const EE_AXIS_EXAMPLE_SEGMENTS = [
  {start_deg: 0, end_deg: 100, speed_deg_s: 5},
  {start_deg: 100, end_deg: 160, speed_deg_s: 2}
];

const PAGE_TITLES = {
  dashboard: "仪表盘",
  joints: "关节控制",
  ee: "夹爪末端 6D 位姿控制",
  chassis: "底盘控制",
  navigation: "导航",
  camera: "相机",
  recognition: "识别",
  vla: "VLA",
  workflow: "原子动作工作流",
  diagnostics: "诊断"
};

let state = {};
let selectedAction = null;
let selectedActionGroup = null;
let selectedSequence = null;
const STREAM_SLOT_COUNT = 4;
let streamSelections = ["head_left_rgb", "head_depth", "wrist_left_rgb", "wrist_right_rgb"];
let streamActive = Array(STREAM_SLOT_COUNT).fill(false);
let streamCaptureStatus = Array(STREAM_SLOT_COUNT).fill("");
let workflowStreamSelection = "";
let codeWorkflowPreviewSelection = "";
let editingStepParams = new Set();
let selectedCodeWorkflowId = "";
let codeWorkflowParamsDirty = false;
let workflowMessages = [];
let codeWorkflowRequestPending = false;
let codeWorkflowRequestGeneration = 0;
let vlaDataset = null;
let vlaDatasets = [];
let vlaFrame = null;
let vlaLastInfer = null;
let vlaCurves = null;
let vlaRequestPending = false;
let vlaFrameLoadTimer = null;
let vlaPlaybackTimer = null;
let vlaPlaybackLoadInFlight = false;
let vlaPlaybackTimeout = null;
let vlaPlaybackRunId = 0;
let vlaFullInferRunId = 0;
let vlaFullInferRunning = false;
let vlaFullInferComparison = null;
let vlaTaskRunId = 0;
let vlaTaskRunning = false;
let vlaFrameCache = new Map();
let vlaMcapEvalResult = null;
let vlaMcapEvalFetchPending = false;
let chassisHoldTimer = null;
let chassisKeyboardActive = false;
let navigationMap = null;
let navigationMapFetchPending = false;
let navigationView = {scale: 1, panX: 0, panY: 0, fit: true};
let navigationImages = {};
let navigationDraft = null;
let navigationDragStart = null;
let streamClickModes = Array(STREAM_SLOT_COUNT).fill(false);
let streamClickPending = Array(STREAM_SLOT_COUNT).fill(false);
let streamClickMarkers = Array(STREAM_SLOT_COUNT).fill(null);
let streamClickActiveSlot = 0;
let streamClickConfiguredCamera = "";
let headDepthProcessorPending = false;
let recognitionRequestPending = "";
let recognitionUiError = "";
let recognitionUiMessage = "";
let recognitionYoloConfigSyncKey = "";
let recognitionAssets = [];
let recognitionAssetsLoading = false;
let recognitionMatchTimer = null;
let recognitionMatchSequence = 0;
let recognitionMatchResult = null;
let recognitionPoseFrame = "base";
let editingActionDirty = false;
let editingSequenceDirty = false;
// The server issues this token only after the operator explicitly validates
// the current continuous-rotation settings.  Any edit invalidates it.
let eeAxisValidationToken = "";
const locks = JSON.parse(localStorage.getItem("r1pro.locks") || "{}");
delete locks.vision;
["joints", "ee", "chassis", "navigation", "camera", "recognition", "vla", "workflow"].forEach((key) => {
  locks[key] = Boolean(locks[key]);
});
let jsonMode = localStorage.getItem("r1pro.jsonMode") === "1";

function token() {
  return localStorage.getItem("r1pro.token") || "";
}

async function fetchJson(path, options = {}, timeoutMs = 10000) {
  const controller = new AbortController();
  const timer = timeoutMs > 0 ? setTimeout(() => controller.abort(), timeoutMs) : null;
  try {
    const response = await fetch(path, {...options, signal: controller.signal});
    const data = await response.json().catch(() => ({}));
    if (!response.ok || data.ok === false) {
      throw new Error(data.message || `${path} failed`);
    }
    return data;
  } catch (exc) {
    if (exc?.name === "AbortError") {
      throw new Error(`${path} 请求超时`);
    }
    throw exc;
  } finally {
    if (timer) clearTimeout(timer);
  }
}

async function post(path, payload = {}, options = {}) {
  return fetchJson(path, {
    method: "POST",
    headers: {"Content-Type": "application/json", "X-Control-Token": token()},
    body: JSON.stringify(payload)
  }, options.timeoutMs ?? 10000);
}

function setConn(ok, text) {
  const el = $("conn");
  el.textContent = text;
  el.className = ok ? "pill" : "pill warn";
}

function renderFacts(root, data) {
  root.innerHTML = "";
  Object.entries(data).forEach(([key, value]) => {
    const dt = document.createElement("dt");
    const dd = document.createElement("dd");
    dt.textContent = key;
    dd.textContent = value === undefined || value === null || value === "" ? "-" : String(value);
    root.append(dt, dd);
  });
}

function summary(root, items) {
  root.innerHTML = "";
  if (!items.length) {
    root.innerHTML = `<div class="empty">无数据</div>`;
    return;
  }
  items.forEach((item) => {
    const div = document.createElement("div");
    div.className = "summary-item";
    div.innerHTML = `<strong>${escapeHtml(item.label)}</strong><span>${escapeHtml(item.value)}</span>`;
    root.appendChild(div);
  });
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
  }[ch]));
}

function clone(value) {
  return JSON.parse(JSON.stringify(value ?? {}));
}

function isScalarArray(value) {
  return Array.isArray(value) && value.every((item) => (
    item === null || ["string", "number", "boolean"].includes(typeof item)
  ));
}

function paramKind(value) {
  if (typeof value === "boolean") return "boolean";
  if (typeof value === "number") return "number";
  if (isScalarArray(value)) return "array";
  if (value === null || value === undefined) return "string";
  if (typeof value === "string") return "string";
  return "complex";
}

function arraySubtype(value) {
  return Array.isArray(value) && value.every((item) => typeof item === "number") ? "number" : "string";
}

function formatParamValue(value) {
  if (Array.isArray(value)) return value.join(", ");
  if (value === null || value === undefined) return "";
  return String(value);
}

function parseParamValue(input) {
  const kind = input.dataset.kind;
  if (kind === "boolean") return input.checked;
  if (kind === "number") return Number(input.value);
  if (kind === "array") {
    const subtype = input.dataset.subtype;
    const values = input.value.split(",").map((item) => item.trim()).filter((item) => item !== "");
    return subtype === "number" ? values.map(Number) : values;
  }
  return input.value;
}

function number(value, digits = 3) {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : "-";
}

function maxOpenM() {
  return Number(state.config?.gripper_max_open_m || 0.10);
}

function gripperOpenToM(open) {
  return Number(open || 0) / 100.0 * maxOpenM();
}

function gripperMToOpen(meters) {
  const max = maxOpenM();
  return max > 0 ? Number(meters || 0) / max * 100.0 : 0;
}

function lockToggleHtml(page) {
  return `
    <label class="lock-toggle ${locks[page] ? "unlocked" : ""}">
      <input type="checkbox" data-lock-toggle="${page}" ${locks[page] ? "checked" : ""}>
      <span>${locks[page] ? "安全锁已打开" : "安全锁已关闭"}</span>
    </label>`;
}

function renderLocks() {
  ["joints", "ee", "chassis", "navigation", "camera", "recognition", "vla", "workflow"].forEach((page) => {
    const root = $(`lock-${page}`);
    if (root) root.innerHTML = lockToggleHtml(page);
  });
  document.querySelectorAll("[data-lock-page]").forEach((el) => {
    const page = el.dataset.lockPage;
    const pending = (el.id === "runCodeWorkflow" && codeWorkflowRequestPending)
      || (el.id === "headDepthProcessorEnable" && headDepthProcessorPending);
    const clickMatch = el.id.match(/^toggleStreamClick(\d+)$/);
    const slot = clickMatch ? Number(clickMatch[1]) : -1;
    const clickBlocked = slot >= 0 && (
      !streamActive[slot]
      || Boolean(streamClickCameraProblem(streamClickCameraKey(streamSelections[slot] || selectedStreamKey(slot))))
    );
    const recognitionProblem = recognitionControlProblem(el.id);
    const disabled = !locks[page] || pending || clickBlocked || Boolean(recognitionProblem);
    el.disabled = disabled;
    el.classList.toggle("disabled", disabled);
    if (recognitionProblem) {
      el.title = recognitionProblem;
      el.dataset.recognitionGate = "1";
    } else if (el.dataset.recognitionGate) {
      el.removeAttribute("title");
      delete el.dataset.recognitionGate;
    }
  });
  updateVlaPolicyControls();
  updateVlaTaskControls();
  updateNavigationControls();
  const runButton = $("runCodeWorkflow");
  if (runButton) runButton.textContent = codeWorkflowRequestPending ? "请求中..." : "执行代码工作流";
}

function requireUnlocked(page) {
  if (locks[page]) return true;
  alert("请先打开本页安全锁");
  return false;
}

function lockAllPages() {
  ["joints", "ee", "chassis", "navigation", "camera", "recognition", "vla", "workflow"].forEach((page) => {
    locks[page] = false;
  });
  localStorage.setItem("r1pro.locks", JSON.stringify(locks));
  renderLocks();
}

function activePage() {
  return document.querySelector(".nav.active")?.dataset.page || "dashboard";
}

function render() {
  const activeProfile = state.active_control_profile || state.control_profiles?.active || "real";
  $("subtitle").textContent = `${state.base_frame || "base_link"} | ${controlModeLabel(activeProfile)}`;
  renderLocks();
  renderJsonMode();
  renderDashboard();
  renderJoints();
  renderEe();
  renderChassis();
  renderNavigation();
  renderVision();
  renderVla();
  renderWorkflow();
  renderDiagnostics();
  renderLocks();
  renderJsonMode();
}

function renderJsonMode() {
  document.body.classList.toggle("json-mode", jsonMode);
  document.querySelectorAll(".json-only").forEach((el) => {
    el.hidden = !jsonMode;
  });
  const toggle = $("toggleJsonMode");
  if (toggle) toggle.textContent = jsonMode ? "关闭 JSON 模式" : "打开 JSON 模式";
  const status = $("jsonModeStatus");
  if (status) status.textContent = jsonMode ? "已打开" : "已关闭";
}

function renderDashboard() {
  renderFacts($("systemFacts"), {
    base_frame: state.base_frame,
    web_namespace: state.web_namespace,
    control: controlModeLabel(state.active_control_profile || "real"),
    feedback_source: controlModeLabel(state.control_profiles?.active_feedback_profile || state.active_control_profile || "real"),
    actions: (state.actions || []).length,
    workflows: (state.sequences || []).length,
    ik: state.ik ? "ready" : "unavailable"
  });
  const battery = state.battery || {};
  renderFacts($("batteryFacts"), {
    status: battery.status,
    percentage: battery.percentage == null ? "-" : `${battery.percentage}%`,
    voltage: battery.voltage == null ? "-" : `${battery.voltage} V`,
    current: battery.current == null ? "-" : `${battery.current} A`,
    capital: battery.capital,
    source: battery.source || "-",
    message: battery.message || "-",
    age_s: battery.age_s
  });
  renderFacts($("timeFacts"), {
    local: new Date().toLocaleString(),
    backend_utc: state.time || "-"
  });
  const runtime = state.sequence_runtime || {};
  renderFacts($("runtimeFacts"), {
    status: runtime.running ? "running" : (runtime.message || "idle"),
    name: runtime.name || "-",
    step: runtime.step_index || "-",
    action: runtime.action_name || "-",
    uptime_s: runtime.uptime_s ?? "-"
  });
  renderControlProfilePanel();
  const root = $("feedbackGrid");
  root.innerHTML = "";
  Object.entries(state.feedback || {}).forEach(([key, fb]) => {
    const div = document.createElement("div");
    div.className = "feedback-item";
    const values = (fb.positions_display || []).map((v) => number(v, key.includes("gripper") ? 2 : 1)).join(", ");
    div.innerHTML = `<strong>${key}</strong><span>${values || "-"}</span><small>age ${fb.age_s ?? "-"}</small>`;
    root.appendChild(div);
  });
}

function controlModeLabel(key) {
  const labels = {real: "真机", sim: "仿真", both: "同时控制"};
  return labels[key] || key || "-";
}

function renderControlProfilePanel() {
  const root = $("controlProfilePanel");
  if (!root) return;
  const profiles = state.control_profiles || {};
  const active = profiles.active || state.active_control_profile || "real";
  const feedbackProfile = profiles.active_feedback_profile || active;
  const modes = profiles.modes || [
    {key: "real", label: "真机"},
    {key: "sim", label: "仿真"},
    {key: "both", label: "同时控制"}
  ];
  root.innerHTML = "";
  const buttons = document.createElement("div");
  buttons.className = "segmented";
  modes.forEach((mode) => {
    const button = document.createElement("button");
    button.textContent = mode.label || controlModeLabel(mode.key);
    button.dataset.controlProfile = mode.key;
    button.className = active === mode.key ? "active" : "";
    buttons.appendChild(button);
  });
  const facts = document.createElement("dl");
  facts.className = "profile-facts";
  const targets = (profiles.target_profiles || [active]).map(controlModeLabel).join(" + ");
  renderFacts(facts, {
    active: controlModeLabel(active),
    targets,
    feedback_source: controlModeLabel(feedbackProfile),
    real_age_s: profileAgeSummary("real"),
    sim_age_s: profileAgeSummary("sim")
  });
  const note = document.createElement("p");
  note.className = "muted-note";
  note.textContent = active === "both" ? "同时控制时反馈和 IK 只使用真机；真机无反馈会按缺失/超时报错。" : "";
  root.append(buttons, facts, note);
}

function profileAgeSummary(profileKey) {
  const profile = state.control_profiles?.profiles?.[profileKey];
  if (!profile) return "-";
  const ages = profile.feedback_age_s || {};
  const values = Object.values(ages).filter((value) => value !== null && value !== undefined);
  if (!values.length) return "waiting";
  return `${Math.min(...values).toFixed(3)}-${Math.max(...values).toFixed(3)}`;
}

function jointRows() {
  const rows = [];
  GROUPS.forEach((group) => {
    const fb = state.feedback?.[group.key] || {};
    for (let index = 0; index < group.count; index += 1) {
      const current = Number((fb.positions_display || [])[index] || 0);
      rows.push({group, index, current});
    }
  });
  return rows;
}

function renderJoints() {
  const root = $("jointTable");
  const rows = jointRows();
  if (!root.dataset.ready) {
    root.dataset.ready = "1";
    root.innerHTML = `
      <div class="table-head">
        <span>选</span><span>部位</span><span>序号</span><span>当前</span><span>目标</span><span>米值</span><span>操作</span>
      </div>
      <div id="jointRows"></div>`;
    $("jointRows").addEventListener("input", onJointInput);
    $("jointRows").addEventListener("click", (event) => {
      if (!event.target.classList.contains("joint-exec")) return;
      if (!requireUnlocked("joints")) return;
      const row = event.target.closest(".joint-row");
      post("/api/control/joint_targets", {targets: selectedJointTargets(row), timeout_s: 3}).catch(alert);
    });
  }
  const body = $("jointRows");
  const existing = new Set([...body.querySelectorAll(".joint-row")].map((el) => el.dataset.key));
  rows.forEach((row) => {
    const key = `${row.group.key}:${row.index}`;
    let el = body.querySelector(`[data-key="${key}"]`);
    if (!el) {
      el = document.createElement("div");
      el.className = "joint-row";
      el.dataset.key = key;
      el.dataset.group = row.group.key;
      el.dataset.index = String(row.index);
      const isGrip = row.group.unit !== "deg";
      el.innerHTML = `
        <input class="joint-check" type="checkbox">
        <span>${row.group.label}</span>
        <span>${row.index + 1}</span>
        <span class="joint-current">-</span>
        <input class="joint-target" type="number" step="${isGrip ? "0.1" : "0.1"}">
        <input class="joint-meter" type="number" step="0.001" ${isGrip ? "" : "disabled"}>
        <button class="joint-exec" data-lock-page="joints">执行</button>`;
      body.appendChild(el);
    }
    existing.delete(key);
    el.querySelector(".joint-current").textContent = row.group.unit === "deg"
      ? `${number(row.current, 2)} deg`
      : `${number(row.current, 2)} / ${number(gripperOpenToM(row.current), 3)} m`;
    if (!el.dataset.initialized || el.dataset.dirty !== "1") {
      el.querySelector(".joint-target").value = number(row.current, row.group.unit === "deg" ? 2 : 2);
      if (row.group.unit !== "deg") el.querySelector(".joint-meter").value = number(gripperOpenToM(row.current), 3);
      el.dataset.initialized = "1";
    }
  });
  existing.forEach((key) => body.querySelector(`[data-key="${key}"]`)?.remove());
  renderPageActions("joints", $("jointActionList"));
}

function onJointInput(event) {
  const row = event.target.closest(".joint-row");
  if (row && (event.target.classList.contains("joint-target") || event.target.classList.contains("joint-meter"))) {
    row.dataset.dirty = "1";
  }
  if (!row || !row.dataset.group.includes("gripper")) return;
  if (event.target.classList.contains("joint-target")) {
    row.querySelector(".joint-meter").value = number(gripperOpenToM(Number(event.target.value)), 3);
  }
  if (event.target.classList.contains("joint-meter")) {
    row.querySelector(".joint-target").value = number(gripperMToOpen(Number(event.target.value)), 2);
  }
}

function selectedJointTargets(rowOnly = null) {
  const rows = rowOnly ? [rowOnly] : [...document.querySelectorAll(".joint-row")].filter((row) => row.querySelector(".joint-check").checked);
  return rows.map((row) => ({
    group: row.dataset.group,
    index: Number(row.dataset.index),
    name: `${row.dataset.group}[${Number(row.dataset.index) + 1}]`,
    unit: row.dataset.group.includes("gripper") ? "open_0_100" : "deg",
    value: Number(row.querySelector(".joint-target").value)
  }));
}

function captureJoints() {
  document.querySelectorAll(".joint-row").forEach((row) => {
    const fb = state.feedback?.[row.dataset.group]?.positions_display || [];
    const value = Number(fb[Number(row.dataset.index)] || 0);
    row.querySelector(".joint-target").value = number(value, 2);
    row.dataset.dirty = "0";
    if (row.dataset.group.includes("gripper")) {
      row.querySelector(".joint-meter").value = number(gripperOpenToM(value), 3);
    }
  });
}

function setJointSelection(mode) {
  document.querySelectorAll(".joint-row .joint-check").forEach((checkbox) => {
    if (mode === "all") checkbox.checked = true;
    else if (mode === "none") checkbox.checked = false;
    else if (mode === "invert") checkbox.checked = !checkbox.checked;
  });
}

function renderEe() {
  const current = state.current_ee_poses || {};
  summary($("eeCurrent"), ["left", "right"].map((arm) => {
    const pose = current[arm];
    return {
      label: arm === "left" ? "左臂" : "右臂",
      value: pose ? `xyz ${pose.position.map((v) => number(v, 3)).join(", ")} | rpy ${pose.rpy_deg.map((v) => number(v, 1)).join(", ")}` : "-"
    };
  }));
  ensureEeForms();
  renderPageActions("ee", $("eeActionList"));
}

function ensureEeForms() {
  if (!$("eeTargetForm").dataset.ready) {
    $("eeTargetForm").dataset.ready = "1";
    $("eeTargetForm").innerHTML = fieldSet([
      ["eeX", "X m", "number", "0.001", "0"],
      ["eeY", "Y m", "number", "0.001", "0"],
      ["eeZ", "Z m", "number", "0.001", "0"],
      ["eeRoll", "Roll deg", "number", "0.1", "0"],
      ["eePitch", "Pitch deg", "number", "0.1", "0"],
      ["eeYaw", "Yaw deg", "number", "0.1", "0"],
      ["eeTimeout", "执行时间 s", "number", "0.1", "3"]
    ]) + `<label>躯干IK<input id="eeAllowTorsoIk" type="checkbox"></label>`;
  }
  if (!$("eeRelativeForm").dataset.ready) {
    $("eeRelativeForm").dataset.ready = "1";
    $("eeRelativeForm").innerHTML = `
      <label>参考<select id="eeRelSource"><option value="current">当前末端</option><option value="latched_ee_pose">锁存末端</option><option value="workflow_pose">工作流位姿</option></select></label>
      <label>坐标系<select id="eeRelFrame"><option value="base">base</option><option value="ee">ee</option></select></label>
      ${fieldSet([
        ["eeDx", "dX m", "number", "0.001", "0"],
        ["eeDy", "dY m", "number", "0.001", "0"],
        ["eeDz", "dZ m", "number", "0.001", "0.10"],
        ["eeDroll", "dRoll deg", "number", "0.1", "0"],
        ["eeDpitch", "dPitch deg", "number", "0.1", "0"],
        ["eeDyaw", "dYaw deg", "number", "0.1", "0"]
      ])}`;
  }
  if (!$("eeJogButtons").dataset.ready) {
    $("eeJogButtons").dataset.ready = "1";
    [
      ["X-", [-1, 0, 0], [0, 0, 0]], ["X+", [1, 0, 0], [0, 0, 0]],
      ["Y-", [0, -1, 0], [0, 0, 0]], ["Y+", [0, 1, 0], [0, 0, 0]],
      ["Z-", [0, 0, -1], [0, 0, 0]], ["Z+", [0, 0, 1], [0, 0, 0]],
      ["R-", [0, 0, 0], [-1, 0, 0]], ["R+", [0, 0, 0], [1, 0, 0]],
      ["P-", [0, 0, 0], [0, -1, 0]], ["P+", [0, 0, 0], [0, 1, 0]],
      ["Yw-", [0, 0, 0], [0, 0, -1]], ["Yw+", [0, 0, 0], [0, 0, 1]]
    ].forEach(([label, xyz, rpy]) => {
      const btn = document.createElement("button");
      btn.textContent = label;
      btn.dataset.eeJog = JSON.stringify({xyz, rpy});
      btn.dataset.lockPage = "ee";
      $("eeJogButtons").appendChild(btn);
    });
  }
  if (!$("eeAxisJogButtons").dataset.ready) {
    $("eeAxisJogButtons").dataset.ready = "1";
    [["绕L+", 1], ["绕L−", -1]].forEach(([label, direction]) => {
      const btn = document.createElement("button");
      btn.textContent = label;
      btn.dataset.eeAxisJog = String(direction);
      btn.dataset.lockPage = "ee";
      btn.classList.add("ee-axis-jog");
      $("eeAxisJogButtons").insertBefore(btn, $("eeAxisJogButtons").firstChild);
    });
  }
}

function fieldSet(specs) {
  return specs.map(([id, label, type, step, value]) => (
    `<label>${label}<input id="${id}" type="${type}" step="${step}" value="${value}"></label>`
  )).join("");
}

function eePosePayload() {
  return {
    arm: $("eeArm").value,
    xyz_m: [Number($("eeX").value), Number($("eeY").value), Number($("eeZ").value)],
    rpy_deg: [Number($("eeRoll").value), Number($("eePitch").value), Number($("eeYaw").value)],
    orientation: "rpy",
    timeout_s: Number($("eeTimeout").value || 3),
    allow_torso_ik: Boolean($("eeAllowTorsoIk")?.checked)
  };
}

function eeRelativePayload() {
  return {
    arm: $("eeArm").value,
    reference_source: $("eeRelSource").value,
    offset_frame: $("eeRelFrame").value,
    delta_xyz_m: [Number($("eeDx").value), Number($("eeDy").value), Number($("eeDz").value)],
    delta_rpy_deg: [Number($("eeDroll").value), Number($("eeDpitch").value), Number($("eeDyaw").value)],
    orientation: "rpy",
    timeout_s: Number($("eeTimeout").value || 3),
    allow_torso_ik: Boolean($("eeAllowTorsoIk")?.checked)
  };
}

function copyEeCurrent() {
  const pose = state.current_ee_poses?.[$("eeArm").value];
  if (!pose) return;
  [$("eeX").value, $("eeY").value, $("eeZ").value] = pose.position.map((v) => number(v, 4));
  [$("eeRoll").value, $("eePitch").value, $("eeYaw").value] = pose.rpy_deg.map((v) => number(v, 2));
}

function renderChassis() {
  ensureChassisForms();
  renderPageActions("chassis", $("chassisActionList"));
  const nav = state.navigation || {};
  const guard = $("chassisNavigationGuard");
  if (guard) {
    guard.textContent = nav.manual_control_allowed
      ? "手动底盘控制可用"
      : `手动控制已禁用：${navigationControlLabel(nav)}`;
    guard.classList.toggle("warn", !nav.manual_control_allowed);
  }
  updateNavigationControls();
}

function ensureChassisForms() {
  if (!$("drivePad").dataset.ready) {
    $("drivePad").dataset.ready = "1";
    [
      ["前", 1, 0, 0], ["左移", 0, 1, 0], ["右转", 0, 0, -1],
      ["后", -1, 0, 0], ["右移", 0, -1, 0], ["左转", 0, 0, 1],
      ["停止", 0, 0, 0]
    ].forEach(([label, x, y, w]) => {
      const btn = document.createElement("button");
      btn.textContent = label;
      btn.dataset.drive = [x, y, w].join(",");
      btn.dataset.lockPage = "chassis";
      $("drivePad").appendChild(btn);
    });
  }
  if (!$("chassisActionForm").dataset.ready) {
    $("chassisActionForm").dataset.ready = "1";
    $("chassisActionForm").innerHTML = fieldSet([
      ["actVx", "vx m/s", "number", "0.01", "0.10"],
      ["actVy", "vy m/s", "number", "0.01", "0.00"],
      ["actOmega", "omega rad/s", "number", "0.01", "0.00"],
      ["actDuration", "duration s", "number", "0.1", "1.0"]
    ]);
  }
}

function chassisPayloadFromScalars(x, y, w, duration = null) {
  const linear = Number($("chassisLinear").value || 0.1);
  const angular = Number($("chassisAngular").value || 0.3);
  return {
    vx: x * linear,
    vy: y * linear,
    omega: w * angular,
    duration_s: duration ?? Number($("chassisDuration").value || 0.35)
  };
}

function chassisActionPayload() {
  return {
    vx_mps: Number($("actVx").value),
    vy_mps: Number($("actVy").value),
    omega_radps: Number($("actOmega").value),
    duration_s: Number($("actDuration").value)
  };
}

const NAVIGATION_CONTROL_LABELS = {
  MANUAL: "手动",
  SWITCHING: "切换中",
  NAVIGATION: "导航",
  FAULT: "故障"
};

function navigationControlLabel(nav = state.navigation || {}) {
  if (nav.localization_spin?.active) return "原地旋转定位中";
  const uiState = nav.ui_state || "FAULT";
  if (uiState === "NAVIGATION") {
    if (nav.nav_output_state === "active") return "导航运动中";
    if (nav.nav_output_state === "armed_idle") return "导航待命";
  }
  return NAVIGATION_CONTROL_LABELS[uiState] || uiState;
}

function updateNavigationControls() {
  const nav = state.navigation || {};
  const switching = nav.ui_state === "SWITCHING" || Boolean(nav.transitioning);
  const manualAllowed = Boolean(nav.manual_control_allowed);
  const goalAllowed = Boolean(nav.navigation_goal_allowed);
  const navigationMode = nav.ui_state === "NAVIGATION";
  const spinActive = Boolean(nav.localization_spin?.active);

  document.querySelectorAll("#drivePad [data-drive]").forEach((button) => {
    button.disabled = !locks.chassis || !manualAllowed;
  });
  if ($("chassisKeyboard")) {
    $("chassisKeyboard").disabled = !manualAllowed;
    if (!manualAllowed) $("chassisKeyboard").checked = false;
  }
  if ($("chassisExecuteAction")) $("chassisExecuteAction").disabled = !locks.chassis || !manualAllowed;

  if ($("navModeManual")) {
    $("navModeManual").disabled = (
      !locks.navigation || switching || spinActive || nav.ui_state === "MANUAL"
    );
  }
  if ($("navModeNavigation")) {
    $("navModeNavigation").disabled = (
      !locks.navigation || switching || spinActive || !nav.service_available || nav.ui_state === "NAVIGATION"
    );
  }
  if ($("navStartLocalizationSpin")) {
    $("navStartLocalizationSpin").disabled = (
      !locks.navigation || switching || !nav.localization_spin_start_allowed
    );
  }
  if ($("navCancelLocalizationSpin")) {
    $("navCancelLocalizationSpin").disabled = (
      !locks.navigation || !nav.localization_spin_cancel_allowed
    );
  }
  if ($("navSendGoal")) $("navSendGoal").disabled = !locks.navigation || !goalAllowed;
  if ($("navCancelGoal")) {
    $("navCancelGoal").disabled = (
      !locks.navigation || switching || spinActive || (!navigationMode && !nav.active_goal)
    );
  }
  if ($("navClearGlobalCostmap")) {
    $("navClearGlobalCostmap").disabled = (
      !locks.navigation
      || switching
      || spinActive
      || nav.emergency_latched
      || !nav.global_costmap_clear_service_ready
    );
  }
  if ($("navEmergencyStop")) $("navEmergencyStop").disabled = !locks.navigation;
}

function renderNavigation() {
  const nav = state.navigation || {};
  const pose = nav.pose || {};
  const map = nav.map || {};
  const goal = navigationDraft || nav.last_goal || {};
  const navOutput = nav.nav_output || {};
  const navOutputStatus = nav.nav_output_state || navOutput.status || "unknown";
  const comprehensiveStatus = nav.navigation_status || {};
  const spin = nav.localization_spin || {};
  const controlLabel = navigationControlLabel(nav);
  renderFacts($("navigationFacts"), {
    control: controlLabel,
    mode: nav.mode || "fault",
    service: nav.service_available ? "ready" : "unavailable",
    navigation_ready: nav.navigation_ready ? "ready" : "not ready",
    navigation_state: comprehensiveStatus.state || "waiting",
    navigation_message: comprehensiveStatus.message || "-",
    status_age: comprehensiveStatus.age_s == null ? "-" : `${number(comprehensiveStatus.age_s, 2)}s`,
    localization: nav.localized ? "ready (map → body TF)" : "not ready",
    inputs: comprehensiveStatus.inputs ? JSON.stringify(comprehensiveStatus.inputs) : "-",
    map: map.online ? `${map.width}x${map.height} @ ${number(map.resolution, 3)} m` : "waiting",
    pose: pose.online ? `${number(pose.x_m, 3)}, ${number(pose.y_m, 3)}, ${number(pose.yaw_deg, 1)} deg` : "waiting",
    goal_topic: `${nav.topics?.goal || "/goal_pose"} subs=${nav.goal_subscribers ?? "-"}`,
    nav_output: `${navOutputStatus}${navOutput.service_ready ? "" : " service waiting"}`,
    global_costmap_clear: nav.global_costmap_clear_service_ready ? "ready" : "service waiting",
    distance_m: nav.distance_to_goal_m ?? "-",
    yaw_error_deg: nav.yaw_error_deg ?? "-",
    plan_topic: nav.topics?.plan || nav.topics?.global_plan || "/plan",
    global_plan: `${nav.global_plan?.points || 0} points`,
    local_plan: `${nav.local_plan?.points || 0} points`,
    active_goal: nav.active_goal ? "yes" : "no",
    localization_spin: spin.active ? "running" : (spin.last_result || "never_started"),
    spin_progress: `${number(spin.progress_rad, 2)} / ${number(spin.target_yaw_rad, 2)} rad`,
    status: nav.last_error || nav.last_warning || nav.status || "-"
  });
  const navOutputStatusEl = $("navOutputStatus");
  if (navOutputStatusEl) {
    navOutputStatusEl.textContent = `ROS: ${navOutputStatus}`;
    navOutputStatusEl.classList.toggle("warn", !nav.service_available || nav.ui_state === "FAULT");
  }
  const controlState = $("navControlState");
  if (controlState) {
    controlState.textContent = controlLabel;
    controlState.classList.toggle("warn", nav.ui_state === "FAULT" || nav.ui_state === "SWITCHING");
  }
  const spinStatus = $("navLocalizationSpinStatus");
  if (spinStatus) {
    spinStatus.textContent = spin.active
      ? `旋转进度 ${number(spin.progress_rad, 2)} / ${number(spin.target_yaw_rad, 2)} rad`
      : `旋转结果：${spin.last_result || "never_started"}`;
    spinStatus.classList.toggle("warn", spin.last_result === "failed");
  }
  if (!navigationDraft && nav.last_goal) {
    setNavigationInputs(nav.last_goal, false);
  }
  const status = $("navigationStatus");
  if (status) {
    const warnings = [];
    if (nav.goal_subscribers === 0) warnings.push(`${nav.topics?.goal || "/goal_pose"} 无订阅者`);
    if (!nav.service_available) warnings.push("导航控制 service 不可用");
    if (!comprehensiveStatus.online) warnings.push("未收到或已失去 /navigation/status");
    else if (!nav.navigation_ready) warnings.push(comprehensiveStatus.message || "导航尚未就绪");
    if (!nav.map_ready) warnings.push("未收到 Web 地图图层 /map");
    if (spin.active) warnings.push("原地旋转定位中，手动控制、目标和模式切换已禁用");
    if (nav.last_error) warnings.push(nav.last_error);
    else if (nav.last_warning) warnings.push(nav.last_warning);
    status.textContent = warnings.length ? warnings.join(" | ") : "地图拖拽选点，拖拽方向，松开后点击发送目标";
    status.classList.toggle("warn", warnings.length > 0);
  }
  if (activePage() === "navigation") {
    const version = navigationVersionMarker(nav);
    const knownVersion = navigationVersionMarker(navigationMap || {});
    if (!navigationMap || version !== knownVersion) fetchNavigationMap(false);
    drawNavigationMap();
  }
  renderPageActions("navigation", $("navigationActionList"));
  updateNavigationControls();
}

function navigationVersionMarker(nav) {
  return [
    nav.map?.version || 0,
    nav.global_costmap?.version || 0,
    nav.local_costmap?.version || 0,
    nav.global_plan?.version || 0,
    nav.local_plan?.version || 0
  ].join(":");
}

function setNavigationInputs(goal, updateDraft = true) {
  if (!goal) return;
  if ($("navGoalX")) $("navGoalX").value = number(goal.x_m, 3);
  if ($("navGoalY")) $("navGoalY").value = number(goal.y_m, 3);
  if ($("navGoalYaw")) $("navGoalYaw").value = number(goal.yaw_deg, 1);
  if (updateDraft) {
    navigationDraft = {
      x_m: Number(goal.x_m),
      y_m: Number(goal.y_m),
      yaw_deg: Number(goal.yaw_deg || 0)
    };
    drawNavigationMap();
  }
}

function navigationGoalPayload() {
  return {
    x_m: Number($("navGoalX").value),
    y_m: Number($("navGoalY").value),
    yaw_deg: Number($("navGoalYaw").value)
  };
}

function navigationAtomicActionPayload() {
  return {
    ...navigationGoalPayload(),
    timeout_s: Number($("navActionTimeout").value || 120),
    position_tolerance_m: Number($("navPositionTolerance").value || 0.20),
    yaw_tolerance_deg: Number($("navYawTolerance").value || 20.0),
    stable_s: Number($("navStableTime").value || 0.5)
  };
}

async function fetchNavigationMap(force = false) {
  if (navigationMapFetchPending && !force) return;
  navigationMapFetchPending = true;
  try {
    navigationMap = await fetchJson(`/api/navigation/map?t=${Date.now()}`, {cache: "no-store"}, 10000);
    prepareNavigationImages();
    navigationView.fit = navigationView.fit || force;
    drawNavigationMap();
  } catch (exc) {
    const status = $("navigationStatus");
    if (status) {
      status.textContent = exc.message || String(exc);
      status.classList.add("warn");
    }
  } finally {
    navigationMapFetchPending = false;
  }
}

function prepareNavigationImages() {
  navigationImages = {};
  ["map", "global_costmap", "local_costmap"].forEach((key) => {
    const item = navigationMap?.[key];
    if (!item?.data_url) return;
    const img = new Image();
    img.onload = drawNavigationMap;
    img.src = item.data_url;
    navigationImages[key] = img;
  });
}

function navigationMeta() {
  return navigationMap?.map || null;
}

function fitNavigationView(canvas, meta) {
  const pad = 18;
  const sx = (canvas.clientWidth - pad * 2) / Math.max(1, meta.width);
  const sy = (canvas.clientHeight - pad * 2) / Math.max(1, meta.height);
  navigationView.scale = Math.max(0.05, Math.min(sx, sy));
  navigationView.panX = (canvas.clientWidth - meta.width * navigationView.scale) / 2;
  navigationView.panY = (canvas.clientHeight - meta.height * navigationView.scale) / 2;
  navigationView.fit = false;
}

function drawNavigationMap() {
  const canvas = $("navigationCanvas");
  const empty = $("navigationEmpty");
  if (!canvas) return;
  const meta = navigationMeta();
  const hasMap = Boolean(meta?.data_url && navigationImages.map?.complete);
  if (empty) empty.hidden = hasMap;
  canvas.hidden = !hasMap;
  if (!hasMap) return;
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(320, canvas.clientWidth);
  const height = Math.max(320, canvas.clientHeight);
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    navigationView.fit = true;
  }
  if (navigationView.fit) fitNavigationView(canvas, meta);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#27313a";
  ctx.fillRect(0, 0, width, height);
  ctx.save();
  ctx.translate(navigationView.panX, navigationView.panY);
  ctx.scale(navigationView.scale, navigationView.scale);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(navigationImages.map, 0, 0, meta.width, meta.height);
  if ($("navShowGlobalCostmap")?.checked && navigationImages.global_costmap?.complete) {
    drawNavigationGrid(ctx, navigationMap?.global_costmap, navigationImages.global_costmap, meta);
  }
  if ($("navShowLocalCostmap")?.checked && navigationImages.local_costmap?.complete) {
    drawNavigationGrid(ctx, navigationMap?.local_costmap, navigationImages.local_costmap, meta);
  }
  if ($("navShowGlobalPlan")?.checked) {
    drawNavigationPlan(ctx, navigationMap?.global_plan, meta, "#2878e8", 4, false);
  }
  if ($("navShowLocalPlan")?.checked) {
    drawNavigationPlan(ctx, navigationMap?.local_plan, meta, "#16a085", 3, true);
  }
  drawNavigationRobot(ctx, navigationMap?.pose, meta);
  drawNavigationPose(ctx, navigationDraft || navigationMap?.last_goal, meta, "#b42318", 13);
  ctx.restore();
}

// Every OccupancyGrid has its own width/height, resolution, origin and frame.
// In particular, Nav2's local costmap is a small 60x60 grid in `odom`, not a
// second 709x755 image in `map`.  Drawing it into the map rectangle (the old
// implementation) stretches it across the whole floor plan.  Use the grid's
// map-frame origin and pixel basis so each layer lands where RViz/Foxglove
// would place it.
function drawNavigationGrid(ctx, grid, image, mapMeta) {
  if (!grid || !image?.complete) return;
  const origin = grid.render_origin;
  if (!origin) return;
  const width = Number(grid.width || image.naturalWidth || 0);
  const height = Number(grid.height || image.naturalHeight || 0);
  const resolution = Number(grid.resolution || 0);
  if (!(width > 0 && height > 0 && resolution > 0)) return;
  const topLeft = navigationGridWorldPoint(origin, width, height, resolution, 0, 0);
  const oneX = navigationGridWorldPoint(origin, width, height, resolution, 1, 0);
  const oneY = navigationGridWorldPoint(origin, width, height, resolution, 0, 1);
  const p0 = navigationImagePoint(topLeft.x_m, topLeft.y_m, mapMeta);
  const px = navigationImagePoint(oneX.x_m, oneX.y_m, mapMeta);
  const py = navigationImagePoint(oneY.x_m, oneY.y_m, mapMeta);
  ctx.save();
  ctx.transform(
    px[0] - p0[0], px[1] - p0[1],
    py[0] - p0[0], py[1] - p0[1],
    p0[0], p0[1]
  );
  ctx.drawImage(image, 0, 0, width, height);
  ctx.restore();
}

function navigationGridWorldPoint(origin, width, height, resolution, pixelX, pixelY) {
  // The PNG is vertically flipped by the backend. Its top-left pixel is
  // therefore at the grid origin plus +Y*height, while image Y increases
  // toward -Y in the ROS frame.
  const yaw = Number(origin.yaw_rad || 0);
  const localX = Number(pixelX) * resolution;
  const localY = (Number(height) - Number(pixelY)) * resolution;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return {
    x_m: Number(origin.x_m || 0) + c * localX - s * localY,
    y_m: Number(origin.y_m || 0) + s * localX + c * localY
  };
}

function drawNavigationPlan(ctx, plan, meta, color, width, dashed) {
  const points = plan?.points || [];
  if (points.length < 2) return;
  // Path topics are volatile and retained in the backend only for diagnosis.
  // Do not present an old completed route as the currently executable route.
  // Nav2 Path publishers are VOLATILE and often publish only when the goal is
  // accepted. Keep that route visible while the goal is active; hide it only
  // after execution has ended so an old route is not mistaken for a live one.
  if (plan?.age_s != null && Number(plan.age_s) > 3.0 && !navigationMap?.active_goal) return;
  if (plan?.frame_id && String(plan.frame_id).replace(/^\//, "") !== String(meta.frame_id || "map").replace(/^\//, "")) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width / navigationView.scale;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.setLineDash(dashed ? [10 / navigationView.scale, 7 / navigationView.scale] : []);
  ctx.beginPath();
  points.forEach((point, index) => {
    const [x, y] = navigationImagePoint(point.x_m, point.y_m, meta);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
  const first = points[0];
  const last = points[points.length - 1];
  for (const [point, radius] of [[first, 6], [last, 7]]) {
    const [px, py] = navigationImagePoint(point.x_m, point.y_m, meta);
    ctx.beginPath();
    ctx.fillStyle = color;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2 / navigationView.scale;
    ctx.arc(px, py, radius / navigationView.scale, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

function drawNavigationRobot(ctx, pose, meta) {
  if (!pose || pose.x_m === undefined || pose.y_m === undefined) return;
  if (pose.frame_id && String(pose.frame_id).replace(/^\//, "") !== String(meta.frame_id || "map").replace(/^\//, "")) return;
  const [x, y] = navigationImagePoint(Number(pose.x_m), Number(pose.y_m), meta);
  const yaw = Number(pose.yaw_deg || 0) * Math.PI / 180;
  const res = Number(meta.resolution || 0.05);
  // R1 Pro footprint in map pixels, with only a small screen-space minimum
  // so it stays visible when the whole floor plan is fitted to the canvas.
  const bodyLength = Math.max(0.82 / res, 12 / navigationView.scale);
  const bodyWidth = Math.max(0.56 / res, 10 / navigationView.scale);
  const axis = Math.max(0.55 / res, 16 / navigationView.scale);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.fillStyle = "rgba(24, 74, 130, 0.88)";
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2 / navigationView.scale;
  ctx.beginPath();
  ctx.roundRect(-bodyLength * 0.5, -bodyWidth * 0.5, bodyLength, bodyWidth, bodyWidth * 0.16);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(bodyLength * 0.5, 0);
  ctx.lineTo(bodyLength * 0.22, -bodyWidth * 0.42);
  ctx.lineTo(bodyLength * 0.22, bodyWidth * 0.42);
  ctx.closePath();
  ctx.fillStyle = "#2f8dea";
  ctx.fill();
  ctx.stroke();
  // X axis (forward) and Y axis make the TF orientation unambiguous.
  ctx.lineWidth = 3 / navigationView.scale;
  ctx.strokeStyle = "#e74c3c";
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(axis, 0); ctx.stroke();
  ctx.strokeStyle = "#2ecc71";
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, -axis * 0.65); ctx.stroke();
  ctx.restore();
}

function drawNavigationPose(ctx, pose, meta, color, sizePx) {
  if (!pose || pose.x_m === undefined || pose.y_m === undefined) return;
  if (pose.frame_id && String(pose.frame_id).replace(/^\//, "") !== String(meta.frame_id || "map").replace(/^\//, "")) return;
  const [x, y] = navigationImagePoint(Number(pose.x_m), Number(pose.y_m), meta);
  const yaw = Number(pose.yaw_deg || 0) * Math.PI / 180;
  const size = sizePx / navigationView.scale;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.fillStyle = color;
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2 / navigationView.scale;
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.7, size * 0.55);
  ctx.lineTo(-size * 0.35, 0);
  ctx.lineTo(-size * 0.7, -size * 0.55);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function navigationImagePoint(x_m, y_m, meta) {
  const res = Number(meta.resolution || 1);
  const origin = meta.origin || {};
  const yaw = Number(origin.yaw_rad || 0);
  const dx = Number(x_m) - Number(origin.x_m || 0);
  const dy = Number(y_m) - Number(origin.y_m || 0);
  const gx = (Math.cos(yaw) * dx + Math.sin(yaw) * dy) / res;
  const gy = (-Math.sin(yaw) * dx + Math.cos(yaw) * dy) / res;
  return [gx, Number(meta.height || 0) - gy];
}

function navigationMapPointFromCanvas(event) {
  const canvas = $("navigationCanvas");
  const meta = navigationMeta();
  if (!canvas || !meta) return null;
  const rect = canvas.getBoundingClientRect();
  const ix = (event.clientX - rect.left - navigationView.panX) / navigationView.scale;
  const iy = (event.clientY - rect.top - navigationView.panY) / navigationView.scale;
  const res = Number(meta.resolution || 1);
  const origin = meta.origin || {};
  const yaw = Number(origin.yaw_rad || 0);
  const localX = ix * res;
  const localY = (Number(meta.height || 0) - iy) * res;
  return {
    x_m: Number(origin.x_m || 0) + Math.cos(yaw) * localX - Math.sin(yaw) * localY,
    y_m: Number(origin.y_m || 0) + Math.sin(yaw) * localX + Math.cos(yaw) * localY
  };
}

async function switchNavigationMode(mode) {
  if (!requireUnlocked("navigation")) return;
  stopChassisHold(false);
  try {
    const data = await post("/api/navigation/mode", {mode, timeout_s: 1.0}, {timeoutMs: 5000});
    if (data.navigation) state.navigation = data.navigation;
    renderNavigation();
    renderChassis();
    await refreshState();
  } catch (exc) {
    alert(exc.message || String(exc));
    await refreshState();
  }
}

function setupNavigationEvents() {
  $("navModeManual").onclick = () => switchNavigationMode("manual");
  $("navModeNavigation").onclick = () => switchNavigationMode("navigation");
  $("navStartLocalizationSpin").onclick = () => {
    if (!requireUnlocked("navigation")) return;
    if (!confirm("确认机器人周围没有人员和障碍物，并开始原地旋转一整圈定位？")) return;
    stopChassisHold(false);
    post("/api/navigation/localization-spin/start", {timeout_s: 1.0}, {timeoutMs: 5000})
      .then((data) => {
        if (data.navigation) state.navigation = data.navigation;
        renderNavigation();
        renderChassis();
        return refreshState();
      }).catch(alert);
  };
  $("navCancelLocalizationSpin").onclick = () => {
    if (!requireUnlocked("navigation")) return;
    post("/api/navigation/localization-spin/cancel", {timeout_s: 1.0}, {timeoutMs: 5000})
      .then(refreshState).catch(alert);
  };
  $("navRefreshMap").onclick = () => fetchNavigationMap(true);
  $("navClearGlobalCostmap").onclick = () => {
    if (!requireUnlocked("navigation")) return;
    if (!confirm("确认清除一次全局代价地图中的临时障碍物？")) return;
    post("/api/navigation/clear-global-costmap", {timeout_s: 2.0}, {timeoutMs: 5000})
      .then(async (data) => {
        if (data.message) alert(data.message);
        await refreshState();
        await fetchNavigationMap(true);
      })
      .catch(alert);
  };
  $("navCenterRobot").onclick = () => centerNavigationOnRobot();
  $("navSendGoal").onclick = () => {
    if (!requireUnlocked("navigation")) return;
    post("/api/navigation/goal", navigationGoalPayload()).then((data) => {
      navigationDraft = data.goal || navigationGoalPayload();
      refreshState();
    }).catch(alert);
  };
  $("navSaveGoalAction").onclick = () => saveActionFromPage(
    "navigation",
    "navigate_to_pose",
    "navigate_to_pose",
    "中文备注：开始时自动切换到导航控制，到达目标位姿后自动切回手动控制；失败、超时或停止也会尝试恢复手动控制。",
    navigationAtomicActionPayload()
  );
  $("navCancelGoal").onclick = () => requireUnlocked("navigation") && post("/api/navigation/cancel").then(refreshState).catch(alert);
  $("navEmergencyStop").onclick = () => {
    if (!requireUnlocked("navigation")) return;
    if (!confirm("确认执行导航急停？导航输出会关闭，活动目标会取消，且不会自动恢复。")) return;
    stopChassisHold(false);
    post("/api/navigation/emergency_stop", {timeout_s: 1.0}, {timeoutMs: 5000})
      .then(refreshState).catch(alert);
  };
  ["navShowGlobalCostmap", "navShowLocalCostmap", "navShowGlobalPlan", "navShowLocalPlan"].forEach((id) => {
    $(id).onchange = drawNavigationMap;
  });
  ["navGoalX", "navGoalY", "navGoalYaw"].forEach((id) => {
    $(id).oninput = () => setNavigationInputs(navigationGoalPayload(), true);
  });
  const canvas = $("navigationCanvas");
  canvas.addEventListener("pointerdown", (event) => {
    if (!state.navigation?.navigation_goal_allowed) return;
    const point = navigationMapPointFromCanvas(event);
    if (!point) return;
    navigationDragStart = point;
    navigationDraft = {...point, yaw_deg: Number($("navGoalYaw").value || 0)};
    setNavigationInputs(navigationDraft, false);
    canvas.setPointerCapture(event.pointerId);
    drawNavigationMap();
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!navigationDragStart) return;
    const point = navigationMapPointFromCanvas(event);
    if (!point) return;
    const yaw = Math.atan2(point.y_m - navigationDragStart.y_m, point.x_m - navigationDragStart.x_m) * 180 / Math.PI;
    navigationDraft = {...navigationDragStart, yaw_deg: yaw};
    setNavigationInputs(navigationDraft, false);
    drawNavigationMap();
  });
  ["pointerup", "pointercancel"].forEach((name) => {
    canvas.addEventListener(name, (event) => {
      if (navigationDragStart) {
        try {
          canvas.releasePointerCapture(event.pointerId);
        } catch {
          // Pointer capture may already be released by the browser.
        }
      }
      navigationDragStart = null;
    });
  });
}

function centerNavigationOnRobot() {
  const canvas = $("navigationCanvas");
  const meta = navigationMeta();
  const pose = navigationMap?.pose || state.navigation?.pose;
  if (!canvas || !meta || !pose) return;
  const [x, y] = navigationImagePoint(pose.x_m, pose.y_m, meta);
  navigationView.panX = canvas.clientWidth / 2 - x * navigationView.scale;
  navigationView.panY = canvas.clientHeight / 2 - y * navigationView.scale;
  navigationView.fit = false;
  drawNavigationMap();
}

function renderVision() {
  ensureVisionForms();
  renderHeadDepthProcessor();
  populateStreamSelects();
  renderDetections();
  renderStreamClicks();
  renderAruco();
  renderFoundation();
  renderRecognitionWorkflow();
  renderPageActions("vision", $("visionActionList"));
}

function headDepthProcessor() {
  const processor = state.head_depth?.processor;
  return processor && typeof processor === "object" ? processor : {};
}

function headDepthProcessorReady() {
  return headDepthProcessor().ready === true;
}

function headDepthPhaseView(processor) {
  const phase = String(processor.phase || "").trim().toLowerCase();
  const nonzeroExit = processor.exit_code !== null
    && processor.exit_code !== undefined
    && Number(processor.exit_code) !== 0;
  if (phase === "waiting_input" || phase === "input_interrupted"
    || /input.*(stall|stale|interrupt|timeout|lost|wait)|waiting.*input|stalled|starved|no[_ -]?input/.test(phase)) {
    return {label: "输入中断", tone: "error"};
  }
  if (/error|failed|crash/.test(phase) || (phase === "exited" && processor.requested === true)) {
    return {label: "错误", tone: "error"};
  }
  if (phase === "stopping" || phase === "stopped" || phase === "exited") {
    return {label: "关闭", tone: "off"};
  }
  if (processor.ready !== true && (processor.last_error || nonzeroExit)) {
    return {label: "错误", tone: "error"};
  }
  if (processor.ready === true) return {label: "运行", tone: "ready"};
  if (phase === "warming_up" || /warm/.test(phase)) return {label: "预热", tone: "loading"};
  if (phase === "starting" || phase === "loading_model" || /load|model/.test(phase)) {
    return {label: "加载模型", tone: "loading"};
  }
  if (processor.requested === true || processor.running === true) {
    return {label: processor.running === true ? "预热" : "加载模型", tone: "loading"};
  }
  return {label: "关闭", tone: "off"};
}

function headDepthMetric(parts, label, value, digits, suffix = "") {
  if (value === null || value === undefined || value === "") return;
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return;
  parts.push(`${label} ${numeric.toFixed(digits)}${suffix}`);
}

function headDepthLogTail(log) {
  const text = Array.isArray(log) ? log.map((line) => String(line ?? "")).join("\n") : String(log || "");
  const lines = text.split(/\r?\n/).map((line) => line.trim()).filter(Boolean);
  const tail = lines.length ? lines[lines.length - 1] : "";
  return tail.length > 240 ? `…${tail.slice(-240)}` : tail;
}

function renderHeadDepthProcessor() {
  const processor = headDepthProcessor();
  const checkbox = $("headDepthProcessorEnable");
  const status = $("headDepthProcessorStatus");
  const details = $("headDepthProcessorDetails");
  const calibration = $("headDepthCalibrationWarning");
  const view = headDepthPhaseView(processor);

  if (checkbox && !headDepthProcessorPending && document.activeElement !== checkbox) {
    checkbox.checked = processor.requested === true;
  }
  if (status) {
    status.textContent = view.label;
    status.className = `pill head-depth-state ${view.tone}`;
  }
  if (details) {
    const parts = [];
    if (processor.backend) parts.push(String(processor.backend));
    headDepthMetric(parts, "fps", processor.fps, 1);
    headDepthMetric(parts, "推理", processor.inference_ms, 1, " ms");
    headDepthMetric(parts, "丢帧", processor.dropped_pairs, 0);
    headDepthMetric(parts, "输出年龄", processor.output_age_s, 2, " s");
    if (processor.exit_code !== null && processor.exit_code !== undefined) {
      parts.push(`exit ${processor.exit_code}`);
    }
    if (processor.last_error) parts.push(String(processor.last_error));
    else if (view.tone === "error") {
      const logTail = headDepthLogTail(processor.log);
      if (logTail) parts.push(logTail);
    }
    details.textContent = parts.join(" · ") || (processor.requested ? "等待处理器状态" : "按需启动，不占用 GPU");
    details.classList.toggle("warn", view.tone === "error");
  }
  if (calibration) {
    const active = processor.requested === true || processor.running === true || processor.ready === true;
    calibration.textContent = active
      ? "标定待复核：使用头部 3D 坐标前，请现场复核双目标定和 head_left 到机器人坐标系的外参。"
      : "提示：头部相机标定仍待现场复核。";
    calibration.classList.toggle("warn", active);
  }
}

function headDepthCameraProblem(cameraKey) {
  if (cameraKey !== "head_left" || headDepthProcessorReady()) return "";
  return "头部相机深度处理尚未就绪，请打开开关并等待状态变为“运行”";
}

function yoloDetectionRunning() {
  if (!state.vision_worker?.running) return false;
  const configured = state.vision?.config?.enable_detection;
  if (typeof configured === "boolean") return configured;
  const statsEnabled = state.vision?.stats?.detection_enabled;
  return statsEnabled === true;
}

function recognitionDetections() {
  return Array.isArray(state.vision?.detections) ? state.vision.detections : [];
}

function recognitionSelectionDiffersFromWorkflow() {
  const workflow = state.recognition_workflow || {};
  const selectedAsset = String($("yoloAsset")?.value || "").trim();
  const selectedCamera = String($("yoloCamera")?.value || "").trim();
  return selectedAsset !== String(workflow.asset_id || "").trim()
    || selectedCamera !== String(workflow.camera_key || "").trim();
}

function recognitionActionControl(id) {
  return [
    "startYoloOnly", "stopYoloOnly",
    "startFullRecognition", "reregisterRecognition", "stopFullRecognition",
    "startArucoPose", "stopArucoPose", "startArucoFine", "stopArucoFine",
    "arucoFineEnable"
  ].includes(id);
}

function recognitionControlProblem(id) {
  if (!recognitionActionControl(id)) return "";
  if (recognitionRequestPending) return `正在${recognitionRequestPending}，请稍候`;
  const workflow = state.recognition_workflow || {};
  const recognitionActive = Boolean(workflow.active || workflow.busy);
  const yoloRunning = yoloDetectionRunning();
  const foundationRunning = Boolean(state.foundation?.running);
  const aruco = state.aruco || {};
  if (id === "startYoloOnly") {
    const prompt = String($("recognitionTargetPrompt")?.value || "").trim();
    return headDepthCameraProblem($("yoloCamera")?.value || "")
      || (!prompt ? "请输入目标提示词" : "")
      || (recognitionActive ? "完整识别正在运行，请先全部停止" : "")
      || (foundationRunning ? "FoundationPose 正在运行，请先全部停止" : "")
      || (yoloRunning ? "YOLO 已经运行" : "");
  }
  if (id === "stopYoloOnly") return yoloRunning ? "" : "YOLO 尚未运行";
  if (id === "startFullRecognition" || id === "reregisterRecognition") {
    const prompt = String($("recognitionTargetPrompt")?.value || "").trim();
    const assetId = String($("yoloAsset")?.value || "").trim();
    return headDepthCameraProblem($("yoloCamera")?.value || "")
      || (!prompt ? "请输入目标提示词" : "")
      || (!assetId ? "请等待自动匹配或选择候选资产" : "")
      || (id === "startFullRecognition" && workflow.busy ? "当前识别正在切换，请稍候" : "")
      || (id === "startFullRecognition" && recognitionActive && !recognitionSelectionDiffersFromWorkflow()
        ? "当前物体已在识别；选择其他资产后可直接切换"
        : "");
  }
  if (id === "stopFullRecognition") return recognitionActive ? "" : "完整识别尚未运行";
  if (id === "startArucoPose") return aruco.box_pose_running ? "ArUco 已在运行" : "";
  if (id === "stopArucoPose") return aruco.box_pose_running ? "" : "ArUco 尚未运行";
  if (id === "startArucoFine") return aruco.fine_running ? "微调节点已在运行" : "";
  if (id === "stopArucoFine") return aruco.fine_running ? "" : "微调节点尚未运行";
  return "";
}

function recognitionStatusIsError(value) {
  const text = String(value || "").trim();
  if (!text || /^not started$|^stopped$/i.test(text)) return false;
  return /error|failed|invalid|exception|traceback|unavailable|exited|cuda_oom|out of memory/i.test(text);
}

function recognitionBackendError() {
  const workflow = state.recognition_workflow || {};
  const yoloOnly = yoloDetectionRunning() && !workflow.active && !workflow.busy;
  if (workflow.recovering) return "";
  if (!yoloOnly && workflow.error) return String(workflow.error);
  const visionStatus = String(state.vision?.status || "");
  if (!/^click\b/i.test(visionStatus) && recognitionStatusIsError(visionStatus)) {
    return `YOLO: ${visionStatus}`;
  }
  if (yoloOnly) return "";
  const foundationStatus = String(state.foundation?.status || "");
  if (recognitionStatusIsError(foundationStatus)) return `FoundationPose: ${foundationStatus}`;
  const debugStatus = state.foundation?.debug?.status;
  if (recognitionStatusIsError(debugStatus)) return `FoundationPose: ${debugStatus}`;
  return "";
}

async function runRecognitionRequest(label, request, onSuccess) {
  if (recognitionRequestPending) return;
  let succeeded = false;
  recognitionRequestPending = label;
  recognitionUiError = "";
  recognitionUiMessage = "";
  renderRecognitionWorkflow();
  renderLocks();
  try {
    const result = await request();
    if (onSuccess) onSuccess(result);
    recognitionUiMessage = result?.message || `${label}完成`;
    succeeded = true;
  } catch (exc) {
    recognitionUiError = `${label}失败：${exc?.message || String(exc)}`;
  } finally {
    await refreshState();
    recognitionRequestPending = "";
    renderRecognitionWorkflow();
    syncDerivedVisionStreams();
    renderLocks();
  }
  return succeeded;
}

function runRecognitionControl(id, label, request, onSuccess) {
  if (!requireUnlocked("recognition")) return;
  const problem = recognitionControlProblem(id);
  if (problem) {
    recognitionUiMessage = "";
    recognitionUiError = problem;
    renderRecognitionWorkflow();
    renderLocks();
    return;
  }
  void runRecognitionRequest(label, request, onSuccess);
}

function setRecognitionStep(stepId, statusId, text, tone) {
  const step = $(stepId);
  const status = $(statusId);
  if (status) status.textContent = text;
  if (step) step.className = `recognition-step ${tone || "idle"}`;
}

function recognitionLogText() {
  const stats = state.vision?.stats || {};
  const debug = state.foundation?.debug || {};
  const latched = state.latched_yolo || {};
  const workflow = state.recognition_workflow || {};
  const lines = [
    `workflow: ${workflow.phase || "idle"} | ${workflow.status || "-"}`,
    `target/asset: ${workflow.target_prompt || $("recognitionTargetPrompt")?.value || "-"} / ${workflow.asset_id || $("yoloAsset")?.value || "-"}`,
    `GPU handoff: YOLO=${workflow.yolo_running ? "running" : "stopped"}, FoundationPose=${workflow.foundation_running ? "running" : "stopped"}`,
    `YOLO: ${state.vision?.status || "not started"}`,
    `YOLO worker: ${yoloDetectionRunning() ? "running" : "stopped"}`,
    `YOLO stats: input=${stats.input_fps ?? "-"} fps, depth=${stats.depth_fps ?? "-"} fps, inference=${stats.inference_fps ?? "-"} fps`,
    `detections: ${recognitionDetections().length}`,
    `asset appearance: ${workflow.latched_asset_match_score ?? Math.max(...recognitionDetections().map((item) => Number(item.asset_match_score) || 0), 0)} (minimum ${state.vision?.config?.asset_match_min_score ?? "-"})`,
    `latched: ${latched.class_name || "-"} ${latched.confidence ?? ""}`.trimEnd(),
    `latched camera/generation: ${latched.camera_key || "-"} / ${latched.config_id ?? "-"} / ${latched.worker_session_id ?? "-"}`,
    `mask: ${latched.mask_file || state.vision?.latest_mask_file || "-"} (${latched.mask_frozen ? "frozen" : "not frozen"})`,
    `FoundationPose: ${state.foundation?.status || "not started"}`,
    `Foundation backend/asset: ${debug.backend || state.foundation?.backend || "-"} / ${debug.asset_id || state.foundation?.asset_id || "-"}`,
    `Foundation mode: ${state.foundation?.mode || debug.mode || "waiting"}`,
    `Foundation inference: ${debug.inference_ms ?? state.foundation?.inference_ms ?? "-"} ms; register=${debug.registration_count ?? 0}, track=${debug.tracking_count ?? 0}, dropped=${debug.latest_only_dropped_sets ?? 0}`,
    `Foundation CUDA: ${debug.cuda ? JSON.stringify(debug.cuda) : "-"}`,
    `Foundation debug: ${Object.keys(debug).length ? JSON.stringify(debug) : "-"}`,
    `ArUco: ${state.aruco?.status || "not started"}`
  ];
  return lines.join("\n");
}

function recognitionPhaseLabel(phase) {
  return ({
    idle: "未启动",
    stopped: "已停止",
    starting_yolo: "启动 YOLO",
    detecting: "YOLO 检测目标",
    gpu_handoff: "锁存完成，释放 YOLO GPU",
    loading_foundation: "加载 FoundationPose",
    registering: "FoundationPose 首次注册",
    tracking: "FoundationPose 实时跟踪",
    retrying: "跟踪丢失，自动重试",
    error: "识别失败"
  })[phase] || phase || "未启动";
}

function renderRecognitionPose() {
  const foundation = state.foundation || {};
  const pose = recognitionPoseFrame === "camera" ? (foundation.camera_pose || {}) : (foundation.object_pose || {});
  const values = $("recognitionPoseValues");
  const meta = $("recognitionPoseMeta");
  const baseTab = $("recognitionBasePoseTab");
  const cameraTab = $("recognitionCameraPoseTab");
  baseTab?.classList.toggle("active", recognitionPoseFrame === "base");
  cameraTab?.classList.toggle("active", recognitionPoseFrame === "camera");
  if (state.recognition_workflow?.recovering || !Array.isArray(pose.position) || pose.position.length < 3) {
    if (values) values.textContent = "暂无有效位姿";
    if (meta) meta.textContent = state.recognition_workflow?.recovering
      ? "跟踪丢失，旧位姿无效；等待重新注册"
      : recognitionPoseFrame === "base" ? "等待 base_link TF 与实时跟踪结果" : "等待相机坐标位姿";
    return;
  }
  const xyz = pose.position.slice(0, 3);
  if (values) {
    values.innerHTML = xyz.map((value, index) => `<span><small>${["x", "y", "z"][index]}</small>${escapeHtml(number(value, 4))}</span>`).join("");
  }
  const age = pose.age_s === null || pose.age_s === undefined ? "-" : `${number(pose.age_s, 2)} s`;
  if (meta) meta.textContent = `${pose.frame_id || (recognitionPoseFrame === "base" ? "base_link" : "camera")} · 数据年龄 ${age}`;
}

function renderRecognitionWorkflow() {
  const workflow = state.recognition_workflow || {};
  const phase = String(workflow.phase || "idle");
  const yoloOnly = yoloDetectionRunning() && !workflow.active && !workflow.busy;
  const debug = state.foundation?.debug || {};
  const cuda = debug.cuda || state.foundation?.cuda || {};
  const pose = state.foundation?.object_pose || state.foundation?.camera_pose || {};
  const age = pose.age_s ?? workflow.latched_age_s;
  const peak = cuda.max_memory_reserved_mib ?? cuda.max_memory_allocated_mib;
  const appearanceScores = recognitionDetections()
    .map((item) => Number(item.asset_match_score))
    .filter(Number.isFinite);
  const appearance = workflow.latched_asset_match_score
    ?? (appearanceScores.length ? Math.max(...appearanceScores) : undefined);
  const stage = $("recognitionStage");
  const retryRemaining = workflow.recovery_remaining_s;
  if (stage) stage.textContent = yoloOnly
    ? `YOLO 单独运行 · ${state.vision?.status || "running"}`
    : `${workflow.status || recognitionPhaseLabel(phase)}${workflow.recovering && retryRemaining !== null && retryRemaining !== undefined ? ` · 剩余 ${number(retryRemaining, 1)} 秒` : ""}`;
  const dot = $("recognitionStateDot");
  if (dot) dot.className = `recognition-state-dot ${yoloOnly || phase === "tracking" ? "ready" : phase === "error" ? "error" : (workflow.busy ? "busy" : "")}`;
  const perf = $("recognitionPerformance");
  if (perf) {
    perf.textContent = `数据 ${age === null || age === undefined ? "-" : `${number(age, 2)}s`} · 外观 ${appearance === undefined ? "-" : number(appearance, 3)} · 推理 ${debug.inference_ms === undefined ? "-" : `${number(debug.inference_ms, 1)}ms`} · CUDA峰值 ${peak === undefined ? "-" : `${number(Number(peak) / 1024, 2)}GiB`}`;
  }
  renderRecognitionPose();

  const selectedAsset = recognitionAssets.find((asset) => String(asset.id) === String($("yoloAsset")?.value || ""));
  const actualPrompts = workflow.yolo_prompts?.length ? workflow.yolo_prompts : (selectedAsset?.detection_prompts || []);
  if ($("recognitionActualPrompt")) $("recognitionActualPrompt").textContent = `YOLO：${actualPrompts.join(", ") || "-"}`;
  if ($("recognitionAssetStatus")) {
    const matchStatus = recognitionMatchResult?.status;
    $("recognitionAssetStatus").textContent = selectedAsset
      ? `资产：${selectedAsset.name || selectedAsset.id} (${selectedAsset.id})`
      : matchStatus === "ambiguous"
        ? "匹配到多个候选，请选择资产"
        : matchStatus === "not_found" || matchStatus === "low_confidence"
          ? "未找到可靠资产，请修改提示词或手动选择"
          : "输入目标后自动匹配资产";
  }

  let guidance = yoloOnly
    ? "YOLO 正在单独运行；停止 YOLO 后可释放其 GPU"
    : workflow.status || "输入目标提示词，确认资产后开始识别";
  if (recognitionRequestPending) guidance = `正在${recognitionRequestPending}...`;
  else if (recognitionUiMessage) guidance = `${recognitionUiMessage} ${guidance}`.trim();
  const gate = $("recognitionGateMessage");
  if (gate) gate.textContent = guidance;
  const visibleError = recognitionUiError || recognitionBackendError();
  const error = $("recognitionError");
  if (error) error.textContent = visibleError;
  const log = $("recognitionLog");
  if (log) log.textContent = recognitionLogText();

  if ($("startYoloOnly")) $("startYoloOnly").textContent = recognitionRequestPending === "启动 YOLO" ? "启动中..." : "仅启动 YOLO";
  if ($("stopYoloOnly")) $("stopYoloOnly").textContent = recognitionRequestPending === "停止 YOLO" ? "停止中..." : "停止 YOLO";
  if ($("startFullRecognition")) {
    const switching = Boolean(workflow.active) && recognitionSelectionDiffersFromWorkflow();
    $("startFullRecognition").textContent = recognitionRequestPending === "切换识别"
      ? "切换中..."
      : recognitionRequestPending === "启动完整识别"
        ? "启动中..."
        : switching ? "切换并识别" : "开始识别";
  }
  if ($("reregisterRecognition")) $("reregisterRecognition").textContent = recognitionRequestPending === "重新注册" ? "处理中..." : "重新注册";
  if ($("stopFullRecognition")) $("stopFullRecognition").textContent = recognitionRequestPending === "停止完整识别" ? "停止中..." : "停止";
}

function recognitionConfigText(value) {
  if (Array.isArray(value)) return value.join(", ");
  return value === undefined || value === null ? "" : String(value);
}

function setRecognitionFormValue(id, value) {
  if (value === undefined || value === null) return;
  const input = $(id);
  if (!input) return;
  const text = recognitionConfigText(value);
  if (input.tagName === "SELECT" && ![...input.options].some((option) => option.value === text)) return;
  input.value = text;
}

function syncRecognitionFormsFromState() {
  const workflow = state.recognition_workflow || {};
  if (workflow.active || workflow.busy) {
    const key = JSON.stringify([workflow.generation, workflow.camera_key, workflow.target_prompt, workflow.asset_id]);
    if (key !== recognitionYoloConfigSyncKey) {
      recognitionYoloConfigSyncKey = key;
      setRecognitionFormValue("yoloCamera", workflow.camera_key);
      setRecognitionFormValue("recognitionTargetPrompt", workflow.target_prompt);
      setRecognitionFormValue("yoloAsset", workflow.asset_id);
    }
  } else {
    recognitionYoloConfigSyncKey = "";
  }
}

function scheduleRecognitionAssetMatch() {
  if (recognitionMatchTimer) clearTimeout(recognitionMatchTimer);
  recognitionMatchTimer = setTimeout(matchRecognitionTarget, 350);
}

async function matchRecognitionTarget() {
  const prompt = String($("recognitionTargetPrompt")?.value || "").trim();
  const sequence = ++recognitionMatchSequence;
  if (!prompt) {
    recognitionMatchResult = null;
    if ($("yoloAsset")) $("yoloAsset").value = "";
    renderRecognitionWorkflow();
    renderLocks();
    return;
  }
  try {
    const result = await post("/api/assets/match", {prompt, limit: 8});
    if (sequence !== recognitionMatchSequence) return;
    recognitionMatchResult = result;
    const matchedId = result.matched && result.asset ? String(result.asset.id || "") : "";
    if ($("yoloAsset")) {
      if (matchedId && [...$("yoloAsset").options].some((option) => option.value === matchedId)) {
        $("yoloAsset").value = matchedId;
      } else if (!result.candidates?.some((item) => String(item.asset?.id || "") === $("yoloAsset").value)) {
        $("yoloAsset").value = "";
      }
    }
    applyRecognitionAsset($("yoloAsset")?.value || "");
  } catch (exc) {
    if (sequence !== recognitionMatchSequence) return;
    recognitionMatchResult = {status: "error"};
    recognitionUiError = `资产匹配失败：${exc?.message || String(exc)}`;
  }
  renderRecognitionWorkflow();
  renderLocks();
}

function bindRecognitionFormEvents() {
  const yoloCamera = $("yoloCamera");
  if (yoloCamera && !yoloCamera.dataset.recognitionBound) {
    yoloCamera.dataset.recognitionBound = "1";
    yoloCamera.addEventListener("change", () => {
      recognitionUiError = "";
      renderRecognitionWorkflow();
      renderLocks();
    });
  }
  const prompt = $("recognitionTargetPrompt");
  if (prompt && !prompt.dataset.recognitionBound) {
    prompt.dataset.recognitionBound = "1";
    prompt.addEventListener("input", () => {
      recognitionUiError = "";
      recognitionUiMessage = "";
      scheduleRecognitionAssetMatch();
      renderRecognitionWorkflow();
      renderLocks();
    });
  }
  const assetSelect = $("yoloAsset");
  if (assetSelect && !assetSelect.dataset.recognitionBound) {
    assetSelect.dataset.recognitionBound = "1";
    assetSelect.addEventListener("change", () => {
      applyRecognitionAsset(assetSelect.value);
      recognitionUiError = "";
      renderRecognitionWorkflow();
      renderLocks();
    });
  }
}

function ensureVisionForms() {
  populateCameraSelect($("yoloCamera"));
  if ($("yoloModel") && !$("yoloModel").value) {
    $("yoloModel").value = state.config?.camera_model_path || "/home/motrix/galaxea/galaxea_overlay/src/wrist_yoloe_segmentation/model/yoloe-26s-seg.pt";
  }
  populateRecognitionAssetSelect();
  ensureRecognitionAssetsLoaded();
  syncRecognitionFormsFromState();
  bindRecognitionFormEvents();
}

function ensureRecognitionAssetsLoaded() {
  if (recognitionAssetsLoading || recognitionAssets.length) return;
  recognitionAssetsLoading = true;
  fetchJson("/api/assets/list")
    .then((result) => {
      recognitionAssets = Array.isArray(result.assets)
        ? result.assets.filter((asset) => asset?.enabled)
        : [];
      populateRecognitionAssetSelect();
      syncRecognitionFormsFromState();
      const select = $("yoloAsset");
      const matchedId = recognitionMatchResult?.matched
        ? String(recognitionMatchResult.asset?.id || "")
        : "";
      if (select && !select.value && matchedId && [...select.options].some((option) => option.value === matchedId)) {
        select.value = matchedId;
      }
      if (select?.value) applyRecognitionAsset(select.value);
      renderRecognitionWorkflow();
      renderLocks();
    })
    .catch((exc) => {
      recognitionUiError = `资产目录加载失败：${exc?.message || String(exc)}`;
      renderRecognitionWorkflow();
    })
    .finally(() => {
      recognitionAssetsLoading = false;
    });
}

function populateRecognitionAssetSelect() {
  const select = $("yoloAsset");
  if (!select) return;
  const current = select.value;
  select.innerHTML = '<option value="">请选择资产</option>';
  recognitionAssets.forEach((asset) => {
    const option = document.createElement("option");
    option.value = String(asset.id || "");
    option.textContent = `${asset.name || asset.id} (${asset.id})`;
    select.appendChild(option);
  });
  if ([...select.options].some((option) => option.value === current)) {
    select.value = current;
  }
}

function applyRecognitionAsset(assetId) {
  const asset = recognitionAssets.find((item) => String(item.id) === String(assetId));
  if (!asset) {
    renderRecognitionWorkflow();
    return;
  }
  const prompts = Array.isArray(asset.detection_prompts) ? asset.detection_prompts : [];
  recognitionUiMessage = `已选择资产 ${asset.name || asset.id}`;
  if ($("recognitionActualPrompt")) $("recognitionActualPrompt").textContent = `YOLO：${prompts.join(", ") || "-"}`;
}

function populateCameraSelect(select) {
  if (!select) return;
  const current = select.value;
  select.innerHTML = "";
  Object.entries(state.cameras || {}).forEach(([key, camera]) => {
    const option = document.createElement("option");
    option.value = key;
    option.textContent = camera.label || key;
    select.appendChild(option);
  });
  if ([...select.options].some((opt) => opt.value === current)) select.value = current;
}

function yoloPayload() {
  return {
    camera_key: $("yoloCamera").value,
    model_path: $("yoloModel").value,
    confidence_threshold: Number($("yoloConf").value),
    image_size: Number($("yoloImageSize").value),
    max_detections: Number($("yoloMaxDet").value),
    asset_id: $("yoloAsset")?.value || "",
    device: $("yoloDevice").value,
    load_model: true
  };
}

function standaloneYoloPayload() {
  const payload = yoloPayload();
  const selectedAsset = recognitionAssets.find(
    (asset) => String(asset.id) === String(payload.asset_id || "")
  );
  const assetPrompts = Array.isArray(selectedAsset?.detection_prompts)
    ? selectedAsset.detection_prompts.map((value) => String(value || "").trim()).filter(Boolean)
    : [];
  const rawPrompt = String($("recognitionTargetPrompt")?.value || "").trim();
  return {
    ...payload,
    prompt: assetPrompts.length ? assetPrompts : rawPrompt,
    clean_restart: true
  };
}

function fullRecognitionPayload() {
  return {
    ...yoloPayload(),
    target_prompt: String($("recognitionTargetPrompt")?.value || "").trim(),
    asset_id: $("yoloAsset")?.value || "",
    detection_timeout_s: 30,
    registration_timeout_s: 90
  };
}

function streamSlots() {
  return Array.from({length: STREAM_SLOT_COUNT}, (_, index) => index);
}

function populateStreamSelects() {
  const options = state.camera_streams || [];
  streamSlots().forEach((i) => {
    const select = $(`streamSelect${i}`);
    if (!select) return;
    const current = streamSelections[i] || select.value;
    select.innerHTML = "";
    options.forEach((item) => {
      const opt = document.createElement("option");
      opt.value = item.key;
      opt.textContent = item.label;
      select.appendChild(opt);
    });
    if (options.some((item) => item.key === current)) select.value = current;
  });
  syncVisionStreams();
}

function connectCameraImageStream(img, key) {
  if (!img || !key) return;
  clearTimeout(img._cameraStreamRetry);
  const source = `/stream/${key}?t=${Date.now()}`;
  img.onerror = () => {
    clearTimeout(img._cameraStreamRetry);
    img._cameraStreamRetry = setTimeout(() => {
      if (img.getAttribute("src") === source && img.isConnected) {
        connectCameraImageStream(img, key);
      }
    }, 2000);
  };
  img.src = source;
}

function setVisionStream(slot, key) {
  const img = $(`streamImage${slot}`);
  const empty = $(`streamEmpty${slot}`);
  if (!img) return;
  const current = img.dataset.streamKey || "";
  if (!key) {
    if (current || img.getAttribute("src")) {
      img.dataset.streamKey = "";
      img.removeAttribute("src");
    }
    img.style.display = "none";
    if (empty) empty.style.display = "block";
    return;
  }
  if (current === key) return;
  img.dataset.streamKey = key;
  img.style.display = "block";
  connectCameraImageStream(img, key);
  if (empty) empty.style.display = "none";
}

function syncVisionStreams() {
  streamSlots().forEach((slot) => {
    updateStreamTitle(slot);
    const key = streamActive[slot] && activePage() === "camera" ? streamSelections[slot] : "";
    setVisionStream(slot, key);
  });
  syncDerivedVisionStreams();
}

function selectedStreamKey(slot) {
  return $(`streamSelect${slot}`)?.value || streamSelections[slot] || "";
}

function streamLabel(key) {
  const label = (state.camera_streams || []).find((item) => item.key === key)?.label || key;
  return label || "未订阅";
}

function streamStatsText(key) {
  if (!key) return "";
  const hz = state.image_hz?.[key];
  const age = state.images?.[key];
  const resolution = streamResolutionText(key);
  const parts = [];
  if (resolution) parts.push(resolution);
  if (hz !== undefined) parts.push(`${number(hz, 1)} fps`);
  if (age !== undefined) parts.push(`age ${number(age, 1)}s`);
  return parts.length ? ` | ${parts.join(" | ")}` : "";
}

function streamResolutionText(key) {
  const meta = state.image_meta?.[key];
  if (!meta?.width || !meta?.height) return "";
  return `${meta.width}x${meta.height}`;
}

function updateStreamTitle(slot) {
  const key = streamSelections[slot];
  const base = streamActive[slot] ? streamLabel(key) : "未订阅";
  const stats = key ? streamStatsText(key) : "";
  $(`streamTitle${slot}`).textContent = `${base}${stats}`;
}

function setStreamSelection(slot, key) {
  streamSelections[slot] = key;
  streamClickModes[slot] = false;
  streamClickMarkers[slot] = null;
  updateStreamTitle(slot);
  renderStreamClick(slot);
}

function defaultSnapshotPath() {
  return localStorage.getItem("r1pro.snapshotPath") || "~/Pictures/r1pro_web_control";
}

function initStreamSavePath(slot) {
  const input = $(`streamSavePath${slot}`);
  if (!input || input.dataset.ready) return;
  input.dataset.ready = "1";
  input.value = localStorage.getItem(`r1pro.snapshotPath.${slot}`) || defaultSnapshotPath();
  input.onchange = () => {
    localStorage.setItem(`r1pro.snapshotPath.${slot}`, input.value.trim());
    localStorage.setItem("r1pro.snapshotPath", input.value.trim());
  };
}

function setStreamCaptureStatus(slot, text, ok = true) {
  streamCaptureStatus[slot] = text || "";
  const el = $(`streamCaptureStatus${slot}`);
  if (!el) return;
  el.textContent = streamCaptureStatus[slot];
  el.classList.toggle("warn", !ok);
}

async function captureStream(slot) {
  if (!requireUnlocked("camera")) return;
  if (!streamActive[slot]) {
    setStreamCaptureStatus(slot, "请先订阅这个视频流", false);
    return;
  }
  const key = streamSelections[slot] || selectedStreamKey(slot);
  if (!key) {
    setStreamCaptureStatus(slot, "未选择视频流", false);
    return;
  }
  const path = ($(`streamSavePath${slot}`)?.value || defaultSnapshotPath()).trim();
  localStorage.setItem(`r1pro.snapshotPath.${slot}`, path);
  localStorage.setItem("r1pro.snapshotPath", path);
  setStreamCaptureStatus(slot, "正在保存...");
  try {
    const result = await post("/api/vision/save_snapshot", {stream_key: key, path}, {timeoutMs: 15000});
    const sizeKb = result.size_bytes ? `${number(result.size_bytes / 1024, 1)} KB` : "";
    const resolution = result.width && result.height ? `${result.width}x${result.height}` : "";
    const detail = [resolution, sizeKb].filter(Boolean).join(" | ");
    setStreamCaptureStatus(slot, `已保存: ${result.path}${detail ? ` (${detail})` : ""}`);
  } catch (exc) {
    setStreamCaptureStatus(slot, exc.message || String(exc), false);
  }
}

function connectStream(slot) {
  const key = selectedStreamKey(slot);
  if (!key) return;
  streamSelections[slot] = key;
  streamActive[slot] = true;
  updateStreamTitle(slot);
  setVisionStream(slot, activePage() === "camera" ? key : "");
  renderStreamClick(slot);
}

function disconnectStream(slot) {
  streamActive[slot] = false;
  streamClickModes[slot] = false;
  streamClickMarkers[slot] = null;
  updateStreamTitle(slot);
  setVisionStream(slot, "");
  renderStreamClick(slot);
}

async function toggleStreamClick(slot) {
  if (!requireUnlocked("camera")) return;
  const key = streamSelections[slot] || selectedStreamKey(slot);
  const cameraKey = streamClickCameraKey(key);
  const cameraProblem = streamClickCameraProblem(cameraKey);
  if (!streamActive[slot]) {
    setStreamCaptureStatus(slot, "请先订阅这个视频流", false);
    return;
  }
  if (cameraProblem) {
    setStreamCaptureStatus(slot, cameraProblem, false);
    return;
  }
  streamClickModes[slot] = !streamClickModes[slot];
  streamClickActiveSlot = slot;
  renderStreamClick(slot);
  if (!streamClickModes[slot] || streamClickConfiguredCamera === cameraKey) return;
  try {
    await post("/api/vision/configure", {camera_key: cameraKey});
    streamClickConfiguredCamera = cameraKey;
  } catch (exc) {
    streamClickModes[slot] = false;
    renderStreamClick(slot);
    alert(exc.message || String(exc));
  }
}

function setDerivedStream(imgId, emptyId, key, enabled, emptyText) {
  const img = $(imgId);
  const empty = $(emptyId);
  if (!img) return;
  if (!enabled || activePage() !== "recognition") {
    img.dataset.streamKey = "";
    img.removeAttribute("src");
    img.style.display = "none";
    if (empty) {
      empty.textContent = emptyText;
      empty.style.display = "block";
    }
    return;
  }
  if (img.dataset.streamKey !== key) {
    img.dataset.streamKey = key;
    img.style.display = "block";
    connectCameraImageStream(img, key);
  }
  if (empty) empty.style.display = "none";
}

function syncDerivedVisionStreams() {
  const foundationRunning = Boolean(state.foundation?.running);
  const yoloRunning = yoloDetectionRunning();
  setDerivedStream(
    "recognitionMainImage",
    "recognitionMainEmpty",
    state.recognition_workflow?.recovering
      ? (yoloRunning ? "yolo" : (state.recognition_workflow.camera_key === "head_left"
        ? "head_left_raw_rgb" : `${state.recognition_workflow.camera_key || "wrist_right"}_rgb`))
      : foundationRunning ? "foundation" : "yolo",
    foundationRunning || yoloRunning || Boolean(state.recognition_workflow?.recovering),
    state.recognition_workflow?.busy ? "识别启动中，等待实时画面" : "识别未启动"
  );
  setDerivedStream(
    "arucoImage",
    "arucoStreamEmpty",
    "aruco",
    Boolean(state.aruco?.box_pose_running),
    "ArUco 识别关闭，未订阅视频流"
  );
}

function renderDetections() {
  const detections = state.vision?.detections || [];
  summary($("detectionSummary"), detections.slice(0, 8).map((det) => ({
    label: `${det.class_name || det.class_id} ${number(det.confidence, 2)}${det.mask_selected ? " mask" : ""}`,
    value: `${det.robot_xyz_m ? det.robot_xyz_m.map((v) => number(v, 3)).join(", ") : det.position_status || "-"}${det.asset_match_score === undefined ? "" : ` | 外观 ${number(det.asset_match_score, 3)} ${det.asset_match_status || ""}`}`
  })).concat([
    {label: "preview", value: `${streamResolutionText("yolo") || "-"} | ${state.image_hz?.yolo !== undefined ? `${number(state.image_hz.yolo, 1)} fps` : "-"}`}
  ]));
}

function xyzText(value) {
  return Array.isArray(value) && value.length >= 3 ? value.slice(0, 3).map((v) => number(v, 3)).join(", ") : "-";
}

function renderAruco() {
  const aruco = state.aruco || {};
  const pose = aruco.pose || {};
  const position = pose.position || [];
  const targetX = Number(aruco.target_x_m ?? state.config?.aruco_fine_target_x_m ?? 0.616);
  const targetY = Number(aruco.target_y_m ?? state.config?.aruco_fine_target_y_m ?? 0.007);
  const errorX = Array.isArray(position) && position.length >= 2 ? Number(position[0]) - targetX : null;
  const errorY = Array.isArray(position) && position.length >= 2 ? Number(position[1]) - targetY : null;
  const enable = $("arucoFineEnable");
  if (enable && document.activeElement !== enable) {
    enable.checked = Boolean(aruco.fine_enabled);
  }
  summary($("arucoSummary"), [
    {label: "pose node", value: aruco.box_pose_running ? "running" : "stopped"},
    {label: "fine node", value: aruco.fine_running ? "running" : "stopped"},
    {label: "fine", value: `${aruco.fine_enabled ? "enabled" : "disabled"} | ${aruco.fine_status || "-"}`},
    {label: "in tolerance", value: aruco.fine_in_tolerance ? "yes" : "no"},
    {label: "pose xyz", value: xyzText(position)},
    {label: "xy error", value: errorX === null ? "-" : `${number(errorX, 3)}, ${number(errorY, 3)}`},
    {label: "pose age", value: pose.age_s === null || pose.age_s === undefined ? "-" : `${number(pose.age_s, 2)}s`},
    {label: "preview", value: `${streamResolutionText("aruco") || "-"} | ${state.image_hz?.aruco !== undefined ? `${number(state.image_hz.aruco, 1)} fps` : "-"}`}
  ]);
}

function renderStreamClicks() {
  streamSlots().forEach((slot) => renderStreamClick(slot));
}

function streamClickCameraKey(streamKey) {
  const item = (state.camera_streams || []).find((entry) => entry.key === streamKey);
  if (item?.type && item.type !== "rgb") return "";
  if (streamKey === "head_left_raw_rgb" || streamKey === "head_right_rgb") return "";
  if (streamKey === "head_rgb" || streamKey === "head_left_rgb") return "head_left";
  const match = String(streamKey || "").match(/^(wrist_left|wrist_right)_rgb$/);
  return match ? match[1] : "";
}

function streamClickCameraProblem(cameraKey) {
  if (!cameraKey) return "当前视频流不能直接取 3D 点";
  const headDepthProblem = headDepthCameraProblem(cameraKey);
  if (headDepthProblem) return headDepthProblem;
  const camera = state.cameras?.[cameraKey];
  if (camera && !camera.camera_info_topic) return "这个相机没有 camera_info";
  return "";
}

function streamClickStatusText(result, slot, cameraKey) {
  if (!streamActive[slot]) return "请先订阅这个视频流";
  const cameraProblem = streamClickCameraProblem(cameraKey);
  if (cameraProblem) return cameraProblem;
  const modeText = streamClickModes[slot] ? "已打开，点击相机画面取点" : "已关闭";
  const pendingText = streamClickPending[slot] ? "，等待坐标..." : "";
  const depthAge = result?.depth_age_s === null || result?.depth_age_s === undefined ? "" : `，depth age ${number(result.depth_age_s, 2)}s`;
  return `${modeText}${pendingText}${depthAge}`;
}

function renderStreamClick(slot) {
  const button = $(`toggleStreamClick${slot}`);
  const viewer = $(`streamViewer${slot}`);
  const status = $(`streamClickStatus${slot}`);
  const result = state.vision?.click_result || {};
  const key = streamSelections[slot] || selectedStreamKey(slot);
  const cameraKey = streamClickCameraKey(key);
  const cameraProblem = streamClickCameraProblem(cameraKey);
  if (cameraProblem && streamClickModes[slot]) {
    streamClickModes[slot] = false;
    streamClickPending[slot] = false;
    streamClickMarkers[slot] = null;
  }
  if (button) {
    button.textContent = streamClickModes[slot] ? "关闭取点" : "点击取点";
    button.classList.toggle("active", streamClickModes[slot]);
    const disabled = !locks.camera || !streamActive[slot] || Boolean(cameraProblem);
    button.disabled = disabled;
    button.classList.toggle("disabled", disabled);
  }
  if (viewer) viewer.classList.toggle("click-mode", streamClickModes[slot] && !cameraProblem);
  if (status) {
    const resultMatchesSlot = Number(result.slot) === slot || result.display_mode === key;
    status.textContent = streamClickStatusText(resultMatchesSlot ? result : {}, slot, cameraKey);
    status.classList.toggle("warn", Boolean(resultMatchesSlot && result.status && !String(result.status).startsWith("ok")));
  }

  const items = [];
  const resultMatchesSlot = Number(result.slot) === slot || result.display_mode === key;
  if (resultMatchesSlot && result.point_px) {
    items.push(
      {label: "图像坐标 px", value: result.point_px.map((v) => number(v, 1)).join(", ")},
      {label: "深度坐标 px", value: Array.isArray(result.depth_point_px) ? result.depth_point_px.map((v) => number(v, 1)).join(", ") : "-"},
      {label: "深度 m", value: result.depth_m === null || result.depth_m === undefined ? "-" : number(result.depth_m, 4)},
      {label: "相机坐标 m", value: xyzText(result.camera_xyz_m)},
      {label: `${result.robot_frame || "base_link"} 坐标 m`, value: xyzText(result.robot_xyz_m)},
      {label: "状态", value: result.status || "-"},
      {label: "相机 frame", value: result.camera_frame || result.camera_info_frame || "-"}
    );
  } else {
    items.push({label: "点击取点", value: cameraProblem || (streamClickModes[slot] ? "点击画面显示坐标" : "打开后可在画面上取点")});
  }
  summary($(`streamClickSummary${slot}`), items);
  updateStreamClickMarker(slot, resultMatchesSlot ? result : {});
}

function imageEventPayload(event, displayMode = "") {
  const img = event.currentTarget;
  const rect = img.getBoundingClientRect();
  if (!rect.width || !rect.height) return null;
  const naturalWidth = img.naturalWidth || 0;
  const naturalHeight = img.naturalHeight || 0;
  if (!naturalWidth || !naturalHeight) return null;
  const scale = Math.min(rect.width / naturalWidth, rect.height / naturalHeight);
  const renderedWidth = naturalWidth * scale;
  const renderedHeight = naturalHeight * scale;
  const offsetX = (rect.width - renderedWidth) * 0.5;
  const offsetY = (rect.height - renderedHeight) * 0.5;
  const localX = event.clientX - rect.left - offsetX;
  const localY = event.clientY - rect.top - offsetY;
  if (localX < 0 || localY < 0 || localX > renderedWidth || localY > renderedHeight) {
    return null;
  }
  return {
    point_px: [
      localX * naturalWidth / renderedWidth,
      localY * naturalHeight / renderedHeight
    ],
    image_width: naturalWidth,
    image_height: naturalHeight,
    display_mode: displayMode
  };
}

async function requestStreamClick(slot, event) {
  if (!streamClickModes[slot]) return;
  if (!requireUnlocked("camera")) return;
  const key = streamSelections[slot] || selectedStreamKey(slot);
  const cameraKey = streamClickCameraKey(key);
  const cameraProblem = streamClickCameraProblem(cameraKey);
  if (cameraProblem) {
    setStreamCaptureStatus(slot, cameraProblem, false);
    return;
  }
  const payload = imageEventPayload(event, key);
  if (!payload) return;
  payload.camera_key = cameraKey;
  payload.slot = slot;
  streamClickPending[slot] = true;
  streamClickMarkers[slot] = {
    x_ratio: payload.point_px[0] / payload.image_width,
    y_ratio: payload.point_px[1] / payload.image_height
  };
  streamClickActiveSlot = slot;
  renderStreamClick(slot);
  try {
    await post("/api/vision/click", payload);
    setTimeout(refreshState, 120);
  } catch (exc) {
    alert(exc.message || String(exc));
  } finally {
    streamClickPending[slot] = false;
    renderStreamClick(slot);
  }
}

function updateStreamClickMarker(slot, result) {
  const marker = $(`streamClickMarker${slot}`);
  const img = $(`streamImage${slot}`);
  const viewer = $(`streamViewer${slot}`);
  if (!marker || !img || !viewer || !streamClickModes[slot]) {
    if (marker) marker.style.display = "none";
    return;
  }
  let ratio = streamClickMarkers[slot];
  if ((!ratio || !Number.isFinite(ratio.x_ratio) || !Number.isFinite(ratio.y_ratio)) && result?.point_px && result?.image_size) {
    ratio = {
      x_ratio: Number(result.point_px[0]) / Number(result.image_size[0]),
      y_ratio: Number(result.point_px[1]) / Number(result.image_size[1])
    };
  }
  if (!ratio || !Number.isFinite(ratio.x_ratio) || !Number.isFinite(ratio.y_ratio)) {
    marker.style.display = "none";
    return;
  }
  const imgRect = img.getBoundingClientRect();
  const viewerRect = viewer.getBoundingClientRect();
  marker.style.display = "block";
  marker.style.left = `${imgRect.left - viewerRect.left + imgRect.width * ratio.x_ratio}px`;
  marker.style.top = `${imgRect.top - viewerRect.top + imgRect.height * ratio.y_ratio}px`;
}

function renderFoundation() {
  const pose = state.foundation?.object_pose || {};
  const maskArea = state.foundation?.mask_area_px ?? state.foundation?.debug?.mask_area_px;
  const maskFile = state.foundation?.latest_mask_file || state.vision?.latest_mask_file || "-";
  const initialMask = state.latched_yolo?.mask_file
    || (state.foundation?.running && state.foundation?.initial_mask_file)
    || "-";
  if ($("fpInitialMask")) {
    $("fpInitialMask").value = initialMask === "-" ? "" : initialMask;
  }
  summary($("foundationSummary"), [
    {label: "status", value: state.foundation?.status || "-"},
    {label: "running", value: state.foundation?.running ? "yes" : "no"},
    {label: "mode", value: state.foundation?.mode || state.foundation?.debug?.mode || "waiting"},
    {label: "mask", value: `${state.foundation?.mask_source || state.foundation?.debug?.mask_source || "-"} ${maskArea ? `(${maskArea}px)` : ""}`},
    {label: "initial mask", value: initialMask},
    {label: "mask file", value: maskFile},
    {label: "preview", value: `${streamResolutionText("foundation") || "-"} | ${state.image_hz?.foundation !== undefined ? `${number(state.image_hz.foundation, 1)} fps` : "-"}`},
    {label: "pose", value: pose.position ? pose.position.map((v) => number(v, 3)).join(", ") : "-"}
  ]);
}

function renderVla() {
  const vla = state.vla || {};
  const status = $("vlaStatus");
  if (status) {
    const pending = vlaRequestPending ? "请求中..." : "";
    const text = [pending, vla.status, vla.last_error].filter(Boolean).join(" | ");
    status.textContent = text || "idle";
    status.classList.toggle("warn", Boolean(vla.last_error));
  }
  if ($("vlaPolicyHost") && !$("vlaPolicyHost").dataset.ready) {
    $("vlaPolicyHost").value = vla.policy_host || "10.30.20.1";
    $("vlaPolicyPort").value = vla.policy_port || 18081;
    $("vlaPolicyHost").dataset.ready = "1";
  }
  populateVlaTaskSelect();
  populateVlaExecutionSchemas();
  syncVlaInputs();
  updateVlaStateSourceControl();
  drawVlaCharts();
  renderVlaTaskSummary();
  renderVlaInferSummary();
  renderVlaMcapEval();
  updateVlaPolicyControls();
  updateVlaTaskControls();
}

function vlaMcapTopics() {
  const value = (id) => ($(id)?.value || "").trim();
  const topics = {
    head_image: value("vlaMcapTopicHead"),
    left_wrist_image: value("vlaMcapTopicLeftWrist"),
    right_wrist_image: value("vlaMcapTopicRightWrist"),
    torso_state: value("vlaMcapTopicTorsoState"),
    left_state: value("vlaMcapTopicLeftState"),
    left_gripper_state: value("vlaMcapTopicLeftGripperState"),
    right_state: value("vlaMcapTopicRightState"),
    right_gripper_state: value("vlaMcapTopicRightGripperState"),
    torso_action: value("vlaMcapTopicTorsoAction"),
    left_action: value("vlaMcapTopicLeftAction"),
    left_gripper_action: value("vlaMcapTopicLeftGripperAction"),
    right_action: value("vlaMcapTopicRightAction"),
    right_gripper_action: value("vlaMcapTopicRightGripperAction")
  };
  return Object.fromEntries(Object.entries(topics).filter(([, item]) => item));
}

function vlaMcapPayload() {
  const endRaw = $("vlaMcapEndFrame")?.value;
  const strideRaw = $("vlaMcapStride")?.value;
  return {
    ...vlaInferPayload(),
    dataset_path: vlaDatasetPath(),
    dataset_format: "mcap",
    state_schema: vlaStateSchemaKey(),
    action_schema: vlaActionSchemaKey(),
    start_frame: Math.max(0, Number($("vlaMcapStartFrame")?.value || 0)),
    end_frame: endRaw === "" ? null : Number(endRaw),
    stride: strideRaw === "" ? null : Math.max(1, Number(strideRaw)),
    max_sync_error_sec: Math.max(0.001, Number($("vlaMcapSyncError")?.value || 0.25)),
    topics: vlaMcapTopics()
  };
}

async function listVlaMcapChannels() {
  try {
    const data = await post("/api/vla/mcap_channels", vlaMcapPayload(), {timeoutMs: 30000});
    $("vlaMcapChannelsOutput").textContent = JSON.stringify(data.files || [], null, 2);
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
  }
}

async function dryRunVlaMcap() {
  setVlaBusy(true);
  try {
    const data = await post("/api/vla/mcap_dry_run", vlaMcapPayload(), {timeoutMs: 125000});
    $("vlaMcapChannelsOutput").textContent = JSON.stringify(data.episodes || [], null, 2);
    $("vlaStatus").textContent = data.message || "MCAP synchronization check complete";
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
  } finally {
    setVlaBusy(false);
  }
}

async function startVlaMcapEval() {
  if (!vlaDatasetPath() || vlaDatasetFormat() !== "mcap") {
    alert("请先选择或输入 MCAP 文件");
    return;
  }
  vlaMcapEvalResult = null;
  const data = await post("/api/vla/mcap_eval_start", vlaMcapPayload(), {timeoutMs: 10000});
  $("vlaStatus").textContent = data.message || "MCAP evaluation started";
  await refreshState();
}

async function stopVlaMcapEval() {
  const data = await post("/api/vla/mcap_eval_stop", {}, {timeoutMs: 10000});
  $("vlaStatus").textContent = data.message || "MCAP evaluation stopped";
  await refreshState();
}

async function fetchVlaMcapEvalResult() {
  if (vlaMcapEvalFetchPending) return;
  vlaMcapEvalFetchPending = true;
  try {
    const data = await post("/api/vla/mcap_eval_result", {}, {timeoutMs: 30000});
    vlaMcapEvalResult = data.result || null;
    drawVlaMcapEvalChart();
  } finally {
    vlaMcapEvalFetchPending = false;
  }
}

function renderVlaMcapEval() {
  const evalState = state.vla?.mcap_eval || {};
  const playback = state.vla?.mcap_playback || {};
  const running = Boolean(evalState.running);
  if ($("vlaMcapEvalStart")) {
    $("vlaMcapEvalStart").disabled = running || vlaRequestPending || vlaTaskRunning || !state.vla?.task_loaded;
  }
  if ($("vlaMcapEvalStop")) $("vlaMcapEvalStop").disabled = !running;
  if ($("vlaExecuteFullResult")) {
    $("vlaExecuteFullResult").disabled = (
      running || Boolean(playback.running) || vlaTaskRunning ||
      !evalState.result_ready || !locks.vla
    );
  }
  if ($("vlaStopFullResult")) $("vlaStopFullResult").disabled = !playback.running;
  summary($("vlaMcapEvalSummary"), [
    {label: "status", value: evalState.status || "idle"},
    {label: "progress", value: `${evalState.progress || 0}/${evalState.total || 0}`},
    {label: "frame", value: evalState.current_frame ?? "-"},
    {label: "result", value: evalState.result_ready ? "ready" : "-"},
    {label: "playback", value: playback.running ? `${playback.profile || "-"} / ${playback.execution_schema || "-"}` : "stopped"},
    {label: "playback steps", value: playback.steps || "-"},
    {label: "playback duration", value: playback.duration_s ? `${number(playback.duration_s, 2)} s` : "-"},
    {label: "error", value: evalState.last_error || "-"}
  ]);
  if (evalState.result_ready && !vlaMcapEvalResult) fetchVlaMcapEvalResult().catch(() => {});
  if (vlaMcapEvalResult) drawVlaMcapEvalChart();
}

async function executeVlaFullResult() {
  if (!requireUnlocked("vla")) return;
  if (!state.vla?.mcap_eval?.result_ready) {
    alert("请先完成一次 MCAP 完整推理");
    return;
  }
  const profile = vlaInferProfile();
  let confirmReal = false;
  if (profile === "real") {
    confirmReal = confirm(
      "即将在真机上执行由录制 MCAP 图像和 state 预先计算的完整轨迹。\n" +
      "请确认机器人初始姿态、周围环境和数据集一致，并确保急停可用。\n\n继续执行吗？"
    );
    if (!confirmReal) return;
  }
  setVlaBusy(true);
  try {
    const data = await post("/api/vla/mcap_eval_execute", {
      profile,
      execution_schema: vlaExecutionSchema(),
      speed_scale: 1.0,
      confirm_real: confirmReal
    }, {timeoutMs: 15000});
    $("vlaStatus").textContent = data.message || "完整推理结果开始执行";
    $("vlaStatus").classList.remove("warn");
    await refreshState();
    return data;
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
    return null;
  } finally {
    setVlaBusy(false);
  }
}

async function stopVlaFullResult() {
  try {
    const data = await post("/api/vla/mcap_eval_execute_stop", {}, {timeoutMs: 10000});
    $("vlaStatus").textContent = data.message || "完整推理结果已停止";
    $("vlaStatus").classList.remove("warn");
    await refreshState();
    return data;
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
    return null;
  }
}

function drawVlaMcapEvalChart() {
  if (!vlaMcapEvalResult) return;
  const datasetLabel = vlaMcapEvalResult.joint_action_space === "absolute_position_radians"
    ? "MCAP absolute action"
    : "MCAP action";
  drawVlaComparisonChart($("vlaMcapEvalChart"), [
    {rows: vlaMcapEvalResult.original_actions || [], label: datasetLabel, style: "dataset"},
    {rows: vlaMcapEvalResult.predicted_actions || [], label: "policy action", style: "policy"}
  ], vlaMcapEvalResult.action_schema || "left_arm_8", vlaMcapEvalResult);
}

const VLA_EXECUTION_LABELS = {
  server: "服务端默认",
  left_arm_8: "左臂 8D",
  right_arm_8: "右臂 8D",
  bimanual_16: "双臂 16D",
  upper_body_20: "上半身 20D"
};

function vlaExecutionSchema() {
  return $("vlaExecutionSchema")?.value || "server";
}

function populateVlaExecutionSchemas() {
  const select = $("vlaExecutionSchema");
  if (!select) return;
  const current = select.value || state.vla?.execution_schema || "server";
  const available = state.vla?.available_execution_schemas || ["server"];
  select.innerHTML = "";
  available.forEach((key) => {
    const option = document.createElement("option");
    option.value = key;
    option.textContent = VLA_EXECUTION_LABELS[key] || key;
    select.appendChild(option);
  });
  select.value = available.includes(current) ? current : "server";
}

function vlaRemoteTasks() {
  return state.vla?.remote_tasks || {};
}

function vlaRemoteTaskKey() {
  return $("vlaRemoteTask")?.value || state.vla?.selected_task || "";
}

function vlaRemoteTaskDef(task = vlaRemoteTaskKey()) {
  if (state.vla?.loaded_task === task && state.vla?.task_metadata) return state.vla.task_metadata;
  return vlaRemoteTasks()[task] || {r1pro_task: task, label: task};
}

function vlaRemoteTaskSchemaKey(task = vlaRemoteTaskKey()) {
  return vlaRemoteTaskDef(task).action_schema || "";
}

function vlaRemoteTaskActionDim(task = vlaRemoteTaskKey()) {
  const schema = vlaRemoteTaskSchemaKey(task);
  return Number(vlaRemoteTaskDef(task).action_dim || (schema ? vlaSchemaDef(schema).action_dim : 0) || 0);
}

function populateVlaTaskSelect() {
  const select = $("vlaRemoteTask");
  if (!select) return;
  const tasks = vlaRemoteTasks();
  const current = select.value || state.vla?.selected_task || "";
  select.innerHTML = "";
  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = Object.keys(tasks).length ? "请选择服务端任务" : "请先连接 Policy Server";
  select.appendChild(placeholder);
  Object.entries(tasks).forEach(([key, task]) => {
    const opt = document.createElement("option");
    opt.value = key;
    opt.textContent = task.label || key;
    select.appendChild(opt);
  });
  if ([...select.options].some((opt) => opt.value === current)) select.value = current;
  select.disabled = !Object.keys(tasks).length || vlaTaskRunning || Boolean(state.vla?.mcap_eval?.running);
}

function renderVlaTaskSummary() {
  const selectedKey = vlaRemoteTaskKey();
  const selected = vlaRemoteTaskDef(selectedKey);
  const loadedKey = state.vla?.loaded_task || "";
  const meta = state.vla?.task_metadata || selected;
  const isLoaded = Boolean(state.vla?.task_loaded) && loadedKey === selectedKey;
  summary($("vlaTaskSummary"), [
    {label: "protocol", value: state.vla?.server_metadata?.protocol_version || "-"},
    {label: "selected", value: selectedKey ? `${selected.label || selectedKey} (${selectedKey})` : "-"},
    {label: "loaded", value: isLoaded ? "yes" : (loadedKey ? `no (${loadedKey})` : "no")},
    {label: "state schema", value: meta.state_schema || selected.state_schema || "-"},
    {label: "action schema", value: meta.action_schema || selected.action_schema || "-"},
    {label: "execute", value: VLA_EXECUTION_LABELS[vlaExecutionSchema()] || vlaExecutionSchema()},
    {label: "execution allowed", value: meta.execution_allowed ? "yes" : "no"},
    {label: "cameras", value: (meta.required_cameras || []).join(", ") || "-"},
    {label: "state/action", value: `${meta.state_dim ?? selected.state_dim ?? "-"} / ${meta.action_dim ?? selected.action_dim ?? "-"}`},
    {label: "state history", value: meta.state_history_enabled
      ? `${number(meta.state_history_seconds, 1)}s | current ${meta.base_state_dim}D + previous infer ${meta.base_state_dim}D`
      : "off"},
    {label: "fixed prompt", value: meta.fixed_prompt || "-"},
    {label: "horizon", value: meta.action_horizon ?? selected.action_horizon ?? "-"},
    {label: "control rate", value: meta.control_frequency_hz ? `${number(meta.control_frequency_hz, 1)} Hz` : "-"},
    {label: "output transform", value: meta.server_output_transform || "-"},
    {label: "joint/gripper", value: `${meta.joint_action_space || "-"} / ${meta.gripper_action_space || "-"}`},
    {label: "checkpoint", value: meta.checkpoint_step ?? state.vla?.checkpoint_step ?? "-"},
    {label: "execution reason", value: meta.execution_disabled_reason || "-"},
    {label: "host", value: state.vla?.loaded_host && state.vla?.loaded_port ? `${state.vla.loaded_host}:${state.vla.loaded_port}` : "-"}
  ]);
}

function updateVlaStateSourceControl() {
  const input = $("vlaInputSource");
  const stateSource = $("vlaStateSource");
  if (!input || !stateSource) return;
  const live = (input.value || "live") === "live";
  stateSource.disabled = live || vlaTaskRunning;
  if (live) stateSource.value = "feedback";
}

function updateVlaPolicyControls() {
  const evalRunning = Boolean(state.vla?.mcap_eval?.running);
  const resultRunning = Boolean(state.vla?.mcap_playback?.running);
  const executionAllowed = Boolean(state.vla?.task_metadata?.execution_allowed);
  const taskLoaded = Boolean(state.vla?.task_loaded);
  const selectedTask = vlaRemoteTaskKey();
  const loadedTask = state.vla?.loaded_task || "";
  const commonDisabled = vlaRequestPending || evalRunning || resultRunning || vlaTaskRunning;
  const load = $("vlaLoadTask");
  if (load) load.disabled = commonDisabled || !selectedTask;
  const inferDisabled = commonDisabled || !taskLoaded || !selectedTask || selectedTask !== loadedTask;
  ["vlaInfer", "vlaFullInfer"].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = inferDisabled;
  });
  ["vlaExecuteFirst", "vlaExecuteChunk"].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = inferDisabled || !locks.vla || !executionAllowed;
  });
  updateVlaTaskControls();
}

function updateVlaTaskControls() {
  const evalRunning = Boolean(state.vla?.mcap_eval?.running);
  const resultRunning = Boolean(state.vla?.mcap_playback?.running);
  const start = $("vlaRunTask");
  const stop = $("vlaStopTask");
  const maxChunks = $("vlaTaskMaxChunks");
  const settle = $("vlaTaskSettleS");
  const remoteTask = $("vlaRemoteTask");
  const executionSchema = $("vlaExecutionSchema");
  const executionAllowed = Boolean(state.vla?.task_metadata?.execution_allowed);
  if (start) start.disabled = vlaRequestPending || vlaTaskRunning || evalRunning || resultRunning || !state.vla?.task_loaded || !locks.vla || !executionAllowed;
  if (stop) stop.disabled = !vlaTaskRunning;
  if (maxChunks) maxChunks.disabled = vlaTaskRunning;
  if (settle) settle.disabled = vlaTaskRunning;
  if (remoteTask) remoteTask.disabled = vlaTaskRunning || evalRunning || resultRunning || vlaRequestPending || !Object.keys(vlaRemoteTasks()).length;
  if (executionSchema) executionSchema.disabled = vlaTaskRunning || evalRunning || resultRunning || vlaRequestPending;
}

function vlaDatasetPath() {
  const custom = ($("vlaDatasetPath")?.value || "").trim();
  if (custom) return custom;
  const selected = vlaSelectedDataset();
  return selected?.path || "";
}

function vlaDatasetFormat() {
  const custom = ($("vlaDatasetPath")?.value || "").trim();
  if (custom) return $("vlaDatasetFormat")?.value || "auto";
  const selected = vlaSelectedDataset();
  return selected?.format || $("vlaDatasetFormat")?.value || "auto";
}

function vlaSelectedDataset() {
  const path = $("vlaDatasetSelect")?.value || "";
  return vlaDatasets.find((item) => item.path === path) || null;
}

async function refreshVlaDatasets() {
  try {
    const data = await post("/api/vla/datasets", {}, {timeoutMs: 10000});
    vlaDatasets = data.datasets || [];
    fillVlaDatasetSelect(data.root || "");
  } catch (exc) {
    vlaDatasets = [];
    fillVlaDatasetSelect("");
    const status = $("vlaStatus");
    if (status) {
      status.textContent = exc.message || String(exc);
      status.classList.add("warn");
    }
  }
}

function fillVlaDatasetSelect(root) {
  const select = $("vlaDatasetSelect");
  if (!select) return;
  const current = select.value;
  select.innerHTML = "";
  const empty = document.createElement("option");
  empty.value = "";
  empty.textContent = vlaDatasets.length ? "选择数据集" : `无数据集${root ? ` (${root})` : ""}`;
  select.appendChild(empty);
  vlaDatasets.forEach((item) => {
    const opt = document.createElement("option");
    opt.value = item.path;
    const suffix = item.format === "mcap" ? "MCAP" : "LeRobot";
    opt.textContent = `${item.name} [${suffix}]`;
    select.appendChild(opt);
  });
  if ([...select.options].some((opt) => opt.value === current)) select.value = current;
  else if (vlaDatasets.length) select.value = vlaDatasets[0].path;
  syncVlaDatasetSelection();
}

function syncVlaDatasetSelection() {
  const selected = vlaSelectedDataset();
  const custom = ($("vlaDatasetPath")?.value || "").trim();
  if (selected && !custom && $("vlaDatasetFormat")) {
    $("vlaDatasetFormat").value = selected.format || "auto";
  }
}

function vlaEpisodeIndex() {
  return Number($("vlaEpisodeSelect")?.value || 0);
}

function vlaFrameIndex() {
  return Math.max(0, Number($("vlaFrameIndex")?.value || $("vlaFrameSlider")?.value || 0));
}

function vlaActionSchemaKey() {
  return $("vlaActionSchema")?.value || "bimanual_16";
}

function vlaStateSchemaKey() {
  return $("vlaStateSchema")?.value || vlaActionSchemaKey();
}

function vlaSchemaDef(schemaKey = vlaActionSchemaKey()) {
  const schemas = state.vla?.action_schemas || [];
  const found = schemas.find((item) => item.key === schemaKey);
  if (found) return found;
  if (schemaKey === "left_arm_8") {
    return {
      key: "left_arm_8",
      label: "左臂 8 维",
      action_dim: 8,
      dim_names: ["left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7", "left_gripper"]
    };
  }
  if (schemaKey === "right_arm_8") {
    return {key: "right_arm_8", label: "右臂 8 维", action_dim: 8, dim_names: []};
  }
  if (schemaKey === "upper_body_20" || schemaKey === "full_20") {
    return {key: "upper_body_20", label: "躯干 + 双臂 20 维", action_dim: 20, dim_names: []};
  }
  return {
    key: "bimanual_16",
    label: "双臂 16 维",
    action_dim: 16,
    dim_names: [
      "left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7", "left_gripper",
      "right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7", "right_gripper"
    ]
  };
}

function vlaPlaybackProfile() {
  return $("vlaPlaybackProfile")?.value || "sim";
}

function vlaInferProfile() {
  return $("vlaInferProfile")?.value || "sim";
}

function vlaControlFrequencyHz() {
  const value = Number(state.vla?.task_metadata?.control_frequency_hz || 0);
  if (!Number.isFinite(value) || value <= 0) throw new Error("服务端未提供有效的 control_frequency_hz");
  return value;
}

function vlaPlaybackRateHz() {
  const fps = Number(vlaCurves?.fps || vlaDataset?.fps || 0);
  return Number.isFinite(fps) && fps > 0 ? fps : 15;
}

function vlaRawLeftArmSpeed() {
  const value = Number($("vlaRawLeftArmSpeed")?.value || 1);
  if (!Number.isFinite(value)) return 1;
  return Math.max(0.1, Math.min(1, value));
}

function vlaPlaybackTimestamps(count = 0) {
  const raw = Array.isArray(vlaCurves?.timestamps) ? vlaCurves.timestamps.map(Number) : [];
  if (raw.length >= count && raw.slice(0, count).every((value) => Number.isFinite(value))) {
    const start = raw[0] || 0;
    return raw.slice(0, count).map((value) => Math.max(0, value - start));
  }
  const rate = vlaPlaybackRateHz();
  return [...Array(Math.max(0, Number(count || 0)))].map((_, index) => index / rate);
}

function vlaPlaybackDurationS(actionCount = 0) {
  const duration = Number(vlaCurves?.duration_s || 0);
  if (Number.isFinite(duration) && duration > 0) return duration;
  const timestamps = vlaPlaybackTimestamps(actionCount);
  return timestamps.length > 1 ? timestamps[timestamps.length - 1] - timestamps[0] : 0;
}

function vlaFrameHoldDurationS(timestamps, index) {
  const fallback = 1 / vlaPlaybackRateHz();
  if (index + 1 < timestamps.length) {
    const delta = Number(timestamps[index + 1]) - Number(timestamps[index]);
    if (Number.isFinite(delta) && delta > 0) return delta;
  }
  return fallback;
}

function setVlaBusy(busy) {
  vlaRequestPending = busy;
  ["vlaRefreshDatasets", "vlaLoadDataset", "vlaConnectPolicy", "vlaLoadTask", "vlaInfer", "vlaFullInfer", "vlaExecuteFullResult", "vlaRunTask", "vlaExecuteFirst", "vlaExecuteChunk", "vlaExecutePlaybackFrame", "vlaExecutePlaybackChunk", "vlaExecuteRawLeftArm"].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = busy || (el.dataset.lockPage && !locks.vla);
  });
  const stop = $("vlaStopFullInfer");
  if (stop) stop.disabled = !vlaFullInferRunning;
  updateVlaPolicyControls();
  updateVlaTaskControls();
  updateVlaStateSourceControl();
}

async function loadVlaDataset() {
  stopVlaPlaybackAnimation();
  clearVlaFrameCache();
  const path = vlaDatasetPath();
  if (!path) {
    $("vlaStatus").textContent = "请输入数据集路径";
    $("vlaStatus").classList.add("warn");
    return;
  }
  setVlaBusy(true);
  try {
    const data = await post("/api/vla/load_dataset", {
      dataset_path: path,
      dataset_format: vlaDatasetFormat()
    }, {timeoutMs: 30000});
    vlaDataset = data.dataset || null;
    fillVlaEpisodes();
    await loadVlaEpisode();
    await loadVlaFrame();
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
  } finally {
    setVlaBusy(false);
  }
}

function fillVlaEpisodes() {
  const select = $("vlaEpisodeSelect");
  if (!select) return;
  const episodes = vlaDataset?.episodes || [];
  select.innerHTML = "";
  episodes.forEach((episode, idx) => {
    const index = episode.episode_index ?? episode.index ?? idx;
    const opt = document.createElement("option");
    opt.value = String(index);
    const len = episode.length ?? episode.num_frames ?? episode.frame_count ?? "";
    opt.textContent = `episode ${index}${len ? ` (${len})` : ""}`;
    select.appendChild(opt);
  });
  if (!episodes.length) {
    const opt = document.createElement("option");
    opt.value = "0";
    opt.textContent = "episode 0";
    select.appendChild(opt);
  }
}

async function loadVlaEpisode() {
  if (!vlaDatasetPath()) return;
  const data = await post("/api/vla/episode", {
    dataset_path: vlaDatasetPath(),
    dataset_format: vlaDatasetFormat(),
    episode_index: vlaEpisodeIndex(),
    action_dim: Number(vlaSchemaDef().action_dim || 16),
    action_schema: vlaActionSchemaKey(),
    state_schema: vlaStateSchemaKey()
  }, {timeoutMs: 30000});
  vlaCurves = data.curves || null;
  const count = Number(data.frame_count || vlaCurves?.frame_count || 0);
  if (count > 0) {
    $("vlaFrameSlider").max = String(count - 1);
    $("vlaFrameIndex").max = String(count - 1);
    if (vlaFrameIndex() > count - 1) setVlaFrameIndex(count - 1);
  }
}

function setVlaFrameIndex(index) {
  const value = Math.max(0, Number(index || 0));
  if ($("vlaFrameSlider")) $("vlaFrameSlider").value = String(value);
  if ($("vlaFrameIndex")) $("vlaFrameIndex").value = String(value);
}

function clearVlaFrameCache() {
  vlaFrameCache = new Map();
  vlaFullInferComparison = null;
}

function vlaFrameCacheKey(frameIndex) {
  return [
    vlaDatasetPath(),
    vlaDatasetFormat(),
    vlaEpisodeIndex(),
    vlaStateSchemaKey(),
    vlaActionSchemaKey(),
    Number(frameIndex || 0)
  ].join("|");
}

function applyVlaFrame(frame) {
  vlaFrame = frame || null;
  const count = Number(vlaFrame?.frame_count || 0);
  if (count > 0) {
    $("vlaFrameSlider").max = String(count - 1);
    $("vlaFrameIndex").max = String(count - 1);
  }
  setVlaDatasetImages(vlaFrame?.images || {});
  drawVlaCharts();
}

function scheduleLoadVlaFrame(delayMs = 120) {
  if (vlaFrameLoadTimer) clearTimeout(vlaFrameLoadTimer);
  vlaFrameLoadTimer = setTimeout(() => {
    vlaFrameLoadTimer = null;
    loadVlaFrame().catch((exc) => {
      const status = $("vlaStatus");
      if (status) {
        status.textContent = exc.message || String(exc);
        status.classList.add("warn");
      }
    });
  }, delayMs);
}

async function fetchVlaFrame(frameIndex) {
  const key = vlaFrameCacheKey(frameIndex);
  if (vlaFrameCache.has(key)) return vlaFrameCache.get(key);
  const data = await post("/api/vla/frame", {
    dataset_path: vlaDatasetPath(),
    dataset_format: vlaDatasetFormat(),
    episode_index: vlaEpisodeIndex(),
    frame_index: frameIndex,
    action_dim: Number(vlaSchemaDef().action_dim || 16),
    action_schema: vlaActionSchemaKey(),
    state_schema: vlaStateSchemaKey()
  }, {timeoutMs: 30000});
  const frame = data.frame || null;
  vlaFrameCache.set(key, frame);
  return frame;
}

async function loadVlaFrame(frameIndexOverride = null) {
  if (vlaFrameLoadTimer) {
    clearTimeout(vlaFrameLoadTimer);
    vlaFrameLoadTimer = null;
  }
  if (!vlaDatasetPath()) return;
  const frameIndex = frameIndexOverride == null ? vlaFrameIndex() : Math.max(0, Number(frameIndexOverride || 0));
  const frame = await fetchVlaFrame(frameIndex);
  if (frameIndex !== vlaFrameIndex()) return frame;
  applyVlaFrame(frame);
  return frame;
}

function stopVlaPlaybackAnimation() {
  vlaPlaybackRunId += 1;
  if (vlaPlaybackTimer) clearInterval(vlaPlaybackTimer);
  if (vlaPlaybackTimeout) clearTimeout(vlaPlaybackTimeout);
  vlaPlaybackTimer = null;
  vlaPlaybackTimeout = null;
  vlaPlaybackLoadInFlight = false;
}

function waitVlaPlaybackMs(ms, runId) {
  return new Promise((resolve) => {
    if (ms <= 0) {
      resolve();
      return;
    }
    vlaPlaybackTimeout = setTimeout(() => {
      vlaPlaybackTimeout = null;
      if (runId === vlaPlaybackRunId) resolve();
    }, ms);
  });
}

function postVlaPlaybackStep(action, durationS) {
  return post("/api/vla/execute", {
    actions: [action],
    mode: "first",
    profile: vlaPlaybackProfile(),
    action_schema: vlaActionSchemaKey(),
    duration_s: Math.max(0.02, Number(durationS || 0))
  }, {timeoutMs: 5000});
}

async function prefetchVlaPlaybackFrames(count) {
  const total = Math.max(0, Number(count || 0));
  const concurrency = 4;
  let next = 0;
  const status = $("vlaStatus");
  const worker = async () => {
    while (next < total) {
      const index = next;
      next += 1;
      if (status && index % 10 === 0) {
        status.textContent = `预取回放图像 ${index + 1}/${total}`;
        status.classList.remove("warn");
      }
      await fetchVlaFrame(index);
    }
  };
  await Promise.all([...Array(Math.min(concurrency, total))].map(worker));
}

async function startVlaSynchronizedPlayback(actions) {
  stopVlaPlaybackAnimation();
  const total = Math.max(0, Math.min(actions.length, Number($("vlaFrameSlider")?.max || 0) + 1));
  if (total <= 0) return;
  const runId = vlaPlaybackRunId;
  const timestamps = vlaPlaybackTimestamps(total);
  await prefetchVlaPlaybackFrames(total);
  if (runId !== vlaPlaybackRunId) return;
  const status = $("vlaStatus");
  if (status) {
    status.textContent = `VLA synchronized playback ${total} frame(s) at ${number(vlaPlaybackRateHz(), 2)} Hz`;
    status.classList.remove("warn");
  }
  const data = await executeVlaActions(
    actions.slice(0, total),
    "chunk",
    vlaPlaybackProfile(),
    "VLA playback execute sent",
    vlaPlaybackRateHz(),
    vlaPlaybackDurationS(total)
  );
  if (!data?.ok || runId !== vlaPlaybackRunId) return;
  const startedAt = performance.now();
  for (let frameIndex = 0; frameIndex < total; frameIndex += 1) {
    if (runId !== vlaPlaybackRunId) return;
    const targetAt = startedAt + Number(timestamps[frameIndex] || 0) * 1000;
    await waitVlaPlaybackMs(Math.max(0, targetAt - performance.now()), runId);
    if (runId !== vlaPlaybackRunId) return;
    setVlaFrameIndex(frameIndex);
    applyVlaFrame(vlaFrameCache.get(vlaFrameCacheKey(frameIndex)));
  }
  if (runId === vlaPlaybackRunId && status) {
    status.textContent = `VLA synchronized playback finished (${total} frame(s))`;
    status.classList.remove("warn");
  }
  stopVlaPlaybackAnimation();
}

function setVlaDatasetImages(images) {
  setVlaDataImage("vlaDatasetImageHigh", "vlaDatasetEmptyHigh", images.cam_high);
  setVlaDataImage("vlaDatasetImageLeft", "vlaDatasetEmptyLeft", images.cam_left_wrist);
  setVlaDataImage("vlaDatasetImageRight", "vlaDatasetEmptyRight", images.cam_right_wrist);
}

function setVlaDataImage(imgId, emptyId, src) {
  const img = $(imgId);
  const empty = $(emptyId);
  if (!img) return;
  if (src) {
    img.src = src;
    img.style.display = "block";
    if (empty) empty.style.display = "none";
  } else {
    img.removeAttribute("src");
    img.style.display = "none";
    if (empty) empty.style.display = "block";
  }
}

function syncVlaInputs() {
  const source = $("vlaInputSource")?.value || "live";
  if (source === "dataset") {
    setVlaDataImage("vlaInferImageHigh", "vlaInferEmptyHigh", vlaFrame?.images?.cam_high);
    setVlaDataImage("vlaInferImageLeft", "vlaInferEmptyLeft", vlaFrame?.images?.cam_left_wrist);
    setVlaDataImage("vlaInferImageRight", "vlaInferEmptyRight", vlaFrame?.images?.cam_right_wrist);
    return;
  }
  setVlaLiveImage("vlaInferImageHigh", "vlaInferEmptyHigh", "head_left_raw_rgb");
  setVlaLiveImage("vlaInferImageLeft", "vlaInferEmptyLeft", "wrist_left_rgb");
  setVlaLiveImage("vlaInferImageRight", "vlaInferEmptyRight", "wrist_right_rgb");
}

function setVlaLiveImage(imgId, emptyId, streamKey) {
  const img = $(imgId);
  const empty = $(emptyId);
  if (!img) return;
  if (!streamKey || activePage() !== "vla") {
    img.removeAttribute("src");
    img.style.display = "none";
    if (empty) empty.style.display = "block";
    return;
  }
  if (img.dataset.streamKey !== streamKey) {
    img.dataset.streamKey = streamKey;
    connectCameraImageStream(img, streamKey);
  }
  img.style.display = "block";
  if (empty) empty.style.display = "none";
}

function vlaInferPayload() {
  const task = vlaRemoteTaskKey();
  return {
    task,
    input_source: $("vlaInputSource").value,
    state_source: $("vlaStateSource").value,
    profile: vlaInferProfile(),
    execution_schema: vlaExecutionSchema(),
    dataset_path: vlaDatasetPath(),
    dataset_format: vlaDatasetFormat(),
    episode_index: vlaEpisodeIndex(),
    frame_index: vlaFrameIndex(),
    host: $("vlaPolicyHost").value || "10.30.20.1",
    port: Number($("vlaPolicyPort").value || 18081)
  };
}

async function connectVlaPolicy() {
  vlaLastInfer = null;
  vlaFullInferComparison = null;
  vlaMcapEvalResult = null;
  setVlaBusy(true);
  try {
    const data = await post("/api/vla/connect", {
      host: $("vlaPolicyHost").value || "10.30.20.1",
      port: Number($("vlaPolicyPort").value || 18081),
      timeout_s: 30
    }, {timeoutMs: 35000});
    $("vlaStatus").textContent = data.message || "Policy connected";
    await refreshState();
    populateVlaTaskSelect();
    return data;
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
    return null;
  } finally {
    setVlaBusy(false);
  }
}

async function loadVlaRemoteTask() {
  if (!vlaRemoteTaskKey()) {
    alert("请先从 Policy Server 返回的任务列表中选择任务");
    return null;
  }
  setVlaBusy(true);
  try {
    const data = await post("/api/vla/load_task", {
      ...vlaInferPayload(),
      force_reload: true,
      timeout_s: 300
    }, {timeoutMs: 305000});
    $("vlaStatus").textContent = data.message || "R1Pro task loaded";
    $("vlaStatus").classList.toggle("warn", !data.ok);
    if ($("vlaExecutionSchema")) $("vlaExecutionSchema").value = "server";
    await refreshState();
    return data;
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
    return null;
  } finally {
    setVlaBusy(false);
  }
}

async function runVlaInfer() {
  if (!state.vla?.task_loaded || state.vla?.loaded_task !== vlaRemoteTaskKey()) {
    alert("请先加载当前选择的服务端任务");
    return;
  }
  setVlaBusy(true);
  try {
    vlaFullInferComparison = null;
    const data = await post("/api/vla/infer", vlaInferPayload(), {timeoutMs: 120000});
    vlaLastInfer = data;
    renderVlaInferSummary();
    drawVlaCharts();
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
  } finally {
    setVlaBusy(false);
  }
}

function vlaInterruptibleSleep(ms, runId) {
  const deadline = performance.now() + Math.max(0, Number(ms) || 0);
  return new Promise((resolve) => {
    const step = () => {
      if (runId !== vlaTaskRunId || !vlaTaskRunning || performance.now() >= deadline) {
        resolve();
        return;
      }
      setTimeout(step, Math.min(100, Math.max(0, deadline - performance.now())));
    };
    step();
  });
}

function stopVlaFullInfer() {
  vlaFullInferRunId += 1;
  vlaFullInferRunning = false;
  if (state.vla?.mcap_eval?.running) {
    post("/api/vla/mcap_eval_stop", {}, {timeoutMs: 10000}).catch(() => {});
  }
  setVlaBusy(false);
  const status = $("vlaStatus");
  if (status) {
    status.textContent = "完整推理已停止";
    status.classList.remove("warn");
  }
}

async function runVlaFullInfer() {
  if (($("vlaInputSource")?.value || "live") !== "dataset") {
    alert("请先把推理输入切到数据集视频");
    return;
  }
  if (!vlaDatasetPath()) {
    alert("请先加载数据集");
    return;
  }
  if (vlaDatasetFormat() !== "mcap") {
    alert("完整曲线评估仅支持 MCAP；LeRobot 数据请使用单帧推理查看输出");
    return;
  }
  stopVlaPlaybackAnimation();
  const inferSchema = vlaRemoteTaskSchemaKey();
  if (vlaActionSchemaKey() !== inferSchema && $("vlaActionSchema")) {
    $("vlaActionSchema").value = inferSchema;
    clearVlaFrameCache();
    try {
      await loadVlaEpisode();
    } catch (exc) {
      $("vlaStatus").textContent = exc.message || String(exc);
      $("vlaStatus").classList.add("warn");
      return;
    }
  }
  const startFrame = vlaFrameIndex();
  const maxFrame = Math.max(0, Number($("vlaFrameSlider")?.max || 0));
  if (startFrame > maxFrame) {
    alert("当前 frame 超出 episode 范围");
    return;
  }
  const runId = vlaFullInferRunId + 1;
  vlaFullInferRunId = runId;
  vlaFullInferRunning = true;
  vlaFullInferComparison = null;
  vlaMcapEvalResult = null;
  setVlaBusy(true);
  const startedAt = performance.now();
  try {
    const horizon = Number(state.vla?.task_metadata?.action_horizon || 0);
    if (!Number.isFinite(horizon) || horizon <= 0) throw new Error("服务端未提供有效的 action horizon");
    const payload = {
      ...vlaMcapPayload(),
      start_frame: startFrame,
      end_frame: maxFrame + 1,
      stride: horizon
    };
    await post("/api/vla/mcap_eval_start", payload, {timeoutMs: 10000});
    while (runId === vlaFullInferRunId && vlaFullInferRunning) {
      await new Promise((resolve) => setTimeout(resolve, 500));
      await refreshState();
      const evalState = state.vla?.mcap_eval || {};
      $("vlaStatus").textContent = `完整推理 ${evalState.status || "running"} ${evalState.progress || 0}/${evalState.total || 0}${evalState.current_frame == null ? "" : ` frame ${evalState.current_frame}`}`;
      $("vlaStatus").classList.toggle("warn", Boolean(evalState.last_error));
      if (!evalState.running) {
        if (evalState.status === "error") throw new Error(evalState.last_error || "MCAP 完整推理失败");
        if (evalState.status === "stopped") return;
        break;
      }
    }
    if (runId !== vlaFullInferRunId || !vlaFullInferRunning) return;
    const response = await post("/api/vla/mcap_eval_result", {}, {timeoutMs: 30000});
    const result = response.result || {};
    const datasetRows = result.original_actions || [];
    const policyRows = result.predicted_actions || [];
    vlaMcapEvalResult = result;
    const wall = (performance.now() - startedAt) / 1000;
    vlaFullInferComparison = {
      startFrame,
      datasetRows,
      policyRows,
      datasetLabel: result.joint_action_space === "absolute_position_radians" ? "MCAP absolute action" : "MCAP action",
      policyLabel: "policy action chunk",
      actionSchema: result.action_schema || inferSchema,
      actionGroupIndices: result.action_group_indices || {},
      gripperActionSpace: result.gripper_action_space || ""
    };
    drawVlaCharts();
    drawVlaMcapEvalChart();
    $("vlaStatus").textContent = `完整推理完成 ${result.frame_count || policyRows.length} frame, horizon ${result.horizon || horizon}, avg ${number(result.avg_latency_s, 3)}s, min ${number(result.min_latency_s, 3)}s, max ${number(result.max_latency_s, 3)}s, wall ${number(wall, 1)}s`;
    $("vlaStatus").classList.remove("warn");
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
  } finally {
    if (runId === vlaFullInferRunId) {
      vlaFullInferRunning = false;
      setVlaBusy(false);
    }
  }
}

async function executeVla(mode) {
  if (!requireUnlocked("vla")) return;
  const actions = vlaLastInfer?.result?.actions;
  if (!actions) {
    alert("还没有策略输出");
    return;
  }
  const actionSchema = vlaLastInfer?.result?.action_schema || vlaLastInfer?.observation?.action_schema || vlaRemoteTaskSchemaKey();
  await executeVlaActions(actions, mode, vlaInferProfile(), "VLA execute sent", vlaControlFrequencyHz(), null, actionSchema, {
    modelActions: true,
    observationState: vlaLastInfer?.observation?.state || [],
    executionSchema: vlaExecutionSchema()
  });
}

async function runVlaTask() {
  if (!requireUnlocked("vla")) return;
  if (vlaTaskRunning) return;
  stopVlaPlaybackAnimation();
  stopVlaFullInfer();
  const maxChunks = Math.max(1, Math.min(100, Number($("vlaTaskMaxChunks")?.value || 20)));
  const settleS = Math.max(0, Math.min(5, Number($("vlaTaskSettleS")?.value || 0.5)));
  const runId = vlaTaskRunId + 1;
  vlaTaskRunId = runId;
  vlaTaskRunning = true;
  vlaFullInferComparison = null;
  updateVlaTaskControls();
  updateVlaPolicyControls();
  const latencies = [];
  const startedAt = performance.now();
  try {
    for (let index = 0; index < maxChunks; index += 1) {
      if (runId !== vlaTaskRunId || !vlaTaskRunning) break;
      $("vlaStatus").textContent = `完整任务 ${index + 1}/${maxChunks}: 推理中`;
      $("vlaStatus").classList.remove("warn");
      const inferData = await post("/api/vla/infer", {
        ...vlaInferPayload(),
        history_mode: "previous_infer",
        reset_history: index === 0
      }, {timeoutMs: 120000});
      if (runId !== vlaTaskRunId || !vlaTaskRunning) break;
      vlaLastInfer = inferData;
      const elapsed = Number(inferData?.result?.elapsed_s);
      if (Number.isFinite(elapsed)) latencies.push(elapsed);
      renderVlaInferSummary();
      drawVlaCharts();
      const actions = inferData?.result?.actions || [];
      if (!Array.isArray(actions) || !Array.isArray(actions[0])) {
        throw new Error("策略没有返回可执行 action chunk");
      }
      const actionSchema = inferData?.result?.action_schema || vlaRemoteTaskSchemaKey();
      const rateHz = vlaControlFrequencyHz();
      const durationS = actions.length / rateHz;
      $("vlaStatus").textContent = `完整任务 ${index + 1}/${maxChunks}: 执行 ${actions.length} step`;
      const executed = await executeVlaActions(
        actions,
        "chunk",
        vlaInferProfile(),
        "VLA task chunk sent",
        rateHz,
        null,
        actionSchema,
        {
          manageBusy: false,
          quiet: true,
          modelActions: true,
          observationState: inferData?.observation?.state || [],
          executionSchema: vlaExecutionSchema()
        }
      );
      if (!executed) throw new Error("执行 chunk 失败");
      $("vlaStatus").textContent = `完整任务 ${index + 1}/${maxChunks}: 等待反馈更新`;
      await vlaInterruptibleSleep((durationS + settleS) * 1000, runId);
    }
    if (runId !== vlaTaskRunId) return;
    const avg = latencies.length ? latencies.reduce((sum, value) => sum + value, 0) / latencies.length : 0;
    const wall = (performance.now() - startedAt) / 1000;
    $("vlaStatus").textContent = `完整任务完成 ${latencies.length}/${maxChunks} chunk, avg infer ${number(avg, 3)}s, wall ${number(wall, 1)}s`;
    $("vlaStatus").classList.remove("warn");
  } catch (exc) {
    if (runId === vlaTaskRunId) {
      $("vlaStatus").textContent = exc.message || String(exc);
      $("vlaStatus").classList.add("warn");
    }
  } finally {
    if (runId === vlaTaskRunId) {
      vlaTaskRunning = false;
      updateVlaTaskControls();
      updateVlaPolicyControls();
    }
  }
}

async function stopVlaTask() {
  vlaTaskRunId += 1;
  vlaTaskRunning = false;
  updateVlaTaskControls();
  updateVlaPolicyControls();
  try {
    await post("/api/control/stop", {}, {timeoutMs: 5000});
  } catch (_exc) {
    // Stop should remain best-effort so the UI can always recover.
  }
  const status = $("vlaStatus");
  if (status) {
    status.textContent = "完整任务已停止";
    status.classList.remove("warn");
  }
}

async function executeVlaPlayback(mode) {
  if (!requireUnlocked("vla")) return;
  const actions = mode === "chunk" ? vlaCurves?.action : (vlaFrame?.action ? [vlaFrame.action] : null);
  if (!actions || !actions.length) {
    alert("当前数据集帧/episode 没有 action 数据");
    return;
  }
  const expectedDim = Number(vlaSchemaDef(vlaActionSchemaKey()).action_dim || 0);
  if (!actions.every((row) => Array.isArray(row) && row.length === expectedDim)) {
    alert(`数据集 action 与当前动作维度不一致（期望 ${expectedDim}D）。请切换维度后重新加载数据集。`);
    return;
  }
  stopVlaPlaybackAnimation();
  if (mode === "chunk") {
    startVlaSynchronizedPlayback(actions).catch((exc) => {
      const status = $("vlaStatus");
      if (status) {
        status.textContent = exc.message || String(exc);
        status.classList.add("warn");
      }
    });
    return;
  }
  const data = await executeVlaActions(
    actions,
    mode,
    vlaPlaybackProfile(),
    "VLA playback execute sent",
    vlaPlaybackRateHz(),
    vlaPlaybackDurationS(actions.length)
  );
  if (data?.ok) {
    scheduleLoadVlaFrame(0);
  }
}

async function executeVlaRawLeftArmPlayback() {
  if (!requireUnlocked("vla")) return;
  stopVlaPlaybackAnimation();
  setVlaBusy(true);
  try {
    const data = await post("/api/vla/raw_playback", {
      dataset_path: vlaDatasetPath(),
      dataset_format: vlaDatasetFormat(),
      episode_index: vlaEpisodeIndex(),
      profile: vlaPlaybackProfile(),
      action_schema: vlaActionSchemaKey(),
      speed_scale: vlaRawLeftArmSpeed()
    }, {timeoutMs: 30000});
    $("vlaStatus").textContent = data.message || "Raw playback started";
    $("vlaStatus").classList.toggle("warn", !data.ok);
    return data;
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
    return null;
  } finally {
    setVlaBusy(false);
  }
}

async function executeVlaActions(actions, mode, profile, fallbackMessage, rateHz = null, durationS = null, actionSchema = vlaActionSchemaKey(), options = {}) {
  const manageBusy = options.manageBusy !== false;
  if (manageBusy) setVlaBusy(true);
  try {
    const data = await post("/api/vla/execute", {
      actions,
      mode,
      profile,
      action_schema: actionSchema,
      execution_schema: options.executionSchema || "server",
      model_actions: Boolean(options.modelActions),
      observation_state: options.observationState || undefined,
      rate_hz: rateHz,
      duration_s: mode === "chunk" && durationS ? durationS : undefined
    }, {timeoutMs: 15000});
    if (!options.quiet) {
      $("vlaStatus").textContent = data.message || fallbackMessage;
      $("vlaStatus").classList.remove("warn");
    }
    return data;
  } catch (exc) {
    $("vlaStatus").textContent = exc.message || String(exc);
    $("vlaStatus").classList.add("warn");
    return null;
  } finally {
    if (manageBusy) setVlaBusy(false);
  }
}

function renderVlaInferSummary() {
  const result = vlaLastInfer?.result || {};
  const obs = vlaLastInfer?.observation || {};
  const first = Array.isArray(result.actions) && result.actions.length ? result.actions[0] : [];
  const task = result.r1pro_task || obs.r1pro_task || vlaRemoteTaskKey();
  const taskMeta = result.task_metadata || vlaRemoteTaskDef(task);
  const schema = result.action_schema || obs.action_schema || taskMeta.action_schema || vlaRemoteTaskSchemaKey();
  summary($("vlaInferSummary"), [
    {label: "task", value: task || "-"},
    {label: "action shape", value: result.action_shape ? result.action_shape.join(" x ") : "-"},
    {label: "selected task", value: result.selected_task || result.r1pro_task || "-"},
    {label: "checkpoint", value: result.checkpoint_step ?? "-"},
    {label: "schema", value: vlaSchemaDef(schema).label || schema},
    {label: "state/action dim", value: `${taskMeta.state_dim ?? "-"} / ${taskMeta.action_dim ?? vlaSchemaDef(schema).action_dim ?? "-"}`},
    {label: "infer elapsed", value: result.elapsed_s == null ? "-" : `${number(result.elapsed_s, 3)} s`},
    {label: "server timing", value: Object.keys(result.server_timing || {}).length ? JSON.stringify(result.server_timing) : "-"},
    {label: "policy timing", value: Object.keys(result.policy_timing || {}).length ? JSON.stringify(result.policy_timing) : "-"},
    {label: "state source", value: obs.state_meta?.source || "-"},
    {label: "feedback", value: obs.state_meta?.profile || "-"},
    {label: "history", value: obs.state_meta?.history?.enabled
      ? `${obs.state_meta.history.source}${obs.state_meta.history.episode_first_padding ? " (episode first)" : ""}`
      : "-"},
    {label: "full infer rows", value: vlaFullInferComparison?.policyRows?.length || "-"},
    {label: "first action", value: first.length ? first.map((v) => number(v, 3)).join(", ") : "-"}
  ]);
}

function vlaCurveIndexes(group, schema = vlaActionSchemaKey(), groupIndices = null, dimHint = 0) {
  const serverGroups = groupIndices && typeof groupIndices === "object" ? groupIndices : {};
  const serverIndexes = Object.values(serverGroups).flatMap((values) => Array.isArray(values) ? values.map(Number) : []);
  const dim = Number(dimHint || (serverIndexes.length ? Math.max(...serverIndexes) + 1 : 0) || vlaSchemaDef(schema).action_dim || 16);
  if (Object.keys(serverGroups).length) {
    if (group === "all") return [...Array(dim)].map((_, i) => i);
    const selected = serverGroups[group];
    return Array.isArray(selected) ? selected.map(Number) : [];
  }
  if (schema === "left_arm_8") {
    if (group === "left_arm") return [...Array(7)].map((_, i) => i);
    if (group === "left_gripper") return [7];
    return [...Array(dim)].map((_, i) => i);
  }
  if (schema === "right_arm_8") {
    if (group === "right_arm") return [...Array(7)].map((_, i) => i);
    if (group === "right_gripper") return [7];
    return [...Array(dim)].map((_, i) => i);
  }
  if (schema === "upper_body_20" || schema === "full_20") {
    if (group === "torso") return [...Array(4)].map((_, i) => i);
    if (group === "left_arm") return [...Array(7)].map((_, i) => i + 4);
    if (group === "left_gripper") return [11];
    if (group === "right_arm") return [...Array(7)].map((_, i) => i + 12);
    if (group === "right_gripper") return [19];
    return [...Array(dim)].map((_, i) => i);
  }
  if (group === "left_arm") return [...Array(7)].map((_, i) => i);
  if (group === "left_gripper") return [7];
  if (group === "right_arm") return [...Array(7)].map((_, i) => i + 8);
  if (group === "right_gripper") return [15];
  return [...Array(dim)].map((_, i) => i);
}

function vlaGripperLimit(contract = null) {
  const value = contract?.gripper_action_space || state.vla?.task_metadata?.gripper_action_space || "";
  const match = String(value).match(/^absolute_0_to_(\d+(?:\.\d+)?)$/);
  const limit = match ? Number(match[1]) : 100;
  return Number.isFinite(limit) && limit > 0 ? limit : 100;
}

function vlaGripperIndexes(schema, groupIndices = null) {
  if (groupIndices && typeof groupIndices === "object" && Object.keys(groupIndices).length) {
    return new Set([
      ...(groupIndices.left_gripper || []),
      ...(groupIndices.right_gripper || [])
    ].map(Number));
  }
  if (schema === "left_arm_8" || schema === "right_arm_8") return new Set([7]);
  if (schema === "upper_body_20" || schema === "full_20") return new Set([11, 19]);
  return new Set([7, 15]);
}

function vlaRowsAsLeftArm8(rows, sourceSchema = vlaActionSchemaKey()) {
  if (!Array.isArray(rows) || !rows.length) return [];
  const armIndexes = vlaCurveIndexes("left_arm", sourceSchema);
  const gripperIndexes = vlaCurveIndexes("left_gripper", sourceSchema);
  if (armIndexes.length < 7 || gripperIndexes.length < 1) return [];
  return rows.map((row) => [
    ...armIndexes.slice(0, 7).map((idx) => Number(row?.[idx] || 0)),
    Number(row?.[gripperIndexes[0]] || 0)
  ]);
}

function vlaRowsForComparison(rows, sourceSchema, targetSchema) {
  if (!Array.isArray(rows) || !rows.length) return [];
  if (sourceSchema === targetSchema) return rows.map((row) => row.map((value) => Number(value || 0)));
  if (targetSchema === "left_arm_8") return vlaRowsAsLeftArm8(rows, sourceSchema);
  if ((sourceSchema === "upper_body_20" || sourceSchema === "full_20") && targetSchema === "bimanual_16") {
    return rows.map((row) => [...row.slice(4, 12), ...row.slice(12, 20)].map((value) => Number(value || 0)));
  }
  return rows.map((row) => row.map((value) => Number(value || 0)));
}

function vlaDatasetComparisonRows(startFrame, count, targetSchema) {
  const sourceRows = vlaCurves?.action?.length ? vlaCurves.action : (vlaCurves?.state || []);
  const label = vlaCurves?.action?.length ? "dataset action" : "dataset state";
  const rows = vlaRowsForComparison(sourceRows, vlaActionSchemaKey(), targetSchema).slice(startFrame, startFrame + count);
  return {rows, label};
}

function drawVlaCharts() {
  const playbackRows = vlaCurves?.action?.length ? vlaCurves.action : (vlaCurves?.state || []);
  drawVlaChart($("vlaPlaybackChart"), playbackRows, vlaFrameIndex(), vlaCurves?.action?.length ? "dataset action" : "dataset state", vlaActionSchemaKey());
  const actions = vlaLastInfer?.result?.actions || [];
  const current = vlaLastInfer?.observation?.state ? [vlaLastInfer.observation.state] : [];
  const inferSchema = vlaLastInfer?.result?.action_schema || vlaLastInfer?.observation?.action_schema || vlaRemoteTaskSchemaKey();
  if (vlaFullInferComparison?.policyRows?.length) {
    const comparisonSchema = vlaFullInferComparison.actionSchema || inferSchema;
    const dataset = Array.isArray(vlaFullInferComparison.datasetRows)
      ? {rows: vlaFullInferComparison.datasetRows, label: vlaFullInferComparison.datasetLabel || "MCAP action"}
      : vlaDatasetComparisonRows(
          Number(vlaFullInferComparison.startFrame || 0),
          vlaFullInferComparison.policyRows.length,
          comparisonSchema
        );
    const comparisonContract = {
      action_group_indices: vlaFullInferComparison.actionGroupIndices || {},
      gripper_action_space: vlaFullInferComparison.gripperActionSpace || ""
    };
    drawVlaComparisonChart($("vlaInferChart"), [
      {rows: dataset.rows, label: dataset.label, style: "dataset"},
      {
        rows: vlaFullInferComparison.policyRows,
        label: vlaFullInferComparison.policyLabel || "policy action chunk",
        style: "policy"
      }
    ], comparisonSchema, comparisonContract);
  } else {
    drawVlaChart(
      $("vlaInferChart"), actions.length ? actions : current, 0,
      actions.length ? "action" : "state", inferSchema,
      vlaLastInfer?.result?.task_metadata || state.vla?.task_metadata || null
    );
  }
}

function drawVlaChart(canvas, rows, cursor, label, schema = vlaActionSchemaKey(), contract = null) {
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const width = rect.width;
  const height = rect.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#fbfcfd";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#d7dde3";
  ctx.strokeRect(0.5, 0.5, width - 1, height - 1);
  const groupIndices = contract?.action_group_indices || null;
  const indexes = vlaCurveIndexes($("vlaCurveGroup")?.value || "all", schema, groupIndices, rows?.[0]?.length || 0);
  if (!rows.length) {
    ctx.fillStyle = "#667381";
    ctx.fillText("无曲线数据", 12, 22);
    return;
  }
  const unit = $("vlaCurveUnit")?.value || "deg";
  if (!indexes.length) {
    ctx.fillStyle = "#667381";
    ctx.fillText("当前契约不包含所选控制组", 12, 22);
    return;
  }
  const gripperIndexes = vlaGripperIndexes(schema, groupIndices);
  const gripperLimit = vlaGripperLimit(contract);
  const isGrip = (idx) => gripperIndexes.has(idx);
  const valueAt = (row, idx) => {
    const raw = Number(row[idx] || 0);
    if (isGrip(idx)) return Math.max(0, Math.min(gripperLimit, raw)) / gripperLimit;
    return unit === "deg" ? raw * 180 / Math.PI : raw;
  };
  const flat = rows.flatMap((row) => indexes.map((idx) => valueAt(row, idx)));
  const min = Math.min(...flat);
  const max = Math.max(...flat);
  const span = Math.max(1e-6, max - min);
  const palette = ["#1b679f", "#23785d", "#b42318", "#946200", "#5b4b8a", "#0f766e", "#9f1239", "#475569"];
  indexes.forEach((idx, lineIdx) => {
    ctx.beginPath();
    ctx.strokeStyle = palette[lineIdx % palette.length];
    rows.forEach((row, rowIdx) => {
      const x = rows.length === 1 ? width / 2 : 8 + rowIdx / (rows.length - 1) * (width - 16);
      const y = height - 10 - (valueAt(row, idx) - min) / span * (height - 26);
      if (rowIdx === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
  if (rows.length > 1) {
    const x = 8 + Math.max(0, Math.min(rows.length - 1, cursor)) / (rows.length - 1) * (width - 16);
    ctx.strokeStyle = "#d92d20";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, 4);
    ctx.lineTo(x, height - 4);
    ctx.stroke();
    ctx.lineWidth = 1;
  }
  ctx.fillStyle = "#1d2630";
  ctx.fillText(`${label} | ${unit} | ${number(min, 2)}..${number(max, 2)}`, 10, 18);
}

function drawVlaComparisonChart(canvas, series, schema = "left_arm_8", contract = null) {
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const width = rect.width;
  const height = rect.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#fbfcfd";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#d7dde3";
  ctx.strokeRect(0.5, 0.5, width - 1, height - 1);

  const usable = (series || []).filter((item) => Array.isArray(item.rows) && item.rows.length);
  if (!usable.length) {
    ctx.fillStyle = "#667381";
    ctx.fillText("无对比曲线数据", 12, 22);
    return;
  }
  const groupIndices = contract?.action_group_indices || null;
  const indexes = vlaCurveIndexes($("vlaCurveGroup")?.value || "all", schema, groupIndices, usable[0]?.rows?.[0]?.length || 0);
  if (!indexes.length) {
    ctx.fillStyle = "#667381";
    ctx.fillText("当前契约不包含所选控制组", 12, 22);
    return;
  }
  const unit = $("vlaCurveUnit")?.value || "deg";
  const gripperIndexes = vlaGripperIndexes(schema, groupIndices);
  const gripperLimit = vlaGripperLimit(contract);
  const isGrip = (idx) => gripperIndexes.has(idx);
  const valueAt = (row, idx) => {
    const raw = Number(row?.[idx] || 0);
    if (isGrip(idx)) return Math.max(0, Math.min(gripperLimit, raw)) / gripperLimit;
    return unit === "deg" ? raw * 180 / Math.PI : raw;
  };
  const flat = usable.flatMap((item) => item.rows.flatMap((row) => indexes.map((idx) => valueAt(row, idx))));
  const min = Math.min(...flat);
  const max = Math.max(...flat);
  const span = Math.max(1e-6, max - min);
  const palette = ["#1b679f", "#23785d", "#b42318", "#946200", "#5b4b8a", "#0f766e", "#9f1239", "#475569"];
  usable.forEach((item) => {
    const isDataset = item.style === "dataset";
    ctx.globalAlpha = isDataset ? 0.45 : 1.0;
    ctx.setLineDash(isDataset ? [] : [7, 4]);
    ctx.lineWidth = isDataset ? 1.4 : 2.0;
    indexes.forEach((idx, lineIdx) => {
      ctx.beginPath();
      ctx.strokeStyle = palette[lineIdx % palette.length];
      item.rows.forEach((row, rowIdx) => {
        const x = item.rows.length === 1 ? width / 2 : 8 + rowIdx / (item.rows.length - 1) * (width - 16);
        const y = height - 10 - (valueAt(row, idx) - min) / span * (height - 26);
        if (rowIdx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    });
  });
  ctx.globalAlpha = 1;
  ctx.setLineDash([]);
  ctx.lineWidth = 1;
  ctx.fillStyle = "#1d2630";
  const labels = usable.map((item) => item.label).join(" vs ");
  ctx.fillText(`${labels} | ${unit} | ${number(min, 2)}..${number(max, 2)}`, 10, 18);
  ctx.fillStyle = "#667381";
  ctx.fillText("dataset=实线浅色, policy=虚线深色", 10, height - 10);
}

function renderWorkflow() {
  renderActionTypeSelect();
  renderCodeWorkflows();
  renderActionLists();
  renderActionGroups();
  renderSequenceLists();
  renderWorkflowRuntime();
}

function renderPageActions(page, root) {
  const actions = (state.actions || []).filter((action) => action.source_page === page || action.generated_template && action.source_page === page);
  renderActionButtons(root, actions, false, page);
}

function isLegacyAction(action) {
  return action.migration_warnings !== undefined || (!action.source_page && !action.generated_template);
}

function workflowActions() {
  return (state.actions || []).filter((action) => !isLegacyAction(action));
}

function isActionGroup(sequence) {
  return String(sequence?.kind || "") === "action_group";
}

function workflowSequences() {
  return (state.sequences || []).filter((seq) => !isActionGroup(seq));
}

function actionGroupSequences() {
  return (state.sequences || []).filter(isActionGroup);
}

function actionGroupByIdent(ident) {
  const key = String(ident || "");
  return actionGroupSequences().find((item) => key === String(item.id) || key === String(item.name));
}

function renderActionLists() {
  renderActionButtons($("newActionList"), workflowActions(), true);
  const picker = $("stepActionPicker");
  const current = picker.value;
  picker.innerHTML = "";
  workflowActions().forEach((action) => {
    const opt = document.createElement("option");
    opt.value = action.name || action.id;
    opt.textContent = `${action.name} [${action.type || "joint_targets"}]`;
    picker.appendChild(opt);
  });
  if ([...picker.options].some((opt) => opt.value === current)) picker.value = current;
  fillActionGroupActionPicker();
}

function actionExecuteTimeoutMs(action = null) {
  const params = action?.params || {};
  const args = params.args || {};
  const timeoutS = Number(args.timeout_s ?? params.timeout_s ?? 60);
  return Math.max(30000, Math.ceil((timeoutS + 20) * 1000));
}

function actionExecuteFailed(exc) {
  const message = exc?.message || String(exc || "unknown error");
  if (message.includes("请求超时")) {
    pushWorkflowMessage(`原子动作执行请求超时：${message}。后端可能仍在执行，正在刷新状态。`);
    refreshState();
    return;
  }
  alert(exc);
}

function executeActionRequest(action, nameOverride = "") {
  const payload = {id: action?.id, name: nameOverride || action?.name};
  return post("/api/actions/execute", payload, {
    timeoutMs: actionExecuteTimeoutMs(action),
  }).then((data) => {
    pushWorkflowMessage(data.message || "原子动作执行完成");
    refreshState();
  }).catch(actionExecuteFailed);
}

function renderActionButtons(root, actions, selectable, executePage = null) {
  root.innerHTML = "";
  if (!actions.length) {
    root.innerHTML = `<div class="empty">无动作</div>`;
    return;
  }
  actions.forEach((action) => {
    const button = document.createElement("button");
    button.className = selectedAction?.id === action.id ? "active list-item" : "list-item";
    button.innerHTML = `<strong>${escapeHtml(action.name || action.id)}</strong><span>${escapeHtml(action.description || action.type || "")}</span>`;
    if (!selectable && executePage) {
      button.dataset.lockPage = executePage;
    }
    button.onclick = () => {
      if (selectable) selectAction(action);
      else {
        if (executePage && !requireUnlocked(executePage)) return;
        executeActionRequest(action);
      }
    };
    root.appendChild(button);
  });
}

function fillActionGroupActionPicker() {
  const picker = $("actionGroupActionPicker");
  if (!picker) return;
  const current = picker.value;
  picker.innerHTML = "";
  workflowActions().forEach((action) => {
    const opt = document.createElement("option");
    opt.value = action.name || action.id;
    opt.textContent = `${action.name} [${action.type || "joint_targets"}]`;
    picker.appendChild(opt);
  });
  if ([...picker.options].some((opt) => opt.value === current)) picker.value = current;
}

function actionGroupStep(group = selectedActionGroup) {
  const steps = Array.isArray(group?.steps) ? group.steps : [];
  const step = steps[0];
  if (step && Array.isArray(step.actions)) return step;
  return {actions: [], trigger: "after_end", delay_s: 0, overrides: {}};
}

function actionGroupActionNames(group = selectedActionGroup) {
  const steps = Array.isArray(group?.steps) ? group.steps : [];
  return steps.flatMap((step) => Array.isArray(step?.actions) ? step.actions.map(String) : []);
}

function setActionGroupActionNames(names) {
  ensureSelectedActionGroup();
  selectedActionGroup.steps = names.map((name, index) => ({
    id: selectedActionGroup.steps?.[index]?.id,
    actions: [String(name)],
    trigger: "after_end",
    delay_s: 0,
    overrides: {},
  }));
}

function ensureSelectedActionGroup() {
  if (!selectedActionGroup) {
    selectedActionGroup = {
      name: $("actionGroupName")?.value || "",
      description: $("actionGroupDescription")?.value || "",
      kind: "action_group",
      steps: [{actions: [], trigger: "after_end", delay_s: 0, overrides: {}}],
    };
  }
  selectedActionGroup.kind = "action_group";
  selectedActionGroup.steps = selectedActionGroup.steps || [{actions: [], trigger: "after_end", delay_s: 0, overrides: {}}];
  if (!selectedActionGroup.steps.length) {
    selectedActionGroup.steps.push({actions: [], trigger: "after_end", delay_s: 0, overrides: {}});
  }
  selectedActionGroup.steps[0].actions = Array.isArray(selectedActionGroup.steps[0].actions)
    ? selectedActionGroup.steps[0].actions
    : [];
}

function selectActionGroupForEdit(group) {
  selectedActionGroup = clone(group);
  renderActionGroups();
}

function renderActionGroups() {
  const root = $("actionGroupList");
  if (!root) return;
  const groups = actionGroupSequences();
  root.innerHTML = "";
  if (!groups.length) {
    root.innerHTML = `<div class="empty">无工作组</div>`;
  } else {
    groups.forEach((group) => {
      const row = document.createElement("div");
      row.className = selectedActionGroup?.id === group.id ? "active list-item action-group-item" : "list-item action-group-item";
      const count = actionGroupActionNames(group).length;
      row.innerHTML = `
        <button class="list-item action-group-run" data-action-group-select="${escapeHtml(group.id || group.name)}">
          <strong>${escapeHtml(group.name || group.id)}</strong>
          <span>${escapeHtml(group.description || `${count} 个原子动作`)}</span>
        </button>
        <button data-action-group-edit="${escapeHtml(group.id || group.name)}">编辑</button>`;
      root.appendChild(row);
    });
  }
  renderActionGroupEditor();
}

function renderActionGroupEditor() {
  const name = $("actionGroupName");
  const description = $("actionGroupDescription");
  const actionsRoot = $("actionGroupActions");
  if (!name || !description || !actionsRoot) return;
  if (!selectedActionGroup && actionGroupSequences().length) {
    selectedActionGroup = clone(actionGroupSequences()[0]);
  }
  const group = selectedActionGroup || {name: "", description: "", steps: [{actions: []}]};
  name.value = group.name || "";
  description.value = group.description || "";
  actionsRoot.innerHTML = "";
  const names = actionGroupActionNames(group);
  if (!names.length) {
    actionsRoot.innerHTML = `<div class="empty">未加入动作</div>`;
    return;
  }
  names.forEach((actionName, index) => {
    const action = actionByName(actionName);
    const div = document.createElement("div");
    div.className = "step-card compact-step";
    div.innerHTML = `
      <div class="step-head">
        <button class="list-item action-group-action-run" data-action-group-execute-action="${escapeHtml(actionName)}">
          <strong>${escapeHtml(actionName)}</strong>
        </button>
        <button data-remove-action-group-action="${index}">删除</button>
      </div>`;
    actionsRoot.appendChild(div);
  });
}

function addActionToGroup() {
  const picker = $("actionGroupActionPicker");
  const actionName = picker?.value || "";
  if (!actionName) return;
  setActionGroupActionNames([...actionGroupActionNames(), actionName]);
  renderActionGroupEditor();
}

function removeActionFromGroup(index) {
  const names = actionGroupActionNames();
  names.splice(Number(index), 1);
  setActionGroupActionNames(names);
  renderActionGroupEditor();
}

function clearActionGroup() {
  setActionGroupActionNames([]);
  renderActionGroupEditor();
}

function newActionGroup() {
  selectedActionGroup = {
    name: "",
    description: "",
    kind: "action_group",
    steps: [],
  };
  renderActionGroupEditor();
}

function currentActionGroupPayload() {
  ensureSelectedActionGroup();
  const actions = actionGroupActionNames();
  return {
    id: selectedActionGroup.id,
    name: $("actionGroupName").value.trim(),
    description: $("actionGroupDescription").value.trim(),
    kind: "action_group",
    steps: actions.map((name, index) => ({
      id: selectedActionGroup.steps?.[index]?.id,
      actions: [name],
      trigger: "after_end",
      delay_s: 0,
      overrides: {},
    })),
  };
}

function saveActionGroup() {
  if (!requireUnlocked("workflow")) return;
  try {
    const sequence = currentActionGroupPayload();
    if (!sequence.name) {
      alert("请输入工作组名称");
      return;
    }
    if (!sequence.steps[0].actions.length) {
      alert("请至少加入一个原子动作");
      return;
    }
    post("/api/sequences/save", {sequence}).then((data) => {
      selectedActionGroup = data.sequence;
      refreshState();
    }).catch(alert);
  } catch (exc) {
    alert(exc);
  }
}

function renameActionGroup() {
  if (!requireUnlocked("workflow")) return;
  const currentName = $("actionGroupName").value || selectedActionGroup?.name || "";
  const newName = prompt("新名称", currentName);
  if (!newName) return;
  post("/api/sequences/rename", {
    id: selectedActionGroup?.id,
    name: currentName,
    new_name: newName,
  }).then((data) => {
    selectedActionGroup = data.sequence;
    refreshState();
  }).catch(alert);
}

function deleteActionGroup() {
  if (!requireUnlocked("workflow")) return;
  if (!selectedActionGroup?.id && !$("actionGroupName").value) return;
  const ok = confirm(`删除工作组 ${$("actionGroupName").value || selectedActionGroup?.name || ""}？`);
  if (!ok) return;
  post("/api/sequences/delete", {id: selectedActionGroup?.id, name: $("actionGroupName").value}).then(() => {
    selectedActionGroup = null;
    refreshState();
  }).catch(alert);
}

function codeWorkflowById(id) {
  return (state.code_workflows || []).find((item) => String(item.id) === String(id));
}

function pushWorkflowMessage(message) {
  if (!message) return;
  workflowMessages.push(`${new Date().toLocaleTimeString()} ${message}`);
  workflowMessages = workflowMessages.slice(-30);
  renderCodeWorkflowStatus();
  renderWorkflowRuntime();
}

function setCodeWorkflowPending(pending) {
  codeWorkflowRequestPending = Boolean(pending);
  renderLocks();
  renderCodeWorkflowStatus();
  renderWorkflowRuntime();
}

function cancelCodeWorkflowRequestUi() {
  codeWorkflowRequestGeneration += 1;
  setCodeWorkflowPending(false);
}

function codeWorkflowRequestFailed(exc, operation) {
  const message = exc?.message || String(exc || "unknown error");
  pushWorkflowMessage(`${operation}请求失败：${message}。请求可能已经到达后端，正在刷新状态。`);
  refreshState();
}

async function stopCodeWorkflowNow(label = "代码工作流") {
  cancelCodeWorkflowRequestUi();
  pushWorkflowMessage(`正在发送${label}停止请求`);
  try {
    const data = await post("/api/code_workflows/stop", {}, {timeoutMs: 5000});
    pushWorkflowMessage(data.message || `${label}停止请求已发送`);
  } catch (exc) {
    codeWorkflowRequestFailed(exc, `停止${label}`);
  } finally {
    setCodeWorkflowPending(false);
    refreshState();
  }
}

async function stopWorkflowStack() {
  cancelCodeWorkflowRequestUi();
  pushWorkflowMessage("正在发送工作流停止请求");
  const requests = [
    post("/api/sequences/stop", {}, {timeoutMs: 5000}),
    post("/api/code_workflows/stop", {}, {timeoutMs: 5000}),
    post("/api/control/stop", {}, {timeoutMs: 5000})
  ];
  const results = await Promise.allSettled(requests);
  const messages = [];
  results.forEach((result) => {
    if (result.status === "fulfilled") {
      const message = result.value?.message;
      if (message && !messages.includes(message)) messages.push(message);
    } else {
      const message = result.reason?.message || String(result.reason || "unknown error");
      messages.push(`停止请求失败: ${message}`);
    }
  });
  messages.forEach((message) => pushWorkflowMessage(message));
  setCodeWorkflowPending(false);
  refreshState();
}

async function stopAllControls() {
  cancelCodeWorkflowRequestUi();
  pushWorkflowMessage("正在发送 STOP");
  try {
    const data = await post("/api/control/stop", {}, {timeoutMs: 5000});
    pushWorkflowMessage(data.message || "STOP 已发送");
  } catch (exc) {
    codeWorkflowRequestFailed(exc, "STOP");
  } finally {
    setCodeWorkflowPending(false);
    refreshState();
  }
}

function renderCodeWorkflows() {
  const workflows = state.code_workflows || [];
  const root = $("codeWorkflowList");
  root.innerHTML = "";
  if (!workflows.length) {
    root.innerHTML = `<div class="empty">无代码工作流</div>`;
    $("codeWorkflowParamForm").innerHTML = `<div class="empty">无参数</div>`;
    return;
  }
  if (!selectedCodeWorkflowId || !codeWorkflowById(selectedCodeWorkflowId)) {
    selectedCodeWorkflowId = workflows[0].id;
    codeWorkflowParamsDirty = false;
  }
  workflows.forEach((workflow) => {
    const button = document.createElement("button");
    button.className = selectedCodeWorkflowId === workflow.id ? "active list-item" : "list-item";
    button.innerHTML = `<strong>${escapeHtml(workflow.name || workflow.id)}</strong><span>${escapeHtml(workflow.description || workflow.id)}</span>`;
    button.onclick = () => {
      selectedCodeWorkflowId = workflow.id;
      codeWorkflowParamsDirty = false;
      renderCodeWorkflows();
    };
    root.appendChild(button);
  });
  const selected = codeWorkflowById(selectedCodeWorkflowId);
  if (!selected) return;
  if (!$("codeWorkflowActionName").value && !codeWorkflowParamsDirty) {
    $("codeWorkflowActionName").value = selected.id;
  }
  if (!codeWorkflowParamsDirty) {
    const visibleKeys = Array.isArray(selected.ui_visible_params) && selected.ui_visible_params.length
      ? new Set(selected.ui_visible_params)
      : null;
    const visibleDefaults = visibleKeys
      ? Object.fromEntries(Object.entries(selected.default_params || {}).filter(([key]) => visibleKeys.has(key)))
      : (selected.default_params || {});
    const visibleSchema = visibleKeys
      ? Object.fromEntries(Object.entries(selected.params_schema || {}).filter(([key]) => visibleKeys.has(key)))
      : (selected.params_schema || {});
    $("codeWorkflowParamForm").innerHTML = paramFieldsHtml(
      visibleDefaults,
      visibleDefaults,
      "code",
      false,
      visibleSchema
    );
  }
  renderCodeWorkflowStatus();
}

function currentCodeWorkflowParams() {
  const selected = codeWorkflowById(selectedCodeWorkflowId);
  if (!selected) return {};
  return collectParamsFromForm(
    $("codeWorkflowParamForm"),
    "[data-code-param]",
    selected.default_params || {}
  );
}

function codeWorkflowStatusLines() {
  const selected = codeWorkflowById(selectedCodeWorkflowId);
  const runtime = state.code_workflow_runtime || {};
  const lines = [
    `选中: ${selected?.name || selectedCodeWorkflowId || "-"}`,
    `状态: ${runtime.running ? "running" : (runtime.message || "idle")}`,
    `运行: ${runtime.name || "-"}`,
    `动作: ${runtime.action_name || "-"}`,
    `视频流: ${runtime.stream_label || selected?.video_stream?.label || selected?.video_stream?.key || "-"}`,
    `运行时间: ${runtime.uptime_s ?? "-"} s`,
  ];
  if (workflowMessages.length) {
    lines.push("", ...workflowMessages.slice(-6));
  }
  const log = Array.isArray(runtime.log) ? runtime.log : [];
  if (log.length) {
    lines.push("", ...log.slice(-10));
  }
  return lines;
}

function renderCodeWorkflowStatus() {
  const root = $("codeWorkflowStatusLog");
  if (!root) return;
  root.textContent = codeWorkflowStatusLines().join("\n");
  renderCodeWorkflowPreview(workflowVisionStreamKey());
  const runButton = $("runCodeWorkflow");
  if (runButton) {
    runButton.disabled = codeWorkflowRequestPending || !locks.workflow;
    runButton.textContent = codeWorkflowRequestPending ? "请求中..." : "执行代码工作流";
  }
}

function renderActionTypeSelect() {
  const select = $("actionType");
  if (select.dataset.ready) return;
  select.dataset.ready = "1";
  ACTION_TYPES.forEach((type) => {
    const opt = document.createElement("option");
    opt.value = type;
    opt.textContent = type;
    select.appendChild(opt);
  });
}

function paramFieldsHtml(params, values, datasetPrefix, disabled = false, schema = {}) {
  const keys = Object.keys(params || {});
  if (!keys.length) return `<div class="empty">无参数</div>`;
  const disabledAttr = disabled ? "disabled" : "";
  return keys.map((key) => {
    const base = params[key];
    const value = Object.prototype.hasOwnProperty.call(values || {}, key) ? values[key] : base;
    const kind = paramKind(base);
    const fieldSchema = schema?.[key] || {};
    const label = fieldSchema.label || key;
    const common = `data-${datasetPrefix}-param="${escapeHtml(key)}" data-kind="${kind}"`;
    if (kind === "boolean") {
      return `<label>${escapeHtml(label)}<input ${common} type="checkbox" ${value ? "checked" : ""} ${disabledAttr}></label>`;
    }
    if (kind === "number") {
      return `<label>${escapeHtml(label)}<input ${common} type="number" step="any" value="${escapeHtml(formatParamValue(value))}" ${disabledAttr}></label>`;
    }
    if (kind === "array") {
      return `<label>${escapeHtml(label)}<input ${common} data-subtype="${arraySubtype(base)}" value="${escapeHtml(formatParamValue(value))}" ${disabledAttr}></label>`;
    }
    if (kind === "string") {
      const options = Array.isArray(fieldSchema.options) ? fieldSchema.options : [];
      if (options.length) {
        const normalizedOptions = options.map((option) => (
          option && typeof option === "object"
            ? {value: String(option.value ?? ""), label: String(option.label ?? option.value ?? "")}
            : {value: String(option), label: String(option)}
        ));
        const currentValue = formatParamValue(value);
        const optionHtml = normalizedOptions.map((option) => (
          `<option value="${escapeHtml(option.value)}" ${option.value === currentValue ? "selected" : ""}>${escapeHtml(option.label)}</option>`
        )).join("");
        return `<label>${escapeHtml(label)}<select ${common} ${disabledAttr}>${optionHtml}</select></label>`;
      }
      return `<label>${escapeHtml(label)}<input ${common} value="${escapeHtml(formatParamValue(value))}" ${disabledAttr}></label>`;
    }
    return `
      <div class="complex-param">
        <strong>${escapeHtml(label)}</strong>
        <span>复杂参数需在诊断页打开 JSON 模式后修改</span>
      </div>`;
  }).join("");
}

function collectParamsFromForm(root, selector, fallback = {}) {
  const params = clone(fallback);
  root.querySelectorAll(selector).forEach((input) => {
    const key = input.dataset.actionParam || input.dataset.stepParam || input.dataset.codeParam;
    params[key] = parseParamValue(input);
  });
  return params;
}

function renderActionParamForm(params) {
  const root = $("actionParamForm");
  if (!root) return;
  root.innerHTML = paramFieldsHtml(params || {}, params || {}, "action");
}

function selectAction(action) {
  selectedAction = action;
  editingActionDirty = false;
  $("actionName").value = action.name || "";
  $("actionDescription").value = action.description || "";
  $("actionType").value = action.type || "joint_targets";
  $("actionParams").value = JSON.stringify(action.params || {}, null, 2);
  renderActionParamForm(action.params || {});
  renderActionLists();
}

function editorActionPayload() {
  return {
    id: selectedAction?.id,
    name: $("actionName").value.trim(),
    description: $("actionDescription").value.trim(),
    type: $("actionType").value,
    source_page: selectedAction?.source_page || "workflow",
    generated_template: Boolean(selectedAction?.generated_template),
    params: jsonMode
      ? JSON.parse($("actionParams").value || "{}")
      : collectParamsFromForm($("actionParamForm"), "[data-action-param]", selectedAction?.params || {})
  };
}

function defaultActionName(prefix) {
  return `${prefix}_${new Date().toISOString().replace(/[-:TZ.]/g, "").slice(0, 14)}`;
}

function renderSequenceLists() {
  const root = $("sequenceList");
  root.innerHTML = "";
  workflowSequences().forEach((seq) => {
    const btn = document.createElement("button");
    btn.className = selectedSequence?.id === seq.id ? "active list-item" : "list-item";
    btn.innerHTML = `<strong>${escapeHtml(seq.name)}</strong><span>${escapeHtml(seq.description || `${(seq.steps || []).length} steps`)}</span>`;
    btn.onclick = () => selectSequence(seq);
    root.appendChild(btn);
  });
  if (!workflowSequences().length) root.innerHTML = `<div class="empty">无工作流</div>`;
  if (!editingSequenceDirty && selectedSequence) {
    renderSequenceEditor();
  }
}

function workflowVisionStreamKey() {
  if (activePage() !== "workflow") return "";
  const codeRuntime = state.code_workflow_runtime || {};
  if (codeRuntime.running) return codeRuntime.stream_key || "code_workflow";
  if (codeRuntime.stream_key && (codeRuntime.message || (codeRuntime.log || []).length || codeRuntime.action_name)) {
    return codeRuntime.stream_key;
  }
  const selectedWorkflow = codeWorkflowById(selectedCodeWorkflowId);
  if (selectedWorkflow?.video_stream?.key) return selectedWorkflow.video_stream.key;
  const runtime = state.sequence_runtime || {};
  if (!runtime.running) return "";
  const action = actionByName(runtime.action_name);
  const type = action?.type || "";
  if (["yolo_open", "latch_yolo", "read_yolo_detections", "yolo_close"].includes(type)) return "yolo";
  if (["foundation_pose_open", "foundation_pose_close", "read_foundation_pose"].includes(type)) return "foundation";
  if (["aruco_pose_open", "aruco_pose_close", "aruco_fine_enable", "aruco_fine_disable"].includes(type)) return "aruco";
  if (state.foundation?.running) return "foundation";
  if (state.aruco?.box_pose_running) return "aruco";
  if (state.vision_worker?.running) return "yolo";
  return "";
}

function setStreamImage(img, key, selectionKey) {
  if (!img) return selectionKey;
  if (key !== selectionKey) {
    selectionKey = key;
    if (key) connectCameraImageStream(img, key);
    else img.removeAttribute("src");
  }
  img.hidden = !key;
  return selectionKey;
}

function renderCodeWorkflowPreview(key) {
  const img = $("codeWorkflowVideo");
  const empty = $("codeWorkflowVideoEmpty");
  const frame = $("codeWorkflowVideoFrame");
  if (!img || !empty || !frame) return;
  const runtime = state.code_workflow_runtime || {};
  const logLength = Array.isArray(runtime.log) ? runtime.log.length : 0;
  const marker = key ? `${key}:${runtime.finished_at || 0}:${runtime.message || ""}:${logLength}` : "";
  if (marker !== codeWorkflowPreviewSelection) {
    codeWorkflowPreviewSelection = marker;
    if (key) img.src = `/snapshot/${key}?t=${Date.now()}`;
    else img.removeAttribute("src");
  }
  img.hidden = !key;
  empty.hidden = Boolean(key);
  frame.classList.toggle("empty-stream", !key);
}

function renderWorkflowRuntime() {
  const key = workflowVisionStreamKey();
  const img = $("workflowVideo");
  const empty = $("workflowVideoEmpty");
  const frame = $("workflowVideoFrame");
  const title = $("workflowVideoTitle");
  workflowStreamSelection = setStreamImage(img, key, workflowStreamSelection);
  empty.hidden = Boolean(key);
  frame.classList.toggle("empty-stream", !key);
  renderCodeWorkflowPreview(key);
  const codeRuntime = state.code_workflow_runtime || {};
  const selectedCodeWorkflow = codeWorkflowById(selectedCodeWorkflowId);
  const selectedStream = selectedCodeWorkflow?.video_stream || {};
  title.textContent = key === "foundation"
    ? "FoundationPose 视频"
    : key === "yolo"
      ? "YOLO 视频"
      : key === "aruco"
      ? "ArUco 视频"
      : key === (codeRuntime.stream_key || selectedStream.key || "code_workflow")
      ? (codeRuntime.stream_label || selectedStream.label || "代码工作流视频")
      : "识别视频";

  const runtime = state.sequence_runtime || {};
  const sequenceLines = [
    `状态: ${runtime.running ? "running" : (runtime.message || "idle")}`,
    `工作流: ${runtime.name || "-"}`,
    `步骤: ${runtime.step_index || "-"}`,
    `动作: ${runtime.action_name || "-"}`,
    `运行时间: ${runtime.uptime_s ?? "-"} s`,
  ];
  const lines = [];
  const hasCodeRuntime = codeWorkflowRequestPending || codeRuntime.running || codeRuntime.message || (codeRuntime.log || []).length;
  if (hasCodeRuntime) {
    lines.push(...codeWorkflowStatusLines(), "", ...sequenceLines);
  } else {
    lines.push(...sequenceLines);
    if (workflowMessages.length) {
      lines.push("", ...workflowMessages.slice(-10));
    }
  }
  const log = Array.isArray(runtime.log) ? runtime.log : [];
  if (log.length) {
    lines.push("", ...log.slice(-12));
  }
  const codeLog = Array.isArray(codeRuntime.log) ? codeRuntime.log : [];
  if (!hasCodeRuntime && codeLog.length) {
    lines.push("", ...codeLog.slice(-12));
  }
  $("continueSequence").disabled = !(runtime.running && runtime.message === "waiting_operator");
  $("workflowRuntimeLog").textContent = lines.join("\n");
}

function selectSequence(seq) {
  selectedSequence = JSON.parse(JSON.stringify(seq));
  editingStepParams = new Set();
  editingSequenceDirty = false;
  $("sequenceName").value = selectedSequence.name || "";
  $("sequenceDescription").value = selectedSequence.description || "";
  renderSequenceEditor();
  renderSequenceLists();
}

function currentSequenceFromEditor() {
  const raw = $("sequenceJson").value.trim();
  let steps = clone(selectedSequence?.steps || []);
  if (jsonMode && raw) {
    const parsed = JSON.parse(raw);
    steps = Array.isArray(parsed) ? parsed : parsed.steps || [];
  }
  return {
    id: selectedSequence?.id,
    name: $("sequenceName").value.trim(),
    description: $("sequenceDescription").value.trim(),
    steps
  };
}

function sequenceSaveAsPayload() {
  const sequence = currentSequenceFromEditor();
  const fallback = sequence.name ? `${sequence.name}_copy` : defaultActionName("workflow");
  const name = prompt("另存为工作流名称", fallback);
  if (!name) return null;
  return {
    name: name.trim(),
    description: sequence.description,
    steps: clone(sequence.steps || [])
  };
}

function actionByName(name) {
  return (state.actions || []).find((action) => (
    String(action.name || "") === String(name) || String(action.id || "") === String(name)
  ));
}

function syncSequenceJsonFromSelected() {
  $("sequenceJson").value = JSON.stringify(selectedSequence?.steps || [], null, 2);
}

function stepParamEditKey(step, index, actionName) {
  return `${step?.id || index}:${actionName}`;
}

function renderSequenceEditor() {
  const steps = selectedSequence?.steps || [];
  syncSequenceJsonFromSelected();
  const root = $("sequenceSteps");
  root.innerHTML = "";
  steps.forEach((step, index) => {
    const div = document.createElement("div");
    div.className = "step-card";
    const actionNames = Array.isArray(step.actions) ? step.actions.map(String) : [];
    const editors = actionNames.map((name) => {
      const action = actionByName(name);
      const baseParams = action?.params || {};
      const override = step.overrides && typeof step.overrides === "object" ? step.overrides[name] || {} : {};
      const editKey = stepParamEditKey(step, index, name);
      const editing = editingStepParams.has(editKey);
      const disabledAttr = editing ? "" : "disabled";
      return `
        <div class="step-action-editor">
          <div class="step-action-title">
            <span>${escapeHtml(name)} <small>${escapeHtml(action?.type || "unknown")}</small></span>
            <div class="row wrap">
              <button data-edit-step-params="${index}" data-action-name="${escapeHtml(name)}" ${editing ? "disabled" : ""}>修改</button>
              <button data-save-step-params="${index}" data-action-name="${escapeHtml(name)}" ${editing ? "" : "disabled"}>保存参数</button>
              <button data-load-step-params="${index}" data-action-name="${escapeHtml(name)}" ${disabledAttr}>载入原子动作参数</button>
              <button data-clear-step-params="${index}" data-action-name="${escapeHtml(name)}" ${disabledAttr}>清空参数</button>
            </div>
          </div>
          <div class="param-grid step-param-grid" data-step-param-form="${index}" data-action-name="${escapeHtml(name)}">
            ${paramFieldsHtml(baseParams, {...baseParams, ...override}, "step", !editing)}
          </div>
          <textarea class="json-only" data-step-override="${index}" data-action-name="${escapeHtml(name)}" rows="5" placeholder="参数覆盖 JSON" ${disabledAttr}>${escapeHtml(JSON.stringify(override, null, 2))}</textarea>
        </div>`;
    }).join("");
    div.innerHTML = `
      <div class="step-head">
        <strong>Step ${index + 1}</strong>
        <span>${escapeHtml(actionNames.join(", ") || "-")}</span>
        <div class="row wrap step-controls">
          <button data-insert-before-step="${index}">前插</button>
          <button data-insert-after-step="${index}">后插</button>
          <button data-move-step="${index}" data-move-dir="-1">上移</button>
          <button data-move-step="${index}" data-move-dir="1">下移</button>
          <button data-remove-step="${index}">删除</button>
        </div>
      </div>
      ${editors || `<div class="empty">无动作</div>`}`;
    root.appendChild(div);
  });
}

function makeSequenceStep(actionName) {
  return {
    id: `${Date.now()}_${Math.random().toString(16).slice(2, 8)}`,
    actions: [actionName],
    trigger: "after_end",
    delay_s: 0,
    overrides: {}
  };
}

function ensureSelectedSequence() {
  if (!selectedSequence) {
    selectedSequence = {name: "", description: "", steps: []};
  }
  selectedSequence.steps = selectedSequence.steps || [];
}

function addSequenceStep() {
  ensureSelectedSequence();
  insertSequenceStep(selectedSequence.steps.length);
}

function insertSequenceStep(index) {
  const actionName = $("stepActionPicker").value;
  if (!actionName) return;
  ensureSelectedSequence();
  const safeIndex = Math.max(0, Math.min(Number(index), selectedSequence.steps.length));
  selectedSequence.steps.splice(safeIndex, 0, makeSequenceStep(actionName));
  editingStepParams = new Set();
  editingSequenceDirty = true;
  renderSequenceEditor();
}

function moveSequenceStep(index, direction) {
  if (!selectedSequence?.steps) return;
  const from = Number(index);
  const to = from + Number(direction);
  if (from < 0 || from >= selectedSequence.steps.length || to < 0 || to >= selectedSequence.steps.length) return;
  const [step] = selectedSequence.steps.splice(from, 1);
  selectedSequence.steps.splice(to, 0, step);
  editingStepParams = new Set();
  editingSequenceDirty = true;
  renderSequenceEditor();
}

function applyStepOverride(index, actionName, text) {
  return applyStepOverrideObject(index, actionName, JSON.parse(text || "{}"));
}

function applyStepOverrideObject(index, actionName, value) {
  const step = selectedSequence?.steps?.[index];
  if (!step) return;
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new Error("参数覆盖必须是 JSON object");
  }
  step.overrides = step.overrides && typeof step.overrides === "object" ? step.overrides : {};
  if (Object.keys(value).length) step.overrides[actionName] = value;
  else delete step.overrides[actionName];
  editingSequenceDirty = true;
  syncSequenceJsonFromSelected();
}

function syncSequenceFromJson() {
  try {
    const seq = currentSequenceFromEditor();
    selectedSequence = {...(selectedSequence || {}), ...seq};
    editingSequenceDirty = true;
  } catch {
    // Keep invalid JSON in the editor until the user saves/validates.
  }
}

function renderDiagnostics() {
  const topics = state.topics || {};
  summary($("topicsSummary"), [
    {label: "feedback", value: Object.keys(topics.robot_feedback || {}).length},
    {label: "targets", value: Object.keys(topics.robot_targets || {}).length},
    {label: "chassis", value: topics.chassis_target || "-"},
    {label: "battery", value: topics.battery || "-"}
  ]);
  const diag = state.diagnostics || {};
  summary($("diagnosticsSummary"), [
    {label: "TF checks", value: (diag.tf || []).length},
    {label: "images", value: Object.keys(diag.topic_health?.image_age_s || {}).length},
    {label: "battery", value: diag.topic_health?.battery?.status || "-"},
    {label: "head depth", value: diag.topic_health?.head_depth?.status || "-"}
  ]);
  $("rawStateJson").textContent = jsonMode ? JSON.stringify(state, null, 2) : "";
}

function setupEvents() {
  renderEeAxisSegments(EE_AXIS_EXAMPLE_SEGMENTS);
  $("token").value = token();
  $("saveToken").onclick = () => {
    localStorage.setItem("r1pro.token", $("token").value.trim());
    alert("token 已保存");
  };
  $("stopAll").onclick = stopAllControls;
  document.querySelectorAll(".nav").forEach((button) => {
    button.onclick = () => {
      document.querySelectorAll(".nav").forEach((nav) => nav.classList.remove("active"));
      document.querySelectorAll(".page").forEach((page) => page.classList.remove("active"));
      button.classList.add("active");
      $(button.dataset.page).classList.add("active");
      $("pageTitle").textContent = PAGE_TITLES[button.dataset.page] || button.textContent;
      syncVisionStreams();
      renderWorkflowRuntime();
      renderStreamClicks();
      if (button.dataset.page === "navigation") fetchNavigationMap(false);
      drawNavigationMap();
    };
  });
  window.addEventListener("resize", () => {
    streamSlots().forEach((slot) => updateStreamClickMarker(slot, state.vision?.click_result || {}));
    drawVlaCharts();
  });
  document.body.addEventListener("change", (event) => {
    const page = event.target.dataset.lockToggle;
    if (page) {
      locks[page] = event.target.checked;
      localStorage.setItem("r1pro.locks", JSON.stringify(locks));
      renderLocks();
    }
  });
  $("controlProfilePanel")?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-control-profile]");
    if (!button) return;
    const profile = button.dataset.controlProfile;
    if (!profile || profile === (state.active_control_profile || "real")) return;
    post("/api/control_profile/select", {profile}).then((data) => {
      state.active_control_profile = data.active_control_profile || profile;
      state.control_profiles = data.control_profiles || state.control_profiles;
      lockAllPages();
      refreshState();
    }).catch(alert);
  });
  $("jointCapture").onclick = captureJoints;
  $("jointSelectAll").onclick = () => setJointSelection("all");
  $("jointInvertSelection").onclick = () => setJointSelection("invert");
  $("jointClearSelection").onclick = () => setJointSelection("none");
  $("jointSendSelected").onclick = () => {
    if (!requireUnlocked("joints")) return;
    post("/api/control/joint_targets", {targets: selectedJointTargets(), timeout_s: 3}).catch(alert);
  };
  $("jointSaveSelected").onclick = () => {
    if (!requireUnlocked("joints")) return;
    const name = prompt("动作名称", defaultActionName("joint"));
    if (!name) return;
    const action = {
      name,
      description: "中文备注：从关节控制页保存的选中关节/夹爪目标。",
      type: "joint_targets",
      source_page: "joints",
      params: {targets: selectedJointTargets(), timeout_s: 3}
    };
    post("/api/actions/save", {action}).then(refreshState).catch(alert);
  };
  $("eeCopyCurrent").onclick = copyEeCurrent;
  $("eeExecutePose").onclick = () => {
    if (!requireUnlocked("ee")) return;
    post("/api/control/ee_plan", eePosePayload()).catch(alert);
  };
  $("eeSavePose").onclick = () => saveActionFromPage("ee", "ee_pose", "ee_plan", "中文备注：末端 6D 绝对位姿原子动作，左右手通过 arm 参数配置。", eePosePayload());
  $("eeExecuteRelative").onclick = () => {
    if (!requireUnlocked("ee")) return;
    post("/api/control/ee_relative", eeRelativePayload()).catch(alert);
  };
  $("eeSaveRelative").onclick = () => saveActionFromPage("ee", "ee_relative", "ee_plan_relative", "中文备注：末端相对运动原子动作，左右手通过 arm 参数配置。", eeRelativePayload());
  $("eeJogButtons").addEventListener("click", (event) => {
    const btn = event.target.closest("[data-ee-jog]");
    if (!btn || !requireUnlocked("ee")) return;
    const spec = JSON.parse(btn.dataset.eeJog);
    jogEe(spec.xyz, spec.rpy);
  });
  $("eeAxisJogButtons").addEventListener("click", (event) => {
    const axisBtn = event.target.closest("[data-ee-axis-jog]");
    if (axisBtn) jogEeAxis(Number(axisBtn.dataset.eeAxisJog));
  });
  $("eeAxisSegmentList").addEventListener("click", (event) => {
    const remove = event.target.closest("[data-axis-segment-remove]");
    if (remove) {
      remove.closest(".axis-segment-row")?.remove();
      invalidateEeAxisValidation();
    }
  });
  $("eeAxisSegmentList").addEventListener("input", invalidateEeAxisValidation);
  $("eeAxisAddSegment").onclick = addEeAxisSegment;
  $("eeAxisLoadExample").onclick = () => {
    $("eeAxisOffsetLateral").value = "0";
    $("eeAxisOffsetVertical").value = "0.014";
    $("eeAxisRotationDirection").value = "-1";
    renderEeAxisSegments(EE_AXIS_EXAMPLE_SEGMENTS);
    invalidateEeAxisValidation();
    $("eeAxisSequenceStatus").textContent = "已载入反向示例：实际 0→−100° @ 5°/s，−100→−160° @ 2°/s，总时长 50s。";
  };
  ["eeArm", "eeAxisOffsetLateral", "eeAxisOffsetVertical", "eeAxisRotationDirection"].forEach((id) => {
    $(id).addEventListener("input", invalidateEeAxisValidation);
    $(id).addEventListener("change", invalidateEeAxisValidation);
  });
  $("eeAxisValidateSequence").onclick = () => runEeAxisSequence(true);
  $("eeAxisRunSequence").onclick = () => runEeAxisSequence(false);
  $("eeAxisStopSequence").onclick = () => {
    post("/api/control/ee_axial_stop", {arm: $("eeArm").value})
      .then((data) => { $("eeAxisSequenceStatus").textContent = data.message || "持续旋转已停止"; })
      .catch(alert);
  };
  setupChassisEvents();
  setupNavigationEvents();
  setupVisionEvents();
  setupVlaEvents();
  setupWorkflowEvents();
  document.addEventListener("keydown", onKeyDown);
  document.addEventListener("keyup", onKeyUp);
}

function saveActionFromPage(page, prefix, type, description, params, sourcePage = page) {
  if (!requireUnlocked(page)) return;
  const name = prompt("动作名称", defaultActionName(prefix));
  if (!name) return;
  post("/api/actions/save", {action: {name, description, type, source_page: sourcePage, params}})
    .then(refreshState).catch(alert);
}

function jogEe(xyzSigns, rpySigns) {
  const linear = Number($("eeStepLinear").value || 0.01);
  const angular = Number($("eeStepAngular").value || 2);
  post("/api/control/ee_step", {
    arm: $("eeArm").value,
    delta_xyz_m: xyzSigns.map((v) => v * linear),
    delta_rpy_deg: rpySigns.map((v) => v * angular),
    timeout_s: Number($("eeStepDuration").value || 0.6)
  }).catch(alert);
}

function renderEeAxisSegments(segments) {
  $("eeAxisSegmentList").innerHTML = (segments || []).map((segment) => `
    <div class="axis-segment-row">
      <input class="axis-segment-start" type="number" step="0.1" value="${Number(segment.start_deg)}">
      <input class="axis-segment-end" type="number" step="0.1" value="${Number(segment.end_deg)}">
      <input class="axis-segment-speed" type="number" min="0.1" step="0.1" value="${Number(segment.speed_deg_s)}">
      <button type="button" data-axis-segment-remove>删除</button>
    </div>
  `).join("");
}

function invalidateEeAxisValidation() {
  eeAxisValidationToken = "";
  const status = $("eeAxisSequenceStatus");
  if (status) status.textContent = "设置已改变，请先点击“仅校验轨迹”；校验通过后才能执行。";
}

function eeAxisSegments() {
  return Array.from(document.querySelectorAll(".axis-segment-row")).map((row) => ({
    start_deg: Number(row.querySelector(".axis-segment-start").value),
    end_deg: Number(row.querySelector(".axis-segment-end").value),
    speed_deg_s: Number(row.querySelector(".axis-segment-speed").value)
  }));
}

function addEeAxisSegment() {
  const segments = eeAxisSegments();
  const previous = segments[segments.length - 1] || {end_deg: 0, speed_deg_s: 5};
  segments.push({
    start_deg: Number(previous.end_deg),
    end_deg: Number(previous.end_deg) + 20,
    speed_deg_s: Number(previous.speed_deg_s) || 5
  });
  renderEeAxisSegments(segments);
  invalidateEeAxisValidation();
}

function eeAxisSequencePayload(dryRun) {
  const payload = {
    arm: $("eeArm").value,
    axis_offset_lateral_m: Number($("eeAxisOffsetLateral").value || 0),
    axis_offset_vertical_m: Number($("eeAxisOffsetVertical").value || 0),
    axis_rotation_direction: Number($("eeAxisRotationDirection").value || -1),
    waypoint_step_deg: 2,
    segments: eeAxisSegments(),
    dry_run: Boolean(dryRun)
  };
  if (!dryRun) payload.validation_token = eeAxisValidationToken;
  return payload;
}

function runEeAxisSequence(dryRun) {
  if (!requireUnlocked("ee")) return;
  if (!dryRun && !eeAxisValidationToken) {
    $("eeAxisSequenceStatus").textContent = "请先点击“仅校验轨迹”，校验通过后才能执行。";
    alert("请先手动校验轨迹，校验通过后才能执行");
    return;
  }
  const payload = eeAxisSequencePayload(dryRun);
  const values = [payload.axis_offset_lateral_m, payload.axis_offset_vertical_m, payload.axis_rotation_direction];
  payload.segments.forEach((segment) => values.push(segment.start_deg, segment.end_deg, segment.speed_deg_s));
  if (!payload.segments.length || !values.every(Number.isFinite)) {
    alert("持续旋转编排必须包含有限数字");
    return;
  }
  const duration = payload.segments.reduce((sum, segment) => (
    sum + Math.abs(segment.end_deg - segment.start_deg) / segment.speed_deg_s
  ), 0);
  if (!dryRun && !confirm(`将执行约 ${number(duration, 1)}s 的连续绕 L 旋转，确认开始？`)) return;
  $("eeAxisSequenceStatus").textContent = dryRun ? "正在离线校验整条轨迹…" : "正在规划并启动持续旋转…";
  post("/api/control/ee_axial_sequence", payload, {timeoutMs: 30000}).then((data) => {
    if (dryRun) {
      eeAxisValidationToken = String(data.validation_token || "");
      if (!eeAxisValidationToken) throw new Error("服务器未返回校验凭证");
    } else {
      eeAxisValidationToken = "";
    }
    $("eeAxisSequenceStatus").textContent = data.message || (dryRun ? "轨迹校验通过" : "持续旋转已启动");
    if (!dryRun) {
      const nextLateral = Number(data.next_offset_lateral_m);
      const nextVertical = Number(data.next_offset_vertical_m);
      if (Number.isFinite(nextLateral) && Number.isFinite(nextVertical)) {
        $("eeAxisOffsetLateral").value = number(nextLateral, 6);
        $("eeAxisOffsetVertical").value = number(nextVertical, 6);
      }
    }
  }).catch((error) => {
    $("eeAxisSequenceStatus").textContent = `持续旋转失败：${error.message}`;
    alert(error);
  });
}

function jogEeAxis(direction) {
  if (!requireUnlocked("ee")) return;
  const angular = Math.abs(Number($("eeAxisStepAngular").value || 2));
  if (!Number.isFinite(angular) || angular <= 0) {
    alert("绕L步长必须是大于 0 的数字");
    return;
  }
  const maxAxisSpeed = Number($("eeAxisMaxSpeed").value || 10);
  if (!Number.isFinite(maxAxisSpeed) || maxAxisSpeed <= 0) {
    alert("最大绕L速度必须是大于 0 的数字");
    return;
  }
  const lateralInput = $("eeAxisOffsetLateral");
  const verticalInput = $("eeAxisOffsetVertical");
  const lateralOffset = Number(lateralInput.value || 0);
  const verticalOffset = Number(verticalInput.value || 0);
  if (![lateralOffset, verticalOffset].every(Number.isFinite)) {
    alert("L 偏移必须是有限数字");
    return;
  }
  const delta = Math.sign(direction) * angular;
  $("eeAxisStatus").textContent = `正在执行绕 L ${delta > 0 ? "+" : ""}${number(delta, 2)}°，限速 ${number(maxAxisSpeed, 2)}°/s`;
  post("/api/control/ee_axial_step", {
    arm: $("eeArm").value,
    delta_angle_deg: delta,
    max_axis_speed_deg_s: maxAxisSpeed,
    axis_offset_lateral_m: lateralOffset,
    axis_offset_vertical_m: verticalOffset,
    timeout_s: Number($("eeStepDuration").value || 0.6)
  }).then((data) => {
    const nextLateral = Number(data.ee_axis?.next_offset_lateral_m);
    const nextVertical = Number(data.ee_axis?.next_offset_vertical_m);
    if (Number.isFinite(nextLateral) && Number.isFinite(nextVertical)) {
      lateralInput.value = number(nextLateral, 6);
      verticalInput.value = number(nextVertical, 6);
    }
    const duration = Number(data.ee_axis?.motion_duration_s);
    const durationText = Number.isFinite(duration) ? `，渐变 ${number(duration, 2)}s` : "";
    $("eeAxisStatus").textContent = `已发送绕 L ${delta > 0 ? "+" : ""}${number(delta, 2)}°${durationText}`;
  }).catch((error) => {
    $("eeAxisStatus").textContent = `绕 L 失败：${error.message}`;
    alert(error);
  });
}

function setupChassisEvents() {
  $("drivePad").addEventListener("pointerdown", (event) => {
    const btn = event.target.closest("[data-drive]");
    if (!btn || !requireUnlocked("chassis")) return;
    const [x, y, w] = btn.dataset.drive.split(",").map(Number);
    sendChassisHold(x, y, w);
  });
  ["pointerup", "pointerleave", "pointercancel"].forEach((name) => {
    $("drivePad").addEventListener(name, stopChassisHold);
  });
  $("chassisExecuteAction").onclick = () => {
    if (!requireUnlocked("chassis")) return;
    const payload = chassisActionPayload();
    post("/api/control/chassis", {
      vx: payload.vx_mps,
      vy: payload.vy_mps,
      omega: payload.omega_radps,
      duration_s: payload.duration_s
    }).catch(alert);
  };
  $("chassisSaveAction").onclick = () => saveActionFromPage("chassis", "chassis_velocity", "chassis_velocity", "中文备注：底盘速度按持续时间下发，到时自动停止。", chassisActionPayload());
  window.addEventListener("blur", () => stopChassisHold());
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopChassisHold();
  });
}

function sendChassisHold(x, y, w) {
  if (!state.navigation?.manual_control_allowed) {
    alert("当前不是手动底盘控制模式");
    return;
  }
  stopChassisHold(false);
  const send = () => post("/api/control/chassis", {
    ...chassisPayloadFromScalars(x, y, w, 0.05),
    manual_hold: true
  }).catch(console.warn);
  send();
  chassisHoldTimer = setInterval(send, 50);
}

function stopChassisHold(publishStop = true) {
  if (chassisHoldTimer) clearInterval(chassisHoldTimer);
  chassisHoldTimer = null;
  chassisKeyboardActive = false;
  if (publishStop && locks.chassis) post("/api/navigation/stop", {}).catch(console.warn);
}

function setupVisionEvents() {
  streamSlots().forEach((slot) => {
    initStreamSavePath(slot);
    $(`streamSelect${slot}`).onchange = (event) => setStreamSelection(slot, event.target.value);
    $(`streamConnect${slot}`).onclick = () => requireUnlocked("camera") && connectStream(slot);
    $(`streamDisconnect${slot}`).onclick = () => disconnectStream(slot);
    $(`streamCapture${slot}`).onclick = () => captureStream(slot);
    $(`toggleStreamClick${slot}`).onclick = () => toggleStreamClick(slot);
    $(`streamImage${slot}`).onclick = (event) => requestStreamClick(slot, event);
    $(`streamImage${slot}`).onload = () => renderStreamClick(slot);
  });
  $("goCameraFromRecognition").onclick = () => {
    document.querySelector('.nav[data-page="camera"]')?.click();
  };
  $("headDepthProcessorEnable").onchange = async (event) => {
    const checkbox = event.currentTarget;
    const backendRequested = headDepthProcessor().requested === true;
    if (!requireUnlocked("camera")) {
      checkbox.checked = backendRequested;
      return;
    }
    const enabled = checkbox.checked === true;
    headDepthProcessorPending = true;
    renderLocks();
    try {
      await post("/api/vision/set_head_depth", {enabled});
      await refreshState();
    } catch (exc) {
      checkbox.checked = backendRequested;
      alert(exc.message || String(exc));
      await refreshState();
    } finally {
      headDepthProcessorPending = false;
      checkbox.checked = headDepthProcessor().requested === true;
      renderHeadDepthProcessor();
      renderStreamClicks();
      renderLocks();
    }
  };
  $("startFullRecognition").onclick = () => {
    const switching = Boolean(state.recognition_workflow?.active)
      && recognitionSelectionDiffersFromWorkflow();
    runRecognitionControl(
      "startFullRecognition",
      switching ? "切换识别" : "启动完整识别",
      () => post(
        switching ? "/api/recognition/reregister" : "/api/recognition/start_full",
        fullRecognitionPayload()
      ),
      () => {
        state.recognition_workflow = {...(state.recognition_workflow || {}), busy: true, active: false, phase: "starting_yolo"};
      }
    );
  };
  $("startYoloOnly").onclick = () => runRecognitionControl(
    "startYoloOnly",
    "启动 YOLO",
    () => post("/api/vision/start_yolo", standaloneYoloPayload()),
    (result) => {
      state.vision_worker = {...(state.vision_worker || {}), running: true};
      if (result?.config) state.vision = {...(state.vision || {}), config: result.config};
    }
  );
  $("stopYoloOnly").onclick = () => runRecognitionControl(
    "stopYoloOnly",
    "停止 YOLO",
    () => post("/api/vision/stop_yolo", {}),
    () => {
      state.vision_worker = {...(state.vision_worker || {}), running: false};
    }
  );
  $("reregisterRecognition").onclick = () => runRecognitionControl(
    "reregisterRecognition",
    "重新注册",
    () => post("/api/recognition/reregister", fullRecognitionPayload()),
    () => {
      state.recognition_workflow = {...(state.recognition_workflow || {}), busy: true, active: false, phase: "starting_yolo"};
    }
  );
  $("stopFullRecognition").onclick = () => runRecognitionControl(
    "stopFullRecognition",
    "停止完整识别",
    () => post("/api/recognition/stop", {}),
    () => {
      state.recognition_workflow = {...(state.recognition_workflow || {}), busy: false, active: false, phase: "stopped"};
    }
  );
  $("recognitionBasePoseTab").onclick = () => {
    recognitionPoseFrame = "base";
    renderRecognitionPose();
  };
  $("recognitionCameraPoseTab").onclick = () => {
    recognitionPoseFrame = "camera";
    renderRecognitionPose();
  };
  $("startArucoPose").onclick = () => runRecognitionControl(
    "startArucoPose",
    "启动 ArUco",
    () => post("/api/aruco/start_pose"),
    () => {
      state.aruco = state.aruco || {};
      state.aruco.box_pose_running = true;
    }
  );
  $("stopArucoPose").onclick = () => runRecognitionControl(
    "stopArucoPose",
    "停止 ArUco",
    () => post("/api/aruco/stop_pose"),
    () => {
      state.aruco = state.aruco || {};
      state.aruco.box_pose_running = false;
    }
  );
  $("startArucoFine").onclick = () => runRecognitionControl(
    "startArucoFine",
    "启动 ArUco 微调",
    () => post("/api/aruco/start_fine"),
    () => {
      state.aruco = state.aruco || {};
      state.aruco.fine_running = true;
    }
  );
  $("stopArucoFine").onclick = () => runRecognitionControl(
    "stopArucoFine",
    "停止 ArUco 微调",
    () => post("/api/aruco/stop_fine"),
    () => {
      state.aruco = state.aruco || {};
      state.aruco.fine_running = false;
      state.aruco.fine_enabled = false;
    }
  );
  $("arucoFineEnable").onchange = async (event) => {
    const checkbox = event.currentTarget;
    const backendEnabled = Boolean(state.aruco?.fine_enabled);
    if (!requireUnlocked("recognition")) {
      checkbox.checked = backendEnabled;
      return;
    }
    const problem = recognitionControlProblem("arucoFineEnable");
    if (problem) {
      checkbox.checked = backendEnabled;
      recognitionUiMessage = "";
      recognitionUiError = problem;
      renderRecognitionWorkflow();
      renderLocks();
      return;
    }
    const enabled = Boolean(checkbox.checked);
    const succeeded = await runRecognitionRequest(
      enabled ? "打开 ArUco 微调" : "关闭 ArUco 微调",
      () => post("/api/aruco/enable_fine", {enabled}),
      () => {
        state.aruco = state.aruco || {};
        state.aruco.fine_enabled = enabled;
        if (enabled) {
          state.aruco.box_pose_running = true;
          state.aruco.fine_running = true;
        }
      }
    );
    if (!succeeded) checkbox.checked = Boolean(state.aruco?.fine_enabled);
  };
  $("saveArucoPoseOpen").onclick = () => saveActionFromPage("recognition", "aruco_pose_open", "aruco_pose_open", "中文备注：启动头部左相机 ArUco id=1 位姿识别。", {}, "vision");
  $("saveArucoPoseClose").onclick = () => saveActionFromPage("recognition", "aruco_pose_close", "aruco_pose_close", "中文备注：关闭 ArUco 位姿识别。", {}, "vision");
  $("saveArucoFineEnable").onclick = () => saveActionFromPage("recognition", "aruco_fine_enable", "aruco_fine_enable", "中文备注：启动 ArUco 识别和底盘微调闭环。", {}, "vision");
  $("saveArucoFineDisable").onclick = () => saveActionFromPage("recognition", "aruco_fine_disable", "aruco_fine_disable", "中文备注：关闭 ArUco 底盘微调闭环。", {}, "vision");
}

function setupVlaEvents() {
  $("vlaRefreshDatasets").onclick = () => refreshVlaDatasets();
  $("vlaLoadDataset").onclick = loadVlaDataset;
  $("vlaConnectPolicy").onclick = connectVlaPolicy;
  $("vlaLoadTask").onclick = loadVlaRemoteTask;
  $("vlaInfer").onclick = runVlaInfer;
  $("vlaFullInfer").onclick = runVlaFullInfer;
  $("vlaStopFullInfer").onclick = stopVlaFullInfer;
  $("vlaStopFullInfer").disabled = true;
  $("vlaExecuteFullResult").onclick = executeVlaFullResult;
  $("vlaStopFullResult").onclick = stopVlaFullResult;
  $("vlaExecuteFullResult").disabled = true;
  $("vlaStopFullResult").disabled = true;
  $("vlaExecuteFirst").onclick = () => executeVla("first");
  $("vlaExecuteChunk").onclick = () => executeVla("chunk");
  $("vlaRunTask").onclick = runVlaTask;
  $("vlaStopTask").onclick = stopVlaTask;
  $("vlaStopTask").disabled = true;
  $("vlaExecutePlaybackFrame").onclick = () => executeVlaPlayback("first");
  $("vlaExecutePlaybackChunk").onclick = () => executeVlaPlayback("chunk");
  $("vlaExecuteRawLeftArm").onclick = executeVlaRawLeftArmPlayback;
  $("vlaMcapChannels").onclick = listVlaMcapChannels;
  $("vlaMcapDryRun").onclick = dryRunVlaMcap;
  $("vlaMcapEvalStart").onclick = () => startVlaMcapEval().catch((exc) => alert(exc.message || String(exc)));
  $("vlaMcapEvalStop").onclick = () => stopVlaMcapEval().catch((exc) => alert(exc.message || String(exc)));
  $("vlaDatasetSelect").onchange = () => {
    if ($("vlaDatasetPath")) $("vlaDatasetPath").value = "";
    clearVlaFrameCache();
    syncVlaDatasetSelection();
  };
  $("vlaDatasetPath").oninput = () => {
    clearVlaFrameCache();
    syncVlaDatasetSelection();
  };
  $("vlaRemoteTask").onchange = () => {
    vlaLastInfer = null;
    vlaFullInferComparison = null;
    renderVlaTaskSummary();
    renderVlaInferSummary();
    drawVlaCharts();
    updateVlaTaskControls();
  };
  $("vlaEpisodeSelect").onchange = () => {
    stopVlaPlaybackAnimation();
    clearVlaFrameCache();
    loadVlaEpisode().then(loadVlaFrame).catch(alert);
  };
  $("vlaFrameSlider").oninput = (event) => {
    stopVlaPlaybackAnimation();
    $("vlaFrameIndex").value = event.target.value;
    drawVlaCharts();
    scheduleLoadVlaFrame();
  };
  $("vlaFrameSlider").onchange = () => loadVlaFrame().catch(alert);
  $("vlaFrameIndex").onchange = (event) => {
    stopVlaPlaybackAnimation();
    $("vlaFrameSlider").value = event.target.value;
    loadVlaFrame().catch(alert);
  };
  ["vlaInputSource"].forEach((id) => {
    $(id).onchange = () => {
      syncVlaInputs();
      updateVlaStateSourceControl();
    };
  });
  $("vlaDatasetFormat").onchange = () => {
    stopVlaPlaybackAnimation();
    clearVlaFrameCache();
    vlaDataset = null;
    vlaFrame = null;
    vlaCurves = null;
    vlaLastInfer = null;
    fillVlaEpisodes();
    setVlaDatasetImages({});
    drawVlaCharts();
  };
  $("vlaActionSchema").onchange = () => {
    stopVlaPlaybackAnimation();
    clearVlaFrameCache();
    vlaFrame = null;
    vlaCurves = null;
    vlaLastInfer = null;
    loadVlaEpisode().then(loadVlaFrame).catch(() => drawVlaCharts());
  };
  $("vlaStateSchema").onchange = () => {
    stopVlaPlaybackAnimation();
    clearVlaFrameCache();
    vlaFrame = null;
    vlaCurves = null;
    vlaLastInfer = null;
    loadVlaEpisode().then(loadVlaFrame).catch(() => drawVlaCharts());
  };
  ["vlaCurveGroup", "vlaCurveUnit"].forEach((id) => {
    $(id).onchange = drawVlaCharts;
  });
  const switchProfile = (event) => {
    const profile = event.target.value;
    if (!profile || profile === (state.active_control_profile || "real")) return;
    post("/api/control_profile/select", {profile}).then((data) => {
      state.active_control_profile = data.active_control_profile || profile;
      state.control_profiles = data.control_profiles || state.control_profiles;
      lockAllPages();
      locks.vla = false;
      localStorage.setItem("r1pro.locks", JSON.stringify(locks));
      refreshState();
    }).catch(alert);
  };
  $("vlaPlaybackProfile").onchange = switchProfile;
  $("vlaInferProfile").onchange = switchProfile;
  refreshVlaDatasets().catch(() => {});
}

function setupWorkflowEvents() {
  ["actionName", "actionDescription", "actionType", "actionParams"].forEach((id) => {
    $(id).addEventListener("input", () => editingActionDirty = true);
  });
  $("actionParamForm").addEventListener("input", () => {
    editingActionDirty = true;
    $("actionParams").value = JSON.stringify(
      collectParamsFromForm($("actionParamForm"), "[data-action-param]", selectedAction?.params || {}),
      null,
      2
    );
  });
  $("codeWorkflowParamForm").addEventListener("input", () => {
    codeWorkflowParamsDirty = true;
  });
  $("codeWorkflowActionName").addEventListener("input", () => {
    codeWorkflowParamsDirty = true;
  });
  ["sequenceName", "sequenceDescription", "sequenceJson"].forEach((id) => {
    $(id).addEventListener("input", () => {
      editingSequenceDirty = true;
      if (id === "sequenceJson") syncSequenceFromJson();
    });
  });
  $("generateTemplates").onclick = () => requireUnlocked("workflow") && post("/api/actions/generate_templates").then(refreshState).catch(alert);
  $("reloadActions").onclick = refreshState;
  $("reloadSequences").onclick = refreshState;
  $("reloadCodeWorkflows").onclick = () => post("/api/code_workflows/reload").then((data) => {
    pushWorkflowMessage(data.message || "代码工作流已重载");
    refreshState();
  }).catch(alert);
  $("runCodeWorkflow").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    if (codeWorkflowRequestPending) return;
    const params = currentCodeWorkflowParams();
    if (params.dry_run === false) {
      const ok = confirm("dry_run 已关闭，将真实下发机器人动作。确认执行代码工作流？");
      if (!ok) {
        pushWorkflowMessage("已取消真实执行代码工作流");
        return;
      }
      params.motion_confirmed = true;
    }
    const requestGeneration = ++codeWorkflowRequestGeneration;
    setCodeWorkflowPending(true);
    pushWorkflowMessage(`正在发送代码工作流运行请求: ${selectedCodeWorkflowId || "-"}`);
    post("/api/code_workflows/run", {
      id: selectedCodeWorkflowId,
      params
    }, {timeoutMs: 30000}).then((data) => {
      if (requestGeneration !== codeWorkflowRequestGeneration) return;
      pushWorkflowMessage(data.message || "代码工作流已启动");
      refreshState();
    }).catch((exc) => {
      if (requestGeneration !== codeWorkflowRequestGeneration) return;
      codeWorkflowRequestFailed(exc, "运行代码工作流");
    }).finally(() => {
      if (requestGeneration === codeWorkflowRequestGeneration) {
        setCodeWorkflowPending(false);
      }
    });
  };
  $("stopCodeWorkflow").onclick = () => stopCodeWorkflowNow("代码工作流");
  $("saveCodeWorkflowAction").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    const selected = codeWorkflowById(selectedCodeWorkflowId);
    if (!selected) return;
    const name = ($("codeWorkflowActionName").value || selected.id).trim();
    if (!name) return;
    // Persist the exact parameters shown in the code-workflow form.  This is
    // important for calibrated workflows: a later sequence execution must
    // not silently fall back to class defaults after only ``workflow_id`` was
    // saved.
    const params = currentCodeWorkflowParams();
    const action = {
      name,
      description: `中文备注：${selected.description || "注册代码工作流默认参数动作。"}`,
      type: "code_workflow",
      source_page: "workflow",
      params: {
        workflow_id: selected.id,
        ...params,
      }
    };
    post("/api/actions/save", {action}).then((data) => {
      pushWorkflowMessage(`已保存代码工作流原子动作: ${data.action?.name || name}`);
      refreshState();
    }).catch(alert);
  };
  $("toggleJsonMode").onclick = () => {
    jsonMode = !jsonMode;
    localStorage.setItem("r1pro.jsonMode", jsonMode ? "1" : "0");
    render();
  };
  $("saveAction").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    post("/api/actions/save", {action: editorActionPayload()}).then((data) => {
      selectedAction = data.action;
      editingActionDirty = false;
      refreshState();
    }).catch(alert);
  };
  $("executeAction").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    executeActionRequest(selectedAction, $("actionName").value);
  };
  $("renameAction").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    const newName = prompt("新名称", $("actionName").value);
    if (newName) post("/api/actions/rename", {id: selectedAction?.id, name: $("actionName").value, new_name: newName}).then(refreshState).catch(alert);
  };
  $("deleteAction").onclick = () => requireUnlocked("workflow") && post("/api/actions/delete", {id: selectedAction?.id, name: $("actionName").value}).then(() => {
    selectedAction = null;
    refreshState();
  }).catch(alert);
  $("actionGroupList").onclick = (event) => {
    const selectButton = event.target.closest("[data-action-group-select]");
    const editButton = event.target.closest("[data-action-group-edit]");
    if (selectButton) {
      const ident = selectButton.dataset.actionGroupSelect;
      const group = actionGroupByIdent(ident);
      if (group) selectActionGroupForEdit(group);
      return;
    }
    if (editButton) {
      const ident = editButton.dataset.actionGroupEdit;
      const group = actionGroupByIdent(ident);
      if (group) selectActionGroupForEdit(group);
    }
  };
  $("newActionGroup").onclick = newActionGroup;
  $("actionGroupName").addEventListener("input", () => {
    ensureSelectedActionGroup();
    selectedActionGroup.name = $("actionGroupName").value;
  });
  $("actionGroupDescription").addEventListener("input", () => {
    ensureSelectedActionGroup();
    selectedActionGroup.description = $("actionGroupDescription").value;
  });
  $("addActionToGroup").onclick = addActionToGroup;
  $("clearActionGroup").onclick = clearActionGroup;
  $("saveActionGroup").onclick = saveActionGroup;
  $("renameActionGroup").onclick = renameActionGroup;
  $("deleteActionGroup").onclick = deleteActionGroup;
  $("actionGroupActions").onclick = (event) => {
    const actionName = event.target.closest("[data-action-group-execute-action]")?.dataset.actionGroupExecuteAction;
    if (actionName) {
      if (!requireUnlocked("workflow")) return;
      const action = actionByName(actionName);
      if (!action) {
        alert(`原子动作不存在: ${actionName}`);
        return;
      }
      executeActionRequest(action);
      return;
    }
    const index = event.target.dataset.removeActionGroupAction;
    if (index !== undefined) removeActionFromGroup(Number(index));
  };
  $("addStep").onclick = addSequenceStep;
  $("sequenceSteps").onclick = (event) => {
    const editIndex = event.target.dataset.editStepParams;
    if (editIndex !== undefined && selectedSequence) {
      const step = selectedSequence.steps?.[Number(editIndex)];
      editingStepParams.add(stepParamEditKey(step, Number(editIndex), event.target.dataset.actionName));
      renderSequenceEditor();
      return;
    }
    const saveIndex = event.target.dataset.saveStepParams;
    if (saveIndex !== undefined && selectedSequence) {
      const index = Number(saveIndex);
      const actionName = event.target.dataset.actionName;
      const step = selectedSequence.steps?.[index];
      const editor = event.target.closest(".step-action-editor");
      try {
        if (jsonMode) {
          const textarea = editor?.querySelector("[data-step-override]");
          applyStepOverride(index, actionName, textarea?.value || "{}");
        } else {
          const form = editor?.querySelector("[data-step-param-form]");
          const action = actionByName(actionName);
          const params = !form || form.dataset.clearOverride === "1"
            ? {}
            : collectParamsFromForm(form, "[data-step-param]", action?.params || {});
          applyStepOverrideObject(index, actionName, params);
        }
        editingStepParams.delete(stepParamEditKey(step, index, actionName));
        renderSequenceEditor();
      } catch (exc) {
        alert(exc);
      }
      return;
    }
    const moveIndex = event.target.dataset.moveStep;
    if (moveIndex !== undefined) {
      moveSequenceStep(Number(moveIndex), Number(event.target.dataset.moveDir));
      return;
    }
    const beforeIndex = event.target.dataset.insertBeforeStep;
    if (beforeIndex !== undefined) {
      insertSequenceStep(Number(beforeIndex));
      return;
    }
    const afterIndex = event.target.dataset.insertAfterStep;
    if (afterIndex !== undefined) {
      insertSequenceStep(Number(afterIndex) + 1);
      return;
    }
    const index = event.target.dataset.removeStep;
    if (index !== undefined && selectedSequence) {
      selectedSequence.steps.splice(Number(index), 1);
      editingStepParams = new Set();
      editingSequenceDirty = true;
      renderSequenceEditor();
      return;
    }
    const loadIndex = event.target.dataset.loadStepParams;
    if (loadIndex !== undefined && selectedSequence) {
      const actionName = event.target.dataset.actionName;
      const action = actionByName(actionName);
      const editor = event.target.closest(".step-action-editor");
      const form = editor?.querySelector("[data-step-param-form]");
      const textarea = editor?.querySelector("[data-step-override]");
      if (!action || !form) return;
      form.dataset.clearOverride = "0";
      editingSequenceDirty = true;
      form.querySelectorAll("[data-step-param]").forEach((input) => {
        const key = input.dataset.stepParam;
        const value = action.params?.[key];
        if (input.dataset.kind === "boolean") input.checked = Boolean(value);
        else input.value = formatParamValue(value);
      });
      if (textarea) textarea.value = JSON.stringify(action.params || {}, null, 2);
      return;
    }
    const clearIndex = event.target.dataset.clearStepParams;
    if (clearIndex !== undefined && selectedSequence) {
      const actionName = event.target.dataset.actionName;
      const action = actionByName(actionName);
      const editor = event.target.closest(".step-action-editor");
      const form = editor?.querySelector("[data-step-param-form]");
      const textarea = editor?.querySelector("[data-step-override]");
      if (!form) return;
      form.dataset.clearOverride = "1";
      editingSequenceDirty = true;
      form.querySelectorAll("[data-step-param]").forEach((input) => {
        const key = input.dataset.stepParam;
        const value = action?.params?.[key];
        if (input.dataset.kind === "boolean") input.checked = Boolean(value);
        else input.value = formatParamValue(value);
      });
      if (textarea) textarea.value = "{}";
    }
  };
  $("sequenceSteps").addEventListener("input", (event) => {
    const input = event.target.closest("[data-step-param]");
    if (input) {
      const form = input.closest("[data-step-param-form]");
      if (!form) return;
      try {
        form.dataset.clearOverride = "0";
        editingSequenceDirty = true;
        collectParamsFromForm(form, "[data-step-param]", actionByName(form.dataset.actionName)?.params || {});
        form.classList.remove("invalid");
      } catch {
        form.classList.add("invalid");
      }
      return;
    }
    const textarea = event.target.closest("[data-step-override]");
    if (!textarea) return;
    try {
      JSON.parse(textarea.value || "{}");
      editingSequenceDirty = true;
      textarea.classList.remove("invalid");
    } catch {
      textarea.classList.add("invalid");
    }
  });
  $("validateSequence").onclick = () => {
    try {
      post("/api/sequences/validate", {sequence: currentSequenceFromEditor()}).then((data) => alert(data.message || "workflow valid")).catch(alert);
    } catch (exc) { alert(exc); }
  };
  $("saveSequence").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    try {
      post("/api/sequences/save", {sequence: currentSequenceFromEditor()}).then((data) => {
        selectedSequence = data.sequence;
        editingSequenceDirty = false;
        refreshState();
      }).catch(alert);
    } catch (exc) { alert(exc); }
  };
  $("saveSequenceAs").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    try {
      const sequence = sequenceSaveAsPayload();
      if (!sequence) return;
      post("/api/sequences/save", {sequence}).then((data) => {
        selectedSequence = data.sequence;
        editingSequenceDirty = false;
        refreshState();
      }).catch(alert);
    } catch (exc) { alert(exc); }
  };
  $("runSequence").onclick = () => requireUnlocked("workflow") && post("/api/sequences/run", {id: selectedSequence?.id, name: $("sequenceName").value}).then((data) => {
    pushWorkflowMessage(data.message || "工作流已启动");
    refreshState();
  }).catch(alert);
  $("renameSequence").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    const newName = prompt("新名称", $("sequenceName").value);
    if (newName) post("/api/sequences/rename", {id: selectedSequence?.id, name: $("sequenceName").value, new_name: newName}).then(refreshState).catch(alert);
  };
  $("deleteSequence").onclick = () => requireUnlocked("workflow") && post("/api/sequences/delete", {id: selectedSequence?.id, name: $("sequenceName").value}).then(() => {
    selectedSequence = null;
    refreshState();
  }).catch(alert);
  $("continueSequence").onclick = () => {
    if (!requireUnlocked("workflow")) return;
    const runtime = state.sequence_runtime || {};
    if (!runtime.running || runtime.message !== "waiting_operator") return;
    if (!confirm("确认遥控器已回中、底盘已停稳，然后继续双臂动作？")) return;
    post("/api/sequences/continue", {step_index: runtime.step_index})
      .then(refreshState).catch(alert);
  };
  $("stopSequence").onclick = stopWorkflowStack;
}

function isTypingTarget(target) {
  if (!target) return false;
  if (target.isContentEditable) return true;
  const tagName = String(target.tagName || "").toLowerCase();
  return tagName === "input" || tagName === "textarea" || tagName === "select";
}

function onKeyDown(event) {
  if (event.repeat) return;
  if ($("chassisKeyboard")?.checked && locks.chassis && state.navigation?.manual_control_allowed && !isTypingTarget(event.target)) {
    const map = {w: [1, 0, 0], s: [-1, 0, 0], a: [0, 1, 0], d: [0, -1, 0], q: [0, 0, 1], e: [0, 0, -1]};
    const item = map[event.key.toLowerCase()];
    if (item) {
      event.preventDefault();
      sendChassisHold(item[0], item[1], item[2]);
      chassisKeyboardActive = true;
    }
  }
  if (activePage() === "ee" && $("eeKeyboard")?.checked && locks.ee) {
    const map = {
      ArrowUp: [[1, 0, 0], [0, 0, 0]], ArrowDown: [[-1, 0, 0], [0, 0, 0]],
      ArrowLeft: [[0, 1, 0], [0, 0, 0]], ArrowRight: [[0, -1, 0], [0, 0, 0]],
      PageUp: [[0, 0, 1], [0, 0, 0]], PageDown: [[0, 0, -1], [0, 0, 0]],
      j: [[0, 0, 0], [0, 0, 1]], l: [[0, 0, 0], [0, 0, -1]],
      i: [[0, 0, 0], [0, 1, 0]], k: [[0, 0, 0], [0, -1, 0]],
      u: [[0, 0, 0], [1, 0, 0]], o: [[0, 0, 0], [-1, 0, 0]]
    };
    const item = map[event.key];
    if (item) {
      event.preventDefault();
      jogEe(item[0], item[1]);
    }
  }
}

function onKeyUp(event) {
  if (!chassisKeyboardActive) return;
  const keys = new Set(["w", "s", "a", "d", "q", "e"]);
  if (keys.has(String(event.key || "").toLowerCase())) {
    event.preventDefault();
    stopChassisHold();
  }
}

async function refreshState() {
  try {
    state = await fetchJson("/api/state", {cache: "no-store"}, 5000);
    setConn(true, "connected");
    render();
  } catch (exc) {
    setConn(false, "offline");
  }
}

function connectEvents() {
  const events = new EventSource("/api/events");
  events.onopen = () => {
    document.querySelectorAll('img[src^="/stream/"]').forEach((img) => {
      const source = img.getAttribute("src") || "";
      const key = source.slice("/stream/".length).split("?")[0];
      if (key) connectCameraImageStream(img, key);
    });
  };
  events.onmessage = (event) => {
    try {
      state = JSON.parse(event.data);
      setConn(true, "live");
      render();
    } catch {
      setConn(false, "bad state");
    }
  };
  events.onerror = () => {
    setConn(false, "reconnecting");
    setTimeout(refreshState, 1500);
  };
}

setupEvents();
refreshState();
connectEvents();
