"use strict";

const els = Object.fromEntries([
  "map-summary", "map-wrap", "map-canvas", "preview-canvas", "crop-x", "crop-y",
  "crop-width", "crop-height", "crop-measurement", "select-all", "fit-view",
  "yaw-range", "yaw-deg", "auto-origin", "origin-x", "origin-y", "output-name",
  "overwrite", "export", "output-dir", "status", "preview-angle", "preview-origin",
  "preview-size",
].map((id) => [id, document.getElementById(id)]));

const state = {
  info: null,
  image: null,
  crop: { x: 0, y: 0, width: 1, height: 1 },
  dragStart: null,
  layout: null,
};

function clamp(value, min, max) { return Math.min(max, Math.max(min, value)); }
function num(value, fallback = 0) { const parsed = Number(value); return Number.isFinite(parsed) ? parsed : fallback; }
function tidy(value, digits = 4) { return Number(value.toFixed(digits)).toString(); }

function setupCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, width: rect.width, height: rect.height };
}

function normalizeCrop(raw) {
  const maxW = state.info.width;
  const maxH = state.info.height;
  const x = clamp(Math.round(num(raw.x)), 0, maxW - 1);
  const y = clamp(Math.round(num(raw.y)), 0, maxH - 1);
  return {
    x, y,
    width: clamp(Math.round(num(raw.width, 1)), 1, maxW - x),
    height: clamp(Math.round(num(raw.height, 1)), 1, maxH - y),
  };
}

function setCrop(crop, updateInputs = true) {
  state.crop = normalizeCrop(crop);
  if (updateInputs) {
    els["crop-x"].value = state.crop.x;
    els["crop-y"].value = state.crop.y;
    els["crop-width"].value = state.crop.width;
    els["crop-height"].value = state.crop.height;
  }
  updateOrigin();
  updateDetails();
  draw();
}

function automaticOrigin() {
  const { resolution, height, origin } = state.info;
  const localX = state.crop.x * resolution;
  const localY = (height - state.crop.y - state.crop.height) * resolution;
  const c = Math.cos(origin[2]);
  const s = Math.sin(origin[2]);
  return [origin[0] + c * localX - s * localY, origin[1] + s * localX + c * localY];
}

function updateOrigin() {
  const automatic = els["auto-origin"].checked;
  els["origin-x"].disabled = automatic;
  els["origin-y"].disabled = automatic;
  if (automatic) {
    const [x, y] = automaticOrigin();
    els["origin-x"].value = tidy(x, 6);
    els["origin-y"].value = tidy(y, 6);
  }
}

function updateDetails() {
  const metresW = state.crop.width * state.info.resolution;
  const metresH = state.crop.height * state.info.resolution;
  els["crop-measurement"].textContent = `${state.crop.width} × ${state.crop.height} px  ·  ${tidy(metresW, 3)} × ${tidy(metresH, 3)} m`;
  els["preview-size"].textContent = `${state.crop.width} × ${state.crop.height} px`;
  els["preview-origin"].textContent = `(${tidy(num(els["origin-x"].value), 4)}, ${tidy(num(els["origin-y"].value), 4)}) m`;
  const angle = num(els["yaw-deg"].value);
  els["preview-angle"].textContent = `yaw ${tidy(angle, 1)}°`;
}

function drawGrid(ctx, width, height, spacing = 28) {
  ctx.fillStyle = "#081419";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "rgba(66, 104, 115, .16)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = spacing + .5; x < width; x += spacing) { ctx.moveTo(x, 0); ctx.lineTo(x, height); }
  for (let y = spacing + .5; y < height; y += spacing) { ctx.moveTo(0, y); ctx.lineTo(width, y); }
  ctx.stroke();
}

function drawArrow(ctx, x1, y1, x2, y2, color, label) {
  const angle = Math.atan2(y2 - y1, x2 - x1);
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - 7 * Math.cos(angle - .45), y2 - 7 * Math.sin(angle - .45));
  ctx.lineTo(x2 - 7 * Math.cos(angle + .45), y2 - 7 * Math.sin(angle + .45));
  ctx.closePath(); ctx.fill();
  ctx.font = "600 10px ui-monospace, monospace"; ctx.fillText(label, x2 + 5, y2 + 3);
}

function drawMap() {
  if (!state.image) return;
  const { ctx, width, height } = setupCanvas(els["map-canvas"]);
  drawGrid(ctx, width, height);
  const pad = 26;
  const scale = Math.min((width - pad * 2) / state.info.width, (height - pad * 2) / state.info.height);
  const imageW = state.info.width * scale;
  const imageH = state.info.height * scale;
  const ox = (width - imageW) / 2;
  const oy = (height - imageH) / 2;
  state.layout = { scale, ox, oy };
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(state.image, ox, oy, imageW, imageH);
  ctx.strokeStyle = "#43606a"; ctx.lineWidth = 1; ctx.strokeRect(ox - .5, oy - .5, imageW + 1, imageH + 1);

  const r = {
    x: ox + state.crop.x * scale,
    y: oy + state.crop.y * scale,
    width: state.crop.width * scale,
    height: state.crop.height * scale,
  };
  ctx.fillStyle = "rgba(2, 9, 12, .62)";
  ctx.beginPath(); ctx.rect(ox, oy, imageW, imageH); ctx.rect(r.x, r.y, r.width, r.height);
  ctx.fill("evenodd");
  ctx.fillStyle = "rgba(73, 214, 218, .10)"; ctx.fillRect(r.x, r.y, r.width, r.height);
  ctx.strokeStyle = "#49d6da"; ctx.lineWidth = 2; ctx.setLineDash([7, 4]); ctx.strokeRect(r.x, r.y, r.width, r.height); ctx.setLineDash([]);

  const axisLength = clamp(Math.min(r.width, r.height) * .28, 22, 52);
  const ax = r.x + 3; const ay = r.y + r.height - 3;
  drawArrow(ctx, ax, ay, ax + axisLength, ay, "#ff6f61", "Xmap");
  drawArrow(ctx, ax, ay, ax, ay - axisLength, "#77db8a", "Ymap");
  ctx.fillStyle = "#edf7f8"; ctx.beginPath(); ctx.arc(ax, ay, 3, 0, Math.PI * 2); ctx.fill();
}

function drawPreview() {
  if (!state.image) return;
  const { ctx, width, height } = setupCanvas(els["preview-canvas"]);
  drawGrid(ctx, width, height, 24);
  const originX = width * .43; const originY = height * .56;
  const maxW = width * .62; const maxH = height * .68;
  const scale = Math.min(maxW / state.crop.width, maxH / state.crop.height);
  const angle = num(els["yaw-deg"].value) * Math.PI / 180;

  ctx.save();
  ctx.translate(originX, originY);
  ctx.rotate(-angle);
  ctx.globalAlpha = .82; ctx.imageSmoothingEnabled = false;
  ctx.drawImage(
    state.image, state.crop.x, state.crop.y, state.crop.width, state.crop.height,
    0, -state.crop.height * scale, state.crop.width * scale, state.crop.height * scale
  );
  ctx.globalAlpha = 1; ctx.strokeStyle = "#49d6da"; ctx.lineWidth = 1.5;
  ctx.strokeRect(0, -state.crop.height * scale, state.crop.width * scale, state.crop.height * scale);
  ctx.restore();

  ctx.setLineDash([4, 4]);
  drawArrow(ctx, originX, originY, originX + Math.min(84, width * .22), originY, "#ff6f61", "Xworld");
  drawArrow(ctx, originX, originY, originX, originY - Math.min(68, height * .32), "#77db8a", "Yworld");
  ctx.setLineDash([]);
  ctx.fillStyle = "#edf7f8"; ctx.beginPath(); ctx.arc(originX, originY, 3, 0, Math.PI * 2); ctx.fill();
}

function draw() { drawMap(); drawPreview(); }

function canvasPoint(event) {
  const rect = els["map-canvas"].getBoundingClientRect();
  const { scale, ox, oy } = state.layout;
  return {
    x: clamp((event.clientX - rect.left - ox) / scale, 0, state.info.width),
    y: clamp((event.clientY - rect.top - oy) / scale, 0, state.info.height),
  };
}

function syncCropInputs() {
  setCrop({
    x: els["crop-x"].value, y: els["crop-y"].value,
    width: els["crop-width"].value, height: els["crop-height"].value,
  });
}

async function exportMap() {
  const button = els.export;
  button.disabled = true;
  showStatus("正在导出…", "success");
  try {
    const response = await fetch("/api/export", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        crop: state.crop,
        yaw_deg: num(els["yaw-deg"].value),
        origin: [num(els["origin-x"].value), num(els["origin-y"].value)],
        name: els["output-name"].value.trim(),
        overwrite: els.overwrite.checked,
      }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `导出失败 (${response.status})`);
    showStatus(`${result.message}<br>${result.files.map((path) => escapeHtml(path)).join("<br>")}`, "success");
  } catch (error) {
    showStatus(error.message, "error");
  } finally { button.disabled = false; }
}

function escapeHtml(value) {
  const div = document.createElement("div"); div.textContent = value; return div.innerHTML;
}

function showStatus(message, kind) {
  els.status.innerHTML = message;
  els.status.className = `status show ${kind}`;
}

function bindEvents() {
  ["crop-x", "crop-y", "crop-width", "crop-height"].forEach((id) => els[id].addEventListener("change", syncCropInputs));
  els["select-all"].addEventListener("click", () => setCrop({ x: 0, y: 0, width: state.info.width, height: state.info.height }));
  els["fit-view"].addEventListener("click", draw);
  els["map-canvas"].addEventListener("dblclick", () => els["select-all"].click());
  els["map-canvas"].addEventListener("pointerdown", (event) => {
    state.dragStart = canvasPoint(event); els["map-canvas"].setPointerCapture(event.pointerId);
  });
  els["map-canvas"].addEventListener("pointermove", (event) => {
    if (!state.dragStart) return;
    const point = canvasPoint(event);
    const x1 = Math.floor(Math.min(state.dragStart.x, point.x));
    const y1 = Math.floor(Math.min(state.dragStart.y, point.y));
    const x2 = Math.ceil(Math.max(state.dragStart.x, point.x));
    const y2 = Math.ceil(Math.max(state.dragStart.y, point.y));
    if (x2 > x1 && y2 > y1) setCrop({ x: x1, y: y1, width: x2 - x1, height: y2 - y1 });
  });
  const endDrag = () => { state.dragStart = null; };
  els["map-canvas"].addEventListener("pointerup", endDrag);
  els["map-canvas"].addEventListener("pointercancel", endDrag);

  const setAngle = (value) => {
    const angle = num(value);
    els["yaw-deg"].value = tidy(angle, 3);
    els["yaw-range"].value = clamp(angle, -180, 180);
    updateDetails(); drawPreview();
  };
  els["yaw-range"].addEventListener("input", (event) => setAngle(event.target.value));
  els["yaw-deg"].addEventListener("input", (event) => {
    if (event.target.value === "") return;
    const angle = Number(event.target.value);
    if (!Number.isFinite(angle)) return;
    els["yaw-range"].value = clamp(angle, -180, 180);
    updateDetails(); drawPreview();
  });
  els["yaw-deg"].addEventListener("change", (event) => setAngle(event.target.value));
  document.querySelectorAll("[data-angle]").forEach((button) => button.addEventListener("click", () => setAngle(button.dataset.angle)));
  els["auto-origin"].addEventListener("change", () => { updateOrigin(); updateDetails(); drawPreview(); });
  ["origin-x", "origin-y"].forEach((id) => els[id].addEventListener("input", updateDetails));
  els.export.addEventListener("click", exportMap);
  window.addEventListener("resize", draw);
}

async function init() {
  try {
    const response = await fetch("/api/map", { cache: "no-store" });
    if (!response.ok) throw new Error("地图信息读取失败");
    state.info = await response.json();
    els["map-summary"].textContent = `${state.info.yaml_name}  ·  ${state.info.width} × ${state.info.height} px  ·  ${state.info.resolution} m/px`;
    els["output-dir"].textContent = state.info.output_dir;
    els["yaw-deg"].value = tidy(state.info.origin_yaw_deg, 3);
    els["yaw-range"].value = clamp(state.info.origin_yaw_deg, -180, 180);
    state.image = new Image();
    state.image.src = `/api/image?t=${Date.now()}`;
    await state.image.decode();
    bindEvents();
    setCrop({ x: 0, y: 0, width: state.info.width, height: state.info.height });
  } catch (error) {
    els["map-summary"].textContent = error.message;
    showStatus(error.message, "error");
  }
}

init();
