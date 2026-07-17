"use strict";

const state = {
  data: null,
  activeSession: null,
  selectedStage: null,
  formSession: null,
  jobCursor: 0,
  jobText: "",
  lastJobStatus: "idle",
  view: "prepare",
  pendingStage: null,
  textAction: null,
};

const $ = (selector, root = document) => root.querySelector(selector);
const $$ = (selector, root = document) => Array.from(root.querySelectorAll(selector));

const labels = {
  mass_kg: "Race-ready mass",
  wheelbase_m: "Wheelbase",
  front_axle_load_N: "Front axle load",
  rear_axle_load_N: "Rear axle load",
  vehicle_length_m: "Vehicle length",
  vehicle_width_m: "Vehicle width",
  vehicle_height_m: "Vehicle height",
  track_front_m: "Front track width",
  track_rear_m: "Rear track width",
  loaded_wheel_radius_m: "Loaded wheel radius",
  wheel_width_m: "Wheel / tread width",
  cg_height_m: "CoG height",
  yaw_inertia_kg_m2: "Yaw inertia",
  rope_spacing_m: "Rope spacing D",
  rope_length_m: "Rope length L",
  period_s: "Mean oscillation period T",
  operator: "Operator",
  measured_utc: "Measurement time (UTC)",
  surface: "Surface and tyre condition",
  battery_state: "Battery configuration / SOC",
  notes: "Campaign notes",
};

const units = {
  mass_kg: "kg", wheelbase_m: "m", front_axle_load_N: "N", rear_axle_load_N: "N",
  vehicle_length_m: "m", vehicle_width_m: "m", vehicle_height_m: "m", track_front_m: "m",
  track_rear_m: "m", loaded_wheel_radius_m: "m", wheel_width_m: "m", cg_height_m: "m",
  yaw_inertia_kg_m2: "kg·m²", rope_spacing_m: "m", rope_length_m: "m", period_s: "s",
};

const confirmationLabels = {
  instructions_read: "I have read the setup, during-test behavior and expected result above.",
  measurements_confirmed: "The race-ready configuration and entered measurements are current and correct.",
  car_secured: "The car is securely supported and all moving parts are clear.",
  car_positioned: "The car is positioned at the requested start with the full manoeuvre envelope available.",
  area_clear: "The test area is clear of people, obstacles and loose objects.",
  estop_ready: "I can immediately interrupt or physically stop the vehicle if behavior is unexpected.",
};

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>'"]/g, character => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;",
  })[character]);
}

function prettyKey(value) {
  return String(value || "").replaceAll("_", " ").replace(/\b\w/g, letter => letter.toUpperCase());
}

function formatNumber(value, digits = 3) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits).replace(/\.?0+$/, "") : "—";
}

async function api(path, options = {}) {
  const request = {method: options.method || "GET", headers: {"Content-Type": "application/json"}};
  if (options.body !== undefined) request.body = JSON.stringify(options.body);
  const response = await fetch(path, request);
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(payload.error || `${response.status} ${response.statusText}`);
  return payload;
}

let noticeTimer = null;
function notice(message, error = false) {
  const element = $("#notice");
  element.textContent = message;
  element.classList.remove("hidden", "error");
  if (error) element.classList.add("error");
  clearTimeout(noticeTimer);
  noticeTimer = setTimeout(() => element.classList.add("hidden"), error ? 8000 : 4000);
}

function switchView(view) {
  state.view = view;
  $$(".nav-item").forEach(button => button.classList.toggle("active", button.dataset.view === view));
  $$(".view").forEach(element => element.classList.toggle("active", element.id === `view-${view}`));
  const headings = {
    prepare: ["Before the first movement", "Prepare the car and workspace"],
    journey: ["One gated stage at a time", "Follow the calibration journey"],
    results: ["Plots, values and provenance", "Understand what the data says"],
  };
  $("#view-eyebrow").textContent = headings[view][0];
  $("#view-title").textContent = headings[view][1];
  if (view === "journey" && state.data) renderJourney(state.data);
  if (view === "results" && state.data) renderResults(state.data);
}

function renderSidebar(data) {
  const manifest = data.manifest;
  $("#session-name").textContent = manifest?.session_id || "No session";
  $("#session-progress-label").textContent = manifest ? `${manifest.completed} of ${manifest.total} stages complete` : "Create or select one to begin";
  $("#session-progress-bar").style.width = `${manifest?.progress_percent || 0}%`;
  const pill = $("#job-pill");
  const job = data.job || {};
  pill.className = `status-pill ${job.running ? "running" : job.status === "failed" ? "failed" : job.status === "succeeded" ? "good" : "idle"}`;
  $("span", pill).textContent = job.running ? job.label : job.status === "failed" ? "Last job failed" : "No job running";
}

function renderSessionPicker(data) {
  for (const [selector, prompt] of [["#session-select", "Select a session…"], ["#session-select-active", "Switch session…"]]) {
    const select = $(selector);
    const current = select.value;
    select.innerHTML = `<option value="">${prompt}</option>` + data.sessions.map(session =>
      `<option value="${escapeHtml(session.id)}">${escapeHtml(session.id)} · ${session.completed}/${session.total}${session.compatible ? "" : " · inspect only"}</option>`
    ).join("");
    const active = data.sessions.find(session => session.active)?.id;
    if (selector === "#session-select-active" && active) select.value = active;
    else if (current && data.sessions.some(session => session.id === current)) select.value = current;
  }
}

function readinessRows(readiness) {
  const rows = [
    ["ROS 2 command available", readiness.ros2_cli, "Source install/setup.bash before starting the GUI."],
    ["colcon build available", readiness.colcon, "Required for reversible candidate builds."],
    ["Workspace has install/setup.bash", readiness.workspace_install, "Build the workspace once if absent."],
    ["ROS environment sourced", readiness.ros_environment_sourced, "The GUI inherits the terminal environment."],
    ["At least 4 GiB free", readiness.free_disk_gb >= 4, `${formatNumber(readiness.free_disk_gb, 1)} GiB currently free.`],
    ["LaTeX PDF compiler", readiness.pdflatex, readiness.pdflatex ? "PDF refreshes after every stage." : "The .tex report works, but install pdflatex for PDF output."],
    ["No interrupted config transaction", !readiness.recovery_lock, readiness.recovery_lock ? "Recover before moving the car." : "VESC source configuration is clean."],
  ];
  return rows;
}

function renderPrepare(data, forceForm = false) {
  renderSessionPicker(data);
  const hasSession = Boolean(data.active_session);
  $("#no-session-card").classList.toggle("hidden", hasSession);
  $("#prepare-content").classList.toggle("hidden", !hasSession);
  if (!hasSession) return;
  $("#active-session-toolbar").textContent = data.manifest?.session_id || data.active_session;
  const profileWarning = $("#profile-warning");
  profileWarning.classList.toggle("hidden", data.manifest?.compatible !== false);
  profileWarning.textContent = data.manifest?.incompatibility_reason || "";

  const rows = readinessRows(data.readiness);
  const good = rows.filter(row => row[1]).length;
  $("#readiness-score").textContent = `${good}/${rows.length}`;
  $("#readiness-list").innerHTML = rows.map(([name, ok, detail]) =>
    `<div class="check-row ${ok ? "ok" : "warn"}"><i>${ok ? "✓" : "!"}</i><div><strong>${escapeHtml(name)}</strong><br><small>${escapeHtml(detail)}</small></div></div>`
  ).join("");
  $("#recover-button").classList.toggle("hidden", !data.readiness.recovery_lock);

  const budget = data.manifest?.campaign_budget || {};
  const minPasses = budget.nominal_driving_trials_min;
  const maxPasses = budget.nominal_driving_trials_max;
  $("#metric-stages").textContent = data.manifest?.total || 28;
  $("#metric-passes").textContent = minPasses == null ? "—" : minPasses === maxPasses ? String(minPasses) : `${minPasses}–${maxPasses}`;
  const hours = budget.operator_time_estimate_hours_without_rework;
  $("#metric-time").textContent = Array.isArray(hours) ? `${hours[0]}–${hours[1]} h` : "10–14 h";
  const room = data.manifest?.room_preflight || {};
  const roomLength = formatNumber(room.physical_room_length_m, 1);
  const roomWidth = formatNumber(room.physical_room_width_m, 1);
  const clearance = formatNumber(room.wall_clearance_m, 1);
  $("#room-badge").innerHTML = data.manifest?.compatible === false
    ? "Older frozen room profile · inspect only"
    : `<span>${roomLength} × ${roomWidth} m</span> ${clearance} m wall clearance · ${formatNumber(room.planned_straight_lane_heading_deg, 0)}° lane · ${formatNumber(100 * room.capture_room_utilization_target, 0)}% capture target · full circles where fitting`;

  const validation = data.metrology.validation;
  const badge = $("#metrology-status");
  badge.className = `status-pill ${validation.ready ? "good" : "warning"}`;
  $("span", badge).textContent = validation.ready ? "Ready" : `${Object.keys(validation.errors).length} items need attention`;
  if (forceForm || state.formSession !== data.active_session) {
    renderMetrologyForm(data.metrology);
    state.formSession = data.active_session;
  } else {
    renderMetrologyValidation(validation);
  }
}

function methodText(item) {
  const method = item?.method || "";
  return method.length > 135 ? `${method.slice(0, 132)}…` : method;
}

function numericField(section, name, item) {
  const unit = units[name] || "SI";
  const source = item?.source_kind ? ` · ${String(item.source_kind).replaceAll("_", " ")}` : "";
  return `<div class="field" data-error-prefix="${section}.${name}">
    <span>${escapeHtml(labels[name] || prettyKey(name))} <em class="unit-label">${escapeHtml(unit)}</em></span>
    <input id="field-${section}-${name}-value" type="number" step="any" inputmode="decimal" value="${item?.value ?? ""}">
    <small>${escapeHtml(methodText(item))}${escapeHtml(source)}</small>
  </div>`;
}

function contextField(name, value) {
  const isLong = ["surface", "battery_state", "notes"].includes(name);
  const control = isLong
    ? `<textarea id="context-${name}">${escapeHtml(value || "")}</textarea>`
    : `<input id="context-${name}" value="${escapeHtml(value || "")}">`;
  return `<label class="field"><span>${escapeHtml(labels[name])}</span>${control}</label>`;
}

function renderMetrologyForm(metrology) {
  const document = metrology.document;
  $("#context-fields").innerHTML = metrology.editable.context.map(name => contextField(name, document.measurement_context?.[name])).join("");
  $("#required-fields").innerHTML = metrology.editable.required.map(name => numericField("required", name, document.required?.[name] || {})).join("");
  $("#recommended-fields").innerHTML = metrology.editable.recommended.map(name => numericField("recommended", name, document.recommended?.[name] || {})).join("");
  $("#bifilar-fields").innerHTML = metrology.editable.bifilar.map(name => numericField("bifilar_yaw_inertia", name, document.bifilar_yaw_inertia?.[name] || {})).join("") +
    `<label class="field"><span>Repeated oscillations</span><input id="bifilar-repetitions" type="number" min="1" step="1" value="${document.bifilar_yaw_inertia?.repetitions ?? ""}"><small>Number used to estimate the mean period.</small></label>`;
  const editableRequired = new Set(metrology.editable.required);
  $("#geometry-fields").innerHTML = Object.entries(document.required || {}).filter(([name]) => !editableRequired.has(name)).map(([name, item]) =>
    `<div class="geometry-chip"><strong>${escapeHtml(prettyKey(name))}</strong>${escapeHtml(String(item.value))}</div>`
  ).join("");
  renderMetrologyValidation(metrology.validation);
}

function renderMetrologyValidation(validation) {
  $$(".field.invalid").forEach(element => element.classList.remove("invalid"));
  Object.keys(validation.errors || {}).forEach(path => {
    const parts = path.split(".");
    if (parts.length >= 2) $(`[data-error-prefix="${parts[0]}.${parts[1]}"]`)?.classList.add("invalid");
  });
  const errorValues = Object.values(validation.errors || {});
  $("#metrology-errors").innerHTML = errorValues.slice(0, 6).map(value => `• ${escapeHtml(value)}`).join("<br>") + (errorValues.length > 6 ? `<br>• and ${errorValues.length - 6} more` : "");
  const closure = validation.axle_load_closure || {};
  const card = $("#closure-card");
  if (closure.expected_weight_N == null) {
    card.className = "closure-card";
    card.textContent = "Axle-load closure appears after mass and both axle loads are entered.";
  } else {
    card.className = `closure-card ${closure.accepted ? "good" : "bad"}`;
    card.textContent = `Expected total ${formatNumber(closure.expected_weight_N)} N · entered ${formatNumber(closure.entered_axle_sum_N)} N · difference ${closure.error_N >= 0 ? "+" : ""}${formatNumber(closure.error_N)} N · allowed ±${formatNumber(closure.tolerance_N)} N`;
  }
}

function collectPair(section, name) {
  return {value: $(`#field-${section}-${name}-value`)?.value ?? ""};
}

function collectMetrology() {
  const editable = state.data.metrology.editable;
  return {
    measurement_context: Object.fromEntries(editable.context.map(name => [name, $(`#context-${name}`)?.value || ""])),
    required: Object.fromEntries(editable.required.map(name => [name, collectPair("required", name)])),
    recommended: Object.fromEntries(editable.recommended.map(name => [name, collectPair("recommended", name)])),
    bifilar_yaw_inertia: {
      ...Object.fromEntries(editable.bifilar.map(name => [name, collectPair("bifilar_yaw_inertia", name)])),
      repetitions: $("#bifilar-repetitions")?.value || "",
    },
  };
}

function stageStatusLabel(status) {
  return ({pending: "Waiting", running: "Running", completed: "Passed", failed: "Failed"})[status] || prettyKey(status);
}

function renderJourney(data) {
  const hasSession = Boolean(data.active_session);
  $("#journey-empty").classList.toggle("hidden", hasSession);
  $("#journey-content").classList.toggle("hidden", !hasSession);
  if (!hasSession) return;
  const stages = data.stages;
  const completed = stages.filter(stage => stage.status === "completed").length;
  $("#rail-progress").textContent = `${completed} / ${stages.length}`;
  if (!state.selectedStage || !stages.some(stage => stage.key === state.selectedStage)) {
    state.selectedStage = data.next_stage || stages[stages.length - 1]?.key;
  }
  $("#stage-list").innerHTML = stages.map(stage => `<button class="stage-item ${escapeHtml(stage.status)} ${stage.key === state.selectedStage ? "selected" : ""}" data-stage="${escapeHtml(stage.key)}">
    <span class="stage-number">${stage.status === "completed" ? "✓" : stage.status === "failed" ? "!" : String(stage.index).padStart(2, "0")}</span>
    <span class="stage-copy"><strong>${escapeHtml(stage.guide.title)}</strong><small>${escapeHtml(stageStatusLabel(stage.status))}</small></span><i class="stage-dot"></i>
  </button>`).join("");
  $$(".stage-item").forEach(button => button.addEventListener("click", () => {
    state.selectedStage = button.dataset.stage;
    renderJourney(state.data);
  }));
  renderStageDetail(stages.find(stage => stage.key === state.selectedStage));
  renderTerminalVisibility(data.job);
}

function list(items) {
  return `<ul>${(items || []).map(item => `<li>${escapeHtml(item)}</li>`).join("")}</ul>`;
}

function setupFigure(kind) {
  const room = `<rect x="18" y="18" width="324" height="224" rx="8" class="room-wall"/>
    <path d="M22 62h22m-22 44h22m-22 44h22m-22 44h22M316 62h22m-22 44h22m-22 44h22m-22 44h22" class="room-feature"/>
    <rect x="40" y="40" width="280" height="180" rx="5" class="clear-room"/>
    <text x="180" y="255" class="figure-dimension">14 × 14 m walls · keep 1 m wall band clear</text>`;
  const car = (x, y, rotate = 0) => `<g transform="translate(${x} ${y}) rotate(${rotate})"><rect x="-18" y="-10" width="36" height="20" rx="5" class="figure-car"/><path d="M7-5l9 5-9 5z" class="figure-arrow"/></g>`;
  let drawing = "";
  let caption = "";
  if (kind === "stand") {
    drawing = `<path d="M70 193h220" class="floor-line"/><rect x="122" y="127" width="116" height="45" rx="10" class="figure-car"/><circle cx="145" cy="174" r="17" class="wheel"/><circle cx="215" cy="174" r="17" class="wheel"/><rect x="128" y="190" width="34" height="12" class="stand-block"/><rect x="198" y="190" width="34" height="12" class="stand-block"/><path d="M145 207v22m70-22v22" class="measure-line"/><text x="180" y="246" class="figure-dimension">Wheels and steering clear</text>`;
    caption = "Secure the car on stable supports before stationary actuator tests.";
  } else if (kind === "metrology") {
    drawing = `${room}<line x1="105" y1="54" x2="105" y2="210" class="axle-line"/><line x1="255" y1="54" x2="255" y2="210" class="axle-line"/><rect x="82" y="78" width="196" height="106" rx="28" class="car-outline"/>${car(180, 131)}<rect x="74" y="51" width="62" height="26" rx="5" class="scale-pad"/><rect x="224" y="51" width="62" height="26" rx="5" class="scale-pad"/><rect x="74" y="186" width="62" height="26" rx="5" class="scale-pad"/><rect x="224" y="186" width="62" height="26" rx="5" class="scale-pad"/><text x="105" y="46" class="figure-label">REAR AXLE</text><text x="255" y="46" class="figure-label">FRONT AXLE</text>`;
    caption = "Keep all four contact patches at equal height and sum each axle’s two scales.";
  } else if (kind === "turning") {
    drawing = `${room}<circle cx="180" cy="131" r="72" class="turn-envelope"/><circle cx="180" cy="131" r="72" class="motion-path"/>${car(108, 131, -90)}<circle cx="180" cy="131" r="3" class="fixed-feature"/><text x="180" y="132" class="figure-label">MARKED CENTRE</text><text x="180" y="32" class="figure-dimension">Clear the complete circle and stopping envelope</text>`;
    caption = "Mark the circle centre and tangent start. Fitting steady-turn conditions record one complete revolution; follow the displayed radius for each trial.";
  } else if (kind === "observability") {
    drawing = `${room}<rect x="33" y="36" width="40" height="24" class="fixed-feature"/><circle cx="311" cy="62" r="18" class="fixed-feature"/><path d="M304 193l26-20 8 35z" class="fixed-feature"/><rect x="42" y="184" width="25" height="36" class="fixed-feature"/><path d="M70 205L288 55" class="motion-path"/>${car(103, 182, -35)}<path d="M103 182L48 50M103 182l208-120M103 182l203 10" class="lidar-ray"/><text x="180" y="32" class="figure-dimension">Asymmetric fixed features at several ranges</text>`;
    caption = "Mark the 45° lane inside the clear central area; keep fixed, asymmetric walls or objects visible on both sides.";
  } else {
    drawing = `${room}<path d="M63 211L299 49" class="clear-lane diagonal-lane"/><path d="M72 204L289 55" class="motion-path"/>${car(101, 184, -35)}<line x1="61" y1="184" x2="91" y2="225" class="start-line"/><line x1="274" y1="34" x2="304" y2="75" class="stop-line"/><text x="68" y="231" class="figure-label">START</text><text x="301" y="37" class="figure-label">STOP</text><text x="180" y="228" class="figure-dimension">45° straight lane + braking envelope</text>`;
    caption = "Use the room diagonal. The preflight includes travel, emergency braking and the full vehicle footprint.";
  }
  return `<figure class="setup-figure"><svg viewBox="0 0 360 270" role="img" aria-label="Stage room setup">${drawing}</svg><figcaption>${caption}</figcaption></figure>`;
}

function renderStageDetail(stage) {
  if (!stage) return;
  const trials = stage.nominal_trials_max === 0 ? "No driving passes" : stage.nominal_trials_min === stage.nominal_trials_max
    ? `${stage.nominal_trials_min} nominal driving passes`
    : `${stage.nominal_trials_min}–${stage.nominal_trials_max} nominal driving passes`;
  const values = (stage.headline_values || []).length
    ? `<div class="stage-values"><h3>Collected / fitted values</h3>${stage.headline_values.map(value => `<code>${escapeHtml(value)}</code>`).join("")}</div>` : "";
  const stats = stage.statistical_summary || {};
  const statistics = stats.artifact ? (stats.sampling === "direct"
    ? `<div class="stage-meta"><span>Direct measurement</span><span>No fitted confidence interval claimed</span></div>`
    : `<div class="stage-meta"><span>Statistical support</span><span>${stats.accepted_rows}/${stats.total_rows} accepted rows</span><span>${stats.independent_units} independent trials/units</span>${stats.condition_count == null ? "" : `<span>${stats.condition_count} conditions · min ${stats.minimum_per_condition ?? "—"} repeats</span>`}</div>`) : "";
  const error = stage.error ? `<div class="stage-error"><strong>Why this stage stopped</strong><br>${escapeHtml(stage.error)}</div>` : "";
  const plot = stage.plot_url ? `<div class="stage-plot"><img src="${stage.plot_url}?t=${Date.now()}" alt="${escapeHtml(stage.guide.title)} plot"></div>` : "";
  let action = "";
  if (stage.can_run) action = `<button class="button primary" id="run-stage">Prepare and run this stage</button>`;
  else if (stage.status === "completed") action = `<span class="status-pill good"><i></i><span>Passed and recorded</span></span>`;
  else if (stage.status === "failed") action = `<button class="button primary" id="retry-stage">Repeat this stage</button>`;
  else action = `<span class="hint">Complete the preceding gated stage first.</span>`;
  if (stage.redo_target) action += `<button class="button danger" id="redo-stage">Discard candidate and restart fresh A</button>`;
  if (stage.report_url) action += `<a class="button secondary" href="${stage.report_url}" target="_blank" rel="noopener">Open stage notes</a>`;
  if (stage.statistics_url) action += `<a class="button secondary" href="${stage.statistics_url}" target="_blank" rel="noopener">Open statistics</a>`;

  $("#stage-detail").innerHTML = `
    <div class="stage-kicker"><span>Stage ${String(stage.index).padStart(2, "0")}</span>${escapeHtml(stage.guide.abc_role)}</div>
    <h2>${escapeHtml(stage.guide.title)}</h2>
    <p class="stage-summary">${escapeHtml(stage.guide.summary)}</p>
    ${setupFigure(stage.guide.setup_figure)}
    <div class="stage-meta"><span>${escapeHtml(prettyKey(stage.guide.category))}</span><span>${escapeHtml(trials)}</span>${stage.trial_note ? `<span>${escapeHtml(stage.trial_note)}</span>` : ""}</div>
    <div class="guide-grid">
      <div class="guide-card"><h3>1 · Set up</h3>${list(stage.guide.setup)}</div>
      <div class="guide-card"><h3>2 · Test</h3>${list(stage.guide.during)}</div>
      <div class="guide-card"><h3>3 · Pass when</h3>${list(stage.guide.expect)}</div>
    </div>
    <details class="stage-evidence"><summary>Recorded data and outputs</summary><div><h3>Bagged evidence</h3>${list(stage.guide.data)}<h3>Generated output</h3>${list(stage.guide.outputs)}</div></details>
    ${statistics}${values}${error}${plot}<div class="stage-actions">${action}</div>`;
  $("#run-stage")?.addEventListener("click", () => openStageConfirmation(stage));
  $("#retry-stage")?.addEventListener("click", () => openStageConfirmation(stage));
  $("#redo-stage")?.addEventListener("click", () => openTextConfirmation({
    title: "Start a genuinely fresh calibration fit?", word: "REDO",
    summary: `This archives the failed ${stage.key} candidate and returns to ${stage.redo_target}. It is the correct choice when validation shows the fitted value itself is wrong.`,
    endpoint: "/api/redo", body: {stage: stage.key},
  }));
}

function renderTerminalVisibility(job) {
  const visible = Boolean(job?.label) && ["running", "stopping", "failed", "succeeded", "stopped"].includes(job.status);
  $("#terminal-panel").classList.toggle("hidden", !visible);
  if (visible) $("#terminal-title").textContent = job.label || "Runner";
  $("#stop-job").classList.toggle("hidden", !job?.running);
  updatePromptControls(job);
}

function updatePromptControls(job) {
  const tail = state.jobText.slice(-1200).toUpperCase();
  const allowed = new Set();
  if (job?.running) {
    if (tail.includes("DECISION [ACCEPT/REDO/SKIP/ABORT]")) ["ACCEPT", "REDO", "SKIP", "ABORT"].forEach(v => allowed.add(v));
    else if (tail.includes("DECISION [REDO/SKIP/ABORT]")) ["REDO", "SKIP", "ABORT"].forEach(v => allowed.add(v));
    else if (tail.includes("DECISION [REDO/ABORT]")) ["REDO", "ABORT"].forEach(v => allowed.add(v));
    else if (tail.includes("TYPE CONFIRM") && tail.includes("REDO")) ["CONFIRM", "REDO", "ABORT"].forEach(v => allowed.add(v));
    else if (tail.includes("READY")) {
      allowed.add("READY"); allowed.add("ABORT");
      if (tail.slice(-350).includes("SKIP")) allowed.add("SKIP");
    } else allowed.add("ABORT");
  }
  $$(".prompt-response").forEach(button => button.classList.toggle("hidden", !allowed.has(button.dataset.response)));
}

function renderResults(data) {
  const plots = data.artifacts?.plots || [];
  const hasResults = plots.length > 0 || Boolean(data.artifacts?.pdf);
  $("#results-empty").classList.toggle("hidden", hasResults);
  $("#results-content").classList.toggle("hidden", !data.active_session);
  if (!data.active_session) return;
  const pdf = $("#open-pdf");
  pdf.classList.toggle("hidden", !data.artifacts.pdf);
  if (data.artifacts.pdf) pdf.href = data.artifacts.pdf;
  const campaignComplete = data.manifest && data.manifest.completed === data.manifest.total;
  $("#promote-candidate").classList.toggle("hidden", !campaignComplete);
  const artifactNames = {latex: "LaTeX source", markdown: "Markdown report", inventory: "Parameter inventory", mpc_bundle: "MPC / simulation bundle", campaign_log: "Campaign log"};
  $("#artifact-links").innerHTML = Object.entries(artifactNames).filter(([key]) => data.artifacts[key]).map(([key, name]) =>
    `<a class="artifact-link" href="${data.artifacts[key]}" target="_blank" rel="noopener">${escapeHtml(name)} ↗</a>`
  ).join("");
  const stageMap = new Map(data.stages.map(stage => [stage.key, stage]));
  $("#results-grid").innerHTML = plots.map(plot => {
    const stage = stageMap.get(plot.name);
    const title = stage?.guide.title || prettyKey(plot.name);
    const values = stage?.headline_values || [];
    return `<article class="result-card"><a href="${plot.url}" target="_blank" rel="noopener"><img src="${plot.url}?t=${Date.now()}" alt="${escapeHtml(title)}"></a><div class="result-copy">
      ${stage ? `<span class="status-pill ${stage.status === "completed" ? "good" : stage.status === "failed" ? "failed" : "warning"}"><i></i><span>${escapeHtml(stageStatusLabel(stage.status))}</span></span>` : ""}
      <h3>${escapeHtml(title)}</h3>${values.slice(0, 5).map(value => `<code>${escapeHtml(value)}</code>`).join("")}</div></article>`;
  }).join("");
}

function renderAll(data, forceForm = false) {
  const sessionChanged = state.activeSession !== data.active_session;
  state.activeSession = data.active_session;
  state.data = data;
  if (sessionChanged) {
    state.selectedStage = data.next_stage;
    state.formSession = null;
    state.jobCursor = 0;
    state.jobText = "";
  }
  renderSidebar(data);
  renderPrepare(data, forceForm || sessionChanged);
  if (state.view === "journey") renderJourney(data);
  if (state.view === "results") renderResults(data);
}

async function refresh(forceForm = false) {
  try {
    renderAll(await api("/api/status"), forceForm);
  } catch (error) {
    notice(error.message, true);
  }
}

function openStageConfirmation(stage) {
  state.pendingStage = stage;
  $("#confirm-title").textContent = `Run stage ${stage.index}: ${stage.guide.title}?`;
  $("#confirm-summary").textContent = `${stage.guide.summary} The runner will stop after this one stage and update the report.`;
  $("#confirm-checks").innerHTML = stage.required_confirmations.map(name => `<label><input type="checkbox" value="${escapeHtml(name)}"><span>${escapeHtml(confirmationLabels[name] || prettyKey(name))}</span></label>`).join("");
  $("#confirm-dialog").showModal();
}

function openTextConfirmation(action) {
  state.textAction = action;
  $("#text-confirm-title").textContent = action.title;
  $("#text-confirm-summary").textContent = action.summary;
  $("#text-confirm-word").textContent = action.word;
  $("#text-confirm-input").value = "";
  $("#text-confirm-dialog").showModal();
}

async function pollJob() {
  try {
    const job = await api(`/api/job?cursor=${state.jobCursor}`);
    if (job.cursor_reset) state.jobText = "";
    if (job.log) {
      state.jobText += job.log;
      if (state.jobText.length > 300000) state.jobText = state.jobText.slice(-300000);
      const terminal = $("#terminal-log");
      const nearBottom = terminal.scrollHeight - terminal.scrollTop - terminal.clientHeight < 80;
      terminal.textContent = state.jobText;
      if (nearBottom || job.running) terminal.scrollTop = terminal.scrollHeight;
    }
    state.jobCursor = job.cursor;
    renderTerminalVisibility(job);
    const endstopPrompt = state.jobText.slice(-600).includes("[a/d z/c x/v l r q]");
    $("#endstop-keys").classList.toggle("hidden", !job.running || !endstopPrompt);
    $("#endstop-help").classList.toggle("hidden", !job.running || !endstopPrompt);
    updatePromptControls(job);
    if (state.lastJobStatus !== job.status && ["succeeded", "failed", "stopped"].includes(job.status)) {
      notice(job.status === "succeeded" ? "Stage job finished. Review the new plot and fitted values." : "The job stopped. Read the terminal and stage failure page before deciding what to do.", job.status !== "succeeded");
      await refresh(true);
    }
    state.lastJobStatus = job.status;
  } catch (error) {
    console.error(error);
  }
}

async function sendTerminal(response, newline = true) {
  try {
    await api("/api/job/input", {method: "POST", body: {response, newline}});
  } catch (error) {
    notice(error.message, true);
  }
}

async function createSession() {
  try {
    await api("/api/session/new", {method: "POST", body: {}});
    notice("New calibration session created. Complete the preparation checklist before running.");
    switchView("prepare");
    await refresh(true);
  } catch (error) { notice(error.message, true); }
}

async function selectSession(sessionId) {
  if (!sessionId) return;
  try {
    await api("/api/session/select", {method: "POST", body: {session_id: sessionId}});
    await refresh(true);
  } catch (error) { notice(error.message, true); }
}

function bindEvents() {
  $$(".nav-item").forEach(button => button.addEventListener("click", () => switchView(button.dataset.view)));
  $("#refresh-button").addEventListener("click", () => refresh(true));
  $("#new-session").addEventListener("click", createSession);
  $("#new-session-active").addEventListener("click", createSession);
  $("#session-select").addEventListener("change", event => selectSession(event.target.value));
  $("#session-select-active").addEventListener("change", event => selectSession(event.target.value));
  $("#metrology-form").addEventListener("submit", async event => {
    event.preventDefault();
    try {
      const response = await api("/api/metrology", {method: "POST", body: collectMetrology()});
      notice(response.validation.ready ? "Measurements saved and ready for the metrology stage." : "Measurements saved. Complete the highlighted items before running metrology.");
      await refresh(true);
    } catch (error) { notice(error.message, true); }
  });
  $("#convert-axles").addEventListener("click", () => {
    const front = Number($("#front-kg").value);
    const rear = Number($("#rear-kg").value);
    if (!Number.isFinite(front) || !Number.isFinite(rear) || front <= 0 || rear <= 0) {
      notice("Enter positive front and rear scale readings in kilograms.", true); return;
    }
    $("#field-required-front_axle_load_N-value").value = (front * 9.80665).toFixed(4);
    $("#field-required-rear_axle_load_N-value").value = (rear * 9.80665).toFixed(4);
    notice("Converted both axle readings to newtons. Save the measurements when the closure check passes.");
  });
  $("#confirm-form").addEventListener("submit", async event => {
    event.preventDefault();
    if (!state.pendingStage) return;
    const checked = $$("#confirm-checks input:checked").map(input => input.value);
    if (checked.length !== state.pendingStage.required_confirmations.length) {
      notice("Confirm every safety item before starting.", true); return;
    }
    try {
      await api("/api/run", {method: "POST", body: {stage: state.pendingStage.key, confirmations: checked}});
      $("#confirm-dialog").close();
      state.jobCursor = 0; state.jobText = ""; state.lastJobStatus = "running";
      switchView("journey");
      await refresh();
    } catch (error) { notice(error.message, true); }
  });
  $("#text-confirm-form").addEventListener("submit", async event => {
    event.preventDefault();
    const action = state.textAction;
    if (!action) return;
    const value = $("#text-confirm-input").value;
    if (value !== action.word) { notice(`Type ${action.word} exactly to continue.`, true); return; }
    try {
      await api(action.endpoint, {method: "POST", body: {...action.body, confirmation: value}});
      $("#text-confirm-dialog").close();
      state.jobCursor = 0; state.jobText = "";
      await refresh();
    } catch (error) { notice(error.message, true); }
  });
  $("#recover-button").addEventListener("click", () => openTextConfirmation({
    title: "Recover the interrupted VESC transaction?", word: "RECOVER",
    summary: "This restores the exact source configuration saved before the interrupted stage and rebuilds the workspace. Do not move the car until it finishes.",
    endpoint: "/api/recover", body: {},
  }));
  $("#build-report").addEventListener("click", async () => {
    try {
      await api("/api/report", {method: "POST", body: {}});
      state.jobCursor = 0; state.jobText = "";
      switchView("journey");
      await refresh();
    } catch (error) { notice(error.message, true); }
  });
  $("#promote-candidate").addEventListener("click", () => openTextConfirmation({
    title: "Promote the fully validated candidate?", word: "PROMOTE",
    summary: "This is the only permanent configuration action in the GUI. It writes the final gated VESC/odometry parameters to the production source and rebuilds. The runner still blocks promotion unless every required stage passed.",
    endpoint: "/api/promote", body: {},
  }));
  $("#stop-job").addEventListener("click", async () => {
    if (!confirm("Interrupt the active stage? The runner will attempt its normal safety stop and configuration restore.")) return;
    try { await api("/api/job/stop", {method: "POST", body: {}}); }
    catch (error) { notice(error.message, true); }
  });
  $$(".quick-responses [data-response]").forEach(button => button.addEventListener("click", () => sendTerminal(button.dataset.response, true)));
  $$(".endstop-keys [data-key]").forEach(button => button.addEventListener("click", () => sendTerminal(button.dataset.key, false)));
  $("#terminal-input-form").addEventListener("submit", event => {
    event.preventDefault();
    const input = $("#terminal-input");
    if (input.value) sendTerminal(input.value, true);
    input.value = "";
  });
}

async function start() {
  bindEvents();
  const requestedView = new URLSearchParams(window.location.search).get("view");
  if (["prepare", "journey", "results"].includes(requestedView)) switchView(requestedView);
  await refresh(true);
  setInterval(() => refresh(false), 2500);
  setInterval(pollJob, 500);
}

document.addEventListener("DOMContentLoaded", start);
