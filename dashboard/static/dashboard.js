"use strict";

const statusTopic = "picocar/vehicle1/status";
const telemetryTopic = "picocar/vehicle1/telemetry";
const byId = (id) => document.getElementById(id);
const views = {
    overview: ["Overview", "Connection evidence and subsystem coverage for vehicle1."],
    motion: ["Motion control", "Buddy 2: motors, encoders and movement feedback."],
    line: ["Line & barcode", "Buddy 3: IR sensors, line tracking and barcode results."],
    imu: ["IMU & terrain", "Buddy 4: current motion measurements and saved event results."],
    obstacle: ["Obstacle detection", "Buddy 5: ultrasonic sensing and obstacle-monitor feedback."],
    safety: ["Controller & safety", "Operating state, execution evidence and vehicle safety."],
    diagnostics: ["MQTT diagnostics", "Raw payloads, envelope fields and browser-observed messages."],
};
const subsystems = [
    { view: "motion", source: 2, name: "Motion control", owner: "Buddy 2", coverage: "Not integrated" },
    { view: "line", source: 3, name: "Line & barcode", owner: "Buddy 3", coverage: "Not integrated" },
    { view: "imu", source: 4, name: "IMU & terrain", owner: "Buddy 4", coverage: "Not integrated" },
    { view: "obstacle", source: 5, name: "Obstacle detection", owner: "Buddy 5", coverage: "Awaiting payload" },
    { view: "safety", source: null, name: "Controller & safety", owner: "Vehicle controller", coverage: "Not integrated" },
];
const sections = {
    motion: [
        ["Motor feedback", [["Left motor speed", "motor-left"], ["Right motor speed", "motor-right"], ["Motor status", "motor-status"]]],
        ["Movement & encoders", [["Left encoder", "encoder-left"], ["Right encoder", "encoder-right"], ["Movement direction", "movement-direction"], ["Distance travelled (cm, signed)", "distance-travelled"]]],
    ],
    line: [
        ["Line sensing", [["Left IR sensor", "ir-left"], ["Centre IR sensor", "ir-centre"], ["Right IR sensor", "ir-right"]]],
        ["Tracking & barcode", [["Line detection", "line-detection"], ["Tracking state", "tracking-state"], ["Barcode result", "barcode-result"]]],
    ],
    imu: [
        ["Current orientation", [["IMU status", "imu-status"], ["Pitch", "pitch"], ["Roll", "roll"], ["Wheel-height difference (cm, preliminary)", "wheel-height"]]],
        ["Terrain & motion", [["Terrain classification", "terrain"], ["Side classification", "side"], ["Estimated motion", "estimated-motion"], ["Turn rate", "turn-rate"]]],
    ],
    obstacle: [
        ["Range observations", [["Ultrasonic range", "ultrasonic-range"], ["Front clearance", "front-clearance"], ["Commanded servo angle", "servo-angle"]]],
        ["Obstacle monitor", [["Detection state", "obstacle-state"], ["Reported fault", "obstacle-fault"], ["Range status", "range-validity"], ["Line reacquired event", "line-reacquired"]]],
    ],
};

let brokerConnected = false;
let statusAvailable = false;
let heartbeatFresh = false;
let heartbeatAge = null;
let telemetryAvailable = false;
let latestEnvelope = null;
let lastTelemetryChange = null;
let hasTelemetryObservation = false;
let refreshInFlight = false;
let refreshTimer;
let lastPollAt = null;
const previousPayloads = new Map();
const sourceObservations = new Map();
const obstacleObservations = new Map();
const pendingCommands = new Set();
let commandOrder = 0;
let logCount = 0;

function metricPanel(title, fields) {
    const panel = document.createElement("section");
    panel.className = "panel";
    const heading = document.createElement("h2");
    heading.textContent = title;
    const list = document.createElement("dl");
    list.className = "metric-list";
    for (const [label, id] of fields) {
        const row = document.createElement("div");
        const term = document.createElement("dt");
        term.textContent = label;
        const value = document.createElement("dd");
        value.id = id;
        value.textContent = "N/A";
        row.append(term, value);
        list.append(row);
    }
    panel.append(heading, list);
    return panel;
}

for (const [view, panels] of Object.entries(sections)) {
    const root = byId(`view-${view}`);
    const note = document.createElement("div");
    note.className = "integration-note";
    note.textContent = view === "obstacle"
        ? "Buddy 5 values are last observations, not a synchronized live snapshot. Width and depth require matching type-56 validity bits. Range status and line reacquisition are not transmitted."
        : view === "imu"
        ? "Buddy 4 is not connected to MQTT. Current measurements and saved event results remain N/A. A fresh vehicle heartbeat does not establish sensor freshness."
        : "Sensor measurements are not integrated into the current MQTT interface. N/A does not mean zero or healthy.";
    const grid = document.createElement("div");
    grid.className = "two-column";
    for (const [title, fields] of panels) grid.append(metricPanel(title, fields));
    root.append(note, grid);
    if (view === "motion") {
        const distanceNote = document.createElement("p");
        distanceNote.className = "panel-note";
        distanceNote.textContent = "Distance is signed odometry since reset, not cumulative path length. Its MQTT mapping is not integrated, so it remains N/A.";
        root.append(distanceNote);
    }
    if (view === "obstacle") {
        const heading = document.createElement("div");
        heading.className = "section-heading";
        heading.innerHTML = "<h2>Obstacle profile &amp; bypass plan</h2><span class='muted'>Reported estimates, not certified clearance</span>";
        const profile = document.createElement("div");
        profile.className = "two-column";
        profile.append(
            metricPanel("Width & bypass", [["Estimated obstacle width", "obstacle-width"], ["Width validity", "obstacle-width-validity"], ["Chosen bypass side", "bypass-side"], ["Planned sideways offset", "bypass-offset"]]),
            metricPanel("Depth estimate", [["Depth lower bound", "depth-low"], ["Depth upper bound", "depth-high"], ["Depth validity", "depth-validity"], ["Validity packet sequence", "obstacle-profile-sequence"]]),
        );
        const profileNote = document.createElement("p");
        profileNote.className = "panel-note";
        profileNote.textContent = "Unknown estimates are never shown as -1 mm. Signed bypass offsets are positive left and negative right. Polling may miss packets in a telemetry burst.";
        root.append(heading, profile, profileNote);
    }
    if (view === "imu") {
        const diagnosticsHeading = document.createElement("div");
        diagnosticsHeading.className = "section-heading";
        diagnosticsHeading.innerHTML = "<h2>Sensor diagnostics</h2><span class='muted'>Sensor freshness and measurement quality are separate</span>";
        const diagnostics = document.createElement("div");
        diagnostics.className = "two-column";
        diagnostics.append(
            metricPanel("Acquisition diagnostics", [["Sensor freshness", "imu-freshness"], ["Last reading age (ms)", "imu-sample-age"], ["I²C / configuration errors", "imu-io-errors"]]),
            metricPanel("Measurement quality", [["IMU measurement quality", "imu-quality"], ["Clipped sample count", "imu-clipped-samples"], ["Unsuitable tilt sample count", "imu-unsuitable-samples"]]),
        );
        root.append(diagnosticsHeading, diagnostics);
        const heading = document.createElement("div");
        heading.className = "section-heading";
        heading.innerHTML = "<h2>Saved event results</h2><span class='muted'>Separate from current sensor measurements</span>";
        const events = document.createElement("div");
        events.className = "two-column";
        const ramp = metricPanel("Ramp-height result", [["Saved stable peak height (cm)", "peak-height"], ["Saved median peak height (cm)", "median-peak-height"], ["Detected side", "peak-side"], ["Event timestamp (device uptime)", "peak-time"]]);
        const rampNote = document.createElement("p");
        rampNote.className = "panel-note";
        rampNote.textContent = "Preliminary estimates. Stable and median peaks have independent validity; saved results are not current wheel-height readings.";
        ramp.append(rampNote);
        const impact = metricPanel("Possible-impact result", [["Latest possible-impact event", "impact-result"], ["Event timestamp (device uptime)", "impact-time"], ["Possible-impact event count", "impact-count"]]);
        const impactNote = document.createElement("p");
        impactNote.className = "panel-note";
        impactNote.textContent = "Observational, not a confirmed collision. Event history remains separate from current sensor validity.";
        impact.append(impactNote);
        events.append(ramp, impact);
        const note = document.createElement("p");
        note.className = "panel-note";
        note.textContent = "Saved results and counters are not integrated into MQTT. N/A does not mean no event occurred or a count of zero.";
        root.append(heading, events, note);
    }
    const details = document.createElement("details");
    const summary = document.createElement("summary");
    summary.textContent = "Integration details and last source observation";
    const explanation = document.createElement("p");
    explanation.id = `${view}-observation`;
    explanation.textContent = "No payload observed from this source in this browser session.";
    const protocolNote = document.createElement("p");
    protocolNote.textContent = view === "obstacle"
        ? "Source 5: type 50 reports state/fault; 51 reports width and the depth upper bound; 52 reports bypass side and signed offset; 53 reports depth bounds; 54 reports front clearance and commanded servo angle. Type 56 value1 carries width/depth validity in bits 0/1, independently of the omitted envelope flags. Estimates require matching packet sequences."
        : view === "imu"
        ? "Buddy 4 has a native snapshot, but no MQTT publisher submission or agreed payload mapping for its measurements. Source 4 and IMU/hump type identifiers alone do not define units, validity or timestamps. Raw payloads remain available in MQTT diagnostics without populating these fields."
        : "Raw source/type fields remain visible in MQTT diagnostics. Measurements need confirmed payload meanings, units and validity before integration.";
    details.append(summary, explanation, protocolNote);
    if (view === "obstacle") {
        for (const text of [
            "The firmware publishes about once per second. State changes are logged to the console; immediate MQTT state-change updates are not wired. A coherent status snapshot is not available through the latest-payload API. Sequence matching reduces mixing but does not provide an atomic device snapshot or a boot identity.",
            "The standard and full-test publishers do not send a range-status field. The full-test profile maps no echo to -1, which is also used for unavailable data. The dashboard therefore reports Unknown instead of declaring a valid range, no echo or a clear path. A sensor fault is shown separately; it is not treated as a range-status packet.",
            "Line reacquisition belongs to shared navigation and is not published as an event. Returning to Monitoring is not proof of a line-reacquired event. Those results remain N/A until integrated.",
        ]) {
            const paragraph = document.createElement("p");
            paragraph.textContent = text;
            details.append(paragraph);
        }
    }
    if (view === "imu") {
        for (const text of [
            "Firmware quality states are GRAVITY_LIKE, NON_GRAVITY, CLIPPED, NONFINITE and UNAVAILABLE. Acquisition validity and gravity-based tilt validity are distinct. Clipped samples also contribute to the unsuitable tilt count; these counts must not be added together.",
            "Sensor age must come from Buddy 4 sample timing and validity, never the vehicle heartbeat or browser payload age. Old or invalid readings must not be shown as live. Ramp event timestamps are device uptime, not wall-clock time; detected sides include LEFT_WHEEL_UP, RIGHT_WHEEL_UP and SIDE_LEVEL.",
            "Possible-impact history is observational. The current Buddy 4 snapshot does not expose a possible-obstruction event or timestamp, so those fields are omitted.",
        ]) {
            const paragraph = document.createElement("p");
            paragraph.textContent = text;
            details.append(paragraph);
        }
    }
    root.append(details);
}

for (const subsystem of subsystems) {
    const row = document.createElement("tr");
    const identity = document.createElement("td");
    identity.textContent = subsystem.name;
    const owner = document.createElement("small");
    owner.textContent = subsystem.owner;
    identity.append(owner);
    const coverage = document.createElement("td");
    const tag = document.createElement("span");
    tag.className = "tag neutral";
    tag.id = `${subsystem.view}-coverage`;
    tag.textContent = subsystem.coverage;
    coverage.append(tag);
    const observation = document.createElement("td");
    observation.id = `${subsystem.view}-last-seen`;
    observation.textContent = "N/A";
    const destination = document.createElement("td");
    const link = document.createElement("a");
    link.href = `#${subsystem.view}`;
    link.className = "text-link";
    link.textContent = "View";
    link.setAttribute("aria-label", `View ${subsystem.name.toLowerCase()}`);
    destination.append(link);
    row.append(identity, coverage, observation, destination);
    byId("subsystem-rows").append(row);
}

function setMenu(open) {
    byId("sidebar").classList.toggle("is-open", open);
    byId("menu-toggle").setAttribute("aria-expanded", String(open));
}

function selectView(focus = false) {
    const name = location.hash.slice(1);
    const view = Object.hasOwn(views, name) ? name : "overview";
    for (const key of Object.keys(views)) byId(`view-${key}`).hidden = key !== view;
    for (const link of document.querySelectorAll("nav a")) {
        if (link.hash === `#${view}`) link.setAttribute("aria-current", "page");
        else link.removeAttribute("aria-current");
    }
    byId("view-title").textContent = views[view][0];
    byId("view-description").textContent = views[view][1];
    document.title = `${views[view][0]} | PicoCar Control Center`;
    setMenu(false);
    if (focus) {
        byId("main").focus({ preventScroll: true });
        window.scrollTo({ top: 0 });
    }
}

byId("menu-toggle").addEventListener("click", () => setMenu(byId("menu-toggle").getAttribute("aria-expanded") !== "true"));
document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && byId("menu-toggle").getAttribute("aria-expanded") === "true") {
        setMenu(false);
        byId("menu-toggle").focus();
    }
});
document.addEventListener("click", (event) => {
    if (!event.target.closest("#sidebar, #menu-toggle")) setMenu(false);
});
document.querySelectorAll("#sidebar a").forEach((link) => link.addEventListener("click", () => {
    if (link.hash === location.hash || (!location.hash && link.hash === "#overview")) selectView(true);
}));
window.addEventListener("hashchange", () => selectView(true));
document.querySelector(".skip-link").addEventListener("click", (event) => {
    event.preventDefault();
    byId("main").focus();
});
selectView();

function setState(id, label, tone) {
    byId(id).textContent = label;
    byId(id).dataset.tone = tone;
}

function updateCommandButtons() {
    byId("start-command").disabled = !statusAvailable || !brokerConnected || pendingCommands.size > 0;
    // STOP remains independent of an in-flight START and can be attempted when connection evidence is unknown.
    byId("stop-command").disabled = pendingCommands.has("STOP") || (statusAvailable && !brokerConnected);
    byId("start-command").title = statusAvailable && brokerConnected ? "Publish START; execution remains unconfirmed" : "START requires a confirmed dashboard-to-broker connection";
    byId("stop-command").title = statusAvailable && !brokerConnected ? "MQTT broker disconnected" : "Publish STOP; this is not a hardware emergency stop";
}

function ageLabel(milliseconds) {
    const seconds = Math.max(0, Math.floor(milliseconds / 1000));
    if (seconds < 60) return `${seconds} s`;
    if (seconds < 3600) return `${Math.floor(seconds / 60)} min`;
    return `${Math.floor(seconds / 3600)} h`;
}

function updateConnectionDisplay() {
    if (!statusAvailable) {
        setState("mqtt-connection", "Unknown", "neutral");
        setState("vehicle-connection", "Unknown", "neutral");
        byId("heartbeat-age").textContent = "Age N/A";
        byId("connection-headline").textContent = "Connection evidence unavailable";
        byId("connection-explanation").textContent = "The status API is unavailable. Broker and vehicle state cannot be confirmed.";
    } else {
        setState("mqtt-connection", brokerConnected ? "Connected" : "Disconnected", brokerConnected ? "good" : "fault");
        const localAge = heartbeatAge === null ? null : heartbeatAge + (Date.now() - lastPollAt) / 1000;
        const fresh = heartbeatFresh && localAge !== null && localAge < 15;
        byId("heartbeat-age").textContent = localAge === null ? "Age N/A" : `${localAge.toFixed(1)} s since status`;
        setState("vehicle-connection", !brokerConnected ? "Link disconnected" : localAge === null ? "Not received" : fresh ? "Fresh" : "Stale", !brokerConnected ? "fault" : fresh ? "good" : "warning");
        byId("connection-headline").textContent = !brokerConnected ? "MQTT link disconnected" : fresh ? "Heartbeat fresh. Readiness unconfirmed." : "Waiting for a fresh vehicle heartbeat";
        byId("connection-explanation").textContent = !brokerConnected
            ? "The dashboard is reachable, but its broker connection is down. Cached payloads do not establish a live vehicle link."
            : fresh ? "Status traffic is recent. Controller execution and safety feedback are still needed to confirm vehicle readiness."
            : "The broker is connected, but vehicle status is missing or stale. Connection alone does not establish readiness.";
    }
    updateCommandButtons();
}

function parseEnvelope(payload) {
    const fields = {};
    for (const match of payload.matchAll(/(?:^|\s)(src|type|seq|value1|value2|flags)=(-?\d+)(?=\s|$)/g)) {
        if (Object.hasOwn(fields, match[1])) return null;
        const number = Number(match[2]);
        if (!Number.isSafeInteger(number)) return null;
        fields[match[1]] = number;
    }
    if (!["src", "type", "seq", "value1", "value2"].every((key) => Object.hasOwn(fields, key))) return null;
    if (fields.src < 0 || fields.src > 255 || fields.type < 0 || fields.type > 255 || fields.seq < 0 || fields.seq > 4294967295) return null;
    if ([fields.value1, fields.value2].some((value) => value < -2147483648 || value > 2147483647)) return null;
    return fields;
}

function addLog(topic, payload, initial) {
    byId("observation-log").querySelector(".empty-log")?.remove();
    const row = document.createElement("li");
    const meta = document.createElement("div");
    meta.className = "log-meta";
    const time = document.createElement("time");
    const now = new Date();
    time.dateTime = now.toISOString();
    time.textContent = now.toLocaleTimeString();
    const label = document.createElement("span");
    label.textContent = topic === statusTopic ? "Status" : "Telemetry";
    const origin = document.createElement("span");
    origin.textContent = initial ? "Initial API value; may be cached" : "Changed API value";
    meta.append(time, label, origin);
    const raw = document.createElement("code");
    raw.className = "log-payload";
    raw.textContent = payload;
    row.append(meta, raw);
    byId("observation-log").prepend(row);
    if (++logCount > 100) {
        byId("observation-log").lastElementChild.remove();
        logCount = 100;
    }
}

function recordPayload(topic, payload) {
    if (typeof payload !== "string" || previousPayloads.get(topic) === payload) return;
    const initial = !previousPayloads.has(topic);
    previousPayloads.set(topic, payload);
    addLog(topic, payload, initial);
    if (topic !== telemetryTopic) return;
    hasTelemetryObservation = true;
    lastTelemetryChange = initial ? null : Date.now();
    latestEnvelope = parseEnvelope(payload);
    if (!latestEnvelope) return;
    const observation = { fields: latestEnvelope, at: Date.now(), initial };
    if (latestEnvelope.src === 5 && latestEnvelope.type >= 50 && latestEnvelope.type <= 56) {
        const previous = sourceObservations.get(5);
        if (previous && latestEnvelope.seq < previous.fields.seq) obstacleObservations.clear();
        obstacleObservations.set(latestEnvelope.type, observation);
    }
    sourceObservations.set(latestEnvelope.src, observation);
}

function observationLabel(observation) {
    if (!observation) return "N/A";
    if (observation.initial) return "Initial cache; age unknown";
    return `Observed ${ageLabel(Date.now() - observation.at)} ago`;
}

function updateMeasurements() {
    const stateNames = ["Monitoring", "Waiting for stop", "Scanning", "Processing", "Avoiding", "Line recovery", "Error"];
    const faultNames = ["None reported", "Sensor fault", "Feedback fault", "Timeout", "No route", "Controller fault", "Pose fault", "Profile fault", "Servo fault"];
    function measured(id, value, observation, tone = "neutral") {
        const element = byId(id);
        element.textContent = value;
        element.dataset.tone = tone;
        const note = document.createElement("small");
        note.textContent = !statusAvailable || !telemetryAvailable ? "Connection unknown; last observation" : !brokerConnected ? "Disconnected; last observation" : !heartbeatFresh || heartbeatAge === null || heartbeatAge + (Date.now() - lastPollAt) / 1000 >= 15 ? "Heartbeat stale; last observation" : observationLabel(observation);
        element.append(note);
    }
    for (const subsystem of subsystems) {
        const observation = sourceObservations.get(subsystem.source);
        byId(`${subsystem.view}-last-seen`).textContent = observationLabel(observation);
        if (observation) {
            byId(`${subsystem.view}-coverage`).textContent = "Payload observed";
            byId(`${subsystem.view}-observation`).textContent = `Source ${observation.fields.src}, type ${observation.fields.type}, sequence ${observation.fields.seq}. ${observationLabel(observation)}. Validity is not established by receiving a payload.`;
        }
    }
    const latest = sourceObservations.get(5);
    if (!latest) return;
    const state = obstacleObservations.get(50);
    measured("obstacle-state", state ? stateNames[state.fields.value1] ?? `Unknown state (${state.fields.value1})` : "Unknown", state || latest, state?.fields.value1 === 6 ? "fault" : "neutral");
    measured("obstacle-fault", state ? faultNames[state.fields.value2] ?? `Unknown fault (${state.fields.value2})` : "Unknown", state || latest, state && state.fields.value2 !== 0 ? "fault" : "neutral");
    const range = obstacleObservations.get(54);
    measured("front-clearance", !range || range.fields.value1 === -1 ? "Unknown" : `${range.fields.value1.toLocaleString()} mm`, range || latest);
    measured("servo-angle", range && range.fields.value2 >= -60 && range.fields.value2 <= 60 ? `${range.fields.value2}°` : "N/A", range || latest);
    measured("range-validity", "Unknown", range || latest);
    const validity = obstacleObservations.get(56);
    const mask = validity?.fields.value1;
    const knownMask = validity && mask >= 0 && (mask & ~23) === 0;
    measured("obstacle-profile-sequence", validity ? String(validity.fields.seq) : "N/A", validity || latest);
    for (const [type, bit, ids, validityId] of [
        [51, 1, ["obstacle-width"], "obstacle-width-validity"],
        [53, 2, ["depth-low", "depth-high"], "depth-validity"],
    ]) {
        const estimate = obstacleObservations.get(type);
        // Validity from another report must never approve an older estimate.
        const matched = knownMask && estimate && estimate.fields.seq === validity.fields.seq && validity.fields.seq === latest.fields.seq;
        const values = estimate ? [estimate.fields.value1, estimate.fields.value2] : [];
        const numericValid = type === 51 ? values[0] > 0 : values[0] >= 0 && values[1] >= values[0];
        const valid = matched && (mask & bit) !== 0 && numericValid;
        const label = !matched ? "Unknown" : !(mask & bit) ? "Invalid (reported)" : valid ? "Valid (reported)" : "Inconsistent";
        measured(validityId, label, validity || estimate || latest);
        ids.forEach((id, index) => measured(id, valid ? `${values[index].toLocaleString()} mm` : "Unknown", estimate || validity || latest));
    }
    const plan = obstacleObservations.get(52);
    const planned = plan && plan.fields.seq === latest.fields.seq && [1, -1].includes(plan.fields.value1);
    measured("bypass-side", planned ? plan.fields.value1 === 1 ? "Left" : "Right" : "Unknown", plan || latest);
    measured("bypass-offset", planned ? `${plan.fields.value2.toLocaleString()} mm` : "Unknown", plan || latest);
}

function updateTelemetryDisplay() {
    byId("telemetry-age").textContent = !hasTelemetryObservation ? "N/A" : lastTelemetryChange === null ? "Cached; age unknown" : `Changed ${ageLabel(Date.now() - lastTelemetryChange)} ago`;
    const fields = latestEnvelope;
    const types = { 1: "Heartbeat", 2: "State", 3: "Motor speed", 4: "Encoder", 5: "Line sensor", 6: "Barcode", 7: "IMU", 8: "Hump", 9: "Ultrasonic", 50: "Obstacle state", 51: "Obstacle size", 52: "Bypass plan", 53: "Depth bounds", 54: "Front clearance", 55: "Obstacle correlation", 56: "Obstacle validity" };
    const rawSequence = previousPayloads.get(telemetryTopic)?.match(/(?:^|\s)seq=(\d+)(?=\s|$)/);
    const sequence = rawSequence && Number(rawSequence[1]) <= 4294967295 ? rawSequence[1] : "N/A";
    byId("sequence").textContent = fields ? String(fields.seq) : sequence;
    byId("source-type").textContent = fields ? `${fields.src} / ${fields.type}` : "N/A";
    for (const [key, field] of [["source", "src"], ["type", "type"], ["sequence", "seq"], ["value1", "value1"], ["value2", "value2"], ["flags", "flags"]]) {
        byId(`diag-${key}`).textContent = fields && Object.hasOwn(fields, field) ? String(fields[field]) : "N/A";
    }
    byId("parse-note").textContent = !hasTelemetryObservation ? "No telemetry payload received." : !fields ? "Unrecognised or incomplete envelope. The raw payload is preserved above." : `${((fields.src === 5 || fields.type <= 9) && types[fields.type]) || "Unmapped telemetry type"}. Raw values have no inferred units. Missing flags leave validity unknown.`;
    updateMeasurements();
}

async function requestJSON(path, options = {}) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
        const response = await fetch(path, { cache: "no-store", ...options, signal: controller.signal });
        const result = await response.json();
        if (!response.ok) throw new Error(result.detail || `Request failed (${response.status})`);
        return result;
    } finally {
        clearTimeout(timeout);
    }
}

async function refreshDashboard() {
    if (refreshInFlight) return;
    clearTimeout(refreshTimer);
    refreshInFlight = true;
    byId("refresh-button").disabled = true;
    byId("refresh-button").setAttribute("aria-busy", "true");
    let statusReceivedAt;
    const [status, telemetry] = await Promise.allSettled([
        requestJSON("/api/status").then((value) => {
            statusReceivedAt = Date.now();
            return value;
        }),
        requestJSON("/api/telemetry"),
    ]);
    statusAvailable = status.status === "fulfilled" && typeof status.value?.mqtt_connected === "boolean" && typeof status.value?.vehicle_heartbeat_fresh === "boolean" && (status.value?.heartbeat_age_seconds === null || (typeof status.value?.heartbeat_age_seconds === "number" && Number.isFinite(status.value.heartbeat_age_seconds) && status.value.heartbeat_age_seconds >= 0));
    if (statusAvailable) {
        brokerConnected = status.value.mqtt_connected;
        heartbeatFresh = status.value.vehicle_heartbeat_fresh;
        heartbeatAge = status.value.heartbeat_age_seconds;
        // A slow telemetry response must not make older status evidence look fresh.
        lastPollAt = statusReceivedAt;
    }
    telemetryAvailable = telemetry.status === "fulfilled" && telemetry.value !== null && typeof telemetry.value === "object" && !Array.isArray(telemetry.value);
    if (telemetryAvailable) {
        for (const topic of [statusTopic, telemetryTopic]) recordPayload(topic, telemetry.value[topic]);
        if (typeof telemetry.value[statusTopic] === "string") byId("status").textContent = telemetry.value[statusTopic];
        if (typeof telemetry.value[telemetryTopic] === "string") byId("telemetry").textContent = telemetry.value[telemetryTopic];
    }
    setState("api", statusAvailable && telemetryAvailable ? "Connected" : statusAvailable || telemetryAvailable ? "Partial failure" : "Unavailable", statusAvailable && telemetryAvailable ? "good" : "fault");
    byId("api-notice").hidden = statusAvailable && telemetryAvailable;
    updateConnectionDisplay();
    updateTelemetryDisplay();
    refreshInFlight = false;
    byId("refresh-button").disabled = false;
    byId("refresh-button").removeAttribute("aria-busy");
    refreshTimer = setTimeout(refreshDashboard, 1000);
}

async function sendCommand(command) {
    if (!["START", "STOP"].includes(command) || pendingCommands.has(command)) return;
    const order = ++commandOrder;
    pendingCommands.add(command);
    const button = byId(command === "START" ? "start-command" : "stop-command");
    button.setAttribute("aria-busy", "true");
    setState("command-result", `${command}: sending publication request…`, "neutral");
    updateCommandButtons();
    try {
        await requestJSON(`/api/command/${command}`, { method: "POST" });
        if (order === commandOrder) setState("command-result", `${command}: publication queued. Execution unconfirmed.`, "neutral");
    } catch (error) {
        const message = error.name === "AbortError" ? "Request timed out; publication outcome unknown. Check diagnostics before retrying." : error.message;
        if (order === commandOrder) setState("command-result", `${command}: ${message}`, "fault");
    } finally {
        pendingCommands.delete(command);
        button.removeAttribute("aria-busy");
        updateCommandButtons();
    }
}

byId("start-command").addEventListener("click", () => sendCommand("START"));
byId("stop-command").addEventListener("click", () => sendCommand("STOP"));
byId("refresh-button").addEventListener("click", refreshDashboard);
byId("clear-log").addEventListener("click", () => {
    byId("observation-log").replaceChildren();
    const empty = document.createElement("li");
    empty.className = "empty-log";
    empty.textContent = "Log cleared. New changed payloads will appear here.";
    byId("observation-log").append(empty);
    logCount = 0;
});
refreshDashboard();
setInterval(() => {
    updateConnectionDisplay();
    updateTelemetryDisplay();
}, 1000);

// Opt-in local preview: swap CSS without clearing telemetry or replaying commands.
if (new URLSearchParams(location.search).get("preview") === "1") {
    const versions = new Map();
    async function watchPreviewFiles() {
        for (const path of ["/static/dashboard.css", "/static/index.html", "/static/dashboard.js"]) {
            try {
                const response = await fetch(path, { method: "HEAD", cache: "no-store", signal: AbortSignal.timeout(3000) });
                const version = response.headers.get("etag") || response.headers.get("last-modified");
                if (!response.ok || !version) continue;
                if (versions.has(path) && versions.get(path) !== version) {
                    if (path.endsWith(".css")) {
                        const current = document.querySelector('link[rel="stylesheet"]');
                        const updated = current.cloneNode();
                        updated.href = `${path}?v=${Date.now()}`;
                        updated.addEventListener("load", () => current.remove(), { once: true });
                        updated.addEventListener("error", () => updated.remove(), { once: true });
                        current.after(updated);
                    } else if (pendingCommands.size === 0) {
                        location.reload();
                        return;
                    } else {
                        continue;
                    }
                }
                versions.set(path, version);
            } catch {
                // Keep the last loaded interface if the local server is restarting.
            }
        }
        setTimeout(watchPreviewFiles, 1000);
    }
    watchPreviewFiles();
}
