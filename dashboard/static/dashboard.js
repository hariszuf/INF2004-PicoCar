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
        ["Movement & encoders", [["Left encoder", "encoder-left"], ["Right encoder", "encoder-right"], ["Movement direction", "movement-direction"]]],
    ],
    line: [
        ["Line sensing", [["Left IR sensor", "ir-left"], ["Centre IR sensor", "ir-centre"], ["Right IR sensor", "ir-right"]]],
        ["Tracking & barcode", [["Line detection", "line-detection"], ["Tracking state", "tracking-state"], ["Barcode result", "barcode-result"]]],
    ],
    imu: [
        ["Current orientation", [["IMU status", "imu-status"], ["Pitch", "pitch"], ["Roll", "roll"]]],
        ["Terrain & motion", [["Terrain classification", "terrain"], ["Side classification", "side"], ["Estimated motion", "estimated-motion"], ["Turn rate", "turn-rate"]]],
    ],
    obstacle: [
        ["Range observations", [["Ultrasonic range", "ultrasonic-range"], ["Front clearance", "front-clearance"], ["Commanded servo angle", "servo-angle"]]],
        ["Obstacle monitor", [["Detection state", "obstacle-state"], ["Reported fault", "obstacle-fault"], ["Range validity", "range-validity"]]],
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
        ? "Only verified Buddy 5 text fields are interpreted. Sensor validity and sample age are not supplied by the current MQTT envelope."
        : "Sensor measurements are not integrated into the current MQTT interface. N/A does not mean zero or healthy.";
    const grid = document.createElement("div");
    grid.className = "two-column";
    for (const [title, fields] of panels) grid.append(metricPanel(title, fields));
    root.append(note, grid);
    if (view === "imu") {
        const heading = document.createElement("div");
        heading.className = "section-heading";
        heading.innerHTML = "<h2>Saved event results</h2><span class='muted'>Separate from current sensor measurements</span>";
        const events = document.createElement("div");
        events.className = "two-column";
        events.append(
            metricPanel("Peak-height result", [["Saved peak height", "peak-height"], ["Event time", "peak-time"]]),
            metricPanel("Possible-impact result", [["Saved impact result", "impact-result"], ["Event time", "impact-time"]]),
        );
        const note = document.createElement("p");
        note.className = "panel-note";
        note.textContent = "Saved results are not integrated into MQTT. No event has been inferred from live orientation.";
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
        ? "Source 5, type 50 reports state/fault; type 54 reports derived front clearance in mm and commanded servo angle in degrees. A clearance of -1 is unavailable. These are browser-held last observations, not guaranteed current samples. Raw messages remain in MQTT diagnostics."
        : "Raw source/type fields remain visible in MQTT diagnostics. Measurements need confirmed payload meanings, units and validity before integration.";
    details.append(summary, explanation, protocolNote);
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
    sourceObservations.set(latestEnvelope.src, observation);
    if (latestEnvelope.src === 5 && [50, 54].includes(latestEnvelope.type)) obstacleObservations.set(latestEnvelope.type, observation);
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
    const state = obstacleObservations.get(50);
    if (state) {
        measured("obstacle-state", stateNames[state.fields.value1] ?? `Unknown state (${state.fields.value1})`, state, state.fields.value1 === 6 ? "fault" : "neutral");
        measured("obstacle-fault", faultNames[state.fields.value2] ?? `Unknown fault (${state.fields.value2})`, state, state.fields.value2 === 0 ? "neutral" : "fault");
    }
    const range = obstacleObservations.get(54);
    if (range) {
        measured("front-clearance", range.fields.value1 === -1 ? "N/A" : `${range.fields.value1.toLocaleString()} mm`, range);
        measured("servo-angle", range.fields.value2 >= -60 && range.fields.value2 <= 60 ? `${range.fields.value2}°` : "N/A", range);
        measured("range-validity", "Unknown", range);
    }
}

function updateTelemetryDisplay() {
    byId("telemetry-age").textContent = !hasTelemetryObservation ? "N/A" : lastTelemetryChange === null ? "Cached; age unknown" : `Changed ${ageLabel(Date.now() - lastTelemetryChange)} ago`;
    const fields = latestEnvelope;
    const types = { 1: "Heartbeat", 2: "State", 3: "Motor speed", 4: "Encoder", 5: "Line sensor", 6: "Barcode", 7: "IMU", 8: "Hump", 9: "Ultrasonic", 50: "Obstacle state", 54: "Front clearance" };
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
