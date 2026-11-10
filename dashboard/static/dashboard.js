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
    { view: "motion", source: 2, name: "Motion control", owner: "Buddy 2", coverage: "Awaiting payload" },
    { view: "line", source: 3, name: "Line & barcode", owner: "Buddy 3", coverage: "Awaiting payload" },
    { view: "imu", source: 4, name: "IMU & terrain", owner: "Buddy 4", coverage: "Awaiting payload" },
    { view: "obstacle", source: 5, name: "Obstacle detection", owner: "Buddy 5", coverage: "Awaiting payload" },
    { view: "safety", source: null, name: "Controller & safety", owner: "Vehicle controller", coverage: "Awaiting payload" },
];
const sections = {
    motion: [
        ["Motor feedback", [["Left motor speed", "motor-left"], ["Right motor speed", "motor-right"], ["Motor status", "motor-status"]]],
        ["Movement & encoders", [["Left encoder", "encoder-left"], ["Right encoder", "encoder-right"], ["Movement direction", "movement-direction"], ["Distance travelled (cm, signed)", "distance-travelled"]]],
    ],
    line: [
        ["Line sensing", [["Left IR sensor", "ir-left"], ["Barcode IR (GP26)", "ir-centre"], ["Right IR sensor", "ir-right"]]],
        ["Tracking & barcode", [["Sensor pattern", "line-detection"], ["Tracking state", "tracking-state"], ["Barcode result", "barcode-result"]]],
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
const sensorObservations = new Map();
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
        value.textContent = id === "barcode-result" ? "No scan yet" : "N/A";
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
        ? "Buddy 5 values are last observations, not a synchronized live snapshot. Width and depth require matching type-56 validity bits. Raw range is the last HC-SR04+ ping; its angle may differ from the current servo command."
        : view === "imu"
        ? "Buddy 4 reports measurement validity and sample age separately from the vehicle heartbeat."
        : "N/A indicates that a measurement is unavailable.";
    const grid = document.createElement("div");
    grid.className = "two-column";
    for (const [title, fields] of panels) grid.append(metricPanel(title, fields));
    root.append(note, grid);
    if (view === "motion") {
        const distanceNote = document.createElement("p");
        distanceNote.className = "panel-note";
        distanceNote.textContent = "Distance is signed odometry since reset, not cumulative path length. Measured from the vehicle's odometry report.";
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
        const ramp = metricPanel("Ramp-height result", [["Saved stable peak height (cm)", "peak-height"], ["Saved median peak height (cm)", "median-peak-height"], ["Highest Hump Recorded (cm)", "highest-hump"], ["Detected side", "peak-side"], ["Event timestamp (device uptime)", "peak-time"]]);
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
        note.textContent = "Saved results appear after the firmware reports an event.";
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
        ? "Source 5: types 50–56 report state, profile, bypass plan and clearance; type 58 reports the last raw ultrasonic range and HC-SR04+ status (0 valid, 1 stuck high, 2 rise timeout, 3 fall timeout, 4 out of range; -1 stale). Type 56 carries width/depth validity in bits 0/1. Estimates require matching packet sequences."
        : view === "imu"
        ? "Source 4 reports tilt, terrain, motion, acquisition diagnostics and the last saved events. The panel uses each packet’s validity flags and shows event times as device uptime."
        : "Raw source/type fields remain visible in MQTT diagnostics. Measurements need confirmed payload meanings, units and validity before integration.";
    details.append(summary, explanation, protocolNote);
    if (view === "obstacle") {
        for (const text of [
            "The firmware publishes about once per second. State changes are logged to the console; immediate MQTT state-change updates are not wired. The dashboard matches obstacle estimate and validity packets by report sequence.",
            "The driving profile reports the last raw echo separately from filtered front clearance. A timeout is not a measured free path; a very short valid echo may be filtered as a self-reflection.",
            "Line reacquisitions are counted by the vehicle controller (source 6). The servo angle is a command, not measured position.",
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

const telemetryValueKeys = {
    "1/1": ["heartbeat_marker", "reserved"],
    "2/3": ["left_speed_mm_s", "right_speed_mm_s"],
    "2/10": ["odometry_mm", "motor_fault"],
    "3/5": ["ir_mask", "line_state"],
    "3/6": ["barcode_ascii", "barcode_age_ms"],
    "4/7": ["pitch_cdeg", "roll_cdeg"],
    "5/50": ["obstacle_state", "obstacle_fault"],
    "5/51": ["width_mm", "depth_high_mm"],
    "5/52": ["bypass_side", "bypass_offset_mm"],
    "5/53": ["depth_low_mm", "depth_high_mm"],
    "5/54": ["front_clearance_mm", "servo_command_deg"],
    "5/55": ["obstacle_id", "report_uptime_ms"],
    "5/56": ["obstacle_validity_mask", "telemetry_drops"],
    "5/58": ["raw_range_mm", "raw_range_status"],
    "5/57": ["halt_reason_code", "halt_count"],
    "2/4": ["left_encoder_ticks", "right_encoder_ticks"],
    "4/11": ["sample_age_ms", "imu_errors"],
    "4/12": ["clipped_samples", "unsuitable_samples"],
    "4/13": ["terrain_state", "side_state"],
    "4/14": ["motion_state", "yaw_rate_cdeg_s"],
    "4/15": ["wheel_height_mm", "measurement_quality"],
    "4/16": ["stable_peak_mm", "median_peak_mm"],
    "4/17": ["peak_side", "peak_end_ms"],
    "4/18": ["impact_count", "last_impact_ms"],
    "4/19": ["highest_hump_mm", "run_id"],
    "6/57": ["line_reacquired_count", "reserved"],
};

function parseEnvelope(payload) {
    const fields = {};
    for (const match of payload.matchAll(/(?:^|\s)([a-z][a-z0-9_]{0,31})=(-?\d+)(?=\s|$)/g)) {
        if (Object.hasOwn(fields, match[1])) return null;
        const number = Number(match[2]);
        if (!Number.isSafeInteger(number)) return null;
        fields[match[1]] = number;
    }
    const keys = telemetryValueKeys[`${fields.src}/${fields.type}`];
    if (keys) {
        fields.value1 ??= fields[keys[0]];
        const drivingValidity = fields.src === 5 && fields.type === 56 && (fields.value1 & 16);
        fields.value2 ??= fields[drivingValidity ? "reserved" : keys[1]];
    }
    if (!Number.isInteger(fields.value1) || !Number.isInteger(fields.value2)) return null;
    if (!["src", "type", "seq", "value1", "value2"].every((key) => Object.hasOwn(fields, key))) return null;
    if (fields.src < 0 || fields.src > 255 || fields.type < 0 || fields.type > 255 || fields.seq < 0 || fields.seq > 4294967295) return null;
    if ([fields.value1, fields.value2].some((value) => value < -2147483648 || value > 2147483647)) return null;
    return fields;
}

function addLog(topic, payload, initial) {
    byId("observation-log").querySelector(".empty-log")?.remove();
    const row = document.createElement("li");
    row.dataset.topic = topic;
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
    byId("export-log").disabled = false;
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
    if (latestEnvelope.src === 5 && latestEnvelope.type >= 50 && latestEnvelope.type <= 58) {
        const previous = sourceObservations.get(5);
        if (previous && latestEnvelope.seq < previous.fields.seq) obstacleObservations.clear();
        obstacleObservations.set(latestEnvelope.type, observation);
    }
    sourceObservations.set(latestEnvelope.src, observation);
}

function observationLabel(observation) {
    if (!observation) return "N/A";
    if (observation.received) {
        const age = Date.now() - observation.at;
        return `${age >= 3000 ? "Stale · received" : "Received"} ${ageLabel(age)} ago`;
    }
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
        note.textContent = observation?.fields.src === 4
            ? (!statusAvailable || !telemetryAvailable ? "Connection unknown; retained observation" : !brokerConnected ? "Disconnected; retained observation" : Date.now() - observation.at >= 3000 ? "Buddy 4 stale; retained observation" : observationLabel(observation))
            : !statusAvailable || !telemetryAvailable ? "Connection unknown; last observation" : !brokerConnected ? "Disconnected; last observation" : !heartbeatFresh || heartbeatAge === null || heartbeatAge + (Date.now() - lastPollAt) / 1000 >= 15 ? "Heartbeat stale; last observation" : observationLabel(observation);
        if (observation?.received && note.textContent !== observationLabel(observation)) {
            note.textContent += ` · ${observationLabel(observation)}`;
        }
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
    const speed = sensorObservations.get("2/3");
    if (speed) {
        const { value1: left, value2: right } = speed.fields;
        measured("motor-left", `${left} mm/s`, speed);
        measured("motor-right", `${right} mm/s`, speed);
        const direction = left * right < 0 ? "Turning" :
            left + right > 0 ? "Forward" : left + right < 0 ? "Reverse" : "Stationary";
        measured("movement-direction", direction, speed);
    }
    const encoders = sensorObservations.get("2/4");
    if (encoders) {
        measured("encoder-left", `${encoders.fields.value1} ticks`, encoders);
        measured("encoder-right", `${encoders.fields.value2} ticks`, encoders);
    }
    const odometry = sensorObservations.get("2/10");
    if (odometry) {
        measured("distance-travelled", `${(odometry.fields.value1 / 10).toFixed(1)} cm`, odometry);
        const fault = odometry.fields.value2;
        const faults = ["No fault reported", "Emergency stop", "Motor stall"];
        measured("motor-status", faults[fault] ?? `Unknown fault (${fault})`, odometry, fault ? "fault" : "neutral");
    }
    const line = sensorObservations.get("3/5");
    if (line) {
        const { value1: mask, value2: state } = line.fields;
        measured("ir-left", mask & 1 ? "Black" : "White", line);
        measured("ir-centre", mask & 4 ? "Black" : "White", line);
        measured("ir-right", mask & 2 ? "Black" : "White", line);
        measured("line-detection", ["Both white", "Left black", "Right black", "Both black"][mask & 3], line);
        const states = ["Idle", "Straight", "Steer left", "Steer right", "Recovering", "Stopped", "Fault", "Frozen (tilt)", "Junction"];
        measured("tracking-state", states[state] ?? "Unknown", line);
    }
    const barcode = sensorObservations.get("3/6");
    byId("diag-buddy3-line").textContent = line?.payload ?? "Waiting for line telemetry.";
    byId("diag-buddy3-barcode").textContent = barcode?.payload ?? "Waiting for a decoded barcode.";
    if (barcode && barcode.fields.value1 >= 65 && barcode.fields.value1 <= 90 && barcode.fields.value2 >= 0) {
        const elapsedMs = barcode.fields.value2 + Math.max(0, Date.now() - barcode.at);
        const letter = String.fromCharCode(barcode.fields.value1);
        measured("barcode-result", `${letter} · read ${ageLabel(elapsedMs)} ago`, barcode);
    } else {
        byId("barcode-result").textContent = "No scan yet";
    }
    const imuLive = observation => observation && statusAvailable && telemetryAvailable && brokerConnected && Date.now() - observation.at < 3000;
    const imu = sensorObservations.get("4/7");
    if (imu) {
        const valid = imuLive(imu) && (imu.fields.flags & 1) !== 0;
        measured("imu-status", !imuLive(imu) ? "Stale" : valid ? "Tilt valid" : "Tilt unavailable", imu);
        measured("pitch", valid ? `${(imu.fields.value1 / 100).toFixed(2)}°` : "N/A", imu);
        measured("roll", valid ? `${(imu.fields.value2 / 100).toFixed(2)}°` : "N/A", imu);
    }
    const sideNames = { "-1": "Left wheel up", 0: "Level", 1: "Right wheel up" };
    const imuReports = [
        [11, "imu-sample-age", "imu-io-errors", f => f.value1 >= 0 ? `${f.value1} ms` : "N/A", f => f.value2],
        [12, "imu-clipped-samples", "imu-unsuitable-samples", f => f.value1, f => f.value2],
        [13, "terrain", "side", f => f.flags & 1 ? ({ "-1": "Downhill", 0: "Flat", 1: "Uphill" })[f.value1] ?? "Unknown" : "N/A",
            f => f.flags & 1 ? sideNames[f.value2] ?? "Unknown" : "N/A"],
        [14, "estimated-motion", "turn-rate",
            f => f.flags & 1 ? ["Unknown", "Stopped", "Forward", "Reverse", "Turning left", "Turning right"][f.value1] ?? "Unknown" : "N/A",
            f => f.flags & 2 ? `${(f.value2 / 100).toFixed(2)}°/s` : "N/A"],
        [15, "wheel-height", "imu-quality", f => f.flags & 1 ? (f.value1 / 10).toFixed(1) : "N/A",
            f => ["Unavailable", "Gravity-like", "Clipped", "Nonfinite", "Non-gravity"][f.value2] ?? "Unknown"],
        [16, "peak-height", "median-peak-height", f => f.flags & 1 ? (f.value1 / 10).toFixed(1) : "N/A",
            f => f.flags & 2 ? (f.value2 / 10).toFixed(1) : "N/A"],
        [17, "peak-side", "peak-time", f => f.flags & 1 ? sideNames[f.value1] ?? "Unknown" : "N/A",
            f => f.flags & 1 ? `${f.value2 >>> 0} ms` : "N/A"],
        [18, "impact-count", "impact-time", f => f.value1 >>> 0, f => f.value1 ? `${f.value2 >>> 0} ms` : "N/A"],
    ];
    for (const [type, firstId, secondId, firstValue, secondValue] of imuReports) {
        const observation = sensorObservations.get(`4/${type}`);
        if (!observation) continue;
        const live = type === 11 || type === 13 || type === 14 || type === 15;
        const first = type === 11 && observation.fields.value1 >= 0
            ? `${Math.round(observation.fields.value1 + Math.max(0, Date.now() - observation.at))} ms`
            : firstValue(observation.fields);
        measured(firstId, live && !imuLive(observation) ? "N/A" : first, observation);
        measured(secondId, live && !imuLive(observation) && (type === 13 || type === 14 || type === 15) ? "N/A" : secondValue(observation.fields), observation);
    }
    const acquisition = sensorObservations.get("4/11");
    if (acquisition) {
        measured("imu-freshness", !imuLive(acquisition) ? "Stale" : acquisition.fields.flags & 1 ? "Fresh at report" : "Unavailable at report", acquisition);
    }
    const impact = sensorObservations.get("4/18");
    if (impact) {
        measured("impact-result", impact.fields.value1 ? "Possible impact recorded" : "No event recorded", impact);
    }
    const highest = sensorObservations.get("4/19");
    if (highest) measured("highest-hump", highest.fields.flags & 1 ? `${(highest.fields.value1 / 10).toFixed(1)} cm` : "N/A", highest);
    const reacquired = sensorObservations.get("6/57");
    if (reacquired) measured("line-reacquired", `${reacquired.fields.value1 >>> 0} this boot`, reacquired);
    const latest = sourceObservations.get(5);
    if (!latest) return;
    const state = obstacleObservations.get(50);
    measured("obstacle-state", state ? stateNames[state.fields.value1] ?? `Unknown state (${state.fields.value1})` : "Unknown", state || latest, state?.fields.value1 === 6 ? "fault" : "neutral");
    measured("obstacle-fault", state ? faultNames[state.fields.value2] ?? `Unknown fault (${state.fields.value2})` : "Unknown", state || latest, state && state.fields.value2 !== 0 ? "fault" : "neutral");
    const range = obstacleObservations.get(54);
    measured("front-clearance", !range || range.fields.value1 === -1 ? "Unknown" : `${range.fields.value1.toLocaleString()} mm`, range || latest);
    measured("servo-angle", range && range.fields.value2 >= -60 && range.fields.value2 <= 60 ? `${range.fields.value2}°` : "N/A", range || latest);
    const rawRange = obstacleObservations.get(58);
    const rawStatus = rawRange?.fields.value2;
    const rawStatusNames = ["Valid echo", "Echo stuck high", "No echo (rise timeout)", "Incomplete echo (fall timeout)", "Out of range"];
    const rawKnown = rawRange && Number.isInteger(rawStatus) && rawStatus >= 0 && rawStatus < rawStatusNames.length;
    measured("ultrasonic-range", rawKnown && rawStatus === 0 && rawRange.fields.value1 >= 0 ? `${rawRange.fields.value1.toLocaleString()} mm` : "Unknown", rawRange || latest);
    measured("range-validity", rawKnown ? rawStatusNames[rawStatus] : "Unknown", rawRange || latest);
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

const telemetryValueLabels = {
    "1/1": ["Heartbeat marker", "Reserved"],
    "2/3": ["Left wheel speed (mm/s)", "Right wheel speed (mm/s)"],
    "2/10": ["Signed odometry (mm)", "Motor fault code"],
    "3/5": ["IR sensor bitmask", "Line-follow state code"],
    "3/6": ["Decoded letter (ASCII)", "Age at publication (ms)"],
    "4/7": ["Pitch (0.01°)", "Roll (0.01°)"],
    "5/50": ["Obstacle state code", "Obstacle fault code"],
    "5/51": ["Obstacle width (mm)", "Upper depth estimate (mm)"],
    "5/52": ["Bypass side code", "Signed bypass offset (mm)"],
    "5/53": ["Lower depth estimate (mm)", "Upper depth estimate (mm)"],
    "5/54": ["Front clearance (mm)", "Commanded servo angle (°)"],
    "5/55": ["Obstacle ID", "Report uptime (ms)"],
    "5/56": ["Obstacle validity bitmask", "Telemetry drops"],
    "5/58": ["Raw ultrasonic range (mm)", "Ultrasonic status code"],
    "5/57": ["Last self-halt reason code", "Self-halt count"],
    "2/4": ["Left encoder ticks", "Right encoder ticks"],
    "4/11": ["Sample age (ms)", "IMU errors"],
    "4/12": ["Clipped samples", "Unsuitable samples"],
    "4/13": ["Terrain state", "Side state"],
    "4/14": ["Motion state", "Encoder yaw rate (0.01°/s)"],
    "4/15": ["Wheel-height difference (mm)", "Measurement quality"],
    "4/16": ["Stable peak height (mm)", "Median peak height (mm)"],
    "4/17": ["Peak side", "Peak end uptime (ms)"],
    "4/18": ["Impact count", "Last impact uptime (ms)"],
    "4/19": ["Highest stable peak (mm)", "Run ID"],
    "6/57": ["Line reacquisition count", "Reserved"],
};

function updateTelemetryDisplay() {
    byId("telemetry-age").textContent = !hasTelemetryObservation ? "N/A" : lastTelemetryChange === null ? "Cached; age unknown" : `Changed ${ageLabel(Date.now() - lastTelemetryChange)} ago`;
    const fields = latestEnvelope;
    const types = { 1: "Heartbeat", 2: "State", 3: "Motor speed", 4: "Encoder", 5: "Line sensor", 6: "Barcode", 7: "IMU", 8: "Hump", 9: "Ultrasonic", 10: "Odometry", 11: "IMU acquisition", 12: "IMU sample counters", 13: "Terrain and side", 14: "Estimated motion and yaw", 15: "Wheel height and quality", 16: "Saved peak heights", 17: "Saved ramp event", 18: "Possible impacts", 19: "Highest hump", 50: "Obstacle state", 51: "Obstacle size", 52: "Bypass plan", 53: "Depth bounds", 54: "Front clearance", 55: "Obstacle correlation", 56: "Obstacle validity", 57: "Line reacquisition", 58: "Raw ultrasonic range" };
    const rawSequence = previousPayloads.get(telemetryTopic)?.match(/(?:^|\s)seq=(\d+)(?=\s|$)/);
    const sequence = rawSequence && Number(rawSequence[1]) <= 4294967295 ? rawSequence[1] : "N/A";
    byId("sequence").textContent = fields ? String(fields.seq) : sequence;
    byId("source-type").textContent = fields ? `${fields.src} / ${fields.type}` : "N/A";
    for (const [key, field] of [["source", "src"], ["type", "type"], ["sequence", "seq"], ["value1", "value1"], ["value2", "value2"], ["flags", "flags"]]) {
        byId(`diag-${key}`).textContent = fields && Object.hasOwn(fields, field) ? String(fields[field]) : "N/A";
    }
    const labels = fields && telemetryValueLabels[`${fields.src}/${fields.type}`];
    for (const index of [1, 2]) {
        byId(`diag-value${index}`).previousElementSibling.textContent =
            index === 2 && fields?.src === 5 && fields.type === 56 && (fields.value1 & 16)
                ? "Reserved"
                : labels?.[index - 1] ?? `Value ${index} (raw)`;
    }
    byId("parse-note").textContent = !hasTelemetryObservation ? "No telemetry payload received." : !fields ? "Unrecognised or incomplete envelope. The raw payload is preserved above." : `${((labels || fields.type <= 10) && types[fields.type]) || "Unmapped telemetry type"}. Values are shown as transmitted. Missing flags leave validity unknown.`;
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
        if (Array.isArray(telemetry.value.sensor_frames)) {
            sensorObservations.clear();
            sourceObservations.clear();
            obstacleObservations.clear();
            const now = Date.now();
            for (const { payload, age_seconds } of telemetry.value.sensor_frames) {
                const fields = parseEnvelope(payload);
                if (!fields) continue;
                const observation = { fields, payload, at: now - age_seconds * 1000, received: true };
                sensorObservations.set(`${fields.src}/${fields.type}`, observation);
                sourceObservations.set(fields.src, observation);
                if (fields.src === 5 && fields.type <= 58) obstacleObservations.set(fields.type, observation);
            }
        }
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
byId("export-log").addEventListener("click", () => {
    const rows = [...byId("observation-log").querySelectorAll("li[data-topic]")].reverse();
    const lines = rows.map((row) =>
        `${row.querySelector("time").dateTime} ${row.dataset.topic} ${row.querySelector(".log-payload").textContent}`);
    const file = new Blob([lines.join("\r\n"), "\r\n"], { type: "text/plain;charset=utf-8" });
    const url = URL.createObjectURL(file);
    const link = document.createElement("a");
    link.href = url;
    link.download = "picocar-browser-payloads.txt";
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
});
byId("clear-log").addEventListener("click", () => {
    byId("observation-log").replaceChildren();
    const empty = document.createElement("li");
    empty.className = "empty-log";
    empty.textContent = "Log cleared. New changed payloads will appear here.";
    byId("observation-log").append(empty);
    logCount = 0;
    byId("export-log").disabled = true;
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
