// Focused contract and rendering checks with synthetic MQTT payloads.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

const script = fs.readFileSync(__dirname + "/static/dashboard.js", "utf8");
const keys = script.slice(script.indexOf("const telemetryValueKeys ="), script.indexOf("function addLog("));
const rendering = script.slice(script.indexOf("function updateMeasurements()"), script.indexOf("const telemetryValueLabels ="));
const elements = new Map();
const sensorObservations = new Map();
const context = {
    Date, Math, Map, Object, Number,
    document: { createElement: () => ({ textContent: "" }) },
    byId: id => {
        if (!elements.has(id)) elements.set(id, { textContent: "N/A", dataset: {}, append(note) { this.note = note.textContent; } });
        return elements.get(id);
    },
    subsystems: [], sourceObservations: new Map(), obstacleObservations: new Map(), sensorObservations,
    statusAvailable: true, telemetryAvailable: true, brokerConnected: true,
    heartbeatFresh: true, heartbeatAge: 0, lastPollAt: Date.now(),
    observationLabel: () => "Received", ageLabel: () => "0 s",
};
vm.createContext(context);
vm.runInContext(keys + rendering + "this.parseEnvelope = parseEnvelope; this.updateMeasurements = updateMeasurements;", context);

function packet(type, first, second, flags = 0, age = 0) {
    const names = {
        7: ["pitch_cdeg", "roll_cdeg"], 11: ["sample_age_ms", "imu_errors"],
        12: ["clipped_samples", "unsuitable_samples"], 13: ["terrain_state", "side_state"],
        14: ["motion_state", "yaw_rate_cdeg_s"], 15: ["wheel_height_mm", "measurement_quality"],
        16: ["stable_peak_mm", "median_peak_mm"], 17: ["peak_side", "peak_end_ms"],
        18: ["impact_count", "last_impact_ms"], 19: ["highest_hump_mm", "run_id"],
    };
    const [a, b] = names[type];
    const fields = context.parseEnvelope(`src=4 type=${type} seq=2 ${a}=${first} ${b}=${second} flags=${flags}`);
    assert(fields, `type ${type} parses`);
    sensorObservations.set(`4/${type}`, { fields, at: Date.now() - age, received: true });
}
function value(id) { return elements.get(id)?.textContent; }

packet(7, -123, 0, 1);
packet(11, 20, 2, 1);
packet(12, 3, 4);
packet(13, -1, 1, 1);
packet(14, 4, -250, 3);
packet(15, 14, 1, 1);
packet(16, 14, 9, 3);
packet(17, -1, 12345, 1);
packet(18, 2, 13000, 0);
packet(19, 14, 1, 1);
context.updateMeasurements();
assert.equal(value("imu-status"), "Tilt valid");
assert.equal(value("pitch"), "-1.23°");
assert.equal(value("roll"), "0.00°");
assert.equal(value("imu-freshness"), "Fresh at report");
assert.match(value("imu-sample-age"), /^\d+ ms$/);
assert(Number.parseInt(value("imu-sample-age"), 10) >= 20);
assert.equal(value("imu-io-errors"), 2);
assert.equal(value("imu-clipped-samples"), 3);
assert.equal(value("imu-unsuitable-samples"), 4);
assert.equal(value("terrain"), "Downhill");
assert.equal(value("side"), "Right wheel up");
assert.equal(value("estimated-motion"), "Turning left");
assert.equal(value("turn-rate"), "-2.50°/s");
assert.equal(value("wheel-height"), "1.4");
assert.equal(value("imu-quality"), "Gravity-like");
assert.equal(value("peak-height"), "1.4");
assert.equal(value("median-peak-height"), "0.9");
assert.equal(value("peak-side"), "Left wheel up");
assert.equal(value("peak-time"), "12345 ms");
assert.equal(value("impact-count"), 2);
assert.equal(value("impact-result"), "Possible impact recorded");
assert.equal(value("impact-time"), "13000 ms");
assert.equal(value("highest-hump"), "1.4 cm");

packet(7, 0, 0, 0);
packet(15, 0, 2, 0);
packet(16, 0, 0, 0);
packet(19, 0, 2, 0);
context.updateMeasurements();
for (const id of ["pitch", "roll", "wheel-height", "peak-height", "median-peak-height", "highest-hump"])
    assert.equal(value(id), "N/A", `${id} invalid zero`);
packet(7, 0, 0, 1);
packet(19, 0, 2, 1);
context.updateMeasurements();
assert.equal(value("pitch"), "0.00°");
assert.equal(value("highest-hump"), "0.0 cm");

packet(7, 500, 600, 1, 5000);
packet(11, 20, 2, 1, 5000);
packet(19, 14, 2, 1, 5000);
context.updateMeasurements();
assert.equal(value("imu-status"), "Stale");
assert.equal(value("pitch"), "N/A");
assert.equal(value("imu-freshness"), "Stale");
assert.equal(value("highest-hump"), "1.4 cm");
assert.match(elements.get("highest-hump").note, /Buddy 4 stale/);
context.brokerConnected = false;
context.updateMeasurements();
assert.equal(value("pitch"), "N/A");
assert.equal(value("highest-hump"), "1.4 cm");
assert.match(elements.get("highest-hump").note, /Disconnected/);
console.log("Buddy 4 synthetic MQTT parser/render checks passed");
