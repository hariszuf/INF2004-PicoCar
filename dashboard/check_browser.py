"""Browser regression checks with simulated APIs; no broker or car commands."""
import argparse
import tempfile
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from playwright.sync_api import sync_playwright, expect


class StaticHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.split("?")[0] == "/":
            self.path = "/static/index.html"
        super().do_GET()

    def log_message(self, *args):
        pass


def check(browser, url, output):
    status = {"mqtt_connected": True, "vehicle_heartbeat_fresh": True, "heartbeat_age_seconds": 1.0}
    telemetry = {
        "picocar/vehicle1/status": "heartbeat seq=1",
        "picocar/vehicle1/telemetry": "src=1 type=1 seq=7 value1=1 value2=0",
    }
    failure = {"status": False, "telemetry": False, "command": False}
    commands = []
    delayed_start = []
    delay_start = False
    delayed_telemetry = []
    delay_telemetry = False
    page = browser.new_page(viewport={"width": 1440, "height": 1000})
    page.clock.install(time="2026-11-02T00:00:00Z")
    page.clock.pause_at("2026-11-02T00:00:00Z")
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))

    def api(route):
        path = route.request.url.split("/api/")[1]
        if path.startswith("command/"):
            commands.append((route.request.method, path))
            if path == "command/START" and delay_start:
                delayed_start.append(route)
                return
            route.fulfill(status=503 if failure["command"] else 200, json={"detail": "MQTT publish failed"} if failure["command"] else {"command": path.split("/")[1], "status": "publish queued"})
        else:
            if path == "telemetry" and delay_telemetry:
                delayed_telemetry.append(route)
                return
            failed = failure[path]
            route.fulfill(status=503 if failed else 200, json={"detail": "Test API unavailable"} if failed else status if path == "status" else telemetry)

    page.route("**/api/**", api)
    page.goto(url)
    expect(page.locator("#mqtt-connection")).to_have_text("Connected")
    expect(page.locator("#vehicle-connection")).to_have_text("Fresh")
    expect(page.locator("#telemetry-age")).to_have_text("Cached; age unknown")
    expect(page.locator("#sequence")).to_have_text("7")
    expect(page.locator(".connection-states")).to_contain_text("Unconfirmed")

    for command in ("START", "STOP"):
        page.locator(f"#{command.lower()}-command").click()
        expect(page.locator("#command-result")).to_have_text(f"{command}: publication queued. Execution unconfirmed.")
    assert commands == [("POST", "command/START"), ("POST", "command/STOP")]
    delay_start = True
    page.locator("#start-command").click()
    expect(page.locator("#start-command")).to_be_disabled()
    expect(page.locator("#stop-command")).to_be_enabled()
    page.locator("#stop-command").click()
    expect(page.locator("#command-result")).to_contain_text("STOP: publication queued")
    assert len(delayed_start) == 1
    delayed_start[0].fulfill(json={"command": "START", "status": "publish queued"})
    expect(page.locator("#start-command")).to_be_enabled()
    expect(page.locator("#command-result")).to_contain_text("STOP: publication queued")
    delay_start = False
    failure["command"] = True
    page.locator("#stop-command").click()
    expect(page.locator("#command-result")).to_contain_text("MQTT publish failed")
    failure["command"] = False

    def refresh():
        page.locator("#refresh-button").click()
        expect(page.locator("#refresh-button")).to_be_enabled()

    status["vehicle_heartbeat_fresh"] = False
    status["heartbeat_age_seconds"] = 15
    refresh()
    expect(page.locator("#vehicle-connection")).to_have_text("Stale")
    expect(page.locator("#mqtt-connection")).to_have_text("Connected")
    status["mqtt_connected"] = False
    refresh()
    expect(page.locator("#mqtt-connection")).to_have_text("Disconnected")
    expect(page.locator("#start-command")).to_be_disabled()
    expect(page.locator("#stop-command")).to_be_disabled()

    failure["status"] = True
    refresh()
    expect(page.locator("#mqtt-connection")).to_have_text("Unknown")
    expect(page.locator("#api")).to_have_text("Partial failure")
    failure["telemetry"] = True
    refresh()
    expect(page.locator("#api")).to_have_text("Unavailable")
    expect(page.locator("#api-notice")).to_be_visible()
    failure.update(status=False, telemetry=False)
    status.update(mqtt_connected=True, vehicle_heartbeat_fresh=True, heartbeat_age_seconds=1)
    status["heartbeat_age_seconds"] = 14.8
    delay_telemetry = True
    with page.expect_response("**/api/status"):
        page.locator("#refresh-button").click()
    page.clock.fast_forward(600)
    assert len(delayed_telemetry) == 1
    delay_telemetry = False
    delayed_telemetry[0].fulfill(json=telemetry)
    expect(page.locator("#refresh-button")).to_be_enabled()
    assert page.locator("#vehicle-connection").inner_text() == "Stale"
    status["heartbeat_age_seconds"] = 1
    telemetry["picocar/vehicle1/telemetry"] = "seq=42"
    refresh()
    expect(page.locator("#sequence")).to_have_text("42")
    telemetry["picocar/vehicle1/telemetry"] = "src=5 type=54 seq=8 value1=0 value2=-30"
    refresh()
    page.goto(url + "#obstacle")
    expect(page.locator("#front-clearance")).to_contain_text("0 mm")
    expect(page.locator("#range-validity")).to_contain_text("Unknown")
    expect(page.locator("#ultrasonic-range")).to_have_text("N/A")
    telemetry["picocar/vehicle1/telemetry"] = "src=5 type=54 seq=9 value1=-1 value2=999"
    refresh()
    expect(page.locator("#front-clearance")).to_contain_text("Unknown")
    expect(page.locator("#servo-angle")).to_contain_text("N/A")
    telemetry["picocar/vehicle1/telemetry"] = "src=5 type=50 seq=10 value1=6 value2=1"
    refresh()
    expect(page.locator("#obstacle-fault")).to_contain_text("Sensor fault")
    assert page.locator("#obstacle-fault").get_attribute("data-tone") == "fault"

    def buddy5(kind, sequence, first, second):
        telemetry["picocar/vehicle1/telemetry"] = f"src=5 type={kind} seq={sequence} value1={first} value2={second}"
        refresh()

    buddy5(56, 11, 3, 0)
    expect(page.locator("#obstacle-width")).to_contain_text("Unknown")
    buddy5(51, 11, 170, 600)
    expect(page.locator("#obstacle-width")).to_contain_text("170 mm")
    expect(page.locator("#obstacle-width-validity")).to_contain_text("Valid (reported)")
    expect(page.locator("#depth-low")).to_contain_text("Unknown")
    buddy5(53, 11, 200, 600)
    expect(page.locator("#depth-low")).to_contain_text("200 mm")
    expect(page.locator("#depth-high")).to_contain_text("600 mm")
    buddy5(52, 11, -1, -190)
    expect(page.locator("#bypass-side")).to_contain_text("Right")
    expect(page.locator("#bypass-offset")).to_contain_text("-190 mm")
    expect(page.locator("#line-reacquired")).to_have_text("N/A")
    expect(page.locator("#range-validity")).to_contain_text("Unknown")

    # A later validity packet cannot validate older size/depth observations.
    buddy5(56, 12, 0, 0)
    for field in ["obstacle-width", "depth-low", "depth-high", "bypass-offset"]:
        expect(page.locator(f"#{field}")).to_contain_text("Unknown")
    buddy5(51, 12, -1, -1)
    expect(page.locator("#obstacle-width-validity")).to_contain_text("Invalid (reported)")
    expect(page.locator("#obstacle-width")).not_to_contain_text("-1 mm")
    buddy5(53, 12, -1, -1)
    expect(page.locator("#depth-validity")).to_contain_text("Invalid (reported)")
    buddy5(56, 13, 3, 0)
    buddy5(51, 13, -1, 600)
    expect(page.locator("#obstacle-width-validity")).to_contain_text("Inconsistent")
    buddy5(53, 13, 700, 200)
    expect(page.locator("#depth-low")).to_contain_text("Unknown")
    expect(page.locator("#depth-validity")).to_contain_text("Inconsistent")
    buddy5(56, 14, 8, 0)
    expect(page.locator("#obstacle-width-validity")).to_contain_text("Unknown")
    buddy5(50, 1, 0, 0)
    expect(page.locator("#obstacle-profile-sequence")).to_contain_text("N/A")
    expect(page.locator("#front-clearance")).to_contain_text("Unknown")
    expect(page.locator("#obstacle-state")).to_contain_text("Monitoring")
    expect(page.locator("#line-reacquired")).to_have_text("N/A")

    page.goto(url + "#motion")
    telemetry["picocar/vehicle1/telemetry"] = "src=2 type=4 seq=15 value1=100 value2=200"
    refresh()
    expect(page.locator("#distance-travelled")).to_have_text("N/A")
    expect(page.locator("#view-motion")).to_contain_text("signed odometry since reset")

    telemetry["picocar/vehicle1/telemetry"] = '<img src=x onerror="window.injected=true">'
    refresh()
    page.goto(url + "#diagnostics")
    expect(page.locator("#telemetry")).to_contain_text("<img")
    assert page.locator("#telemetry img").count() == 0
    assert page.evaluate("window.injected") is None
    expect(page.locator("#parse-note")).to_contain_text("Unrecognised")
    telemetry["picocar/vehicle1/telemetry"] = "src=1 type=1 seq=11 value1=1 value2=0"
    refresh()
    expect(page.locator("#diag-flags")).to_have_text("N/A")
    page.locator("#clear-log").click()
    expect(page.locator("#observation-log")).to_contain_text("Log cleared")
    page.locator("#stop-command").click()
    expect(page.locator("#command-result")).to_contain_text("STOP: publication queued")

    # A source/type declaration is not a Buddy 4 measurement schema.
    imu_fields = [
        "imu-status", "pitch", "roll", "wheel-height", "terrain", "side",
        "estimated-motion", "turn-rate", "imu-quality", "imu-freshness",
        "imu-sample-age", "imu-io-errors", "imu-clipped-samples",
        "imu-unsuitable-samples", "peak-height", "median-peak-height",
        "peak-side", "peak-time", "impact-result", "impact-time", "impact-count",
    ]
    page.goto(url + "#imu")
    for kind in (7, 8):
        telemetry["picocar/vehicle1/telemetry"] = f"src=4 type={kind} seq={20 + kind} value1=1234 value2=5678"
        refresh()
        for field in imu_fields:
            expect(page.locator(f"#{field}")).to_have_text("N/A")
    expect(page.locator("#view-imu")).to_contain_text("Preliminary estimates")
    expect(page.locator("#view-imu")).to_contain_text("Observational, not a confirmed collision")
    status.update(vehicle_heartbeat_fresh=False, heartbeat_age_seconds=20)
    refresh()
    expect(page.locator("#vehicle-connection")).to_have_text("Stale")
    for field in imu_fields:
        expect(page.locator(f"#{field}")).to_have_text("N/A")
    status.update(vehicle_heartbeat_fresh=True, heartbeat_age_seconds=1)
    refresh()

    # Synthetic input stays in this check; screenshots are browser-test evidence.
    views = ["overview", "motion", "line", "imu", "obstacle", "safety", "diagnostics"]
    for width, height in [(1440, 1000), (768, 1024), (390, 844), (320, 740)]:
        page.set_viewport_size({"width": width, "height": height})
        if width == 1440:
            expect(page.locator("#menu-toggle")).not_to_be_visible()
        for view in views:
            page.goto(url + "#" + view)
            expect(page.locator(f"#view-{view}")).to_be_visible()
            assert page.locator(".view:visible").count() == 1
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth"), (width, view, "overflow")
            assert page.locator("#stop-command").bounding_box()["y"] < height, (width, view, "STOP below fold")
            if view == "imu":
                for field in imu_fields:
                    expect(page.locator(f"#{field}")).to_have_text("N/A")
            if view == "motion":
                expect(page.locator("#distance-travelled")).to_have_text("N/A")
            if width >= 390:
                page.screenshot(path=str(output / f"{view}-{width}.png"), full_page=True)
        if width <= 768:
            page.locator("#menu-toggle").click()
            expect(page.locator("#sidebar")).to_be_visible()
            page.keyboard.press("Escape")
            expect(page.locator("#menu-toggle")).to_be_focused()
            page.locator("#menu-toggle").click()
            page.locator("nav a[href='#imu']").click()
            expect(page.locator("#view-imu")).to_be_visible()
            expect(page.locator("#sidebar")).not_to_be_visible()
    page.goto(url + "#safety")
    page.locator("summary").filter(has_text="Command transport").focus()
    page.keyboard.press("Enter")
    assert page.locator("#view-safety details").get_attribute("open") is not None
    page.goto(url)
    page.keyboard.press("Tab")
    expect(page.locator(".skip-link")).to_be_focused()
    page.keyboard.press("Enter")
    expect(page.locator("#main")).to_be_focused()
    page.goto(url + "#imu")
    page.locator(".skip-link").focus()
    page.keyboard.press("Enter")
    expect(page.locator("#view-imu")).to_be_visible()
    expect(page.locator("#main")).to_be_focused()
    assert not errors, errors
    page.close()
    print("PASS: seven views at four widths, keyboard navigation, API failures, heartbeat, safe payload rendering, command publishing and N/A states")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chromium", help="Path to a Chromium executable; otherwise use Playwright's installed Chromium")
    parser.add_argument("--output", type=Path, default=Path(tempfile.gettempdir()) / "picocar-dashboard-review")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    handler = partial(StaticHandler, directory=str(Path(__file__).parent))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(executable_path=args.chromium, headless=True, args=["--no-sandbox"])
            try:
                check(browser, f"http://127.0.0.1:{server.server_port}/", args.output)
                check_preview(browser, f"http://127.0.0.1:{server.server_port}/")
            finally:
                browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def check_preview(browser, url):
    page = browser.new_page()
    page.clock.install(time="2026-11-02T00:00:00Z")
    page.clock.pause_at("2026-11-02T00:00:00Z")
    versions = {"dashboard.css": "1", "index.html": "1", "dashboard.js": "1"}
    commands = []

    def static(route):
        if route.request.method == "HEAD":
            name = route.request.url.rsplit("/", 1)[1]
            route.fulfill(headers={"etag": versions[name]})
        else:
            route.continue_()

    def api(route):
        if "/command/" in route.request.url:
            commands.append(route.request.url)
        route.fulfill(json={} if route.request.url.endswith("telemetry") else {
            "mqtt_connected": False, "vehicle_heartbeat_fresh": False, "heartbeat_age_seconds": None,
        })

    def last_head(response):
        return response.request.method == "HEAD" and response.url.endswith("dashboard.js")

    page.route("**/static/**", static)
    page.route("**/api/**", api)
    with page.expect_response(last_head):
        page.goto(url + "?preview=1#imu")
    page.wait_for_timeout(100)
    page.evaluate("window.previewMarker = true")
    versions["dashboard.css"] = "2"
    with page.expect_response(last_head):
        with page.expect_request(lambda request: "dashboard.css?v=" in request.url):
            page.clock.fast_forward(1000)
    page.wait_for_timeout(100)
    expect(page.locator('link[rel="stylesheet"]')).to_have_count(1)
    assert page.evaluate("window.previewMarker") is True
    expect(page.locator("#view-imu")).to_be_visible()
    versions["index.html"] = "2"
    with page.expect_navigation(wait_until="domcontentloaded"):
        page.clock.fast_forward(1000)
    assert page.evaluate("window.previewMarker") is None
    expect(page.locator("#view-imu")).to_be_visible()
    assert not commands
    page.close()
    print("PASS: preview CSS swaps preserve page state; HTML changes reload without publishing commands")


if __name__ == "__main__":
    main()
