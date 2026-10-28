# Buddy 1 startup software checks

Run from the repository root with Python 3, a host GCC, and a complete Pico SDK:

```powershell
python tools/buddy1_startup_tests/run_network_tests.py --sdk C:/Users/muhdh/.pico-sdk/sdk/2.3.1
python tools/buddy1_startup_tests/run_usb_tests.py
python tools/test_wifi_startup.py
python tools/buddy1_startup_tests/test_led_diagnostic.py
```

Both new runners accept `--repo <source-tree>` and `--cc <compiler>`.
Network checks compile the actual `mqtt_utk.c` and the SDK's lwIP DHCP/TCP/MQTT
implementations against an in-memory Ethernet peer. No sockets, USB, radio,
broker, dashboard, or board are accessed. Simulated time advances in 10 ms
steps; selected cases inject allocation/connect-call errors. The host needs
8-byte heap alignment, so its adapter overrides the firmware's 4-byte setting.
Buddy 1 command and telemetry source is compiled unchanged against simulated
message buffers. The buddy_io case checks telemetry forwarding and payload,
command subscription, fragmented START and STOP delivery to the consumer,
and rejection of wrong-topic/invalid commands. No motor code is linked. Assertions stay enabled.

USB checks compile the actual `usb_console.c` for the SMP=0 profile and replace
the kernel/TinyUSB boundary. They check full-buffer drops, sleep requests with
no host, an enumerated host accepting no data, a finite event burst, and failed
USB initialization. They do not model hardware interrupts or prove scheduler
fairness. The existing join test exercises the production control block with
a fake clock/radio in MQTT and qualification configurations.

Generated executables, headers, compilation logs and results go in ignored
`build/`, `build-usb/` and `build-led/`. The LED check exercises the production
helper, including its guard against uninitialized radio I/O and all pulse states. Passing these checks establishes software behavior
under the supplied conditions; it does not establish a physical DHCP lease,
radio-driver progress, power stability, or reliable standalone operation.
