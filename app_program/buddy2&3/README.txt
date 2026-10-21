IRMotorTest — standalone Pico W test (not muT-Kernel)

FILES
- main.c                   IR-controlled motor test using Buddy 2's encoder PID code
- CMakeLists.txt            One executable: IRMotorTest
- pico_sdk_import.cmake    Finds your installed Pico SDK

HARDWARE
Main IR GP2, backup IR GP0, barcode IR GP26 (1=BLACK, 0=WHITE).
Left motor GP8/GP9; right motor GP10/GP11.
Left encoder GP6; right encoder GP4.
Ensure GP26 is NOT also connected to a motor signal.
Power sensor outputs at safe Pico 3.3 V logic levels.

BEHAVIOUR
Either GP2 or GP0 sees BLACK: both motors commanded FORWARD using PID (800 encoder counts/s).
Both see WHITE: motor drive is switched off (coast, not active braking).
GP26 is displayed but does not affect movement. No barcode decoding.
No turning or actual line tracking yet.

BUILD
1. Extract to a new folder, and open THAT folder in VS Code.
2. Configure project with Raspberry Pi Pico VS Code extension (board Pico W).
3. Compile Project.
4. Find build/.../IRMotorTest.uf2 (location depends on build configuration).
5. Flash with BOOTSEL and open Serial Monitor at 115200.

SAFETY
First test with car supported and wheels off the floor.
Keep an accessible motor power switch and a clear testing area.
The program is an early experiment, not a complete robot safety system.
Do not compile alongside another file with main().

NOTE
If the SDK isn't found, copy the pico_sdk_import.cmake from your previously working
Pico project, or configure PICO_SDK_PATH to your installed SDK.
