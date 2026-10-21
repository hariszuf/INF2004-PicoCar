# Buddy 4 pre-refactor archive

Preserved superseded files; excluded from all active builds. Original source and existing binary are historical references, not fresh firmware. No files were permanently deleted.

| Files | Reason |
| --- | --- |
| UpDownHillLeftRightUpWheel/IMUProject.c, CMakeLists.txt, IMUProject.uf2 | Tested reference preserved unchanged; reusable logic now lives in the active module. |
| lsm303d.c, lsm303d.h | Older driver with different configuration and no fresh-status API. |
| terrain.c, terrain.h, tests/test_terrain.c | Earlier pitch-only thresholds and transient experiment; replaced by reference-based pitch/roll and event tests. |
| main.c, CMakeLists.txt, README.md | Superseded automatic pitch calibration, standalone wiring and MQTT build/documentation. |
| mqtt_telemetry.c, mqtt_telemetry.h, lwipopts.h, local_config.example.h | Older optional Wi-Fi/MQTT experiment; outside the current IMU scope. |

SHA-256 hashes of preserved files:

- `CMakeLists.txt`: `97a12a26abd4347a997925e0aa8793630ddd16914aaaf137c07ea11ff5213013`
- `local_config.example.h`: `8b6d998eb1048329b324b382194d19aeb1108469ad3a75fa1c7b551c31350879`
- `lsm303d.c`: `727a8decd0692159e41ff57d8d3f7ecb8c49f81c746e65efec5c6c392c888199`
- `lsm303d.h`: `d4522e17267b1c5dd32c7baa72b2ddfed7f401c9934676209d9bed5ae098ab3e`
- `lwipopts.h`: `555c45bd72277be1d9eb726a7feb73637a7bfce077f7a4c70fe5774353e7b968`
- `main.c`: `ae9d962af5fd62e4d1b7ff6186972640a76f9e0ca511e1e76b133a679404c4c8`
- `mqtt_telemetry.c`: `39dbc2bbe618df0ece21a6435f5fc90bd1bde35d0b617324969ac3adeb06c636`
- `mqtt_telemetry.h`: `3554e166bc86fbaaf7e1ff4ade935ab83934b17c822ab72ae13b6064c1920e01`
- `README.md`: `f611b1edeb2b3795159ee69607d30264b31c871c6fdea92bd4ffe05719a0a0d2`
- `terrain.c`: `a49d45c095bc638c001a3f6c662a0d93abc5d7969872f3c37e9f58a5a8170b87`
- `terrain.h`: `33eac3d5373df8db878a502841e2393c56281704ab3d391df64ac356033fb0d3`
- `tests/test_terrain.c`: `da503520cda0ba4edd0db235bdf58fd974b18876af2f5eb2c1a8380ef50430c3`
- `UpDownHillLeftRightUpWheel/CMakeLists.txt`: `43b7ac62474dec07077e6646df09b8563ba8f9c0ea3aa7fc31e07dc56030f8b1`
- `UpDownHillLeftRightUpWheel/IMUProject.c`: `7f5b693dd31685558293e59e3e1c68a7a8c92a9f464e744dac0069b1d837cd8a`
- `UpDownHillLeftRightUpWheel/IMUProject.uf2`: `cc8d9af86fbca23cf402fa603536138ec319f0173413349ecd9696091f7dfc3a`
