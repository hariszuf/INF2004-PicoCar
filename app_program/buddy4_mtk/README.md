# Buddy 4 micro T Kernel trial

Status: separate trial, not merged into the live team application. Uses the professor-based kernel already in this repository. No changes are pushed to the professor's repository.

## Why this folder exists

The working Buddy 4 CMake project, driving-test firmware and shared team startup should remain stable while the native task integration is reviewed and hardware-tested. The previously applied native integration was relocated here; the live files were restored to their verified pre-integration versions.

`overlay/` contains the 17 proposed source/build/documentation files, mirrored by their future repository paths. These files are inactive: they are applied only to a generated source copy under this folder's ignored `build/` directory. They are ordinary application and board-driver changes against the professor's port, not a replacement RTOS or custom standalone kernel.

`prepare_source.py` checks the fingerprints in `base_hashes.json`, copies the current team source and applies the overlay inside `build/source/`. If a teammate changes a relevant base file, it stops and requests overlay review instead of silently overwriting team work. Fingerprints use UTF-8 text with normalised line endings and no BOM, so Windows/Linux checkout conventions alone do not cause a mismatch. It recreates only the disposable source copy; do not edit code in `build/source/`.

Edit proposed integration files inside `overlay/`, then run tests/build. Do not update fingerprints blindly after a base change: compare and rebase affected files first.

## Commands from repository root

Requires the existing Python, GNU Make, native host GCC/g++ and Arm toolchain installation. Paths match the documented Windows layout; adjust on another computer.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/build.ps1
```

Output:

```text
app_program/buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2
```

Build/test scripts never flash or commit. Each preparation discards only `build/source/`, so do not run them concurrently. Build outputs are ignored; only overlay, scripts, manifest and documentation should be committed.

## Proposed integration

- Register a Buddy 4 IMU task alongside existing Buddy 1/5 tasks.
- Use the professor port's kernel mutex/time/device APIs and registered `iica` I2C driver.
- Keep the established calibration, processing algorithms, raw data and geometry.
- Publish protected snapshots; leave encoder motion/yaw invalid until a native Buddy 2 producer supplies fresh observations.
- Include board-specific GP16/17 mapping and native I2C STOP/restart completion fixes with host regressions.
- Keep the ordinary CMake standalone and driving tests available.

See [detailed proposed native integration notes](overlay/app_program/buddy4/README_MTK.md). Paths in those notes refer to the generated trial source tree and later merge layout, not active changes in the live application.

## Existing verification and limitations

Before relocation, the identical overlay source passed native task/device tests, register/IRQ driver regressions, existing host/CSV suites, native USB team firmware build and both CMake firmware builds in a staging copy. After relocation, source preparation produced an identical 17-file overlay in the isolated tree and the native task/driver host tests passed there. The relocated build.ps1 wrapper was not rerun; the included ignored image is the previously built, identical staged source. Hardware operation remains unverified.

The optional Buddy-1-only compatibility build was not run. Four bare-metal newlib file-I/O stub linker warnings remain, along with the existing two Buddy 3 warnings in its regression/CMake builds. Physical I2C, scheduling, stack use and coexistence with ultrasonic timing remain unverified.

This trial does not migrate Buddy 2/3 controls, validate collisions/yaw accuracy, resolve course-board height bias, implement Tribunal 1 or finish final course integration. The RTOS application does not implement the driving-test M/L/R/G/S commands. Keep motor power off for initial qualification.

Merge into live team files only after review and the required tests. Until then, use the original working firmware for the existing driving tests.
