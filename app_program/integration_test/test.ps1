$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force build-host | Out-Null
    $common = @('-std=c11', '-Wall', '-Wextra', '-DMOTION_HOST_TEST', '-I../buddy2', '-I../buddy2/test/host')
    $motion = @('../buddy2/motion_control.c', '../buddy2/motion_pid.c', '../buddy2/test/host/sim_hw.c')
    & gcc @common @motion ../buddy2/test/host/test_motion.c -o build-host/motion.exe -lm
    if ($LASTEXITCODE) { throw 'Motion compilation failed' }
    & ./build-host/motion.exe
    if ($LASTEXITCODE) { throw 'Motion tests failed' }
    & gcc @common -I../buddy3 -I../buddy3/test/host '-DMOTION_MIN_DUTY=0.10f' '-DMOTION_BREAKAWAY_DUTY=0.22f' '-DMOTION_STALL_DUTY=0.6f' '-DMOTION_STALL_TIME_MS=1500u' '-DMOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f' @motion ../buddy3/line_follow.c ../buddy3/test/host/sim_track.c ../buddy3/test/host/test_line.c -o build-host/line.exe -lm
    if ($LASTEXITCODE) { throw 'Line compilation failed' }
    & ./build-host/line.exe
    if ($LASTEXITCODE) { throw 'Line tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror test_scheduler.c -o build-host/scheduler.exe
    if ($LASTEXITCODE) { throw 'Scheduler compilation failed' }
    & ./build-host/scheduler.exe
    if ($LASTEXITCODE) { throw 'Scheduler tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -I../buddy2 -I../buddy3 drive_control.c test_drive_control.c -o build-host/drive.exe
    if ($LASTEXITCODE) { throw 'Drive compilation failed' }
    & ./build-host/drive.exe
    if ($LASTEXITCODE) { throw 'Drive safety tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -UNDEBUG -I../buddy4 ../buddy4/buddy4_imu.c ../buddy4/tests/test_imu.c -o build-host/imu.exe -lm
    if ($LASTEXITCODE) { throw 'IMU compilation failed' }
    & ./build-host/imu.exe
    if ($LASTEXITCODE) { throw 'IMU tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -UNDEBUG -I../buddy4 test_median.c -o build-host/median.exe -lm
    if ($LASTEXITCODE) { throw 'Median compilation failed' }
    & ./build-host/median.exe
    if ($LASTEXITCODE) { throw 'Median regressions failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -UNDEBUG -I../buddy4 ../buddy4/buddy4_motion.c test_motion_class.c -o build-host/motion-class.exe -lm
    if ($LASTEXITCODE) { throw 'Motion classifier compilation failed' }
    & ./build-host/motion-class.exe
    if ($LASTEXITCODE) { throw 'Motion classifier tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -UNDEBUG -I../buddy4 ../buddy4/buddy4_impact.c test_impact.c -o build-host/impact.exe -lm
    if ($LASTEXITCODE) { throw 'Possible-impact compilation failed' }
    & ./build-host/impact.exe
    if ($LASTEXITCODE) { throw 'Possible-impact tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -UNDEBUG -I../buddy4 ../buddy4/buddy4_encoder_yaw.c test_encoder_yaw.c -o build-host/encoder-yaw.exe -lm
    if ($LASTEXITCODE) { throw 'Encoder yaw estimate compilation failed' }
    & ./build-host/encoder-yaw.exe
    if ($LASTEXITCODE) { throw 'Encoder yaw estimate tests failed' }
    & python test_csv_schema.py
    if ($LASTEXITCODE) { throw 'CSV schema checks failed' }
} finally { Pop-Location }


