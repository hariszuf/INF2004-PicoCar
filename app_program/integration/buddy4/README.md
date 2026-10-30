# Buddy 4 interface

Buddy 4's IMU code and μT-Kernel prototype are in `app_program/buddy4/` and
`app_program/buddy4_mtk/`. They are not linked into this driving application
yet. The existing prototype has its own startup and I2C ownership, which
must be reconciled with the shared vehicle controller before inclusion.

