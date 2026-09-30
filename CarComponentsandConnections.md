# Autonomous Robot Car - Hardware Connections

## Raspberry Pi Pico W Pin Map

This document contains the hardware connections used for the INF2004 Embedded Systems autonomous robotic car.

---

## 1. Left Motor (M1)

| Connection | Pico W GPIO |
|---|---|
| M1A | GP8 |
| M1B | GP9 |

---

## 2. Right Motor (M2)

| Connection | Pico W GPIO |
|---|---|
| M2A | GP10 |
| M2B | GP11 |

---

## 3. Servo Motor

| Connection | Pico W Connection |
|---|---|
| Signal (Orange) | GP12 |
| VCC (Red) | VCC |
| GND (Brown) | GND |

The servo is used to rotate the ultrasonic sensor for obstacle scanning.

---

## 4. Ultrasonic Sensor

| Connection | Pico W Connection |
|---|---|
| Trig | GP28 |
| Echo | GP7 |
| VCC | 3V3 |
| GND | GND |

The ultrasonic sensor is used for obstacle detection and distance measurement.

---

## 5. IR Sensors

Three IR sensors are used for line following and barcode detection.

| Sensor | Connection | Pico W GPIO |
|---|---|---|
| Left (Barcode) | DO | GP26 |
| Centre | DO | GP2 |
| Right | DO | GP0 |
| All IR Sensors | VCC | 3V3 |
| All IR Sensors | GND | GND |

---

## 6. IMU

The IMU communicates with the Pico W using I2C.

| Connection | Pico W Connection |
|---|---|
| SDA | GP16 |
| SCL | GP17 |
| VCC | 3V3 |
| GND | GND |

The IMU is used for motion and terrain monitoring, including:

- Tilt detection
- Hump detection
- Peak height measurement
- Motion classification
- Collision detection
- Turn-rate measurement

---

## Complete GPIO Reference

```text
GP0  -> Right IR Sensor DO
GP2  -> Centre IR Sensor DO

GP7  -> Ultrasonic Echo

GP8  -> Left Motor M1A
GP9  -> Left Motor M1B

GP10 -> Right Motor M2A
GP11 -> Right Motor M2B

GP12 -> Servo Signal

GP16 -> IMU SDA
GP17 -> IMU SCL

GP26 -> Left IR Sensor DO (Barcode)
GP28 -> Ultrasonic Trig
```

---

## Connection Overview

```text
Raspberry Pi Pico W
│
├── Motion Control
│   ├── GP8  -> Left Motor M1A
│   ├── GP9  -> Left Motor M1B
│   ├── GP10 -> Right Motor M2A
│   └── GP11 -> Right Motor M2B
│
├── Line Following / Barcode
│   ├── GP26 -> Left IR Sensor
│   ├── GP2  -> Centre IR Sensor
│   └── GP0  -> Right IR Sensor
│
├── Obstacle Detection
│   ├── GP12 -> Servo Signal
│   ├── GP28 -> Ultrasonic Trig
│   └── GP7  -> Ultrasonic Echo
│
└── IMU / Terrain Monitoring
    ├── GP16 -> SDA
    └── GP17 -> SCL
```

---

## Buddy Subsystems

| Buddy | Subsystem | Main Hardware |
|---|---|---|
| Buddy 1 | WiFi Communication & Telemetry | Pico W WiFi |
| Buddy 2 | Motion Control | Motors |
| Buddy 3 | Barcode & Line Following | 3x IR Sensors |
| Buddy 4 | IMU & Terrain Monitoring | IMU |
| Buddy 5 | Obstacle Detection | Ultrasonic + Servo |

The final system integrates all five subsystems into a single autonomous robotic car.