# Lab Project — Wall-Following Robot (AVR)

This repository contains firmware for a simple wall-following robot implemented on an AVR (ATmega328P). The code provides ultrasonic distance sensing, servo-based scanning, and motor control with a small state machine for wall-following and obstacle avoidance.

## Key features
- Timer-based timekeeping (millis/micros) using Timer2
- PWM motor control (Timer0) and servo control (Timer1)
- Ultrasonic distance measuring (HC-SR04 style) using pin-change interrupts for echo timing
- Smoothed wall-following algorithm with short reconfirmation for lost-wall events
- UART debug output at 9600 baud

## Files of interest
- `main.c` — high-level state machine (check front, check right, follow)
- `motors.c`, `motors.h` — motor setup and PWM control
- `servo.c`, `servo.h` — servo PWM and helper
- `ultrasonic.c`, `ultrasonic.h` — ultrasonic trigger/echo logic
- `time.c`, `time.h` — centralized millis()/micros() using Timer2
- `uart.c`, `uart.h` — simple UART helpers
- `Makefile` — build and flash targets

## Wiring / Pin mapping (from source)
- Left control (direction): PD2
- Left PWM: PD5 (OC0B)
- Right control (direction): PD4
- Right PWM: PD6 (OC0A)
- Servo PWM: PB2 (OC1B)
- Ultrasonic Trigger: PB4
- Ultrasonic Echo (PCINT): PB5

> Note: Verify your hardware wiring matches these pins. Motor direction polarity in `motors.c` may require flipping depending on motor driver wiring.

## Build & Flash
Requirements: avr-gcc toolchain (avr-gcc, avr-libc, avrdude).

1. Build:

```powershell
make clean
make
```

2. Flash (adjust `Makefile` programmer settings if needed):

```powershell
make flash
```

## Serial output
- Baud: 9600
- Logs include sensor readings (inches), action markers, and confirmation messages.

## Units
- Distances in this project use inches (converted from original samples in cm). The ultrasonic module returns inches.

## Tuning
- Speed and timing constants live in `main.c` (BASE_PERCENT, FAST_PERCENT_MULT, movement durations) and `motors.h` (MOTOR_TOP_SPEED). Tune these values to match your robot's motors, battery voltage, and friction.
- `SERVO_SETTLE_MS` and ultrasonic sample delays can be reduced if your servo is fast and your sensor is reliable.

## Troubleshooting
- If printed floats show as `?` ensure the final link includes `-lprintf_flt`. The provided `Makefile` includes LDFLAGS to link floating printf.
- If motors act opposite of expectation, swap the forward/backward polarity in `motors.c` or reverse wiring of direction pins.
- If ultrasonic returns 0.0 frequently, check wiring for trigger/echo, and ensure `PCICR`/`PCMSK0` are configured for the correct pin (PB5).

## How the state machine works
1. Check front — if obstacle < threshold, perform `avoid_obstacle()` (left turn + forward)
2. Check right — servo to right, read distance
   - If too close -> small correction (left)
   - If wall lost (distance too large) -> `find_wall()` routine
   - Else -> `follow_wall_behavior()` burst (fast mode)
3. Loop

## Next improvements
- Convert blocking delays to non-blocking state machine with `millis()` for better responsiveness
- Add compile-time `DEBUG` flag to reduce UART output in release
- Add unit-test harness for sensor and motor simulations

## Contact / Authors
See `ABOUT.md` for a short project summary and authorship.


