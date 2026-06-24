# Self-Balancing Robot

A two-wheeled self-balancing robot built around the ATmega32 microcontroller, MPU6050 IMU, and dual H-bridge motor drivers. Uses a complementary filter for tilt estimation and a PID control loop running at 100 Hz to keep the robot upright.

---

## Table of contents

- [Hardware](#hardware)
- [Circuit connections](#circuit-connections)
- [Firmware overview](#firmware-overview)
- [PID tuning guide](#pid-tuning-guide)
- [Setpoint calibration](#setpoint-calibration)
- [Fuse bits](#fuse-bits)
- [Building and flashing](#building-and-flashing)
- [Troubleshooting](#troubleshooting)

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ATmega32 (40-pin DIP) |
| Clock | 16 MHz external crystal |
| IMU sensor | MPU6050 / HW-123 / GY-521 breakout board |
| Motor driver | Dual H-bridge (e.g. L298N) |
| Motors | 2× DC geared motors |
| Power | 7.4–12V LiPo or Li-ion battery |
| Logic power | 5V linear regulator (e.g. 7805) |

---

## Circuit connections

### ATmega32 pin map

| Pin | Name | Connects to |
|---|---|---|
| 9 | RESET | 10kΩ pull-up to 5V + optional reset button to GND |
| 10 | VCC | 5V regulator output |
| 11 | GND | Common ground rail |
| 12 | XTAL2 | Crystal leg 2 + 22pF cap to GND |
| 13 | XTAL1 | Crystal leg 1 + 22pF cap to GND |
| 30 | AVCC | 5V regulator output |
| 31 | GND | Common ground rail |
| 4 | PB3 / OC0 | Left H-bridge EN / PWM input |
| 17 | PD3 | Left H-bridge IN1 |
| 18 | PD4 | Left H-bridge IN2 |
| 19 | PD5 | Right H-bridge IN3 |
| 20 | PD6 | Right H-bridge IN4 |
| 21 | PD7 / OC2 | Right H-bridge EN / PWM input |
| 22 | PC0 / SCL | MPU6050 SCL |
| 23 | PC1 / SDA | MPU6050 SDA |
| 6 | MOSI | ISP programmer MOSI |
| 7 | MISO | ISP programmer MISO |
| 8 | SCK | ISP programmer SCK |

### MPU6050 (HW-123 breakout)

| MPU6050 pin | Connects to |
|---|---|
| VCC | 5V rail |
| GND | Common ground rail |
| SCL | ATmega32 pin 22 (PC0) |
| SDA | ATmega32 pin 23 (PC1) |
| AD0 | GND (fixes I2C address at 0x68) |
| INT | Not connected — unused (polling-based design) |

> **Note:** The HW-123 board has built-in pull-up resistors on SCL and SDA. Do **not** add external pull-ups — adding them in parallel will reduce the resistance too much for reliable 400 kHz I2C.

### Left motor channel

| H-bridge pin | Connects to |
|---|---|
| VMS (motor supply) | Battery + |
| GND | Common ground rail |
| EN / ENA | ATmega32 pin 4 (PB3) |
| IN1 | ATmega32 pin 17 (PD3) |
| IN2 | ATmega32 pin 18 (PD4) |
| OUT1, OUT2 | Left DC motor leads |

### Right motor channel

| H-bridge pin | Connects to |
|---|---|
| VMS (motor supply) | Battery + |
| GND | Common ground rail |
| EN / ENB | ATmega32 pin 21 (PD7) |
| IN3 | ATmega32 pin 19 (PD5) |
| IN4 | ATmega32 pin 20 (PD6) |
| OUT3, OUT4 | Right DC motor leads |

### Power supply

| Module | Pin | Connects to |
|---|---|---|
| 5V regulator | Vin | Battery + |
| 5V regulator | GND | Common ground rail |
| 5V regulator | Vout | ATmega32 VCC / AVCC + MPU6050 VCC |

> **Critical:** The motor supply (battery → H-bridge VMS) must **not** pass through the 5V logic regulator. Keep motor power and logic power on separate rails, joined only at the common ground node.

### Passive components

| Component | Value | Location |
|---|---|---|
| Crystal | 16 MHz | Between XTAL1 and XTAL2 |
| Load capacitors (×2) | 22 pF | Each crystal leg to GND |
| Reset pull-up | 10 kΩ | RESET pin to 5V |
| Regulator input cap | 0.1 µF | Regulator Vin to GND |
| Regulator output caps | 0.1 µF + 10 µF | Regulator Vout to GND |
| MCU bypass caps (×2) | 0.1 µF | VCC–GND and AVCC–GND, placed directly at the chip pins |

---

## Firmware overview

### Control loop

Timer1 generates a CTC interrupt at exactly 100 Hz (every 10 ms). Each interrupt sets a flag; the main loop waits for the flag, then runs one full control cycle:

```
read MPU6050 (I2C) → update pitch (complementary filter) → compute PID → drive motors
```

### Tilt estimation — complementary filter

The MPU6050 provides raw accelerometer and gyroscope readings over I2C. The firmware computes tilt angle using a complementary filter:

```
pitch = 0.98 × (pitch + gyro_rate × dt) + 0.02 × accel_angle
input = pitch + 180.0
```

The gyroscope is fast and smooth but drifts slowly over time. The accelerometer is noisy but drift-free. The 0.98/0.02 blend keeps the response smooth while correcting long-term drift.

### PID control

```c
error     = setpoint - input
integral += Ki × error × dt          // accumulates over time
dInput    = input - lastInput         // derivative on measurement
output    = Kp × error + integral - Kd × (dInput / dt)
```

- **Kp** — proportional gain. Controls how hard the motors push back against tilt. Too low: robot falls before reacting. Too high: oscillates rapidly.
- **Kd** — derivative gain. Damps oscillation caused by Kp. Acts like a shock absorber on the correction.
- **Ki** — integral gain. Corrects slow steady-state drift. Keep as small as possible while still preventing lean.

Anti-windup is implemented using conditional integration: the integral only accumulates further if doing so would pull the output back toward zero, not push it further into saturation.

### Motor output

PWM is generated by Timer0 (left motor, OC0 on PB3) and Timer2 (right motor, OC2 on PD7), both in Fast PWM mode with prescaler 8.

The raw PID output (±255) is scaled to a deadband-aware PWM range:

```
scaled_pwm = MIN_ABS_SPEED + (abs_output / 255) × (255 - MIN_ABS_SPEED)
```

`MIN_ABS_SPEED = 110` prevents the motors from stalling in the deadband below which they don't actually turn.

### Safety

- If `input` falls outside 160°–200°, `isFallen` is set and motors stop.
- Recovery is allowed when `input` returns to 172°–182° and no hardware error is active.
- `hardwareError` is set on I2C timeout; the TWI peripheral is fully re-initialised (not just disabled) to recover from bus lockup.

---

## PID tuning guide

Tune one parameter at a time, in this order. Re-flash after each change.

### Step 1 — find Kp (Kd = 0, Ki = 0)

Start with:
```c
const float Kp = 30.0f;
const float Kd = 0.0f;
const float Ki = 0.0f;
```

Hold the robot upright and tip it a few degrees. If the wheels barely react, increase Kp in large steps (try 50 → 80 → 120). Stop when tipping it causes visible oscillation. Back off 20% from that value. That is your working Kp.

### Step 2 — add Kd to damp oscillation

Keep Kp fixed. Start with `Kd = 0.5` and increase in steps of 0.5. You want the oscillation to settle quickly and smoothly without the response feeling sluggish. Typical range: 0.5–2.5.

### Step 3 — test free standing

With Kp and Kd set, let the robot stand on its own. If it balances but slowly drifts to one side over several seconds, proceed to Ki.

### Step 4 — add Ki to fix drift

Start with `Ki = 5`. Increase in steps of 5 until the drift disappears. Stop as soon as it does — too much Ki causes a slow rocking or surging motion.

---

## Setpoint calibration

`setpoint` is the value of `input` when the robot is physically perpendicular to the ground. It is not a universal constant — it must be measured on your specific robot.

**How to find your setpoint:**

1. Set `Kd = 0`, `Ki = 0`, and a moderate `Kp` (enough for visible wheel response).
2. Hold the robot against a wall or use a spirit level to verify true vertical.
3. Release it gently and observe which direction it immediately drives.
4. Adjust `setpoint` in the direction that reduces that drive: if it drives forward, increase `setpoint`; if it drives backward, decrease it.
5. Re-flash and repeat, narrowing in until releasing from true vertical produces no consistent directional drive.

> **Why 176.4?** The `+ 180.0` in `input = pitch + 180.0` shifts the atan2 output range away from the ±180° wraparound boundary. The `−3.6` offset from 180 is a sensor-specific calibration value from whoever originally wrote the base firmware — it will almost certainly be wrong for your robot.

---

## Fuse bits

The fuse bits must be programmed **separately** from flashing the firmware, using your ISP programmer.

For a 16 MHz full-swing crystal oscillator with maximum startup time (recommended):

| Fuse | Value |
|---|---|
| Low fuse | `0xFF` |
| High fuse | `0x89` (JTAG disabled) or `0x99` (JTAG enabled) |

> **Warning:** Incorrect fuse bits are the most common reason a correctly-compiled 16 MHz firmware produces no output at all. If the chip runs on the internal 1 MHz RC oscillator instead, every timer, PWM frequency, and I2C bit rate will be wrong by a factor of 16.

---

## Building and flashing

### Requirements

- `avr-gcc` toolchain
- `avr-libc`
- `avrdude` (for flashing)
- An ISP programmer (USBasp, AVRISP mkII, etc.)

### Compile

```bash
avr-gcc -mmcu=atmega32 -DF_CPU=16000000UL -Wall -Os -o self_balance.elf self_balance.c -lm
avr-objcopy -O ihex -R .eeprom self_balance.elf self_balance.hex
```

### Flash

```bash
avrdude -c usbasp -p m32 -U flash:w:self_balance.hex:i
```

Replace `-c usbasp` with your programmer type if different.

### Memory usage

| Section | Used | Available | % |
|---|---|---|---|
| Flash | 4206 bytes | 32768 bytes | 12.8% |
| SRAM | 63 bytes | 2048 bytes | 3.1% |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot falls in one consistent direction even when released from true vertical | Setpoint mismatch | Re-calibrate setpoint using the drift-test method above |
| Motors barely react to tilt | Kp too low | Increase Kp in large steps until there is visible resistance |
| Rapid oscillation, can't settle | Kp too high or Kd too low | Reduce Kp by 20% or increase Kd |
| Balances briefly then slowly drifts | Ki too low | Increase Ki in small steps |
| Slow rocking or surging motion | Ki too high | Reduce Ki |
| Correct direction one way, wrong direction the other | One motor wired in reverse | Swap OUT1/OUT2 (or OUT3/OUT4) on the relevant H-bridge channel |
| Both directions wrong | Motor direction inverted globally | Set `INVERT_MOTOR_DIRECTION true` in firmware |
| Robot unresponsive, no motor movement | I2C lockup or sensor fault | Check MPU6050 wiring; confirm AD0 is tied to GND; check 5V supply stability |
| Compiles but runs at wrong speed | Fuse bits still set to internal RC oscillator | Re-program fuse bits for external crystal (see Fuse bits section) |
| `initializer element is not constant` compiler error | Non-const global used to initialise another global | Already fixed in this firmware — ensure `originalSetpoint` is declared `const` |

---

## Key firmware constants

```c
#define F_CPU              16000000UL  // must match your crystal frequency
#define MIN_ABS_SPEED      110         // PWM floor — below this motors stall
#define INVERT_MOTOR_DIRECTION false   // set true if both motors drive the wrong way

const float Kp             = 30.0f;   // proportional gain — tune first
const float Kd             = 0.0f;    // derivative gain  — tune second
const float Ki             = 0.0f;    // integral gain    — tune last

float setpoint             = 176.40f; // calibrate this for your specific robot
const float alpha          = 0.98f;   // complementary filter blend (gyro weight)
const float dt             = 0.01f;   // control loop period — do not change
const float motorSpeedFactorLeft  = 0.80f;  // trim if one motor is faster
const float motorSpeedFactorRight = 0.80f;
```
