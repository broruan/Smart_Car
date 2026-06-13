# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded C firmware for a **line-following smart car** based on the **STC32G12K128** microcontroller (80251 ISA). All real-time control runs in ISRs; the main loop is intentionally empty. The car uses 5 IR reflectance sensors for line detection and drives two DC motors via PWM (H-bridge motor driver). An OLED display (SSD1306) and HC-SR04 ultrasonic sensor are implemented but currently disabled.

## Build System

- **IDE:** Keil MDK-ARM (uVision) — project file: `PWM.uvproj`
- **Toolchain:** MCS-251 (80251 instruction set)
- **Output:** `.\list\PWM.hex`
- **Target CPU clock in Keil:** 35 MHz
- **`MAIN_Fosc` in `config.h`:** 24 MHz (timing calculations use this value)

No Makefile, no CLI build toolchain. Development is done entirely within Keil uVision.

## Architecture

### Entry Point

`main()` in `main.c:312` — initializes system, configures peripherals, enables interrupts, then enters an empty `while(1)`. All active logic runs in Timer2 ISR.

### ISR-Driven Control Flow

| Timer | Period | Purpose |
|-------|--------|---------|
| Timer0 | 1 ms | Sets `T0_1ms` flag (for general timing) |
| Timer1 | 10 µs | Increments `time_us` counter for ultrasonic sensor |
| Timer2 | 100 µs | **Core control loop** — reads IR sensors, adjusts motor PWM |

### Line-Following Logic

Located in `Timer2_ISR_Handler()` (`main.c:229`). Five IR sensors (`zuo2, zuo1, zhong, you1, you2` = P0.3, P0.4, P0.2, P0.1, P0.0) form a lateral array. Each sensor returns 0 when over the dark line (reflectance low) and 1 when over the light surface. A chain of if/else-if conditions maps sensor patterns to differential PWM values for left/right motors. This is a **bang-bang controller** with ~15 discrete steering cases — not a PID controller.

Motor speeds are set as percentages (0–100) via `PWM_Run(left%, right%)` which scales against a period of 2000 counts and calls `UpdatePwm(PWMB, &PWMB_Duty)`.

### PWM Motor Control

- **PWMB Channel 5** → P2.0 → Right motor
- **PWMB Channel 6** → P2.1 → Left motor
- Period: 2000 counts
- Helper functions in `main.c:195-215`: `PWM_Left()`, `PWM_Right()`, `PWM_Run()`

### Motor Driver Pins

- `AIN1=P4.5`, `AIN2=P2.7` — Left motor direction (H-bridge)
- `BIN1=P2.5`, `BIN2=P2.6` — Right motor direction (H-bridge)
- Both motors are set to forward at startup.

### Subsystems (currently disabled in main loop)

- **Ultrasonic:** `Ultrasonic_GetDistance()` in `main.c:133` — HC-SR04 driver using P0.6 (TRIG) and P1.4 (ECHO). Returns distance in mm. Uses Timer1 for microsecond counting.
- **OLED:** SSD1306 128x64 via I2C (P5.4=SDA, P3.3=SCL), address 0x78. Drivers in `oled.c`/`iic.c`, fonts in `font.c`. Supports 6x8/8x16 ASCII, 16x16 Chinese characters, and bitmaps.
- **Buttons/LEDs:** 3 buttons (KEY3=P2.4, KEY4=P1.3, KEY5=P1.7) and 3 LEDs (LED1=P4.1, LED2=P4.2, LED3=P4.4) — test code commented out.

## Key Files

| File | Role |
|------|------|
| `main.c` | Application: `main()`, all ISRs, PWM helpers, ultrasonic driver |
| `config.h` | `MAIN_Fosc` (24 MHz), includes core headers |
| `Type_def.h` | Type aliases (`u8`, `u16`, `u32`), constants (`ENABLE`/`DISABLE`, interrupt priorities) |
| `STC32G.H` | Master SFR/sbit register definitions for STC32G series (vendor) |
| `STC32G_PWM.c/.h` | PWM peripheral driver (PWMA channels 1-4, PWMB channels 5-8) |
| `STC32G_Timer.c/.h` | Timer peripheral driver (Timer0-4, multiple modes) |
| `STC32G_GPIO.c/.h` | GPIO initialization (quasi-bidirectional, push-pull, open-drain, high-impedance) |
| `STC32G_NVIC.c/.h` | NVIC interrupt controller abstraction |
| `STC32G_Delay.c/.h` | Busy-loop millisecond delay (calibrated to `MAIN_Fosc`) |
| `STC32G_Timer_Isr.c` | Default Timer ISR stubs (Timer2 ISR is **overridden in main.c**) |
| `STC32G_PWM_Isr.c` | PWM interrupt stubs (clear flags, no application logic) |
| `iic.c/.h` | Bit-banged I2C for OLED (SDA=P5.4, SCL=P3.3) |
| `oled.c/.h` | SSD1306 OLED driver |
| `font.c/.h` | Font bitmaps (6x8 ASCII, 8x16 ASCII, 16x16 Chinese, logo BMP) |
| `STC32G_Soft_I2C.c/.h` | Alternative software I2C (P0.0/P0.1) — **unused, redundant** |

## Pin Assignments

```
P0.0 = you2 (IR sensor, rightmost)
P0.1 = you1 (IR sensor)
P0.2 = zhong (IR sensor, center)
P0.3 = zuo2 (IR sensor, leftmost)
P0.4 = zuo1 (IR sensor)
P0.6 = TRIG (HC-SR04 trigger)
P1.3 = KEY4
P1.4 = ECHO (HC-SR04 echo, high-impedance input)
P1.7 = KEY5
P2.0 = PWM5 (right motor)
P2.1 = PWM6 (left motor)
P2.2 = KEY1
P2.3 = KEY2
P2.4 = KEY3
P2.5 = BIN1 (right motor direction)
P2.6 = BIN2 (right motor direction)
P2.7 = AIN2 (left motor direction)
P3.3 = I2C SCL (OLED)
P4.1 = LED1
P4.2 = LED2
P4.4 = LED3
P4.5 = AIN1 (left motor direction)
P5.4 = I2C SDA (OLED)
```

## Important Notes

- **Encoding:** Source files use GB2312/GBK encoding. Chinese comments will appear garbled (Mojibake) when read as UTF-8. Do not re-save files in UTF-8 — Keil expects the original encoding.
- **Clock mismatch:** Keil project sets CPU at 35 MHz; `config.h` defines `MAIN_Fosc` as 24 MHz. Timer reload values are computed from `MAIN_Fosc`. If the actual hardware runs at a different frequency, timing will be off.
- **Timer2 ISR override:** `STC32G_Timer_Isr.c` contains a commented-out Timer2 ISR. The active Timer2 ISR is in `main.c:229`. Do not enable the one in `STC32G_Timer_Isr.c` or there will be a linker conflict.
- **No test infrastructure.** Testing is done by flashing to hardware and observing behavior.
- **The project name "PWM"** is a misnomer — it started from a PWM example project but evolved into the full smart car firmware.
