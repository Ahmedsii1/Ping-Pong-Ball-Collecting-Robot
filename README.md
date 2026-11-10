# Robo League — Ball-Collecting Robot

A remote-controlled competition robot built from scratch for **Robo League**, a 1v1 robotics collection contest organised by the **IEEE Robotics & Automation Society – HU Student Chapter** (Jordan).

The robot drives around a shared arena, picks up ping-pong balls with a roller intake, stores them in an onboard hopper, and drops them into a raised basket, all while the opposing robot is competing for the same balls.

> The goal of this project is to learn how to design and build a complete robot end to end: mechanical CAD, 3D printing, electronics, and especially the embedded firmware.

---

## The challenge

| Parameter | Value |
|---|---|
| Format | Head-to-head 1v1, both robots in one shared arena |
| Match length | 3 minutes |
| Arena | 2 m × 2 m, 30 cm perimeter wall, textured floor |
| Balls | 30 ping-pong balls (40 mm): 22 white (+1 pt), 8 golden (+3 pts, in two corner clusters of 4) |
| Basket | 15 × 15 cm opening, 20 cm above the floor, on the opposite side from the start zone |
| Robot footprint | Max 30 × 30 cm at all times (height unrestricted) |
| Contact | Pushing and blocking allowed; damaging, trapping or flipping the opponent is not |

A separate judging panel also scores **mechanical design, code quality, and wiring quality**, independent of match results.

### The key design constraint
Rule 8 of the contest says that **any ball displaced without being collected is worth zero points for the rest of the match.** That one rule shaped the whole design:

- **No shooting or launching mechanism.** A missed shot means a lost ball.
- **The intake has to collect what it touches.** A mechanism that bumps balls away is worse than not having one.
- That is why the intake uses **compliant PETG flaps** with a deliberate torque margin, so that it grabs balls instead of deflecting them.

---

## Robot design

- **Drive:** 4WD skid-steer. The two motors on each side are wired in parallel on one driver channel.
- **Intake:** a motorised roller with compliant PETG flaps that sweeps balls into a channel/hopper. The roller is bidirectional, so a jam can be cleared by reversing it.
- **Storage and scoring:** a hopper with a **servo-controlled rear discharge gate**.
- **Lift:** a pulley lift driven by a NEMA17 stepper, which raises the ball bucket to the 20 cm basket height.
- **Structure:** 3D-printed parts designed in Fusion 360 and printed on a Bambu Lab A1. PETG is used for flexures and structural parts, PLA for everything else.

### Engineering notes
- **Curved paddles:** a curved paddle (40 mm concave radius) cuts the fold angle the flap needs from about 100° to about 80°. That makes a single-extrusion PETG flexure viable.
- **PETG over PLA for flexures:** PLA is stiffer and brittle, so it delaminates after repeated flexing. PETG survives cyclic bending.
- **Hood clearance drives intake torque:** most of the roller load comes from how far the flaps deflect against the hood. Shimming the hood helps more than swapping the motor.
- **Driver current headroom:** two stalled gearmotors can exceed a single TB6612FNG channel's continuous rating, so motor channels are paralleled where needed.
- **Tall prints fail from leverage, not adhesion:** sacrificial cone buttresses fixed tall, narrow parts that a brim alone could not hold.

---

## Electronics

The robot has gone through two control-system revisions.

### Rev A: ESP32 + PS4 controller
| Component | Part |
|---|---|
| Controller | ESP32-WROOM-32 (38-pin DevKit) |
| Motor drivers | 2 × TB6612FNG (left drive / right drive + intake roller) |
| Intake motor | JSUMO 12 V 250 RPM DC gearmotor |
| Gate servo | MG996R |
| Remote | Sony DualShock 4 over Bluetooth (Bluepad32) |
| Power | 3S motor rail (12.6 V full charge); separate battery + buck converter for logic |

**Pin map**

| Function | GPIO |
|---|---|
| Left drive: PWMA / AIN1 / AIN2 | 25 / 26 / 27 |
| Right drive: PWMA / AIN1 / AIN2 | 32 / 33 / 13 |
| Driver standby (shared) | 4 |
| Intake roller: PWM / DIR | 19 / 21, 22 |
| Gate servo | 18 |

### Rev B: Arduino Uno + Bluetooth phone app (with lift)
| Component | Part |
|---|---|
| Controller | Arduino Uno |
| Wireless | HC-05 Bluetooth (SoftwareSerial on D11/D12) |
| Drive | 4 × N20 gearmotors on one TB6612FNG (A = right, B = left) |
| Lift | NEMA17 stepper (JK42HS40-1704) on an A4988, full-step (STEP/DIR on D6/D9) |
| Gate servo | SG90 on D10 |
| Power | 12 V for the stepper driver; 6 V buck for the TB6612 motor supply and servo |

---

## Controls

### Rev A: DualShock 4
| Input | Action |
|---|---|
| R2 (analog) | Drive forward |
| L2 (analog) | Drive reverse |
| Left stick X | Steering |
| R1 | Toggle intake forward |
| L1 | Toggle intake reverse (switches direction instantly if already running) |
| D-pad ↑ / ↓ | Trim intake speed (±10, range 200–250) to compensate for battery sag |
| Cross / Square / Circle | Gate presets: 25° / 45° / 65° |
| D-pad → / ← | Fine-adjust gate ±5° (limited to 25–70°) |

### Rev B: Phone app (single-character commands)
| Command | Action |
|---|---|
| `W` / `S` / `A` / `D` | Forward / reverse / left / right |
| `X` | Stop |
| `1` / `2` | Lift up / down |
| `O` | Toggle gate |

---

## Firmware highlights (Rev A)

- **Analog throttle with deadzone and steering gain:** trigger-based throttle mixed with stick steering for skid-steer control.
- **Slew-rate limiting:** PWM changes are ramped every 10 ms tick. This protects the drivetrain and gearboxes from sudden reversals and limits current spikes.
- **Link-loss failsafe:** the motors stop automatically if the controller connection drops for more than 300 ms.
- **Live intake trim:** intake duty can be adjusted from the controller during a match to make up for battery voltage droop.
- **Bounded servo control:** gate presets plus clamped fine adjustment. The servo parks at a known position on power-up.
- **Bench-test sketches:** standalone sketches for setting drive and roller PWM over Serial, used to characterise each subsystem before integration.

---

## Tools

- **Firmware:** Arduino IDE / ESP32 Arduino core, Bluepad32
- **CAD:** Fusion 360, with parametric Python API scripts that share a single parameter file
- **Slicing / printing:** Bambu Studio, Bambu Lab A1
- **Editor:** VS Code

---

## Repository structure

```
├── firmware/
│   ├── rev_a_esp32_ds4/        # ESP32 + DualShock 4 competition firmware
│   ├── rev_b_uno_hc05_lift/    # Arduino Uno + HC-05 + stepper lift
│   └── bench_tests/            # Serial-controlled motor/roller test sketches
├── cad/
│   ├── scripts/                # Fusion 360 Python API scripts (RL##_Name)
│   └── stl/                    # Printable parts
├── docs/
│   ├── wiring/                 # Wiring diagrams and pinouts
│   └── RoboLeague_Booklet.pdf  # Contest rulebook
└── README.md
```

---

## Status

🚧 **In progress.** The drive, intake and gate firmware is written. Mechanical CAD (chassis and roller core done; side plates, hood, container and mounts in progress) and lift integration are ongoing.

## Author

**Ahmed Al Aydi**, Mechatronics Engineering, University of Jordan
