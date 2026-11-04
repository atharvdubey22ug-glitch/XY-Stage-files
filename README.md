# F²R XY Flexure Stages

Open design files for the **F²R (Fiction to Reality)** 3D-printed XY positioning stages: compliant (flexure) mechanisms driven by low-cost stepper motors, built to bring nanometre-scale positioning to labs, classrooms and makers at a fraction of the cost of commercial stages.

> **Licence:** F²R Hardware License v1.0. Personal, educational, experimental and non-commercial research use is permitted with attribution. Commercial manufacture, sale or use requires prior written permission from F²R. See [`LICENSE.pdf`](LICENSE.pdf).

---

## Contents

1. [Which stage is which](#which-stage-is-which)
2. [Repository files](#repository-files)
3. [Performance](#performance)
4. [Bill of materials](#bill-of-materials)
5. [Step 1 — Print the parts](#step-1--print-the-parts)
6. [Step 2 — Convert the motors to bipolar](#step-2--convert-the-motors-to-bipolar)
7. [Step 3 — Wire the electronics](#step-3--wire-the-electronics)
8. [Step 4 — Set the driver current and microstepping](#step-4--set-the-driver-current-and-microstepping)
9. [Step 5 — Assemble the stage](#step-5--assemble-the-stage)
10. [Step 6 — First motion test](#step-6--first-motion-test)
11. [Troubleshooting](#troubleshooting)
12. [Buying a pre-assembled stage](#buying-a-pre-assembled-stage)
13. [Attribution and contact](#attribution-and-contact)

---

## Which stage is which

| Stage | File | Notes |
|---|---|---|
| **XY One** (original) | `XY STAGE 1.step` | First F²R stage; the one that demonstrated sub-20 nm steps. |
| **XY One** (revised) | `XY STAGE UPDATED.step` | Updated revision of XY One. Use this one if you are building XY One. |
| **XY Lab** (current) | `XY Lab.step` | Follow-on model. Drives the flexures through a leadscrew, with the flexure itself providing the preload. M3 mounting holes on the fixed frame and a 9-hole moving platform. |

If you are building one for the first time, **start with XY Lab**.

## Repository files

| File | What it is |
|---|---|
| `XY Lab.step` | CAD for the XY Lab stage |
| `XY STAGE 1.step` | CAD for the original XY One |
| `XY STAGE UPDATED.step` | CAD for the revised XY One |
| `Electronics and parts.pdf` | Original parts list with purchase links |
| `LICENSE.pdf` / `LICENSE.docx` / `F2R Hardware License v1.0.docx` | Licence text |

`.step` files open in FreeCAD, Fusion 360, Onshape, SolidWorks and most other CAD tools. Export each body to STL (or 3MF) for slicing.

## Performance

Measured on XY One:

- **First validation (4 July 2026):** under a microscope against a lithographed wafer with 4 µm-spaced bars, counting single-step taps needed to cross a 4 µm gap gave **< 20 nm per step**, repeated over many runs.
- **Optical tracker metrology:** ~8.4 nm resolved microstep, ~0.6 nm measurement noise floor, and a **~210 nm repeatable minimum increment** (25 microsteps per command: mean 210.5 nm, CV 0.13, no backward steps).

The difference between *resolved* and *repeatable* matters: single microsteps can be resolved, but at that scale the stage shows dead steps and occasional retrograde steps (stiction / motor detent torque). For reliable positioning, command moves in groups of microsteps.

---

## Bill of materials

Quantities are for a complete stage with its electronics. Links are in `Electronics and parts.pdf`.

### Mechanical

| Qty | Part | Notes |
|---|---|---|
| 1 | Main body (printed) | From the `.step` file for your stage |
| 1 per motor | Motor couplers (printed) | |
| as needed | M3 hex screws and nuts | |
| — | **PETG filament** | PETG only. PLA creeps and fatigues in flexures. |

### Electronics

| Qty | Part | Notes |
|---|---|---|
| 2–3 | 28BYJ-48 5 V stepper motors | One per axis. Must be converted to bipolar (Step 2). |
| 2–3 | TMC2208 stepper driver modules | One per motor |
| 2–3 | Stepper driver expansion boards (A4988-style carrier) | One per driver; makes wiring and microstep jumpers easy |
| 1 | Arduino Nano (or ESP32) | Controller |
| 1 | 12 V, 2 A DC power supply | Plus a barrel-jack connector |
| — | Jumper wires | |
| optional | WS2812 LED ring or any light source | Illumination if you use the stage under a microscope |

---

## Step 1 — Print the parts

1. Open the `.step` file for your stage and export each body to STL/3MF.
2. Print in **PETG**. The flexures rely on PETG's toughness and fatigue resistance.
3. Suggested starting point (tune for your printer):
   - Print the main body **flat on the bed**, in the orientation it appears in the CAD file, so the flexure hinges are formed by in-plane perimeters rather than stacked layers.
   - 0.2 mm layer height, 0.4 mm nozzle.
   - Enough walls that the thin flexure sections print as **solid perimeters with no infill**.
   - Slow down outer walls for clean, consistent hinge thickness.
4. After printing, check that every flexure hinge is intact and free of strings or blobs. A cracked or uneven hinge will cause drift and uneven motion.

## Step 2 — Convert the motors to bipolar

The 28BYJ-48 is sold as a unipolar motor. Converting it to bipolar lets it run on the TMC2208 with microstepping and gives noticeably more precise motion.

1. Pop off the small blue plastic cover on the motor.
2. Cut the PCB trace connected to the **red (centre-tap) wire**, so the two half-coils of each phase are joined in series.
3. Leave the red wire disconnected. The motor now has two coils on four wires:
   - **Coil A:** orange + pink
   - **Coil B:** yellow + blue
4. Check with a multimeter: orange–pink and yellow–blue should each show a resistance; orange–yellow should be open circuit.

Video walkthrough: <https://www.youtube.com/watch?v=pOMRzBdshEY&t=94s>

## Step 3 — Wire the electronics

> ⚠️ Never connect or disconnect a motor while the driver is powered. It can destroy the TMC2208.

1. Plug each TMC2208 into an expansion board. Match the module's **EN** pin to the board's EN position (check the pin labels; do not go by orientation alone).
2. Connect the 12 V supply to each expansion board's power input. Observe polarity.
3. Connect each motor to its board's motor header: **coil A → 1A/1B, coil B → 2A/2B**. If a motor vibrates but doesn't turn, swap the two wires of *one* coil.
4. Connect the Nano to each board. Suggested pin map (change it in your firmware if needed):

| Axis | STEP | DIR | EN |
|---|---|---|---|
| X | D2 | D3 | D8 (shared) |
| Y | D4 | D5 | D8 (shared) |
| Z / 3rd (optional) | D6 | D7 | D8 (shared) |

5. Connect Nano **GND to the 12 V supply GND** (common ground) and Nano **5 V** to each board's logic supply if the board needs it.
6. Power the Nano from USB while testing.

## Step 4 — Set the driver current and microstepping

**Current (Vref).** Before connecting the motors, power the board and turn the TMC2208's trim potentiometer while measuring the voltage between the pot and GND.

- Start low, around **0.3–0.4 V**. The bipolar-converted 28BYJ-48 has high coil resistance, so it does not need much current.
- Run the motor for a few minutes and check it stays only warm to the touch. Increase Vref slightly only if it skips steps.

**Microstepping.** In standalone mode the TMC2208's MS1/MS2 pins set the microstep mode (set by the jumpers on the expansion board):

| MS1 | MS2 | Mode |
|---|---|---|
| low | low | 1/8 (default, no jumpers) |
| high | low | 1/2 |
| low | high | 1/4 |
| high | high | 1/16 |

Use **1/16** for the finest motion. Check your module's pin labels, since TMC2208 carriers differ.

## Step 5 — Assemble the stage

1. Press-fit or screw the motor couplers onto the motor shafts.
2. Mount the motors to the main body with M3 screws and nuts. Do not over-tighten into printed plastic.
3. Connect each coupler to its drive input (the leadscrew on XY Lab). On XY Lab the flexure provides the preload, so the drive should seat without any extra springs.
4. Move each axis gently by hand. It should flex smoothly and spring back to centre. Any grinding or binding means a part is misaligned.
5. Bolt the stage to a rigid, heavy base (an optical breadboard is ideal). If you are imaging the stage, mount the microscope or camera to the **base, not to the stage**, so motor vibration does not reach the optics.

## Step 6 — First motion test

Upload this minimal sketch to the Nano. It moves the X axis forward and back by a fixed number of microsteps.

```cpp
// Minimal STEP/DIR test for one axis (X). Adjust pins to your wiring.
const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int EN_PIN   = 8;

const int STEPS        = 25;   // microsteps per move (~210 nm repeatable increment on XY One)
const int STEP_DELAY_US = 800; // lower = faster

void move(int steps, bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // LOW enables the TMC2208
}

void loop() {
  move(STEPS, true);
  delay(1000);
  move(STEPS, false);
  delay(1000);
}
```

Then:

1. Power the 12 V supply, then the Nano.
2. Watch the platform under a microscope or with a dial indicator. It should step back and forth once a second.
3. To check resolution yourself, image a known feature (a calibration slide or a lithographed pattern) and count steps needed to cross a known distance.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Motor hums or vibrates but doesn't turn | Coil wires paired wrongly | Recheck pairs with a multimeter; swap the two wires of one coil |
| Motor doesn't move at all | EN not pulled low, no 12 V, or no common ground | Check EN pin, supply, and Nano GND ↔ supply GND |
| Motor gets hot | Vref too high | Lower Vref |
| Skipped steps / stalls | Vref too low or stepping too fast | Raise Vref slightly or increase `STEP_DELAY_US` |
| Single microsteps sometimes do nothing or go backward | Normal at this scale (stiction / detent torque) | Command moves in groups of microsteps (e.g. 25) |
| Platform drifts over time | Flexure creep, wrong material, or damaged hinge | Use PETG, inspect hinges, allow settling time after large moves |
| Image shakes while moving | Optics mounted to the stage | Mount optics to the base, not to the stage |

## Buying a pre-assembled stage

A pre-assembled **XY Lab** with electronics (Arduino Nano or ESP32) is available from [f2r.site](https://f2r.site). The 12 V power supply is not included.

## Attribution and contact

If you build, modify or share this design, credit **F²R (Fiction to Reality)**, link back to this repository, and clearly state what you changed. Modified designs must not be presented as official F²R designs.

For commercial licensing or questions, reach out via [f2r.site](https://f2r.site) or open an issue in this repository.

© 2026 Atharv Dubey / F²R
