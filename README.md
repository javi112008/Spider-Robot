An ESP32-powered, four-legged robot with 12 servos, inverse kinematics, and Bluetooth phone control. We created this project for the Grand Terrace High School Engineering Pathway Halloween Design Contest.

The project brings together CAD, hardware assembly, and embedded programming to coordinate three joints on each leg. Instead of setting every servo angle manually for each movement, the code calculates joint angles from target foot positions.

## Features

- Forward and backward walking, plus left and right turning through the Dabble GamePad.
- A crawler gait that moves one leg at a time in the order **front left ,back right, front right, back left**.
- Smooth foot movements using smoothstep interpolation and sine-shaped lifting arcs.
- An adjustable X stance and a gradual move into the starting pose after servo centering.
- Wave, bounce, tap, spin, and combined demo routines.
- Serial commands for testing individual legs and tuning the gait without reflashing.

## Hardware

The firmware targets an ESP32 development board using the Arduino framework. Each of the four legs uses three servos: coxa, femur, and tibia, for **12 controlled joints** total.

### Servo signal connections

| Leg | ID | Coxa GPIO | Femur GPIO | Tibia GPIO |
| --- | --- | --- | --- | --- |
| Front left | 0 | 2 | 4 | 16 |
| Front right | 1 | 17 | 5 | 18 |
| Back left | 2 | 12 | 13 | 14 |
| Back right | 3 | 25 | 26 | 27 |

Use a servo power supply that matches your servos' voltage and current requirements, and connect its ground to the ESP32 ground. Do not power all 12 servos from the ESP32's 3.3 V pin. The repository does not yet specify the servo model, battery, or power distribution hardware.

### Geometry and calibration

The current code uses a **63 mm femur** and **95 mm tibia**, with `Y_Rest = 34 mm`, `Z_Rest = -80 mm`, and a tibia angle offset of `15.4°`. 

Joint inversion arrays account for mirrored servo mounting. Check these settings and the geometry in [`src/main.cpp`](src/main.cpp) before using the firmware on a different build.

## Build and upload

This repository uses PlatformIO. The configuration in [`platformio.ini`](platformio.ini) selects:

| Setting | Value |
| --- | --- |
| Board | `esp32dev` |
| Framework | Arduino |
| Espressif32 platform | `6.9.0` |
| Libraries | `ESP32Servo`, `DabbleESP32` |
| Serial monitor | 115200 baud, LF line ending |

1. Install VS Code and the PlatformIO IDE extension.
2. Clone this repository and open its root folder in VS Code.
3. Connect the ESP32 over USB.
4. Use PlatformIO **Build**, then **Upload**. PlatformIO installs the declared dependencies.
5. Open the serial monitor at **115200 baud** with **LF/newline** enabled.

If you already have the PlatformIO CLI, run these commands from the project folder:

```bash
pio run
pio run --target upload
pio device monitor
```

Support the robot with its feet clear before the first powered test. On startup, the code commands every servo to 90°, then moves the legs into their neutral stance. Check servo orientation and mechanical clearance before letting it walk.

## Phone controls

Open the Dabble app, connect to **`SpiderBot`**, and use the **GamePad** module.

| Input | Action |
| --- | --- |
| Hold D-pad up | Walk forward |
| Hold D-pad down | Walk backward |
| Hold D-pad left | Turn left |
| Hold D-pad right | Turn right |
| Release D-pad | Finish the current leg's step, then return to neutral stance |
| Triangle | Wave |
| Circle | Bounce |
| Square | Tap |
| Cross | Spin routine |

Trigger demo routines while the robot stands idle. The firmware finishes each demo before resuming normal control. The spin routine performs a fixed turning sequence; it does not measure or guarantee a full rotation.

## Serial commands

Send one command per line. Use lowercase command names as shown. Coordinates and distances use millimeters unless noted otherwise.

| Command | Function |
| --- | --- |
| `help` | Print the command list |
| `standx` | Stop walking and move to the neutral X stance |
| `xshow` | Print stance coordinates and selected gait settings |
| `xmode 1` / `xmode 2` | Apply a stance preset; mode 2 swaps the preset front/rear X positions |
| `xfront <mm>` | Set both front legs' neutral X positions |
| `xrear <mm>` | Set both rear legs' neutral X positions |
| `xspread <mm>` | Set the signed Y spread for the left and right legs |
| `leg N X Y Z` | Move one leg using its ID from the wiring table |
| `all X Y Z` | Send the same local XYZ target to every leg |
| `stride <mm>` | Set the gait's stride parameter |
| `lift <mm>` | Set the foot-lift arc height |
| `step <deg>` | Set the startup servo ramp increment; range 0.1â€“20Â° |
| `delay <ms>` | Set the movement frame delay; range 1â€“100 ms |
| `pace <ms>` | Set the pause between gait phases; range 0â€“1000 ms |
| `swing <frames>` | Set swing interpolation frames; range 4â€“60 |
| `glide <frames>` | Set support-glide interpolation frames; range 4â€“60 |
| `dance wave` | Run the wave routine |
| `dance bounce` | Run the bounce routine |
| `dance tap` | Run the tap routine |
| `dance spin` | Run the turning routine |
| `dance combo` | Run bounce, wave, tap, then spin |

For example, while the robot stands idle:

```text
xshow
stride 20
lift 25
swing 30
glide 20
standx
```

These values show the command format, not a validated calibration for every build. Serial tuning only changes values in RAM; restarting restores the source-code defaults. To keep a setting, change its default in `src/main.cpp` and upload again. The `step` command controls the startup ramp, which has already run by the time serial commands become available.

The startup stance uses front X = **15 mm**, rear X = **15 mm**, and spread = **10 mm**. Calling `xmode 1` instead loads front X = **15 mm** and rear X = **âˆ’15 mm**, so it does not restore the exact startup pose.

## How the movement works

For each step, the robot first shifts the support legs opposite the requested direction. It then lifts the selected leg along an arc and glides all four legs back to their neutral positions. The inverse-kinematics function converts each intermediate foot target into coxa, femur, and tibia servo angles.

The default gait uses a 30 mm stride parameter, a 50 mm lift, 30 swing frames, 20 glide frames, and a 12 ms frame delay. `supportShiftRatio` and `swingReachRatio` scale the support shift and swinging leg's reach. These values describe commanded motion, not measured travel distance.

## Current limitations

Due to time constraints, we weren't able build out a better chassis for the robot. As such, most of this code has worked but the robot itself cannot properly grip on surfaces 

The firmware uses open-loop servo commands: it does not measure body tilt, foot contact, or actual joint position. Tracked foot coordinates represent commanded positions. Walking performance depends on servo alignment, power delivery, mechanical geometry, and the surface underneath the robot.

Movement routines use blocking loops. The code processes Dabble input during motion, but it handles serial commands between routines. Releasing the D-pad requests a stop after the current leg's step; neither that release nor `standx` provides an immediate emergency stop unfortuantely.

The code clamps calculated angles, but it does not fully validate the reachable workspace or mechanical collisions... impleting this was far out of the scope for this project


## Contributors

- **Javier "Javi" Medorio Cancino** - CAD design, hardware design, and main programming.
- **Jonthan Taylor** - main assembly, programming, and physics calculations.
