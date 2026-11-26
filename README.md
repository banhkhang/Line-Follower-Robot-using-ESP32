# ESP32 Line-Following Robot

A two-wheeled line-following robot built on a custom ESP32-WROOM board.
It follows a line using an array of 7 TCRT5000 reflective sensors, and
reads a load cell through an HX711 amplifier.

The code is written to be **easy for newcomers**: plain C, no RTOS tasks,
one simple loop, and a strict folder layout so you always know where to look.

## Current status

| Feature | State |
|---|---|
| HX711 / load cell reading | Done - use it to calibrate |
| 7x TCRT5000 line sensor array | Planned |
| Motor driver (PWM) | Planned |
| Wheel encoders | Planned |
| Line-following control | Planned |

## Hardware

- ESP32-WROOM-32 on a custom PCB (3.3 V logic)
- 7x TCRT5000 reflective sensors (analog)
- HX711 load cell amplifier + load cell (powered from 3V3)
- 2 DC motors with encoders, driven by a dual H-bridge (2 PWM pins per motor)

## Pin map

Taken from the board schematic. **`components/config/include/config.h` is the
single source of truth** - if the PCB changes, edit it there.

| Function | GPIO | Schematic net |
|---|---|---|
| HX711 data out | 14 | DOUT |
| HX711 clock | 27 | PD_SCK |
| Line sensor 1..7 | 36, 39, 34, 35, 32, 33, 25 | A1..A7 |
| Sensor control | 26 | CTRL |
| Motor A PWM | 12, 13 | PWMA1, PWMA2 |
| Motor B PWM | 16, 15 | PWMB1, PWMB2 |
| Encoder A (A / B) | 21 / 19 | ENCA-A / ENCA-B |
| Encoder B (A / B) | 18 / 17 | ENCB-A / ENCB-B |
| User LED | 2 | USR_LED |
| User button | 4 | USR_BTN (pressed = LOW) |
| Free, on headers | 5, 22, 23 | JP4 / JP6 |

### Things to know about the pins

- **GPIO 34, 35, 36, 39 are input-only.** Fine for sensors, never use them as outputs.
- **GPIO 25 uses ADC2**, which stops working while Wi-Fi is on. We do not use Wi-Fi, so this is fine.
- **GPIO 0, 2, 5, 12, 15 are strapping pins** (they are read at boot). Do not hold them HIGH/LOW with external hardware during reset. In particular, motor driver inputs on 12 and 15 must be LOW at power-up.
- **CTRL (GPIO 26)** is assumed to enable the sensor array emitters. Please confirm and update this README.

## Project structure

```
sample_project/
|-- CMakeLists.txt              project file (do not touch)
|-- main/
|   |-- main.c                  just calls app_start()
|   `-- CMakeLists.txt
`-- components/
    |-- config/include/config.h pins and tunable numbers
    |-- drivers/                talks to ONE chip/peripheral each
    |   |-- hx711.c
    |   `-- include/hx711.h
    |-- service/                makes driver data useful (averaging, grams...)
    |   |-- loadcell_service.c
    |   `-- include/loadcell_service.h
    `-- application/            decides what the robot does
        |-- app.c
        `-- include/app.h
```

### The three layers

Each layer only talks to the one directly below it:

```
application  ->  service  ->  drivers  ->  hardware
 (what to do)   (what data     (how to talk
                 means)         to the chip)
```

- **drivers**: the lowest level. Reads or writes pins. Knows nothing about grams, lines or goals.
- **service**: uses drivers and adds meaning (average, subtract zero, convert to grams).
- **application**: the robot's behavior. The only place with "if the line is on the left, turn left" logic.

Rule of thumb: if you are unsure where code goes, ask *"does this touch a pin?"*
(driver), *"does it turn numbers into meaning?"* (service), *"does it decide something?"* (application).

> The folder is called `drivers` (plural) on purpose. ESP-IDF already has a
> component named `driver`, and using the same name would break the build.

### Adding a new part

To add e.g. the motors: put `motor.c` and `include/motor.h` in `components/drivers/`,
add `"motor.c"` to `SRCS` in `components/drivers/CMakeLists.txt`, then create a
`motor_service` the same way as `loadcell_service`.

## Getting started

Requirements: [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) v5.x and a USB-serial connection to the board.

```bash
idf.py set-target esp32     # first time only
idf.py build
idf.py -p COMx flash monitor   # Linux/Mac: /dev/ttyUSB0
```

Exit the monitor with `Ctrl + ]`.

## Calibrating the load cell

1. **Wire it up**: HX711 `VCC` to 3V3, `GND` to GND, `DOUT` to GPIO 14, `SCK` to GPIO 27, load cell wires to `E+ E- A+ A-`.
2. **Flash** and open the monitor. With the load cell **empty**, wait 2 seconds. The program zeroes itself (tare) and then prints:
   ```
   raw=84213  net=3  grams=3.00
   ```
   `net` should be close to 0 and only jitter slightly. If the program says *"No answer from HX711"*, check wiring.
3. **Place a known weight** (for example 500 g) and note the `net` value, say `250000`.
4. **Compute the scale**: `scale = net / known_weight_in_grams` = `250000 / 500` = `500.0`.
5. **Put it in `config.h`**:
   ```c
   #define LOADCELL_SCALE   500.0f
   ```
6. Rebuild and flash. `grams` now shows real grams. Check with a second, different weight.

Tips:
- The scale is re-zeroed at every boot, so start the robot with nothing on the load cell. Press the **EN** (reset) button to tare again.
- If `net` goes **negative** when you add weight, swap `A+` and `A-` or use a negative scale.
- Let the HX711 warm up for a minute for stable numbers.
- Noisy readings? Increase `LOADCELL_SAMPLES` in `config.h` (each sample takes 100 ms).

## Code conventions

- Plain C, no RTOS tasks or queues. One loop in `app.c`.
- Every function has a short comment saying what it does.
- Pins and magic numbers only in `config.h`.
- Print with `ESP_LOGI(TAG, ...)`, not `printf`.

Self note:
To open UDP socket via MobaXterm:
- Ctrl + Alt + T
- python3 -c "import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1); s.bind(('0.0.0.0',5000)); [print(s.recv(2048).decode(errors='replace'),end='',flush=True) for _ in iter(int,1)]"