# ESP32 Sequence Game Refactor

This is the same sketch split into small modules so `src/main.cpp` stays small.

## Where files go

For PlatformIO:

```text
include/config.h
src/main.cpp
src/app/Settings.h
src/app/State.h
src/app/State.cpp
src/hardware/Pins.h
src/hardware/Pins.cpp
src/hardware/OledDisplay.h
src/hardware/OledDisplay.cpp
src/input/Buttons.h
src/input/Buttons.cpp
src/led/LedController.h
src/led/LedController.cpp
src/network/NetworkManager.h
src/network/NetworkManager.cpp
src/power/PowerManager.h
src/power/PowerManager.cpp
src/utils/Utils.h
src/utils/Utils.cpp
```

Copy your existing `config.h` into `include/config.h`, or copy `include/config.example.h` to `include/config.h` and fill in your WiFi/backend values.

## Module overview

- `main.cpp`: only setup/loop orchestration.
- `app/Settings.h`: compile-time switches and timing constants.
- `app/State.*`: shared global state used between modules/tasks.
- `hardware/Pins.*`: pin mappings and hardware constants.
- `hardware/OledDisplay.*`: OLED display and cross-task display requests.
- `input/Buttons.*`: button reads, debounce, socket-button request handling.
- `led/LedController.*`: LED pins, manual control, random sequence, backend sequence task.
- `network/NetworkManager.*`: WiFi, Socket.IO, JSON events, HTTP demo request.
- `power/PowerManager.*`: inactivity tracking and light sleep.
- `utils/Utils.*`: small shared helper functions.
"# esp32" 
