---

# STM32F411 (BlackPill F411CE) — PlatformIO Starter

Minimal, **production-ready** PlatformIO template for STM32F411CE (“BlackPill”) boards.
Zero fluff — just code that builds and flashes *now*.

---

## 🧩 Structure

| Path                    | Purpose                                                    |
| ----------------------- | ---------------------------------------------------------- |
| `platformio.ini`        | Two pre-configured environments (Arduino / STM32Cube HAL). |
| `src_arduino/main.cpp`  | Arduino sketch — LED blink + Serial.                       |
| `src_cube/main.c`       | STM32Cube HAL example — pure C, LED blink.                 |
| `include/`, `lib/`      | Placeholders for headers and static libs.                  |
| `.clang-format`         | Shared formatting rules for C / C++.                       |
| `.vscode/settings.json` | Editor config — enables autoformat on save.                |

---

## ⚙️ Build & Upload

```bash
# Arduino environment (default)
pio run -t upload -e f411ce_arduino

# STM32Cube HAL environment
pio run -t upload -e f411ce_cube

# Serial monitor
pio device monitor -b 115200
```

> 💡 Use `pio run -t clean` before switching between environments.

---

## 🔧 Environment Notes

### Arduino (`f411ce_arduino`)

* Framework: **Arduino Core for STM32**
* Serial: `Serial.begin(115200)`
* LED: `PC13` (active-low)
* USB CDC already enabled
* Upload via `stlink` or `dfu` (change in `platformio.ini`)

### STM32Cube (`f411ce_cube`)

* Framework: **STM32Cube HAL**
* Pure C entry point in `src_cube/main.c`
* Default PLL config assumes **25 MHz HSE**
* Adjust in `SystemClock_Config()` if your crystal is 8 MHz:

  ```ini
  build_flags = -D HSE_VALUE=8000000U
  ```
* Toggle LED on `PC13` every 500 ms

---

## 🧠 Developer Tips

* **Board ID:**
  If you use another STM32F4 variant — change

  ```ini
  board = blackpill_f411ce
  ```

  in `platformio.ini`.

* **Format your code:**
  Use `clang-format` before commit:

  ```bash
  clang-format -i --style=file src_arduino/*.cpp src_cube/*.c
  ```

  or autoformat on save in VS Code via `.vscode/settings.json`.

* **Hook (optional):**
  Add a pre-commit Git hook to reformat changed files automatically.

* **Monitor output:**

  ```
  pio device monitor -b 115200
  ```

  Press **Reset** to see boot messages.

---

## 🧰 Toolchain

| Tool                     | Purpose                  |
| ------------------------ | ------------------------ |
| PlatformIO               | Build / flash automation |
| OpenOCD or ST-Link       | Debug & upload           |
| Clang-Format             | Code style enforcement   |
| VS Code / PlatformIO IDE | Editing & monitoring     |

---

## 🚀 Summary

✅ Ready-to-build STM32F411 project
✅ Arduino + Cube in one repo, isolated
✅ Clean formatting & structure
✅ Instant upload with ST-Link or DFU
✅ Configurable clock / LED / CDC

