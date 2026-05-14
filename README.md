![Logo](https://i.goopics.net/1q71jm.png)

# 🚀 Avalon

Welcome to the **Avalon Project** GitHub page!

Avalon is an open-source autonomous UAV (Unmanned Aerial Vehicle) project started in **January 2023** by two electrical engineering students passionate about aerospace, embedded systems, and autonomous flight systems.

At first, we planned to build a rocket 🚀  
But since many people were already working on similar projects, we decided to take on a different challenge:

> ✈️ Designing and building a fully autonomous flying wing.

The objective of Avalon is to develop both the **hardware** and the **software** required for a low-cost, repairable, and self-flying UAV platform.

The project combines multiple engineering fields, including:

- Embedded systems
- Electronics design
- Aerodynamics
- Autonomous navigation
- Telemetry and communication
- Power management

Avalon is also a long-term learning project that allows us to experiment, improve our skills, and progressively build more advanced autonomous flight systems.

---

# 🧩 Control Board Philosophy

The Avalon control board was designed to be flexible and reusable across multiple types of autonomous vehicles.

The main objective is to support both:

- ✈️ **2-axis flying wings and aircraft**
- 🚀 **3-axis rockets and experimental vehicles**

By keeping the hardware architecture modular, the same board can be adapted to different flight configurations with only software and parameter changes.

---

# 🔌 Communication & Control Interface

The board exposes its internal configuration and control system through a **register-based interface** accessible over USB communication.

This approach allows a computer or ground station software to:

- 📊 Read real-time telemetry values
- ⚙️ Configure onboard parameters
- 🎮 Manually control the vehicle
- 💾 Upload autonomous flight settings
- 🧪 Debug and test peripherals
- 🚀 Enable autonomous flight modes

The communication system behaves similarly to a memory-mapped register interface commonly found in embedded systems.

Each register corresponds to a specific feature or data field, such as:

- REVISION_STATUS
- SYSTEM_STATUS
- PID_CONTROL
- COMMAND
- SENSOR_CONFIG
- DEBUG
- ACTUATOR_CONFIG
- FLIGHT_CONTROL
- ALERTS
- MEMORY_CONTROL

This architecture provides a simple and scalable way to interact with the firmware while remaining hardware-independent.

---

# 📌 Roadmap

> 🚧 This section will be completed later.

---

# 🧠 Project Architecture

The project is organized into multiple folders, each with a dedicated role:

```text
Avalon/
├── mx/            # STM32CubeMX generated peripheral configuration
├── lib/           # External libraries imported using git submodules
├── test/          # Sanity and validation tests executed before commits
├── platformio/    # PlatformIO extension configuration files
├── src/           # Main firmware source files
├── inc/           # Header files and firmware interfaces
```

## 📂 Folder Details

### `mx/`

Contains STM32 peripheral configuration generated using:

- 🧩 STM32CubeMX
- ⚙️ STM32CubeIDE

This folder is mainly used to generate initialization code for peripherals such as:

- UART
- SPI
- I2C
- Timers
- DMA
- GPIO
- etc.

---

### `lib/`

Contains external dependencies and libraries imported using **Git submodules**.

This allows the project to:

- keep dependencies separated,
- simplify updates,
- and maintain cleaner firmware code.

---

### `test/`

Contains all sanity tests and validation routines that should pass before committing changes.

These tests help ensure:

- firmware stability,
- hardware reliability,
- and regression prevention.

---

### `platformio/`

Contains the configuration files used by the **PlatformIO** extension.

PlatformIO is used for:

- project building,
- dependency management,
- flashing,
- and debugging.

---

### `src/` and `inc/`

Main firmware source and header files.

These folders contain:

- flight logic,
- drivers,
- telemetry,
- navigation,
- control algorithms,
- and all embedded application code.

---

# 🛠️ First Prototype Overview

The first prototype is based on a custom board using the:

- 🧠 STM32G474RET6 MCU

It integrates:

- 📌 BNO055 IMU
- 🌡️ BMP390 barometric pressure sensor
- 📍 L80 GPS module
- 📡 Wio-E5 telemetry module
- 🔋 Current and battery voltage monitoring
- ⚡ Additional onboard peripherals

For the aircraft itself, we decided to use a **3D printed flying wing** in order to focus first on hardware and software validation.

The selected model is:

> ✈️ **EBW-160** from Eclipson

The aircraft has a wingspan of **160 cm (5.2 ft)**, making it a fairly large platform.

## 🧱 Materials

### First prototype

- Printed using standard PLA
- Lower cost and easier to replace during testing

### Future prototype

- Planned in LW-PLA (Lightweight PLA)
- Reduced weight for improved flight performance

# 📸 Prototype V1

![PCB_V1_Front](https://github.com/Gabibou/Avalon-/assets/100377842/f3d56f3c-d216-4641-97ab-885c58d7cbd2)

![PCB_V1_Back](https://github.com/Gabibou/Avalon-/assets/100377842/ae4605fd-2f81-4c70-bdc7-ab92b161ce24)

![PCB_V1_Front](https://github.com/Gabibou/Avalon-/assets/100377842/4c123e75-89c8-42f4-b1b9-96b7fef8e4b2)

---

# ✈️ Second Prototype Overview

![3](https://github.com/Gabibou/Avalon-/assets/100377842/3b982855-6625-4e5e-af99-b0ac4b951086)

![2](https://github.com/Gabibou/Avalon-/assets/100377842/15719c67-abf8-411c-8773-0dc2c518725d)

![4](https://github.com/Gabibou/Avalon-/assets/100377842/bdb9e4fc-5298-45ac-9387-bad0bebaa782)

![5](https://github.com/Gabibou/Avalon-/assets/100377842/781caa51-0c11-4fb7-86e4-799b0d28d572)

![6](https://github.com/Gabibou/Avalon-/assets/100377842/2cb2b9c9-7d9d-47dc-99a5-98edd1a6b5de)

---

# 📦 Cloning the Project

This project uses **Git submodules** for external libraries.

Because of this, cloning the repository normally is **not enough** ❌

You must clone the repository recursively to automatically download all dependencies.

## ✅ Recommended Clone Command

```bash
git clone --recurse-submodules https://github.com/Gabibou/Avalon-.git
```

---

## 🔄 Updating Submodules

If you already cloned the repository without submodules, run:

```bash
git submodule update --init --recursive
```

To update all submodules later:

```bash
git submodule update --remote --recursive
```

---

# 📜 License

This project is distributed under the MIT License.

📄 [MIT License](https://choosealicense.com/licenses/mit/)

---

# 🙌 Acknowledgements

Special thanks to:

- 🌱 [Seeed Studio](https://www.seeedstudio.com/) for their partnership and support.
