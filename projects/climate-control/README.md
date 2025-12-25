# Climate Control System for K1921VK028

**Simple. Reliable. Like an AK-47!**

## Overview

Bare-metal C11 implementation of a room climate control system for the NIIET K1921VK028 microcontroller (ARM Cortex-M4F).

## Features

- **11 Temperature Sensors** via CAN bus (6 on CAN0, 5 on CAN1)
- **6 Heaters** + **4 Air Conditioners** control
- **UART Communication** with laptop (commands + 500ms status reports)
- **Simple Control Algorithm** - bang-bang control with 1°C hysteresis
- **DMA-based I/O** for efficiency
- **Interrupt-driven** architecture
- **Outlier filtering** for reliable temperature averaging

## System Architecture

### Hardware Layer
- CAN0/CAN1: Sensor and actuator communication (500kbps)
- UART0: Command reception from laptop (115200 baud)
- UART1: Status transmission to laptop (115200 baud)
- Timer: 500ms periodic tasks
- DMA: Efficient data transfers

### Software Layers
1. **HAL (Hardware Abstraction Layer)**
   - `can_hal.c`: CAN interface
   - `uart_hal.c`: UART with DMA
   - `timer_hal.c`: Periodic timer
   - `gpio_hal.c`: Status LEDs

2. **Protocol Handlers**
   - `can_protocol.c`: Sensor/actuator message encoding/decoding
   - `uart_protocol.c`: Binary protocol with checksums

3. **Device Managers**
   - `sensor_manager.c`: Temperature sensor tracking
   - Actuator control integrated in main loop

4. **Application Logic**
   - `main.c`: Main control loop with simple bang-bang control

## CAN Message IDs

### CAN0 Bus
| Device | ID Range | Description |
|--------|----------|-------------|
| Room Sensors | 0x100-0x103 | 4 room temperature sensors |
| Outdoor Sensors | 0x104-0x105 | 2 outdoor sensors |
| Heaters | 0x110-0x112 | 3 heater commands |
| ACs | 0x113-0x114 | 2 AC commands |

### CAN1 Bus
| Device | ID Range | Description |
|--------|----------|-------------|
| Room Sensors | 0x200-0x204 | 5 room temperature sensors |
| Heaters | 0x210-0x212 | 3 heater commands |
| ACs | 0x213-0x214 | 2 AC commands |

## UART Protocol

### Frame Format
```
[Header:2][Length:2][Type:1][Payload:N][Checksum:2]
```

- **Header**: 0xAA55
- **Length**: Payload length in bytes
- **Type**: Command/Status type
- **Checksum**: 16-bit sum of all bytes except checksum itself

### Command Types
- `0x01`: Set target temperature
- `0x02`: Set AUTO/MANUAL mode
- `0x03`: Manual heater control
- `0x04`: Manual AC control
- `0x05`: Request status

### Status Report (Type 0x80)
Sent every 500ms, includes:
- System uptime
- Average room temperature
- Target temperature
- All 11 sensor readings and online status
- All 6 heater states and power levels
- All 4 AC states and power levels
- Control mode
- Error flags

## Control Algorithm

**Simple Bang-Bang Control with 1°C Hysteresis:**

```c
if (temp < target - 1.0°C)
    Turn ON heaters at 80% power
    Turn OFF air conditioners
else if (temp > target + 1.0°C)
    Turn OFF heaters
    Turn ON air conditioners at 80% power
else
    Turn OFF everything (within acceptable range)
```

## Building

### Prerequisites
- GCC ARM Embedded Toolchain
- Make
- K1921VK SDK

### Build Commands
```bash
cd projects/climate-control
make clean
make all
```

### Flashing
```bash
make flash
```

## Memory Usage (Estimated)

- **RAM**: ~4 KB
  - Sensor data: ~220 bytes
  - UART buffers: ~768 bytes
  - Stack: ~2 KB
  - Other: ~1 KB

- **Flash**: ~20 KB
  - HAL layer: ~8 KB
  - Protocol handlers: ~4 KB
  - Application logic: ~6 KB
  - Initialization: ~2 KB

## File Structure

```
climate-control/
├── inc/                    # Header files
│   ├── common_types.h      # Common type definitions
│   ├── can_hal.h           # CAN HAL interface
│   ├── uart_hal.h          # UART HAL interface
│   ├── timer_hal.h         # Timer HAL interface
│   ├── gpio_hal.h          # GPIO HAL interface
│   ├── can_protocol.h      # CAN protocol handler
│   ├── uart_protocol.h     # UART protocol handler
│   └── sensor_manager.h    # Sensor manager
├── src/
│   ├── hal/                # Hardware abstraction layer
│   │   ├── can_hal.c
│   │   ├── uart_hal.c
│   │   ├── timer_hal.c
│   │   └── gpio_hal.c
│   ├── protocol/           # Protocol handlers
│   │   ├── can_protocol.c
│   │   └── uart_protocol.c
│   ├── managers/           # Device managers
│   │   └── sensor_manager.c
│   └── app/                # Application
│       └── main.c
├── Makefile
└── README.md
```

## Design Principles

1. **Simplicity**: Keep it simple, like an AK-47 - reliable and maintainable
2. **Modularity**: Clear separation of concerns with well-defined interfaces
3. **Reliability**: Sensor timeout detection, message validation, error handling
4. **Efficiency**: DMA for I/O, interrupt-driven architecture
5. **C11 Standard**: Modern C with static allocation, no malloc

## Status LEDs

- **LED_STATUS**: System running (ON after init)
- **LED_ERROR**: System error detected
- **LED_CAN0**: Toggles on CAN0 message received
- **LED_CAN1**: Toggles on CAN1 message received

## Error Handling

Error flags in system state:
- `ERR_SENSOR_TIMEOUT`: Sensor not responding
- `ERR_ACTUATOR_FAULT`: Actuator failure
- `ERR_CAN_BUS_OFF`: CAN bus off
- `ERR_UART_OVERFLOW`: UART buffer overflow
- `ERR_INVALID_COMMAND`: Invalid command received
- `ERR_TEMP_OUT_OF_RANGE`: Temperature reading out of valid range

## Future Enhancements

- PID control algorithm
- Data logging to external flash
- Web interface via Ethernet
- Predictive maintenance algorithms
- Advanced outlier filtering (IQR method)

## License

See project root LICENSE file.

## Author

Climate Control Team

---

**Built with the K1921VK SDK - Simple, Reliable, Effective!**
