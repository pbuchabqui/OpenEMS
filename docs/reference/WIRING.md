# ECU P4 Pro-Spec - Wiring Diagram

## Overview

This document describes the wiring connections for the ECU P4 Pro-Spec system, including both the ESP32-P4 (Engine Control) and ESP32-C6 (Supervision) modules.

## ESP32-P4 (Engine Control) Wiring

### Power Supply
- **12V Input**: Connect to vehicle battery positive
- **GND**: Connect to vehicle chassis ground

### Sensor Connections

#### MAP Sensor (Manifold Absolute Pressure)
- **Signal**: GPIO 32
- **VCC**: 5V
- **GND**: Ground

#### CLT Sensor (Coolant Temperature)
- **Signal**: GPIO 25
- **VCC**: 5V
- **GND**: Ground

#### TPS Sensor (Throttle Position)
- **Signal**: GPIO 33
- **VCC**: 5V
- **GND**: Ground

#### IAT Sensor (Intake Air Temperature)
- **Signal**: GPIO 26
- **VCC**: 5V
- **GND**: Ground

#### O2 Sensor (Oxygen)
- **Signal**: GPIO 27
- **VCC**: 5V
- **GND**: Ground

#### Battery Voltage
- **Signal**: GPIO 14
- **VCC**: 5V
- **GND**: Ground

#### CKP Sensor (Crankshaft Position)
- **Signal**: GPIO 34
- **VCC**: 5V
- **GND**: Ground

#### CMP Sensor (Camshaft Position)
- **Signal**: GPIO 35
- **VCC**: 5V
- **GND**: Ground

### Actuator Connections

#### Fuel Injectors (4 channels)
- **Channel 1**: GPIO 12
- **Channel 2**: GPIO 13
- **Channel 3**: GPIO 15
- **Channel 4**: GPIO 2

#### Ignition Coils (4 channels)
- **Channel 1**: GPIO 4
- **Channel 2**: GPIO 16
- **Channel 3**: GPIO 17
- **Channel 4**: GPIO 5

### Communication Interfaces

#### SPI (Inter-MCU Communication)
- **MISO**: GPIO 19
- **MOSI**: GPIO 18
- **SCLK**: GPIO 23
- **CS**: GPIO 22

#### CAN Bus
- **RX**: GPIO 36
- **TX**: GPIO 39

## ESP32-C6 (Supervision) Wiring

### Power Supply
- **5V Input**: Connect to regulated 5V supply
- **GND**: Connect to ground

### Communication Interfaces

#### CAN Bus
- **RX**: GPIO 36
- **TX**: GPIO 39

#### USB
- **USB D+**: Internal
- **USB D-**: Internal

#### WiFi/Bluetooth
- **Antenna**: Internal

## Pin Configuration Summary

### ESP32-P4 Pinout

| GPIO | Function | Description |
|------|----------|-------------|
| 2 | Injector 4 | Fuel injector control |
| 4 | Ignition 1 | Ignition coil control |
| 5 | Ignition 4 | Ignition coil control |
| 12 | Injector 1 | Fuel injector control |
| 13 | Injector 2 | Fuel injector control |
| 14 | Battery Voltage | Battery monitoring |
| 15 | Injector 3 | Fuel injector control |
| 16 | Ignition 2 | Ignition coil control |
| 17 | Ignition 3 | Ignition coil control |
| 18 | SPI MOSI | Inter-MCU communication |
| 19 | SPI MISO | Inter-MCU communication |
| 22 | SPI CS | Inter-MCU communication |
| 23 | SPI SCLK | Inter-MCU communication |
| 25 | CLT Sensor | Coolant temperature |
| 26 | IAT Sensor | Intake air temperature |
| 27 | O2 Sensor | Oxygen sensor |
| 32 | MAP Sensor | Manifold pressure |
| 33 | TPS Sensor | Throttle position |
| 34 | CKP Sensor | Crankshaft position |
| 35 | CMP Sensor | Camshaft position |
| 36 | CAN RX | CAN bus receive |
| 39 | CAN TX | CAN bus transmit |

### ESP32-C6 Pinout

| GPIO | Function | Description |
|------|----------|-------------|
| 36 | CAN RX | CAN bus receive |
| 39 | CAN TX | CAN bus transmit |

## Wiring Guidelines

1. **Signal Integrity**: Use twisted pair wires for sensor signals to reduce noise.
2. **Power Supply**: Ensure stable power supply with proper filtering.
3. **Grounding**: Use star grounding configuration to minimize ground loops.
4. **Shielded Cables**: Use shielded cables for high-frequency signals.
5. **Fuse Protection**: Add appropriate fuses for power inputs.
6. **EMI Protection**: Add ferrite beads on power lines near the ECU.

## Connector Types

- **Sensors**: 3-pin connectors (VCC, Signal, GND)
- **Injectors**: 2-pin connectors
- **Ignition Coils**: 2-pin connectors
- **CAN Bus**: 4-pin connectors (CAN_H, CAN_L, VCC, GND)
- **Power**: 2-pin connectors (12V, GND)

## Testing Procedures

1. **Continuity Testing**: Verify all connections are properly made.
2. **Voltage Testing**: Check power supply voltages.
3. **Signal Testing**: Verify sensor signals with oscilloscope.
4. **Communication Testing**: Test CAN bus communication.
5. **Functional Testing**: Verify all actuators respond correctly.

## Troubleshooting

### Common Issues
- **No Power**: Check power supply and fuses
- **Sensor Errors**: Verify sensor connections and calibration
- **Communication Failures**: Check CAN bus termination and wiring
- **Actuator Issues**: Verify control signals and power supply

### Diagnostic Steps
1. Check power supply voltages
2. Verify ground connections
3. Test sensor signals
4. Check communication protocols
5. Verify actuator control signals

## Safety Considerations

- **High Voltage**: Exercise caution when working with vehicle electrical systems
- **Fuel System**: Ensure proper safety measures when working with fuel injectors
- **Ignition System**: High voltage present in ignition circuits
- **ESD Protection**: Use proper ESD protection when handling components

## Maintenance

- **Regular Inspection**: Check wiring connections periodically
- **Connector Cleaning**: Clean connectors to prevent corrosion
- **Signal Verification**: Verify sensor signals during maintenance
- **Firmware Updates**: Keep firmware updated for optimal performance