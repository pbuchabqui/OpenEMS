# ECU P4 Pro-Spec - Technical Specification

## System Architecture

### Dual-Processor Design
- **Main Processor**: ESP32-P4
  - Engine control and management
  - Real-time processing
  - Sensor interfacing
- **Supervision Processor**: ESP32-C6
  - Connectivity management
  - System monitoring
  - User interface

## Hardware Specifications

### ESP32-P4 (Engine Control)
- **Processor**: ESP32-P4 dual-core
- **Clock Speed**: Up to 240MHz
- **Memory**: 8MB PSRAM, 16MB Flash
- **ADC**: 12-bit, 8 channels
- **PWM**: 4 channels (MCPWM)
- **RMT**: 4 channels
- **SPI**: 1 interface
- **CAN**: 1 interface

### ESP32-C6 (Supervision)
- **Processor**: ESP32-C6 single-core
- **Clock Speed**: Up to 160MHz
- **Memory**: 4MB PSRAM, 8MB Flash
- **Connectivity**: WiFi, Bluetooth, USB
- **CAN**: 1 interface

## Software Architecture

### Real-Time Operating System
- **RTOS**: FreeRTOS
- **Task Priorities**:
  - Control Loop: Priority 10
  - Sensor Reading: Priority 9
  - Communication: Priority 8
  - System Monitoring: Priority 7

### Communication Protocols
- **CAN Bus**: 500kbps
- **SPI**: 1Mbps
- **USB CDC**: 115200 baud
- **WiFi**: 802.11 b/g/n
- **Bluetooth**: BLE 5.0

## Engine Control Parameters

### Fuel Injection
- **Injector Type**: Port fuel injection
- **Flow Rate**: 240cc/min per injector
- **Pulse Width Range**: 500-20000 microseconds
- **Control Method**: Speed-density mapping with 16x16 fuel map

### Ignition Timing
- **Base Advance**: 10 degrees
- **Maximum Advance**: 35 degrees
- **Minimum Advance**: 5 degrees
- **Control Method**: Load-based mapping with 16x16 ignition map

### Sensor Configuration
- **MAP Sensor**: 0-250 kPa
- **CLT Sensor**: -40 to 120°C
- **TPS Sensor**: 0-100%
- **IAT Sensor**: -40 to 120°C
- **O2 Sensor**: 0-1V
- **Battery Voltage**: 8-16V

## System States

### Normal Operation
- Engine running within parameters
- All sensors functioning
- Communication active

### Limp Mode
- Triggered by heartbeat loss
- Reduced performance
- Basic functionality maintained

### Error States
- Over-rev protection
- Overheating protection
- Sensor failure detection
- Communication failure

## Performance Specifications

### Engine Control Loop
- **Update Rate**: 1kHz
- **Latency**: <1ms
- **Accuracy**: ±2% fuel delivery
- **Timing Precision**: ±0.5 degrees

### Sensor Reading
- **Update Rate**: 100Hz
- **Resolution**: 12-bit
- **Conversion Time**: <10ms

### Communication
- **CAN Latency**: <5ms
- **SPI Throughput**: 1Mbps
- **USB Latency**: <10ms

## Safety Features

### Protection Systems
- **RPM Limiter**: 8000 RPM
- **Temperature Protection**: 120°C
- **Fuel Cutoff**: 7500 RPM
- **Battery Protection**: 8-16V range

### Error Detection
- **Sensor Validation**: Range checking
- **Communication Monitoring**: Heartbeat system
- **System Health**: Watchdog timers

## Development Environment

### Build System
- **CMake**: Version 3.16+
- **ESP-IDF**: Version 4.4+
- **Toolchain**: ESP32

### Debugging
- **Serial Output**: 115200 baud
- **Debug Mode**: Configurable
- **Error Logging**: Real-time

## Testing Procedures

### Unit Testing
- **Sensor Calibration**: Individual sensor testing
- **Control Algorithms**: Fuel and ignition mapping
- **Communication Protocols**: CAN and SPI testing

### Integration Testing
- **System Startup**: Complete initialization sequence
- **Real-time Performance**: Control loop timing
- **Error Handling**: Protection system activation

## Compliance

### Automotive Standards
- **EMI/EMC**: Automotive grade
- **Temperature Range**: -40 to 85°C
- **Vibration**: Automotive specifications
- **Power Supply**: 12V automotive system

## Future Enhancements

### Planned Features
- **Advanced Diagnostics**: Real-time monitoring
- **Data Logging**: SD card support
- **Remote Tuning**: WiFi-based configuration
- **Performance Optimization**: Adaptive control algorithms

### Hardware Upgrades
- **Additional Sensors**: Wideband O2, knock detection
- **Enhanced Connectivity**: Cellular modem support
- **Power Management**: Battery monitoring and control