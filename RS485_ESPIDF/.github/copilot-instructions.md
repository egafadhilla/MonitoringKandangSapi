# Copilot Instructions for MonitoringKandangSapi

This document provides guidance for AI coding agents working on the MonitoringKandangSapi project. It outlines the architecture, workflows, and conventions specific to this codebase.

## Project Overview
The MonitoringKandangSapi project is an ESP-IDF-based system for monitoring gas levels in a cattle barn. It uses RS485 communication to interface between master and slave devices. Each slave device reads sensor data (e.g., gas levels) and communicates it to the master.

### Key Components
1. **Master Program** (`rs485_Master`):
   - Manages communication with multiple slave devices over RS485.
   - Sends commands and receives sensor data.

2. **Slave Devices** (`slave_gas`, `slave_MQ_TGS`, etc.):
   - Each slave reads data from specific sensors (e.g., gas sensors like MICS6814).
   - Communicates with the master via RS485.

3. **Sensor Integration**:
   - Sensors are read using ADC (Analog-to-Digital Converter) peripherals.
   - Mutexes are used to protect shared data between tasks.

## Developer Workflows

### Building the Project
- Use `idf.py build` to compile the project.
- Ensure the correct ESP-IDF environment is set up before building.

### Flashing the Firmware
- Use `idf.py flash` to upload the firmware to the ESP32 device.
- Ensure the device is connected via USB and the correct port is specified.

### Monitoring Logs
- Use `idf.py monitor` to view runtime logs from the ESP32 device.
- Logs include sensor readings, RS485 communication events, and error messages.

### Debugging
- Use `idf.py monitor` for runtime debugging.
- Add `ESP_LOGI`, `ESP_LOGW`, or `ESP_LOGE` statements in the code for additional insights.

## Project-Specific Conventions

### RS485 Communication
- RS485 communication is half-duplex, requiring manual control of TX/RX mode using GPIO.
- Use `RS485_SetTX()` and `RS485_SetRX()` to switch modes.
- Data is sent using `RS485_Send()`.

### Sensor Data Handling
- Sensor readings are protected by a mutex (`sensor_data_mutex`) to ensure thread safety.
- ADC values are converted to meaningful data (e.g., gas concentration) using calibration formulas.

### Task Structure
- **`uart_event_task`**: Handles RS485 communication.
- **`sensor_read_task`**: Periodically reads sensor data and logs it.

## Key Files and Directories
- `main/main.c`: Contains the main logic for slave devices, including sensor reading and RS485 communication.
- `CMakeLists.txt`: Configuration files for building the project.
- `README.md`: Provides an overview of the project structure and usage.

## External Dependencies
- **ESP-IDF**: The official development framework for ESP32.
- **RS485**: Communication protocol used for master-slave communication.
- **Gas Sensors**: Includes MICS6814 and others, integrated via ADC.

## Examples
### Sending Data via RS485
```c
char response_buffer[100];
snprintf(response_buffer, sizeof(response_buffer), "{\"tgs2602\":%d,\"mq136\":%d}", tgs_val, mq_val);
RS485_Send(UART_NUM_2, (uint8_t*)response_buffer, strlen(response_buffer));
```

### Reading Sensor Data
```c
int co2 = 0;
esp_err_t co2_status = adc_oneshot_read(adc1_handle, MISC6814_CO2_CHANNEL, &co2);
if (co2_status == ESP_OK) {
    co2 = 4095 - co2; // Invert ADC value for gas concentration
}
```

## Notes for AI Agents
- Follow the ESP-IDF coding style and use designated initializers for structs.
- Ensure thread safety when accessing shared resources.
- Use `ESP_LOG` macros for logging and debugging.

For further details, refer to the ESP-IDF documentation: https://docs.espressif.com/projects/esp-idf/en/latest/
