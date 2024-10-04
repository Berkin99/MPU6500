# MPU6500 Platform Independent C Driver

This project provides a platform-independent C driver for the **TDK InvenSense MPU6500** 6-axis accelerometer and gyroscope sensor. The driver supports SPI and I2C communication interfaces, along with functions for sensor calibration, data transformation, and configuration management.

## Features

- **Platform-Independent**: Works with any platform that supports C, allowing flexible integration.
- **Communication Interfaces**: Supports both SPI and I2C protocols.
- **Calibration**: Includes functionality for accelerometer and gyroscope calibration.
- **Raw and Processed Data Access**: Provides access to both raw sensor data and processed data (in Gs and degrees/second).
- **Configurable Settings**: Offers customizable settings for accelerometer and gyroscope ranges, DLPF (Digital Low-Pass Filter) bandwidth, and sample rate.

## Usage

### Device Initialization

To initialize the MPU6500, you need to create a device object using the `MPU6500_NewDevice()` function and then call `MPU6500_Init()` to configure and calibrate the sensor.

```c
#include "mpu6500.h"

// Example for initializing the MPU6500 device:
MPU6500_Device_t mpu6500_dev = MPU6500_NewDevice(spi_handle, MPU6500_INTF_SPI, read_func, write_func, delay_func);
int8_t init_status = MPU6500_Init(&mpu6500_dev);
```

### Setting Configuration

You can configure the device by setting the desired accelerometer and gyroscope ranges, DLPF bandwidth, and sample rate using `MPU6500_ApplySettings()`.

```c
MPU6500_Settings settings = {
    .acc_range = MPU6500_ACCEL_RANGE_4G,
    .gyr_range = MPU6500_GYRO_RANGE_500DPS,
    .dlpf = MPU6500_DLPF_BAND_WIDTH_41HZ,
    .srd = MPU6500_SAMPLE_RATE_DIVIDER_1000HZ,
    .clock_source = MPU6500_CLOCK_PLL_XGYRO
};

MPU6500_ApplySettings(&mpu6500_dev, settings);
```

### Reading Sensor Data

Once the device is configured, you can read accelerometer and gyroscope data with either raw values or processed values in Gs and degrees/second.

```c
int16_t acc[3], gyr[3];  // For raw data
float acc_g[3], gyr_dps[3];  // For processed data

// Raw data
MPU6500_GetDataRaw(&mpu6500_dev, acc, gyr);

// Processed data
MPU6500_GetData(&mpu6500_dev, acc_g, gyr_dps);
```

### Calibration

To calibrate the accelerometer or gyroscope, use the respective functions:

```c
MPU6500_AccCalibration(&mpu6500_dev, 5000);  // Calibrate for 5 seconds
MPU6500_GyrCalibration(&mpu6500_dev, 5000);  // Calibrate for 5 seconds
```

## API Reference

Detailed information about each function can be found in the [mpu6500.h](./mpu6500.h) file. Key functions include:

- `MPU6500_NewDevice()`: Create a new device object.
- `MPU6500_Init()`: Initialize and calibrate the device.
- `MPU6500_ApplySettings()`: Apply configuration settings.
- `MPU6500_GetData()`: Retrieve processed sensor data.
- `MPU6500_GetDataRaw()`: Retrieve raw sensor data.
- `MPU6500_AccCalibration()`: Calibrate the accelerometer.
- `MPU6500_GyrCalibration()`: Calibrate the gyroscope.

## Contributions

Bug reports and contributions are welcome. Please visit the [GitHub repository](https://github.com/Berkin99/MPU6500) to submit issues or pull requests.

## References

1. MPU6500 Datasheet: [PS-MPU-6500A-01-v1.3](./docs/PS-MPU-6500A-01-v1.3.pdf)
2. Register Map: [MPU-6500-Register-Map2](./docs/MPU-6500-Register-Map2.pdf)
