#ifndef BMI088_REG_H
#define BMI088_REG_H

/* SPI protocol */
#define BMI088_SPI_READ_MASK                  0x80U
#define BMI088_SPI_WRITE_MASK                 0x7FU
#define BMI088_SPI_DUMMY_BYTE                 0x00U

/* Accelerometer registers */
#define BMI088_ACCEL_CHIP_ID_REG              0x00U
#define BMI088_ACCEL_CHIP_ID_EXPECTED         0x1EU
#define BMI088_ACCEL_ERROR_REG                0x02U
#define BMI088_ACCEL_ERROR_MASK               0x05U
#define BMI088_ACCEL_STATUS_REG               0x03U
#define BMI088_ACCEL_DATA_READY_MASK          0x80U
#define BMI088_ACCEL_DATA_REG                 0x12U
#define BMI088_ACCEL_SENSOR_TIME_REG          0x18U
#define BMI088_ACCEL_TEMPERATURE_REG          0x22U
#define BMI088_ACCEL_CONFIG_REG               0x40U
#define BMI088_ACCEL_CONFIG_800HZ_NORMAL      0xABU
#define BMI088_ACCEL_RANGE_REG                0x41U
#define BMI088_ACCEL_RANGE_3G                 0x00U
#define BMI088_ACCEL_INT1_IO_CTRL_REG         0x53U
#define BMI088_ACCEL_INT1_PUSH_PULL_LOW       0x08U
#define BMI088_ACCEL_INT_MAP_DATA_REG         0x58U
#define BMI088_ACCEL_INT1_DATA_READY          0x04U
#define BMI088_ACCEL_PWR_CONF_REG             0x7CU
#define BMI088_ACCEL_PWR_ACTIVE               0x00U
#define BMI088_ACCEL_PWR_CTRL_REG             0x7DU
#define BMI088_ACCEL_ENABLE                   0x04U
#define BMI088_ACCEL_SOFTRESET_REG            0x7EU

/* Gyroscope registers */
#define BMI088_GYRO_CHIP_ID_REG               0x00U
#define BMI088_GYRO_CHIP_ID_EXPECTED          0x0FU
#define BMI088_GYRO_DATA_REG                  0x02U
#define BMI088_GYRO_INT_STATUS_REG            0x0AU
#define BMI088_GYRO_DATA_READY_MASK           0x80U
#define BMI088_GYRO_RANGE_REG                 0x0FU
#define BMI088_GYRO_RANGE_2000DPS             0x00U
#define BMI088_GYRO_BANDWIDTH_REG             0x10U
#define BMI088_GYRO_BANDWIDTH_1000_116HZ      0x02U
#define BMI088_GYRO_POWER_MODE_REG            0x11U
#define BMI088_GYRO_POWER_NORMAL              0x00U
#define BMI088_GYRO_SOFTRESET_REG             0x14U
#define BMI088_GYRO_INT_CTRL_REG              0x15U
#define BMI088_GYRO_DATA_READY_ENABLE         0x80U
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG     0x16U
#define BMI088_GYRO_INT3_PUSH_PULL_LOW        0x00U
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG      0x18U
#define BMI088_GYRO_DATA_READY_TO_INT3        0x01U

#define BMI088_SOFTRESET_COMMAND              0xB6U

#endif /* BMI088_REG_H */
