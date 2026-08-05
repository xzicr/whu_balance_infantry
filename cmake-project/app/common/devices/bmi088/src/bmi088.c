#include "bmi088.h"

#include <stddef.h>
#include <string.h>

#include "bmi088_reg.h"
#include "bsp_bmi088.h"
#include "bsp_delay.h"

#define BMI088_SPI_TIMEOUT_MS                 10U
#define BMI088_POWER_ON_DELAY_MS              50U
#define BMI088_ACCEL_RESET_DELAY_MS            2U
#define BMI088_GYRO_RESET_DELAY_MS            35U
#define BMI088_ACCEL_SPI_SWITCH_DELAY_US      150U
#define BMI088_REGISTER_DELAY_US              150U
#define BMI088_MAX_BURST_BYTES                 16U

#define BMI088_STANDARD_GRAVITY_MPS2           9.80665f
#define BMI088_PI                              3.14159265358979323846f
#define BMI088_ACCEL_SCALE_MPS2_PER_LSB        \
    ((3.0f * BMI088_STANDARD_GRAVITY_MPS2) / 32768.0f)
#define BMI088_GYRO_SCALE_RADS_PER_LSB         \
    ((2000.0f * BMI088_PI / 180.0f) / 32768.0f)
#define BMI088_TEMPERATURE_SCALE_C_PER_LSB     0.125f
#define BMI088_TEMPERATURE_OFFSET_C            23.0f
#define BMI088_SENSOR_TIME_SECONDS_PER_TICK    0.0000390625f

typedef struct
{
    bsp_bmi088_device_t device;
    uint8_t register_address;
    uint8_t register_value;
    uint8_t verify_mask;
    uint32_t delay_us;
} bmi088_config_step_t;

static bool bmi088_initialized;

static const bmi088_config_step_t bmi088_accel_config[] =
{
    {BSP_BMI088_ACCEL, BMI088_ACCEL_PWR_CTRL_REG,
     BMI088_ACCEL_ENABLE, 0xFFU, 1000U},
    {BSP_BMI088_ACCEL, BMI088_ACCEL_PWR_CONF_REG,
     BMI088_ACCEL_PWR_ACTIVE, 0xFFU, 1000U},
    {BSP_BMI088_ACCEL, BMI088_ACCEL_CONFIG_REG,
     BMI088_ACCEL_CONFIG_800HZ_NORMAL, 0xFFU, 1000U},
    {BSP_BMI088_ACCEL, BMI088_ACCEL_RANGE_REG,
     BMI088_ACCEL_RANGE_3G, 0x03U, 1000U},
    {BSP_BMI088_ACCEL, BMI088_ACCEL_INT1_IO_CTRL_REG,
     BMI088_ACCEL_INT1_PUSH_PULL_LOW, 0x0EU, 150U},
    {BSP_BMI088_ACCEL, BMI088_ACCEL_INT_MAP_DATA_REG,
     BMI088_ACCEL_INT1_DATA_READY, 0x44U, 150U}
};

static const bmi088_config_step_t bmi088_gyro_config[] =
{
    {BSP_BMI088_GYRO, BMI088_GYRO_RANGE_REG,
     BMI088_GYRO_RANGE_2000DPS, 0x07U, 150U},
    {BSP_BMI088_GYRO, BMI088_GYRO_BANDWIDTH_REG,
     BMI088_GYRO_BANDWIDTH_1000_116HZ, 0x07U, 150U},
    {BSP_BMI088_GYRO, BMI088_GYRO_POWER_MODE_REG,
     BMI088_GYRO_POWER_NORMAL, 0xA0U, 150U},
    {BSP_BMI088_GYRO, BMI088_GYRO_INT_CTRL_REG,
     BMI088_GYRO_DATA_READY_ENABLE, 0xC0U, 150U},
    {BSP_BMI088_GYRO, BMI088_GYRO_INT3_INT4_IO_CONF_REG,
     BMI088_GYRO_INT3_PUSH_PULL_LOW, 0x0FU, 150U},
    {BSP_BMI088_GYRO, BMI088_GYRO_INT3_INT4_IO_MAP_REG,
     BMI088_GYRO_DATA_READY_TO_INT3, 0xA5U, 150U}
};

static bmi088_status_t bmi088_status_from_bsp(bsp_status_t status)
{
    return (status == BSP_OK) ? BMI088_OK : BMI088_COMMUNICATION_ERROR;
}

static int16_t bmi088_decode_le_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] |
                     ((uint16_t)data[1] << 8));
}

static bmi088_status_t bmi088_read_registers(
    bsp_bmi088_device_t device,
    uint8_t register_address,
    uint8_t *data,
    size_t length)
{
    uint8_t tx_data[BMI088_MAX_BURST_BYTES + 2U] = {0U};
    uint8_t rx_data[BMI088_MAX_BURST_BYTES + 2U] = {0U};
    size_t transfer_length;
    size_t data_offset;
    bsp_status_t bsp_status;

    if ((data == NULL) || (length == 0U) ||
        (length > BMI088_MAX_BURST_BYTES))
    {
        return BMI088_INVALID_ARGUMENT;
    }

    if (device == BSP_BMI088_ACCEL)
    {
        data_offset = 2U;
    }
    else if (device == BSP_BMI088_GYRO)
    {
        data_offset = 1U;
    }
    else
    {
        return BMI088_INVALID_ARGUMENT;
    }

    tx_data[0] = register_address | BMI088_SPI_READ_MASK;
    transfer_length = length + data_offset;

    bsp_status = bsp_bmi088_transfer(device,
                                     tx_data,
                                     rx_data,
                                     transfer_length,
                                     BMI088_SPI_TIMEOUT_MS);
    if (bsp_status != BSP_OK)
    {
        return bmi088_status_from_bsp(bsp_status);
    }

    memcpy(data, &rx_data[data_offset], length);
    return BMI088_OK;
}

static bmi088_status_t bmi088_write_register(
    bsp_bmi088_device_t device,
    uint8_t register_address,
    uint8_t register_value)
{
    const uint8_t tx_data[2] =
    {
        register_address & BMI088_SPI_WRITE_MASK,
        register_value
    };
    uint8_t rx_data[2] = {0U};
    bsp_status_t bsp_status;

    bsp_status = bsp_bmi088_transfer(device,
                                     tx_data,
                                     rx_data,
                                     sizeof(tx_data),
                                     BMI088_SPI_TIMEOUT_MS);
    return bmi088_status_from_bsp(bsp_status);
}

static bmi088_status_t bmi088_write_and_verify(
    const bmi088_config_step_t *step)
{
    bmi088_status_t status;
    uint8_t readback_value = 0U;

    if (step == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }

    status = bmi088_write_register(step->device,
                                   step->register_address,
                                   step->register_value);
    if (status != BMI088_OK)
    {
        return status;
    }

    bsp_delay_us(BMI088_REGISTER_DELAY_US);

    status = bmi088_read_registers(step->device,
                                   step->register_address,
                                   &readback_value,
                                   1U);
    if (status != BMI088_OK)
    {
        return status;
    }

    if ((readback_value & step->verify_mask) !=
        (step->register_value & step->verify_mask))
    {
        return BMI088_REGISTER_VERIFY_ERROR;
    }

    bsp_delay_us(step->delay_us);
    return BMI088_OK;
}

static bmi088_status_t bmi088_apply_config(
    const bmi088_config_step_t *config,
    size_t step_count)
{
    size_t index;
    bmi088_status_t status;

    if ((config == NULL) || (step_count == 0U))
    {
        return BMI088_INVALID_ARGUMENT;
    }

    for (index = 0U; index < step_count; ++index)
    {
        status = bmi088_write_and_verify(&config[index]);
        if (status != BMI088_OK)
        {
            return status;
        }
    }

    return BMI088_OK;
}

static bmi088_status_t bmi088_read_accel_chip_id(uint8_t *chip_id)
{
    bmi088_status_t status;

    if (chip_id == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }

    /* The first transaction switches the accelerometer from I2C to SPI. */
    status = bmi088_read_registers(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_CHIP_ID_REG,
                                   chip_id,
                                   1U);
    if (status != BMI088_OK)
    {
        return status;
    }

    bsp_delay_us(BMI088_ACCEL_SPI_SWITCH_DELAY_US);

    return bmi088_read_registers(BSP_BMI088_ACCEL,
                                 BMI088_ACCEL_CHIP_ID_REG,
                                 chip_id,
                                 1U);
}

static bmi088_status_t bmi088_read_gyro_chip_id(uint8_t *chip_id)
{
    return bmi088_read_registers(BSP_BMI088_GYRO,
                                 BMI088_GYRO_CHIP_ID_REG,
                                 chip_id,
                                 1U);
}

static bmi088_status_t bmi088_read_temperature_raw(int16_t *temperature)
{
    uint8_t raw_data[2];
    int16_t raw_value;
    bmi088_status_t status;

    if (temperature == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }

    status = bmi088_read_registers(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_TEMPERATURE_REG,
                                   raw_data,
                                   sizeof(raw_data));
    if (status != BMI088_OK)
    {
        return status;
    }

    if (raw_data[0] == 0x80U)
    {
        return BMI088_DATA_INVALID;
    }

    raw_value = (int16_t)(((uint16_t)raw_data[0] << 3) |
                          ((uint16_t)raw_data[1] >> 5));
    if ((raw_value & 0x0400) != 0)
    {
        raw_value -= 0x0800;
    }

    *temperature = raw_value;
    return BMI088_OK;
}

bmi088_status_t bmi088_probe(bmi088_chip_id_t *chip_id)
{
    bmi088_status_t status;

    if (chip_id == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }

    chip_id->accel_id = 0U;
    chip_id->gyro_id = 0U;

    status = bmi088_read_accel_chip_id(&chip_id->accel_id);
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_read_gyro_chip_id(&chip_id->gyro_id);
    if (status != BMI088_OK)
    {
        return status;
    }

    if (chip_id->accel_id != BMI088_ACCEL_CHIP_ID_EXPECTED)
    {
        return BMI088_ACCEL_ID_MISMATCH;
    }

    if (chip_id->gyro_id != BMI088_GYRO_CHIP_ID_EXPECTED)
    {
        return BMI088_GYRO_ID_MISMATCH;
    }

    return BMI088_OK;
}

bmi088_status_t bmi088_soft_reset(void)
{
    bmi088_status_t status;

    bmi088_initialized = false;

    status = bmi088_write_register(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_SOFTRESET_REG,
                                   BMI088_SOFTRESET_COMMAND);
    if (status != BMI088_OK)
    {
        return status;
    }
    bsp_delay_ms(BMI088_ACCEL_RESET_DELAY_MS);

    status = bmi088_write_register(BSP_BMI088_GYRO,
                                   BMI088_GYRO_SOFTRESET_REG,
                                   BMI088_SOFTRESET_COMMAND);
    if (status != BMI088_OK)
    {
        return status;
    }
    bsp_delay_ms(BMI088_GYRO_RESET_DELAY_MS);

    return BMI088_OK;
}

bmi088_status_t bmi088_init(bmi088_chip_id_t *chip_id)
{
    bmi088_status_t status;
    uint8_t accel_error = 0U;

    if (chip_id == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }

    bmi088_initialized = false;
    bsp_bmi088_init();
    bsp_delay_ms(BMI088_POWER_ON_DELAY_MS);

    status = bmi088_probe(chip_id);
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_soft_reset();
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_probe(chip_id);
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_apply_config(
        bmi088_accel_config,
        sizeof(bmi088_accel_config) / sizeof(bmi088_accel_config[0]));
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_apply_config(
        bmi088_gyro_config,
        sizeof(bmi088_gyro_config) / sizeof(bmi088_gyro_config[0]));
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_read_registers(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_ERROR_REG,
                                   &accel_error,
                                   1U);
    if (status != BMI088_OK)
    {
        return status;
    }
    if ((accel_error & BMI088_ACCEL_ERROR_MASK) != 0U)
    {
        return BMI088_SENSOR_ERROR;
    }

    bmi088_initialized = true;
    return BMI088_OK;
}

bool bmi088_is_initialized(void)
{
    return bmi088_initialized;
}

bmi088_status_t bmi088_read_accel_raw(bmi088_vec3i16_t *accel)
{
    uint8_t raw_data[6];
    bmi088_status_t status;

    if (accel == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }
    if (!bmi088_initialized)
    {
        return BMI088_NOT_INITIALIZED;
    }

    status = bmi088_read_registers(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_DATA_REG,
                                   raw_data,
                                   sizeof(raw_data));
    if (status != BMI088_OK)
    {
        return status;
    }

    accel->x = bmi088_decode_le_i16(&raw_data[0]);
    accel->y = bmi088_decode_le_i16(&raw_data[2]);
    accel->z = bmi088_decode_le_i16(&raw_data[4]);
    return BMI088_OK;
}

bmi088_status_t bmi088_read_gyro_raw(bmi088_vec3i16_t *gyro)
{
    uint8_t raw_data[6];
    bmi088_status_t status;

    if (gyro == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }
    if (!bmi088_initialized)
    {
        return BMI088_NOT_INITIALIZED;
    }

    status = bmi088_read_registers(BSP_BMI088_GYRO,
                                   BMI088_GYRO_DATA_REG,
                                   raw_data,
                                   sizeof(raw_data));
    if (status != BMI088_OK)
    {
        return status;
    }

    gyro->x = bmi088_decode_le_i16(&raw_data[0]);
    gyro->y = bmi088_decode_le_i16(&raw_data[2]);
    gyro->z = bmi088_decode_le_i16(&raw_data[4]);
    return BMI088_OK;
}

bmi088_status_t bmi088_read_sensor_time(uint32_t *sensor_time_ticks)
{
    uint8_t raw_data[3];
    bmi088_status_t status;

    if (sensor_time_ticks == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }
    if (!bmi088_initialized)
    {
        return BMI088_NOT_INITIALIZED;
    }

    status = bmi088_read_registers(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_SENSOR_TIME_REG,
                                   raw_data,
                                   sizeof(raw_data));
    if (status != BMI088_OK)
    {
        return status;
    }

    *sensor_time_ticks = (uint32_t)raw_data[0] |
                         ((uint32_t)raw_data[1] << 8) |
                         ((uint32_t)raw_data[2] << 16);
    return BMI088_OK;
}

bmi088_status_t bmi088_read_temperature(float *temperature_c)
{
    int16_t temperature_raw;
    bmi088_status_t status;

    if (temperature_c == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }
    if (!bmi088_initialized)
    {
        return BMI088_NOT_INITIALIZED;
    }

    status = bmi088_read_temperature_raw(&temperature_raw);
    if (status != BMI088_OK)
    {
        return status;
    }

    *temperature_c = ((float)temperature_raw *
                      BMI088_TEMPERATURE_SCALE_C_PER_LSB) +
                     BMI088_TEMPERATURE_OFFSET_C;
    return BMI088_OK;
}

bmi088_status_t bmi088_get_data_ready(bool *accel_ready,
                                      bool *gyro_ready)
{
    uint8_t accel_status;
    uint8_t gyro_status;
    bmi088_status_t status;

    if ((accel_ready == NULL) || (gyro_ready == NULL))
    {
        return BMI088_INVALID_ARGUMENT;
    }
    if (!bmi088_initialized)
    {
        return BMI088_NOT_INITIALIZED;
    }

    status = bmi088_read_registers(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_STATUS_REG,
                                   &accel_status,
                                   1U);
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_read_registers(BSP_BMI088_GYRO,
                                   BMI088_GYRO_INT_STATUS_REG,
                                   &gyro_status,
                                   1U);
    if (status != BMI088_OK)
    {
        return status;
    }

    *accel_ready = ((accel_status & BMI088_ACCEL_DATA_READY_MASK) != 0U);
    *gyro_ready = ((gyro_status & BMI088_GYRO_DATA_READY_MASK) != 0U);
    return BMI088_OK;
}

bmi088_status_t bmi088_read_raw(bmi088_raw_data_t *data)
{
    uint8_t accel_and_time[9];
    uint8_t gyro_data[6];
    bmi088_status_t status;

    if (data == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }
    if (!bmi088_initialized)
    {
        return BMI088_NOT_INITIALIZED;
    }

    status = bmi088_read_registers(BSP_BMI088_ACCEL,
                                   BMI088_ACCEL_DATA_REG,
                                   accel_and_time,
                                   sizeof(accel_and_time));
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_read_registers(BSP_BMI088_GYRO,
                                   BMI088_GYRO_DATA_REG,
                                   gyro_data,
                                   sizeof(gyro_data));
    if (status != BMI088_OK)
    {
        return status;
    }

    status = bmi088_read_temperature_raw(&data->temperature);
    if (status != BMI088_OK)
    {
        return status;
    }

    data->accel.x = bmi088_decode_le_i16(&accel_and_time[0]);
    data->accel.y = bmi088_decode_le_i16(&accel_and_time[2]);
    data->accel.z = bmi088_decode_le_i16(&accel_and_time[4]);
    data->sensor_time = (uint32_t)accel_and_time[6] |
                        ((uint32_t)accel_and_time[7] << 8) |
                        ((uint32_t)accel_and_time[8] << 16);

    data->gyro.x = bmi088_decode_le_i16(&gyro_data[0]);
    data->gyro.y = bmi088_decode_le_i16(&gyro_data[2]);
    data->gyro.z = bmi088_decode_le_i16(&gyro_data[4]);
    return BMI088_OK;
}

bmi088_status_t bmi088_read(bmi088_data_t *data)
{
    bmi088_raw_data_t raw_data;
    bmi088_status_t status;

    if (data == NULL)
    {
        return BMI088_INVALID_ARGUMENT;
    }

    status = bmi088_read_raw(&raw_data);
    if (status != BMI088_OK)
    {
        return status;
    }

    data->accel_mps2.x = (float)raw_data.accel.x *
                         BMI088_ACCEL_SCALE_MPS2_PER_LSB;
    data->accel_mps2.y = (float)raw_data.accel.y *
                         BMI088_ACCEL_SCALE_MPS2_PER_LSB;
    data->accel_mps2.z = (float)raw_data.accel.z *
                         BMI088_ACCEL_SCALE_MPS2_PER_LSB;

    data->gyro_rads.x = (float)raw_data.gyro.x *
                        BMI088_GYRO_SCALE_RADS_PER_LSB;
    data->gyro_rads.y = (float)raw_data.gyro.y *
                        BMI088_GYRO_SCALE_RADS_PER_LSB;
    data->gyro_rads.z = (float)raw_data.gyro.z *
                        BMI088_GYRO_SCALE_RADS_PER_LSB;

    data->temperature_c = ((float)raw_data.temperature *
                           BMI088_TEMPERATURE_SCALE_C_PER_LSB) +
                          BMI088_TEMPERATURE_OFFSET_C;
    data->sensor_time_ticks = raw_data.sensor_time;
    data->sensor_time_s = (float)raw_data.sensor_time *
                          BMI088_SENSOR_TIME_SECONDS_PER_TICK;
    return BMI088_OK;
}
