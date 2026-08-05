#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BMI088_OK = 0,
    BMI088_INVALID_ARGUMENT,
    BMI088_NOT_INITIALIZED,
    BMI088_COMMUNICATION_ERROR,
    BMI088_REGISTER_VERIFY_ERROR,
    BMI088_ACCEL_ID_MISMATCH,
    BMI088_GYRO_ID_MISMATCH,
    BMI088_SENSOR_ERROR,
    BMI088_DATA_INVALID
} bmi088_status_t;

typedef struct
{
    uint8_t accel_id;
    uint8_t gyro_id;
} bmi088_chip_id_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} bmi088_vec3i16_t;

typedef struct
{
    float x;
    float y;
    float z;
} bmi088_vec3f_t;

typedef struct
{
    bmi088_vec3i16_t accel;
    bmi088_vec3i16_t gyro;
    int16_t temperature;
    uint32_t sensor_time;
} bmi088_raw_data_t;

typedef struct
{
    bmi088_vec3f_t accel_mps2;
    bmi088_vec3f_t gyro_rads;
    float temperature_c;
    uint32_t sensor_time_ticks;
    float sensor_time_s;
} bmi088_data_t;

/*
 * Initialize and configure the BMI088 for:
 *   accelerometer: +/-3 g, 800 Hz, normal filter, INT1 data-ready
 *   gyroscope:     +/-2000 dps, 1000 Hz/116 Hz BW, INT3 data-ready
 */
bmi088_status_t bmi088_init(bmi088_chip_id_t *chip_id);

/* Probe both dies. This function can be used before bmi088_init(). */
bmi088_status_t bmi088_probe(bmi088_chip_id_t *chip_id);

/* Reset both dies. Call bmi088_init() again before reading data. */
bmi088_status_t bmi088_soft_reset(void);

bool bmi088_is_initialized(void);

/* Read all useful sensor channels and convert them to SI units. */
bmi088_status_t bmi088_read(bmi088_data_t *data);

/* Read all useful sensor channels without unit conversion. */
bmi088_status_t bmi088_read_raw(bmi088_raw_data_t *data);

bmi088_status_t bmi088_read_accel_raw(bmi088_vec3i16_t *accel);
bmi088_status_t bmi088_read_gyro_raw(bmi088_vec3i16_t *gyro);
bmi088_status_t bmi088_read_temperature(float *temperature_c);
bmi088_status_t bmi088_read_sensor_time(uint32_t *sensor_time_ticks);

/* Read the data-ready flags from both sensor dies. */
bmi088_status_t bmi088_get_data_ready(bool *accel_ready,
                                      bool *gyro_ready);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
