#include "bsp_bmi088.h"

#include "main.h"
#include "spi.h"

static void bmi088_set_chip_select(
    bsp_bmi088_device_t device,
    GPIO_PinState state)
{
    switch (device)
    {
        case BSP_BMI088_ACCEL:
            HAL_GPIO_WritePin(
                CS1_ACCEL_GPIO_Port,
                CS1_ACCEL_Pin,
                state);
            break;

        case BSP_BMI088_GYRO:
            HAL_GPIO_WritePin(
                CS1_GYRO_GPIO_Port,
                CS1_GYRO_Pin,
                state);
            break;

        default:
            break;
    }
}

static bsp_status_t bmi088_convert_hal_status(
    HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
        case HAL_OK:
            return BSP_OK;

        case HAL_BUSY:
            return BSP_BUSY;

        case HAL_TIMEOUT:
            return BSP_TIMEOUT;

        case HAL_ERROR:
        default:
            return BSP_ERROR;
    }
}

void bsp_bmi088_init(void)
{
    bmi088_set_chip_select(
        BSP_BMI088_ACCEL,
        GPIO_PIN_SET);

    bmi088_set_chip_select(
        BSP_BMI088_GYRO,
        GPIO_PIN_SET);
}

bsp_status_t bsp_bmi088_transfer(
    bsp_bmi088_device_t device,
    const uint8_t *tx_data,
    uint8_t *rx_data,
    size_t length,
    uint32_t timeout_ms)
{
    if ((device != BSP_BMI088_ACCEL) &&
        (device != BSP_BMI088_GYRO))
    {
        return BSP_INVALID_ARGUMENT;
    }

    if ((tx_data == NULL) || (rx_data == NULL))
    {
        return BSP_INVALID_ARGUMENT;
    }

    if ((length == 0U) || (length > UINT16_MAX))
    {
        return BSP_INVALID_ARGUMENT;
    }

    if (timeout_ms == 0U)
    {
        return BSP_INVALID_ARGUMENT;
    }

    HAL_StatusTypeDef hal_status;

    bmi088_set_chip_select(
        device,
        GPIO_PIN_RESET);

    hal_status = HAL_SPI_TransmitReceive(
        &hspi1,
        (uint8_t *)tx_data,
        rx_data,
        (uint16_t)length,
        timeout_ms);

    bmi088_set_chip_select(
        device,
        GPIO_PIN_SET);

    return bmi088_convert_hal_status(hal_status);
}