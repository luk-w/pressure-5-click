/*
 * bmp3_aurix.c
 *
 * Created on: 08.01.2024
 * Author: Administrator
 */

#include "BSP.h"
#include "bmp3_aurix.h"
#include "SPI.h"
#include "UART.h"

/* BMP3 sensor settings */
struct bmp3_settings bmp3_settings_struct = {
    .op_mode = 0,
    .press_en = BMP3_ENABLE,
    .temp_en = BMP3_ENABLE,
    .odr_filter = {
        .press_os = BMP3_OVERSAMPLING_2X,
        .temp_os = BMP3_OVERSAMPLING_2X,
        .iir_filter = 0,
        .odr = BMP3_ODR_100_HZ},
    .int_settings = {
                     .output_mode = 0,
                     .level = BMP3_ENABLE,
                     .latch = 0,
                     .drdy_en = 0},
    .adv_settings = {.i2c_wdt_en = 0, .i2c_wdt_sel = 0}};

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
static BMP3_INTF_RET_TYPE read(uint8 reg_addr, uint8 *reg_data, uint32 len, void *intf_ptr)
{
    (void)intf_ptr;

    // Use the platform-specific SPI read function
    spi_read(reg_addr, reg_data, len);

    return BMP3_INTF_RET_SUCCESS;
}

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
static BMP3_INTF_RET_TYPE write(uint8 reg_addr, uint8 *reg_data, uint32 len, void *intf_ptr)
{
    (void)intf_ptr;

    // Use the platform-specific SPI write function
    spi_write(reg_addr, reg_data, len);

    return BMP3_INTF_RET_SUCCESS;
}

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related callbacks
 */
static void wait_us(uint32 period, void *intf_ptr)
{
    (void)intf_ptr;

    // Use the platform-specific delay function
    waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, period));
}

/* Interface pointer */
uint8 intf_ptr;

/* BMP3 device structure initialization */
struct bmp3_dev bmp3_dev_structure = {
    .read = read,
    .write = write,
    .delay_us = wait_us,
    .dummy_byte = 1u,
    .intf = BMP3_SPI_INTF,
    .intf_ptr = &intf_ptr,
};

static sint8 rslt;

/* BMP3 AURIX initialization */
void bmp3_aurix_init()
{
    // Initialize BMP3 device structure
    bmp3_init(&bmp3_dev_structure);

    // Desired BMP3 sensor settings
    uint32 desired_settings = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;

    // Set desired sensor settings
    rslt = bmp3_set_sensor_settings(desired_settings, &bmp3_settings_struct, &bmp3_dev_structure);

    // Set BMP3 operation mode to normal
    bmp3_settings_struct.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&bmp3_settings_struct, &bmp3_dev_structure);
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
}

struct bmp3_status bmp3_status_struct = {{0}};
struct bmp3_data sensor_data = {0};

/* BMP3 AURIX readout */
void bmp3_aurix_readout()
{
    rslt = bmp3_get_status(&bmp3_status_struct, &bmp3_dev_structure);

    if ((rslt == BMP3_OK) && (bmp3_status_struct.intr.drdy == BMP3_ENABLE))
    {
        // Read sensor data
        bmp3_get_sensor_data(BMP3_PRESS_TEMP, &sensor_data, &bmp3_dev_structure);
        rslt = bmp3_get_status(&bmp3_status_struct, &bmp3_dev_structure);
        waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));

        // Print sensor data
        uint8 msg[100];
        sprintf((char *)msg, "Pressure: %d Pa, Temperature: %d C\n", sensor_data.pressure, sensor_data.temperature);


        uart_sendMessage((uint8 *)msg, strlen((char *)msg));
        rslt = bmp3_get_status(&bmp3_status_struct, &bmp3_dev_structure);
    }
}
