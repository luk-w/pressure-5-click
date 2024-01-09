/*
 * bmp3_aurix.c
 *
 *  Created on: 08.01.2024
 *      Author: Administrator
 */

#include "IfxStm.h"
#include "bmp3_aurix.h"
#include "SPI.h"

struct bmp3_settings settings = {
    .op_mode = 0,
    .press_en = BMP3_ENABLE,
    .temp_en = BMP3_ENABLE,
    .odr_filter = {
        .press_os = BMP3_OVERSAMPLING_2X,
        .temp_os = BMP3_OVERSAMPLING_2X,
        .iir_filter = 0,
        .odr = BMP3_ODR_100_HZ},
    .int_settings = {/* Register 0x19 settings */
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
static BMP3_INTF_RET_TYPE read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;

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
 *
 */
static BMP3_INTF_RET_TYPE write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;

    spi_write(reg_addr, reg_data, len);

    return BMP3_INTF_RET_SUCCESS;
}

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
static void wait_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, 1000));
}

struct bmp3_dev bmp3_dev_structure = {
        .read = read,
        .write = write,
        .delay_us = wait_us,
        .dummy_byte = 1u;
};

void bmp3_aurix_init()
{
    // Your initialization code here
}
