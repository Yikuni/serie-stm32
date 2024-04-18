/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_ms5837_interface_template.c
 * @brief     driver ms5837 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-09-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/09/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_ms5837_interface.h"

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t ms5837_interface_iic_init(void)
{
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t ms5837_interface_iic_deinit(void)
{
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is iic device write address
 * @param[in]  reg is iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t ms5837_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
		HAL_I2C_Mem_Read(&hi2c1,addr|0x01,reg,1,buf,len,1000);
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr is iic device write address
 * @param[in] reg is iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ms5837_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
		HAL_I2C_Mem_Write(&hi2c1,addr,reg,1,buf,len,1000);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void ms5837_interface_delay_ms(uint32_t ms)
{

   for(uint32_t i = 0; i < ms; i++) {
        for(uint32_t j = 0; j < 72000; j++) {
            // 空循环
            __NOP();
        }
    }

}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void ms5837_interface_debug_print(const char *const fmt, ...)
{
	va_list args;
	char fmt2[200] = {""};
	char send_msg[200] = {""};
	strcpy(fmt2, fmt);
	strcat(fmt2, "\r\n");
	va_start(args, fmt);
	vsprintf(send_msg, fmt2, args);
	va_end(args);
	HAL_UART_Transmit(&huart1, (uint8_t*)send_msg, strlen(send_msg), 50);
}
