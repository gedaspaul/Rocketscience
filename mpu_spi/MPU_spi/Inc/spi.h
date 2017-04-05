/**
  ******************************************************************************
  * @file    spi.h
  * @author  Eldar Sabanovic
  * @version V1.0.0
  * @date    2016-03-26
  * @brief   SPI driver
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  TODO
  
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 VGTU</center></h2>
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32l1xx_hal.h"

typedef enum _SPI1_devices_t
{
    SPI1_LPS25HB,
    SPI1_H3LIS331DL,
    SPI1_LSM303D,
    SPI1_LAST
} SPI1_devices_t;     
     
typedef enum _SPI2_devices_t
{
    SPI2_MPU9250,
    SPI2_LAST
} SPI2_devices_t;     
     
typedef enum _SPI3_devices_t
{
    SPI3_L3G4200D,
    SPI3_LAST
} SPI3_devices_t;     
     
typedef enum _SPI4_devices_t
{
    SPI4_M25P16,
    SPI4_RFM22B,
    SPI4_LAST
} SPI4_devices_t;

typedef struct _bsp_gpio_list
{
    GPIO_TypeDef  *port;
    uint32_t      pin;
} spi_select_list_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
     
void BSP_SPI_Init(SPI_HandleTypeDef *hspi);
void BSP_SPI_Denit(SPI_HandleTypeDef *hspi);
void BSP_SPI1_Deselect(void);
void BSP_SPI1_Select(SPI1_devices_t device);
void BSP_SPI2_Deselect(void);
void BSP_SPI2_Select(SPI2_devices_t device);
void BSP_SPI3_Deselect(void);
void BSP_SPI3_Select(SPI3_devices_t device);
void BSP_SPI4_Deselect(void);
void BSP_SPI4_Select(SPI4_devices_t device);
bool BSP_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *tx_buf, uint16_t tx_size, uint32_t timeout);
bool BSP_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *rx_buf, uint16_t rx_size, uint32_t timeout);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
