/**
  ******************************************************************************
  * @file    spi.c
  * @author  Eldar Sabanovic
  * @version V1.0.0
  * @date    2016-12-28
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "cmsis_os.h"
#include <stdbool.h>

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private Constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
osSemaphoreId SPI_1_RX_CompleteID;
osSemaphoreId SPI_1_TX_CompleteID;
osSemaphoreId SPI_2_RX_CompleteID;
osSemaphoreId SPI_2_TX_CompleteID;
osSemaphoreId SPI_4_RX_CompleteID;
osSemaphoreId SPI_4_TX_CompleteID;
osSemaphoreDef(SPI_1_RX_Complete);
osSemaphoreDef(SPI_1_TX_Complete);
osSemaphoreDef(SPI_2_RX_Complete);
osSemaphoreDef(SPI_2_TX_Complete);
osSemaphoreDef(SPI_4_RX_Complete);
osSemaphoreDef(SPI_4_TX_Complete);

spi_select_list_t SPI1_Select_List[SPI1_LAST] =
{
//   Port,   Pin
	{GPIOA,  GPIO_PIN_8},  //!< SPI1_LPS25HB
};

spi_select_list_t SPI2_Select_List[SPI2_LAST] =
{
//   Port,   Pin
	{GPIOB,  GPIO_PIN_12}, //!< SPI2_MPU9250
};

spi_select_list_t SPI4_Select_List[SPI4_LAST] =
{
//   Port,   Pin
	{GPIOE,  GPIO_PIN_1},  //!< SPI4_M25P16
	{GPIOE,  GPIO_PIN_4}   //!< SPI4_RFM22B
};

void BSP_SPI1_Select(SPI1_devices_t device)
{
    BSP_SPI1_Deselect();
    HAL_GPIO_WritePin(SPI1_Select_List[device].port, SPI1_Select_List[device].pin, GPIO_PIN_RESET);
}

void BSP_SPI1_Deselect(void)
{
    uint8_t i;
    for(i = 0; i < SPI1_LAST; i++)
    {
        HAL_GPIO_WritePin(SPI1_Select_List[i].port, SPI1_Select_List[i].pin, GPIO_PIN_SET);
    }
}

void BSP_SPI2_Select(SPI2_devices_t device)
{
    BSP_SPI2_Deselect();
    HAL_GPIO_WritePin(SPI2_Select_List[device].port, SPI2_Select_List[device].pin, GPIO_PIN_RESET);
}

void BSP_SPI2_Deselect(void)
{
    uint8_t i;
    for(i = 0; i < SPI2_LAST; i++)
    {
        HAL_GPIO_WritePin(SPI2_Select_List[i].port, SPI2_Select_List[i].pin, GPIO_PIN_SET);
    }
}

void BSP_SPI4_Select(SPI4_devices_t device)
{
    BSP_SPI4_Deselect();
    HAL_GPIO_WritePin(SPI4_Select_List[device].port, SPI4_Select_List[device].pin, GPIO_PIN_RESET);
}

void BSP_SPI4_Deselect(void)
{
    uint8_t i;
    for(i = 0; i < SPI4_LAST; i++)
    {
        HAL_GPIO_WritePin(SPI4_Select_List[i].port, SPI4_Select_List[i].pin, GPIO_PIN_SET);
    }
}

void BSP_SPI_Init(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        SPI_1_RX_CompleteID = osSemaphoreCreate(osSemaphore(SPI_1_RX_Complete), 1);
        SPI_1_TX_CompleteID = osSemaphoreCreate(osSemaphore(SPI_1_TX_Complete), 1);
        osSemaphoreWait(SPI_1_RX_CompleteID, 0);
        osSemaphoreWait(SPI_1_TX_CompleteID, 0);
    }
    if (hspi->Instance == SPI2)
    {
        SPI_2_RX_CompleteID = osSemaphoreCreate(osSemaphore(SPI_2_RX_Complete), 1);
        SPI_2_TX_CompleteID = osSemaphoreCreate(osSemaphore(SPI_2_TX_Complete), 1);
        osSemaphoreWait(SPI_2_RX_CompleteID, 0);
        osSemaphoreWait(SPI_2_TX_CompleteID, 0);
    }
    if (hspi->Instance == SPI4)
    {
        SPI_4_RX_CompleteID = osSemaphoreCreate(osSemaphore(SPI_4_RX_Complete), 1);
        SPI_4_TX_CompleteID = osSemaphoreCreate(osSemaphore(SPI_4_TX_Complete), 1);
        osSemaphoreWait(SPI_4_RX_CompleteID, 0);
        osSemaphoreWait(SPI_4_TX_CompleteID, 0);
    }
}

void BSP_SPI_Denit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        osSemaphoreDelete(SPI_1_RX_CompleteID);
        osSemaphoreDelete(SPI_1_TX_CompleteID);
    }
    if (hspi->Instance == SPI2)
    {
        osSemaphoreDelete(SPI_2_RX_CompleteID);
        osSemaphoreDelete(SPI_2_TX_CompleteID);
    }
    if (hspi->Instance == SPI4)
    {
        osSemaphoreDelete(SPI_4_RX_CompleteID);
        osSemaphoreDelete(SPI_4_TX_CompleteID);
    }
}

bool BSP_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *tx_buf, uint16_t tx_size, uint32_t timeout)
{
    if (hspi->Instance == SPI1)
    {
        if (HAL_SPI_Transmit_DMA(hspi, tx_buf, tx_size) == HAL_OK)
        {
            if (osSemaphoreWait(SPI_1_TX_CompleteID, timeout) == osOK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }
    else if (hspi->Instance == SPI2)
    {
        if (HAL_SPI_Transmit_DMA(hspi, tx_buf, tx_size) == HAL_OK)
        {
            if (osSemaphoreWait(SPI_2_TX_CompleteID, timeout) == osOK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }
    else if (hspi->Instance == SPI4)
    {
        if (HAL_SPI_Transmit_DMA(hspi, tx_buf, tx_size) == HAL_OK)
        {
            if (osSemaphoreWait(SPI_4_TX_CompleteID, timeout) == osOK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }
    return false;
}

bool BSP_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *rx_buf, uint16_t rx_size, uint32_t timeout)
{
    if (hspi->Instance == SPI1)
    {
        if (HAL_SPI_Receive_DMA(hspi, rx_buf, rx_size) == HAL_OK)
        {
            if (osSemaphoreWait(SPI_1_RX_CompleteID, timeout) == osOK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else return false;
    }
    else if (hspi->Instance == SPI2)
    {
        if (HAL_SPI_Receive_DMA(hspi, rx_buf, rx_size) == HAL_OK)
        {
            if (osSemaphoreWait(SPI_2_RX_CompleteID, timeout) == osOK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else return false;
    }
    else if (hspi->Instance == SPI4)
    {
        if (HAL_SPI_Receive_DMA(hspi, rx_buf, rx_size) == HAL_OK)
        {
            if (osSemaphoreWait(SPI_4_RX_CompleteID, timeout) == osOK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else return false;
    }
    else
    {
        return false;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        osSemaphoreRelease(SPI_1_TX_CompleteID);
    }
    if (hspi->Instance == SPI2)
    {
        osSemaphoreRelease(SPI_2_TX_CompleteID);
    }
    if (hspi->Instance == SPI4)
    {
        osSemaphoreRelease(SPI_4_TX_CompleteID);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        osSemaphoreRelease(SPI_1_RX_CompleteID);
    }
    if (hspi->Instance == SPI2)
    {
        osSemaphoreRelease(SPI_2_RX_CompleteID);
    }
    if (hspi->Instance == SPI4)
    {
        osSemaphoreRelease(SPI_4_RX_CompleteID);
    }
}

/************************ (C) COPYRIGHT VGTU *****END OF FILE****/
