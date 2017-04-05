/**
  ******************************************************************************
  * @file    MPU9250.h
  * @author  Eldar Sabanovic
  * @version V1.0.0
  * @date    2016-04-03
  * @brief   MPU9250 Serial Flash memory driver
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
#ifndef __MPU9250_H
#define __MPU9250_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "spi.h"
    
#define AK8963_ADDRESS_READ   (0x8C)  
#define AK8963_ADDRESS_WRITE  (0x0C)       
     
/* Exported types ------------------------------------------------------------*/
typedef enum _ak8963_reg_t
{	
    AK8963_REG_WIA                 = 0x00,
    AK8963_REG_INFO                = 0x01,
    AK8963_REG_ST1                 = 0x02,
    AK8963_REG_HXL                 = 0x03,
    AK8963_REG_HXH                 = 0x04,
    AK8963_REG_HYL                 = 0x05,
    AK8963_REG_HYH                 = 0x06,
    AK8963_REG_HZL                 = 0x07,
    AK8963_REG_HZH                 = 0x08,
    AK8963_REG_ST2                 = 0x09,
    AK8963_REG_CNTL1               = 0x0A,
    AK8963_REG_CNTL2               = 0x0B,
    AK8963_REG_ASTC                = 0x0C,
    AK8963_REG_TS1                 = 0x0D,
    AK8963_REG_TS2                 = 0x0E,
    AK8963_REG_I2CDIS              = 0x0F,
    AK8963_REG_ASAX                = 0x10,
    AK8963_REG_ASAY                = 0x11,
    AK8963_REG_ASAZ 			   = 0x12,
} ak8963_reg_t;		 
	 
typedef enum _mpu9250_reg_t
{	
    MPU9250_REG_SELF_TEST_X_GYRO    = 0x00,
    MPU9250_REG_SELF_TEST_Y_GYRO    = 0x01,
    MPU9250_REG_SELF_TEST_Z_GYRO    = 0x02,
    
    MPU9250_REG_SELF_TEST_X_ACCEL   = 0x0D,
    MPU9250_REG_SELF_TEST_Y_ACCEL   = 0x0E,
    MPU9250_REG_SELF_TEST_Z_ACCEL   = 0x0F,
    
    MPU9250_REG_XG_OFFSET_H         = 0x13,
    MPU9250_REG_XG_OFFSET_L         = 0x14,
    MPU9250_REG_YG_OFFSET_H         = 0x15,
    MPU9250_REG_YG_OFFSET_L         = 0x16,
    MPU9250_REG_ZG_OFFSET_H         = 0x17,
    MPU9250_REG_ZG_OFFSET_L         = 0x18,
    
    MPU9250_REG_SMPLRT_DIV          = 0x19,
    MPU9250_REG_CONFIG              = 0x1A,
    MPU9250_REG_GYRO_CONFIG         = 0x1B,
    MPU9250_REG_ACCEL_CONFIG        = 0x1C,
    MPU9250_REG_ACCEL_CONFIG2       = 0x1D,
    MPU9250_REG_LP_ACCEL_ODR        = 0x1E,
    MPU9250_REG_WOM_THR             = 0x1F,
		
    MPU9250_REG_FIFO_EN             = 0x23,
    MPU9250_REG_I2C_MST_CTRL        = 0x24,
    MPU9250_REG_I2C_SLV0_ADDR       = 0x25,
    MPU9250_REG_I2C_SLV0_REG        = 0x26,  
    MPU9250_REG_I2C_SLV0_CTRL       = 0x27,      
    MPU9250_REG_I2C_SLV1_ADDR       = 0x28,
    MPU9250_REG_I2C_SLV1_REG        = 0x29,
    MPU9250_REG_I2C_SLV1_CTRL       = 0x2A,
    MPU9250_REG_I2C_SLV2_ADDR       = 0x2B,
    MPU9250_REG_I2C_SLV2_REG        = 0x2C,
    MPU9250_REG_I2C_SLV2_CTRL       = 0x2D,
    MPU9250_REG_I2C_SLV3_ADDR       = 0x2E,
    MPU9250_REG_I2C_SLV3_REG        = 0x2F,
    MPU9250_REG_I2C_SLV3_CTRL       = 0x30,
    MPU9250_REG_I2C_SLV4_ADDR       = 0x31,
    MPU9250_REG_I2C_SLV4_REG        = 0x32,
    MPU9250_REG_I2C_SLV4_DO         = 0x33,
    MPU9250_REG_I2C_SLV4_CTRL       = 0x34,
    MPU9250_REG_I2C_SLV4_DI         = 0x35,
    
    MPU9250_REG_I2C_MST_STATUS      = 0x36,
    
    MPU9250_REG_INT_PIN_CFG         = 0x37,
    MPU9250_REG_INT_ENABLE          = 0x38,
    MPU9250_REG_INT_STATUS          = 0x3A,
    
    MPU9250_REG_ACCEL_XOUT_H        = 0x3B,
    MPU9250_REG_ACCEL_XOUT_L        = 0x3C,
    MPU9250_REG_ACCEL_YOUT_H        = 0x3D,
    MPU9250_REG_ACCEL_YOUT_L        = 0x3E,
    MPU9250_REG_ACCEL_ZOUT_H        = 0x3F,
    MPU9250_REG_ACCEL_ZOUT_L        = 0x40,
    
    MPU9250_REG_TEMP_OUT_H          = 0x41,
    MPU9250_REG_TEMP_OUT_L          = 0x42,
    
    MPU9250_REG_GYRO_XOUT_H         = 0x43,
    MPU9250_REG_GYRO_XOUT_L         = 0x44,
    MPU9250_REG_GYRO_YOUT_H         = 0x45,
    MPU9250_REG_GYRO_YOUT_L         = 0x46,
    MPU9250_REG_GYRO_ZOUT_H         = 0x47,
    MPU9250_REG_GYRO_ZOUT_L         = 0x48,
    
    MPU9250_REG_EXT_SENS_DATA_00    = 0x49,
    MPU9250_REG_EXT_SENS_DATA_01    = 0x4A,
    MPU9250_REG_EXT_SENS_DATA_02    = 0x4B,
    MPU9250_REG_EXT_SENS_DATA_03    = 0x4C,
    MPU9250_REG_EXT_SENS_DATA_04    = 0x4D,
    MPU9250_REG_EXT_SENS_DATA_05    = 0x4E,
    MPU9250_REG_EXT_SENS_DATA_06    = 0x4F,
    MPU9250_REG_EXT_SENS_DATA_07    = 0x50,
    MPU9250_REG_EXT_SENS_DATA_08    = 0x51,
    MPU9250_REG_EXT_SENS_DATA_09    = 0x52,
    MPU9250_REG_EXT_SENS_DATA_10    = 0x53,
    MPU9250_REG_EXT_SENS_DATA_11    = 0x54,
    MPU9250_REG_EXT_SENS_DATA_12    = 0x55,
    MPU9250_REG_EXT_SENS_DATA_13    = 0x56,
    MPU9250_REG_EXT_SENS_DATA_14    = 0x57,
    MPU9250_REG_EXT_SENS_DATA_15    = 0x58,
    MPU9250_REG_EXT_SENS_DATA_16    = 0x59,
    MPU9250_REG_EXT_SENS_DATA_17    = 0x5A,
    MPU9250_REG_EXT_SENS_DATA_18    = 0x5B,
    MPU9250_REG_EXT_SENS_DATA_19    = 0x5C,
    MPU9250_REG_EXT_SENS_DATA_20    = 0x5D,
    MPU9250_REG_EXT_SENS_DATA_21    = 0x5E,
    MPU9250_REG_EXT_SENS_DATA_22    = 0x5F,
    MPU9250_REG_EXT_SENS_DATA_23    = 0x60,
    
    MPU9250_REG_I2C_SLV0_DO         = 0x63,
    MPU9250_REG_I2C_SLV1_DO         = 0x64,
    MPU9250_REG_I2C_SLV2_DO         = 0x65,
    MPU9250_REG_I2C_SLV3_DO         = 0x66,
    
    MPU9250_REG_I2C_MST_DELAY_CTRL  = 0x67,
    MPU9250_REG_SIGNAL_PATH_RESET   = 0x68,
    MPU9250_REG_MOT_DETECT_CTRL     = 0x69,
    MPU9250_REG_USER_CTRL           = 0x6A,
    MPU9250_REG_PWR_MGMT_1          = 0x6B,
    MPU9250_REG_PWR_MGMT_2          = 0x6C,
    
    MPU9250_REG_FIFO_COUNTH         = 0x72,
    MPU9250_REG_FIFO_COUNTL         = 0x73,
    MPU9250_REG_FIFO_R_W            = 0x74,
    
    MPU9250_REG_WHO_AM_I            = 0x75,
    
    MPU9250_REG_XA_OFFSET_H         = 0x77,
    MPU9250_REG_XA_OFFSET_L         = 0x78,
    MPU9250_REG_YA_OFFSET_H         = 0x7A,
    MPU9250_REG_YA_OFFSET_L         = 0x7B,
    MPU9250_REG_ZA_OFFSET_H         = 0x7D,
    MPU9250_REG_ZA_OFFSET_L         = 0x7E
} mpu9250_reg_t;

typedef enum _mpu9250_ret_t
{
    MPU9250_OK,
    MPU9250_ERROR_NOT_INITIALIZED,
    MPU9250_ERROR_RX_TIMEOUT,
    MPU9250_ERROR_TX_TIMEOUT,
} mpu9250_ret_t;

typedef enum _mpu9250_acc_fs_t
{
    MPU9250_ACC_FS_2G,
    MPU9250_ACC_FS_4G,
    MPU9250_ACC_FS_8G,
    MPU9250_ACC_FS_16G,
    MPU9250_ACC_FS_LAST
} mpu9250_acc_fs_t;

typedef enum _mpu9250_gyro_fs_t
{
    MPU9250_GYRO_FS_250DPS,
    MPU9250_GYRO_FS_500DPS,
    MPU9250_GYRO_FS_1000DPS,
    MPU9250_GYRO_FS_2000DPS,
    MPU9250_GYRO_FS_LAST
} mpu9250_gyro_fs_t;

typedef enum _mpu9250_mag_fs_t
{
    MPU9250_MAG_FS_4800uT_16bit,
    MPU9250_MAG_FS_4800uT_14bit,
    MPU9250_MAG_FS_LAST
} mpu9250_mag_fs_t;

typedef struct _mpu9250_data_t
{
    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    int16_t raw_mag_x;
    int16_t raw_mag_y;
    int16_t raw_mag_z;
    int16_t raw_temp;
    float bias_acc_x;
    float bias_acc_y;
    float bias_acc_z;
    float scale_acc_x;
    float scale_acc_y;
    float scale_acc_z;
    float value_acc_x;
    float value_acc_y;
    float value_acc_z;
    float bias_gyro_x;
    float bias_gyro_y;
    float bias_gyro_z;
    float scale_gyro_x;
    float scale_gyro_y;
    float scale_gyro_z;
    float value_gyro_x;
    float value_gyro_y;
    float value_gyro_z;
    float bias_mag_x;
    float bias_mag_y;
    float bias_mag_z;
    float scale_mag_x;
    float scale_mag_y;
    float scale_mag_z;
    float value_mag_x;
    float value_mag_y;
    float value_mag_z;
    float value_temp;
} mpu9250_data_t;

typedef struct _mpu9250_t
{
    uint8_t active : 1;
    uint8_t id;
    uint8_t sw_lpf_enable;
    float sw_lpf_coef;
    SPI_HandleTypeDef *hspi;
    //mpu9250_status_t status;
    //mpu9250_fifo_src_t fifo_status;
    //mpu9250_int1_src_t int1_status;
    //mpu9250_settings_t settings;
    mpu9250_data_t data;
} mpu9250_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

bool MPU9250_Init(mpu9250_t * mpu9250, SPI_HandleTypeDef *hspi);
bool MPU9250_Deinit(mpu9250_t * mpu9250);
mpu9250_ret_t MPU9250_Identify(mpu9250_t * mpu9250);
mpu9250_ret_t MPU9250_Read_Register(mpu9250_t * mpu9250, mpu9250_reg_t reg, uint8_t * value);
mpu9250_ret_t MPU9250_Write_Register(mpu9250_t * mpu9250, mpu9250_reg_t reg, uint8_t value);
//mpu9250_ret_t MPU9250_Set_Settings(mpu9250_t * mpu9250);
//mpu9250_ret_t MPU9250_Get_Settings(mpu9250_t * mpu9250);
mpu9250_ret_t MPU9250_Get_Status(mpu9250_t * mpu9250);
mpu9250_ret_t MPU9250_Get_Data(mpu9250_t * mpu9250);
mpu9250_ret_t MPU9250_Configure(mpu9250_t * mpu9250);
void MPU9250_Calculate_Values(mpu9250_t * mpu9250);

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
