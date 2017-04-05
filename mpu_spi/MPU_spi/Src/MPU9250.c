/**
  ******************************************************************************
  * @file    MPU9250.c
  * @author  Eldar Sabanovic
  * @version V1.0.0
  * @date    2016-04-03
  * @brief   MPU9250 Ultra Stable Gyroscope driver
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
#include "MPU9250.h"
#include "cmsis_os.h"
#include "debug.h"

#define MPU9250_SPI_TIMEOUT (100)
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private Constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

float mpu9250_default_acc_scale_x[MPU9250_ACC_FS_LAST] =
{
    #if MY_BOARD
    0.00006103515625f * 0.9976f,
    0.0001220703125f,
    0.000244140625f,
    0.00048828125f
    #else
    0.00006103515625f,
    0.0001220703125f,
    0.000244140625f,
    0.00048828125f
    #endif
};

float mpu9250_default_acc_scale_y[MPU9250_ACC_FS_LAST] =
{
    #if MY_BOARD
    0.00006103515625f *  0.998f,
    0.0001220703125f,
    0.000244140625f,
    0.00048828125f
    #else
    0.00006103515625f,
    0.0001220703125f,
    0.000244140625f,
    0.00048828125f
    #endif
};

float mpu9250_default_acc_scale_z[MPU9250_ACC_FS_LAST] =
{
    #if MY_BOARD
    0.00006103515625f * 0.993f,
    0.0001220703125f,
    0.000244140625f,
    0.00048828125f
    #else
    0.00006103515625f,
    0.0001220703125f,
    0.000244140625f,
    0.00048828125f
    #endif
};

float mpu9250_default_acc_bias_x[MPU9250_ACC_FS_LAST] =
{
    #if MY_BOARD
    0.01f,
    0.0f,
    0.0f
    #else
    0.0f,
    0.0f,
    0.0f
    #endif
};


float mpu9250_default_acc_bias_y[MPU9250_ACC_FS_LAST] =
{
    #if MY_BOARD
    0.005f,
    0.0f,
    0.0f
    #else
    0.0f,
    0.0f,
    0.0f
    #endif
};

float mpu9250_default_acc_bias_z[MPU9250_ACC_FS_LAST] =
{
    #if MY_BOARD
    0.031f,
    0.0f,
    0.0f
    #else
    0.0f,
    0.0f,
    0.0f
    #endif
};

float mpu9250_default_gyro_scale_x[MPU9250_GYRO_FS_LAST] =
{
    0.0076335877862595f,
    0.0152671755725191f,
    0.0304878048780488f,
    0.0609756097560976f
};

float mpu9250_default_gyro_scale_y[MPU9250_GYRO_FS_LAST] =
{
    0.0076335877862595f,
    0.0152671755725191f,
    0.0304878048780488f,
    0.0609756097560976f
};
float mpu9250_default_gyro_scale_z[MPU9250_GYRO_FS_LAST] =
{
    0.0076335877862595f,
    0.0152671755725191f,
    0.0304878048780488f,
    0.0609756097560976f
};

float mpu9250_default_gyro_bias_x[MPU9250_GYRO_FS_LAST] =
{
    #if MY_BOARD
    1.171644f,
    0.0f,
    0.0f,
    1.087179f,
    #else
    1.1f,
    0.0f,
    0.0f,
    0,0f
    #endif
};

float mpu9250_default_gyro_bias_y[MPU9250_GYRO_FS_LAST] =
{
    #if MY_BOARD
    1.680136f,
    0.0f,
    0.0f,
    1.820836f
    #else
    1.5f,
    0.0f,
    0.0f,
    0.0f
    #endif
};

float mpu9250_default_gyro_bias_z[MPU9250_GYRO_FS_LAST] =
{
    #if MY_BOARD
    0.923616f,
    0.0f,
    0.0f,
    0.846139f
    #else
    1.1f,
    0.0f,
    0.0f,
    0.0f
    #endif
};

float mpu9250_default_mag_scale_x[MPU9250_MAG_FS_LAST] =
{
    #if MY_BOARD
    0.15f * 1.18359375f * 1.02564f,
    0.6f * 1.18359375f //uF/LSB
    #else
    0.15f,
    0.6f //uF/LSB
    #endif
};

float mpu9250_default_mag_scale_y[MPU9250_MAG_FS_LAST] =
{
    #if MY_BOARD
    0.15f * 1.1953125f * 1.006944f,
    0.6f * 1.1953125f,//uF/LSB
    #else
    0.15f,
    0.6f //uF/LSB
    #endif
};
float mpu9250_default_mag_scale_z[MPU9250_MAG_FS_LAST] =
{
    #if MY_BOARD
    0.15f * 1.14453125f,
    0.6f * 1.14453125f //uF/LSB
    #else
    0.15f,
    0.6f //uF/LSB
    #endif
};

float mpu9250_default_mag_bias_x[MPU9250_MAG_FS_LAST] =
{
    #if MY_BOARD
    19.0f
    #else
    0.0f
    #endif
};

float mpu9250_default_mag_bias_y[MPU9250_MAG_FS_LAST] =
{
    #if MY_BOARD
    36.0f
    #else
    0.0f
    #endif
};

float mpu9250_default_mag_bias_z[MPU9250_MAG_FS_LAST] =
{
    #if MY_BOARD
    15.0f
    #else
    0.0f
    #endif
};

bool MPU9250_Init(mpu9250_t * mpu9250, SPI_HandleTypeDef *hspi)
{
    mpu9250->hspi = hspi;
    mpu9250->active = 1;
    mpu9250->sw_lpf_enable = 1;
    mpu9250->sw_lpf_coef = 0.3;
   
    if (hspi->State == HAL_SPI_STATE_READY)
    {
        if (MPU9250_Identify(mpu9250) == MPU9250_OK)
        {
            if (mpu9250->id == 0x71)
            {
                DEBUG_SENSOR("MPU9250 initialization successful. ID: %d", mpu9250->id);
                DEBUG_SENSOR("MPU9250 configuration.");
                if (MPU9250_Configure(mpu9250) == MPU9250_OK)
                {
                    DEBUG_SENSOR("MPU9250 configuration successful.");
                    return true;
                }
                else
                {
                    DEBUG_SENSOR("MPU9250 configuration fail.");
                    return false;
                }
            }
            else
            {
                DEBUG_SENSOR("MPU9250 initialization fail: bad id.");
                MPU9250_Deinit(mpu9250);
                return false;
            }
        }
        else
        {
            DEBUG_SENSOR("MPU9250 initialization fail: can not identify.");
            MPU9250_Deinit(mpu9250);
            return false;
        }
    }
    else
    {
        DEBUG_SENSOR("MPU9250 initialization fail: SPI not ready.");
        MPU9250_Deinit(mpu9250);
        return false;
    }
}

bool MPU9250_Deinit(mpu9250_t * mpu9250)
{
    if (mpu9250->active == 1)
    {
        mpu9250->active = 0;
    }
    return true;
}

mpu9250_ret_t MPU9250_Identify(mpu9250_t * mpu9250)
{
    mpu9250_ret_t ret;
    if (mpu9250->active)
    {      
        ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_WHO_AM_I, &mpu9250->id);
    }
    else ret = MPU9250_ERROR_NOT_INITIALIZED;
    return ret;
}

mpu9250_ret_t MPU9250_Get_Status(mpu9250_t * mpu9250)
{
    mpu9250_ret_t ret;
    if (mpu9250->active)
    {      
//        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_STATUS, (uint8_t*)&mpu9250->status)) != MPU9250_OK) return ret;
//        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_STATUS, (uint8_t*)&mpu9250->fifo_status)) != MPU9250_OK) return ret;
//        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_SRC, (uint8_t*)&mpu9250->int1_status)) != MPU9250_OK) return ret;
    }
    else ret = MPU9250_ERROR_NOT_INITIALIZED;
    return ret;
}
/*
mpu9250_ret_t MPU9250_Get_Settings(mpu9250_t * mpu9250)
{
    mpu9250_ret_t ret;
    uint8_t value8;
    uint16_t value16;
    if (mpu9250->active)
    {        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_CTRL_REG1, &value8)) != MPU9250_OK) return ret;
        mpu9250->settings.data_rate = (mpu9250_odr_t)(value8>>6);
        mpu9250->settings.bandwidth = 0x03&(value8>>4);
        mpu9250->settings.power_down_mode = 0x01&(value8>>3);
        mpu9250->settings.z_axis_enabled = 0x01&(value8>>2);
        mpu9250->settings.y_axis_enabled = 0x01&(value8>>1);
        mpu9250->settings.x_axis_enabled = 0x01&value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_CTRL_REG2, &value8)) != MPU9250_OK) return ret;
        mpu9250->settings.hp_filter_mode = 0x03&(value8>>4);
        mpu9250->settings.hp_filter_cutoff = 0x0F&value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_CTRL_REG3, &value8)) != MPU9250_OK) return ret;
        mpu9250->settings.interrupt_enabled_on_int1 = 0x01&(value8>>7);
        mpu9250->settings.boot_status_on_int1 = 0x01&(value8>>6);
        mpu9250->settings.int1_active_level = 0x01&(value8>>5);
        mpu9250->settings.pp1_or_od0 = 0x01&(value8>>4);
        mpu9250->settings.drdy_on_int2 = 0x01&(value8>>3);
        mpu9250->settings.fifo_wtm_interrupt_on_int2 = 0x01&(value8>>2);
        mpu9250->settings.fifo_orun_interrupt_on_int2 = 0x01&(value8>>1);
        mpu9250->settings.fifo_wtm_interrupt_on_int2 = 0x01&value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_CTRL_REG4, &value8)) != MPU9250_OK) return ret;
        mpu9250->settings.block_data_update = 0x01&(value8>>7);
        mpu9250->settings.big1_little0_endian = 0x01&(value8>>6);
        mpu9250->settings.full_scale = (mpu9250_fs_t)(0x03&(value8>>4));
        mpu9250->settings.self_test_enabled = 0x03&(value8>>1);
        mpu9250->settings.spi_mode_4wire1_3wire0 = 0x01&value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_CTRL_REG5, &value8)) != MPU9250_OK) return ret;
        mpu9250->settings.reboot_mem_content = 0x01&(value8>>7);
        mpu9250->settings.fifo_enabled = 0x01&(value8>>6);
        mpu9250->settings.hp_filter_enabled = 0x01&(value8>>4);
        mpu9250->settings.int1_sel = 0x03&(value8>>2);
        mpu9250->settings.out_sel = 0x03&value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_REFERENCE, &value8)) != MPU9250_OK) return ret;
        mpu9250->settings.ref_value_for_interrupt = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_FIFO_CTRL, &value8)) != MPU9250_OK) return ret;
        mpu9250->settings.fifo_mode = (mpu9250_fifo_mode_t)(0x07&(value8>>5));
        mpu9250->settings.fifo_wtm_ths = 0x0F&value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_CFG, (uint8_t*)&mpu9250->settings.int1_cfg)) != MPU9250_OK) return ret;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_THS_XH, &value8)) != MPU9250_OK) return ret;
        value16 = ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_THS_XL, &value8)) != MPU9250_OK) return ret;
        value16 |= value8;
        mpu9250->settings.int1_ths_x = value16;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_THS_YH, &value8)) != MPU9250_OK) return ret;
        value16 = ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_THS_YL, &value8)) != MPU9250_OK) return ret;
        value16 |= value8;
        mpu9250->settings.int1_ths_y = value16;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_THS_ZH, &value8)) != MPU9250_OK) return ret;
        value16 = ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_THS_ZL, &value8)) != MPU9250_OK) return ret;
        value16 |= value8;
        mpu9250->settings.int1_ths_z = value16;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_INT1_DURATION, &value8)) != MPU9250_OK) return ret;
        value16 |= value8;
        mpu9250->settings.int1_duration = value8;
    }
    else ret = MPU9250_ERROR_NOT_INITIALIZED;
    return ret;
}

mpu9250_ret_t MPU9250_Set_Settings(mpu9250_t * mpu9250)
{
    mpu9250_ret_t ret;
    uint8_t value8;
    if (mpu9250->active)
    {        
        value8  = mpu9250->settings.data_rate << 6;
        value8 |= mpu9250->settings.bandwidth << 4;
        value8 |= mpu9250->settings.power_down_mode << 3;
        value8 |= mpu9250->settings.z_axis_enabled << 2;
        value8 |= mpu9250->settings.z_axis_enabled << 1;
        value8 |= mpu9250->settings.z_axis_enabled;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_CTRL_REG1, value8)) != MPU9250_OK) return ret;
        
        value8  = mpu9250->settings.hp_filter_mode << 4;
        value8 |= mpu9250->settings.hp_filter_cutoff;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_CTRL_REG2, value8)) != MPU9250_OK) return ret;
        
        value8  = mpu9250->settings.interrupt_enabled_on_int1 << 7;
        value8 |= mpu9250->settings.boot_status_on_int1 << 6;
        value8 |= mpu9250->settings.int1_active_level << 5;
        value8 |= mpu9250->settings.pp1_or_od0 << 4;
        value8 |= mpu9250->settings.drdy_on_int2 << 3;
        value8 |= mpu9250->settings.fifo_wtm_interrupt_on_int2 << 2;
        value8 |= mpu9250->settings.fifo_orun_interrupt_on_int2 << 1;
        value8 |= mpu9250->settings.fifo_wtm_interrupt_on_int2;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_CTRL_REG3, value8)) != MPU9250_OK) return ret;
     
        value8  = mpu9250->settings.block_data_update  << 7;
        value8 |= mpu9250->settings.big1_little0_endian << 6;
        value8 |= mpu9250->settings.full_scale << 4;
        value8 |= mpu9250->settings.self_test_enabled  << 1;
        value8 |= mpu9250->settings.spi_mode_4wire1_3wire0;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_CTRL_REG4, value8)) != MPU9250_OK) return ret;
        
        value8  = mpu9250->settings.reboot_mem_content  << 7;
        value8 |= mpu9250->settings.fifo_enabled << 6;
        value8 |= mpu9250->settings.hp_filter_enabled << 4;
        value8 |= mpu9250->settings.int1_sel  << 1;
        value8 |= mpu9250->settings.out_sel;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_CTRL_REG5, value8)) != MPU9250_OK) return ret;
        
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_REFERENCE, mpu9250->settings.ref_value_for_interrupt)) != MPU9250_OK) return ret;
        
        value8  = mpu9250->settings.fifo_mode << 5;
        value8 |= mpu9250->settings.fifo_wtm_ths;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_FIFO_CTRL, value8)) != MPU9250_OK) return ret;
        
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_CFG, *(uint8_t*)&mpu9250->settings.int1_cfg)) != MPU9250_OK) return ret;
        
        value8 = mpu9250->settings.int1_ths_x >> 8;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_THS_XH, value8)) != MPU9250_OK) return ret;
        
        value8 = 0xFF & mpu9250->settings.int1_ths_x;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_THS_XL, value8)) != MPU9250_OK) return ret;
        
        value8 = mpu9250->settings.int1_ths_y >> 8;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_THS_YH, value8)) != MPU9250_OK) return ret;
        
        value8 = 0xFF & mpu9250->settings.int1_ths_y;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_THS_YL, value8)) != MPU9250_OK) return ret;
        
        value8 = mpu9250->settings.int1_ths_z >> 8;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_THS_ZH, value8)) != MPU9250_OK) return ret;
        
        value8 = 0xFF & mpu9250->settings.int1_ths_z;
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_THS_ZL, value8)) != MPU9250_OK) return ret;
        
        if ((ret = MPU9250_Write_Register(mpu9250, MPU9250_REG_INT1_DURATION, mpu9250->settings.int1_duration)) != MPU9250_OK) return ret;
    }
    else ret = MPU9250_ERROR_NOT_INITIALIZED;
    return ret;
}
*/
mpu9250_ret_t MPU9250_Configure(mpu9250_t * mpu9250)
{
    mpu9250_ret_t ret;
    
    //if ((ret = MPU9250_Get_Settings(mpu9250)) == MPU9250_OK)
    //{ 
        /* Set current scale and bias parameters */
//        mpu9250->data.bias_x = mpu9250_default_bias_x[mpu9250->settings.full_scale];
//        mpu9250->data.bias_y = mpu9250_default_bias_y[mpu9250->settings.full_scale];
 //       mpu9250->data.bias_z = mpu9250_default_bias_z[mpu9250->settings.full_scale];
        
//       mpu9250->data.scale_x = mpu9250_default_scale_x[mpu9250->settings.full_scale];
 //       mpu9250->data.scale_y = mpu9250_default_scale_y[mpu9250->settings.full_scale];
 //       mpu9250->data.scale_z = mpu9250_default_scale_z[mpu9250->settings.full_scale];
        
        /* Please change default configuration here */ 
//        mpu9250->settings.power_down_mode = 1;
 //       mpu9250->settings.bandwidth = 3;
 //       mpu9250->settings.data_rate = 3;
 //       ret = MPU9250_Set_Settings(mpu9250);
    //}
    
    mpu9250->data.bias_acc_x = mpu9250_default_acc_bias_x[0];
    mpu9250->data.bias_acc_y = mpu9250_default_acc_bias_y[0];
    mpu9250->data.bias_acc_z = mpu9250_default_acc_bias_z[0];
        
    mpu9250->data.scale_acc_x = mpu9250_default_acc_scale_x[0];
    mpu9250->data.scale_acc_y = mpu9250_default_acc_scale_y[0];
    mpu9250->data.scale_acc_z = mpu9250_default_acc_scale_z[0];
    
    mpu9250->data.bias_gyro_x = mpu9250_default_gyro_bias_x[3];
    mpu9250->data.bias_gyro_y = mpu9250_default_gyro_bias_y[3];
    mpu9250->data.bias_gyro_z = mpu9250_default_gyro_bias_z[3];
        
    mpu9250->data.scale_gyro_x = mpu9250_default_gyro_scale_x[3];
    mpu9250->data.scale_gyro_y = mpu9250_default_gyro_scale_y[3];
    mpu9250->data.scale_gyro_z = mpu9250_default_gyro_scale_z[3];
    
    mpu9250->data.bias_mag_x = mpu9250_default_mag_bias_x[0];
    mpu9250->data.bias_mag_y = mpu9250_default_mag_bias_y[0];
    mpu9250->data.bias_mag_z = mpu9250_default_mag_bias_z[0];
        
    mpu9250->data.scale_mag_x = mpu9250_default_mag_scale_x[0];
    mpu9250->data.scale_mag_y = mpu9250_default_mag_scale_y[0];
    mpu9250->data.scale_mag_z = mpu9250_default_mag_scale_z[0];
    
    /*Temporary configuration*/
    MPU9250_Write_Register(mpu9250, MPU9250_REG_PWR_MGMT_1, 0x80); /* H_Reset */
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_PWR_MGMT_1, 0x01); /* Auto-select best clock, PLL if it is ready, internal if not */
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_PWR_MGMT_2, 0x00); /* All is enabled */
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_CONFIG, 0x01); /* DLPF_CFG = 1 */
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_GYRO_CONFIG, 0x18); /* Gyro full scale = +-2000, DLPF is enabled on 184 Hz*/
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_ACCEL_CONFIG, 0x00); /* Acc full scale = +-2, DLPF is enabled  on 184 Hz*/
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_ACCEL_CONFIG2, 0x05); /* DLPF is enabled on 184 Hz*/
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_INT_PIN_CFG, 0x30); /* Int pin configured as actrive high, and cleared if any read is done */
    osDelay(10);
    
    /* Temporary configuration for integrated AK8975 magnetometer*/
    /* AUX master enable */
    MPU9250_Write_Register(mpu9250, MPU9250_REG_USER_CTRL, 0x20);
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_MST_CTRL, 0x0D);
    osDelay(10);
    
    /* Slave 0 config */
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_ADDR, AK8963_ADDRESS_WRITE);
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_CNTL2);
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_DO, 0x01); /*RESET AK8963*/
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_CTRL, 0x81); /*ENABLE trasmission*/
    osDelay(100);
    
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_REG, AK8963_REG_CNTL1);
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_DO, 0x16); /* Register value to 100Hz continuous measurement in 16bit*/
    osDelay(10);
    MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_CTRL, 0x81); /* ENABLE trasmission */
    osDelay(10);
		    
    ret = MPU9250_OK;
    return ret;
}

mpu9250_ret_t MPU9250_Get_Data(mpu9250_t * mpu9250)
{
    mpu9250_ret_t ret;
    uint8_t value8;
    int16_t value16[10] = {0};
    if (mpu9250->active)
    {         
        /* Command magnetometer data read */
        MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_ADDR, AK8963_ADDRESS_READ);
        MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_REG , AK8963_REG_HXL);
        MPU9250_Write_Register(mpu9250, MPU9250_REG_I2C_SLV0_CTRL, 0x87);
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_ACCEL_XOUT_L, &value8)) != MPU9250_OK) return ret;
        value16[0] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_ACCEL_XOUT_H, &value8)) != MPU9250_OK) return ret;
        value16[0] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_ACCEL_YOUT_L, &value8)) != MPU9250_OK) return ret;
        value16[1] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_ACCEL_YOUT_H, &value8)) != MPU9250_OK) return ret;
        value16[1] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_ACCEL_ZOUT_L, &value8)) != MPU9250_OK) return ret;
        value16[2] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_ACCEL_ZOUT_H, &value8)) != MPU9250_OK) return ret;
        value16[2] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_TEMP_OUT_L, &value8)) != MPU9250_OK) return ret;
        value16[3] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_TEMP_OUT_H, &value8)) != MPU9250_OK) return ret;
        value16[3] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_GYRO_XOUT_L, &value8)) != MPU9250_OK) return ret;
        value16[4] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_GYRO_XOUT_H, &value8)) != MPU9250_OK) return ret;
        value16[4] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_GYRO_YOUT_L, &value8)) != MPU9250_OK) return ret;
        value16[5] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_GYRO_YOUT_H, &value8)) != MPU9250_OK) return ret;
        value16[5] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_GYRO_ZOUT_L, &value8)) != MPU9250_OK) return ret;
        value16[6] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_GYRO_ZOUT_H, &value8)) != MPU9250_OK) return ret;
        value16[6] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_EXT_SENS_DATA_00, &value8)) != MPU9250_OK) return ret;
        value16[7] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_EXT_SENS_DATA_01, &value8)) != MPU9250_OK) return ret;
        value16[7] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_EXT_SENS_DATA_02, &value8)) != MPU9250_OK) return ret;
        value16[8] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_EXT_SENS_DATA_03, &value8)) != MPU9250_OK) return ret;
        value16[8] |= ((uint16_t)value8) << 8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_EXT_SENS_DATA_04, &value8)) != MPU9250_OK) return ret;
        value16[9] = value8;
        
        if ((ret = MPU9250_Read_Register(mpu9250, MPU9250_REG_EXT_SENS_DATA_05, &value8)) != MPU9250_OK) return ret;
        value16[9] |= ((uint16_t)value8) << 8;
        
        mpu9250->data.raw_acc_x = -value16[1];
        mpu9250->data.raw_acc_y = -value16[0];
        mpu9250->data.raw_acc_z = -value16[2];
        mpu9250->data.raw_temp  = value16[3];
        mpu9250->data.raw_gyro_x = -value16[5];
        mpu9250->data.raw_gyro_y = -value16[4];
        mpu9250->data.raw_gyro_z = -value16[6];
        mpu9250->data.raw_mag_x = -value16[7];
        mpu9250->data.raw_mag_y = -value16[8];
        mpu9250->data.raw_mag_z = value16[9];
        
    }
    else ret = MPU9250_ERROR_NOT_INITIALIZED;
    
    MPU9250_Calculate_Values(mpu9250);
    
    return ret;
}

void MPU9250_Calculate_Values(mpu9250_t * mpu9250)
{
    static float data[9] = {0,0,0,0,0,0,0,0,0};
    
    if (mpu9250->sw_lpf_enable)
    {
        /* Accelerometer values conversion */
        data[0] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_acc_x * mpu9250->data.raw_acc_x) - data[0]);
        data[1] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_acc_y * mpu9250->data.raw_acc_y) - data[1]);
        data[2] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_acc_z * mpu9250->data.raw_acc_z) - data[2]);
        
        mpu9250->data.value_acc_x = (data[0] + mpu9250->data.bias_acc_x);
        mpu9250->data.value_acc_y = (data[1] + mpu9250->data.bias_acc_y);
        mpu9250->data.value_acc_z = (data[2] + mpu9250->data.bias_acc_z);
        
        /* Gyroscope values conversion */
        data[3] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_gyro_x * mpu9250->data.raw_gyro_x) - data[3]);
        data[4] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_gyro_y * mpu9250->data.raw_gyro_y) - data[4]);
        data[5] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_gyro_z * mpu9250->data.raw_gyro_z) - data[5]);
        
        mpu9250->data.value_gyro_x = (data[3] + mpu9250->data.bias_gyro_x);
        mpu9250->data.value_gyro_y = (data[4] + mpu9250->data.bias_gyro_y);
        mpu9250->data.value_gyro_z = (data[5] + mpu9250->data.bias_gyro_z);
        
        /* Magnetometer values conversion */
        data[6] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_mag_x * mpu9250->data.raw_mag_x) - data[6]);
        data[7] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_mag_y * mpu9250->data.raw_mag_y) - data[7]);
        data[8] += mpu9250->sw_lpf_coef * ((mpu9250->data.scale_mag_z * mpu9250->data.raw_mag_z) - data[8]);
        
        mpu9250->data.value_mag_x = (data[6] + mpu9250->data.bias_mag_x);
        mpu9250->data.value_mag_y = (data[7] + mpu9250->data.bias_mag_y);
        mpu9250->data.value_mag_z = (data[8] + mpu9250->data.bias_mag_z);
    }
    else
    {
        /* Accelerometer values conversion */
        mpu9250->data.value_acc_x = (mpu9250->data.scale_acc_x * mpu9250->data.raw_acc_x + mpu9250->data.bias_acc_x);
        mpu9250->data.value_acc_y = (mpu9250->data.scale_acc_y * mpu9250->data.raw_acc_y + mpu9250->data.bias_acc_y);
        mpu9250->data.value_acc_z = (mpu9250->data.scale_acc_z * mpu9250->data.raw_acc_z + mpu9250->data.bias_acc_z);
        
        /* Gyroscope values conversion */
        mpu9250->data.value_gyro_x = (mpu9250->data.scale_gyro_x * mpu9250->data.raw_gyro_x + mpu9250->data.bias_gyro_x);
        mpu9250->data.value_gyro_y = (mpu9250->data.scale_gyro_y * mpu9250->data.raw_gyro_y + mpu9250->data.bias_gyro_y);
        mpu9250->data.value_gyro_z = (mpu9250->data.scale_gyro_z * mpu9250->data.raw_gyro_z + mpu9250->data.bias_gyro_z);
        
        /* Magnetometer values conversion */
        mpu9250->data.value_mag_x = (mpu9250->data.scale_mag_x * mpu9250->data.raw_mag_x + mpu9250->data.bias_mag_x);
        mpu9250->data.value_mag_y = (mpu9250->data.scale_mag_y * mpu9250->data.raw_mag_y + mpu9250->data.bias_mag_y);
        mpu9250->data.value_mag_z = (mpu9250->data.scale_mag_z * mpu9250->data.raw_mag_z + mpu9250->data.bias_mag_z);
        
    }
    /* Temperature value conversion */
    mpu9250->data.value_temp = mpu9250->data.raw_temp * 0.0029951777638003f;
}

mpu9250_ret_t MPU9250_Read_Register(mpu9250_t * mpu9250, mpu9250_reg_t reg, uint8_t * value)
{
    mpu9250_ret_t ret;
    uint8_t tx = ((uint8_t)reg+0x80);
    if (mpu9250->active == 1)
    {
        BSP_SPI2_Select(SPI2_MPU9250);
        if (BSP_SPI_Transmit(mpu9250->hspi, &tx, 1, MPU9250_SPI_TIMEOUT))
        {
            if (BSP_SPI_Receive(mpu9250->hspi, value, 1, MPU9250_SPI_TIMEOUT))
            {
                ret = MPU9250_OK;
            }
            else ret = MPU9250_ERROR_RX_TIMEOUT;
        }
        else ret = MPU9250_ERROR_TX_TIMEOUT;
        BSP_SPI2_Deselect();
    }
    else ret = MPU9250_ERROR_NOT_INITIALIZED;
    return ret;
}

mpu9250_ret_t MPU9250_Write_Register(mpu9250_t * mpu9250, mpu9250_reg_t reg, uint8_t value)
{
    mpu9250_ret_t ret;
    uint8_t tx_buf[2] = {(uint8_t)reg, value};
    if (mpu9250->active == 1)
    {
        BSP_SPI2_Select(SPI2_MPU9250);
        if (BSP_SPI_Transmit(mpu9250->hspi, &tx_buf[0], 2, MPU9250_SPI_TIMEOUT))
        {
            ret = MPU9250_OK;
        }
        else ret = MPU9250_ERROR_TX_TIMEOUT;
        BSP_SPI2_Deselect();
    }
    else ret = MPU9250_ERROR_NOT_INITIALIZED;
    return ret;
}


/************************ (C) COPYRIGHT VGTU *****END OF FILE****/
