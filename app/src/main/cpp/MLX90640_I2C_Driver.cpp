/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <jni.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#define I2C_MSG_FMT __u8

#include "android/log.h"
static const char *TAG="i2c_port";

#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)

#include "MLX90640_I2C_Driver.h"

int I2Cfile;

void MLX90640_I2CInit(int i2cfile, u_int8_t i2cslaveaddress)
{
    I2Cfile = i2cfile;
}


//This command will reset all devices on the bus. As the I2C is shared calling this function could break other peripherals on the bus.
int MLX90640_I2CGeneralReset(void)
{
    int rc;
    char cmd[1] = {0};
    
    cmd[0] = 0x06;

    if (ioctl(I2Cfile, I2C_SLAVE, 0x00) < 0) {
        LOGE("Failed to set I2C slave to address 0x00");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        return -1;
    }

    rc = write(I2Cfile, cmd, 1);
    if (rc <= 0) {
        LOGE("Failed to reset I2C bus");
        return -1;
    }

    usleep(50);
    
    return 0;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    char i2cData[1664] = {0};
    int cnt = 0;
    int i = 0;

    int result;
    char cmd[2] = {(char)(startAddress >> 8), (char)(startAddress & 0xFF)};
    uint16_t *p = data;
    struct i2c_msg i2c_messages[2];
    struct i2c_rdwr_ioctl_data i2c_messageset[1];

    i2c_messages[0].addr = slaveAddr;
    i2c_messages[0].flags = 0;
    i2c_messages[0].len = 2;
    i2c_messages[0].buf = (I2C_MSG_FMT*)cmd;

    i2c_messages[1].addr = slaveAddr;
    i2c_messages[1].flags = I2C_M_RD;
    i2c_messages[1].len = nMemAddressRead * 2;
    i2c_messages[1].buf = (I2C_MSG_FMT*)i2cData;

    i2c_messageset[0].msgs = i2c_messages;
    i2c_messageset[0].nmsgs = 2;

    memset(i2cData, 0, nMemAddressRead * 2);

    if (ioctl(I2Cfile, I2C_RDWR, &i2c_messageset) < 0) {
        LOGE("Failed to read. I2C error.");
        return -1;
    }

    for(int count = 0; count < nMemAddressRead; count++){
        int i = count << 1;
        *p++ = ((uint16_t)i2cData[i] << 8) | i2cData[i+1];
    }

    return 0;   
} 

void MLX90640_I2CFreqSet(int freq)
{
    //Unsupported in Linux. I2C frequency is fixed in the device tree
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    int rc;
    char cmd[4] = {0,0,0,0};
    struct i2c_msg i2c_messages[1];
    struct i2c_rdwr_ioctl_data i2c_messageset[1];

    cmd[0] = writeAddress >> 8;
    cmd[1] = writeAddress & 0x00FF;
    cmd[2] = data >> 8;
    cmd[3] = data & 0x00FF;

    i2c_messages[0].addr = slaveAddr;
    i2c_messages[0].flags = 0;
    i2c_messages[0].len = 4;
    i2c_messages[0].buf = (I2C_MSG_FMT*)cmd;

    i2c_messageset[0].msgs = i2c_messages;
    i2c_messageset[0].nmsgs = 1;

    if (ioctl(I2Cfile, I2C_RDWR, &i2c_messageset) < 0)
    {
        LOGE("Failed to write. I2C error.");
        return -1;
    }
    
    return 0;
}

