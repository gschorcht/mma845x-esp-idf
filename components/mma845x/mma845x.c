/*
 * Driver for MMA845X 3-axes digital accelerometer connected to I2C.
 *
 * This driver is for the usage with the ESP8266 and FreeRTOS (esp-open-rtos)
 * [https://github.com/SuperHouse/esp-open-rtos]. It is also working with ESP32
 * and ESP-IDF [https://github.com/espressif/esp-idf.git] as well as Linux
 * based systems using a wrapper library for ESP8266 functions.
 *
 * ---------------------------------------------------------------------------
 *
 * The BSD License (3-clause license)
 *
 * Copyright (c) 2018 Gunar Schorcht (https://github.com/gschorcht)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The information provided is believed to be accurate and reliable. The
 * copyright holder assumes no responsibility for the consequences of use
 * of such information nor for any infringement of patents or other rights
 * of third parties which may result from its use. No license is granted by
 * implication or otherwise under any patent or patent rights of the copyright
 * holder.
 */

#include <string.h>
#include <stdlib.h>

#include "mma845x.h"

#if defined(MMA845X_DEBUG_LEVEL_2)
#define debug(s, f, ...) printf("%s %s: " s "\n", "MMA845X", f, ## __VA_ARGS__)
#define debug_dev(s, f, d, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "MMA845X", f, d->bus, d->addr, ## __VA_ARGS__)
#else
#define debug(s, f, ...)
#define debug_dev(s, f, d, ...)
#endif

#if defined(MMA845X_DEBUG_LEVEL_1) || defined(MMA845X_DEBUG_LEVEL_2)
#define error(s, f, ...) printf("%s %s: " s "\n", "MMA845X", f, ## __VA_ARGS__)
#define error_dev(s, f, d, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "MMA845X", f, d->bus, d->addr, ## __VA_ARGS__)
#else
#define error(s, f, ...)
#define error_dev(s, f, d, ...)
#endif

// register addresses
#define MMA845X_REG_STATUS              0x00
#define MMA845X_REG_F_STATUS            0x00
#define MMA845X_REG_OUT_X_MSB           0x01
#define MMA845X_REG_OUT_X_LSB           0x02
#define MMA845X_REG_OUT_Y_MSB           0x03
#define MMA845X_REG_OUT_Y_LSB           0x04
#define MMA845X_REG_OUT_Z_MSB           0x05
#define MMA845X_REG_OUT_Z_LSB           0x06

#define MMA845X_REG_F_SETUP             0x09
#define MMA845X_REG_TRIG_CFG            0x0a
#define MMA845X_REG_SYSMOD              0x0b
#define MMA845X_REG_INT_SOURCE          0x0c
#define MMA845X_REG_WHO_AM_I            0x0d
#define MMA845X_REG_XYZ_DATA_CFG        0x0e
#define MMA845X_REG_HP_FILTER_CUTOFF    0x0f
#define MMA845X_REG_PL_STATUS           0x10
#define MMA845X_REG_PL_CFG              0x11
#define MMA845X_REG_PL_COUNT            0x12
#define MMA845X_REG_PL_BF_ZCOMP         0x13
#define MMA845X_REG_PL_THS              0x14
#define MMA845X_REG_FF_MT_CFG           0x15
#define MMA845X_REG_FF_MT_SRC           0x16
#define MMA845X_REG_FF_MT_THS           0x17
#define MMA845X_REG_FF_MT_COUNT         0x18

#define MMA845X_REG_TRANSIENT_CFG       0x1d
#define MMA845X_REG_TRANSIENT_SRC       0x1e
#define MMA845X_REG_TRANSIENT_THS       0x1f
#define MMA845X_REG_TRANSIENT_COUNT     0x20
#define MMA845X_REG_PULSE_CFG           0x21
#define MMA845X_REG_PULSE_SRC           0x22
#define MMA845X_REG_PULSE_THSX          0x23
#define MMA845X_REG_PULSE_THSY          0x24
#define MMA845X_REG_PULSE_THSZ          0x25
#define MMA845X_REG_PULSE_TMLT          0x26
#define MMA845X_REG_PULSE_LTCY          0x27
#define MMA845X_REG_PULSE_WIND          0x28
#define MMA845X_REG_ASLP_COUNT          0x29
#define MMA845X_REG_CTRL1               0x2a
#define MMA845X_REG_CTRL2               0x2b
#define MMA845X_REG_CTRL3               0x2c
#define MMA845X_REG_CTRL4               0x2d
#define MMA845X_REG_CTRL5               0x2e
#define MMA845X_REG_OFF_X               0x2f
#define MMA845X_REG_OFF_Y               0x30
#define MMA845X_REG_OFF_Z               0x31


// register structure definitions
struct mma845x_reg_status 
{
    uint8_t XDR  :1; // STATUS<0>   X axis new data ready
    uint8_t YDR  :1; // STATUS<1>   Y axis new data ready 
    uint8_t ZDR  :1; // STATUS<2>   Z axis new data ready
    uint8_t ZYXDR:1; // STATUS<3>   X, Y and Z axis new data ready
    uint8_t XOW  :1; // STATUS<4>   X axis data overwrtie
    uint8_t YOW  :1; // STATUS<5>   Y axis data overwrite
    uint8_t ZOW  :1; // STATUS<6>   Z axis data overwrite
    uint8_t ZYXOW:1; // STATUS<7>   X, Y and Z axis data overrun
};

#define MMA845X_ANY_DATA_READY    0x0f    // MMA845X_REG_STATUS<3:0>

struct mma845x_reg_f_status 
{
    uint8_t F_CNT      :6; // F_STATUS<5:0> Number of data samples stored  X axis new data ready
    uint8_t F_WMRK_FLAG:1; // F_STATUS<1>   F_CNT > watermark
    uint8_t F_OVF      :1; // F_STATUS<2>   FIFO overflow happened
};

struct mma845x_reg_ctrl1 
{
    uint8_t ACTIVE   :1; // CTRL1<0>   Full-scale selection
    uint8_t F_READ   :1; // CTRL1<1>   Fast read mode (single byte read)
    uint8_t LNOISE   :1; // CTRL1<2>   Reduced noise, reduced maximum range
    uint8_t DR       :3; // CTRL1<5:3> Data-rate selection.
    uint8_t ASLP_RATE:2; // CTRL1<7:6> Data rate selection
};

struct mma845x_reg_ctrl2 
{
    uint8_t MODS  :2; // CTRL2<1:0> Active mode power scheme selection.
    uint8_t SLPE  :1; // CTRL2<2>   Auto-sleep enable
    uint8_t SMODS :2; // CTRL2<4:3> Sleep mode power scheme selection
    uint8_t unused:1; // CTRL2<5>   unused
    uint8_t RST   :1; // CTRL2<6>   Software reset
    uint8_t ST    :1; // CTRL2<7>   Self-test enable
};

struct mma845x_reg_ctrl3
{
    uint8_t PP_OD      :1; // CTRL3<0>   Push-pull/open drain interrupt pad
    uint8_t IPOL       :1; // CTRL3<1>   Interrupt polarity
    uint8_t unused     :1; // CTRL3<2>   unused
    uint8_t WAKE_FF_MT :1; // CTRL3<3>   Freefall/motion function wake up
    uint8_t WAKE_PULSE :1; // CTRL3<4>   Pulse function wake up
    uint8_t WAKE_LNDPRT:1; // CTRL3<5>   Orientatoin function wake up
    uint8_t WAKE_TRANS :1; // CTRL3<6>   Transient function wake up
    uint8_t FIFO_GATE  :1; // CTRL3<7>   FIFO gate handling in state transition

};

struct mma845x_reg_ctrl4
{
    uint8_t INT_EN_DRDY  :1; // CTRL4<0>   Data ready interrupt enable
    uint8_t unused       :1; // CTRL4<1>   unused
    uint8_t INT_EN_FF_MT :1; // CTRL4<2>   Freefall/motion interrupt enable
    uint8_t INT_EN_PULSE :1; // CTRL4<3>   Pulse interrupt enable
    uint8_t INT_EN_LNDPRT:1; // CTRL4<4>   Orientation interrupt enable
    uint8_t INT_EN_TRANS :1; // CTRL4<5>   Transient interrupt enable
    uint8_t INT_EN_FIFO  :1; // CTRL4<6>   FIFO interrupt enable
    uint8_t INT_EN_ASLP  :1; // CTRL4<7>   Auto-sleep/wake interrupt enable

};

struct mma845x_reg_ctrl5
{
    uint8_t INT_CFG_DRDY  :1; // CTRL5<0>   Data ready interrupt configuration
    uint8_t unused        :1; // CTRL5<1>   unused
    uint8_t INT_CFG_FF_MT :1; // CTRL5<2>   Freefall/motion interrupt configuration
    uint8_t INT_CFG_PULSE :1; // CTRL5<3>   Pulse interrupt configuration
    uint8_t INT_CFG_LNDPRT:1; // CTRL5<4>   Orientation interrupt configuration
    uint8_t INT_CFG_TRANS :1; // CTRL5<5>   Transient interrupt configuration
    uint8_t INT_CFG_FIFO  :1; // CTRL5<6>   FIFO interrupt configuration
    uint8_t INT_CFG_ASLP  :1; // CTRL5<7>   Auto-sleep/wake interrupt configuration
};

struct mma845x_reg_f_setup
{
    uint8_t F_WMRK:6; // FIFO_CTRL<5:0> FIFO event sample count watermark
    uint8_t F_MODE:2; // FIFO_CTRL<7:6> FIFO buffer overflow mode
};

struct mma845x_reg_trig_cfg
{
    uint8_t unused1    :2; // TRIG_CFG<1:0> unused
    uint8_t Trig_FF_MT :1; // TRIG_CFG<2>   Free fall/motion interrupt trigger bit
    uint8_t Trig_PULSE :1; // TRIG_CFG<3>   Pulse interrupt trigger bit
    uint8_t Trig_LNDPRT:1; // TRIG_CFG<4>   Orientiration interrupt trigger bit
    uint8_t Trig_TRANS :1; // TRIG_CFG<5>   Transient interrupt trigger bit
    uint8_t unused2    :2; // TRIG_CFG<7:6> unused
};

struct mma845x_reg_sysmod
{
    uint8_t SYSMOD:2; // SYSMOD<1:0> System mode
    uint8_t FGT   :5; // SYSMOD<6:2> Number of ODR units since FGERR asserted
    uint8_t FGERR :1; // SYSMOD<7>   FIFO gate erro 
};

struct mma845x_reg_int_source
{
    uint8_t SRC_DRDY  :1; // INT_SOURCE<0>   Data ready interrupt
    uint8_t unused    :1; // INT_SOURCE<1>   unused
    uint8_t SRC_FF_MT :1; // INT_SOURCE<2>   Freefall/motion interrupt
    uint8_t SRC_PULSE :1; // INT_SOURCE<3>   Pulse interrupt
    uint8_t SRC_LNDPRT:1; // INT_SOURCE<4>   Orientation interrupt
    uint8_t SRC_TRANS :1; // INT_SOURCE<5>   Transient interrupt
    uint8_t SRC_FIFO  :1; // INT_SOURCE<6>   FIFO interrupt
    uint8_t SRC_ASLP  :1; // INT_SOURCE<7>   Auto-sleep/wake interrupt
};

struct mma845x_reg_xyz_data_cfg
{
    uint8_t FS     :2; // XYZ_DATA_CFG<1:0> Output buffer data format full scale
    uint8_t unused1:2; // XYZ_DATA_CFG<3:2> unused
    uint8_t HPF_OUT:1; // XYZ_DATA_CFG<4>   Enable high-pass output data
    uint8_t unused2:3; // XYZ_DATA_CFG<7:5> unused
};

struct mma845x_reg_hp_filter_cutoff
{
    uint8_t SEL          :2; // HP_FILTER_CUTOFF<1:0> HPF cutoff frequency selection
    uint8_t unused1      :2; // HP_FILTER_CUTOFF<0>   unused
    uint8_t Pulse_LPF_EN :1; // HP_FILTER_CUTOFF<0>   Enable low-pass filter (LPF) for pulse function
    uint8_t Pulse_HPF_BYP:1; // HP_FILTER_CUTOFF<0>   Bypass high-pass filter (HPF) for pulse function
    uint8_t unused2      :2; // HP_FILTER_CUTOFF<0>   unused
};

struct mma845x_reg_pulse_cfg
{
    uint8_t XSPEFE:1; // PULSE_CFG<0>   Single pulse event enable on X-axis
    uint8_t XDPEFE:1; // PULSE_CFG<1>   Double pulse event enable on X-axis
    uint8_t YSPEFE:1; // PULSE_CFG<2>   Single pulse event enable on Y-axis
    uint8_t YDPEFE:1; // PULSE_CFG<3>   Double pulse event enable on Y-axis
    uint8_t ZSPEFE:1; // PULSE_CFG<4>   Single pulse event enable on Z-axis
    uint8_t ZDPEFE:1; // PULSE_CFG<5>   Double pulse event enable on Z-axis
    uint8_t ELE   :1; // PULSE_CFG<6>   Pulse event flags are latched
    uint8_t DPA   :1; // PULSE_CFG<7>   Double-pulse abort
};

struct mma845x_reg_ff_mt_cfg
{
    uint8_t unused:3; // FF_MT_CFG<2:0> unused
    uint8_t XEFE  :1; // FF_MT_CFG<3>   Event flag enable on X event
    uint8_t YEFE  :1; // FF_MT_CFG<4>   Event flag enable on X event
    uint8_t ZEFE  :1; // FF_MT_CFG<5>   Event flag enable on X event
    uint8_t OAE   :1; // FF_MT_CFG<6>   Motion detect / freefall detect selection
    uint8_t ELE   :1; // FF_MT_CFG<6>   Event latch enable
};

struct mma845x_reg_ff_mt_ths
{
    uint8_t THS   :7; // FF_MT_CFG<6:0> Freefall/motion threshold
    uint8_t DBCNTM:1; // FF_MT_CFG<7>   Debounce counter mode selection
};

struct mma845x_reg_pl_status
{
    uint8_t BAFRO :1; // PL_STATUS<0>   Back or front orientation
    uint8_t LAPO  :2; // PL_STATUS<2:1> Landscape/portrait orientation 
    uint8_t unused:3; // PL_STATUS<5:3> unused
    uint8_t LO    :1; // PL_STATUS<6>   Z-tilt angle lockout
    uint8_t NEWLP :1; // PL_STATUS<7>   Landscape/portrait status change flag
};

struct mma845x_reg_pl_cfg
{
    uint8_t unused:6; // PL_CFG<5:0> unused
    uint8_t PL_EN :1; // PL_CFG<6>   Portrait/landscape detection enable
    uint8_t DBCNTM:1; // PL_CFG<7>   Debounce counter mode selection
};

struct mma845x_reg_pl_bf_zcomp
{
    uint8_t ZLOCK :3; // PL_BF_ZCOMP<2:0> Z-lock angle threshold
    uint8_t unused:3; // PL_BF_ZCOMP<5:3> unused
    uint8_t BKFR  :2; // PL_BF_ZCOMP<7:6> Back/front trip angle threshold
};

struct mma845x_reg_pl_ths
{
    uint8_t HYS   :3; // PL_THS<2:0> added to the threshold angle for smoother transition
    uint8_t PL_THS:5; // PL_THS<7:3> Portrait/landscape trip threshold angle
};

struct mma845x_reg_transient_cfg
{
    uint8_t HPF_BYP:1; // TRANSIENT_CFG<0>   Bypass high-pass filter
    uint8_t XTEFE  :1; // TRANSIENT_CFG<1>   X transient event flag enable
    uint8_t YTEFE  :1; // TRANSIENT_CFG<2>   Y transient event flag enable
    uint8_t ZTEFE  :1; // TRANSIENT_CFG<3>   Z transient event flag enable
    uint8_t ELE    :1; // TRANSIENT_CFG<4>   Transient event flags are latched
    uint8_t unused :3; // TRANSIENT_CFG<7:5> unused
};

struct mma845x_reg_transient_ths
{
    uint8_t THS   :7; // TRANSIENT_CFG<6:0> transient event threshold
    uint8_t DBCNTM:1; // TRANSIENT_CFG<7>   Debounce counter mode selection
};


/** Forward declaration of functions for internal use */

static bool    mma845x_reset       (mma845x_sensor_t* dev);
static bool    mma845x_is_available(mma845x_sensor_t* dev);

static bool    mma845x_i2c_read    (mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len);
static bool    mma845x_i2c_write   (mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len);

#define msb_lsb_to_type(t,b,o) (t)(((t)b[o] << 8) | b[o+1])
#define lsb_msb_to_type(t,b,o) (t)(((t)b[o+1] << 8) | b[o])
#define lsb_to_type(t,b,o)     (t)(b[o])

#define mma845x_update_reg(dev,addr,type,elem,value) \
        { \
            struct type __reg; \
            if (!mma845x_reg_read (dev, (addr), (uint8_t*)&__reg, 1)) \
                return false; \
            __reg.elem = (value); \
            if (!mma845x_reg_write (dev, (addr), (uint8_t*)&__reg, 1)) \
                return false; \
        }

mma845x_sensor_t* mma845x_init_sensor (uint8_t bus, uint8_t addr)
{
    mma845x_sensor_t* dev;

    if ((dev = malloc (sizeof(mma845x_sensor_t))) == NULL)
        return NULL;

    // init sensor data structure
    dev->bus    = bus;
    dev->addr   = addr;

    dev->error_code = MMA845X_OK;
    dev->scale      = mma845x_scale_2_g;
    dev->fifo_mode  = mma845x_disabled;
    dev->active     = false;
    dev->fast_read  = false;
    
    // check availability of the sensor
    if (!mma845x_is_available (dev))
    {
        error_dev ("Sensor is not available.", __FUNCTION__, dev);
        free (dev);
        return NULL;
    }

    // reset the sensor
    if (!mma845x_reset(dev))
    {
        error_dev ("Could not reset the sensor device.", __FUNCTION__, dev);
        free (dev);
        return NULL;
    }
    
    return dev;
}

bool mma845x_set_mode (mma845x_sensor_t* dev,
                       mma845x_oversampling_mode_t mode, mma845x_odr_t odr,
                       bool low_noise, bool fast_read)
{
    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_ctrl1 ctrl1;
    struct mma845x_reg_ctrl2 ctrl2;
    
    // read current register values
    if (!mma845x_reg_read (dev, MMA845X_REG_CTRL1, (uint8_t*)&ctrl1, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL2, (uint8_t*)&ctrl2, 1))
        return false;
    
    dev->fast_read = fast_read;
    dev->active  = true;
    
    ctrl1.ACTIVE = true; // switch to active mode
    ctrl1.DR     = odr;
    ctrl1.LNOISE = low_noise;
    ctrl1.F_READ = fast_read;
    ctrl2.MODS   = mode;
    
    // write back register values
    if (!mma845x_reg_write (dev, MMA845X_REG_CTRL1, (uint8_t*)&ctrl1, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_CTRL2, (uint8_t*)&ctrl2, 1))
        return false;

    // wait 150 ms
    vTaskDelay (150/portTICK_PERIOD_MS);

    return false;
}


bool mma845x_set_autosleep (mma845x_sensor_t* dev, 
                            mma845x_oversampling_mode_t mode, 
                            mma845x_aslp_rate_t rate,
                            uint8_t count, bool activate)
{
    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_ctrl1 ctrl1;
    struct mma845x_reg_ctrl2 ctrl2;
    
    // read current register values
    if (!mma845x_reg_read (dev, MMA845X_REG_CTRL1, (uint8_t*)&ctrl1, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL2, (uint8_t*)&ctrl2, 1))
        return false;
        
    ctrl1.ACTIVE    = dev->active; // switch back to active mode if necessary
    ctrl1.ASLP_RATE = activate ? rate : 0;
    ctrl2.SLPE      = activate;
    ctrl2.SMODS     = mode;
    
    // write back register values
    if (!mma845x_reg_write (dev, MMA845X_REG_CTRL1, (uint8_t*)&ctrl1, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_CTRL2, (uint8_t*)&ctrl2, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_ASLP_COUNT, &count, 1))
        return false;

    // wait 150 ms
    vTaskDelay (150/portTICK_PERIOD_MS);

    return false;
}


mma845x_system_mode_t mma845x_get_system_mode (mma845x_sensor_t* dev)
{
    if (!dev) return mma845x_unknown_mode;

    struct mma845x_reg_sysmod sysmod;
    
    // read current register values
    if (!mma845x_reg_read (dev, MMA845X_REG_SYSMOD, (uint8_t*)&sysmod, 1))
        return mma845x_unknown_mode;
    
    return sysmod.SYSMOD;
}



bool mma845x_set_scale (mma845x_sensor_t* dev, mma845x_scale_t scale)
{
    if (!dev) return false;
    
    dev->error_code = MMA845X_OK;
    dev->scale = scale;
    
    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    // read current register value and write scale
    mma845x_update_reg (dev, MMA845X_REG_XYZ_DATA_CFG, mma845x_reg_xyz_data_cfg, FS, scale);

    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);
    
    return true;
}


bool mma845x_set_fifo_mode (mma845x_sensor_t* dev, 
                            mma845x_fifo_mode_t mode, uint8_t watermark, 
                            mma845x_fifo_trigger_t trigger, bool block)
{
    if (!dev) return false;
    
    if (dev->id != mma8451q_id)
    {
        error_dev ("FIFO is not by this sensor", __FUNCTION__, dev);
        dev->error_code = MMA845X_NOT_SUPPORTED;
        return false;
    }
    
    dev->error_code = MMA845X_OK;
    dev->fifo_mode  = mode;
    
    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_f_setup  f_setup;
    struct mma845x_reg_trig_cfg f_trigger = {};
    
    f_setup.F_MODE = mode;
    f_setup.F_WMRK = watermark;
    
    f_trigger.Trig_FF_MT  = trigger.free_fall_motion;
    f_trigger.Trig_PULSE  = trigger.pulse;
    f_trigger.Trig_LNDPRT = trigger.orientation;
    f_trigger.Trig_TRANS  = trigger.transient;
    
    // write F_SETUP register
    if (!mma845x_reg_write (dev, MMA845X_REG_F_SETUP , (uint8_t*)&f_setup  , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_TRIG_CFG, (uint8_t*)&f_trigger, 1))
        return false;
    
    mma845x_update_reg (dev, MMA845X_REG_CTRL3, mma845x_reg_ctrl3, FIFO_GATE, block);
    
    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


bool mma845x_new_data (mma845x_sensor_t* dev)
{
    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    struct mma845x_reg_status   status;
    struct mma845x_reg_f_status f_status;
    
    if (dev->fifo_mode == mma845x_disabled)
    {
        if (!mma845x_reg_read (dev, MMA845X_REG_STATUS, (uint8_t*)&status, 1))
        {
            error_dev ("Could not read sensor status register", __FUNCTION__, dev);
            return false;
        }
        return status.ZYXDR;
    }
    else
    {
        if (!mma845x_reg_read (dev, MMA845X_REG_F_STATUS, (uint8_t*)&f_status, 1))
        {
            error_dev ("Could not read sensor FIFO status register", __FUNCTION__, dev);
            return false;
        }
        return f_status.F_CNT;
    }
}

/**
 * Scaling factors for the conversion of raw sensor data to floating point g
 * values. Scaling factors are from mechanical characteristics in datasheet.
 *
 *  scale/sensitivity  resolution
 *       +-2g           4096 counts/g
 *       +-4g           2048 counts/g
 *       +-8g           1024 counts/g
 */
const static double  MMA845X_SCALES[4] = { 1.0/4096, 1.0/2048, 1.0/1024 };

bool mma845x_get_float_data (mma845x_sensor_t* dev, mma845x_float_data_t* data)
{
    if (!dev || !data) return false;

    mma845x_raw_data_t raw;
    
    if (!mma845x_get_raw_data (dev, &raw))
        return false;

    data->ax = MMA845X_SCALES[dev->scale] * (raw.ax >> 2);
    data->ay = MMA845X_SCALES[dev->scale] * (raw.ay >> 2);
    data->az = MMA845X_SCALES[dev->scale] * (raw.az >> 2);

    return true;
}


uint8_t mma845x_get_float_data_fifo (mma845x_sensor_t* dev, mma845x_float_data_fifo_t data)
{
    if (!dev || !data) return false;

    mma845x_raw_data_fifo_t raw;
    
    uint8_t num = mma845x_get_raw_data_fifo (dev, raw);

    for (int i = 0; i < num; i++)
    {
        data[i].ax = MMA845X_SCALES[dev->scale] * (raw[i].ax >> 2);
        data[i].ay = MMA845X_SCALES[dev->scale] * (raw[i].ay >> 2);
        data[i].az = MMA845X_SCALES[dev->scale] * (raw[i].az >> 2);
    }
    return num;
}


bool mma845x_get_raw_data (mma845x_sensor_t* dev, mma845x_raw_data_t* raw)
{
    if (!dev || !raw) return false;

    dev->error_code = MMA845X_OK;

    uint8_t data[6];
    
    if (!mma845x_reg_read (dev, MMA845X_REG_OUT_X_MSB, data, (dev->fast_read) ? 3 : 6))
    {
        error_dev ("Could not get raw data sample", __FUNCTION__, dev);
        dev->error_code |= MMA845X_GET_RAW_DATA_FAILED;
        return false;
    }

    if (dev->fast_read)
    {
        raw->ax = (uint16_t)data[0] << 8;
        raw->ay = (uint16_t)data[1] << 8;
        raw->az = (uint16_t)data[2] << 8;
    }
    else
    {
        raw->ax = ((uint16_t)data[0] << 8) | data[1];
        raw->ay = ((uint16_t)data[2] << 8) | data[3];
        raw->az = ((uint16_t)data[4] << 8) | data[5];
    }
    
    return true;
}


uint8_t mma845x_get_raw_data_fifo (mma845x_sensor_t* dev, mma845x_raw_data_fifo_t raw)
{
    if (!dev) return 0;

    dev->error_code = MMA845X_OK;

    // if FIFO disabled, use mma845x_get_raw_data to return one sample
    if (dev->fifo_mode == mma845x_disabled)
        return mma845x_get_raw_data (dev, raw) ? 1 : 0;
        
    struct mma845x_reg_f_status f_status;
    
    // read FIFO state
    if (!mma845x_reg_read (dev, MMA845X_REG_F_STATUS, (uint8_t*)&f_status, 1))
    {
        error_dev ("Could not read FIFO satus register", __FUNCTION__, dev);
        return 0;
    }

    // if nothing is in the FIFO, just return with 0
    if (!f_status.F_CNT)
        return 0;

    uint8_t samples = f_status.F_CNT;

    // read F_CNT samples from FIFO
    for (int i = 0; i < samples; i++)
        if (!mma845x_get_raw_data (dev, &raw[i]))
        {
            error_dev ("Could not get raw data samples", __FUNCTION__, dev);
            dev->error_code |= MMA845X_GET_RAW_DATA_FIFO_FAILED;
            return i;
        }

    mma845x_reg_read (dev, MMA845X_REG_F_STATUS, (uint8_t*)&f_status, 1);
    
    // if F_CNT is not 0 after all samples read, ODR is higher than fetching rate
    if (f_status.F_CNT)
    {
        dev->error_code = MMA845X_ODR_TOO_HIGH;
        error_dev ("New samples stored in FIFO while reading, "
                   "output data rate (ODR) is too high", __FUNCTION__, dev);
    }

    return samples;
}


bool mma845x_enable_int (mma845x_sensor_t* dev, 
                         mma845x_int_type_t type, 
                         mma845x_int_signal_t signal, bool value)
{
    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_ctrl4  ctrl4;
    struct mma845x_reg_ctrl5  ctrl5;

    // read current register values
    if (!mma845x_reg_read (dev, MMA845X_REG_CTRL4, (uint8_t*)&ctrl4, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL5, (uint8_t*)&ctrl5, 1))
    {   
        error_dev ("Could not read interrupt control registers", __FUNCTION__, dev);
        dev->error_code |= MMA845X_INT_CONFIG_FAILED;
        return false;
    }

    bool cfg = (signal == mma845x_int1_signal);
    
    // change the register
    switch (type)
    {
        case mma845x_int_data_ready:    ctrl4.INT_EN_DRDY  = value;
                                        ctrl5.INT_CFG_DRDY = cfg;
                                        break;
                                        
        case mma845x_int_fifo_watermark:                                        
        case mma845x_int_fifo_overrun:  ctrl4.INT_EN_FIFO  = value;
                                        ctrl5.INT_CFG_FIFO = cfg;
                                        break;
                                        
        case mma845x_int_transient:     ctrl4.INT_EN_TRANS  = value;
                                        ctrl5.INT_CFG_TRANS = cfg;
                                        break;
                                        
        case mma845x_int_orientation:   ctrl4.INT_EN_LNDPRT  = value;
                                        ctrl5.INT_CFG_LNDPRT = cfg;
                                        break;
                                        
        case mma845x_int_event:         ctrl4.INT_EN_FF_MT  = value;
                                        ctrl5.INT_CFG_FF_MT = cfg;
                                        break;
                                        
        case mma845x_int_pulse:         ctrl4.INT_EN_PULSE  = value;
                                        ctrl5.INT_CFG_PULSE = cfg;
                                        break;
                      
        case mma845x_int_autosleep:     ctrl4.INT_EN_ASLP  = value;
                                        ctrl5.INT_CFG_ASLP = cfg;
                                        break;
                      
        default: dev->error_code = MMA845X_INT_WRONG_TYPE; 
                 error_dev ("Wrong interrupt type", __FUNCTION__, dev);
                 return false;
    }        
    
    if (!mma845x_reg_write (dev, MMA845X_REG_CTRL4, (uint8_t*)&ctrl4, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_CTRL5, (uint8_t*)&ctrl5, 1))
    {   
        error_dev ("Could not enable/disable interrupt", __FUNCTION__, dev);
        dev->error_code |= MMA845X_INT_CONFIG_FAILED;
        return false;
    }
    
    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


bool mma845x_get_int_status(mma845x_sensor_t* dev, 
                            mma845x_int_status_t* status)
{
    if (!dev || !status) return false;

    dev->error_code = MMA845X_OK;

    struct mma845x_reg_int_source int_source;

    if (!mma845x_reg_read (dev, MMA845X_REG_INT_SOURCE, (uint8_t*)&int_source, 1))
    {   
        error_dev ("Could not read source of interrupt from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_INT_SOURCE_FAILED;
        return false;
    }

    status->data_ready = int_source.SRC_DRDY;
    status->fifo       = int_source.SRC_FIFO;
    status->event      = int_source.SRC_FF_MT;
    status->transient  = int_source.SRC_TRANS;
    status->pulse      = int_source.SRC_PULSE;
    status->orientation= int_source.SRC_LNDPRT;
    status->autosleep  = int_source.SRC_ASLP;

    return true;
}

                       
bool mma845x_get_int_data_source (mma845x_sensor_t* dev, 
                                  mma845x_int_data_source_t* source)
{
    if (!dev || !source) return false;

    dev->error_code = MMA845X_OK;

    struct mma845x_reg_int_source int_source;
    struct mma845x_reg_f_status   f_status;

    if (!mma845x_reg_read (dev, MMA845X_REG_INT_SOURCE, (uint8_t*)&int_source, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_F_STATUS, (uint8_t*)&f_status, 1))
    {   
        error_dev ("Could not read source of interrupt from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_DATA_SOURCE_FAILED;
        return false;
    }

    source->data_ready     = int_source.SRC_DRDY;
    source->fifo_watermark = int_source.SRC_FIFO & f_status.F_WMRK_FLAG;
    source->fifo_overrun   = int_source.SRC_FIFO & f_status.F_OVF;

    return true;
}

bool mma845x_set_int_event_config (mma845x_sensor_t* dev,
                                   mma845x_int_event_config_t* config)
{
    if (!dev || !config) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_ff_mt_cfg ff_mt_cfg;
    struct mma845x_reg_ff_mt_ths ff_mt_ths;

    ff_mt_cfg.XEFE  = config->x_enabled;
    ff_mt_cfg.YEFE  = config->y_enabled;
    ff_mt_cfg.ZEFE  = config->z_enabled;
    ff_mt_cfg.OAE   = config->mode;
    ff_mt_cfg.ELE   = config->latch;
    ff_mt_ths.THS   = config->threshold;
    ff_mt_ths.DBCNTM= config->debounce_cnt_clr;

    if (!mma845x_reg_write (dev, MMA845X_REG_FF_MT_CFG  , (uint8_t*)&ff_mt_cfg , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_FF_MT_THS  , (uint8_t*)&ff_mt_ths , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_FF_MT_COUNT, &config->debounce_cnt, 1))
    {   
        error_dev ("Could not write event interrupt configuration to sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_EVENT_CONFIG_FAILED;
        return false;
    }

    mma845x_update_reg (dev, MMA845X_REG_CTRL3, mma845x_reg_ctrl3, WAKE_FF_MT, config->sleep_active);
    
    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


bool mma845x_get_int_event_config (mma845x_sensor_t* dev,
                                   mma845x_int_event_config_t* config)
{
    if (!dev || !config) return false;

    dev->error_code = MMA845X_OK;

    struct mma845x_reg_ff_mt_cfg ff_mt_cfg;
    struct mma845x_reg_ff_mt_ths ff_mt_ths;
    struct mma845x_reg_ctrl3     ctrl3;

    if (!mma845x_reg_read (dev, MMA845X_REG_FF_MT_CFG  , (uint8_t*)&ff_mt_cfg , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_FF_MT_THS  , (uint8_t*)&ff_mt_ths , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_FF_MT_COUNT, &config->debounce_cnt, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
    {   
        error_dev ("Could not read event interrupt configuration from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_EVENT_CONFIG_FAILED;
        return false;
    }

    config->x_enabled = ff_mt_cfg.XEFE;
    config->y_enabled = ff_mt_cfg.YEFE;
    config->z_enabled = ff_mt_cfg.ZEFE;

    config->mode  = ff_mt_cfg.OAE;
    config->latch = ff_mt_cfg.ELE;

    config->threshold = ff_mt_ths.THS;
    config->debounce_cnt_clr = ff_mt_ths.DBCNTM;
    
    config->sleep_active = ctrl3.WAKE_FF_MT;

    return true;
}


bool mma845x_get_int_event_source (mma845x_sensor_t* dev,
                                   mma845x_int_event_source_t* source)
{
    if (!dev || !source) return false;

    dev->error_code = MMA845X_OK;

    if (!mma845x_reg_read (dev, MMA845X_REG_FF_MT_SRC, (uint8_t*)source, 1))
    {   
        error_dev ("Could not read the pulse interrupt source from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_EVENT_SOURCE_FAILED;
        return false;
    }

    return true;
}


bool mma845x_set_int_transient_config (mma845x_sensor_t* dev,
                                       mma845x_int_transient_config_t* config)
{
    if (!dev || !config) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_transient_cfg transient_cfg;
    struct mma845x_reg_transient_ths transient_ths;
    struct mma845x_reg_ctrl3         ctrl3;

    // read current register value
    if (!mma845x_reg_read (dev, MMA845X_REG_TRANSIENT_CFG, (uint8_t*)&transient_cfg, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
        return false;
        
    transient_cfg.XTEFE   = config->x_enabled;
    transient_cfg.YTEFE   = config->y_enabled;
    transient_cfg.ZTEFE   = config->z_enabled;
    transient_cfg.ELE     = config->latch;
    transient_cfg.HPF_BYP = config->hpf_bypassed;
    transient_ths.THS     = config->threshold;
    transient_ths.DBCNTM  = config->debounce_cnt_clr;

    ctrl3.WAKE_TRANS = config->sleep_active;
    
    if (!mma845x_reg_write (dev, MMA845X_REG_TRANSIENT_CFG  , (uint8_t*)&transient_cfg , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_TRANSIENT_THS  , (uint8_t*)&transient_ths , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_TRANSIENT_COUNT, &config->debounce_cnt, 1)||
        !mma845x_reg_write (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
    {   
        error_dev ("Could not write event interrupt configuration to sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_TRANSIENT_CONFIG_FAILED;
        return false;
    }

    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


bool mma845x_get_int_transient_config (mma845x_sensor_t* dev,
                                       mma845x_int_transient_config_t* config)
{
    if (!dev || !config) return false;

    dev->error_code = MMA845X_OK;

    struct mma845x_reg_transient_cfg transient_cfg;
    struct mma845x_reg_transient_ths transient_ths;
    struct mma845x_reg_ctrl3         ctrl3;

    if (!mma845x_reg_read (dev, MMA845X_REG_TRANSIENT_CFG  , (uint8_t*)&transient_cfg , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_TRANSIENT_THS  , (uint8_t*)&transient_ths , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_TRANSIENT_COUNT, &config->debounce_cnt, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
    {   
        error_dev ("Could not read event interrupt configuration from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_TRANSIENT_CONFIG_FAILED;
        return false;
    }

    config->x_enabled = transient_cfg.XTEFE;
    config->y_enabled = transient_cfg.YTEFE;
    config->z_enabled = transient_cfg.ZTEFE;

    config->threshold = transient_ths.THS;
    config->debounce_cnt_clr = transient_ths.DBCNTM;

    config->latch = transient_cfg.ELE;
    config->hpf_bypassed = transient_cfg.HPF_BYP;

    config->sleep_active = ctrl3.WAKE_TRANS;

    return true;
}


bool mma845x_get_int_transient_source (mma845x_sensor_t* dev,
                                       mma845x_int_transient_source_t* source)
{
    if (!dev || !source) return false;

    dev->error_code = MMA845X_OK;

    if (!mma845x_reg_read (dev, MMA845X_REG_TRANSIENT_SRC, (uint8_t*)source, 1))
    {   
        error_dev ("Could not read the pulse interrupt source from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_TRANSIENT_SOURCE_FAILED;
        return false;
    }

    return true;
}


bool mma845x_set_int_pulse_config (mma845x_sensor_t* dev,
                                   mma845x_int_pulse_config_t* config)
{
    if (!dev || !config) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_pulse_cfg        pulse_cfg;
    struct mma845x_reg_hp_filter_cutoff filter_cfg;
    struct mma845x_reg_ctrl3            ctrl3;

    // read current register values
    if (!mma845x_reg_read (dev, MMA845X_REG_HP_FILTER_CUTOFF, (uint8_t*)&filter_cfg, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
        return false;

    pulse_cfg.XSPEFE = config->x_single;
    pulse_cfg.XDPEFE = config->x_double;

    pulse_cfg.YSPEFE = config->y_single;
    pulse_cfg.YDPEFE = config->y_double;

    pulse_cfg.ZSPEFE = config->z_single;
    pulse_cfg.ZDPEFE = config->z_double;

    config->x_threshold &= 0x7f;
    config->y_threshold &= 0x7f;
    config->z_threshold &= 0x7f;

    pulse_cfg.DPA = config->pulse_abort;
    pulse_cfg.ELE = config->latch;

    filter_cfg.Pulse_LPF_EN  = config->lpf_enabled;
    filter_cfg.Pulse_HPF_BYP = config->hpf_bypassed;
    
    ctrl3.WAKE_PULSE = config->sleep_active;
    
    if (!mma845x_reg_write (dev, MMA845X_REG_PULSE_CFG , (uint8_t*)&pulse_cfg, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PULSE_THSX, (uint8_t*)&config->x_threshold, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PULSE_THSY, (uint8_t*)&config->y_threshold, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PULSE_THSZ, (uint8_t*)&config->z_threshold, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PULSE_TMLT, (uint8_t*)&config->pulse_limit  , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PULSE_LTCY, (uint8_t*)&config->pulse_latency, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PULSE_WIND, (uint8_t*)&config->pulse_window , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_HP_FILTER_CUTOFF, (uint8_t*)&filter_cfg, 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
    {   
        error_dev ("Could not write pulse detection configuration to sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_PULSE_CONFIG_FAILED;
        return false;
    }
    

    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}

bool mma845x_get_int_pulse_config (mma845x_sensor_t* dev,
                                   mma845x_int_pulse_config_t* config)
{
    if (!dev || !config) return false;

    dev->error_code = MMA845X_OK;

    struct mma845x_reg_pulse_cfg        pulse_cfg;
    struct mma845x_reg_hp_filter_cutoff filter_cfg;
    struct mma845x_reg_ctrl3            ctrl3;
    
    if (!mma845x_reg_read (dev, MMA845X_REG_PULSE_CFG , (uint8_t*)&pulse_cfg, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PULSE_THSX, (uint8_t*)&config->x_threshold, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PULSE_THSY, (uint8_t*)&config->y_threshold, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PULSE_THSZ, (uint8_t*)&config->z_threshold, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PULSE_TMLT, (uint8_t*)&config->pulse_limit  , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PULSE_LTCY, (uint8_t*)&config->pulse_latency, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PULSE_WIND, (uint8_t*)&config->pulse_window , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_HP_FILTER_CUTOFF, (uint8_t*)&filter_cfg, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
    {   
        error_dev ("Could not read pulse detection configuration from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_PULSE_CONFIG_FAILED;
        return false;
    }
    
    config->x_single = pulse_cfg.XSPEFE;
    config->x_double = pulse_cfg.XDPEFE;

    config->y_single = pulse_cfg.YSPEFE;
    config->y_double = pulse_cfg.YDPEFE;

    config->z_single = pulse_cfg.ZSPEFE;
    config->z_double = pulse_cfg.ZDPEFE;
 
    config->pulse_abort = pulse_cfg.DPA;
    config->latch       = pulse_cfg.ELE;
    
    config->lpf_enabled  = filter_cfg.Pulse_LPF_EN;
    config->hpf_bypassed = filter_cfg.Pulse_HPF_BYP;
    
    config->sleep_active = ctrl3.WAKE_PULSE;
    
    return true;
}

bool mma845x_get_int_pulse_source (mma845x_sensor_t* dev,
                                   mma845x_int_pulse_source_t* source)
{
    if (!dev || !source) return false;

    dev->error_code = MMA845X_OK;

    if (!mma845x_reg_read (dev, MMA845X_REG_PULSE_SRC, (uint8_t*)source, 1))
    {   
        error_dev ("Could not read the pulse interrupt source from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_PULSE_SOURCE_FAILED;
        return false;
    }

    return true;
}                                     


bool mma845x_set_orientation_config (mma845x_sensor_t* dev,
                                     mma845x_orientation_config_t* config)
{
    if (!dev || !config) return false;

    if (dev->id != mma8451q_id)
    {
        error_dev ("Orientation detection not supported by this sensor", __FUNCTION__, dev);
        dev->error_code = MMA845X_NOT_SUPPORTED;
        return false;
    }
    
    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    struct mma845x_reg_pl_cfg      pl_cfg;
    struct mma845x_reg_pl_ths      pl_ths;
    struct mma845x_reg_pl_bf_zcomp pl_zcomp;

    pl_cfg.PL_EN   = config->enabled;
    pl_cfg.DBCNTM  = config->debounce_cnt_clr;
    pl_ths.PL_THS  = config->pl_threshold;
    pl_ths.HYS     = config->pl_hysteresis;
    pl_zcomp.BKFR  = config->bf_threshold;
    pl_zcomp.ZLOCK = config->z_lock;

    if (!mma845x_reg_write (dev, MMA845X_REG_PL_CFG     , (uint8_t*)&pl_cfg    , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PL_THS     , (uint8_t*)&pl_ths    , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PL_BF_ZCOMP, (uint8_t*)&pl_zcomp  , 1) ||
        !mma845x_reg_write (dev, MMA845X_REG_PL_COUNT   , &config->debounce_cnt, 1))
    {   
        error_dev ("Could not write orientation detection configuration to sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_ORIENT_CONFIG_FAILED;
        return false;
    }
        
    mma845x_update_reg (dev, MMA845X_REG_CTRL3, mma845x_reg_ctrl3, WAKE_LNDPRT, config->sleep_active);

    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


bool mma845x_get_orientation_config (mma845x_sensor_t* dev,
                                     mma845x_orientation_config_t* config)
{
    if (!dev || !config) return false;

    if (dev->id != mma8451q_id)
    {
        error_dev ("Orientation detection not supported by this sensor", __FUNCTION__, dev);
        dev->error_code = MMA845X_NOT_SUPPORTED;
        return false;
    }
    
    dev->error_code = MMA845X_OK;

    struct mma845x_reg_pl_cfg      pl_cfg;
    struct mma845x_reg_pl_ths      pl_ths;
    struct mma845x_reg_pl_bf_zcomp pl_zcomp;
    struct mma845x_reg_ctrl3       ctrl3;

    if (!mma845x_reg_read (dev, MMA845X_REG_PL_CFG     , (uint8_t*)&pl_cfg    , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PL_THS     , (uint8_t*)&pl_ths    , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PL_BF_ZCOMP, (uint8_t*)&pl_zcomp  , 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_PL_COUNT   , &config->debounce_cnt, 1) ||
        !mma845x_reg_read (dev, MMA845X_REG_CTRL3, (uint8_t*)&ctrl3, 1))
    {
        error_dev ("Could not read orientation detection configuration from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_ORIENT_CONFIG_FAILED;
        return false;
    }

    config->debounce_cnt_clr = pl_cfg.DBCNTM;
       
    config->enabled       = pl_cfg.PL_EN;
    config->pl_threshold  = pl_ths.PL_THS;
    config->pl_hysteresis = pl_ths.HYS;
    config->bf_threshold  = pl_zcomp.BKFR;
    config->z_lock        = pl_zcomp.ZLOCK;
    config->sleep_active  = ctrl3.WAKE_LNDPRT;
    
    return true;
}


bool mma845x_get_orientation (mma845x_sensor_t* dev,
                              mma845x_orientation_status_t* status)
{
    if (!dev || !status) return false;

    if (dev->id != mma8451q_id)
    {
        error_dev ("Orientation detection not supported by this sensor", __FUNCTION__, dev);
        dev->error_code = MMA845X_NOT_SUPPORTED;
        return false;
    }
    
    dev->error_code = MMA845X_OK;

    uint8_t data[2];

    if (!mma845x_reg_read (dev, MMA845X_REG_PL_STATUS, data, 2))
    {   
        error_dev ("Could not read orientation status from sensor", __FUNCTION__, dev);
        dev->error_code |= MMA845X_ORIENT_SOURCE_FAILED;
        return false;
    }

    struct mma845x_reg_pl_status* pl_status = (struct mma845x_reg_pl_status*)&data[0];
    struct mma845x_reg_pl_cfg*    pl_config = (struct mma845x_reg_pl_cfg*)&data[1];
   
    if (!pl_config->PL_EN)
    {
        error_dev ("Orientation detection not enabled", __FUNCTION__, dev);
        dev->error_code |= MMA845X_ORIENT_NOT_ENABLED;
        return false;
    }
    
    status->changed = pl_status->NEWLP;
    status->lockout = pl_status->LO;
    status->portrait_landscape = pl_status->LAPO;
    status->back_front = pl_status->BAFRO;

    return true;
} 


bool mma845x_config_int_signals (mma845x_sensor_t* dev, 
                                 mma845x_int_signal_level_t level,
                                 mma845x_int_signal_type_t type)
{
    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    mma845x_update_reg (dev, MMA845X_REG_CTRL3, mma845x_reg_ctrl3, IPOL, level);
    mma845x_update_reg (dev, MMA845X_REG_CTRL3, mma845x_reg_ctrl3, PP_OD, type);
    
    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


bool mma845x_config_hpf (mma845x_sensor_t* dev, uint8_t cutoff, bool data)
{
    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);

    mma845x_update_reg (dev, MMA845X_REG_HP_FILTER_CUTOFF, mma845x_reg_hp_filter_cutoff, SEL, cutoff);
    mma845x_update_reg (dev, MMA845X_REG_XYZ_DATA_CFG , mma845x_reg_xyz_data_cfg , HPF_OUT, data);

    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


bool mma845x_set_offset (mma845x_sensor_t* dev, 
                         uint8_t x, uint8_t y, uint8_t z)
{
    // if in active mode, sensor has to switch in standby mode first
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 0);
    
    uint8_t data[3] = { x, y, z };
    
    if (!mma845x_reg_write (dev, MMA845X_REG_OFF_X, data, 3))
        return false;
    
    // if in active mode, sensor has to switch back from standby mode to active mode
    if (dev->active)
        mma845x_update_reg (dev, MMA845X_REG_CTRL1, mma845x_reg_ctrl1, ACTIVE, 1);

    return true;
}


/** Functions for internal use only */

/**
 * @brief   Check the chip ID to test whether sensor is available
 */
static bool mma845x_is_available (mma845x_sensor_t* dev)
{
    uint8_t chip_id;

    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    if (!mma845x_reg_read (dev, MMA845X_REG_WHO_AM_I, &chip_id, 1))
        return false;

    switch (chip_id)
    {
        case MMA8451Q_CHIP_ID: dev->id = mma8451q_id; break;
        case MMA8452Q_CHIP_ID: dev->id = mma8452q_id; break;
        case MMA8453Q_CHIP_ID: dev->id = mma8453q_id; break;
        default:
            error_dev ("Chip id %02x is wrong, should be %02x, %02x or %02x.",
                        __FUNCTION__, dev, chip_id, 
                        MMA8451Q_CHIP_ID, MMA8452Q_CHIP_ID, MMA8453Q_CHIP_ID);
            dev->error_code = MMA845X_WRONG_CHIP_ID;
            return false;
    }

    return true;
}

static bool mma845x_reset (mma845x_sensor_t* dev)
{
    if (!dev) return false;

    dev->error_code = MMA845X_OK;

    mma845x_update_reg (dev, MMA845X_REG_CTRL2, mma845x_reg_ctrl2, RST, 1);
    vTaskDelay (100/portTICK_PERIOD_MS);
    
    return true;
}


bool mma845x_reg_read(mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !data) return false;

    return mma845x_i2c_read (dev, reg, data, len);
}


bool mma845x_reg_write(mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !data) return false;

    return mma845x_i2c_write (dev, reg, data, len);
}

static bool mma845x_i2c_read(mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !data) return false;

    debug_dev ("Read %d byte from i2c slave register %02x.", __FUNCTION__, dev, len, reg);

    int result = i2c_slave_read(dev->bus, dev->addr, &reg, data, len);

    if (result)
    {
        dev->error_code |= (result == -EBUSY) ? MMA845X_I2C_BUSY : MMA845X_I2C_READ_FAILED;
        error_dev ("Error %d on read %d byte from I2C slave register %02x.",
                    __FUNCTION__, dev, result, len, reg);
        return false;
    }

#   ifdef MMA845X_DEBUG_LEVEL_2
    printf("MMA845X %s: Read following bytes: ", __FUNCTION__);
    printf("%02x: ", reg & 0x7f);
    for (int i=0; i < len; i++)
        printf("%02x ", data[i]);
    printf("\n");
#   endif

    return true;
}


static bool mma845x_i2c_write(mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !data) return false;

    debug_dev ("Write %d byte to i2c slave register %02x.", __FUNCTION__, dev, len, reg);

    int result = i2c_slave_write(dev->bus, dev->addr, &reg, data, len);

    if (result)
    {
        dev->error_code |= (result == -EBUSY) ? MMA845X_I2C_BUSY : MMA845X_I2C_WRITE_FAILED;
        error_dev ("Error %d on write %d byte to i2c slave register %02x.",
                    __FUNCTION__, dev, result, len, reg);
        return false;
    }

#   ifdef MMA845X_DEBUG_LEVEL_2
    printf("MMA845X %s: Wrote the following bytes: ", __FUNCTION__);
    printf("%02x: ", reg & 0x7f);
    for (int i=0; i < len; i++)
        printf("%02x ", data[i]);
    printf("\n");
#   endif

    return true;
}
