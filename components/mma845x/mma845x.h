/**
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
 */

#ifndef __MMA845X_H__
#define __MMA845X_H__

// Uncomment one of the following defines to enable debug output
// #define MMA845X_DEBUG_LEVEL_1    // only error messages
// #define MMA845X_DEBUG_LEVEL_2    // debug and error messages

// MMA845X addresses
#define MMA845X_I2C_ADDRESS_1           0x1c  // SDO pin is low
#define MMA845X_I2C_ADDRESS_2           0x1d  // SDO pin is high

// MMA845X chip ids
#define MMA8451Q_CHIP_ID                0x1a  // MMA845X_REG_WHO_AM_I<7:0>
#define MMA8452Q_CHIP_ID                0x2a  // MMA845X_REG_WHO_AM_I<7:0>
#define MMA8453Q_CHIP_ID                0x3a  // MMA845X_REG_WHO_AM_I<7:0>

// Definition of error codes
#define MMA845X_OK                      0
#define MMA845X_NOK                     -1

#define MMA845X_INT_ERROR_MASK          0x000f
#define MMA845X_DRV_ERROR_MASK          0xfff0

// Error codes for I2C interfaces ORed with MMA845X driver error codes
#define MMA845X_I2C_READ_FAILED         1
#define MMA845X_I2C_WRITE_FAILED        2
#define MMA845X_I2C_BUSY                3

// MMA845X driver error codes ORed with error codes for I2C interfaces
#define MMA845X_WRONG_CHIP_ID              ( 1 << 8)
#define MMA845X_WRONG_PARAMETER_VALUE      ( 2 << 8)
#define MMA845X_NOT_SUPPORTED              ( 3 << 8)
#define MMA845X_ODR_TOO_HIGH               ( 4 << 8)
#define MMA845X_GET_RAW_DATA_FAILED        ( 5 << 8)
#define MMA845X_GET_RAW_DATA_FIFO_FAILED   ( 6 << 8)
#define MMA845X_INT_CONFIG_FAILED          ( 7 << 8)
#define MMA845X_INT_SOURCE_FAILED          ( 8 << 8)
#define MMA845X_INT_WRONG_TYPE             ( 9 << 8)
#define MMA845X_DATA_SOURCE_FAILED         (10 << 8)
#define MMA845X_PULSE_CONFIG_FAILED        (11 << 8)
#define MMA845X_PULSE_SOURCE_FAILED        (12 << 8)
#define MMA845X_EVENT_CONFIG_FAILED        (13 << 8)
#define MMA845X_EVENT_SOURCE_FAILED        (14 << 8)
#define MMA845X_TRANSIENT_CONFIG_FAILED    (15 << 8)
#define MMA845X_TRANSIENT_SOURCE_FAILED    (16 << 8)
#define MMA845X_ORIENT_CONFIG_FAILED       (17 << 8)
#define MMA845X_ORIENT_SOURCE_FAILED       (18 << 8)
#define MMA845X_ORIENT_NOT_ENABLED         (19 << 8)


#include "mma845x_platform.h"
#include "mma845x_types.h"

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief   Initialize the sensor
 *
 * Reset the sensor and switch to power down mode. All registers are reset to 
 * default values.
 *
 * @param   bus        I2C at which MMA845X sensor is connected
 * @param   addr       I2C addr of the MMA845X sensor
 * @return             pointer to sensor data structure, or NULL on error
 */
mma845x_sensor_t* mma845x_init_sensor (uint8_t bus, uint8_t addr);


/**
 * @brief   Set sensor mode and switch to active mode
 *
 * @param   dev        pointer to the sensor device data structure
 * @param   mode       oversampling mode in wake sate [CTRL_REG2.MODS]
 * @param   odr        sensor output data rate (ODR) [CTRL_REG1.DR]
 * @param   low_noise  reduced noise enabled [CTRL_REG1.LNOISE]
 * @param   fast_read  fast read enabled [CTRL_REG1.F_READ]
 * @return             true on success, false on error
 */
bool mma845x_set_mode (mma845x_sensor_t* dev,
                       mma845x_oversampling_mode_t mode, mma845x_odr_t odr,
                       bool low_noise, bool fast_read);
                       

/**
 * @brief   Set sleep mode configuration and activate auto-sleep
 *
 * @param   dev        pointer to the sensor device data structure
 * @param   mode       oversampling mode in sleep sate [CTRL_REG2.SMODS]
 * @param   rate       auto-wake sample frequency [CTRL_REG1.ASLP_RATE]
 * @param   count      inactivity counter before autosleep [ASLP_COUNT]
 * @param   activate   activate auto-sleep/wake function [CTRL_REG2.SPLE]
 * @return             true on success, false on error
 */
bool mma845x_set_autosleep (mma845x_sensor_t* dev, 
                            mma845x_oversampling_mode_t mode,
                            mma845x_aslp_rate_t rate, 
                            uint8_t count, bool activate);

/**
 * @brief   Get current system mode of the sensor [SYSMOD.SYSMOD]
 *
 * @param   dev        pointer to the sensor device data structure
 * @return             sensor system mode or unknown mode on error
 */
mma845x_system_mode_t mma845x_get_system_mode (mma845x_sensor_t* dev);


/**
 * @brief   Set scale (full scale range)
 *
 * @param   dev        pointer to the sensor device data structure
 * @param   scale      full range scale [XYZ_DATA_CFG.FS]
 * @return             true on success, false on error
 */
bool mma845x_set_scale (mma845x_sensor_t* dev, mma845x_scale_t scale);
                              
                              
/**
 * @brief   Set FIFO mode
 *
 * FIFO watermark is used to generate an interrupt when the FIFO sample count
 * exceeds this value. Furthermore, it is used in FIFO trigger mode to define
 * the size of the circular buffer that is used before it switches to FIFO fill
 * mode triggered by the given trigger event.
 *
 * @param   dev        pointer to the sensor device data structure
 * @param   mode       FIFO mode [F_SETUP.F_MODE]
 * @param   watermark  FIFO watermark [F_SETUP.F_WMRK]
 * @param   trigger    trigger events used in trigger mode to switch from
 *                     circular buffer to fill buffer mode [TRIG_CFG]
 * @param   block      FIFO gate function, the FIFO is blocked and hold last
                       data on sleep/wake transitions [CTRL_REG3.FIFO_GATE]
 * @return             true on success, false on error
 */
bool mma845x_set_fifo_mode (mma845x_sensor_t* dev, 
                            mma845x_fifo_mode_t mode, uint8_t watermark, 
                            mma845x_fifo_trigger_t trigger, bool block);
                            

/**
 * @brief   Test whether new data samples are available
 *
 * @param   dev     pointer to the sensor device data structure
 * @return          true on new data, otherwise false
 */
bool mma845x_new_data (mma845x_sensor_t* dev);


/**
 * @brief   Get one sample of sensor data as floating point values (unit g)
 *
 * Function works only in bypass mode and fails in FIFO modes. In FIFO modes,
 * function *mma845x_get_float_data_fifo* has to be used instead to get data.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   data    pointer to float data structure filled with g values
 * @return          true on success, false on error
 */
bool mma845x_get_float_data (mma845x_sensor_t* dev,
                             mma845x_float_data_t* data);


/**
 * @brief   Get all samples of sensor data stored in the FIFO (unit g)
 *
 * In bypass mode, it returns only one sensor data sample.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   data    array of 32 float data structures filled with g values
 * @return          number of data sets read from fifo on success or 0 on error
 */
uint8_t mma845x_get_float_data_fifo (mma845x_sensor_t* dev,
                                     mma845x_float_data_fifo_t data);


/**
 * @brief   Get one sample of raw sensor data as 16 bit two's complements
 *
 * Function works only in bypass mode and fails in FIFO modes. In FIFO modes,
 * function *mma845x_get_raw_data_fifo* has to be used instead to get data.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   raw     pointer to raw data structure filled with values
 * @return          true on success, false on error
 */
bool mma845x_get_raw_data (mma845x_sensor_t* dev, mma845x_raw_data_t* raw);


/**
 * @brief   Get all samples of raw sensor data stored in the FIFO
 *
 * In bypass mode, it returns only one raw data sample.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   raw     array of 32 raw data structures
 * @return          number of data sets read from fifo on success or 0 on error
 */
uint8_t mma845x_get_raw_data_fifo (mma845x_sensor_t* dev,
                                   mma845x_raw_data_fifo_t raw);
                                   

/**
 * @brief   Enable / disable an interrupt on signal INT1 or INT2
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   type    interrupt to be enabled or disabled
 * @param   signal  interrupt signal that is activated for the interrupt
 * @param   value   true to enable or false to disable the interrupt
 * @return          true on success, false on error
 */
bool mma845x_enable_int (mma845x_sensor_t* dev, 
                         mma845x_int_type_t type, 
                         mma845x_int_signal_t signal, bool value);
                                   
/**
 * @brief   Get interrupt status
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   status  pointer to interrupt status data structure
 * @return          true on success, false on error
 */
bool mma845x_get_int_status(mma845x_sensor_t* dev, 
                            mma845x_int_status_t* status);


/**
 * @brief   Get the source of data ready and FIFO interrupts
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   source   pointer to the interrupt source
 * @return           true on success, false on error
 */
bool mma845x_get_int_data_source (mma845x_sensor_t* dev, 
                                  mma845x_int_data_source_t* source);


/**
 * @brief   Set inertial event configuration (freefall / motion detection)
 *
 * Inertial interrupt generator produce an interrupt when a certain inertial
 * event occures (event interrupts), that is, the acceleration of defined axes
 * is higher or lower than a defined threshold and one of the following event
 * is recognized: motion, free fall
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the interrupt generator configuration
 * @return           true on success, false on error
 */
bool mma845x_set_int_event_config (mma845x_sensor_t* dev,
                                   mma845x_int_event_config_t* config);


/**
 * @brief   Get inertial event configuration (freefall / motion detection)
 *
 * Inertial interrupt generator produce an interrupt when a certain inertial
 * event occures (event interrupts), that is, the acceleration of defined axes
 * is higher or lower than a defined threshold and one of the following event
 * is recognized: motion, free fall
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the interrupt generator configuration
 * @return           true on success, false on error
 */
bool mma845x_get_int_event_config (mma845x_sensor_t* dev,
                                   mma845x_int_event_config_t* config);


/**
 * @brief   Get source of an inertial event interrupt
 *
 * Returns a byte with flags that indicate the event which triggered
 * the interrupt signal, see FF_MT_SRC register in datasheet for details.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   source   pointer to the interrupt source data structure
 * @return           true on success, false on error
 */
bool mma845x_get_int_event_source (mma845x_sensor_t* dev,
                                   mma845x_int_event_source_t* source);


/**
 * @brief   Set transient event configuration
 *
 * Transient events are short term changes of accelerations higher than a 
 * given threshold. High-pass filtered data are used for detection function.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the transient detection interrupt configuration
 * @return           true on success, false on error
 */
bool mma845x_set_int_transient_config (mma845x_sensor_t* dev,
                                       mma845x_int_transient_config_t* config);


/**
 * @brief   Get transient event configuration
 *
 * Transient events are short term changes of accelerations higher than a 
 * given threshold. High-pass filtered data are used for detection function.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the transient detection interrupt configuration
 * @return           true on success, false on error
 */
bool mma845x_get_int_transient_config (mma845x_sensor_t* dev,
                                       mma845x_int_transient_config_t* config);


/**
 * @brief   Get source of a transient event interrupt
 *
 * Returns a byte with flags that indicate the transient event which triggered
 * the interrupt signal, see TRANSIENT_SRC register in datasheet for details.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   source   pointer to the interrupt source data structure
 * @return           true on success, false on error
 */
bool mma845x_get_int_transient_source (mma845x_sensor_t* dev,
                                       mma845x_int_transient_source_t* source);


/**
 * @brief   Set configuration of single/double tap detection interrupt
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the interrupt generator configuration
 * @return           true on success, false on error
 */
bool mma845x_set_int_pulse_config (mma845x_sensor_t* dev,
                                   mma845x_int_pulse_config_t* config);

/**
 * @brief   Get configuration of single/double tap detection interrupt
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the interrupt generator configuration
 * @return           true on success, false on error
 */
bool mma845x_get_int_pulse_config (mma845x_sensor_t* dev,
                                   mma845x_int_pulse_config_t* config);


/**
 * @brief   Get source of the click detection interrupt
 *
 * Returns a byte with flags that indicate the activity which triggered
 * the interrupt signal, see PULSE_SRC register in datasheet for details.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   source   pointer to the interrupt source
 * @return           true on success, false on error
 */
bool mma845x_get_int_pulse_source (mma845x_sensor_t* dev, 
                                   mma845x_int_pulse_source_t* source);
                                     

/**
 * @brief   Set configuration and enable orientation detection
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the orientation detection configuration
 * @return           true on success, false on error
 */
bool mma845x_set_orientation_config (mma845x_sensor_t* dev,
                                     mma845x_orientation_config_t* config);


/**
 * @brief   Get configuration of orientation detection
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   pointer to the orientation detection configuration
 * @return           true on success, false on error
 */
bool mma845x_get_orientation_config (mma845x_sensor_t* dev,
                                     mma845x_orientation_config_t* config);
                                     

/**
 * @brief   Get current orientation if orientation detection enabled
 *
 * See PL_STATUS register in datasheet for details.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   status   pointer to the orientation status
 * @return           true on success, false on error
 */
bool mma845x_get_orientation (mma845x_sensor_t* dev,
                              mma845x_orientation_status_t* status);
                              

/**
 * @brief   Set signal configuration for INT1 and INT2 signals
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   level    define interrupt signal as low or high active
 * @param   type     define interrupt signal as push-pull or open drain signal
 * @return           true on success, false on error
 */
bool mma845x_config_int_signals (mma845x_sensor_t* dev,
                                 mma845x_int_signal_level_t level,
                                 mma845x_int_signal_type_t type);

                              
/**
 * @brief   Configure high pass filter (HPF)
 *
 * HPF configuration is enabled per default transient event detection and
 * pulse. It can be enabled optionally for sensor output data.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   cutoff   HPF cutoff frequency (depends on ODR) [0 ... 3]
 * @param   data     HPF used optionally for sensor output data
 * @return           true on success, false on error
 */
bool mma845x_config_hpf (mma845x_sensor_t* dev, 
                         uint8_t cutoff, bool data);

/**
 * @brief   Set offset correction registers (see AN4069)
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   x        offset correction for x axis
 * @param   y        offset correction for y axis
 * @param   z        offset correction for z axis
 * @return           true on success, false on error
 */
bool mma845x_set_offset (mma845x_sensor_t* dev, 
                         uint8_t x, uint8_t y, uint8_t z);


// ---- Low level interface functions -----------------------------

/**
 * @brief   Direct write to register
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   reg      address of the first register to be changed
 * @param   data     pointer to the data to be written to the register
 * @param   len      number of bytes to be written to the register
 * @return           true on success, false on error
 */
bool mma845x_reg_write (mma845x_sensor_t* dev, 
                        uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief   Direct read from register
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   reg      address of the first register to be read
 * @param   data     pointer to the data to be read from the register
 * @param   len      number of bytes to be read from the register
 * @return           true on success, false on error
 */
bool mma845x_reg_read (mma845x_sensor_t* dev, 
                       uint8_t reg, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __MMA845X_H__ */
