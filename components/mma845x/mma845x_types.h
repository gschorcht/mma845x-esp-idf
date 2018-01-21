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
 * ARE DISCLAIMED. IN NO Activity SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MMA845X_TYPES_H__
#define __MMA845X_TYPES_H__

#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   MMA845x sensor ID type
 */
typedef enum {

    mma8451q_id,
    mma8452q_id,
    mma8453q_id,
    
} mma845x_sensor_id_t;

    
/**
 * @brief   Output data rates (ODR) in Hz
 */
typedef enum {

    mma845x_odr_800 = 0,  //  800 Hz, period 1.25 ms
    mma845x_odr_400,      //  400 Hz, period 2.5 ms
    mma845x_odr_200,      //  200 Hz, period 5 ms
    mma845x_odr_100,      //  100 Hz, period 10 ms
    mma845x_odr_50,       //   50 Hz, period 20 ms
    mma845x_odr_12_5,     // 12.5 Hz, period 80 ms
    mma845x_odr_6_25,     // 6.25 Hz, period 160 ms
    mma845x_odr_1_56,     // 1.56 Hz, period 640 ms

} mma845x_odr_t;


/**
 * @brief   Auto-sleep/wake sample frequency in sleep mode (ASLP_RATE)
 */
typedef enum {

    mma845x_aslp_rate_50 = 0,  //   50 Hz
    mma845x_aslp_rate_12_5,    // 12.5 Hz
    mma845x_aslp_rate_6_25,    // 6.25 Hz
    mma845x_aslp_rate_1_56     // 1.56 Hz

} mma845x_aslp_rate_t;


/**
 * @brief   System mode
 */
typedef enum {

    mma845x_standby_mode = 0,   // standby mode
    mma845x_wake_mode,          // acitve/wake mode
    mma845x_sleep_mode,         // active/sleep mode
    mma845x_unknown_mode

} mma845x_system_mode_t;


/**
 * @brief   Power mode used in active wake and sleep mode, respectively
 *          Determines the oversampling rate (see datasheet)
 */
typedef enum {

    mma845x_normal = 0,  // Normal
    mma845x_low_noise,   // Low noise, low power
    mma845x_high_res,    // High resolution
    mma845x_low_power,   // Low poweer

} mma845x_oversampling_mode_t;


/**
 * @brief   Full scale measurement range
 */
typedef enum {

    mma845x_scale_2_g = 0,     // default
    mma845x_scale_4_g,
    mma845x_scale_8_g

} mma845x_scale_t;


/**
 * @brief   FIFO mode
 */
typedef enum {

    mma845x_disabled = 0,  // FIFO is disabled (default)
    mma845x_circular = 1,  // overwrites oldest samples (circular buffer)
    mma845x_fill     = 2,  // stops when overflowed
    mma845x_trigger  = 3   // circular mode until trigger, then fill mode

} mma845x_fifo_mode_t;


/**
 * @brief   FIFO trigger event set
 */
typedef struct {

    uint8_t free_fall_motion:1;
    uint8_t pulse           :1;
    uint8_t orientation     :1;
    uint8_t transient       :1;

} mma845x_fifo_trigger_t;


/**
 * @brief   Interrupt signals
 */
typedef enum {

    mma845x_int1_signal = 0,
    mma845x_int2_signal = 1    

} mma845x_int_signal_t;
 
 
/**
 * @brief   Interrupt signal activity level
 */
typedef enum {

    mma845x_low_active = 0,      // default
    mma845x_high_active

} mma845x_int_signal_level_t;
    

/**
 * @brief   Interrupt signal type
 */
typedef enum {

    mma845x_push_pull = 0,      // default
    mma845x_open_drain

} mma845x_int_signal_type_t;
    
    
/**
 * @brief   Interrupt types for interrupt signals INT1/INT2
 */
typedef enum {

    mma845x_int_data_ready,      // data ready for read interrupt

    mma845x_int_fifo_watermark,  // FIFO exceeds the threshold interrupt
    mma845x_int_fifo_overrun,    // FIFO completely filled interrupt
    
    mma845x_int_event,           // inertial event interrupt (freefall/motion)
    mma845x_int_transient,       // transient event interrupt
    mma845x_int_pulse,           // single/double tap detection interrupt
    mma845x_int_orientation,     // orientation detection interrupt
    
    mma845x_int_autosleep        // autosleep/wake transition interrupt
    
} mma845x_int_type_t;


/**
 * @brief   Interrupt status type for interrupt signals INT1/INT2
 */
typedef struct {

    bool data_ready;   // data ready interrupt
    bool fifo;         // FIFO watermark or overrun interrupt
    bool event;        // inertial event interrupt (freefall/motion detection)
    bool transient;    // transient event interrupt
    bool pulse;        // single/double tap detection interrupt
    bool orientation;  // orientation detection interrupt
    bool autosleep;    // autosleep/wake transition interrupt
    
} mma845x_int_status_t;


/**
 * @brief   Data ready and FIFO status interrupt source
 */
typedef struct {

    bool data_ready;      // true when acceleration data are ready to read

    bool fifo_watermark;  // true when FIFO exceeds the FIFO threshold
    bool fifo_overrun;    // true when FIFO is completely filled
    
} mma845x_int_data_source_t;


/**
 * @brief   Inertial event interrupt configuration
 *
 * Inertial events are: freefall, motion. Please refer datasheet and AN4070
 * for more details.
 */
typedef struct {

    enum {                     // Motion detect/freefall detect flag selection
        mma845x_freefall = 0,  // OAE = 0 - AND combination (freefall)
        mma845x_motion   = 1   // OAE = 1 - OR  combination (motion)
    } mode;                   

    uint8_t threshold;         // threshold used for all axes (0.063 g/count)

    bool    x_enabled;         // x higher than threshold enabled
    bool    y_enabled;         // y higher than threshold enabled
    bool    z_enabled;         // z higher than threshold enabled

    uint8_t debounce_cnt;      // debounce counter
    bool    debounce_cnt_clr;  // true  - increment and clear
                               // false - increment and decrement

    bool    latch;             // latch the interrupt when true until the
                               // interrupt source has been read
    bool    sleep_active;      // false - function is bypassed in sleep mode    
                               // true  - function is active in sleep mode
                               //         (can wake up the system)
} mma845x_int_event_config_t;


/**
 * @brief   Inertial event interrupt source
 */
typedef struct {
    
    bool x_sign  :1;   // x motion event was negative g
    bool x_motion:1;   // x motion has been detected

    bool y_sign  :1;   // y motion event was negative g
    bool y_motion:1;   // y motion has been detected

    bool z_sign  :1;   // z motion event was negative g
    bool z_motion:1;   // z motion has been detected

    bool unused  :1;

    bool active  :1;     // true - one ore more events occured
    
} mma845x_int_event_source_t;


/**
 * @brief   Transient event detection interrupt configuration
 *
 * Transient events are short term changes of accelerations higher than a 
 * given threshold. High-pass filtered data are used. Please refer datasheet
 * and AN4071 for more details.
 */
typedef struct {

    bool    x_enabled;        // x higher than transient threshold enabled
    bool    y_enabled;        // y higher than transient threshold enabled
    bool    z_enabled;        // z higher than transient threshold enabled

    uint8_t threshold;        // threshold used for all axes (0.063 g/count)

    uint8_t debounce_cnt;     // debounce counter
    bool    debounce_cnt_clr; // true  - increment and clear
                              // false - increment and decrement

    bool    latch;            // latch the interrupt when true until the
                              // interrupt source has been read

    bool    hpf_bypassed;     // true  - bypass high-pass filter
                              // false - don't bypass high-pass filter (default)

    bool    sleep_active;     // false - function is bypassed in sleep mode    
                              // true  - function is active in sleep mode
                              //         (can wake up the system)
} mma845x_int_transient_config_t;


/**
 * @brief   Transient event interrupt source
 */
typedef struct {
    
    bool x_sign  :1;   // x transient event was negative g
    bool x_event :1;   // x transient acceleration greater than threshold

    bool y_sign  :1;   // y transient event was negative g
    bool y_event :1;   // y transient acceleration greater than threshold

    bool z_sign  :1;   // z transient event was negative g
    bool z_event :1;   // z transient acceleration greater than threshold

    bool active  :1;     // true - one ore more events occured
    bool unused  :1;
    
} mma845x_int_transient_source_t;


/**
 * @brief   Single/double directional tap detection interrupt configuration
 *
 * Please note: Members *pulse_limit*, *pulse_latency* and *pulse_window* 
 * are given in time units that depend on ODR And power mode. Please refer
 * the datasheet an AN4072 for more details.
 */
typedef struct {

    bool    x_single;      // x-axis single tap interrupt enabled
    bool    x_double;      // x-axis double tap interrupt enabled
    
    bool    y_single;      // y-axis single tap interrupt enabled
    bool    y_double;      // y-axis double tap interrupt enabled

    bool    z_single;      // z-axis single tap interrupt enabled
    bool    z_double;      // z-axis double tap interrupt enabled

    uint8_t x_threshold;   // x-axis threshold (0.063 g/count)
    uint8_t y_threshold;   // x-axis threshold (0.063 g/count)
    uint8_t z_threshold;   // x-axis threshold (0.063 g/count)
    
    uint8_t pulse_limit;   // maximum time interval between the start and the
                           // end of a tap (accel increases and falls back)
    uint8_t pulse_latency; // click detection is disabled for that time after 
                           // a was click detected
    uint8_t pulse_window;  // time interval in which the second click has to
                           // to be detected in double clicks
    bool    pulse_abort;   // abort double detection when when second pulse
                           // starts within *pulse_latency*

    bool    latch;         // latch the interrupt when true until the
                           // interrupt source has been read

    bool    hpf_bypassed;  // true  - bypass high-pass filter in pulse detection
                           // false - don't bypass high-pass filter (default)
    bool    lpf_enabled;   // true  - use low-pass filter for pulse detection
                           // false - don't use low-pass filter (default)

    bool    sleep_active;  // false - pulse detection is bypassed in sleep mode    
                           // true  - pulse detection is active in sleep mode
                           //         (can wake up the system)
} mma845x_int_pulse_config_t;


/**
 * @brief   Single/double directional tap detection interrupt source
 */
typedef struct {

    bool x_sign :1;    // x-pulse sign (0 - posisitive, 1 - negative)
    bool y_sign :1;    // y-pulse sign (0 - posisitive, 1 - negative)
    bool z_sign :1;    // z-pulse sign (0 - posisitive, 1 - negative)

    bool d_pulse:1;    // double pulse detected

    bool x_pulse:1;    // pulse detected in x direction
    bool y_pulse:1;    // pulse detected in y direction
    bool z_pulse:1;    // pulse detected in z direction

    bool active :1;    // true - one ore more event occured

} mma845x_int_pulse_source_t;


/**
 * @brief   Orientation detection function configuration
 *
 * Please refer datasheet and AN4068 for more details.
 */
typedef struct
{
    uint8_t enabled;           // portrait/landscape detection enable
    
    uint8_t pl_threshold;      // portrait/landscape trip angle threshold 
    uint8_t pl_hysteresis;     // added to threshold for smother transitions
    
    uint8_t bf_threshold;      // back/front trip angle threshold
    uint8_t z_lock;            // z-lock angle threshold
    
    uint8_t debounce_cnt;      // debounce counter
    bool    debounce_cnt_clr;  // true  - increment and clear
                               // false - increment and decrement

    bool    sleep_active;      // false - function is bypassed in sleep mode    
                               // true  - function is active in sleep mode
                               //         (can wake up the system)
} mma845x_orientation_config_t;


/**
 * @brief   Orientation status
 */
typedef struct
{
    bool  changed;                 // Landscape/portrait status changed
    bool  lockout;                 // Z-tilt angle lockout

    enum 
    {
        mma845x_portrait_up = 0,
        mma845x_portrait_down,
        mma845x_landscape_left,
        mma845x_landscape_right
        
    } portrait_landscape;
    
    enum 
    {
        mma845x_back = 0,
        mma845x_front
        
    } back_front;
    
} mma845x_orientation_status_t;


/**
 * @brief   Raw data set as two complements
 */
typedef struct {

    int16_t ax; // acceleration on x axis
    int16_t ay; // acceleration on y axis
    int16_t az; // acceleration on z axis

} mma845x_raw_data_t;


/**
 * @brief   Raw data FIFO type
 */
typedef mma845x_raw_data_t mma845x_raw_data_fifo_t[32];


/**
 * @brief   Floating point output value set in g
 */
typedef struct {

    float ax;   // acceleration on x axis
    float ay;   // acceleration on y axis
    float az;   // acceleration on z axis

} mma845x_float_data_t;


/**
 * @brief   Floating point output value FIFO type
 */
typedef mma845x_float_data_t mma845x_float_data_fifo_t[32];


/**
 * @brief   MMA845X sensor device data structure type
 */
typedef struct {

    int       error_code;            // error code of last operation

    uint8_t   bus;                   // I2C = x
    uint8_t   addr;                  // I2C = slave address

    bool active;                     // sensor is in active mode
    bool fast_read;                  // fast read modus activated
      
    mma845x_sensor_id_t  id;         // sensor id MMA8451Q, MMA842Q, MMA8453Q
    mma845x_scale_t      scale;      // full range scale (default 2 g)
    mma845x_fifo_mode_t  fifo_mode;  // FIFO operation mode (default disabled)

} mma845x_sensor_t;
                                 

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __MMA845X_TYPES_H__ */
