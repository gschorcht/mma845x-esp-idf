# Driver for the MMA845X 3-axes digital accelerometer 

The driver is for the usage with the ESP8266 and [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos). If you can't find it in folder [extras/MMA845X](https://github.com/SuperHouse/esp-open-rtos/tree/master/extras) of original repository, it is not yet merged. Please take a look to branch [MMA845X](https://github.com/gschorcht/esp-open-rtos/tree/MMA845X) of my fork in that case.

It is also working with ESP32 and [ESP-IDF](https://github.com/espressif/esp-idf.git) using a wrapper component for ESP8266 functions, see folder ```components/esp8266_wrapper```, as well as Linux based systems using a wrapper library.

The driver can be used with MMA8451Q, MMA8452Q, and MMA8453Q.

## About the sensor

MMA845X sensors are smart, low-power, **3-axis accelerometers** connected to **I2C** with a full scale of up to **±8 g**.

**Main features** of the sensor are:
 
- ±2 g/±4 g/±8 g dynamically selectable full scale
- output data rates (ODR) from 1.56 Hz to 800 Hz
- 14/12/10-bit and 8-bit digital output
- two programmable interrupt pins for seven interrupt sources
- inertial event detection (freefall or motion detection)
- transient event detection (jolt detection)
- pulse detection (single and double tap detection)
- orientation detection (portrait and landscape) with programmable hysteresis (only MMA8451Q)
- automatic ODR change for auto-wake and return to sleep
- embedded 32-sample FIFO (only MMA8451Q)
- embedded high-pass filter

## Sensor operation

### Sensor modes

MMA845X sensors provides different operating modes. There are two basic modes, the standby mode and the active mode.

**Standby mode** is started automatically after the power up boot sequence. In this mode, only digital blocks are enabled. Analog subsystems and the internal clocks are disabled. Registers are preserved and accessible for read and write operations. The standby mode is used to configure the sensor.

**Avtive mode** is the measurement mode. Once the sensor is configured, it can be switched to active mode to start measurements with a resolution of **14 bit** with MMA8451Q, **12 bit** with MMA8452Q, or **10 bit** with MMA8453Q at a defined output data rate (**ODR**).

In active mode, the sensor can operate in two different submodes using different output data rates (ODR)

- the **wake mode** with high output data rate and
- the **sleep mode** with a reduced output data rate.

The transition between these two active modes is automatic based on an inactivity timer and four of the interrupt sources. The sensor enters automatically the sleep mode after not detecting an interrupt for more than a user-defined timeout. When one of the following interrupts occurs, it will revert automatically from sleep to wake mode: tap detection, orientation detection, motion or freefall detection, and transient detection.

### Output Data Rates

In both active modes, measurements are made at a defined output rate. The following table shows the output data rates (ODR) supported by the sensor in wake mode and in sleep mode.

ODR | Driver symbol in wake mode | Driver symbol in sleep mode
-------:|:-------------|:-------------
1.56 Hz | ```mma845x_odr_1_56``` | ```mma845x_aslp_rate_1_56```
6.25 Hz | ```mma845x_odr_6_25``` | ```mma845x_aslp_rate_6_25```
12.5 Hz | ```mma845x_odr_12_5``` | ```mma845x_aslp_rate_12_5```
50 Hz   | ```mma845x_odr_50``` | ```mma845x_aslp_rate_50```
100 Hz | ```mma845x_odr_100``` | -
200 Hz | ```mma845x_odr_200``` | -
400 Hz | ```mma845x_odr_400``` | -
800 Hz | ```mma845x_odr_800``` | -

### Oversampling modes

In the wake mode as well as the sleep mode, the sensor can operate in different oversampling modes that define the oversampling rate and affect thus the power consumption of the sensor.

Mode   | Driver symbol
:------|:-------------
Normal | ```mma845x_normal```
Low Noise, Low Power | ```mma845x_low_noise```
Low Power | ```mma845x_low_power```
High Resolution | ```mma845x_high_res```

According to the selected oversampling mode (also called power scheme) the following oversampling rates are used by the sensor for the supported ODRs.

ODR | Low Power | Low Noise | Normal | High Resolution
-------:|:------:|:------:|:-----:|:-----:
1.56 Hz | 16 | 32 | 128 | 1024 
6.25 Hz |  4 |  8 |  32 |  256 
12.5 Hz |  2 |  4 |  16 |  128 
  50 Hz |  2 |  4 |   4 |   32 
 100 Hz |  2 |  4 |   4 |   16 
 200 Hz |  2 |  4 |   4 |    8 
 400 Hz |  2 |  4 |   4 |    4 
 800 Hz |  2 |  2 |   2 |    2 

### Simple usage

The **simplest way to use the sensor** is to initialize the sensor with ```mma845x_init_sensor``` function and then switch it to active mode with ```mma845x_set_mode``` function to start measurements with a given output data rate (ODR).

```
...
static mma845x_sensor_t* sensor;
...
if ((sensor = mma845x_init_sensor (I2C_BUS, MMA845X_I2C_ADDRESS_2, 0)))
{
    ...
    mma845x_set_mode (sensor, mma845x_high_res, mma845x_odr_50, true, false);
    ...
}
...

```
In this example, a MMA845X sensor connected to I2C with address ```MMA845X_I2C_ADDRESS_2``` is initialized and set to the **high-resolution mode** to start measurements with an output data rate (ODR) of **50 Hz**. The additional boolean parameters in this example define that the **noise reduction** function with reduced full dynamic range is activated and that the **fast reading** of 8-bit output data is not used (see register ```CTRL_REG1``` in [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) for more details). Auto-sleep/auto-wake functionality is disabled per default.

**Please note:** 
- ```mma845x_init_sensor``` function resets the sensor completely, switches it to the standby mode, and returns a pointer to a sensor device data structure on success. All registers are reset to default values and the embedded FIFO is cleared.
- All sensor configurations should be done before calling ```mma845x_set_mode``` function. In particular, the interrupt configuration should be performed before to avoid loosing the first interrupt and locking the system.

## Measurement results

### Output data format

The sensor determines periodically the accelerations for all axes and produces output data with the selected output data rate (ODR).

Raw **output data** (**raw data**) are given as 16-bit signed integer values in 2’s complement representation with a resolution of 14-bit for MMA8451Q, 12-bit for MMA8452Q, or 10-bit for MMA8453Q. These data are always left-aligned. The sensitivity depends on the selected full scale. MMA845X sensor allows to select the following full scales with the following sensitivities:

Full Scale  | Driver symbol | 14-bit data<br>MMA8451Q | 12-bit data<br>MMA8452Q | 10-bit data<br>MMA8451Q
---------------------:|:-----------:|-----------:|---------------:|-----:
 ±2 g | ```mma845x_scale_2_g```  |  1/4096 g  | 1/1024 g |  1/256 g
 ±4 g | ```mma845x_scale_4_g```  |  1/2048 g  | 1/512 g  |  1/128 g
 ±8 g | ```mma845x_scale_8_g```  |  1/1024 g  | 1/256 g  |  1/64 g

By default, a full scale of ±2 g is used. ```mma845x_set_scale``` function can be used to change it.

```
mma845x_set_scale(sensor, mma845x_scale_4_g);
```

**Please note:** In fast read mode, only the high byte of the 16-bit output data is used. 

### Fetching output data

To get the information whether new data are available, the user task can either use

- the ```mma845x_new_data``` function to check periodically whether new output data are available, or
- the data ready interrupt which is generated as soon as new output data are available (see below).

Last measurement results can then be fetched either 

- as raw data using ```mma845x_get_raw_data``` function or 
- as floating point values measured in g using ```mma845x_get_float_data``` function.

It is recommended to use ```mma845x_get_float_data``` function since it already converts measurement results to real values according to the selected full scale.

```
void user_task_periodic(void *pvParameters)
{
    mma845x_float_data_t data;

    while (1)
    {
        // execute task every 100 ms
        vTaskDelay (100/portTICK_PERIOD_MS);
        ...
        // test for new data
        if (!mma845x_new_data (sensor))
            continue;
    
        // fetch new data
        if (mma845x_get_float_data (sensor, &data))
        {
            // do something with data
            ...
        }
    }
}
```

**Please note:** 
```mma845x_get_float_data``` and ```mma845x_get_raw_data``` functions always return the last available results. If these functions are called more often than measurements are taken, some measurement results are retrieved multiple times. If these functions are called too rarely, some measurement results will be lost.

### High pass filtering

MMA845X sensors have a built-in high-pass filter to remove the offset and slower changing acceleration data, also called **static acceleration**. Please refer the [application note AN4071](https://www.nxp.com/docs/en/application-note/AN4071.pdf) for more details.

The high-pass filter can be applied to 

- the raw output data,
- the data used for transient event detection function (jolt detection), and
- the data used for pulse detection function (single and double tap detection)

The cutoff frequency of the high-pass filter depends on the output data rate (ODR) and the selected oversampling mode. For each output data rate and oversampling mode, four different HPF cutoff frequencies can be selected (see register ```HP_FILTER_CUTOFF``` in the [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf)).

The cutoff frequency is selected with ```mma845x_config_hpf``` function. The additional boolean parameters is used to enable or disable high-pass filtering for raw output data. For other functions, the high-pass filter is enabled and disabled in the configuration of the function (see below).

```
...
// configure HPF
mma845x_config_hpf (sensor, 0, true);
...
```
 
### FIFO

In order to limit the rate at which the host processor has to fetch the data, the MMA8451Q embeds a first-in first-out buffer (FIFO). This is in particular helpful at high output data rates. The FIFO buffer can work in four different modes and is able to store up to 32 accelerometer samples. Please refer the [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) or [application note AN4073](https://www.nxp.com/docs/en/application-note/AN4073.pdf) for more details.

FIFO mode | Driver symbol
--------------|-------------------------
FIFO disabled | ```mma845x_disabled```
Circular buffer mode | ```mma845x_circular```
Fill buffer mode     | ```mma845x_fill```
Trigger buffer mode  | ```mma845x_trigger```

While the FIFO in **fill buffer** mode stops accepting new samples when overflowed, the FIFO contains always the most recent samples in **circular buffer** mode when overflowed. 

In **trigger buffer** mode FIFO operates in circular buffer mode up to the number of samples set in the watermark. When one of the defined triggers occur, the FIFO will switch to fill buffer mode and collect samples until the buffer if full. This allows to collect data both before and after a trigger event.

The FIFO mode can be set using ```mma845x_set_fifo_mode``` function. This function takes as parameters

- the FIFO mode,
- a watermark value, and
- a set of trigger events that are used in trigger buffer mode.

The additional boolean parameter defines whether the FIFO gate function is used. Whith FIFO gate function, the FIFO blocks on wake / sleep transitions and preserves the contents of the FIFO buffer.

The watermark level is used by the sensor to set a watermark and to generate an interrupt if the FIFO content exceeds that level. It can be used to collect a minimum number of acceleration samples with the sensor before fetching the data as a single read operation from the sensor.

```
...
// activate FIFO mode
mma845x_fifo_trigger_t trigger = {};

trigger.orientation = true;
trigger.transient = true;

mma845x_set_fifo_mode (sensor, mma845x_trigger, 20, trigger, false);
...
```

In this example, the FIFO is used in trigger buffer mode. The FIFO always collects the last 20 acceleration data samples in circular buffer mode. Once the orientation is changed or a transient event is detected, the FIFO fills the remaining 12 acceleration samples before blocking.

**Please note**: 
- To flush the FIFO at any time, set the FIFO mode to ```mma845x_disabled``` and back to the desired FIFO mode.
- The FIFO is cleared also on transition from standby mode to active mode, or during an automatic ODR change on auto-sleep/wake transitions when FIFO gate function is disabled.

To **fetch data from the FIFO**, simply use either 

- the ```mma845x_get_raw_data_fifo``` function to all get raw output data stored in FIFO or
- the ```mma845x_get_float_data_fifo``` function to get all data stored in FIFO and converted to real values in g. 

Both functions flush the FIFO implicitly and return the number of samples read from the FIFO. 

```
void user_task_periodic (void *pvParameters)
{
    mma845x_float_data_fifo_t fifo;

    while (1)
    {
        // execute task every 100 ms
        vTaskDelay (100/portTICK_PERIOD_MS);
        ...
        // test for new data
        if (!mma845x_new_data (sensor))
            continue;
    
        // fetch data from fifo
        uint8_t num = mma845x_get_float_data_fifo (sensor, fifo);

        for (int i=0; i < num; i++)
        {
           // do something with data fifo[i] ...
        }
}
```

## Interrupts

The MMA845X sensors support two dedicated interrupt signals **```INT1```** and **```INT2```** as well as different types of interrupts:

- **data** interrupts (data ready and FIFO status),
- **inertial event** detection interrupts (freefall and motion detection),
- **transient event** detection interrupts (jolt detection),
- **orientation** detection interrupts (landscape and portrait detection),
- **pulse** detection interrupts (single and double tap detection),
- **auto-sleep/wake transition** interrupts.

Each of these interrupt types can be enabled or disabled separately with ```mma845x_enable_int``` function. By default all interrupts are disabled. All interrupts can be routed to both interrupt signals. 

```
mma845x_enable_int (sensor, mma845x_int_event, mma845x_int2_signal, true);
```

Whenever an interrupt is generated, the ```mma845x_get_int_status``` function can be used to determine what type of interrupt is generating the interrupt signal. 

```
mma845x_int_status_t int_status;
mma845x_get_int_status (sensor, &int_status);

if (int_status.event)
{
    // inertial event happened
    ...
}

if (int_status.orientation)
{
    // orientation changed
    ...
}
...
```

For each type of interrupt there are further functions to get more information about the source of the interrupt, see below.

### Data interrupts (data ready and FIFO status)

Following sources can generate data interrupts:

Interrupt source | Driver symbol
:-----------------|:-------------
Output data become ready to read | ```mma845x_int_data_ready```
FIFO content exceeds the watermark level | ```mma845x_int_fifo_watermark```
FIFO is completely filled | ```mma845x_int_fifo_overrun```

Each of these interrupt sources can be enabled or disabled separately with the ```mma845x_enable_int``` function.

```
mma845x_enable_int (sensor, mma845x_int_data_ready, mma845x_int2_signal, true);
```

Whenever a data interrupt is generated, the ```mma845x_get_int_data_source``` function can be used to determine the source of the data interrupt. This function returns a data structure of type ```mma845x_int_data_source_t``` that contain a boolean member for each source that can be tested for true.

```
void int_handler ()
{
    mma845x_int_data_source_t data_src;

    // get the source of the interrupt
    mma845x_int_data_source_t data_src;
    mma845x_get_int_data_source (sensor, &data_src);

    // in case of data ready interrupt read one data sample and do something with data
    if (data_src.data_ready)
        ... // read data
   
    // in case of FIFO interrupts read the whole FIFO and do something with data
    else if (data_src.fifo_watermark || data_src.fifo_overrun)
        ... // read data
    ...
}
```

**Please note:** While FIFO interrupts are reset as soon as the interrupt source is read, the data ready interrupts are not reset until the data has been read.

### Inertial event interrupts (motion and freefall detection)

The motion and freefall detection function of the sensor allows to generate interrupts when certain inertial events occur (event interrupts), that is, **the static acceleration of enabled axes is higher or lower than a defined threshold**. If activated, the acceleration of each enabled axis is compared with the defined threshold to check whether it is below or above the threshold. The results of all comparisons are then combined AND or OR to generate the interrupt signal.

The configuration of the threshold, the activated comparisons and the selected AND/OR combination allows to recognize special situations:

- **motion detection** refers the special condition that the acceleration measured along any axis is above the defined threshold (OR = ```mma845x_motion```).
- **Free fall detection** refers the special condition that the acceleration measured along all the axes goes to zero (AND = ```mma845x_freefall```).

Please refer the [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) or [application note AN4070](https://www.nxp.com/docs/en/application-note/AN4070.pdf) for more details.

Inertial event interrupts can be configured with the ```mma845x_set_int_event_config``` function, which takes the configuration of type ```mma845x_int_event_config_t``` as parameter. The configuration defines

- the inertial event that should be detected (```mma845x_motion``` or ```mma845x_freefall```),
- the threshold value given in 0.063 g used for the event detection,
- the axes which are used for the event detection,
- a debounce counter that defines the minimum time over which the interrupt condition has to be present,
- the behavior of the debounce counter when interrupt condition is not given any longer,
- whether the interrupt should be latched until the interrupt source is read, and
- whether the event detection is also active in sleep mode and can wake up the sensor.

Once the inertial event detection is configured, ```mma845x_enable_int``` function is used to enable the interrupt. The interrupt signal to which the interrupt is routed is given as parameter. For example, motion detection interrupts generated on signal ```INT1``` could be configured and enabled as following:

```
mma845x_int_event_config_t event_config;
    
event_config.mode = mma845x_motion;     // OR-condition
event_config.threshold = 2;             // 0.063 g/count = 0.1266 g
event_config.x_enabled = true;          // x-axis is used for event detection
event_config.y_enabled = true;          // y-axis is used for event detection
event_config.z_enabled = false;         // z-axis is not used for event detection
event_config.debounce_cnt = 5;          // 100 ms at ODR=50Hz in normal oversampling mode
event_config.debounce_cnt_decr = true;  // increment and decrement debounce counter
event_config.latch = true;              // interrupt is latched until its source is read
event_config.sleep_active = true;       // active in sleep mode
        
mma845x_set_int_event_config (sensor, &event_config);
mma845x_enable_int (sensor, mma845x_int_event1, mma845x_int1_signal, true);
```
**Please note:** The time that is defined by the debounce counter depends on current ODR and the selected oversampling mode, see register ```FF_MT_COUNT``` in [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) for more details. In the example, the time step counted by the debounce counter is 20 ms.

As with data interrupts, ```mma845x_get_int_event_source``` function can be used to determine the source of an inertial event interrupt whenever it is generated. This function returns a data structure of type ```mma845x_int_event_source_t``` which contains a boolean member for each source that can be tested for true.

```
void int_handler ()
{
    mma845x_int_status_t int_status;
    mma845x_get_int_status (sensor, &int_status);

    ...
    // in case of inertial event interrupt
    if (int_status.event)
    {
        // get the source of the interrupt and reset *INTx* signals
        mma845x_int_event_source_t event_src = {};
        mma845x_get_int_event_source (sensor, &event_src);

        // test for the interrupt source
        if (event_src.x_motion) ... // do something
        if (event_src.y_motion) ... // do something
        if (event_src.z_motion) ... // do something

        ...
    }
    ...
}
```

**Please note:** If the interrupt is configured to be latched, the interrupt signal is active until the interrupt source is read. Otherwise the interrupt signal is only active as long as the interrupt condition is present.

### Transient event interrupts (jolt detection)

The transient event detection function of the sensor allows to generate interrupts when **the change in acceleration of any axis exceeds a defined threshold**. For this purpose, only the dynamic part of the measured acceleration data is used. Therefore, the data used for transient event detection are high-pass filtered to remove the static part of the acceleration data.

**Please note:** High-pass filtering of data for transient event detection can be disabled. In that case the transient event detection works like motion detection.

Please refer the [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) or [application note AN4071](https://www.nxp.com/docs/en/application-note/AN4071.pdf) for more details.

Transient event interrupts can be configured with the ```mma845x_set_int_transient_config``` function, which takes the configuration of type ```mma845x_int_transient_config_t``` as parameter. The configuration defines

- the threshold value given in 0.063 g used for the event detection,
- the axes which are used for the event detection,
- a debounce counter that defines the minimum time over which the interrupt condition has to be present,
- the behavior of the debounce counter when interrupt condition is not given any longer,
- whether the interrupt should be latched until the interrupt source is read,
- whether the high-pass filtering of data should be disabled, and
- whether the event detection is also active in sleep mode and can wake up the sensor.

Once the transient event detection is configured, ```mma845x_enable_int``` function is used to enable the interrupt. The interrupt signal to which the interrupt is routed is given as parameter. For example, transient event interrupts generated on signal ```INT1``` could be configured and enabled as following:

```
mma845x_int_transient_config_t trans_config;
    
trans_config.threshold = 8;             // 0.063 g/count = 0.504 g
trans_config.x_enabled = true;          // x-axis is used for event detection
trans_config.y_enabled = true;          // y-axis is used for event detection
trans_config.z_enabled = false;         // z-axis is not used for event detection
trans_config.debounce_cnt = 2;          // 40 ms at ODR=50Hz in normal oversampling mode
trans_config.debounce_cnt_decr = false; // increment and clear debounce counter
trans_config.latch = true;              // interrupt is latched until its source is read
trans_config.hpf_bypassed = false;      // HPF is not bypassed (default)
trans_config.sleep_active = true;       // active in sleep mode
        
mma845x_set_int_transient_config (sensor, &trans_config);
mma845x_enable_int (sensor, mma845x_int_transient, mma845x_int1_signal, true);

```
**Please note:** The time that is defined by the debounce counter depends on current ODR and the selected oversampling mode, see register ```TRANSIENT_COUNT``` in [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) for more details. In the example, the time step counted by the debounce counter is 20 ms.

As with other interrupts, ```mma845x_get_int_transient_source``` function can be used to determine the source of a transient event interrupt whenever it is generated. This function returns a data structure of type ```mma845x_int_transient_source_t``` which contains a boolean member for each source that can be tested for true.

```
void int_handler ()
{
    mma845x_int_status_t int_status;
    mma845x_get_int_status (sensor, &int_status);

    ...
    // in case of transient event interrupt
    if (int_status.transient)
    {
        // get the source of the interrupt and reset *INTx* signals
        mma845x_int_transient_source_t trans_src = {};
        mma845x_get_int_transient_source (sensor, &trans_src);
    
        // test for the interrupt source
        if (trans_src.x_event) ... // do something
        if (trans_src.y_event) ... // do something
        if (trans_src.z_event) ... // do something
    }
    ...
}
```

**Please note:** If the interrupt is configured to be latched, the interrupt signal is active until the interrupt source is read. Otherwise the interrupt signal is only active as long as the interrupt condition is present.

### Pulse detection interrupts (single and double tap detection)

The signature of accelerations, that is, the **sequence of acceleration data over time** measured along certain axes can be used to detect single and double taps. 

A **single tap** is given when the acceleration due to a pulse exceeds a specified acceleration threshold and crosses up and down within a specified time limit, also called pulse limit. A **double tap** is given, if a second tap is recognized within a time limit called pulse window. This pulse window starts after waiting a specified pulse latency after the first tap.

The data used for pulse detection are high-pass filtered to remove the static part of the acceleration data.

Please refer the [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) or [application note AN4072](https://www.nxp.com/docs/en/application-note/AN4072.pdf) for more details.

Pulse detection interrupts can be configured with the ```mma845x_set_int_pulse_config``` function, which takes the configuration of type ```mma845x_int_pulse_config_t``` as parameter. The configuration defines

- the threshold values given in 0.063 g used for the pulse detection for each axis,
- the axes enabled for the single tap detection,
- the axes enabled for the double tap detection,
- a pulse limit that defines the time limit within the acceleration must cross up and down the threshold
- a pulse latency that defines the time before the second tap event can be recognized for double tap detection
- a pulse window that defines the time window within the second tap have to be recognized for double tap detection
- whether the interrupt should be latched until the interrupt source is read,
- whether the high-pass filtering of data should be disabled,
- whether the low-pass filtering of data should be enabled, and
- whether the event detection is also active in sleep mode and can wake up the sensor.

Once the pulse detection is configured, ```mma845x_enable_int``` function is used to enable the interrupt. The interrupt signal to which the interrupt is routed is given as parameter. For example, single tap detection interrupts generated on signal ```INT1``` could be configured and enabled as following:

```
mma845x_int_pulse_config_t pulse_config;
        
pulse_config.x_threshold = 25;      // 0.063 g/count = 1.575 g 
pulse_config.y_threshold = 25;      // 0.063 g/count = 1.575 g 
pulse_config.z_threshold = 42;      // 0.063 g/count = 1.65 g 
pulse_config.x_single = true;       // single tap detection in x direction enabled
pulse_config.x_double = false;      // double tap detection in x direction enabled
pulse_config.y_single = true;       // single tap detection in y direction enabled
pulse_config.y_double = false;      // double tap detection in y direction enabled
pulse_config.z_single = true;       // single tap detection in z direction enabled
pulse_config.z_double = false;      // double tap detection in z direction enabled
pulse_config.pulse_limit   = 10;    //  50 ms at ODR=50 Hz in normal mode and no LPF
pulse_config.pulse_latency = 20;    // 200 ms at ODR=50 Hz in normal mode and no LPF
pulse_config.pulse_window  = 30;    // 300 ms at ODR=50 Hz in normal mode and no LPF
pulse_config.latch = true;          // interrupt is latched until its source is read
pulse_config.hpf_bypassed = false;  // HPF is not bypassed (default)
pulse_config.lpf_enabled  = false;  // LPF is not used (default)
pulse_config.sleep_active = true;   // active in sleep mode
      
mma845x_set_int_pulse_config (sensor, &pulse_config);
mma845x_enable_int (sensor, mma845x_int_pulse, mma845x_int1_signal, true);
```

**Please note:** The times that are defined as pulse limit, pulse latency and pulse window depend on current ODR, the selected oversampling mode, and whether low-pass filtering is enabled, see register ```PULSE_TMLT```, ```PULSE_LTCY```, and ```PULSE_WIND``` in [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) for more details. In the example, the pulse limit is given in 5 ms time steps while pulse latency and pulse window are given in 10 ms time steps.

As with other interrupts, ```mma845x_get_int_pulse_source``` function can be used to determine the source of an transient event interrupt whenever it is generated. This function returns a data structure of type ```mma845x_int_pulse_source_t``` which contains a boolean member for each source that can be tested for true.

```
void int_handler ()
{
    mma845x_int_status_t int_status;
    mma845x_get_int_status (sensor, &int_status);

    ...
    // in case of tap detection interrupt
    if (int_status.pulse)
    {
        // get the source of the interrupt and reset *INTx* signals
        mma845x_int_pulse_source_t pulse_src = {};
        mma845x_get_int_pulse_source (sensor, &pulse_src);
        
        if (pulse.x_pulse) ... // do something
        if (pulse.y_pulse) ... // do something
        if (pulse.z_pulse) ... // do something
    }
    ...
}
```

**Please note:** If the interrupt is configured to be latched, the interrupt signal is active until the interrupt source is read. Otherwise the interrupt signal is only active as long as the interrupt condition is present.

### Orientation detection interrupts (landscape and portrait detection)

The orientation detection function (MMA8451Q only) uses the gravity and measured accelerations to determine the orientation of the sensor. Detected orientations supported by the sensor are:

Orientation | Driver symbol
:-----------|:-------------
Portrait Up     | ```mma845x_portrait_up```
Portrait Down   | ```mma845x_portrait_down```
Landscape Left  | ```mma845x_lanscape_left```
Landscape Right | ```mma845x_lanscape_right```
Back            | ```mma845x_back```
Front           | ```mma845x_front```

The orientation of the device is given by one of the four landscape/portrait orientations in combination with front/back orientation.

The orientation detection function is configured with the ```mma845x_set_orientation_config``` function, which takes the configuration of type ```mma845x_orientation_config_t``` as parameter. This configuration defines

- a portrait/landscape orientation trip angle threshold,
- a hysteresis that is added to the trip angle threshold for smother transitions,
- a back/front orientation trip angle threshold,
- a z-lock angle threshold,
- a debounce counter that defines the minimum time over which the orientation has to be present,
- the behavior of the debounce counter when orientation is not given any longer, and
- whether the orientation detection is also active in sleep mode and can wake up the sensor.

Please refer the [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) or [application note AN4068](https://www.nxp.com/docs/en/application-note/AN4068.pdf) for more details.

Once the orientation detection function is configured, ```mma845x_get_orientation``` function can be used to retrieve the current orientation of the sensor at any time. This function returns a datastructure of type ```mma845x_orientation_status_t``` that consists of the two members ```portrait_landscape``` and ```back_front``` that contain the current orientation.

For example, the orientation detection could be configured and used as following:
```
...
mma845x_orientation_config_t orient_config;

orient_config.enabled = true;           // enable portrait/landscape detection
orient_config.sleep_active = true;      // active in sleep mode
orient_config.pl_threshold = 16;        // trip angle 45°   
orient_config.pl_hysteresis= 4;         // hysteresis +-14°
orient_config.bf_threshold = 1;         // 285° < z < 75° back->front, 
                                        // 105° < z < 255° front->back 
orient_config.z_lock = 4;               // z >= 29°
orient_config.debounce_cnt = 5;         // 100 ms at ODR=50Hz in normal mode
orient_config.debounce_cnt_decr = true; // increment and decrement debounce counter

mma845x_set_orientation_config (sensor, &orient_config);
...
```
```
...
mma845x_orientation_status_t orientation;

if (mma845x_get_orientation (sensor, &orientation)
{
    switch (orientation.portrait_landscape)
    {
        case mma845x_portrait_up:     ... // do something
        case mma845x_portrait_down:   ... // do something
        case mma845x_landscape_left:  ... // do something
        case mma845x_landscape_right: ... // do something
    }
    switch (orientation.back_front)
    {
        case mma845x_back:  ... // do something
        case mma845x_front: ... // do something
    }
}
...
```

It is possible to generate interrupts when the orientation is changed. Once the orientation detection function is configured, ```mma845x_enable_int``` function can used to enable the interrupt. The interrupt signal to which the interrupt is routed is given as parameter. 

```
mma845x_enable_int (sensor, mma845x_int_orientation, mma845x_int1_signal, true);
```

```mma845x_get_int_status``` function in conjunction with ```mma845x_get_orientation``` function can be used to determine the source of an orientation detection interrupt whenever it is generated. Member ```changed``` of the data structure of type ```mma845x_orientation_status_t``` can be tested for true.

```
void int_handler ()
{
    mma845x_int_status_t int_status;
    mma845x_get_int_status (sensor, &int_status);

    ...
    // in case of orientation change detection interrupt
    if (int_status.orientation)
    {
        // get the source of the interrupt and reset *INTx* signals
        mma845x_orientation_status_t orient;
        mma845x_get_orientation (sensor, &orient);
    
        if (orient.changed)
        {  
            .. // do something
        }
    }
    ...
}
```

### Auto-sleep/wake interrupts

The sensor can be configured to enter automatically in the sleep mode after not detecting an interrupt for more than a user-defined timeout. In sleep mode it operates with a reduced output data rate, also called auto-wake sample frequency. When one of the following interrupts occurs, it will revert automatically from sleep to wake mode: 

- pulse detection (single or double tap)
- orientation changed detection (landscape and portrait detection)
- inertial event detection (motion or free fall)
- transient event detection (jolt detection)

The auto-sleep/wake function is configured with the ```mma845x_set_autosleep``` function, which takes following parameters as configuration:

- the oversampling mode in sleep mode
- the auto-wake sample frequency (output data rate in sleep mode)
- an inactivity counter that defines the inactivity time-out before auto-sleep
- a flag that activates the auto-sleep/wake function

Please refer the [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) or [application note AN4074](https://www.nxp.com/docs/en/application-note/AN4074.pdf) for more details.

For example, the auto-sleep/wake function could be configured as following:
```
mma845x_set_autosleep (sensor, mma845x_normal, mma845x_aslp_rate_1_56, 10, true);
```

In this example, the normal oversampling mode and an output data rate of 1.56 Hz are used in sleep mode. The sensor enters the sleep mode after not detecting an interrupt for 10 time units, i.e., 3.2 seconds. The boolean parameter enables the auto-sleep/wake function.

**Please note:** The time that is defined by the inactivity counter depends on current ODR, see register ```ASLP_COUNT``` in [datasheet of MMA8541Q](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf) for more details. In the example, the time step counted by the inactivity counter is 320 ms.

It is possible to generate interrupts when the mode changes from wake to sleep and vise versa. Once the auto-sleep/wake function is configured, ```mma845x_enable_int``` function can used to enable the interrupt. The interrupt signal to which the interrupt is routed is given as parameter. 

```
mma845x_enable_int (sensor, mma845x_int_autosleep, mma845x_int1_signal, true);
```

```mma845x_get_int_status``` function in conjunction with ```mma845x_get_system_mode``` function can be used to determine the source of an auto-sleep/wake function interrupt whenever it is generated. ```mma845x_get_system_mode``` function returns the current system mode.

```
void int_handler ()
{
    mma845x_int_status_t int_status;
    mma845x_get_int_status (sensor, &int_status);

    ...
    // in case of auto-sleep/wake interrupt
    if (int_status.autosleep)
    {
        switch (mma845x_get_system_mode(sensor))
        {
            case mma845x_standby_mode: ... // do something
            case mma845x_wake_mode:    ... // do something
            case mma845x_sleep_mode:   ... // do something
        }
    }
    ...
}
```

### Interrupt signal properties

By default, interrupt signals are low active. Using ```mma845x_config_int_signals``` function, the polarity of the interrupt signal and the mode of the interrupt outputs can be changed.

Driver symbol | Meaning
:-------------|:-------
```mma845x_high_active``` | Interrupt signal is high active
```mma845x_low_active``` | Interrupt signal is low active (default)

Driver symbol | Meaning
:-------------|:-------
```mma845x_push_pull```  | Interrupt output is pushed/pulled
```mma845x_open_drain``` | Interrupt output is open-drain


## Low level functions

The MMA845X is a very complex and flexible sensor with a lot of features. It can be used for a large number of different use cases. Since it is quite impossible to implement a high level interface which is generic enough to cover all the functionality of the sensor for all different use cases, there are two low level interface functions that allow direct read and write access to the registers of the sensor.

```
bool mma845x_reg_read  (mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len);
bool mma845x_reg_write (mma845x_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len);
```

**Please note**
These functions should only be used to do something special that is not covered by drivers's  high level interface AND if you exactly know what you do and what it might affect. Please be aware that it might always affect the high level interface.


## Usage

First, the hardware configuration has to be established.

### Hardware configurations

Following figure shows a possible hardware configuration for ESP8266 and ESP32 to connect the sensor using an I2C interface .

```
  +-----------------+     +----------+
  | ESP8266 / ESP32 |     | MMA845X  |
  |                 |     |          |
  |   GPIO 14 (SCL) >-----> SCL      |
  |   GPIO 13 (SDA) <-----> SDA      |
  |   GPIO 5        <------ INT1     |
  |   GPIO 4        <------ INT2     |
  +-----------------+     +----------+
```


### Communication interface settings

Dependent on the hardware configuration, the communication interface and interrupt settings have to be defined. In case ESP32 is used, the configuration could look like 

```
#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

#else  // ESP8266 (esp-open-rtos)

// user task stack depth for ESP8266
#define TASK_STACK_DEPTH 512

#endif  // ESP_PLATFORM

// I2C interface defintions for ESP32 and ESP8266
#define I2C_BUS       0
#define I2C_SCL_PIN   14
#define I2C_SDA_PIN   13
#define I2C_FREQ      I2C_FREQ_400K

// interrupt GPIOs defintions for ESP8266 and ESP32
#define INT1_PIN      5
#define INT2_PIN      4
```

### Main program

#### Initialization

First, I2C interfaces that are used have to be initialized.

```
i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
```

Once the interfaces are initialized, function ```mma845x_init_sensor``` has to be called for each MMA845X sensor in order to initialize the sensor and to check its availability as well as its error state. This function returns a pointer to a sensor device data structure or NULL in case of error.

The parameter *bus* specifies the ID of the I2C bus to which the sensor is connected.

```
static mma845x_sensor_t* sensor;
```

For sensors connected to an I2C interface, a valid I2C slave address has to be defined as parameter *addr*.

```
sensor = mma845x_init_sensor (I2C_BUS, MMA845X_I2C_ADDRESS_2);

```

#### Configuring the sensor

Optionally, you could wish to set some measurement parameters. For details see the sections above, the header file of the driver ```mma845x.h```, and of course the datasheet and application notes of the sensor.

#### Starting measurements

Once the sensor is configured, the sensor mode has be set to start periodic measurement. This sensor mode can be changed anytime later.

```
...
// start periodic measurement with output data rate of 50 Hz
mma845x_set_mode (sensor, mma845x_odr_10, mma845x_high_res, true, true, true);
...
```

#### Periodic user task

Finally, a user task that uses the sensor has to be created. 

**Please note:** To avoid concurrency situations when driver functions are used to access the sensor, for example to read data, the user task must not be created until the sensor configuration is completed.

The user task can use different approaches to fetch new data. Either new data are fetched periodically or interrupt signals are used when new data are available or configured events occur.

If new data are fetched **periodically** the implementation of the user task is quite simple and could look like following.

```
void user_task_periodic(void *pvParameters)
{
    // small delay before accessing the sensor the first time
    vTaskDelay (100/portTICK_PERIOD_MS);
    
    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // test for new data
        if (!mma845x_new_data (sensor))
            continue;
    
        // fetch new data
        if (mma845x_get_float_data (sensor, &data))
        {
            // do something with data
            ...
        }
        
        // passive waiting until 100 ms are over
        vTaskDelayUntil(&last_wakeup, 100/portTICK_PERIOD_MS);
    }
}

...
// create a user task that fetches data from sensor periodically
xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);
```

The user task simply tests periodically with a given rate whether new data are available. If new data are available, it fetches the data and does something with them.

#### Interrupt user task

Another approach is to use one of the **interrupts** signals ```INT1``` or ```INT2```. In this case, the user has to implement an interrupt handler. Since interrupt handlers must be kept small in execution time, the interrupt handler should not fetch the data directly, but only send an event to a waiting task that processes the interrupts from different sources.

```
static QueueHandle_t gpio_evt_queue = NULL;

// Interrupt handler which sends an interrupt event to a user task

void IRAM int_signal_handler (uint8_t gpio)
{
    // send an event with GPIO to the interrupt user task
    xQueueSendFromISR(gpio_evt_queue, &gpio, NULL);
}

// User task that reacts on interrupt events

void user_task_interrupt (void *pvParameters)
{
    uint8_t gpio_num;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            mma845x_float_data_t  data;
    
            // test for new data
            if (!mma845x_new_data (sensor))
                continue;
    
            // fetch new data
            if (mma845x_get_float_data (sensor, &data))
            {
                // do something with data
                ...
            }
        }
    }
}
...

// create a task that is triggered in case of interrupts
xTaskCreate(user_task_interrupt, "user_task_interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);
...
```

Finally, interrupt handlers have to be activated for the GPIOs which are connected to the interrupt signals.

```
// configure interrupt pins for *INT1* and *INT2* signals and set the interrupt handler
gpio_set_interrupt(INT1_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);
gpio_set_interrupt(INT2_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);
```

Furthermore, the interrupts have to be enabled and configured in the MMA845X sensor, see section **Interrupts** above.

## Full Example

In the following example, all the different functions of the sensor can be tested by defining one or more of the following constants. The default example mode is ```PERIODIC```.

A good demonstration of the sensor's features is the combination of the constants ```PERIODIC```, ```FIFO_MODE```, ```INT_TRANS```, and ```AUTOSLEEP```. Using these constants, the sensor generates output data at a rate of 50 Hz that are stored in the FIFO. The FIFO is periodically read by the user task every 100 ms. After 3.2 seconds without interrupts, the sensor goes into sleep mode and reduces the output data rate to 1.56 Hz. If a transient event is detected, e.g., by shaking the sensor, an interrupt is generated and the sensor returns to the output data rate of 50 Hz.

```
/* -- use following constants to define the example mode ----------- */

// #define PERIODIC    // fetch data periodically (not possible in combination with INT_DATA)
// #define FIFO_MODE   // multiple sample read mode
// #define INT_DATA    // data interrupts used (data ready and FIFO status)
// #define INT_EVENT   // inertial event interrupts used (free fall, motion)
// #define INT_TRANS   // transient event interrupts used
// #define INT_PULSE   // single/double tap detection interrupts used
// #define INT_ORIENT  // orientation change interrupts used
// #define AUTOSLEEP   // autosleep used

#if !defined(PERIODIC)
#define PERIODIC
#endif

#if defined(INT_DATA)
#define INT_USED
#undef PERIODIC        // to avoid the combination of INT_DATA and PERIODIC
#endif
#if defined(INT_EVENT) || defined(INT_TRANS) || defined(INT_PULSE) || defined(INT_ORIENT)
#define INT_USED
#endif
#if defined(AUTOSLEEP)
#define INT_USED
#endif

/* -- includes ----------------------------------------------------- */

#include "mma845x.h"

/** -- platform dependent definitions ------------------------------ */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

#else  // ESP8266 (esp-open-rtos)

// user task stack depth for ESP8266
#define TASK_STACK_DEPTH 512

#endif  // ESP_PLATFORM

// I2C interface defintions for ESP32 and ESP8266
#define I2C_BUS       0
#define I2C_SCL_PIN   14
#define I2C_SDA_PIN   13
#define I2C_FREQ      I2C_FREQ_400K

// interrupt GPIOs defintions for ESP8266 and ESP32
#define INT1_PIN      5
#define INT2_PIN      4

/* -- user tasks --------------------------------------------------- */

static mma845x_sensor_t* sensor;

/**
 * Common function used to get sensor data.
 */
void read_data ()
{
    #ifdef FIFO_MODE

    mma845x_float_data_fifo_t fifo;

    if (mma845x_new_data (sensor))
    {
        uint8_t num = mma845x_get_float_data_fifo (sensor, fifo);

        printf("%.3f MMA845X num=%d\n", (double)sdk_system_get_time()*1e-3, num);

        for (int i=0; i < num; i++)
            // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
            printf("%.3f MMA845X (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
                   (double)sdk_system_get_time()*1e-3, 
                   fifo[i].ax, fifo[i].ay, fifo[i].az);
    }

    #else

    mma845x_float_data_t  data;

    if (mma845x_new_data (sensor) &&
        mma845x_get_float_data (sensor, &data))
        // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
        printf("%.3f MMA845X (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
               (double)sdk_system_get_time()*1e-3, 
                data.ax, data.ay, data.az);
        
    #endif // FIFO_MODE
}


#ifdef INT_USED
/**
 * When interrupts are used, handlers must be defined that must be kept small.
 * Therefore, the interrupt handler only sends an event to a waiting task that
 * handles the interrupts from different sources.
 */

static QueueHandle_t gpio_evt_queue = NULL;

// Interrupt handler which sends an interrupt event to a user task

void IRAM int_signal_handler (uint8_t gpio)
{
    // send an event with GPIO to the interrupt user task
    xQueueSendFromISR(gpio_evt_queue, &gpio, NULL);
}

// User task that reacts on interrupt events

void user_task_interrupt (void *pvParameters)
{
    uint8_t gpio_num;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            if (gpio_num == INT2_PIN)
            {
                // INT2: data ready and FIFO interrupts are routed to INT2

                // get the source of the interrupt
                mma845x_int_data_source_t data_src;
                mma845x_get_int_data_source (sensor, &data_src);

                // in case of data ready interrupt read one data sample and do something with data
                if (data_src.data_ready)
                    read_data ();
   
                // in case of FIFO interrupts read the whole FIFO and do something with data
                else if (data_src.fifo_watermark || data_src.fifo_overrun)
                    read_data ();
            }
            else if (gpio_num == INT1_PIN)
            {
                // INT1: all other interrupts are routed to INT1 in the exampled
                
                // get the source of the interrupt
                mma845x_int_status_t int_status;
                mma845x_get_int_status (sensor, &int_status);

                // in case of inertial event interrupt
                if (int_status.event)
                {
                    // get the source of the interrupt and reset *INTx* signals
                    mma845x_int_event_source_t event_src = {};
                    mma845x_get_int_event_source (sensor, &event_src);

                    if (event_src.active)
                    {
                        printf("%.3f MMA845X motion event: ", (double)sdk_system_get_time()*1e-3);
                        if (event_src.x_motion) printf("%cx ", event_src.x_sign ? '-' : '+');
                        if (event_src.y_motion) printf("%cy ", event_src.y_sign ? '-' : '+');
                        if (event_src.z_motion) printf("%cz ", event_src.z_sign ? '-' : '+');
                        printf("\n");
                    }
                }
            
                // in case of transient event interrupt
                if (int_status.transient)
                {
                    // get the source of the interrupt and reset *INTx* signals
                    mma845x_int_transient_source_t trans_src = {};
                    mma845x_get_int_transient_source (sensor, &trans_src);
    
                    if (trans_src.active)
                    {
                        printf("%.3f MMA845X transient event: ", (double)sdk_system_get_time()*1e-3);
                        if (trans_src.x_event) printf("%cx ", trans_src.x_sign ? '-' : '+');
                        if (trans_src.y_event) printf("%cy ", trans_src.y_sign ? '-' : '+');
                        if (trans_src.z_event) printf("%cz ", trans_src.z_sign ? '-' : '+');
                        printf("\n");
                    }
                }    
    
                // in case of tap detection interrupt
                if (int_status.pulse)
                {
                    // get the source of the interrupt and reset *INTx* signals
                    mma845x_int_pulse_source_t pulse_src = {};
                    mma845x_get_int_pulse_source (sensor, &pulse_src);
        
                    if (pulse_src.active)
                       printf("%.3f MMA845X %s\n", (double)sdk_system_get_time()*1e-3, 
                              pulse_src.d_pulse ? "double tap" : "single tap");
                }
    
                // in case of orientation change detection interrupt
                if (int_status.orientation)
                {
                    // get the source of the interrupt and reset *INTx* signals
                    mma845x_orientation_status_t orient;
                    mma845x_get_orientation (sensor, &orient);
    
                    if (orient.changed)
                    {  
                        printf("%.3f MMA845X orientation ", (double)sdk_system_get_time()*1e-3);
                        switch (orient.portrait_landscape)
                        {
                            case mma845x_portrait_up:     printf("portait up" ); break;
                            case mma845x_portrait_down:   printf("portait down" ); break;
                            case mma845x_landscape_left:  printf("landscape left" ); break;
                            case mma845x_landscape_right: printf("landscape right" ); break;
                        }
                        printf(", %s\n", orient.back_front == mma845x_back ? "back" : "front");
                    }
                }
                
                // in case of auto-sleep/wake interrupt
                if (int_status.autosleep)
                {
                    printf("%.3f MMA845X new system mode: ", (double)sdk_system_get_time()*1e-3);
                    switch (mma845x_get_system_mode(sensor))
                    {
                        case mma845x_standby_mode: printf("standby"); break;
                        case mma845x_wake_mode:    printf("wake");    break;
                        case mma845x_sleep_mode:   printf("sleep");   break;
                        default: printf("unknown");
                    }
                    printf("\n");
                }
            }
        }
    }
}

#endif

#ifdef PERIODIC
/*
 * User task that fetches the sensor values periodically every 100 ms.
 */

void user_task_periodic(void *pvParameters)
{
    // small delay before accessing the sensor the first time
    vTaskDelay (100/portTICK_PERIOD_MS);
    
    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // read sensor data
        read_data ();
        
        // passive waiting until 100 ms are over
        vTaskDelayUntil(&last_wakeup, 100/portTICK_PERIOD_MS);
    }
}

#endif // defined(PERIODIC) && !defined(INT_DATA)

/* -- main program ------------------------------------------------- */

void user_init(void)
{
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);

    /** -- MANDATORY PART -- */

    #ifdef SPI_USED

    // init the sensor connnected to SPI
    spi_bus_init (SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);

    // init the sensor connected to SPI_BUS with SPI_CS_GPIO as chip select.
    sensor = mma845x_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    
    #else

    // init all I2C bus interfaces at which MMA845X  sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    
    // init the sensor with slave address MMA845X_I2C_ADDRESS_2 connected to I2C_BUS.
    sensor = mma845x_init_sensor (I2C_BUS, MMA845X_I2C_ADDRESS_2);

    #endif
    
    if (sensor)
    {
        #ifdef INT_USED

        /** --- INTERRUPT CONFIGURATION PART ---- */
        
        // Interrupt configuration has to be done before the sensor is set
        // into measurement mode to avoid losing interrupts

        // create an event queue to send interrupt events from interrupt
        // handler to the interrupt task
        gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));

        // configure interupt pins for *INT1* and *INT2* signals and set the interrupt handler
        gpio_enable(INT1_PIN, GPIO_INPUT);
        gpio_enable(INT2_PIN, GPIO_INPUT);
        gpio_set_interrupt(INT1_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);
        gpio_set_interrupt(INT2_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);

        #endif // INT_USED
        
        /** -- SENSOR CONFIGURATION PART --- */

        // set polarity and type of INT signals if necessary
        mma845x_config_int_signals (sensor, mma845x_high_active, mma845x_push_pull);

        #ifdef INT_DATA
        // enable data interrupts on INT2 (data ready or FIFO status interrupts)
        // data ready and FIFO status interrupts must not be enabled at the same time
        #ifdef FIFO_MODE
        mma845x_enable_int (sensor, mma845x_int_fifo_overrun  , mma845x_int2_signal, true);
        mma845x_enable_int (sensor, mma845x_int_fifo_watermark, mma845x_int2_signal, true);
        #else
        mma845x_enable_int (sensor, mma845x_int_data_ready, mma845x_int2_signal, true);
        #endif // FIFO_MODE
        #endif // INT_DATA
        
        #ifdef INT_EVENT
        // enable inertial event interrupts on INT1 
        // configuration of registers FF_MT_CFG, FF_MT_THS, FF_MT_COUNT 
        // see datasheet and application note AN4070

        mma845x_int_event_config_t event_config;
    
        // event_config.mode = mma845x_free_fall;
        event_config.mode = mma845x_motion;     // OR-condition
        event_config.threshold = 2;             // 0.063 g/count = 0.1266 g
        event_config.x_enabled = true;          // x-axis is used for event detection
        event_config.y_enabled = true;          // y-axis is used for event detection
        event_config.z_enabled = false;         // z-axis is not used for event detection
        event_config.debounce_cnt = 5;          // 100 ms at ODR=50Hz in normal oversampling mode
        event_config.debounce_cnt_decr = true;  // increment and decrement debounce counter
        event_config.latch = true;              // interrupt is latched until its source is read
        event_config.sleep_active = true;       // active in sleep mode
        
        mma845x_set_int_event_config (sensor, &event_config);
        mma845x_enable_int (sensor, mma845x_int_event, mma845x_int1_signal, true);
        #endif // INT_EVENT

        #ifdef INT_TRANS
        // enable transient event interrupts on INT1 
        // configuration of registers TRANSIENT_CFG, TRANSIENT_THS, TRANSIENT_COUNT 
        // see datasheet and application note AN4071

        mma845x_int_transient_config_t trans_config;
    
        trans_config.threshold = 8;             // 0.063 g/count = 0.504 g
        trans_config.x_enabled = true;          // x-axis is used for event detection
        trans_config.y_enabled = true;          // y-axis is used for event detection
        trans_config.z_enabled = false;         // z-axis is not used for event detection
        trans_config.debounce_cnt = 2;          // 40 ms at ODR=50Hz in normal oversampling mode
        trans_config.debounce_cnt_decr = false; // increment and clear debounce counter
        trans_config.latch = true;              // interrupt is latched until its source is read
        trans_config.hpf_bypassed = false;      // HPF is not bypassed (default)
        trans_config.sleep_active = true;       // active in sleep mode
        
        mma845x_set_int_transient_config (sensor, &trans_config);
        mma845x_enable_int (sensor, mma845x_int_transient, mma845x_int1_signal, true);
        #endif // INT_TRANS

        #ifdef INT_PULSE
        // enable single/double tap detection interrupts on INT1
        // configuration of registers PULSE_CFG, PULSE_THS*, PULSE_TMLT, PULSE_LTCY, PULSE_WIND
        // see datasheet and application note AN4072
        
        mma845x_int_pulse_config_t pulse_config;
        
        pulse_config.x_threshold = 25;      // 0.063 g/count = 1.575 g 
        pulse_config.y_threshold = 25;      // 0.063 g/count = 1.575 g 
        pulse_config.z_threshold = 42;      // 0.063 g/count = 1.65 g 
        pulse_config.x_single = true;       // single tap detection in x direction enabled
        pulse_config.x_double = false;      // double tap detection in x direction enabled
        pulse_config.y_single = true;       // single tap detection in y direction enabled
        pulse_config.y_double = false;      // double tap detection in y direction enabled
        pulse_config.z_single = true;       // single tap detection in z direction enabled
        pulse_config.z_double = false;      // double tap detection in y direction enabled
        pulse_config.pulse_limit   = 10;    //  50 ms at ODR=50 Hz in normal mode and no LPF
        pulse_config.pulse_latency = 20;    // 200 ms at ODR=50 Hz in normal mode and no LPF
        pulse_config.pulse_window  = 30;    // 300 ms at ODR=50 Hz in normal mode and no LPF
        pulse_config.latch = true;          // interrupt is latched until its source is read
        pulse_config.hpf_bypassed = false;  // HPF is not bypassed (default)
        pulse_config.lpf_enabled  = false;  // LPF is not used (default)
        pulse_config.sleep_active = true;   // active in sleep mode
        
        mma845x_set_int_pulse_config (sensor, &pulse_config);
        mma845x_enable_int (sensor, mma845x_int_pulse, mma845x_int1_signal, true);
        #endif // INT_PULSE

        #ifdef INT_ORIENT
        // enable orientation change detection interrupts on INT1
        // configuration of registers PL_CFG, PL_COUNT, PL_BF_ZCOMP, PL_THS
        // see datashee and applicatio note AN4068
        
        mma845x_orientation_config_t orient_config;

        orient_config.enabled = true;           // enable portrait/landscape detection
        orient_config.sleep_active = true;      // active in sleep mode
        orient_config.pl_threshold = 16;        // trip angle 45°   
        orient_config.pl_hysteresis= 4;         // hysteresis +-14°
        orient_config.bf_threshold = 1;         // 285° < z < 75° back->front, 
                                                // 105° < z < 255° front->back 
        orient_config.z_lock = 4;               // z >= 29°
        orient_config.debounce_cnt = 5;         // 100 ms at ODR=50Hz in normal mode
        orient_config.debounce_cnt_decr = true; // increment and decrement debounce counter

        mma845x_set_orientation_config (sensor, &orient_config);
        mma845x_enable_int (sensor, mma845x_int_orientation, mma845x_int1_signal, true);
        #endif // INT_ORIENT

        #ifdef AUTOSLEEP
        // switch to autosleep after 3.2 seconds of inactivity
        // configuration of registers ASLP_COUNT, CTRL_REG1, CTRL_REG2
        // see datashee and applicatio note AN4074
        mma845x_set_autosleep (sensor, mma845x_normal, mma845x_aslp_rate_1_56, 10, true);
        mma845x_enable_int (sensor, mma845x_int_autosleep, mma845x_int1_signal, true);
        #endif // AUTOSLEEP

        #ifdef FIFO_MODE
        // activate FIFO mode if needed
        // configuration of registers F_SETUP, TRIG_CFG, CTRL_REG3
        // see datashee and applicatio note AN4073
        mma845x_fifo_trigger_t trigger = {};
        mma845x_set_fifo_mode (sensor, mma845x_fill, 20, trigger, false);
        #endif

        // config HPF and enable it for sensor output data
        mma845x_config_hpf (sensor, 0, true);
        
        // LAST STEP: Finally set scale and mode to start measurements
        mma845x_set_scale(sensor, mma845x_scale_2_g);
        mma845x_set_mode (sensor, mma845x_high_res, mma845x_odr_50, true, false);

        /** -- TASK CREATION PART --- */

        // must be done last to avoid concurrency situations with the sensor
        // configuration part

        #ifdef INT_USED
        // create a task that is triggered in case of interrupts
        xTaskCreate(user_task_interrupt, "user_task_interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);
        #endif 

        #ifdef PERIODIC
        // create a user task that fetches data from sensor periodically
        xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);
        #endif
    }
    else
        printf("Could not initialize MMA845X sensor\n");
}

```
