/**
 * Simple example with one sensor connected to I2C. It demonstrates the
 * different approaches to fetch the data. Either one of the interrupt signals
 * is used or new data are fetched periodically.
 *
 * Harware configuration:
 *
 *   I2C
 *
 *   +-----------------+   +----------+
 *   | ESP8266 / ESP32 |   | MMA845X  |
 *   |                 |   |          |
 *   |   GPIO 14 (SCL) ----> SCL      |
 *   |   GPIO 13 (SDA) <---> SDA      |
 *   |   GPIO 5        <---- INT1     |
 *   |   GPIO 4        <---- INT2     |
 *   +-----------------+   +----------+
*/

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

    // init all I2C bus interfaces at which MMA845X  sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    
    // init the sensor with slave address MMA845X_I2C_ADDRESS_2 connected to I2C_BUS.
    sensor = mma845x_init_sensor (I2C_BUS, MMA845X_I2C_ADDRESS_2);

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
        event_config.debounce_cnt_mode = false; // increment and decrement debounce counter
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
        trans_config.debounce_cnt_mode = true;  // increment and clear debounce counter
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
        #endif // INT_PULSE

        #ifdef INT_ORIENT
        // enable orientation change detection interrupts on INT1
        // configuration of registers PL_CFG, PL_COUNT, PL_BF_ZCOMP, PL_THS
        // see datashee and applicatio note AN4068
        
        mma845x_orientation_config_t orient_config;

        orient_config.enabled = true;            // enable portrait/landscape detection
        orient_config.sleep_active = true;       // active in sleep mode
        orient_config.pl_threshold = 16;         // trip angle 45°   
        orient_config.pl_hysteresis= 4;          // hysteresis +-14°
        orient_config.bf_threshold = 1;          // 285° < z < 75° back->front, 
                                                 // 105° < z < 255° front->back 
        orient_config.z_lock = 4;                // z >= 29°
        orient_config.debounce_cnt = 5;          // 100 ms at ODR=50Hz in normal mode
        orient_config.debounce_cnt_mode = false; // increment and decrement debounce counter

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

