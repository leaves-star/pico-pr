#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "ws2812b.pio.h"
#include "pico/divider.h"
#include "pico/multicore.h"

/* global */
#define SIX_SECONDS (6000000)

#define PIN_MASK_TACH   ( (1<<18)|(1<<19)|(1<<20)|(1<<21) )
#define PIN_MASK_TEMP   ( (1<<4) |(1<<5) |(1<<6) |(1<<7) )
                        //  1098765432109876543210
#define MT_HLLH_MASK     (0b0101000000000010010000)  //pin1/4
#define MT_HLLL_MASK     (0b0100000000000010000000)  //pin1
#define MT_LLHL_MASK     (0b0010000000000000100000)  //pin3
#define MT_LHHL_MASK     (0b1010000000000001100000)  //pin2/3
#define MT_LHLL_MASK     (0b1000000000000001000000)  //pin2
#define MT_LLLH_MASK     (0b0001000000000000010000)  //pin4

#define RO_TURN_L (-1)
#define RO_TURN_R (1)
#define RO_STOP   (0)
#define MAX_X27_COUNT_STEP  (945)   // 315degree x 3step = 945
#define MAX_TACHO_STEPS     (675)   // Maximum value on the tachometer scale. 
                                    // 720 means that the shaft rotates 240 degrees.
#define MAX_WTEMP_STEPS     (540)   // 540 means that the shaft rotates 180 degrees.
#define ACC_MODE_LO         (11)
#define ACC_MODE_HI         (4)     // <<setting>>If the needle is heavy,
                                    // 765 means that the shaft rotates 255 degrees.
#define MT_BUFFER_COUNT     (ACC_MODE_LO-ACC_MODE_HI+1)
#define MT_BUFFER_COUNT_END (MT_BUFFER_COUNT-1)

#define RPM_ALERT_RED       (950)
#define RPM_ALERT_YELLOW    (800)


/* Pin */
#define PULSE_PIN   (3)
#define OIL_PIN     (15)
#define HIBEAM_PIN  (16)
#define FLASHER_PIN (17)
#define TEMP_PIN    (26)
#define LUX_PIN     (27)

#define ADC_WTEMP   (0)
#define ADC_LUX     (1)
#define ADC_AIR     (4)

uint32_t    g_rpm = 0;
bool g_flag_timeout_check = false;

/* WS2812B Define */
#define WS2812_PIN          (2)
#define NUM_PIXELS          (20)
#define NUM_CIRCLE_PIXELS   (16)
#define WTEMP_NEEDLE_PIXEL  (16)
#define TACHO_NEEDLE_PIXEL  (18)

const uint LED_PIN = 25; // 内蔵LEDピン
volatile bool led_state = false; // LEDの状態を保持
uint32_t needle_pos_thresholds[NUM_CIRCLE_PIXELS+1] 
    = {0,22,67,112,157,202,247,292,337,382,427,472,517,562,607,652,MAX_X27_COUNT_STEP};
//     0  1  2   3   4   5   6   7   8   9  10  11  12  13  14  15  16

/* Color code */
#define COL_WHITE   (0xFFFFFF)
#define COL_RED     (0xFF0000)
#define COL_BLUE    (0x0000FF)
#define COL_GREEN   (0x00FF00) 
#define COL_YELLOW  (0xFFFF00)
#define COL_ORANGE  (0xFD7E00)

typedef struct _PIODATA {
    PIO pio;
    uint sm;
    uint offset;
} PIODATA;

#define MOTOR_NUMBER_0 (0) // Tacho meter
#define MOTOR_NUMBER_1 (1) // Tmp meter

typedef struct _MOTOR_STATE {
    int8_t motor_number = MOTOR_NUMBER_0;
    int8_t motor_phase = 0;
    uint8_t acceleration_mode = ACC_MODE_LO;
    int8_t dir_buffer[MT_BUFFER_COUNT] = {RO_STOP};
    int16_t pos_current = 0;
    int16_t pos_target = 0;
} MOTOR_STATE;

mutex_t mutex;

/**
 * @class int16_t_with_mutex
 * @brief A thread-safe wrapper around an int16_t value using a mutex.
 *
 * This class provides safe concurrent access to an int16_t variable by
 * protecting read and write operations with a mutex. It supports assignment
 * and implicit conversion to int16_t, ensuring that access is synchronized.
 */
class int16_t_with_mutex {
private:
    int16_t value; ///< The protected integer value.

public:
    /**
     * @brief Constructs a new int16_t_with_mutex object.
     * @param v Initial value to assign to the internal variable. Defaults to 0.
     */
    int16_t_with_mutex(int16_t v = 0) : value(v) {}

    /**
     * @brief Thread-safe assignment operator.
     *
     * Assigns a new value to the internal variable,
     * using a blocking mutex to ensure exclusive access.
     * @param v The value to assign.
     * @return Reference to the current object.
     */
    int16_t_with_mutex& operator=(int16_t v) {
        mutex_enter_blocking(&mutex);
        value = v;
        mutex_exit(&mutex);
        return *this;
    }

    /**
     * @brief Thread-safe implicit conversion operator.
     *
     * Retrieves the current value in a thread-safe manner,
     * using a blocking mutex to ensure safe access.
     * @return The stored int16_t value.
     */
    operator int16_t() const {
        mutex_enter_blocking(&mutex);
        int16_t c = value;
        mutex_exit(&mutex);
        return c;
    }
};

/**
 * @class uint16_t_average
 * @brief Computes a running average over a fixed-size buffer of uint16_t values.
 *
 * This class maintains a circular buffer of recent uint16_t values and
 * provides an implicit cast operator to retrieve their average. It is useful
 * for smoothing noisy signals or calculating moving averages in embedded applications.
 */
class uint16_t_average {
    static constexpr uint8_t MAX_SIZE = 20; ///< Maximum number of values that can be stored.

    uint16_t values[MAX_SIZE] = {0}; ///< Circular buffer storing the most recent values.
    uint8_t count = 0;               ///< Number of valid entries currently stored in the buffer.
    uint8_t index = 0;               ///< Index of the next value to overwrite in the buffer.
    uint8_t array_size = MAX_SIZE;   ///< Effective buffer size, limited by MAX_SIZE.

public:
    /**
     * @brief Constructs a uint16_t_average with a specified buffer size.
     * @param limit_set Desired size of the buffer; capped at MAX_SIZE.
     */
    explicit uint16_t_average(uint8_t limit_set = MAX_SIZE) {
        array_size = (limit_set >= MAX_SIZE) ? MAX_SIZE : limit_set;
    }

    /**
     * @brief Inserts a new value into the circular buffer.
     *
     * Overwrites the oldest value if the buffer is full,
     * and updates the average accordingly.
     *
     * @param value The new uint16_t value to add to the buffer.
     */
    void operator=(uint16_t value) {
        values[index] = value;
        index = (index + 1) % array_size;
        if (count < array_size) count++;
    }

    /**
     * @brief Computes the average of the stored values.
     *
     * Returns 0 if no values have been added yet.
     *
     * @return The average of the stored uint16_t values.
     */
    operator uint16_t() const {
        if (count == 0) return 0;
        uint32_t sum = 0;
        for (size_t i = 0; i < count; ++i) {
            sum += values[i];
        }
        return static_cast<uint16_t>(sum / count);
    }
};


// Global values related needles.
int16_t_with_mutex g_tacho_pos_current;
int16_t_with_mutex g_tacho_pos_target;
int16_t_with_mutex g_wtemp_pos_current;
int16_t_with_mutex g_wtemp_pos_target;

/* proto type */
void led_setup();
void core1_entry();
void x27_motor(MOTOR_STATE* pms);
void Pixels_control(PIO pio, uint sm, uint32_t lux, uint16_t rpm, uint16_t wtemp, uint16_t airtemp);
void pulse_callback(uint gpio, uint32_t events);
bool pulse_callback_timeout_check(struct repeating_timer *t);

uint16_t rpm_to_step(uint32_t rpm);
uint16_t adc_to_wtemp(uint16_t adc);
uint16_t wtemp_adc_to_step(uint16_t temp);
float read_internal_temperature();
void initialize_motor(PIODATA* piodata);
static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb);
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);
uint32_t urgbp_u32(uint32_t rgb, uint8_t p);
void pin_setup();

/**
 * Entry point for the application.
 * Initializes peripherals, sets up motor control, and runs the main control loop.
 *
 * @return int Always returns 0 (should never reach).
 */
int main()
{
    stdio_init_all();
    mutex_init(&mutex);
    adc_init();
    pin_setup();

    // Enable internal tempeture sensor
    adc_set_temp_sensor_enabled(true);

    // PIO setup
    PIODATA piodata;
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &piodata.pio, &piodata.sm, &piodata.offset, WS2812_PIN, 1, true);
    hard_assert(success);
    ws2812_program_init(piodata.pio, piodata.sm, piodata.offset, WS2812_PIN, 800000, false); 

    // Core1 Start
    multicore_launch_core1(core1_entry);

    // Initial operation to align the needle.
    initialize_motor(&piodata);

    // Puls pin irq setup
    gpio_set_irq_enabled_with_callback(PULSE_PIN, GPIO_IRQ_EDGE_FALL, true, &pulse_callback);

    // Start pulse signal timeout monitoring
    struct repeating_timer timer_pulse_timeout_check;
    add_repeating_timer_ms(100, pulse_callback_timeout_check, NULL, &timer_pulse_timeout_check);
    
    MOTOR_STATE wtemp;
    wtemp.motor_number = MOTOR_NUMBER_1;

    uint16_t_average adc_lux;
    uint16_t_average adc_wtemp;
    uint16_t_average air_temp;
    uint16_t_average pos_target(10);
    while(1) {
        pos_target = rpm_to_step(g_rpm);
        g_tacho_pos_target = pos_target;

        // Water sensor
        adc_select_input(ADC_WTEMP);
        adc_wtemp = adc_read();

        // Lux temperature
        adc_select_input(ADC_LUX);
        adc_lux = adc_read();

        // Pico inner temperature
        air_temp = (uint16_t)read_internal_temperature();

        //printf("RPM: %d ADC0/tmp: %d ADC1/lux: %d temperature: %d\n", g_rpm, (uint16_t)adc_wtemp, (uint16_t)adc_lux, (uint16_t)air_temp);

        wtemp.pos_target = wtemp_adc_to_step(adc_wtemp);
        x27_motor(&wtemp);

        // LED control
        Pixels_control(piodata.pio,piodata.sm,adc_lux, g_rpm, adc_wtemp, air_temp);

        sleep_ms(10);
    }
    // This will free resources and unload pio program
    pio_remove_program_and_unclaim_sm(&ws2812_program, piodata.pio, piodata.sm, piodata.offset);
}

/**
 * Entry point for the second core.
 * Handles tacho meter motor control independently.
 */
void core1_entry() {
    uint motor_wait_us[]
     //= {556,570,600,630,740,838,1040,1292,1630,2008,2500,3168,4000};
        = {611,627,660,693,814,921,1144,1421,1793,2208,2750,3484,4400};

    uint motor_wait_us_size = sizeof(motor_wait_us)/sizeof(motor_wait_us[0]);

    uint64_t start_time, past_time;
    MOTOR_STATE tacho;
    tacho.motor_number = MOTOR_NUMBER_0;

    while (true) {
        start_time = to_us_since_boot(get_absolute_time());

        // Drive tacho meter
        tacho.pos_current = g_tacho_pos_current;
        tacho.pos_target = g_tacho_pos_target;
        x27_motor(&tacho);

        g_tacho_pos_current = tacho.pos_current;
        g_tacho_pos_target = tacho.pos_target;

        past_time = to_us_since_boot(get_absolute_time()) - start_time;
        if(motor_wait_us[tacho.acceleration_mode] > past_time) {
            sleep_us(motor_wait_us[tacho.acceleration_mode]-past_time);
        }

    } // end of while(true)
}

/**
 * Drives an X27 stepper motor to the desired position.
 * Handles direction, phase, and acceleration logic.
 *
 * @param pms Pointer to a MOTOR_STATE struct describing the motor to be controlled.
 */
void x27_motor(MOTOR_STATE* pms) {
    uint8_t ii;
    int8_t l_new_dir = RO_STOP, sum_dir = 0;
    uint8_t unmatch_flag;

    if (pms->pos_current > pms->pos_target) {
        l_new_dir = RO_TURN_L;
        pms->pos_current--;
    } else if (pms->pos_current < pms->pos_target) {
        l_new_dir = RO_TURN_R;
        pms->pos_current++;
    } else {
        l_new_dir = RO_STOP;
    }

    /*
     * Set motor phase.
     * X27 has 6 different phases. By changing the phase in sequence with
     * the digital output signal, X27 rotates.
     * For details, please check the X27 specifications.
     */
     if (RO_TURN_L == pms->dir_buffer[0]) {
        if (5 == pms->motor_phase) {   // 5 is max of l_motor_phase
            pms->motor_phase = 0;
        } else {
            pms->motor_phase++;
        }
    } else if (RO_TURN_R == pms->dir_buffer[0]) {
        if (0 == pms->motor_phase) {
            pms->motor_phase = 5;
        } else {
            pms->motor_phase--;
        }
    } else {
        // RO_STOP does nothing.
    }

    // Rotate motor.
    uint32_t mask;
    if(MOTOR_NUMBER_0 == pms->motor_number) {
        mask = PIN_MASK_TACH;
    } else if(MOTOR_NUMBER_1 == pms->motor_number) {
        mask = PIN_MASK_TEMP;
    }
    switch (pms->motor_phase) {
        case 0: gpio_put_masked(mask,MT_LLLH_MASK);break; //MTLLLH(); break;
        case 1: gpio_put_masked(mask,MT_LHLL_MASK);break; //MTLHLL(); break;
        case 2: gpio_put_masked(mask,MT_LHHL_MASK);break; //MTLHHL(); break;
        case 3: gpio_put_masked(mask,MT_LLHL_MASK);break; //MTLLHL(); break;
        case 4: gpio_put_masked(mask,MT_HLLL_MASK);break; //MTHLLL(); break;
        case 5: gpio_put_masked(mask,MT_HLLH_MASK);break; //MTHLLH(); break;
        default: break;
    }

    /*
     * Set up a buffer for look-ahead.
     * If it is found that the direction of the motor will change,
     * unmatch_flag turn on, and reduce the speed of the needle in advance.
     */
    unmatch_flag = false;
    for (ii = 0; ii < (MT_BUFFER_COUNT_END); ii++) {

        pms->dir_buffer[ii] = pms->dir_buffer[ii + 1];
        
        if (l_new_dir != pms->dir_buffer[ii]) {
            unmatch_flag = true;
        }
    }
    pms->dir_buffer[ii] = l_new_dir;

    /*
     * Next acceleration mode
     * If the new rotation direction is different from the rotation in the buffer,
     * the rotation speed will be slowed down.
     * If they are all the same, the rotation speed will be accelerated.
     */
    if (unmatch_flag) {
        // Lower limit of needle speed.
        if (pms->acceleration_mode < ACC_MODE_LO) {
            pms->acceleration_mode++;
        }
    } else {
        // Upper limit of needle speed.
        if (pms->acceleration_mode > ACC_MODE_HI) {
            pms->acceleration_mode--;
        }
    }
}

/**
 * Controls WS2812B LEDs to display tachometer and temperature status.
 *
 * @param pio PIO instance.
 * @param sm State machine ID.
 * @param lux Current lux value for brightness control.
 * @param rpm Current rpm value..
 * @param wtemp Current water temperature value.
 */
void Pixels_control(PIO pio, uint sm, uint32_t lux, uint16_t rpm, uint16_t wtemp,uint16_t air_temp) {
    uint32_t pixels[NUM_PIXELS] = {0};
    int ii;

    // illuminance adjustment
    uint8_t power;
    /*照度センサーの値によって、20～250の範囲でLEDを発光する*/
    if(lux>1200) {
        power = 125;
    } else {
        power = (lux / 15) + 50;
    }
    if( air_temp > 60) {
        power *= 0.5;
    }

    pixels[TACHO_NEEDLE_PIXEL]      = urgbp_u32(COL_WHITE,power);
    pixels[TACHO_NEEDLE_PIXEL+1]    = urgbp_u32(COL_WHITE,power);

    if(1 == gpio_get(FLASHER_PIN)) {
        for(ii=0; ii < NUM_CIRCLE_PIXELS; ii++) {
            pixels[ii] = urgbp_u32(COL_ORANGE, 125);
        }
    } else {
        if(rpm >= RPM_ALERT_RED) { // rpm over 9,500
            for(ii=0; ii < NUM_CIRCLE_PIXELS; ii++) {
                pixels[ii] = urgbp_u32(COL_RED, power);
            }
            pixels[TACHO_NEEDLE_PIXEL]      = urgbp_u32(COL_RED,power);
            pixels[TACHO_NEEDLE_PIXEL+1]    = urgbp_u32(COL_RED,power);
        } else if(rpm >= RPM_ALERT_YELLOW) { 
            for(ii=0; ii < NUM_CIRCLE_PIXELS; ii++) {
                pixels[ii] = urgbp_u32(COL_YELLOW,power);
            }
        } else {
            for(ii=0; ii < NUM_CIRCLE_PIXELS; ii++) {
                pixels[ii] = urgbp_u32(COL_WHITE,power);
            }
        }
    }

    if(wtemp>2134) { // Blue: Water temp under 49c
        pixels[WTEMP_NEEDLE_PIXEL]      = urgbp_u32(COL_BLUE,power);
        pixels[WTEMP_NEEDLE_PIXEL+1]    = urgbp_u32(COL_BLUE,power);
    } else if(wtemp>1188) { // Green 50c to 79c
        pixels[WTEMP_NEEDLE_PIXEL]      = urgbp_u32(COL_GREEN,power);
        pixels[WTEMP_NEEDLE_PIXEL+1]    = urgbp_u32(COL_GREEN,power);
    } else if(wtemp>958) {  // Yellow 80c to 89c
        pixels[WTEMP_NEEDLE_PIXEL]      = urgbp_u32(COL_YELLOW,power);
        pixels[WTEMP_NEEDLE_PIXEL+1]    = urgbp_u32(COL_YELLOW,power);
    } else {                // Red over 90c
        pixels[TACHO_NEEDLE_PIXEL]      = urgbp_u32(COL_RED,power);
        pixels[TACHO_NEEDLE_PIXEL+1]    = urgbp_u32(COL_RED,power);
        pixels[WTEMP_NEEDLE_PIXEL]      = urgbp_u32(COL_RED,power);
        pixels[WTEMP_NEEDLE_PIXEL+1]    = urgbp_u32(COL_RED,power);
    }

    // Send LED data to PIO
    for(ii=0; ii<NUM_PIXELS;ii++){
        put_pixel(pio, sm, pixels[ii]);   
    }

}

/**
 * GPIO callback function for pulse input.
 * Calculates RPM based on pulse timing.
 *
 * @param gpio The GPIO number that triggered the callback.
 * @param events The events that occurred on the GPIO pin.
 */
void pulse_callback(uint gpio, uint32_t events) {
    static uint64_t last_time = 0;
    uint64_t pulse_interval;
    uint64_t current_time = to_us_since_boot(get_absolute_time());
    uint32_t rpm;

    g_flag_timeout_check = true;

    if (last_time != 0) {
        pulse_interval = current_time - last_time;
        if( pulse_interval < 4000 ) { // Over 15,000rpm
            return; // This is probably noise signal.
        } else if( pulse_interval > 600000 ) { // Under 100rpm
            g_rpm = 0;
        } else {
            rpm = SIX_SECONDS / (pulse_interval);
            g_rpm = rpm;
        }
    }
    last_time = current_time;
}

/**
 * Repeating timer callback to check if the engine has stopped.
 *
 * If no pulse signal has been detected since the last check,
 * it resets the global RPM value to zero.
 *
 * @param t Pointer to the repeating_timer structure (not used).
 * @return true Always returns true to continue the timer.
 */
bool pulse_callback_timeout_check(struct repeating_timer *t) {
    if( g_flag_timeout_check ) {
        g_flag_timeout_check = false;
    } else {
        g_rpm = 0;
    }
    return true;
}

/**
 * Converts RPM value to corresponding step value for the stepper motor.
 * Customizable for different tachometer gauge characteristics.
 *
 * @param rpm Current RPM value.
 * @return uint16_t Corresponding step count for the stepper motor.
 */
uint16_t rpm_to_step(uint32_t rpm) {
    uint16_t target_step;
    if(rpm==0){
        target_step = 0;
    } else if(rpm<100){          // 0-900rpm -> 0-15step
        target_step = (uint16_t)(rpm*0.15);
    } else if(rpm<200) {         // 1000-1900rpm -> 15-45 step
        target_step = 15+(uint16_t)((rpm-100)*0.3);
    } else if(rpm<700) {         // 2000-5900rpm
        target_step = 45+(uint16_t)((rpm-200)*0.45);
    } else{                     // 7000rpm over 30deg/1000rpm
        target_step = 270+(uint16_t)(((rpm-700)*0.9));
    }

    if(target_step > MAX_TACHO_STEPS){
        target_step = MAX_TACHO_STEPS;
    }
    return target_step;
}

/**
 * Converts ADC value to water temperature in degrees Celsius.
 *
 * @param adc Raw ADC value.
 * @return uint16_t Water temperature estimate (20–120°C).
 */
uint16_t adc_to_wtemp(uint16_t adc) {
    static uint16_t wtemp_adc_list[] = { // Water temperature list from 20c to 120c
        3193,3162,3131,3099,3067,3034,3001,2967,2933,2899,2864,2829,2794,2758,2722,2686,2650,2613,2577,2540,2503,2466,2429,2392,2355,
        2318,2281,2244,2207,2170,2134,2097,2061,2025,1989,1954,1919,1884,1849,1815,1781,1747,1714,1681,1649,1617,1585,1554,1523,1492,
        1462,1433,1404,1375,1347,1319,1292,1265,1239,1213,1188,1163,1138,1114,1090,1067,1045,1022,1000, 979, 958, 938, 917, 898, 879,
        860, 841, 823, 805, 788, 771, 755, 738, 722, 707, 692, 677, 663, 648, 635, 621, 608, 595, 582, 570, 558, 546, 534, 523, 512 ,502
    };

    uint16_t wtemp;
    // Convert tempature from adc value.
    for(wtemp=0; wtemp<100; wtemp++) {
        if(adc > wtemp_adc_list[wtemp]) {
            break;
        }
    }
    return wtemp+20;
}

/**
 * Converts ADC temperature reading to step position for the water temperature meter.
 *
 * @param temp Raw ADC temperature value.
 * @return uint16_t Corresponding step count for the temperature meter.
 */
uint16_t wtemp_adc_to_step(uint16_t wtemp_adc) {
    uint16_t target_step;
    uint16_t wtemp;

    wtemp = adc_to_wtemp(wtemp_adc);

    // Convert steps from tempature.
    if(wtemp<50) { // under 50c
        target_step = (uint16_t)(4.5*(wtemp-20));
    } else if(wtemp>90) { // over 90c
        target_step = (uint16_t)(4.5*(wtemp-90))+405;
    } else { // between 50c-90c
        target_step = (uint16_t)(6.75*(wtemp-50))+135;
    }

    if(target_step > MAX_WTEMP_STEPS){
        target_step = MAX_WTEMP_STEPS;
    }

    return target_step;
}

/**
 * @brief Reads the internal temperature sensor and returns the temperature in degrees Celsius.
 *
 * This function selects ADC input 4, which is connected to the internal temperature sensor
 * on the Raspberry Pi Pico. It then reads the raw ADC value, converts it to voltage, and
 * calculates the corresponding temperature using the conversion formula provided in the datasheet:
 *
 * Temperature [°C] = 27 - (Vtemp - 0.706) / 0.001721
 *
 * @return float Temperature in degrees Celsius.
 */
float read_internal_temperature() {
    adc_select_input(ADC_AIR); // Internal temperature sensor is connected to ADC4
    uint16_t raw = adc_read();

    const float conversion_factor = 3.3f / (1 << 12); // 12-bit ADC conversion
    float voltage = raw * conversion_factor;
    float internal_temp = 27.0f - (voltage - 0.706f) / 0.001721f;
    return internal_temp;
}

/**
 * Initializes the stepper motor state and performs alignment and a demonstration.
 *
 * @param piodata Pointer to a PIODATA struct containing PIO and state machine info.
 */
void initialize_motor(PIODATA* piodata) {
    struct repeating_timer timer;
    MOTOR_STATE wtemp;
    uint16_t adc_wtemp = 0, adc_lux = 0;
    /*
     * First, turn the left fully to fix the position.
     * We never know the state of the shaft when the power is turned on.
     * By turning it beyond its limit, the shaft alignment is complete.
     */
    g_tacho_pos_target = 0;
    g_tacho_pos_current = MAX_X27_COUNT_STEP;

    adc_select_input(1);
    adc_lux = adc_read();

    Pixels_control(piodata->pio,piodata->sm,adc_lux,0,0,0);

    // Tacho needle roll back
    while (0!=g_tacho_pos_current) {sleep_ms(1);}

    // Water temp needle roll back
    wtemp.motor_number = MOTOR_NUMBER_1;
    wtemp.pos_current = MAX_X27_COUNT_STEP;
    wtemp.pos_target = 0;
    while(wtemp.pos_current != wtemp.pos_target) {
        x27_motor(&wtemp);
        sleep_ms(1);
    }

    /*
     * Just a demonstration, feel free to do as you like with it.
     */
    wtemp.pos_target = MAX_WTEMP_STEPS;
    adc_select_input(0);
    adc_wtemp = adc_read();
    adc_select_input(1);
    adc_lux = adc_read();

    for(int rpm=0;;rpm+=100) {
        g_tacho_pos_target = rpm_to_step(rpm);
        if(g_tacho_pos_target >= MAX_TACHO_STEPS) {
            break;
        }

        Pixels_control(piodata->pio,piodata->sm,adc_lux,rpm,adc_wtemp,0);
        while (g_tacho_pos_current != g_tacho_pos_target) {
            x27_motor(&wtemp);
            sleep_ms(1);
        }
        for(int jj = 0; jj < 20; jj++) {
            x27_motor(&wtemp);
            sleep_ms(10);
        }
    }

    /*
     * Last, turn the left fully to fix the position 0.
     */
    g_tacho_pos_target = 0;
    wtemp.pos_target = 0;
//    wtemp.pos_current = MAX_X27_COUNT_STEP;
    while ((g_tacho_pos_current != g_tacho_pos_target) || (wtemp.pos_current != wtemp.pos_target)) {
        x27_motor(&wtemp);
        sleep_ms(1);
    };

}

/**
 * Sends a 24-bit GRB pixel value to the WS2812B LED.
 *
 * @param pio PIO instance.
 * @param sm State machine ID.
 * @param pixel_grb 24-bit GRB pixel data.
 */
static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

/**
 * Converts individual RGB color components into a single 24-bit GRB value.
 *
 * @param r Red component (0-255).
 * @param g Green component (0-255).
 * @param b Blue component (0-255).
 * @return uint32_t GRB formatted pixel value.
 */
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

uint32_t urgbp_u32(uint32_t rgb, uint8_t p) {
    uint8_t r = ((rgb >> 16) & 0xFF);
    uint8_t g = ((rgb >> 8)  & 0xFF);
    uint8_t b = ((rgb)       & 0xFF);
    uint16_t sum = r+g+b;

    r = (uint8_t)((r*p)/sum);
    g = (uint8_t)((g*p)/sum);
    b = (uint8_t)((b*p)/sum);

    // The data array for WS2812B is GRB, not RGB.
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

/**
 * Initializes all necessary GPIO pins.
 */
void pin_setup() {
    // Tacho meter
    gpio_init_mask(PIN_MASK_TACH);
    gpio_set_dir_out_masked(PIN_MASK_TACH);
    // Temp meter
    gpio_init_mask(PIN_MASK_TEMP);
    gpio_set_dir_out_masked(PIN_MASK_TEMP);

    // Puls in
    gpio_init(PULSE_PIN);
    gpio_set_dir(PULSE_PIN, GPIO_IN);
    gpio_pull_up(PULSE_PIN);

    gpio_init(OIL_PIN);
    gpio_set_dir(OIL_PIN, GPIO_IN);
    gpio_pull_up(OIL_PIN);

    gpio_init(HIBEAM_PIN);
    gpio_set_dir(HIBEAM_PIN, GPIO_IN);
    gpio_pull_up(HIBEAM_PIN);

    gpio_init(FLASHER_PIN);
    gpio_set_dir(FLASHER_PIN, GPIO_IN);
    gpio_pull_up(FLASHER_PIN);

    // misc
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // adc
    // lux
    adc_gpio_init(TEMP_PIN);    // GPIO26（ADC0)
    adc_gpio_init(LUX_PIN);     // GPIO27（ADC1）

}

