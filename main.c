 /*
 * ES Project: Group 5
 * @authors: Jiaying Wu, Aya Elbadri, Zhaoyuan Qin, Yuzhi Chen
 * main.c
 *
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include <drivers/pwm.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <math.h>

/* Size for K thread */
#define MY_STACK_SIZE 1000

#define BUTTON_DEBOUNCE_DELAY_MS 100

/* Get PWM bindings info from device tree */
#define PWM_LED0_NODE DT_ALIAS(pwm_led0)
#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL DT_PWMS_CHANNEL(PWM_LED0_NODE)
#define PWM_FLAGS DT_PWMS_FLAGS(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR DT_INVALID_NODE
#define PWM_CHANNEL 0
#define PWM_FLAGS 0
#endif

/* Get LED0  bindings info from device tree */
#define LED0_NODE 	DT_ALIAS(led0)
#define LED0		DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)

/* Define period and duty cycles for PWM */
#define PERIOD 20000/1U
#define PULSE0 20000/1U  
#define PULSE1 16000/1U
#define PULSE2 12000/1U
#define PULSE3 8000/1U
#define PULSE4 4000/1U
#define PULSE5 0

/* Define two timers*/
K_TIMER_DEFINE(sos_timer, NULL, NULL);
K_TIMER_DEFINE(move_timer, NULL, NULL);

struct k_mutex my_mutex;
int k_mutex_init(struct k_mutex * my_mutex);

int move_timer_flag = 0;
int sos_timer_flag = 0;
int brake_flag = 0;

typedef enum _light_state 
{
    bright_zero,
    bright_one,
    bright_two,
    bright_three,
    bright_four,
    brake_light,
    sos_blink
}light_state_t;

typedef enum _veml_state
{
    no_move,
    move
}veml_state_t;

typedef enum _brake_state
{
    no_brake,
    brake
}brake_state_t;

typedef enum _sos_state
{
    no_sos,
    sos
}sos_state_t;

static volatile light_state_t global_state = bright_zero;
static volatile light_state_t state1 = bright_zero; 
static volatile veml_state_t state_move = no_move;
static volatile brake_state_t state_brake = no_brake;
static volatile sos_state_t state_sos = no_sos;

/**** << START >> ********* BLINKY TASK ***************/
void blinky_task()
{
    const struct device *pwm;
    int ret;
    pwm = DEVICE_DT_GET(PWM_CTLR);

    /* Set up LED */
	const struct device *led0;
	led0 = device_get_binding(LED0);
	gpio_pin_configure(led0, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
		
    while (1) 
    {
        k_msleep(100);

        k_mutex_lock(&my_mutex, K_FOREVER); 
        state1 = global_state;  // read current state
        k_mutex_unlock(&my_mutex); 

        if(state1==bright_zero)  
        {   ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PULSE0, PWM_FLAGS);

        }else if(state1==bright_one){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PULSE1, PWM_FLAGS);
           
        }else if(state1==bright_two){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PULSE2, PWM_FLAGS);
            
        }else if(state1==bright_three){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PULSE3, PWM_FLAGS);
           
        }else if(state1==bright_four){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PULSE4, PWM_FLAGS);
            
        }else if(state1==brake_light){ // maximum brightness
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PULSE5, PWM_FLAGS);
            printf("*******************BRAKE*************************\n");
        }else if(state1==sos_blink){
            printf("-----------------state_sos-----------------------\n");
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PULSE0, PWM_FLAGS); 
            gpio_pin_set(led0,LED0_PIN,0); k_msleep(150);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);
            gpio_pin_set(led0,LED0_PIN,0); k_msleep(150);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);
            gpio_pin_set(led0,LED0_PIN,0); k_msleep(150);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);

            gpio_pin_set(led0,LED0_PIN,0); k_msleep(500);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);
            gpio_pin_set(led0,LED0_PIN,0); k_msleep(500);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);
            gpio_pin_set(led0,LED0_PIN,0); k_msleep(500);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);

            gpio_pin_set(led0,LED0_PIN,0); k_msleep(150);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);
            gpio_pin_set(led0,LED0_PIN,0); k_msleep(150);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(200);
            gpio_pin_set(led0,LED0_PIN,0); k_msleep(150);
            gpio_pin_set(led0,LED0_PIN,1); k_msleep(500);
            
        }
    }
}
/**** << END >> ********* BLINKY TASK ***************/


/**** << START >> ********* VEML7700 TASK ***************/
void veml7700_task()
{
    const struct device *veml7700 = device_get_binding("VEML7700");

    if (veml7700 == NULL) 
    {
        printf("No device \"%s\" found; did initialization fail?\n", "VEML7700");
        return -1;
    }

    struct sensor_value lux;

    while (true) 
    {
        if((state_move == move)&&(state_brake == no_brake)&&(state_sos == no_sos))
        {
                sensor_sample_fetch(veml7700);
                sensor_channel_get(veml7700, SENSOR_CHAN_LIGHT, &lux);
                printf("  Lux: %f\n", sensor_value_to_double(&lux));

                if( sensor_value_to_double(&lux) >= 1000 )
                {
                    global_state = bright_zero;
                } 
                else if( (100 <= sensor_value_to_double(&lux)) && (sensor_value_to_double(&lux) < 1000 ))
                {
                    global_state = bright_one;
                }
                else if( (200 <= sensor_value_to_double(&lux)) && (sensor_value_to_double(&lux) < 500 ))
                {
                    global_state = bright_two;
                }
                else if( (50 <= sensor_value_to_double(&lux)) && (sensor_value_to_double(&lux) < 200 ))
                {
                    global_state = bright_three;
                }
                else if( sensor_value_to_double(&lux) < 50)
                {
                    global_state = bright_four;
                } 
                    k_msleep(1000);
        }else{
            printf("  ");
            k_msleep(1000);
        }
    }
}
/**** << END >> ********* VEML7700 TASK ***************/


/* Get SW1  bindings info from device tree */
#define SW1_NODE DT_ALIAS(sw1)
#define SW1 DT_GPIO_LABEL(SW1_NODE, gpios)
#define SW1_PIN DT_GPIO_PIN(SW1_NODE, gpios)
#define SW1_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(SW1_NODE, gpios))

/* Get SW2  bindings info from device tree */
#define SW2_NODE DT_ALIAS(sw2)
#define SW2 DT_GPIO_LABEL(SW2_NODE, gpios)
#define SW2_PIN DT_GPIO_PIN(SW2_NODE, gpios)
#define SW2_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(SW2_NODE, gpios))

/* For button pressed */
static uint32_t time, last_time=0;
static struct gpio_callback button_cb_data_2;


/**** << START >> *** BUTTON DEBOUNCING ***********/
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    time = k_uptime_get_32();

    if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS)
    {
        last_time = time;
        return;
    }
    if(pins == BIT(SW2_PIN))
    {
        k_timer_stop(&sos_timer); //stop timer once the button is pressed
    } 
    last_time = time;
}
/**** << END >> *** BUTTON DEBOUNCING ***********/


/**** << START >> *** USE GYROSCOPE FOR MOVE/FALL DETECTION ***********/
double x_axis, y_axis, z_axis;
int gyro_counter = 0;	
int gyro_i, gyro_j = 0;
double gx_axis_collect[5]; // store the latest 5 values
double gy_axis_collect[5];
double gz_axis_collect[5];

int flag_fall = 0;
int flag_move = 0;

static int process_mpu6050(const struct device *dev)
{
    struct sensor_value temperature;
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) 
    {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	
		x_axis = sensor_value_to_double(&gyro[0]);
		y_axis = sensor_value_to_double(&gyro[1]);
		z_axis = sensor_value_to_double(&gyro[2]);
    
	    flag_fall = 0;
	    flag_move = 0;

        if(gyro_counter < 5) 
            {
                gyro_counter = gyro_counter + 1;
                gx_axis_collect[gyro_j] = x_axis;
                gy_axis_collect[gyro_j] = y_axis;
                gz_axis_collect[gyro_j] = z_axis;
                
                if(gyro_j == 4) {gyro_j = 0;} else {gyro_j = gyro_j+1;}

                printf("%d\n", gyro_counter);
            }else   // the first 5 values has already been written into the array
            {
                for(gyro_i = 0; gyro_i < 5; gyro_i++)
                {
                    /* After comparison, if the difference is over 2, fall action is confirmed
                    Otherwise it is normal movement (0.2, 2) */
                    if( ((fabs(gx_axis_collect[gyro_i] - x_axis) > 2)&&(fabs(gy_axis_collect[gyro_i] - y_axis) > 2))
                    ||  ((fabs(gx_axis_collect[gyro_i] - x_axis) > 2)&&(fabs(gz_axis_collect[gyro_i] - z_axis) > 2))
                    ||  ((fabs(gy_axis_collect[gyro_i] - y_axis) > 2)&&(fabs(gz_axis_collect[gyro_i] - z_axis) > 2)) )
                    { 
                        flag_fall ++;
                    }else if(
                        ((fabs(gx_axis_collect[gyro_i] - x_axis) > 0.2)&&(fabs(gy_axis_collect[gyro_i] - y_axis) > 0.2))
                    ||  ((fabs(gx_axis_collect[gyro_i] - x_axis) > 0.2)&&(fabs(gz_axis_collect[gyro_i] - z_axis) > 0.2))
                    ||  ((fabs(gy_axis_collect[gyro_i] - y_axis) > 0.2)&&(fabs(gz_axis_collect[gyro_i] - z_axis) > 0.2)) )
                    {
                        flag_move ++;
                    }
                }

                gx_axis_collect[gyro_j] = x_axis;
                gy_axis_collect[gyro_j] = y_axis;
                gz_axis_collect[gyro_j] = z_axis;

                if(gyro_j == 4) {gyro_j = 0;} else {gyro_j = gyro_j+1;}
            }
        if(flag_fall > 3)
        {
            printf("      Fall is detected!!!!!!!!!!!\n");
            sos_timer_flag = 1;
            k_timer_start(&sos_timer, K_SECONDS(7), K_SECONDS(1));
        }
        else if(flag_move > 2)
        {
            printf("      The bike is moving .............\n");
            move_timer_flag = 1;
            state_move = move;
            k_timer_start(&move_timer, K_SECONDS(15), K_SECONDS(0)); // restart every movement happens
        }
            printf( "  Temperature : %g Cel\n" 
                    "  gyro  %f %f %f rad/s\n",
            sensor_value_to_double(&temperature),
            sensor_value_to_double(&gyro[0]),
            sensor_value_to_double(&gyro[1]),
            sensor_value_to_double(&gyro[2]));
	} else 
    {
		printf("sample fetch/get failed: %d\n", rc);
	}
	return rc;
    k_msleep(200);
}
/**** << END >> *** USE GYROSCOPE FOR MOVE/FALL DETECTION ***********/


/**** << START >> *** CONFIG_MPU6050_TRIGGER ***********/
#ifdef CONFIG_MPU6050_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu6050_drdy(const struct device *dev,
				struct sensor_trigger *trig)
{
	int rc = process_mpu6050(dev);

	if (rc != 0) {
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}
#endif 
/**** << END >> *** CONFIG_MPU6050_TRIGGER ***********/


/**** << START >> ********* MPU6050 TASK ***************/
void mpu6050_task()
{
    const char *const label = DT_LABEL(DT_INST(0, invensense_mpu6050));
	const struct device *mpu6050 = device_get_binding(label);

	if (!mpu6050) {
		printf("Failed to find sensor %s\n", label);
		return;
	}

#ifdef CONFIG_MPU6050_TRIGGER
	trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(mpu6050, &trigger,
			       handle_mpu6050_drdy) < 0) {
		printf("Cannot configure trigger\n");
		return;
	}
	printk("Configured for triggered sampling.\n");
#endif

	while (!IS_ENABLED(CONFIG_MPU6050_TRIGGER)) {
		int rc = process_mpu6050(mpu6050);

		if (rc != 0) {
			printf("°°°°°°°°°Pass out°°°°°°°°° \n");
			//break;
		}
		k_msleep(1000);
	}
	/* triggered runs with its own thread after exit */
}
/**** << END >> ********* MPU6050 TASK ***************/


/**** << START >> ********* BRAKE TASK ***************/
void brake_task()
{
    /* Set up button1 interrupt */
    const struct device *sw1;
    sw1 = device_get_binding(SW1);
    gpio_pin_configure(sw1, SW1_PIN, GPIO_INPUT | SW1_FLAGS);
    gpio_pin_interrupt_configure(sw1, SW1_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    
    while(1)
    {   
        k_msleep(100);
        if (gpio_pin_get_raw(sw1,SW1_PIN)==0){
            while(1)
            {
                if(gpio_pin_get_raw(sw1,SW1_PIN)==0)
                {
                    global_state = brake_light;
                    state_brake = brake;

                } else
                {
                    state_brake = no_brake;
                    global_state = bright_zero;
                    brake_flag = 0;
                    break;
                }
                k_msleep(100);   
            }
        }
    }   
}
/**** << END >> ********* BRAKE TASK ***************/


/**** << START >> ********* SOS TASK ***************/
void sos_task()
{
    /* Set up button interrupt */
    const struct device *sw2;
    sw2 = device_get_binding(SW2);
    gpio_pin_configure(sw2, SW2_PIN, GPIO_INPUT | SW2_FLAGS);
    gpio_pin_interrupt_configure(sw2, SW2_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data_2, button_pressed, BIT(SW2_PIN));
    gpio_add_callback(sw2, &button_cb_data_2);
    
   while(1)
   {    if (sos_timer_flag == 1)
        { 
            while(1)
            {
                /* check timer status */
                if (k_timer_status_get(&sos_timer) > 0) 
                {
                    /* timer has expired */
                    printf("SOS!!!!!!!!!!!\n");
                    global_state = sos_blink;
                    state_sos = sos;
                } else if (k_timer_remaining_get(&sos_timer) == 0) 
                {
                    /* timer was stopped (by someone else) before expiring */
                    state_sos = no_sos;
                    sos_timer_flag = 0;
                    global_state = bright_zero;
                    
                    break;
                } else 
                {
                    printf("sos timer running \n");
                }
                k_msleep(1000);
            }
        }
        k_msleep(1000);
   }
}
/**** << END >> ********* SOS TASK ***************/


/**** << START >> *********  MOVE TIMER TASK ***************/
void move_timer_task()
{
    while(1)
    {
        if(move_timer_flag == 1)
        {
            while(1)
            {
                    /* check timer status */
                if (k_timer_status_get(&move_timer) > 0) 
                {
                    /* timer has expired */
                    state_move = no_move;
                    global_state = bright_zero;
                    move_timer_flag = 0;
                    break;
                } 
                k_msleep(1000);
             }
        }
        printf("  NO MOVEMENT  ");
        k_msleep(1000);
    }   
}
/**** << >END >> *********  MOVE TIMER TASK ***************/


K_THREAD_DEFINE(move_timer_id, MY_STACK_SIZE, move_timer_task, 
NULL, NULL, NULL, 6, 0, 0);

K_THREAD_DEFINE(sos_id, MY_STACK_SIZE, sos_task,
NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(blinky_id, MY_STACK_SIZE, blinky_task,
NULL, NULL, NULL, 4, 0, 0);

K_THREAD_DEFINE(veml7700_id, MY_STACK_SIZE, veml7700_task,
NULL, NULL, NULL, 3, 0, 0); 

K_THREAD_DEFINE(mpu6050_id, MY_STACK_SIZE, mpu6050_task,
NULL, NULL, NULL, 2, 0, 0); 

K_THREAD_DEFINE(brake_id, MY_STACK_SIZE, brake_task,
NULL, NULL, NULL, 1, 0, 0); 