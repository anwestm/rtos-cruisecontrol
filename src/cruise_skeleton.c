#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include <time.h>

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK Button_Stack[TASK_STACKSIZE]; 
OS_STK Switch_Stack[TASK_STACKSIZE];
OS_STK Watchdog_Stack[TASK_STACKSIZE]; 
OS_STK Overload_Stack[TASK_STACKSIZE];
OS_STK Extra_Stack[TASK_STACKSIZE]; 

// Task Priorities
 
#define STARTTASK_PRIO     5
#define WATCHDOG_PRIO      7
#define OVERLOAD_PRIO	  18
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define SWITCH_PRIO       9
#define BUTTON_PRIO       8
#define EXTRA_PRIO		  15

// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define BUTTON_PERIOD  300
#define SWITCH_PERIOD  300
#define HYPER_PERIOD  300


/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Engine;
OS_EVENT *Mbox_Gear;
OS_EVENT *Mbox_Gas;
OS_EVENT *Mbox_Cruise;
OS_EVENT *Mbox_Overload;
OS_EVENT *Mbox_OverloadInput;

// Semaphores

OS_EVENT *v_sem, *c_sem, *b_sem, *s_sem, *overload_sem, *extra_sem;

// SW-Timer

OS_TMR *v_timer, *c_timer, *b_timer, *s_timer, *overload_timer, *extra_timer;

/*
 * Types
 */
enum active {on = 2, off = 1};


/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */
  
  return delay;
}

static int b2sLUT[] = {0x40, //0
		       0x79, //1
		       0x24, //2
		       0x30, //3
		       0x19, //4
		       0x12, //5
		       0x02, //6
		       0x78, //7
		       0x00, //8
		       0x18, //9
		       0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  
  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
	int tmp = target_vel;
        int out;
        INT8U out_high = 0;
        INT8U out_low = 0;
        INT8U out_sign = 0;

        if (target_vel < 0) {
            out_sign = int2seven(10);
            tmp *= -1;
        } else {
            out_sign = int2seven(0);
        }

        out_high = int2seven(tmp / 10);
        out_low = int2seven(tmp - (tmp/10) * 10);

        out = int2seven(0) << 21 |
              out_sign << 14 |
              out_high << 7  |
              out_low;
		IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{	
	int led_id = 4096; // 2^12
	const int step = 400; // distance/leds
	led_id = led_id << (position/step); //bitshift
	led_red = led_red | led_id;
	
}

//Timer callbacks

void c_timer_callback(){
	OSSemPost(c_sem);
}

void v_timer_callback(){
	OSSemPost(v_sem);
}

void b_timer_callback(){
	OSSemPost(b_sem);
}

void s_timer_callback(){
	OSSemPost(s_sem);
}

void overload_timer_callback(){
	OSSemPost(overload_sem);
}

void extra_timer_callback(){
	OSSemPost(extra_sem);
}
/*
 * The task 'VehicleTask' updates the current velocity of the vehicle
 */

void ExtraTask(void* pdata){
	INT8U err;
	INT16U overload = 0;
	INT16U overload_p;
	void *msg;
	while(1){
		OSSemPend(extra_sem,0,&err);
		
		msg = OSMboxPend(Mbox_OverloadInput, 0, &err);
		overload = *((INT16U*) msg);
		
		overload_p = 2*overload;
		
		printf("EXTRA TASK OVERLOAD: %d %% \n", overload_p > 100 ? 100 : overload_p);

		INT32U time_start = OSTimeGet();

		while(OSTimeGet() - time_start < 3* overload_p) //hyperperiod = 300
		;
		
	}
}

void OverloadDetectionTask(void* pdata){
	INT8U err;
    int ok_signal = 1;
    while(1)
    {
        OSSemPend(overload_sem,0,&err);  
        err = OSMboxPost(Mbox_Overload, (void *) ok_signal);
	}

} 


void WatchdogTask(void* pdata){
	INT8U err;
    void *msg;
    while(1)
    {
        msg = OSMboxPend(Mbox_Overload,300, &err);
        msg = OSMboxAccept(Mbox_Overload); 
        if(err == OS_ERR_TIMEOUT)
			printf("OVERLOAD\n");
  
	}
}


void VehicleTask(void* pdata)
{ 
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT8S retardation;   
  INT16U position = 0; 
  INT16S velocity = 0;  
  enum active brake_pedal = off;
  enum active engine = off;

  printf("Vehicle task created!\n");

  while(1)
    {
      err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

	  OSSemPend(v_sem, 0, &err); // wait for timer

      /* Non-blocking read of mailbox: 
	 - message in mailbox: update throttle
	 - no message:         use old throttle
      */
      msg = OSMboxPend(Mbox_Throttle, 1, &err); 
      if (err == OS_NO_ERR) 
	throttle = (INT8U*) msg;
      msg = OSMboxPend(Mbox_Brake, 1, &err); 
      if (err == OS_NO_ERR) 
	brake_pedal = *((enum active*) msg);
	  
	  msg = OSMboxPend(Mbox_Engine, 1, &err); 
      if (err == OS_NO_ERR) 
	engine = *((enum active*) msg);
	  
      // vehichle cannot effort more than 80 units of throttle
      if (*throttle > 80) *throttle = 80;

      // brakes + wind
      if (brake_pedal == off)
	{
	  acceleration = - wind_factor*velocity; // changed
	  if (engine == on)
		acceleration += *throttle;
	  if (400 <= position && position < 800)
	    acceleration -= 2; // traveling uphill
	  else if (800 <= position && position < 1200)
	    acceleration -= 4; // traveling steep uphill
	  else if (1600 <= position && position < 2000)
	    acceleration += 4; //traveling downhill
	  else if (2000 <= position)
	    acceleration += 2; // traveling steep downhill
	}
      else
	acceleration = -brake_factor*velocity;

      printf("Position: %d m\n", position);
      printf("Velocity: %d m/s\n", velocity);
      printf("Accell: %d m/s2\n", acceleration);
      printf("Throttle: %d V\n", *throttle);

      position = position + velocity * VEHICLE_PERIOD / 1000;
      velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
      if(position > 2400)
	position = 0;

      show_velocity_on_sevenseg((INT8S) velocity);
	  show_position(position);

	  //display and reset leds;
	  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
	  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
	  led_red = 0;
      led_green = 0;
    }
} 
 
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
	enum active gas_pedal = off;
	enum active brake_pedal = off;
	enum active top_gear = off;
	enum active engine = off;
	enum active cruise_control = off; 
    enum active cruise_toggle = off;

	INT16S vel_err = 0;

  void* msg;
  INT16S* current_velocity;
  INT8U target_vel = 0;

  printf("Control Task created!\n");

  while(1)
    {
      msg = OSMboxPend(Mbox_Velocity, 0, &err);
      current_velocity = (INT16S*) msg;

	  msg = OSMboxPend(Mbox_Gas, 0, &err);
      gas_pedal = *((enum active*) msg);

	  msg = OSMboxPend(Mbox_Brake, 0, &err);
      brake_pedal = *((enum active*) msg);

	  msg = OSMboxPend(Mbox_Gear, 0, &err);
      top_gear = *((enum active*) msg);

	  msg = OSMboxPend(Mbox_Engine, 0, &err);
      engine = *((enum active*) msg);

	  msg = OSMboxPend(Mbox_Cruise, 0, &err);
      cruise_control = *((enum active*) msg);

	  throttle = 0;

	  //Cruise_toggle
	  if(cruise_control == on && cruise_toggle == off){
			cruise_toggle = on;
			target_vel = (INT8U) *current_velocity;
	  }
	
	  


		
      if (engine == off && *current_velocity > 0){
		engine = on;
	  }

	  if(top_gear == on && *current_velocity >= 20 && gas_pedal == off && brake_pedal == off && cruise_toggle == on){
			led_green = led_green | LED_GREEN_2; //CruiseControll led
			
			//PI-controller
			vel_err +=  target_vel - *current_velocity;			
			throttle = 3*(target_vel - *current_velocity) + vel_err;	
		}
		else{
			cruise_toggle = off;
			target_vel = 0;
			vel_err = 0;
		}

		if(engine == on){
			led_red = led_red | LED_RED_0;
		}

		if(gas_pedal == on && engine == on){
			led_green = led_green | LED_GREEN_6;
			if(top_gear == on)
				throttle = 80;
			else
				throttle = 40;
		}
	  err = OSMboxPost(Mbox_Throttle, (void *) &throttle);
	  err = OSMboxPost(Mbox_Engine, (void *) &engine);
	  
	  show_target_velocity(target_vel);

	  OSSemPend(c_sem, 0, &err); // wait for timer
    }
}

void SwitchIOTask(void* pdata){
	INT8U err;
	enum active top_gear = off;
	enum active engine = off;
	INT16U overload;
	int switches;
	while(1){
		OSSemPend(s_sem, 0, &err);
		switches = switches_pressed();
		overload = 0;		

		if(switches & ENGINE_FLAG){
			engine = on;
		}
		else{
			engine = off;
		}

		if (switches & TOP_GEAR_FLAG){
			led_red = led_red | LED_RED_1;
			top_gear = on;
		}
		else{
			top_gear = off;
		}

		overload = switches & 0x3F0; //switches pressed
		led_red = led_red | overload;
		overload = overload >> 4;
		
		err = OSMboxPost(Mbox_OverloadInput, (void *) &overload);
		err = OSMboxPost(Mbox_Engine, (void *) &engine);
 		err = OSMboxPost(Mbox_Gear, (void *) &top_gear);
	}
}

void ButtonIOTask(void* pdata){
	INT8U err;
	enum active gas_pedal = off;
	enum active brake_pedal = off;
	enum active cruise_control = off; 
	int buttons;

	while(1){
		OSSemPend(b_sem, 0, &err);	
		buttons = buttons_pressed();
		
		
		if(buttons & CRUISE_CONTROL_FLAG){
			cruise_control = on;
		}
		else{
			cruise_control = off;
		}


		if (buttons & BRAKE_PEDAL_FLAG){
			led_green = led_green | LED_GREEN_4;
			brake_pedal = on;
		}
		else{
			brake_pedal = off;
		}

		if (buttons & GAS_PEDAL_FLAG){
			gas_pedal = on;
		}
		else{
			gas_pedal = off;
		}
		err = OSMboxPost(Mbox_Gas, (void *) &gas_pedal);
 		err = OSMboxPost(Mbox_Brake, (void *) &brake_pedal);
		err = OSMboxPost(Mbox_Cruise, (void *) &cruise_control);
	}
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */
  
  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
		       delay,
		       alarm_handler,
		       context) < 0)
    {
      printf("No system clock available!n");
    }

  /* 
   * Create and start Software Timer 
   */
	
	BOOLEAN timer_on;
	v_sem = OSSemCreate(0);
    c_sem = OSSemCreate(0);
    b_sem = OSSemCreate(0);
    s_sem = OSSemCreate(0);
	overload_sem = OSSemCreate(0);
	extra_sem = OSSemCreate(0);
	
	v_timer = OSTmrCreate(0, VEHICLE_PERIOD/100, OS_TMR_OPT_PERIODIC, v_timer_callback, NULL, NULL, &err); // /100 to get accureate timing

	c_timer = OSTmrCreate(0, CONTROL_PERIOD/100, OS_TMR_OPT_PERIODIC, c_timer_callback, NULL, NULL, &err);

	b_timer = OSTmrCreate(0, BUTTON_PERIOD/100, OS_TMR_OPT_PERIODIC, b_timer_callback, NULL, NULL, &err); // /100 to get accureate timing

	s_timer = OSTmrCreate(0, SWITCH_PERIOD/100, OS_TMR_OPT_PERIODIC, s_timer_callback, NULL, NULL, &err);

overload_timer = OSTmrCreate(0, HYPER_PERIOD/100, OS_TMR_OPT_PERIODIC, overload_timer_callback, NULL, NULL, &err);

extra_timer = OSTmrCreate(0, HYPER_PERIOD/100, OS_TMR_OPT_PERIODIC, extra_timer_callback, NULL, NULL, &err);
	
	timer_on = OSTmrStart(c_timer, &err);
	timer_on = OSTmrStart(v_timer, &err);
	timer_on = OSTmrStart(b_timer, &err);
	timer_on = OSTmrStart(s_timer, &err);
	timer_on = OSTmrStart(overload_timer, &err);
	timer_on = OSTmrStart(extra_timer, &err);
	

  /*
   * Creation of Kernel Objects
   */
  
  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Engine = OSMboxCreate((void*) 0); 
  Mbox_Gas = OSMboxCreate((void*) 0);
  Mbox_Gear = OSMboxCreate((void*) 0);
  Mbox_Cruise = OSMboxCreate((void*) 0);
  Mbox_Overload = OSMboxCreate((void*) 0);
  Mbox_OverloadInput = OSMboxCreate((void*) 0);
   
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
			ControlTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			CONTROLTASK_PRIO,
			CONTROLTASK_PRIO,
			(void *)&ControlTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			VehicleTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			VEHICLETASK_PRIO,
			VEHICLETASK_PRIO,
			(void *)&VehicleTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			SwitchIOTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Switch_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			SWITCH_PRIO,
			SWITCH_PRIO,
			(void *)&Switch_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			ButtonIOTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Button_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			BUTTON_PRIO,
			BUTTON_PRIO,
			(void *)&Button_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

err = OSTaskCreateExt(
			WatchdogTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Watchdog_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			WATCHDOG_PRIO,
			WATCHDOG_PRIO,
			(void *)&Watchdog_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

err = OSTaskCreateExt(
			OverloadDetectionTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Overload_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			OVERLOAD_PRIO,
			OVERLOAD_PRIO,
			(void *)&Overload_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

err = OSTaskCreateExt(
			ExtraTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Extra_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			EXTRA_PRIO,
			EXTRA_PRIO,
			(void *)&Extra_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);


  
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");
 
  OSTaskCreateExt(
		  StartTask, // Pointer to task code
		  NULL,      // Pointer to argument that is
		  // passed to task
		  (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
		  // of task stack 
		  STARTTASK_PRIO,
		  STARTTASK_PRIO,
		  (void *)&StartTask_Stack[0],
		  TASK_STACKSIZE,
		  (void *) 0,  
		  OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	 
  OSStart();
  
  return 0;
}
