/*
             ___.                 
  ____   ____\_ |__  __ __  ____  
_/ __ \_/ __ \| __ \|  |  \/ ___\ 
\  ___/\  ___/| \_\ \  |  / /_/  >
 \___  >\___  >___  /____/\___  / 
     \/     \/    \/     /_____/  

  Written by Jian Yuan Lee for the TI's MSP430 Platform
*/

#include <stdint.h>
#include <msp430.h>

///////////////////
// Configuration //
///////////////////

// PWM constants
uint16_t max_duty_cycle = 60;
uint16_t min_duty_cycle = 0;
uint16_t pid_p_term = max_duty_cycle;
uint16_t pid_i_term = 0; // Divided by 100000
uint16_t pid_d_term = 10;
uint16_t base_duty_cycle = max_duty_cycle;


/////////////////
// Port 1 Bits //
/////////////////
#define LEFT_MOTOR BIT7
#define RIGHT_MOTOR BIT6
#define LEFT_SENSOR BIT2
#define RIGHT_SENSOR BIT3
#define BUTTON BIT4
#define LEFT_LED BIT0
#define RIGHT_LED BIT1
#define IR_SENSORS BIT5

// ADC channels for sensors
#define LEFT_SENSOR_CH INCH_2
#define RIGHT_SENSOR_CH INCH_3

/////////////
// Sensors //
/////////////

#define CALIBRATE_SENSOR_READINGS 5

// Sensor min and max values
int16_t left_sensor_max = 930;
int16_t left_sensor_min = 599;
int16_t right_sensor_max = 789;
int16_t right_sensor_min = 110;

// Function prototypes
uint16_t analog_read(bool);
uint16_t calibrate_sensor(bool);
float left_sensor_error = 0;
float right_sensor_error = 0;
void refresh_sensor_errors();
inline float get_line_position();
inline bool is_center_of_line();
inline bool is_intersection_of_line();
inline bool has_lost_line();

///////////////////////
// 7 Segment Display //
///////////////////////

// Port 2 is used entirely by the display

/*
 *      A
 *     ---
 * F  | G |  B
 *     ---
 * E  |   |  C
 *     ---
 *      D
 */
#define SEGA BIT5
#define SEGB BIT6
#define SEGC BIT0
#define SEGD BIT1
#define SEGE BIT2
#define SEGF BIT4
#define SEGG BIT3

// Digit Map
const uint8_t digit_map[] = {
	SEGA + SEGB + SEGC + SEGD + SEGE + SEGF,			// 0
	SEGB + SEGC,										// 1
	SEGA + SEGB + SEGD + SEGE + SEGG,					// 2
	SEGA + SEGB + SEGC + SEGD + SEGG,					// 3
	SEGB + SEGC + SEGF + SEGG,							// 4
	SEGA + SEGC + SEGD + SEGF + SEGG,					// 5
	SEGA + SEGC + SEGD + SEGE + SEGF + SEGG,			// 6
	SEGA + SEGB + SEGC,									// 7
	SEGA + SEGB + SEGC + SEGD + SEGE + SEGF + SEGG,		// 8
	SEGA + SEGB + SEGC + SEGD + SEGF + SEGG,			// 9
};

// Characters
#define CHAR_S digit_map[5]
#define CHAR_T digit_map[7]
#define CHAR_O digit_map[0]
#define CHAR_P (SEGA + SEGB + SEGE + SEGF + SEGG)
#define CHAR_C (SEGA + SEGD + SEGE + SEGF)
#define CHAR_A (SEGA + SEGB + SEGC + SEGE + SEGF + SEGG)
#define CHAR_L (SEGD + SEGE + SEGF)
#define CHAR_K (SEGB + SEGD + SEGE + SEGF + SEGG)

// Function prototypes
inline void segment_print(uint8_t);
inline void segment_print_digit(uint8_t);
inline void segment_off();

////////////////
// Bug States //
////////////////

enum modes { GO, STOPPING, IDLE, COUNTDOWN, CALIBRATION };

// Set default mode to IDLE
volatile modes current_mode = IDLE;

/////////
// PWM //
/////////

// PWM variables
volatile uint16_t pwm_ticks = 0;
volatile uint16_t left_motor_duty_cycle = 0;
volatile uint16_t right_motor_duty_cycle = 0;

/////////
// PID //
/////////

// PID variables
float current_line_position = 0.0;
float previous_line_position = 0.0;
float proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
int16_t power;
int16_t tmp_left_duty_cycle, tmp_right_duty_cycle;

//////////////////////////////////////
// Timekeeping and Live Map Display //
//////////////////////////////////////

// Total number of ticks
volatile uint32_t ticks = 0;

// Debounce button
volatile uint32_t debounce_tick = 0;

// Previous tick (ie. starting time)
volatile uint32_t previous_tick = 0;

// Number of ticks per segment
volatile uint32_t ticks_per_segment;

// Number of half laps taken
volatile uint32_t laps = 0;

// Is bug on the left side of the map?
bool map_left_side = false;

// Function prototypes
void delay(uint32_t);
uint8_t map_segment(uint8_t);
void refresh_map();
void update_lap_counter();

/*
 * Utility Function Prototypes
 */
inline int16_t map(int16_t, int16_t, int16_t, int16_t, int16_t);

/*
 * Sets the configuration bits needed for microcontroller
 */
inline void setup_config_bits()
{
	// Disable watchdog timer
	WDTCTL = WDTPW + WDTHOLD;
	
	// Set all P1 outputs to be low
	P1OUT = IR_SENSORS;
	
	// Set left and right motor snd left and right LEDs to be output
	P1DIR = LEFT_MOTOR + RIGHT_MOTOR + LEFT_LED + RIGHT_LED + IR_SENSORS;
	
	// Enable interrupt for external button
	P1IE = BUTTON;

	// Select edge for interrupt
	// 0 - Rising edge
	// 1 - Falling edge
	P1IES = 1;
	
	// Clear all P1 flag interrupt
	P1IFG = 0;

	// Port 2 is used entirely by the seven segment display
	P2SEL = 0;
	P2OUT = 0;
	P2DIR = 0x7F;
	P2IES = 0;
	P2IFG = 0;
	
	// Setup basic clock system
	BCSCTL2 = SELM_0 + DIVM_0 + DIVS_0;
	/*
    if (CALBC1_1MHZ != 0xFF) {
        /* Follow recommended flow. First, clear all DCOx and MODx bits. Then
         * apply new RSELx values. Finally, apply new DCOx and MODx bit values.
         * /
        DCOCTL = 0x00;
        BCSCTL1 = CALBC1_1MHZ;      /* Set DCO to 1MHz * /
        DCOCTL = CALDCO_1MHZ;
    }
    */
    if (CALBC1_8MHZ != 0xFF) {
        /* Follow recommended flow. First, clear all DCOx and MODx bits. Then
         * apply new RSELx values. Finally, apply new DCOx and MODx bit values.
         */
        DCOCTL = 0x00;
        BCSCTL1 = CALBC1_8MHZ;      /* Set DCO to 1MHz */
        DCOCTL = CALDCO_8MHZ;
    }
    
    BCSCTL1 |= XT2OFF + DIVA_0;
    BCSCTL3 = XT2S_0 + LFXT1S_2 + XCAP_1;
    
    // Set up timer for PWM
	TACCTL0 = CCIE;
	//TACCR0 = 99;
	TACCR0 = 800-1;//1600-1;
	TACTL = TASSEL_2 + MC_1;
	
	// ADC10 Config
	ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON;
	ADC10CTL1 = SHS_0 + ADC10DIV_0 + ADC10SSEL_0 + CONSEQ_0;
	ADC10AE0 = LEFT_SENSOR + RIGHT_SENSOR;
	
	// Enable Interrupts (ie. For our one and only external button)
	__enable_interrupt();
}

int main()
{
	setup_config_bits();
	
	while (true)
	{
		switch (current_mode)
		{
			/*
			 * Countdown mode
			 */
			case COUNTDOWN:
			
				for (int i = 8; i > 0; i--)
				{
					if (current_mode != COUNTDOWN)
					{
						// Countdown has been stopped by interrupt
						break;
					}
					
					segment_print_digit(i);
					
					// Delay for 1000 milliseconds = 1 second
					delay(1000);
				}
				
				segment_off();
				
				if (current_mode == COUNTDOWN)
				{
					current_mode = GO;
				}
				
			break;
			
			/*
			 * Go mode
			 * - Bug starts to follow the line
			 */
			case GO:
				
				// Refresh sensor errors
				refresh_sensor_errors();
				
				// Get the error
				current_line_position = get_line_position();
				
				// Proportional
				proportional = current_line_position;
				
				// Derivative
				derivative = current_line_position - previous_line_position;
				
				// Integral
				integral += proportional;
				
				// Save current line position
				previous_line_position = current_line_position;
				
				// Compute power
				power = (proportional * pid_p_term) + (integral * pid_i_term / 100000.0) + (derivative * pid_d_term);
				
				tmp_left_duty_cycle = base_duty_cycle + power;
				tmp_right_duty_cycle = base_duty_cycle - power;
				
				if (tmp_left_duty_cycle > max_duty_cycle)
				{
					tmp_left_duty_cycle = max_duty_cycle;
				}
				else if (tmp_left_duty_cycle < min_duty_cycle)
				{
					tmp_left_duty_cycle = min_duty_cycle;
				}
				
				if (tmp_right_duty_cycle > max_duty_cycle)
				{
					tmp_right_duty_cycle = max_duty_cycle;
				}
				else if (tmp_right_duty_cycle < min_duty_cycle)
				{
					tmp_right_duty_cycle = min_duty_cycle;
				}
				
				left_motor_duty_cycle = tmp_left_duty_cycle;
				right_motor_duty_cycle = tmp_right_duty_cycle;
				
				if (current_line_position > 0)
				{
					P1OUT |= RIGHT_LED;
					P1OUT &= ~LEFT_LED;
				}
				else if (current_line_position < 0)
				{
					P1OUT |= LEFT_LED;
					P1OUT &= ~RIGHT_LED;
				}
				else
				{
					P1OUT |= LEFT_LED + RIGHT_LED;
				}
				
				//if (has_lost_line())
				//{
				//	P1OUT |= LEFT_LED;
				//}
				
				if (is_intersection_of_line())
				{
					update_lap_counter();
				}
				
				refresh_map();
				
			break;
			
			/*
			 * Stopping mode
			 * - Stops the motor
			 * - Changes mode to IDLE
			 */
			case STOPPING:
			
				// Turn off motor
				left_motor_duty_cycle = 0;
				right_motor_duty_cycle = 0;
				
				// Turn off indicator LEDs
				P1OUT &= ~(LEFT_LED + RIGHT_LED + LEFT_MOTOR + RIGHT_MOTOR);
				
				// Clear PID memory
				proportional = 0;
				integral = 0;
				derivative = 0;
				
				// Clear map memory
				ticks = 0;
				previous_tick = 0;
				ticks_per_segment = 0;
				laps = 0;
				map_left_side = false;

				// Turn off segment
				segment_off();
				
				current_mode = IDLE;
				
			break;
			
			/*
			 * Calibration mode
			 */
			case CALIBRATION:
			
				// Display "CAL" on display
				segment_print(CHAR_C);
				delay(150);
				segment_print(CHAR_A);
				delay(150);
				segment_print(CHAR_L);
				delay(150);
				segment_off();
				
				delay(500);
				
				left_sensor_min = calibrate_sensor(false);
				right_sensor_min = calibrate_sensor(true);
				
				left_motor_duty_cycle = max_duty_cycle;
				right_motor_duty_cycle = max_duty_cycle;
				
				delay(300);
				
				left_motor_duty_cycle = 0;
				right_motor_duty_cycle = 0;
				
				delay(500);
				
				left_sensor_max = calibrate_sensor(false);
				right_sensor_max = calibrate_sensor(true);
				
				// Display "OK" on display
				segment_print(CHAR_O);
				delay(150);
				segment_print(CHAR_K);
				delay(150);
				segment_off();
				
				current_mode = IDLE;
			
			break;
			
			/*
			 * Idle mode
			 * - Do nothing
			 */
			case IDLE:
			
				 // NOP
				
			break;
		}
	}
}

/*
 * Analog to Digital Conversion
 * First parameter tells which channel to perform ADC on:
 * - False: Left Channel
 * - True: Right Channel
 */
uint16_t analog_read(bool channel)
{
	// Disable conversion
	ADC10CTL0 &= ~ENC;
	
	if (channel == false)
	// Left channel
	{
		ADC10CTL1 &= ~RIGHT_SENSOR_CH;
		ADC10CTL1 |= LEFT_SENSOR_CH;
	}
	else
	// Right channel
	{
		ADC10CTL1 &= ~LEFT_SENSOR_CH;
		ADC10CTL1 |= RIGHT_SENSOR_CH;
	}
	
	// Enable conversion + Start conversion
	ADC10CTL0 |= ENC + ADC10SC;
	
	// Wait until conversion is complete
	while (ADC10CTL1 & ADC10BUSY);
	
	// Return the conversion result
	return ADC10MEM;
}

/*
 * Refresh sensor errors
 * Maps error from 0.00 to 1.00
 */
void refresh_sensor_errors()
{
	uint16_t left_sensor_reading = analog_read(false);
	uint16_t right_sensor_reading = analog_read(true);
	
	left_sensor_error = map(left_sensor_reading, left_sensor_min, left_sensor_max, 0, 10) / 10.0;
	right_sensor_error = map(right_sensor_reading, right_sensor_min, right_sensor_max, 0, 10) / 10.0;
	
	if (left_sensor_error < 0)
	{
		left_sensor_error = 0;
	}
	else if (left_sensor_error > 1)
	{
		left_sensor_error = 1;
	}
	
	if (right_sensor_error < 0)
	{
		right_sensor_error = 0;
	}
	else if (right_sensor_error > 1)
	{
		right_sensor_error = 1;
	}
}

/*
 * Get line position
 * -1.00 indicates line is on the left
 *  0.00 indicates line is in the middle
 *  1.00 indicates line is on the right
 */
inline float get_line_position()
{
	/*
	if (has_lost_line())
	{
		return -1;
	}
	*/
	
	return left_sensor_error - right_sensor_error;
}

/*
 * Is the bug at the center of the line?
 */
inline bool is_center_of_line()
{
	return (get_line_position() == 0.0);
}

/*
 * Is bug at the intersection of the line?
 */
inline bool is_intersection_of_line()
{
	return ((left_sensor_error < 0.1) && (right_sensor_error < 0.1));
}

/*
 * Has the bug lost the line?
 */
inline bool has_lost_line()
{
	return ((left_sensor_error > 0.8) && (right_sensor_error > 0.8));
}

/*
 * Calibrate sensor function for debugging
 */
/*
void calibrate_sensor()
{
	//uint16_t tmp_left_sensor = 0, tmp_right_sensor = 0;
	uint16_t left_sum = 0, right_sum = 0;
	
	for (int i = 0; i < 5; i++)
	{
		left_sum += analog_read(0);
		__delay_cycles(50);
		right_sum += analog_read(1);
		__delay_cycles(50);
	}
	
	*left_sensor = left_sum / 5;
	*right_sensor = right_sum / 5;
	//tmp_left_sensor = left_sum / 5;
	//tmp_right_sensor = right_sum / 5;
	//__delay_cycles(1000);
}
*/

uint16_t calibrate_sensor(bool channel)
{
	uint16_t sum = 0;
	for (int i = 0; i < CALIBRATE_SENSOR_READINGS; i++)
	{
		sum += analog_read(channel);
		delay(100);
	}
	return sum / CALIBRATE_SENSOR_READINGS;
}

/*
 * Output bits on 7 segment display
 */
inline void segment_print(uint8_t character_map)
{
	P2OUT = character_map;
}

/*
 * Print a digit
 */
inline void segment_print_digit(uint8_t digit)
{
	segment_print(digit_map[digit]);
}

/*
 * Turn off display
 */
inline void segment_off()
{
	P2OUT = 0;
}

/*
 * Delay execution in milliseconds
 */
void delay(uint32_t milliseconds)
{
	unsigned long wake_time = ticks + (milliseconds * 10);
	while (ticks < wake_time);
}

/*
 * Translates position into segments
 */
uint8_t map_segment(uint8_t position)
{
	if (map_left_side)
	{
		switch (position)
		{
			case 0: return SEGG;
			case 1: return SEGG + SEGE;
			case 2: return SEGE;
			case 3: return SEGE + SEGD;
			case 4: return SEGD;
			case 5: return SEGD + SEGC;
			case 6: return SEGC;
			case 7: return SEGC + SEGG;
		}
	}
	else
	{
		switch (position)
		{
			case 0: return SEGG;
			case 1: return SEGG + SEGF;
			case 2: return SEGF;
			case 3: return SEGF + SEGA;
			case 4: return SEGA;
			case 5: return SEGA + SEGB;
			case 6: return SEGB;
			case 7: return SEGB + SEGG;
		}
	}
	
	// We're not suppose to be here
	return 0;
}

/*
 * Refresh map display
 */
void refresh_map()
{
	if (laps > 0)
	{
		// Calculate the position
		uint8_t position = (ticks - previous_tick) / ticks_per_segment;
		
		// Ensure position is between 0 to 7
		if (position > 7)
		{
			position = 7;
		}
		
		// Print map segment on 7 segment display
		segment_print(map_segment(position));
	}
}

/*
 * Update half lap counter
 */
void update_lap_counter()
{
	// Allow 1 second for the bug to pass through the intersection
	if ((ticks - previous_tick) > 10000)
	{
		if (laps > 0)
		{
			ticks_per_segment = (ticks - previous_tick) / 8;
		}
		
		laps++;
		previous_tick = ticks;
		map_left_side = ! map_left_side;
	}
	// segment_print_digit((laps % 10));
}

/*
 * PORT1 interrupt service routine
 * - this has been configured to trigger when the edge of the t RISES
 */
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR()
{
	uint32_t tick_difference = ticks - debounce_tick;
	
	// 500ms debounce
	if (tick_difference >= 5000)
	{
		switch (current_mode)
		{
			case COUNTDOWN:
				segment_off();
				
				// Within 1s of count down
				if (tick_difference <= 10000)
				{
					current_mode = CALIBRATION;
					break;
				}
				
				// No break here so that STOP can be displayed
			
			case GO:
			
				P1OUT &= ~(LEFT_MOTOR + RIGHT_MOTOR);
				
				// Display "STOP" on display
				segment_print(CHAR_S);
				__delay_cycles(1200000);
				segment_print(CHAR_T);
				__delay_cycles(1200000);
				segment_print(CHAR_O);
				__delay_cycles(1200000);
				segment_print(CHAR_P);
				__delay_cycles(1200000);
				segment_off();
				
				__delay_cycles(2000000);
				
				current_mode = STOPPING;
			break;
			
			case IDLE:
				current_mode = COUNTDOWN;
			break;
			
			case STOPPING:
			default:
			
				// NOP
				
			break;
		}
		
		debounce_tick = ticks;
	}
	
	// Clear PORT 1 BUTTON Flag because we're done handling with it
	P1IFG &= ~BUTTON;
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMERA0_ISR()
{
	/*
	 * Ticks for timekeeping
	 */
	ticks++;
	
	/*
	 * Software PWM
	 */
	pwm_ticks++;
	
	if (pwm_ticks >= 100 && current_mode != IDLE && current_mode != STOPPING)
	{
		pwm_ticks = 0;
		P1OUT |= LEFT_MOTOR + RIGHT_MOTOR;
	}
	
	if (pwm_ticks == left_motor_duty_cycle)
	{
		P1OUT &= ~LEFT_MOTOR;
	}
	
	if (pwm_ticks == right_motor_duty_cycle)
	{
		P1OUT &= ~RIGHT_MOTOR;
	}
}


/*
 * Utility: Map
 */
inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}