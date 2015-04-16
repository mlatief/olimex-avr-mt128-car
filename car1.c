#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include <util/delay.h>

#include "lcd.h"
#include "lcd2.h"
#include "utils.h"
#include "Bxxxxx.h"

#define SAMPLE_RATE 8000
// Loop music
#include "jingle.h"

// Crash sound
#include "crash18605.h"

void init(void);
void get_buttons(void);

void setup_lcd(void);
void reset_state(void);
void show_intro(void);
void wait_car_jump(void);
void game_loop(void);
void show_game_over(void);

void enable_sound(void);
void disable_sound(void);
void play_loop(void);
void play_crash(void);

void draw_road(void);

const int ROADLEN = 15;          // Maximum length of the road, used to circularly iterate over the road map
const int MAXSTEPDURATION = 600; // Start slowly (300ms between steps), each step is 1 millisec shorter.
const int MINSTEPDURATION = 150; // This is as fast as it gets
const int CARJUMPSTEPS = 4;      // MOD of number of steps a car jump stays 

const char RIGHTLANE = 1;       // B01
const char LEFTLANE = 2;        // B10
const char BOTHLANES = 3;       // B11

const unsigned int SCORE_ADDR = 0x0004;   // Arbitary address in EEPROM to store High Score between power cycles

// 1: Left lane obstacle, 2: Right lane, 3: Both lanes! extra 12 to make obstacles propability 1/5
const int MAXOBSTACLES = 5 * 3;      

const int NGLYPHS = 5;
const char BLANK = 32;

const int GLYPH_CAR = 1;
const int GLYPH_CAR_JUMP = 2;
const int GLYPH_OBSTACLE = 3;
const int GLYPH_CRASH = 4;
const int GLYPH_HSCORE = 5;

char glyphs[5][8] = {
	// 1: car normal
   {B00111,
	B00011,
	B01110,
	B01110,
	B01110,
	B01110,
	B00011,
	B00111}
	// 2: car jump!
  ,{B11100,
	B11100,
	B11000,
	B11000,
	B11000,
	B11000,
	B11100,
	B11100}
	// 3: obstacle
  ,{B10000,
	B11000,
	B11100,
	B11000,
	B11000,
	B11100,
	B11000,
	B10000}
	// 4: crash
  ,{B10001,
	B11011,
	B01110,
	B01010,
	B01010,
	B01110,
	B11011,
	B10001}
    // 5: New High Score
  ,{B00100,
	B01110,
	B11111,
	B01110,
	B01010,
	B11011,
	B11011,
	B10001}
};


/* State Variables*/
char car_pos;       // 1: Right lane, 2: Left lane
int car_jump;       // 0: On the ground, 1~3: Jumping and rotates back to 0 then stops
int crash;          // Indicates an unfortunate car crash!
long score;         // Accumlate the score ( time passed without crash in msec )

int step_duration;  // Moving speed, speed should increase with 1msec each loop
char road[15];      // Rotating array of road with random obstacles placed
int road_index;     // Current index in the road
char line_buffer[17];

volatile uint16_t sample;
#define SOUND_MUSIC  1
#define SOUND_CRASH  2

int sound_loop;
int sound_type;     // Only Music and Crash sounds are currently supported!

ISR(TIMER1_COMPA_vect) {

    int loop_length = 0;

    // Determine maximum samples based on sound_type
    if(sound_type == SOUND_MUSIC)
        loop_length = sound_music_length;
    else if (sound_type == SOUND_CRASH)
        loop_length = sound_crash_length;
    
    // Check if at the end of the sound
    if(sample>=loop_length)
    {
        // If loop flag set, repeat
        if(sound_loop)
            sample=0;
        else
            disable_sound(); // Otherwise, stop the timers
    }

    // Play different sound according to sound_type
    if(sound_type == SOUND_MUSIC)
    {
        OCR3BL = pgm_read_byte(&sound_music_data[sample]);
        OCR3CL = pgm_read_byte(&sound_music_data[sample]);
    }
    else if(sound_type == SOUND_CRASH)
    {
        OCR3BL = pgm_read_byte(&sound_crash_data[sample]);
        OCR3CL = pgm_read_byte(&sound_crash_data[sample]);
    }
    else
    {
        OCR3BL = 0x00;
        OCR3CL = 0xFF;
    }
    
    // Next sample
    ++sample;
}

/* Timer2 Interrupt Handler for more responsive Inputs */
ISR(TIMER2_COMP_vect) {

    get_buttons();
}

/*
   - Prepare the Timers and interrupts
   - Sound is generated using two Timers 1 and 3 
   - Timer1 works as sampler at 8kHz
   - Timer3 works as 8-bit output (using PWM)
   - Both pins OC3B & OC3C are connected to the Buzzer
   - OC3B/OC3C are used for non-inverting / inverting PWM output!
*/
void enable_sound(void)
{
    
    /* Set buzzer pins as output*/
    DDRE  |=  (1 << PE4) | (1 << PE5);
    
    /* Timer1 used for sampling sound on 8kHz*/
    // Set Timer1 to CTC mode
    // WGM13:0 = b0100
    TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
    TCCR1B |=  (1 << WGM12);
    TCCR1B &= ~(1 << WGM13);
    // Set Timer1 no prescaler 16 000 000 Hz CS12:0 = 001
    TCCR1B |=  (1 << CS10);
    TCCR1B &= ~((1 << CS12) | (1 << CS11));
    // Set OCR1A register value to F_CPU / SAMPLE_RATE 16e6 / 8000 (corresponds to 8kHz)
    OCR1A = F_CPU/SAMPLE_RATE;
    // Enable per-sample interrupt for Timer1
    TIMSK |= (1 << OCIE1A);
        
    /* Timer3 is used to generate sound using Fast PWM output */
    // Set Timer3 to Fast PWM 8-bit mode
    // WGM33:0 = b0101
    TCCR3A |=  (1<<WGM30);
    TCCR3A &= ~(1<<WGM31);
    TCCR3B |=  (1<<WGM32);
    TCCR3B &= ~(1<<WGM33);
    // Do non-inverting PWM on OC3B
    // COM3B1:0 = b10
    TCCR3A &= ~(1<<COM3B0);
    TCCR3A |= (1<<COM3B1);
    // And inverting on the other PIN
    // COM3C1:0 = b11
    TCCR3A |= (1<<COM3C1) | (1<<COM3C0);
    // No prescalar for Timer3
    // CS32:0 = b001
    TCCR3B |=  (1<<CS30); 
    TCCR3B &= ~((1<<CS32) |(1<<CS31)); 
    // Set initial pulse width value
    OCR3B = 0x00; // 128 
    OCR3C = 0x00; // 128 
    // No need for interrupts, the Timer3 is used merely to generate modulated PWM signal

    // Set the other pin of the buzzer to 0 always!
    //PORTE &= ~(1 << PE5);
    //PORTE |= (1 << PE5);
    
    sample = 0;
    //current_sound = 0;
}

/*
    Stop Timers and disable Timer1 interrupt and mute the buzzer!
*/
void disable_sound(void)
{
    // Disable per-sample interrupt for Timer1
    TIMSK &= ~(1 << OCIE1A);

    // Disable the per-sample timer completely.
    TCCR1B &= ~(1<<CS10);

    // Disable the PWM timer.
    TCCR3B &= ~(1<<CS10);
    
    // Write low to the output pins OC3B, OC3C
    PORTE &= ~(1 << PE4);  
    PORTE &= ~(1 << PE5);
    sample = 0;
}

// Play loop music
void play_loop(void)
{
    disable_sound();
    sound_loop = 1;
    sound_type = SOUND_MUSIC;
    enable_sound();
}

// Play crash sound
void play_crash(void)
{
    disable_sound();
    sound_loop = 0;
    sound_type = SOUND_CRASH;
    enable_sound();
}

void init(void) {

   		/* Clear interrupts */
		cli();

        /* Timer2 used for reading user inputs in responsive way*/
		// Set Timer2 to CTC mode, and normal port operation
        // WGM21:0 = b10 and COM21:0 = 00
        TCCR2 &= ~( (1 << WGM20) | (1 << COM21) | (1 << COM20));
        TCCR2 |=  (1 << WGM21);
        // Set Timer2 prescaler (16 000 000 / 1024) = 15625 Hz
        TCCR2 |= (1 << CS22) | (1 << CS20);
        TCCR2 &= ~(1<< CS21);
        // Set OCR2 register value to 0x003e (62 ~ 15625/250) (corresponds to ~250hz)
        OCR2 = 0x3e; //62
		// Enable Output Compare Match Interrupt for Timer 2
        TIMSK |= (1 << OCIE2);
        
		/* näppäin pinnit sisääntuloksi */
		DDRA &= ~(1 << PA0);
		DDRA &= ~(1 << PA2);
		DDRA &= ~(1 << PA4);

		/* rele/led pinni ulostuloksi */
		DDRA |= (1 << PA6);

		/* lcd-näytön alustaminen */
		lcd_init();
        
}


int main(void) 
{
    init();
    sei();
    
    setup_lcd();
    
    // Basic state machine (0) Intro -> (1) Wait to Start -> (2) Game Loop -> (3) Game Over
    //                                   ^_____________________________________|
    // (0)
    show_intro();
    
    // Stepping through the states
    while(1)
    {
        // (1)
        wait_car_jump();
        
        reset_state();
        play_loop();
        
        // (2)
        game_loop();

        play_crash();
        
        // (3)
        show_game_over();
    }
}

// Indicates if Jump button is pressed but not yet released
int jump_down = 0;

/* Read user inputs: Right, Left and Jump! */
void get_buttons(void)
{
    // Button 1 moves the car to Right Lane
    if(!(PINA & 0b00000001))
    {
        car_pos = RIGHTLANE;
    }
    // Mutually exclusive checking, so only one button takes precedency
    else if(!(PINA & 0b00010000))
    {
        // Button 5 moves the car to Left Lane
        car_pos = LEFTLANE;
    }
    
    // Jump when button released after it was pressed, to avoid Flying Car!
    if(!(PINA & 0b00000100))
    {
        jump_down = 1;
    }
    else
    {
        if(jump_down){
            car_jump = 1;
            jump_down = 0;
        }
    }
}

// Mainly to setup LCD characters
void setup_lcd(void)
{
    for (int i = 0; i < NGLYPHS; i++) {
        lcd_create_glyph(i + 1, glyphs[i]);	// create glyphs
    }
    
    lcd_write_ctrl(LCD_ON);
    lcd_write_ctrl(LCD_CLEAR);
    
}

// Reset game state and clear previous display
void reset_state(void)
{
    // Initialize seed to Timer2 which roughly works as global time
    int seed = TCNT2;
    srand(seed);
    
    car_pos = RIGHTLANE; // Initially in the right lane
    step_duration = MAXSTEPDURATION; // Start from the slowest step speed ( maximum duration )
    
    // Brand new obstacle free road ! 
    for(int i=0; i<ROADLEN;i++)
    {
        road[i] = 0;
    }
    
    line_buffer[ROADLEN+1] = '\0';
    car_jump = 0;
    crash = 0;
    
    road_index = 0;
    score = 0 ;
    
    lcd_write_ctrl(LCD_CLEAR);
}

// Print Splash Screen!
void show_intro(void)
{
    char welcome_1[] = "Avoid  Obstacles";
    char welcome_2[] = ">Jump to Start!<";
    
    lcd_print(welcome_1);
    lcd_set_cursor(0,1);
    lcd_print(welcome_2);
    //delay_msec(2000);
    
    //lcd_write_ctrl(LCD_CLEAR);
}

// Wait until proper Jump button press/release
void wait_car_jump(void)
{
    while (1) 
    {
        delay_msec(50);
        if(car_jump) {
            car_jump = 0;
            return;
        }
    }    
}

/* Main game loop generating obstacles, drawing the road and checking for crash*/
void game_loop(void)
{
    /* ikuinen silmukka */
    while (1) {
        
        /*if(!crash){
        }*/
        // Check if car_pos is the same as an Obstacle ( or two lanes obstacles )
        if(car_jump==0)
            crash = (car_pos == road[road_index]) || (road[road_index] == BOTHLANES) ;

        if(crash){
            return;
        }
        else
        {
            // Put new random obstacle, but first make sure obstacles are avoidable
            // So, check if previous position had an obstacle if so don't put new obstacles
            int prev_obstacle;
            if(road_index==0)
                prev_obstacle = road[ROADLEN-1];
            else
                prev_obstacle = road[(road_index-1)%ROADLEN];
            
            // By default there is no obstacle!
            road[road_index] = 0;

            if(prev_obstacle==0){
                int new_obstacle = rrand(MAXOBSTACLES) + 1; // Get random pattern of obstacles (1,2,3 or otherwise)
                if(new_obstacle<=3){
                    road[road_index] = new_obstacle; // Set the pattern of the new obstacle
                }
            }
            
            road_index = (road_index + 1) % ROADLEN;            
            draw_road();
            
            if(car_jump)
            {
                car_jump = (car_jump+1)%CARJUMPSTEPS;
            }
            
            delay_msec(step_duration);
            if (step_duration > MINSTEPDURATION) {
                step_duration--; 
            }
            score += step_duration;

        }
    }
}


// Draw crash glyph and print Scores
void show_game_over(void)
{
    char game_over_1[] = " Score: nnn  "; // indices: 7,8,9 to write the score digits, 11 for HScore Symbol
    char game_over_2[] = " Highest: nnn"; // indices: 9,10,11 to write the high score digits
    
    long hscore = eeprom_read_dword((uint32_t*)SCORE_ADDR);
    if(hscore < score)
    {
        eeprom_write_dword((uint32_t*)SCORE_ADDR, score);
        game_over_1[12] = GLYPH_HSCORE;
    }
    
    long score_sec = score/1000; // Score in seconds
    long hscore_sec = hscore/1000; // High Score in seconds

    game_over_1[10] = score_sec%10 + '0';  score_sec /= 10;
    game_over_1[9] = score_sec%10 + '0';  score_sec /= 10;
    game_over_1[8] = score_sec%10 + '0';

    game_over_2[12] = hscore_sec%10 + '0';  hscore_sec /= 10;
    game_over_2[11] = hscore_sec%10 + '0';  hscore_sec /= 10;
    game_over_2[10] = hscore_sec%10 + '0';
    
    road_index = (road_index + 1) % ROADLEN;
    draw_road();
 
	lcd_set_cursor(1, 0);
    lcd_print(game_over_1);
	lcd_set_cursor(1, 1);
    lcd_print(game_over_2);
}

// Draw the scrolling road and render new obstacles as well as the score, car, and crash symbol!
void draw_road(void)
{
    // Score is displayed on two lines
    char score_lines[2];
    
    // Get the Higher and Lower order bytes of the score
    long score_sec = score/1000;
    score_lines[1] = score_sec%10;
    score_lines[0] = (score_sec/10)%10;
    
    //Loop through the lanes
    for (int i = 1; i <= 2; i++) {
        // Print the score byte;
        line_buffer[0] = score_lines[i-1] + '0';

        // Print the car, car jump or the crash symbol, or BLANK if nothing in this lane;
        if(car_pos == i)
        {
            if (crash)
            {
                line_buffer[ROADLEN] = GLYPH_CRASH;
            }
            else
            {
                if(car_jump)
                {
                    line_buffer[ROADLEN] = GLYPH_CAR_JUMP;
                }
                else
                {
                    line_buffer[ROADLEN] = GLYPH_CAR;
                }
            }
        }
        else
        {
            line_buffer[ROADLEN] = BLANK;
        }


        // Print the obstacles
		for (int j = 0; j < ROADLEN - 1; j++) {
			int obs = road[(j + road_index)%ROADLEN];
			
            // Here we do bitwise AND between obstacle pattern and lane
            // Use the fact that obstacle patterns are: 1 (B01) right lane, 2 (B10) left lane and 3 (B11) both lanes
            line_buffer[ROADLEN - j - 1] = (obs & i) ? GLYPH_OBSTACLE : BLANK;
		}
        
        // Null-terminate in order for lcd_print() to function properly
        line_buffer[ROADLEN+1] = 0;
        
        // Position the cursor in place
		lcd_set_cursor(0, i-1);
        
        // Actually print the line
		lcd_print(line_buffer);
	}    
}

