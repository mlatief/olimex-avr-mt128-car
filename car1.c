#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include "lcd2.h"

#include "Bxxxxx.h"

/* Constants */
const int ROADLEN = 15;
const int NGLYPHS = 6;
const char BLANK = 32;

char glyphs[6][8] = {
	// 1: car up
   {B00000,
	B01110,
	B11111,
	B01010,
	B00000,
	B00000,
	B00000,
	B00000}
	// 2: car down
  ,{B00000,
	B00000,
	B00000,
	B00000,
	B01110,
	B11111,
	B01010,
	B00000}
	// 3: truck up
  ,{B00000,
	B11110,
	B11111,
	B01010,
	B00000,
	B00000,
	B00000,
	B00000}
	// 4: truck down
  ,{B00000,
	B00000,
	B00000,
	B00000,
	B11110,
	B11111,
	B01010,
	B00000}
	// 5: crash up
  ,{B10101,
	B01110,
	B01110,
	B10101,
	B00000,
	B00000,
	B00000,
	B00000}
	// 6: crash down
  ,{B00000,
	B00000,
	B00000,
	B10101,
	B01110,
	B01110,
	B10101,
	B00000}
};

/* State Variables*/
char car_pos = 0;   // 0: 1st line of the LCD, 1: 2nd line
char road[15];      // 
int road_index;

void init(void) {

   		/* estet‰‰n kaikki keskeytykset */
		cli();

        /* kaiutin pinnit ulostuloksi */
        DDRE  |=  (1 << PE4) | (1 << PE5);
        /* pinni PE4 nollataan */
        PORTE &= ~(1 << PE4);
        /* pinni PE5 asetetaan */
        PORTE |=  (1 << PE5);   
        
        /* ajastin nollautuu, kun sen ja OCR1A rekisterin arvot ovat samat */
		/* set the timer to CTC mode */
        TCCR1A &= ~( (1 << WGM11) | (1 << WGM10) );
        TCCR1B |=    (1 << WGM12);
        TCCR1B &=   ~(1 << WGM13);

        /* salli keskeytys, jos ajastimen ja OCR1A rekisterin arvot ovat samat */
		/* enable Output Compare A Match Interrupt */
        //TIMSK |= (1 << OCIE1A);
        TIMSK &= ~(1 << OCIE1A);

        /* set OCR1A register value to 0x003e (corresponds to ~250hz) */
        /* OCR1AH = 0x00; */
        /* OCR1AL = 0x3e; */

		/* set OCR1A register value to 0x0013 (corresponds to ~800hz) */
        OCR1AH = 0x00;
        OCR1AL = 0x13;

        /* k‰ynnist‰ ajastin ja k‰yt‰ kellotaajuutena (16 000 000 / 1024) Hz */
		/* start the counter (16 000 000 / 1024) Hz */
        TCCR1B |= (1 << CS12) | (1 << CS10);

		/* n‰pp‰in pinnit sis‰‰ntuloksi */
		DDRA &= ~(1 << PA0);
		DDRA &= ~(1 << PA2);
		DDRA &= ~(1 << PA4);

		/* rele/led pinni ulostuloksi */
		DDRA |= (1 << PA6);

		/* lcd-n‰ytˆn alustaminen */
		lcd_init();
        
        for (int i = 0; i < NGLYPHS; i++) {
            lcd_create_glyph(i + 1, glyphs[i]);	// create glyphs
        }
		
        lcd_write_ctrl(LCD_ON);
		lcd_write_ctrl(LCD_CLEAR);
        lcd_write_data('<');
        lcd_write_data(BLANK);
        lcd_write_data(1);
        lcd_write_data(2);
        lcd_write_data(3);
        lcd_write_data(4);
        lcd_write_data(5);
        lcd_write_data(6);
        lcd_write_data(BLANK);
        lcd_write_data('>');
}


int main(void) 
{


		/* alusta laitteen komponentit */
		init();
		sei();
			
        /* ikuinen silmukka */
        while (1) {

		}
}

ISR(TIMER1_COMPA_vect) {

	/* vaihdetaan kaiutin pinnien tilat XOR operaatiolla */
 	PORTE ^= (1 << PE4) | (1 << PE5); 
}


