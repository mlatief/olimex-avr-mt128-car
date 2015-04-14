/* lcd.c - HD44780 LCD driver */

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

static void write_nibble(int, int);
static int read_nibble(int);
static int read_byte(int);
static int pulse(void);
static void wait(void);


/* initialize lcd (see HD44780 datasheet) */
void
lcd_init(void)
{
	/* set interface to be 4 bits long */
  write_nibble(3,0); _delay_ms(5);
  write_nibble(3,0); _delay_ms(5);
  write_nibble(3,0); _delay_ms(5);
  write_nibble(2,0); _delay_ms(5);

	/* function set: set number of lines (bit 3) and font (bit 2) */
  lcd_write_ctrl(LCD_FUNCTION|0x0C);
  lcd_write_ctrl(LCD_OFF);
  lcd_write_ctrl(LCD_CLEAR);
	/* entry mode set: increment DDRAM pointer when a character is written */
  lcd_write_ctrl(LCD_MODE|0x02);
}

/* write a byte into register rs (0 = control, 1 = data). */
void
lcd_write(int v, int rs)
{
  write_nibble(v>>4,rs);     /* first the high nibble */
  write_nibble(v,rs);        /* and then the low */
  wait();
}

static void
write_nibble(int v, int rs)
{
  DDRC = 0xFF;               /* set data pins to output */
  PORTC = rs|((v&0xF)<<4);   /* set register and data */
  pulse();                   /* write pulse */
}

/* read a byte from lcd register rs. */
int
lcd_read(int rs)
{
  int v;
  
  v = read_byte(rs);
  wait();

  return v;
}

/* read a byte without waiting for the busy flag. */
static int
read_byte(int rs)
{
  int v;
  
  v = read_nibble(rs)<<4;
  v|= read_nibble(rs);

  return v;
}

static int
read_nibble(int rs)
{
  DDRC = 0x0F;               /* set data pins to input */
  PORTC = rs|2;              /* set register */
  return pulse();            /* read pulse */
}

/* read/write pulse */
static int
pulse(void)
{
  int v;

  PORTC |= 4;                /* EN = 1 */
  _delay_ms(0.1);            /* short delay */
  v = PINC>>4;               /* read data bus */
  PORTC &= ~4;               /* EN = 0 */

  return v;
}

/* wait until the busy flag is clear. */
void
wait(void)
{
  while (read_byte(0)&0x80)
    ;
}
