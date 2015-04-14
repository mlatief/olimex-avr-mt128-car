/* lcd.h - HD44780 LCD driver */

#ifndef LCD_H
#define LCD_H

#define LCD_CLEAR     0x01  /* clear display */
#define LCD_HOME      0x02  /* return home */
#define LCD_MODE      0x04  /* entry mode set */
#define LCD_OFF       0x08  /* display off */
#define LCD_ON        0x0C  /* display on */
#define LCD_SHIFT     0x10  /* cursor or display shift */
#define LCD_FUNCTION  0x20  /* function set */
#define LCD_CGRAM     0x40  /* set CGRAM address */
#define LCD_DDRAM     0x80  /* set DDRAM address */

/* function prototypes */
void lcd_init(void);
void lcd_write(int, int);
int lcd_read(int);

/* macros to read/write control/data registers */
#define lcd_write_ctrl(v) lcd_write(v,0)
#define lcd_write_data(v) lcd_write(v,1)
#define lcd_read_ctrl() lcd_read(0)
#define lcd_read_data() lcd_read(1)

#endif
