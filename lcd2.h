/* lcd2.h - Extension to HD44780 LCD driver */

#ifndef LCD2_H
#define LCD2_H

void lcd_create_glyph(int,const char*);
void lcd_print(const char*);
void lcd_set_cursor(unsigned int, unsigned int);

#endif
