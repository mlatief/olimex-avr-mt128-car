/* lcd2.c - Extension to HD44780 LCD driver */

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include "lcd.h"
#include "lcd2.h"

/* Write new character to CGRAM */
void lcd_create_glyph(int loc, const char* charmap){
    // Using 5 x 8 character maps, then only 8 characters allowed
    loc &=  0x07;
    
    // Send Set CGRAM address instruction
    lcd_write_ctrl( LCD_CGRAM | (loc<<3) );
    
    // Address Counter automatically increment, 
    // so we start sending the character row by row
    for(int i=0; i<8; i++){
        lcd_write_data( charmap[i] );
    }
}

void
lcd_print(const char* buffer){
    unsigned int size = strlen(buffer);
    while(size--){
        lcd_write_data( *buffer++ );
    }
}
