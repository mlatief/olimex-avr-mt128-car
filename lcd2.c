/* lcd2.c - Extension to HD44780 LCD driver */

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include "lcd.h"
#include "lcd2.h"

const unsigned int NUMROWS = 2;
/* First character position mapping to DDRAM address for each LCD row */
char _row_offsets[]= {0x00, 0x40};

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

/* Print a null-terminated string to the LCD from the current position */
void
lcd_print(const char* buffer){
    unsigned int size = strlen(buffer);
    while(size--){
        lcd_write_data( *buffer++ );
    }
}


/* Set the cursor (by setting DDRAM address) */
void
lcd_set_cursor(unsigned int col, unsigned int row)
{
    if(row > NUMROWS-1) row = NUMROWS - 1;
    char _row_offset = _row_offsets[row];
    
    lcd_write_ctrl(LCD_DDRAM | (col + _row_offset));
}