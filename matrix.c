/*
Copyright 2012 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"


/*
 * Happy Buckling Keyboard(IBM Model M mod)
 *
 * Pin usage:
 *   COL: PD0-7
 *   pin: C4 C5 C6 C7 F4 F5 F6 F7 lance
 *   ROW: PB0-7, PF4-7
 *   pin: B4 B3 F0 F1 F2 D2 D3 D4 D5 D7 E0 E1 C0 C1 C2 C3 lance
 */
#ifndef DEBOUNCE
#   define DEBOUNCE	10
#endif
static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];

static matrix_row_t read_cols(void);
static void unselect_rows(void);
static void select_row(uint8_t row);


void matrix_init(void)
{
    // JTAG disable for PORT F. write JTD bit twice within four cycles.
    MCUCR |= (1<<JTD);
    MCUCR |= (1<<JTD);

    // initialize rows
    unselect_rows();

    // initialize columns to input with pull-up(DDR:0, PORT:1)
    //DDRD = 0x00;
    //PORTD = 0xFF;
    DDRC  &= ~0b11110000; /* sets bit 4-7 to 0 input */
    PORTC |=  0b11110000; /* sets bit 4-7 to 1 pullup resistor*/
    DDRF  &= ~0b11110000; /* sets bit 4-7 to 0 input */
    PORTF |=  0b11110000; /* sets bit 4-7 to 1 pullup resistor*/

    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
        matrix_debouncing[i] = 0;
    }
}

uint8_t matrix_scan(void)
{
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        select_row(i);
        _delay_us(30);  // without this wait read unstable value.
        matrix_row_t cols = read_cols();
        if (matrix_debouncing[i] != cols) {
            matrix_debouncing[i] = cols;
            if (debouncing) {
                debug("bounce!: "); debug_hex(debouncing); debug("\n");
            }
            debouncing = DEBOUNCE;
        }
        unselect_rows();
    }

    if (debouncing) {
        if (--debouncing) {
            _delay_ms(1);
        } else {
            for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
                matrix[i] = matrix_debouncing[i];
            }
        }
    }

    return 1;
}

inline
matrix_row_t matrix_get_row(uint8_t row)
{
    return matrix[row];
}

inline
static matrix_row_t read_cols(void)
{
    //return ~PIND;
	//pin: C4 C5 C6 C7 F4 F5 F6 F7 lance
    return (PINC&(1<<4) ? 0 : (1<<0)) |
           (PINC&(1<<5) ? 0 : (1<<1)) |
           (PINC&(1<<6) ? 0 : (1<<2)) |
           (PINC&(1<<7) ? 0 : (1<<3)) |
           (PINF&(1<<4) ? 0 : (1<<4)) |
           (PINF&(1<<5) ? 0 : (1<<5)) |
           (PINF&(1<<6) ? 0 : (1<<6)) |
           (PINF&(1<<7) ? 0 : (1<<7)) ;
}

inline
static void unselect_rows(void)
{
    // Hi-Z(DDR:0, PORT:0) to unselect
    //DDRB  &= ~0b11111111;
    //PORTB &= ~0b11111111;
    //DDRF  &= ~0b11110000;
    //PORTF &= ~0b11110000;

    DDRB  &= ~0b00011000;
    PORTB &=  0b00011000;
    DDRC  &= ~0b00001111;
    PORTC &=  0b00001111;
    DDRD  &= ~0b10111100;
    PORTD &=  0b10111100;
    DDRE  &= ~0b00000011;
    PORTE &=  0b00000011;
    DDRF  &= ~0b00000111;
    PORTF &=  0b00000111;
}

inline
static void select_row(uint8_t row)
{
    // Output low(DDR:1, PORT:0) to select
	//pin: B4 B3 F0 F1 F2 D2 D3 D4 D5 D7 E0 E1 C0 C1 C2 C3 lance
    switch (row) {
        case 0:
            DDRB  |=  (1<<4);
            PORTB &= ~(1<<4);
            break;
        case 1:
            DDRB  |=  (1<<3);
            PORTB &= ~(1<<3);
            break;
        case 2:
            DDRF  |=  (1<<0);
            PORTF &= ~(1<<0);
            break;
        case 3:
            DDRF  |=  (1<<1);
            PORTF &= ~(1<<1);
            break;
        case 4:
            DDRF  |=  (1<<2);
            PORTF &= ~(1<<2);
            break;
        case 5:
            DDRD  |=  (1<<2);
            PORTD &= ~(1<<2);
            break;
        case 6:
            DDRD  |=  (1<<3);
            PORTD &= ~(1<<3);
            break;
        case 7:
            DDRD  |=  (1<<4);
            PORTD &= ~(1<<4);
            break;
        case 8:
            DDRD  |=  (1<<5);
            PORTD &= ~(1<<5);
            break;
        case 9:
            DDRD  |=  (1<<7);
            PORTD &= ~(1<<7);
            break;
        case 10:
            DDRE  |=  (1<<0);
            PORTE &= ~(1<<0);
            break;
        case 11:
            DDRE  |=  (1<<1);
            PORTE &= ~(1<<1);
            break;
        case 12:
            DDRC  |=  (1<<0);
            PORTC &= ~(1<<0);
            break;
        case 13:
            DDRC  |=  (1<<1);
            PORTC &= ~(1<<1);
            break;
        case 14:
            DDRC  |=  (1<<2);
            PORTC &= ~(1<<2);
            break;
        case 15:
            DDRC  |=  (1<<3);
            PORTC &= ~(1<<3);
            break;
    }
}
