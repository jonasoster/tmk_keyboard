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

/*
 * scan matrix
 */
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"


#ifndef DEBOUNCE
#   define DEBOUNCE	5
#endif
static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];

static matrix_row_t read_cols(void);
static void init_cols(void);
static void unselect_rows(void);
static void select_row(uint8_t row);

typedef uint8_t jonas_matrix_t[6];

static void jonas_init_pins(void)
{
	/* Row 4 L */
	DDRF &= ~(1 << DDF5);
	PORTF |= (1 << PORTF5);
	/* Row 3 L */
	DDRF &= ~(1 << DDF7);
	PORTF |= (1 << PORTF7);
	/* Row 2 L */
	DDRB &= ~(1 << DDB3);
	PORTB |= (1 << PORTB3);
	/* Row 1 L */
	DDRB &= ~(1 << DDB6);
	PORTB |= (1 << PORTB6);
	
	/* Row 4 R */
	DDRD &= ~(1 << DDD0);
	PORTD |= (1 << PORTD0);
	/* Row 3 R */
	DDRC &= ~(1 << DDC6);
	PORTC |= (1 << PORTC6);
	/* Row 2 R */
	DDRE &= ~(1 << DDE6);
	PORTE |= (1 << PORTE6);
	/* Row 1 R */
	DDRB &= ~(1 << DDB5);
	PORTB |= (1 << PORTB5);
	
	/* Col 1 */
	DDRF |= (1 << DDF6);
	PORTF &= ~(1 << PORTF6);
	/* Col 2 */
	DDRD |= (1 << DDD4);
	PORTD &= ~(1 << PORTD4);
	/* Col 3 */
	DDRB |= (1 << DDB1);
	PORTB &= ~(1 << PORTB1);
	/* Col 4 */
	DDRD |= (1 << DDD7);
	PORTD &= ~(1 << PORTD7);
	/* Col 5 */
	DDRB |= (1 << DDB2);
	PORTB &= ~(1 << PORTB2);
	/* Col 6 */
	DDRB |= (1 << DDB4);
	PORTB &= ~(1 << PORTB4);
}

static uint8_t jonas_scan_rows(void)
{
        uint8_t state = 0;
	
	/* Row 4 L */
	if ((PINF & (1 << PINF5)) == 0)
	  state |= (1 << 0);
	
	/* Row 3 L */
	if ((PINF & (1 << PINF7)) == 0)
	  state |= (1 << 1);
	
	/* Row 2 L */
	if ((PINB & (1 << PINB3)) == 0)
	  state |= (1 << 2);
	
	/* Row 1 L */
	if ((PINB & (1 << PINB6)) == 0)
	  state |= (1 << 3);
	
	/* Row 4 R */
	if ((PIND & (1 << PIND0)) == 0)
	  state |= (1 << 4);
	
	/* Row 3 R */
	if ((PINC & (1 << PINC6)) == 0)
	  state |= (1 << 5);
	
	/* Row 2 R */
	if ((PINE & (1 << PINE6)) == 0)
	  state |= (1 << 6);
	
	/* Row 1 R */
	if ((PINB & (1 << PINB5)) == 0)
	  state |= (1 << 7);

	return state;
}

#define JONAS_DELAY_US 50

void jonas_scan_matrix(void)
{
        /* Col 1 */
        PORTF &= ~(1 << PORTF6);
        _delay_us(JONAS_DELAY_US);
        matrix[0] = jonas_scan_rows();
        PORTF |= (1 << PORTF6);
        /* Col 2 */
        PORTD &= ~(1 << PORTD4);
        _delay_us(JONAS_DELAY_US);
        matrix[1] = jonas_scan_rows();
        PORTD |= (1 << PORTD4);
        /* Col 3 */
        PORTB &= ~(1 << PORTB1);
        _delay_us(JONAS_DELAY_US);
        matrix[2] = jonas_scan_rows();
        PORTB |= (1 << PORTB1);
        /* Col 4 */
        PORTD &= ~(1 << PORTD7);
        _delay_us(JONAS_DELAY_US);
        matrix[3] = jonas_scan_rows();
        PORTD |= (1 << PORTD7);
        /* Col 5 */
        PORTB &= ~(1 << PORTB2);
        _delay_us(JONAS_DELAY_US);
        matrix[4] = jonas_scan_rows();
        PORTB |= (1 << PORTB2);
        /* Col 6 */
        PORTB &= ~(1 << PORTB4);
        _delay_us(JONAS_DELAY_US);
        matrix[5] = jonas_scan_rows();
        PORTB |= (1 << PORTB4);
}

inline
uint8_t matrix_rows(void)
{
    return MATRIX_ROWS;
}

inline
uint8_t matrix_cols(void)
{
    return MATRIX_COLS;
}

void matrix_init(void)
{
    // initialize row and col
    /* unselect_rows(); */
    /* init_cols(); */
    jonas_init_pins();
    
    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
        matrix_debouncing[i] = 0;
    }
    debug("matrix_init\n");
}

uint8_t matrix_scan(void)
{
   /* print("\x1b[1;1H"); */
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {

        _delay_us(50);  // without this wait read unstable value.
    }
    jonas_scan_matrix();
    /* matrix[2] ^= 0x5555; */

    /* matrix_print(); */
    return 1;
}

bool matrix_is_modified(void)
{
    if (debouncing) return false;
    return true;
}

inline
bool matrix_is_on(uint8_t row, uint8_t col)
{
    return (matrix[row] & ((matrix_row_t)1<<col));
}

inline
matrix_row_t matrix_get_row(uint8_t row)
{
    return matrix[row];
}

void matrix_print(void)
{
    print("\nr/c 0123456789ABCDEF\n");
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        phex(row); print(": ");
        pbin_reverse16(matrix_get_row(row));
        print("\n");
    }
}

uint8_t matrix_key_count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        count += bitpop16(matrix[i]);
    }
    return count;
}

/* Column pin configuration
 * col: 0   1   2   3   4   5   6   7   8   9   10
 * pin: B7  D6  F7  F6  B6  D4  E6  B4  B5  C6  D7
 */
static void  init_cols(void)
{
    DDRB = DDRC = DDRE = DDRF = 0; // columns
    PORTB = PORTC = PORTE = PORTF = 255; // pullup resistors on inputs
    DDRD = 15; // rows (1 2 4 8) high and columns (16 32 64 128) low
    PORTD = 15;
}

static matrix_row_t read_cols(void)
{
#ifdef TEENSY
    return (PINF&(1<<6) ? 0 : (1<<0)) |
           (PINF&(1<<5) ? 0 : (1<<1)) |
           (PINF&(1<<4) ? 0 : (1<<2)) |
           (PINB&(1<<7) ? 0 : (1<<3)) |
           (PINB&(1<<6) ? 0 : (1<<4)) |
           (PINB&(1<<5) ? 0 : (1<<5)) |
           (PINB&(1<<4) ? 0 : (1<<6)) |
           (PINB&(1<<3) ? 0 : (1<<7)) |
           (PINB&(1<<2) ? 0 : (1<<8)) |
           (PINB&(1<<1) ? 0 : (1<<9)) |
           (PINB&(1<<0) ? 0 : (1<<10)) ;
#elseif PCBFLIP
    return (PINF&(1<<6) ? 0 : (1<<10)) |
           (PINF&(1<<5) ? 0 : (1<<9)) |
           (PINF&(1<<4) ? 0 : (1<<8)) |
           (PINB&(1<<7) ? 0 : (1<<7)) |
           (PINB&(1<<6) ? 0 : (1<<6)) |
           (PINB&(1<<5) ? 0 : (1<<5)) |
           (PINB&(1<<4) ? 0 : (1<<4)) |
           (PINB&(1<<3) ? 0 : (1<<3)) |
           (PINB&(1<<2) ? 0 : (1<<2)) |
           (PINB&(1<<1) ? 0 : (1<<1)) |
           (PINB&(1<<0) ? 0 : (1<<0)) ;
#else
    return (PINB&(1<<7) ? 0 : (1<<0)) |
           (PIND&(1<<6) ? 0 : (1<<1)) |
           (PINF&(1<<7) ? 0 : (1<<2)) |
           (PINF&(1<<6) ? 0 : (1<<3)) |
           (PINB&(1<<6) ? 0 : (1<<4)) |
           (PIND&(1<<4) ? 0 : (1<<5)) |
           (PINE&(1<<6) ? 0 : (1<<6)) |
           (PINB&(1<<4) ? 0 : (1<<7)) |
           (PINB&(1<<5) ? 0 : (1<<8)) |
           (PINC&(1<<6) ? 0 : (1<<9)) |
           (PIND&(1<<7) ? 0 : (1<<10)) ;
#endif
}

/* Row pin configuration
 * row: 0   1   2   3
 * pin: D0  D1  D3  D2    (a-star micro)
 * pin: D0  D1  D2  D3    (teensy2)
 */
static void unselect_rows(void)
{
    PORTD = 15;
}

#define ROW_COUNT 4
#ifdef TEENSY
int rows[ROW_COUNT] = {0, 1, 2, 3};
#else
int rows[ROW_COUNT] = {0, 1, 3, 2};
#endif

static void select_row(uint8_t row)
{
  PORTD = (char)(~(1 << rows[row]));
}
