// TODO: change comm handlers and unsigned char new bytes to ints to prepare for 16 effects

// ======================= //
// === Include Headers === //
// ======================= //

#include <stdio.h>
#include <stdlib.h>
#include <htc.h>
#include <xc.h>

// =========================== //
// === Redefine Statements === //
// =========================== //

// i2c

#define EIO_CLK			RB6
#define EIO_CLK_F		TRISB6
#define EIO_DAT			RB5
#define EIO_DAT_F		TRISB5

#define INT_A			RC7
#define INT_B			RB7

#define ROM_CLK			RB4
#define ROM_CLK_F		TRISB4
#define ROM_DAT			RC2
#define ROM_DAT_F		TRISC2

#define DEBOUNCE		20

// global

#define BUF_LEN            	16
#define NUM_FX           	4
#define NUM_BIT           	2
#define NUM_OC                  1

#define W			0X00
#define R			0X01

// MCP23017

#define I1			0X40
#define O1			0X42

#define ICXR			0X00
#define OCXR			0X01
#define CPXR			0X02

#define IODIR			0X00
#define IPOL			0X02
#define GPINTEN			0X04
#define DEFVAL			0X06
#define INTCON			0X08
#define IOCON			0X0A
#define GPPU			0X0C
#define INTF			0X0E
#define INTCAP			0X10
#define GPIO			0X12
#define OLAT			0X14

#define A			0X00
#define B			0X01

// 24LC16

#define M1                      0xA0

// #define err_out_dispute         0x00 // OLAT does not match GPIO after output assignment fault
// #define err_comm                0x01 // communication cycle iterated out fault

// random

#define RAND                    RC1

// testing redefines

#define CLK_LED			RC0
#define CLK_LED_F		TRISC0
#define DAT_LED			RC1
#define DAT_LED_F		TRISC1
//#define FLG_LED			RC2
//#define FLG_LED_F		TRISC2
#define FLG_LED			RA2
#define FLG_LED_F		TRISA2

// ========================= //
// === CONFIG Statements === //
// ========================= //

#pragma config FOSC = INTRCIO
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF

// ============================= //
// === Function Declarations === //
// ============================= //

void initialize(void);

// i2c

void set_dat(char);
void set_clk(char);

void start(void);
void stop(void);

unsigned char ack_w(void);
void ack_r(void);

void byte_w(unsigned char);
unsigned char byte_r(void);

unsigned char comm(unsigned char, unsigned char, unsigned char, unsigned char);

// global

void buf_to_byte(void);
void byte_to_buf(void);
unsigned char io_to_buf(unsigned char, unsigned char, unsigned char);
unsigned char buf_to_io(unsigned char, unsigned char, unsigned char);
unsigned char is_multi_bit(unsigned char);

// extended io

void mode_sel(unsigned char);
void mode_exe(unsigned char);
void eio_get(void);
void eio_set(void);

// easy mode

void easy_sel(void);
void easy_exe(unsigned char);

// hard mode

void hard_sel(void);
void hard_exe(unsigned char);

// save mode

void save_exe(unsigned char);
void save_sel(void);

// call mode

void call_sel(void);

// edit mode

void edit_sel(void);
void edit_exe(unsigned char);

// snip mode

void snip_sel(void);

// rand mode

void rand_sel(void);

// testing
void on_off(void);
void in_out(void);
void buf_to_byte_test(void);
void io_to_buf_test(void);

// ============================= //
// === Variable Declarations === //
// ============================= //

// i2c

unsigned char g_eio_i2c  = 0;
unsigned char g_rom_i2c = 0;

// global

unsigned char g_mode = 0x00;

unsigned char g_comm = 0;

// BUF_LEN + 1
    // pin byte holds which relay was turned on last
    // pin byte only holds one on bit unless denoting the end of the buffer stack (0xFF)
    // data byte's LSB choose port A (0) or port B (1)
    // data byte's upper 7 MSBs hold the chip number
unsigned char g_buf_pin[BUF_LEN + 1];
unsigned char g_buf_dat[BUF_LEN + 1];

unsigned char g_ports[NUM_OC][2] =
  {0x00, 0x00}; // O1: A, B

unsigned char g_poly1 = 0;
unsigned char g_poly2 = 0;
unsigned char g_poly3 = 0;

// ==================== //
// === Main Program === //
// ==================== //

int main(int argc, char** argv) {

    initialize();

    while(1)
    {
//        on_off();
//        in_out();
//        buf_to_byte_test();
//        io_to_buf_test();
        eio_get();
    }

    return (EXIT_SUCCESS);
}

void initialize(void) {

    ANS0 = 0; ANS1 = 0; ANS2 = 0; ANS3 = 0;
    ANS4 = 0; ANS5 = 0; ANS6 = 0; ANS7 = 0;
    ANS8 = 0; ANS9 = 0; ANS10 = 0; ANS11 = 0; // turn analog pin functionality off

    TRISA = 0b11111111; // all uC pins input
    TRISB = 0b11111111; // all uC pins input
    TRISC = 0b11111111; // all uC pins input

    PORTA = 0b11111111; // all uC pins high
    PORTB = 0b11111111; // all uC pins high
    PORTC = 0b11111111; // all uC pins high

    CLK_LED_F = 0;
    DAT_LED_F = 0;
    FLG_LED_F = 0;

    CLK_LED = 0;
    DAT_LED = 0;
    FLG_LED = 0;

    OSCCON = 0b01110111;

    for (unsigned int i = 0; i < 65535; i++); // wait for all components to properly wake

    g_comm = 0;

    while(g_comm == 0) g_comm = comm((I1 + W), (IOCON + A), 0B00110000, 0x00); // eio configuration
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((I1 + W), (IOCON + B), 0B00110000, 0x00); // eio configuration
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((O1 + W), (IOCON + A), 0B00110000, 0x00); // eio configuration
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((O1 + W), (IOCON + B), 0B00110000, 0x00); // eio configuration
    g_comm = 0;

    while(g_comm == 0) g_comm = comm((I1 + W), (IPOL + A), 0XFF, 0x00); // invert logic for input pins
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((I1 + W), (IPOL + B), 0XFF, 0x00); // invert logic for input pins
    g_comm = 0;

    while(g_comm == 0) g_comm = comm((I1 + W), (GPINTEN + A), 0XFF, 0x00); // enable interrupting for input pins
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((I1 + W), (GPINTEN + B), 0XFF, 0x00); // enable interrupting for input pins
    g_comm = 0;

    while(g_comm == 0) g_comm = comm((O1 + W), (IODIR + A), 0X00, 0x00); // set output pins to be outputs
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((O1 + W), (IODIR + B), 0X00, 0x00); // set output pins to be outputs
    g_comm = 0;

    g_comm = comm((I1 + R), (GPIO + A), 0x00, 0x00); // read input pins to clear potential interrupts
    g_comm = comm((I1 + R), (GPIO + B), 0x00, 0x00); // read input pins to clear potential interrupts

    g_comm = comm((I1 + R), (INTCAP + A), 0x00, 0x00); // read input pins to clear potential interrupts
    g_comm = comm((I1 + R), (INTCAP + B), 0x00, 0x00); // read input pins to clear potential interrupts

    g_comm = comm((I1 + R), (INTF + A), 0x00, 0x00); // read input pins to clear potential interrupts
    g_comm = comm((I1 + R), (INTF + B), 0x00, 0x00); // read input pins to clear potential interrupts
    g_comm = 0;

    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + A), 0x00, 0x00); // set output pins low
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + B), 0x00, 0x00); // set output pins low
    g_comm = 0;

    for (char i = 0; i < BUF_LEN; i++) // reset buffer
    {
        g_buf_pin[i] = 0;
        g_buf_dat[i] = 0;
    }

    g_buf_pin[0] = 0xFF; // end of stack

    g_ports[0][0] = 0; // output chip 0 port A reset
    g_ports[0][1] = 0; // output chip 0 port B reset

    easy_sel();
}

// ===================== //
// === I2C Functions === //
// ===================== //

void set_dat(char level)
{
    EIO_DAT_F = level || !g_eio_i2c ;
    EIO_DAT = level || !g_eio_i2c ;

    ROM_DAT_F = level || !g_rom_i2c;
    ROM_DAT = level || !g_rom_i2c;

    //DAT_LED = level;
}

void set_clk(char level)
{
    EIO_CLK_F = level || !g_eio_i2c ;
    EIO_CLK = level || !g_eio_i2c ;

    ROM_CLK_F = level || !g_rom_i2c;
    ROM_CLK = level || !g_rom_i2c;

    for(unsigned int i = 0; i < DEBOUNCE; i++);

    //CLK_LED = level;
}

void start(void)
{
    set_dat(1);
    set_clk(1);
    set_dat(0);
    set_clk(0);
}

void stop(void)
{
    set_clk(0);
    set_dat(0);
    set_clk(1);
    set_dat(1);
}

unsigned char ack_w(void)
{
    unsigned char cancel = 0;
    set_dat(1);
    set_clk(1);
    while(!EIO_CLK || !ROM_CLK);
    if((EIO_DAT && g_eio_i2c ) || (ROM_DAT && g_rom_i2c)) cancel++;
    set_clk(0);
    return(cancel);

    //if(cancel) FLG_LED = !FLG_LED;
}

void ack_r(void)
{
    set_dat(1);
    set_clk(1);
    while(!EIO_CLK || !ROM_CLK);
    set_clk(0);
}

void byte_w(unsigned char byte)
{
    for(char i = 0; i < 8; i++)
    {
        set_dat((byte & 0x80) != 0);
        byte <<= 1;
        set_clk(1);
        set_clk(0);
    }
}

unsigned char byte_r(void)
{
    unsigned char value = 0;
    set_dat(1);
    for(char i = 0; i < 8; i++)
    {
	value <<= 1;
	set_clk(1);
        value |= (EIO_DAT && g_eio_i2c ) || (ROM_DAT && g_rom_i2c);
	set_clk(0);
    }
    return(value);
}

unsigned char comm(unsigned char addr, unsigned char inst, unsigned char data, unsigned char sets)
{
    switch(sets) // handle comm pin assignments
    {
        case 0x00: // output chip 1
            g_eio_i2c = 1;
            g_rom_i2c = 0;
            break;

        case 0xFF: // rom chip 1
            g_eio_i2c = 0;
            g_rom_i2c = 1;
            break;

        default: // none on
            g_eio_i2c = 0;
            g_rom_i2c = 0;
            break;
    }

    unsigned char cancel = 0;
    unsigned char value = 0;

    // write
    if(!(addr & 0x01))
    {
        start();

        byte_w(addr);
        cancel = ack_w();

        if(cancel) stop();

        else
        {
            byte_w(inst);
            cancel = ack_w();

            if(cancel) stop();

            else
            {
                byte_w(data);
                cancel = ack_w();
                if(!cancel) value = 1;
                stop();
            }
        }
    }

    // read
    else
    {
        start();

        byte_w(addr & 0xFE);
        cancel = ack_w();

        if(cancel) stop();

        else
        {
            byte_w(inst);
            cancel = ack_w();

            if(cancel) stop();

            else
            {
                // restart
                start();

                byte_w(addr | 0X01);
                cancel = ack_w();

                if(cancel) stop();

                else
                {
                    value = byte_r();
                    ack_r();
                    stop();
                }
            }
        }
    }

    return(value);
}

// ======================== //
// === Global Functions === //
// ======================== //

void buf_to_byte(void)
{
    g_ports[0][0] = 0; // output chip 0 port A reset
    g_ports[0][1] = 0; // output chip 0 port B reset

    for(unsigned char i = 0; i < BUF_LEN; i++) // loop through buffer
    {
        if(is_multi_bit(g_buf_pin[i])) break; // end of stack detect
        g_ports[(g_buf_dat[i] >> 1)][(g_buf_dat[i] & 0x01)] |= g_buf_pin[i]; // separate buffer data into port data
    }
}

void byte_to_buf(void)
{
    for (char i = 0; i < BUF_LEN; i++) // reset buffer
    {
        g_buf_pin[i] = 0;
        g_buf_dat[i] = 0;
    }

    g_buf_pin[0] = 0xFF; // end of stack

    unsigned char index = 0;

    for (char i = 0; i < 8; i++) // pin
    {
        for (char j = 0; i < 2; i++) // port
        {
            for (char k = 0; i < NUM_OC; i++) // chip
            {
                if((g_ports[k][j] >> i) & 0x01) // construct buffer data from port data
                {
                    g_buf_pin[index] = 1 << i;
                    g_buf_dat[index] = (k << 1) | j;

                    if(index < BUF_LEN - 1)
                    {
                        g_buf_pin[index + 1] = 0xFF; // end of stack
                        index++;
                    }
                }
            }
        }
    }
}

// function to change for different number of effects
// takes input and output bytes as seen from the EIO port
// returns data and pin bytes corresponding to buffer protocol
// depending on the flag byte
unsigned char io_to_buf(unsigned char in, unsigned char out, unsigned char flag)
{
    unsigned char value = 0; // return byte

    if(!flag) // pin byte
    {
        value = out; // default to out value, shift as needed

        if((in & 0b10101010) > 0) value <<= 4; // check if odd effect by bit (nibble check)
    }

    else // data byte
    {
        if((in & 0b00001100) > 0) value = 1; // chip 0 port B, check if last 2 effects by bit (port check)
    }

    return(value);
}

// function to change for different number of effects
// takes pin and data bytes as seen from the buffer
// returns input and output bytes corresponding to buffer protocol
// depending on the flag byte
unsigned char buf_to_io(unsigned char pin, unsigned char dat, unsigned char flag)
{
    unsigned char value = 0; // return byte

    if(flag) // 1 / input byte
    {
        value = 1; // default to 00000001, shift as needed

        if(dat & 0x01) value << 2; // chip 0 port B
        if(pin & 0b11110000) value << 1; // upper nibble
    }

    else // 0 / output byte
    {
        value = pin; // default to pin value, shift as needed

        if(value & 0b11110000) value >>= 4;
    }

    return(value);
}

unsigned char is_multi_bit(unsigned char byte)
{
    unsigned char value = 0;

    for (char i = 0; i < 8; i++)
    {
        if((byte >> i) & 0x01) value++;
    }

    value--;

    if(value > 0) value = 1;

    return(value);
}

// ============================= //
// === Extended IO Functions === //
// ============================= //

void eio_get(void)
{
    g_comm = 0;

    if(!INT_A) // input chip 0 port A changed (active low)
    {
        while(!INT_A) g_comm = comm((I1 + R), (GPIO + A), 0x00, 0x00); // reading the port resets the interrupt

	if((g_comm != 0x00) && !(is_multi_bit(g_comm))) mode_sel(g_comm); // check for press release and multi press
    }

    if(!INT_B) // input chip 0 port A changed (active low)
    {
        while(!INT_B) g_comm = comm((I1 + R), (GPIO + B), 0x00, 0x00); // reading the port resets the interrupt

        if((g_comm != 0x00) && !(is_multi_bit(g_comm))) mode_exe(g_comm); // check for press release and multi press
    }
}

void eio_set(void)
{
    buf_to_byte(); // construct port bytes from buffer data

    g_comm = 0;

    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + A), g_ports[0][0], 0x00); // output chip 0 port A
    g_comm = 0;

    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + B), g_ports[0][1], 0x00); // output chip 0 port B
    g_comm = 0;
}

void mode_sel(unsigned char new_byte)
{
    switch(new_byte)
    {
        case 0x01: // easy
            easy_sel();
            break;
        case 0x02: // hard
            hard_sel();
            break;
        case 0x04: // save
            save_sel();
            break;
        case 0x08: // call
            call_sel();
            break;
        case 0x10: // edit
            edit_sel();
            break;
        case 0x20: // snip
            snip_sel();
            break;
        case 0x40: // rand
            rand_sel();
            break;
        case 0x80: // open
            // do sel action
            break;
        default:
            break;
    }
}

void mode_exe(unsigned char new_byte)
{
    switch(g_mode)
    {
        case 0x01: // easy
            easy_exe(new_byte);
            break;
        case 0x02: // hard
            hard_exe(new_byte);
            break;
        case 0x04: // save
            save_exe(new_byte);
            break;
        case 0x08: // call (functionally identical to save)
            save_exe(new_byte);
            break;
        case 0x10: // edit
            edit_exe(new_byte);
            break;
        case 0x20: // snip
            // no action needed
            break;
        case 0x40: // rand
            // no action needed
            break;
        case 0x80: // open
            // do exe action
            break;
        default:
            break;
    }
}

// =========================== //
// === Easy Mode Functions === //
// =========================== //

void easy_sel(void)
{
    if(g_mode != 0x01) // if not already easy mode
    {
        g_poly1 = 0; // flag byte where bits correspond to each effect

        easy_exe(0x00); // self reset

        g_mode = 0x01; // set mode to easy mode
    }
}

void easy_exe(unsigned char new_byte)
{
    g_poly1 ^= new_byte;

    for (char i = 0; i < BUF_LEN; i++) // reset buffer
    {
        g_buf_pin[i] = 0;
        g_buf_dat[i] = 0;
    }

    g_buf_pin[0] = 0xFF; // end of stack

    g_poly2 = 0; // flag byte for first effect assigned
    g_poly3 = 0; // holds the last effect number

    unsigned char index = 0; // next available stack index holder

    for(char i = 1; i < 8; i++)
    {
         if((g_poly1 & 0xFE) == 0) // check if no effects on disregarding 0
         {
             g_poly2 = 0; // reset flag byte for first effect assigned
             break;
         }

        if((g_poly1  >> i) & 0x01)
        {
            index = 0; // reset index holder
            while(!is_multi_bit(g_buf_pin[index]) && (index < BUF_LEN - 1)) index++; // get next available stack index

            if(!g_poly2) // assign first effect
            {
                g_poly2 = 1; // set the flag

                g_buf_pin[index] = io_to_buf(1 << 0, 1 << i, 0); // master input to this effect
                g_buf_dat[index] = io_to_buf(1 << 0, 1 << i, 1); // master input to this effect
            }

            else // assign intermediate effects
            {
                g_buf_pin[index] = io_to_buf(g_poly3, 1 << i, 0); // last effect to this effect
                g_buf_dat[index] = io_to_buf(g_poly3, 1 << i, 1); // last effect to this effect
            }

            if(index < BUF_LEN - 1) g_buf_pin[index + 1] = 0xFF; // end of stack at index holder if allowable

            g_poly3 = 1 << i; // set the last effect
        }
    }

    if(!g_poly2) // input to output
    {
        index = 0; // reset index holder
        while(!is_multi_bit(g_buf_pin[index]) && (index < BUF_LEN - 1)) index++; // get next available stack index

        g_buf_pin[index] = io_to_buf(1 << 0, 1 << 0, 0);
        g_buf_dat[index] = io_to_buf(1 << 0, 1 << 0, 1);

        if(index < BUF_LEN - 1) g_buf_pin[index + 1] = 0xFF; // end of stack at index holder if allowable
    }

    else // assign output line
    {
        index = 0; // reset index holder
        while(!is_multi_bit(g_buf_pin[index]) && (index  < BUF_LEN - 1)) index++; // get next available stack index

        g_buf_pin[index] = io_to_buf(g_poly3, 1 << 0, 0);
        g_buf_dat[index] = io_to_buf(g_poly3, 1 << 0, 1);

        if(index < BUF_LEN - 1) g_buf_pin[index + 1] = 0xFF; // end of stack at index holder if allowable
    }

    eio_set();
}

// =========================== //
// === Hard Mode Functions === //
// =========================== //

void hard_sel(void)
{
    if(g_mode == 0x01) // if coming from easy mode / to turn off input to output / delete if input to output desired
    {
        for (char i = 0; i < BUF_LEN; i++) // reset buffer
        {
            g_buf_pin[i] = 0;
            g_buf_dat[i] = 0;
        }

        g_buf_pin[0] = 0xFF; // end of stack

        eio_set();
    }

    if(g_mode != 0x02) // if not already hard mode
    {
        g_poly1 = 0; // input byte
        g_poly2 = 0; // input selected flag / output byte
        g_poly3 = 0; // repeat assignment flag

        g_mode = 0x02; // set mode to hard mode
    }
}

void hard_exe(unsigned char new_byte)
{
    if(g_poly2) // input line already assigned
    {
        g_poly2 = new_byte; // set output line
        g_poly3 = 0; // reset repeat flag

        unsigned char pin = io_to_buf(g_poly1, g_poly2, 0); // convert input/output lines to pin byte
        unsigned char dat = io_to_buf(g_poly1, g_poly2, 1); // convert input/output lines to dat byte

        unsigned char index; // end of stack flag index holder

        for(unsigned char i = 0; i < BUF_LEN; i++) // loop through buffer
        {
            index = i;

            if(is_multi_bit(g_buf_pin[i]) || g_poly3) break; // end of stack or flag detect

            if((pin == g_buf_pin[i]) && (dat == g_buf_dat[i])) g_poly3 = 1; // set repeat flag
        }

        if(!g_poly3) // not a repeat
        {
            g_buf_pin[index] = pin; // place in buffer
            g_buf_dat[index] = dat; // place in buffer
            if(index < BUF_LEN - 1) g_buf_pin[index + 1] = 0xFF; // end of stack at index holder if allowable

            eio_set();
        }

        g_poly1 = 0; // reset input byte
        g_poly2 = 0; // reset input selected flag / output byte
        // g_poly3 = 0; // reset repeat flag
    }

    else
    {
        g_poly1 = new_byte; // set input
        g_poly2 = 1; // set input selected flag
        // g_poly3 = 0; // reset repeat flag
    }
}

// =========================== //
// === Save Mode Functions === //
// =========================== //

void save_sel(void)
{
    if(g_mode != 0x04) // if not alrady save mode
    {
        buf_to_byte(); // put the buffer in order
        byte_to_buf(); // put the buffer in order

        g_poly1 = 0; // memory "page"
        g_poly2 = 0; // memory "slot"

        g_mode = 0x04; // set mode to save mode
    }

    else
    {
        g_comm = 0;

        g_comm = comm(M1 + (g_poly1 << 1) + W, g_poly2 * 2 + 0, g_ports[0][0], 0xFF); // set byte to memory
        g_comm = comm(M1 + (g_poly1 << 1) + W, g_poly2 * 2 + 1, g_ports[0][1], 0xFF); // set byte to memory

        hard_sel(); // set mode to hard mode
    }
}

void save_exe(unsigned char new_byte) // memory selection interface
{
    unsigned char effect = 0;

    while(new_byte != 1)
    {
        new_byte >>= 1;
        effect++;
    }

    if(g_poly1 != effect)
    {
        g_poly1 = effect;
        g_poly2 = 0;
    }

    else
    {
        if(g_poly2 < 127) // 127 = 256 / 2 - 1, 2 bytes required for each port
        {
            g_poly2++;
        }

        else
        {
            g_poly2 = 0;
        }
    }
}

// =========================== //
// === Call Mode Functions === //
// =========================== //

void call_sel(void)
{
    if(g_mode != 0x08) // if not already call mode
    {
        g_poly1 = 0; // input line holder
        g_poly2 = 0; // output line holder
        g_poly3 = 0; // index holder and flag

        g_mode = 0x08; // set mode to call mode
    }

    else
    {
        g_ports[0][0] = comm(M1 + (g_poly1 << 1) + R, g_poly2 * 2 + 0, 0x00, 0xFF); // get byte from memory
        g_ports[0][1] = comm(M1 + (g_poly1 << 1) + R, g_poly2 * 2 + 1, 0x00, 0xFF); // get byte from memory

        byte_to_buf(); // construct the buffer

        eio_set();

        hard_sel(); // set mode to hard mode
    }
}

// =========================== //
// === Edit Mode Functions === //
// =========================== //

void edit_sel(void)
{
    if(g_mode != 0x10) // if not already edit mode
    {
        buf_to_byte();
        byte_to_buf(); // put the buffer in order

        g_poly1 = 0; // input line holder
        g_poly2 = 0; // output line holder
        g_poly3 = 0; // last used index holder

        g_mode = 0x10; // set mode to edit mode
    }
}

void edit_exe(unsigned char new_byte)
{
    if((g_poly1 == new_byte) && (g_poly3 < BUF_LEN - 1)) // next slot if not at last of stack
    {
        unsigned char index = g_poly3 + 1; // index holder for starting point in for loop

        g_poly3 = 0; // clear index

        for(unsigned char i = index; i < BUF_LEN; i++) // loop through rest of buffer
        {
            if(is_multi_bit(g_buf_pin[i]) || (g_poly3 > 0)) break; // end of stack or index change detect

            if(new_byte == buf_to_io(g_buf_pin[i], g_buf_dat[i], 1)) // next slot for this input line
            {
                g_poly1 = buf_to_io(g_buf_pin[i], g_buf_dat[i], 1); // input line
                g_poly2 = buf_to_io(g_buf_pin[i], g_buf_dat[i], 0); // output line
                g_poly3 = i; // hold this index
            }
        }

        if(!(g_poly3 > 0)) // check for no change / last occurence in buffer / cycle to first
        {
            g_poly1 = 0; // null input line
            g_poly2 = 0; // null output line
            g_poly3 = 0; // clear index
        }
    }

    else
    {
        g_poly1 = 0; // null input line
        g_poly2 = 0; // null output line
        g_poly3 = 0; // clear index
    }

    if(g_poly1 == 0) // go to first slot for this input line
    {
        for(unsigned char i = 0; i < BUF_LEN; i++) // loop through buffer
        {
            if(is_multi_bit(g_buf_pin[i]) || (g_poly3 > 0)) break; // end of stack or index change detect

            if(new_byte == buf_to_io(g_buf_pin[i], g_buf_dat[i], 1)) // first slot for this input line
            {
                g_poly1 = buf_to_io(g_buf_pin[i], g_buf_dat[i], 1); // input line
                g_poly2 = buf_to_io(g_buf_pin[i], g_buf_dat[i], 0); // first output line
                g_poly3 = i; // hold this index
            }
        }
    }
}

// =========================== //
// === Snip Mode Functions === //
// =========================== //

void snip_sel(void)
{
    if(g_mode != 0x10) // if not coming from edit mode
    {
        g_poly3 = 0; // buffer index holder
        while(!is_multi_bit(g_buf_pin[g_poly3]) && (g_poly3 < BUF_LEN - 1)) g_poly3++; // get next available stack index

        g_mode = 0x20; // set mode to snip mode for good reset in hard sel
    }

    for (unsigned char i = 0; i < BUF_LEN; i++) // loop through buffer
    {
        if(is_multi_bit(g_buf_pin[i])) // end of stack detect
        {
            if(i > 0) // check lower limit / empty buffer
            {
                g_buf_pin[i] = 0; // new end of stack flag is at index - 1
                g_buf_dat[i] = 0; // clear this index of the buffer
            }

            break;
        }

        if(i >= g_poly3)
        {
            if(i < BUF_LEN - 1) // check upper limit / last of stack
            {
                g_buf_pin[i] = g_buf_pin[i + 1]; // shift buffer
                g_buf_dat[i] = g_buf_dat[i + 1]; // shift buffer
            }

            else // buffer was full / there was no last of stack flag / insert one
            {
                g_buf_pin[i] = 0xFF; // end of stack
                g_buf_dat[i] = 0; // end of stack
            }
        }
    }

    eio_set();

    if(g_mode != 0x10) hard_sel(); // set mode to hard mode unless coming from edit mode
}

// =========================== //
// === Rand Mode Functions === //
// =========================== //

void rand_sel(void)
{
    for (char i = 0; i < BUF_LEN; i++) // reset buffer
    {
        g_buf_pin[i] = 0;
        g_buf_dat[i] = 0;
    }

    g_buf_pin[0] = 0xFF; // end of stack

    g_mode = 0x40; // set mode to random mode for good reset in hard sel

    hard_sel(); // set mode to hard mode

    unsigned char length = 0; // random length of buffer
    unsigned char byte = 0; // general use byte

    for (unsigned char i = 0; i < 4; i++) // generate random buffer length where BUF_LEN = 16 so iterate 4 bits
    {
        if(RAND) length |= 1 << i;
    }

    // iterate through the buffer until met random length
    while(byte != length)
    {
        byte = 0; // reset for shifting use

        for (char i = 0; i < 2; i++) // length of byte for 3 effects is 4 so iterate 2 bits
        {
            if(RAND) byte |= 1 << i;
        }

        byte = 1 << byte; // convert to random byte with one bit on

        hard_exe(byte); // send to hard exe for handling the buffer

        byte = 0; // reset to get length of buffer
        while(!is_multi_bit(g_buf_pin[byte]) && (byte < BUF_LEN - 1)) byte++;
    }
}

// ========================= //
// === Testing Functions === //
// ========================= //

void on_off(void)
{
    g_comm = 0;

    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + A), 0b10101010, 0x00);
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + B), 0b01010101, 0x00);
    g_comm = 0;
    for (unsigned int i = 0; i < 65535; i++);
    for (unsigned int i = 0; i < 65535; i++);

    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + A), 0b01010101, 0x00);
    g_comm = 0;
    while(g_comm == 0) g_comm = comm((O1 + W), (OLAT + B), 0b10101010, 0x00);
    for (unsigned int i = 0; i < 65535; i++);
    for (unsigned int i = 0; i < 65535; i++);
}

void in_out(void)
{
    g_comm = 0;

    unsigned char true_byte = 0;

    DAT_LED = INT_A;
    CLK_LED = INT_B;

    if(!INT_A)
    {
        while(!INT_A) g_comm = comm((I1 + R), (GPIO + A), 0x00, 0x00);

        while(true_byte == 0) true_byte = comm((O1 + W), (OLAT + A), g_comm, 0x00);
        true_byte = 0;
    }

    if(!INT_B)
    {
        while(!INT_B) g_comm = comm((I1 + R), (GPIO + B), 0x00, 0x00);

        while(true_byte == 0) true_byte = comm((O1 + W), (OLAT + B), g_comm, 0x00);
    }
}

void buf_to_byte_test(void)
{
    for (unsigned char i = 0; i < BUF_LEN; i++) // reset buffer
    {
        g_buf_pin[i] = 0;
        g_buf_dat[i] = 0;
    }

    g_buf_pin[0] = 0xFF; // end of stack

    for (char port = 0; port < 2; port++)
    {
        for (unsigned char i = 0; i < 8; i++)
        {
//            // #1 / cycle around
//            g_buf_pin[1] = 0xFF; // end of stack
//            g_buf_pin[0] = 1 << i;
//            g_buf_dat[0] = port;

//            // #2 / port accumulate around
//            if(i < BUF_LEN - 1) g_buf_pin[i + 1] = 0xFF; // end of stack
//            g_buf_pin[i] = 1 << i;
//            g_buf_dat[i] = port;

            // #3 / chip accumulate around
            if(port * 8 + i < BUF_LEN - 1) g_buf_pin[port * 8 + i + 1] = 0xFF; // end of stack
            g_buf_pin[port * 8 + i] = 1 << i;
            g_buf_dat[port * 8 + i] = port;

            eio_set();

            for (unsigned int i = 0; i < 65535; i++);
            for (unsigned int i = 0; i < 65535; i++);
        }
    }
}

void io_to_buf_test(void)
{
    for (unsigned char i = 0; i < BUF_LEN; i++) // reset buffer
    {
        g_buf_pin[i] = 0;
        g_buf_dat[i] = 0;
    }

    g_buf_pin[0] = 0xFF; // end of stack

    for (unsigned char in = 0; in < NUM_FX; in++)
    {
        for (unsigned char out = 0; out < NUM_FX; out++)
        {
//            // #1 / cycle around
//            g_buf_pin[1] = 0xFF; // end of stack
//            g_buf_pin[0] = io_to_buf(1 << in, 1 << out, 0);
//            g_buf_dat[0] = io_to_buf(1 << in, 1 << out, 1);

            // #2 / chip accumulate around
            if(in * 4 + out < BUF_LEN - 1) g_buf_pin[in * 4 + out + 1] = 0xFF; // end of stack
            g_buf_pin[in * 4 + out] = io_to_buf(1 << in, 1 << out, 0);
            g_buf_dat[in * 4 + out] = io_to_buf(1 << in, 1 << out, 1);

            eio_set();

            for (unsigned int i = 0; i < 65535; i++);
            for (unsigned int i = 0; i < 65535; i++);
        }
    }
}