/*
 * File:   main.c
 * Author: sakthi
 * Can based automotive dashboard - ECU2 - SPEED and GEAR Status 
 * Created on December 3, 2025, 4:49 PM
 */

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define _XTAL_FREQ 20000000

#define TRUE    1
#define FALSE   0

// ADC Defines
#define CHANNEL0        0x00
#define CHANNEL1        0x01
#define CHANNEL2        0x02
#define CHANNEL3        0x03
#define CHANNEL4        0x04
#define CHANNEL5        0x05
#define SPEED_ADC_CHANNEL   CHANNEL4

// Matrix Keypad Defines
#define STATE_CHANGE        1
#define LEVEL_CHANGE        0
#define MATRIX_KEYPAD_PORT  PORTB
#define ROW3                PORTBbits.RB7
#define ROW2                PORTBbits.RB6
#define ROW1                PORTBbits.RB5
#define COL4                PORTBbits.RB4
#define COL3                PORTBbits.RB3
#define COL2                PORTBbits.RB2
#define COL1                PORTBbits.RB1
#define MK_SW1              1
#define MK_SW2              2
#define MKP_ALL_RELEASED    0xFF
#define HI                  1
#define LO                  0

// LCD Defines
#define CLCD_RS         RC1
#define CLCD_RW         RC0
#define CLCD_EN         RC2
#define CLCD_PORT       PORTD
#define CLCD_TRIS       TRISD
#define LINE1(addr)     (0x80 + (addr))
#define LINE2(addr)     (0xC0 + (addr))

// CAN Defines
#define CAN_SET_OPERATION_MODE_NO_WAIT(mode) \
{ \
    CANCON &= 0x1F; \
    CANCON |= mode; \
}

#define EIDH            0
#define EIDL            1
#define SIDH            2
#define SIDL            3
#define DLC             4
#define D0              5
#define D1              6
#define D2              7
#define D3              8
#define D4              9
#define D5              10
#define D6              11
#define D7              12

// Message IDs
#define SPEED_MSG_ID    0x35E
#define GEAR_MSG_ID     0x35E

// ECU Sensor Defines
#define MAX_GEAR        6

unsigned char can_payload[13];

// ENUMS
typedef enum _CanOpMode {
    e_can_op_mode_bits    = 0xE0,
    e_can_op_mode_normal  = 0x00,
    e_can_op_mode_sleep   = 0x20,
    e_can_op_mode_loop    = 0x40,
    e_can_op_mode_listen  = 0x60,
    e_can_op_mode_config  = 0x80
} CanOpMode;

// FUNCTION PROTOTYPES

// ADC Module
void init_adc(void);
unsigned short read_adc(unsigned char channel);

// Matrix Keypad Module
void init_matrix_keypad(void);
unsigned char scan_key(void);
unsigned char read_switches(unsigned char detection_type);

// LCD Module
void init_clcd(void);
void clcd_write_cmd(unsigned char cmd);
void clcd_write_data(unsigned char data);
void clcd_print(const char *s, unsigned char addr);
void clcd_putch(unsigned char data, unsigned char addr);
void clcd_pulse(void);

// CAN Module
void init_can(void);
unsigned char can_receive(void);
void can_transmit(void);

// ECU1 Sensor Module
uint16_t get_speed(void);
unsigned char get_gear_pos(void);

void delay(unsigned short factor);

void init_adc(void)
{
    /* Selecting right justified ADRES Registers order */
    ADFM = 1;
    
    /* Acquisition time selection bits - Set for 4 Tad */
    ACQT2 = 0;
    ACQT1 = 1;
    ACQT0 = 0;
    
    /* Selecting the conversion clock of Fosc / 32 */
    ADCS0 = 0;
    ADCS1 = 1;
    ADCS2 = 0;
    
    /* Stop the conversion to start with */
    GODONE = 0;
    
    /* Voltage reference bit as VSS */
    VCFG1 = 0;
    /* Voltage reference bit as VDD */
    VCFG0 = 0;
    
    /* Clear ADRESH & ADRESL registers */
    ADRESH = 0;
    ADRESL = 0;
    
    /* Turn ON the ADC module */
    ADON = 1;
}

unsigned short read_adc(unsigned char channel)
{
    unsigned short reg_val;
    
    /* Select the channel */
    ADCON0 = (ADCON0 & 0xC3) | (channel << 2);
    
    /* Start the conversion */
    GO = 1;
    while (GO);
    
    reg_val = (ADRESH << 8) | ADRESL;
    return reg_val;
}

// MATRIX KEYPAD MODULE

void init_matrix_keypad(void)
{
    /* Config PORTB as digital */
    ADCON1 = 0x0F;
    
    /* Set Rows (RB7 - RB5) as Outputs and Columns (RB4 - RB1) as Inputs */
    TRISB = 0x1E;
    
    /* Set PORTB input as pull up for columns */
    RBPU = 0;
    
    MATRIX_KEYPAD_PORT = MATRIX_KEYPAD_PORT | 0xE0;
}

unsigned char scan_key(void)
{
    ROW1 = LO;
    ROW2 = HI;
    ROW3 = HI;
    
    if (COL1 == LO)
        return 1;
    else if (COL2 == LO)
        return 4;
    else if (COL3 == LO)
        return 7;
    else if (COL4 == LO)
        return 10;
    
    ROW1 = HI;
    ROW2 = LO;
    ROW3 = HI;
    
    if (COL1 == LO)
        return 2;
    else if (COL2 == LO)
        return 5;
    else if (COL3 == LO)
        return 8;
    else if (COL4 == LO)
        return 11;
    
    ROW1 = HI;
    ROW2 = HI;
    ROW3 = LO;
    ROW3 = LO;
    
    if (COL1 == LO)
        return 3;
    else if (COL2 == LO)
        return 6;
    else if (COL3 == LO)
        return 9;
    else if (COL4 == LO)
        return 12;
    
    return 0xFF;
}

unsigned char read_switches(unsigned char detection_type)
{
    static unsigned char once = 1, key;
    
    if (detection_type == STATE_CHANGE)
    {
        key = scan_key();
        if(key != 0xFF && once)
        {
            once = 0;
            return key;
        }
        else if(key == 0xFF)
        {
            once = 1;
        }
    }
    else if (detection_type == LEVEL_CHANGE)
    {
        return scan_key();
    }
    
    return 0xFF;
}

// LCD MODULE
void clcd_pulse(void)
{
    CLCD_EN = 1;
    __delay_us(50);
    CLCD_EN = 0;
    __delay_us(50);
}

void clcd_write_cmd(unsigned char cmd)
{
    CLCD_RS = 0;
    CLCD_RW = 0;
    CLCD_PORT = cmd;
    clcd_pulse();
    __delay_ms(5);
}

void clcd_write_data(unsigned char data)
{
    CLCD_RS = 1;
    CLCD_RW = 0;
    CLCD_PORT = data;
    clcd_pulse();
    __delay_us(100);
}

void init_clcd(void)
{
    CLCD_TRIS = 0x00;
    TRISC &= 0xF8;
    
    CLCD_RS = 0;
    CLCD_RW = 0;
    CLCD_EN = 0;
    
    __delay_ms(50);
    
    clcd_write_cmd(0x38);
    __delay_ms(5);
    clcd_write_cmd(0x38);
    __delay_ms(1);
    clcd_write_cmd(0x38);
    __delay_ms(1);
    
    clcd_write_cmd(0x0C);
    __delay_ms(1);
    
    clcd_write_cmd(0x01);
    __delay_ms(2);
    
    clcd_write_cmd(0x06);
    __delay_ms(1);
}

void clcd_print(const char *s, unsigned char addr)
{
    clcd_write_cmd(addr);
    __delay_ms(1);
    while(*s)
    {
        clcd_write_data(*s++);
    }
}

void clcd_putch(unsigned char data, unsigned char addr)
{
    clcd_write_cmd(addr);
    __delay_ms(1);
    clcd_write_data(data);
}
// CAN MODULE 
void init_can(void)
{
    /* CAN_TX = RB2, CAN_RX = RB3 */
    TRISB2 = 0;     /* CAN TX */
    TRISB3 = 1;     /* CAN RX */
    
    /* Enter CAN module into config mode */
    CAN_SET_OPERATION_MODE_NO_WAIT(e_can_op_mode_config);
    
    /* Wait until desired mode is set */
    while (CANSTAT != 0x80);
    
    /* Enter CAN module into Mode 0 */
    ECANCON = 0x00;
    
    /* Initialize CAN Timing 8MHz */
    BRGCON1 = 0xE1;     /* 1110 0001, SJW=4 TQ, BRP 4 */
    BRGCON2 = 0x1B;     /* 0001 1011, SEG2PHTS 1 sampled once PS1=4TQ PropagationT 4TQ */
    BRGCON3 = 0x03;     /* 0000 0011, PS2, 4TQ */
    
    /* Enable Filters - Filter 0 */
    RXFCON0 = 0x00;
    
    /* Initialize Receive Filters - Filter 0 = 0x35E */
    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;
    RXF0SIDH = 0x6B;
    RXF0SIDL = 0xC0;
    
    /* Enter CAN module into Loop back mode */
    CAN_SET_OPERATION_MODE_NO_WAIT(e_can_op_mode_normal);
    
    /* Set Receive Mode for buffers */
    RXB0CON = 0x00;
}

unsigned char can_receive(void)
{
    if (RXB0FUL) /* CheckRXB0 */
    {
        can_payload[EIDH] = RXB0EIDH;
        can_payload[EIDL] = RXB0EIDL;
        can_payload[SIDH] = RXB0SIDH;
        can_payload[SIDL] = RXB0SIDL;
        can_payload[DLC] = RXB0DLC;
        can_payload[D0] = RXB0D0;
        can_payload[D1] = RXB0D1;
        can_payload[D2] = RXB0D2;
        can_payload[D3] = RXB0D3;
        can_payload[D4] = RXB0D4;
        can_payload[D5] = RXB0D5;
        can_payload[D6] = RXB0D6;
        can_payload[D7] = RXB0D7;
        
        RXB0FUL = 0;
        RXB0IF = 0;
        
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
// ECU1 SENSOR MODULE 
uint16_t get_speed(void)
{
    uint16_t speed;
    
    speed = read_adc(SPEED_ADC_CHANNEL);
    speed = speed/10.23;   // Scaling to 0-100
    
    return speed;
}

unsigned char get_gear_pos(void)
{
    static unsigned char gear = '0';
    unsigned char key;
    
    key = read_switches(STATE_CHANGE);
    
    if (key != 0xFF)
    {
        if (key == MK_SW1)   // Gear UP - Switch 1
        {
            if (gear < '5')
                gear++;
            else if (gear == '5')
                gear = '6';
        }
        else if (key == MK_SW2)  // Gear DOWN - Switch 2
        {
            if (gear > '1')
                gear--;
            else if (gear == '1')
                gear = '0';
        }
    }
    
    return gear;
}

void delay(unsigned short factor)
{
    unsigned short i, j;
    for (i = 0; i < factor; i++)
    {
        for (j = 500; j--; );
    }
}
// MAIN 
void init_config(void)
{
    /* Initialize Matrix Keypad */
    init_matrix_keypad();
    
    /* Initialize CLCD module */
    init_clcd();
    
    /* Initialize ADC module */
    init_adc();
    
    /* Initialize CAN module */
    init_can();
    
    /* Display initial message */
    clcd_print("CAN Loopback", LINE1(2));
    delay(1000);
}

void main(void)
{
    unsigned int speed = 0;
    unsigned char gear = '0';
    unsigned char buf[16];
    
    init_config();
    
    /* Clear and show application labels */
    clcd_write_cmd(0x01);
    delay(10);
    clcd_print("Speed:     km/h", LINE1(0));
    clcd_print("Gear:          ", LINE2(0));
    
    while (1)
    {
        /* Get speed from ADC */
        speed = get_speed();
        
        /* Get gear position from keypad */
        gear = get_gear_pos();
        
        /* Prepare speed data for transmission */
        sprintf((char*)buf, "%03u", speed);
        
        /* Set speed data in CAN transmit buffer */
        TXB0EIDH = 0x00;
        TXB0EIDL = 0x00;
        TXB0SIDH = 0x6B;
        TXB0SIDL = 0xC0;
        TXB0DLC = 0x03;
        TXB0D0 = buf[0];
        TXB0D1 = buf[1];
        TXB0D2 = buf[2];
        TXB0D3 = 0;
        TXB0D4 = 0;
        TXB0D5 = 0;
        TXB0D6 = 0;
        TXB0D7 = 0;
        TXB0REQ = 1;
        
        delay(10);

        /* Prepare gear data for transmission */
        if (gear == '0')
            buf[0] = 'N';
        else if (gear == '6')
            buf[0] = 'R';
        else
            buf[0] = gear;
        
        /* Set gear data in CAN transmit buffer */
        TXB0EIDH = 0x00;
        TXB0EIDL = 0x00;
        TXB0SIDH = 0x6B;
        TXB0SIDL = 0xC0;
        TXB0DLC = 0x01;
        TXB0D0 = buf[0];
        TXB0D1 = 0;
        TXB0D2 = 0;
        TXB0D3 = 0;
        TXB0D4 = 0;
        TXB0D5 = 0;
        TXB0D6 = 0;
        TXB0D7 = 0;
        TXB0REQ = 1;
        
        delay(100);
    }
}