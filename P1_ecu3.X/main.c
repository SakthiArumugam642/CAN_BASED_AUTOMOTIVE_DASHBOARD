/*
 * File:   main.c
 * Author: sakthi
 * CAN based automotive dashboard - ECU3 - Dashboard Display Unit
 * Receives RPM & Indicator from ECU1 (0x36E)
 * Receives Speed & Gear from ECU2 (0x35E)
 * Created on December 11, 2025
 */

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define _XTAL_FREQ 20000000

#define TRUE    1
#define FALSE   0

// LED Defines
#define LED_LEFT    RB0
#define LED_RIGHT   RB7

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

// Message IDs from ECU1 and ECU2
#define ECU1_MSG_ID    0x36E   // RPM & Indicator from ECU1
#define ECU2_MSG_ID    0x35E   // Speed & Gear from ECU2

// CAN Filter Configuration
#define ECU1_SIDH      0x6D    // For 0x36E
#define ECU1_SIDL      0xC0
#define ECU2_SIDH      0x6B    // For 0x35E
#define ECU2_SIDL      0xC0

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

// LED Module
void init_leds(void);
void update_indicator_leds(unsigned char ind_data[3]);

// Utility
void delay(unsigned short factor);

// ============================================================================
// LCD MODULE
// ============================================================================
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

// ============================================================================
// CAN MODULE
// ============================================================================
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
    
    /* Initialize CAN Timing - SAME AS ECU1 & ECU2 */
    BRGCON1 = 0xE1;     /* 1110 0001, SJW=4 TQ, BRP 4 */
    BRGCON2 = 0x1B;     /* 0001 1011, SEG2PHTS 1 sampled once PS1=4TQ PropagationT 4TQ */
    BRGCON3 = 0x03;     /* 0000 0011, PS2, 4TQ */
    
    /* Enable Filters - Filter 0 and Filter 1 */
    RXFCON0 = 0x03;     /* Enable Filter 0 and Filter 1 */
    
    /* Initialize Receive Filters */
    /* Filter 0 for ECU2 = 0x35E (Speed & Gear) */
    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;
    RXF0SIDH = ECU2_SIDH;
    RXF0SIDL = ECU2_SIDL;
    
    /* Filter 1 for ECU1 = 0x36E (RPM & Indicator) */
    RXF1EIDH = 0x00;
    RXF1EIDL = 0x00;
    RXF1SIDH = ECU1_SIDH;
    RXF1SIDL = ECU1_SIDL;
    
    /* Enter CAN module into Normal mode (not loopback) */
    CAN_SET_OPERATION_MODE_NO_WAIT(e_can_op_mode_normal);
    
    /* Wait until Normal mode is set */
    while ((CANSTAT & 0xE0) != 0x00);
    
    /* Set Receive Mode for buffers */
    RXB0CON = 0x00;     /* Receive all valid messages */
    RXB1CON = 0x00;     /* Receive all valid messages */
}

unsigned char can_receive(void)
{
    /* Check RXB0 first */
    if (RXB0FUL)
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
    /* Check RXB1 */
    else if (RXB1FUL)
    {
        can_payload[EIDH] = RXB1EIDH;
        can_payload[EIDL] = RXB1EIDL;
        can_payload[SIDH] = RXB1SIDH;
        can_payload[SIDL] = RXB1SIDL;
        can_payload[DLC] = RXB1DLC;
        can_payload[D0] = RXB1D0;
        can_payload[D1] = RXB1D1;
        can_payload[D2] = RXB1D2;
        can_payload[D3] = RXB1D3;
        can_payload[D4] = RXB1D4;
        can_payload[D5] = RXB1D5;
        can_payload[D6] = RXB1D6;
        can_payload[D7] = RXB1D7;
        
        RXB1FUL = 0;
        RXB1IF = 0;
        
        return TRUE;
    }
    
    return FALSE;
}

// ============================================================================
// LED MODULE
// ============================================================================
void init_leds(void)
{
    /* Configure RB0 and RB7 as outputs for indicator LEDs */
    TRISB0 = 0;     /* Left indicator */
    TRISB7 = 0;     /* Right indicator */
    
    /* Turn off both LEDs initially */
    LED_LEFT = 0;
    LED_RIGHT = 0;
}

void update_indicator_leds(unsigned char ind_data[3])
{
    static unsigned int blink_counter = 0;
    
    /* Determine indicator state from received data */
    if (ind_data[0] == '<' && ind_data[1] == '-' && ind_data[2] == '-')
    {
        /* Left indicator */
        if (blink_counter++ >= 5)
        {
            blink_counter = 0;
            LED_LEFT = !LED_LEFT;
            LED_RIGHT = 0;
        }
    }
    else if (ind_data[0] == '-' && ind_data[1] == '-' && ind_data[2] == '>')
    {
        /* Right indicator */
        if (blink_counter++ >= 5)
        {
            blink_counter = 0;
            LED_LEFT = 0;
            LED_RIGHT = !LED_RIGHT;
        }
    }
    else if (ind_data[0] == '<' && ind_data[1] == '-' && ind_data[2] == '>')
    {
        /* Hazard - both blink */
        if (blink_counter++ >= 5)
        {
            blink_counter = 0;
            LED_LEFT = !LED_LEFT;
            LED_RIGHT = !LED_RIGHT;
        }
    }
    else
    {
        /* Off */
        LED_LEFT = 0;
        LED_RIGHT = 0;
        blink_counter = 0;
    }
}

// ============================================================================
// UTILITY
// ============================================================================
void delay(unsigned short factor)
{
    unsigned short i, j;
    for (i = 0; i < factor; i++)
    {
        for (j = 500; j--; );
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void init_config(void)
{
    /* Initialize LEDs */
    init_leds();
    
    /* Initialize CLCD module */
    init_clcd();
    
      /* FIX: Configure PORTB before CAN init */
    ADCON1 = 0x0F;  // Set PORTB as digital
    
    /* Initialize CAN module */
    init_can();
    
    /* Display initial message */
    clcd_print("  DASHBOARD ", LINE1(2));
    clcd_print("Initializing...", LINE2(1));
    delay(1000);
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================
void main(void)
{
    unsigned char rpm_str[5] = "0000";
    unsigned char speed_str[4] = "000";
    unsigned char gear = 'N';
    unsigned char indicator[3] = {' ', ' ', ' '};
    unsigned int msg_id;
    
    init_config();
    
    /* Clear and show application labels */
    clcd_write_cmd(0x01);
    delay(10);
    
    /* Line 1: Speed and Gear */
    clcd_print("S:    km/h G:  ", LINE1(0));
    
    /* Line 2: RPM and Indicator */
    clcd_print("R:    rpm I:   ", LINE2(0));
    
    while (1)
    {

        
        unsigned char messages_processed = 0;
        
        
        for (messages_processed = 0; messages_processed < 4; messages_processed++)
        {
            if (can_receive())
            {
                /* Determine message ID from SIDH and SIDL */
                msg_id = ((unsigned int)can_payload[SIDH] << 3) | 
                         ((can_payload[SIDL] >> 5) & 0x07);
                
                /* Process based on message ID */
                if (msg_id == 0x35E)  /* ECU2: Speed & Gear */
                {
                    /* Check DLC to determine if it's Speed or Gear */
                    if (can_payload[DLC] == 3)
                    {
                        /* Speed data (3 bytes) */
                        speed_str[0] = can_payload[D0];
                        speed_str[1] = can_payload[D1];
                        speed_str[2] = can_payload[D2];
                        
                        /* Update Speed on LCD immediately */
                        clcd_putch(speed_str[0], LINE1(2));
                        clcd_putch(speed_str[1], LINE1(3));
                        clcd_putch(speed_str[2], LINE1(4));
                    }
                    else if (can_payload[DLC] == 1)
                    {
                        /* Gear data (1 byte) */
                        gear = can_payload[D0];
                        
                        /* Update Gear on LCD immediately */
                        clcd_putch(gear, LINE1(13));
                    }
                }
                else if (msg_id == 0x36E)  /* ECU1: RPM & Indicator */
                {
                    /* Check DLC to determine if it's RPM or Indicator */
                    if (can_payload[DLC] == 4)
                    {
                        /* RPM data (4 bytes) */
                        rpm_str[0] = can_payload[D0];
                        rpm_str[1] = can_payload[D1];
                        rpm_str[2] = can_payload[D2];
                        rpm_str[3] = can_payload[D3];
                        
                        /* Update RPM on LCD immediately */
                        clcd_putch(rpm_str[0], LINE2(2));
                        clcd_putch(rpm_str[1], LINE2(3));
                        clcd_putch(rpm_str[2], LINE2(4));
                        clcd_putch(rpm_str[3], LINE2(5));
                    }
                    else if (can_payload[DLC] == 3)
                    {
                        /* Indicator data (3 bytes) */
                        indicator[0] = can_payload[D0];
                        indicator[1] = can_payload[D1];
                        indicator[2] = can_payload[D2];
                        
                        /* Update Indicator on LCD immediately */
                        clcd_putch(indicator[0], LINE2(12));
                        clcd_putch(indicator[1], LINE2(13));
                        clcd_putch(indicator[2], LINE2(14));
                    }
                }
            }
            else
            {
               
                break;
            }
        }
        
        update_indicator_leds(indicator);
        
        
        delay(5);
        
    }
}