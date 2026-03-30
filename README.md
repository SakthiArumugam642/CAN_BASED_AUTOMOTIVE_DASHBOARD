Here's a full README for this CAN-based automotive dashboard project:
  ::view-transition-group(*),
  ::view-transition-old(*),
  ::view-transition-new(*) {
    animation-duration: 0.25s;
    animation-timing-function: cubic-bezier(0.19, 1, 0.22, 1);
  }
VvisualizeVvisualize show_widget
CAN-Based Automotive Dashboard
A multi-ECU automotive dashboard system built on PIC18F microcontrollers, communicating over a CAN bus. Three ECUs work together — two sensor nodes send data, and one display node renders everything on an LCD with indicator LEDs.

System Overview
The system has three ECUs connected on a shared CAN bus running at 500 kbps:
ECU1 reads engine RPM from an ADC-connected potentiometer (simulating a sensor) and indicator switch state from a 3×4 matrix keypad. It transmits two CAN messages on ID 0x36E: a 4-byte RPM string and a 3-byte indicator direction string.
ECU2 reads vehicle speed (ADC) and gear position, transmitting on ID 0x35E — a 3-byte speed string and a 1-byte gear character.
ECU3 is the dashboard display unit. It receives both message streams, decodes them by ID and DLC, updates a 16×2 character LCD, and drives two LEDs for left/right turn indicators with blinking logic.

Hardware
ComponentDetailMicrocontrollerPIC18F series (all 3 ECUs)Clock20 MHz external crystalCAN transceiverConnected on RB2 (TX) / RB3 (RX)LCD16×2 character LCD on PORTD, RS/RW/EN on RC1/RC0/RC2Matrix keypad3×4 on PORTB (rows RB5–RB7, cols RB1–RB4)ADC inputChannel 4 (AN4) for RPM / speed sensingIndicator LEDsRB0 (left), RB7 (right)

CAN Bus Configuration
All three ECUs use identical CAN timing for 500 kbps at 8 MHz (post-prescaler):
BRGCON1 = 0xE1   // SJW = 4 TQ, BRP = 4
BRGCON2 = 0x1B   // PS1 = 4 TQ, Propagation = 4 TQ
BRGCON3 = 0x03   // PS2 = 4 TQ
Message structure:
Msg IDSourceDLCContent0x36EECU14RPM as ASCII string, e.g. "1500"0x36EECU13Indicator: "<--", "-->", "<->", or "   "0x35EECU23Speed as ASCII string, e.g. "060"0x35EECU21Gear as single char: 'N', '1'–'6'
ECU3 uses CAN filters to selectively receive both IDs. Message type is disambiguated by DLC (data length code).

LCD Layout (ECU3)
Line 1:  S:060 km/h G:3
Line 2:  R:1500 rpm I:<--

Speed and gear are updated from ECU2 messages (0x35E)
RPM and indicator symbol are updated from ECU1 messages (0x36E)


Indicator LED Logic
ECU3 maintains a blink_counter and toggles the appropriate LED(s) every 5 loop cycles:
Indicator stateLED behavior<-- (left)RB0 blinks, RB7 off--> (right)RB7 blinks, RB0 off<-> (hazard)Both blink together    (off)Both off

RPM Calculation (ECU1)
The ADC reads 10 samples from CH4, averages them, then scales to a 0–6000 RPM range:
crpm = (adc_avg / 10.23) * 60;
```

This maps the 10-bit ADC range (0–1023) to 0–6000 RPM.

---

## Project Structure
```
project/
├── ECU1/main.c     — RPM + indicator transmitter
├── ECU2/main.c     — Speed + gear transmitter
└── ECU3/main.c     — Dashboard receiver + display
All three files share the same peripheral drivers (ADC, LCD, CAN, keypad) with ECU-specific sensor and display logic in main().
<img width="1440" height="678" alt="image" src="https://github.com/user-attachments/assets/98553c26-d2fa-40d8-bb98-072e0e512fc5" />
