/*
    usb2lds.c

    Copyright 2014 - Nicolas "Xevel" Saugnier

    Based on:
    - LUFA library, by Dean Camera
    - USB2AX, By Nicolas Saugnier and Richard Ibbotson  
*/
/*
  Original notice:

  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the usb2lds firmware. It does the hardware initialization, reads data from one end and sends it to the other.
 */

#include "usb2lds.h"
#include "reset.h"
#include <nlds/nlds.h>
#include "time.h"

// Pin wired:
//   USART RX   (PD2/RXD1)
//   USART TX   (PD3/TXD1)
//   MOT PWM    (PB7/OC0A) inverted logic
//   TX LED     (PD0)
//   MISO       PB3
//   MOSI       PB2
//   SCK        PB1
//   !SS        PB0


USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = 0,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

static FILE USBSerialStream;

static RingBuffer_t USBtoUSART_Buffer;
static uint8_t      USBtoUSART_Buffer_Data[128];
static RingBuffer_t USARTtoUSB_Buffer;
static uint8_t      USARTtoUSB_Buffer_Data[128];

uint8_t needs_bootload = false; // In EVENT_CDC_Device_LineEncodingChanged, this flag is set when the baudrate is at a pre-defined value
uint8_t setup_mode;
uint8_t display_mode;

#define DISPLAY_NONE 0
#define DISPLAY_GRAPH 1
#define DISPLAY_VALUES 2

uint16_t rpm;
uint16_t last_rpm;
uint8_t has_new_rpm_value;
uint8_t mot_should_run;
uint32_t last_rpm_timestamp;

int16_t rpm_setpoint; //speed is expressed in 64th of rpm (6 bit shift) and we want it to turn at 300 rpm (= 5 rotation/s)
uint8_t motor_pwm;

uint8_t feedforward_fixed;
double feedforward_coeff;
uint8_t feedforward_mode;

#define FEEDFORWARD_NONE            (0)
#define FEEDFORWARD_FIXED           (1)
#define FEEDFORWARD_PROPORTIONAL    (2)

double Kp;
double Ki;
double Kd;
double err_acc;
double last_err;
double delta;

// assign 4 byte for each, even the ones that are smaller than that in case we want to change their size later without having to chang all the other ones.
#define EE_ADDR_KP          (   (float*)32)
#define EE_ADDR_KI          (   (float*)36)
#define EE_ADDR_KD          (   (float*)40)
#define EE_ADDR_SETPOINT    ((uint16_t*)44)
#define EE_ADDR_DELTA       (   (float*)48)
#define EE_ADDR_FF_MODE     ( (uint8_t*)52)
#define EE_ADDR_FF_FIXED    ( (uint8_t*)56)
#define EE_ADDR_FF_COEFF    (   (float*)60)
#define EE_ADDR_MAGIC       ((uint32_t*)64)
uint32_t eeprom_magic_value = 0xbadbeef;

void save_eeprom(){
    eeprom_update_float(EE_ADDR_KP, Kp);
    eeprom_update_float(EE_ADDR_KI, Ki);
    eeprom_update_float(EE_ADDR_KD, Kd);
    eeprom_update_word(EE_ADDR_SETPOINT, rpm_setpoint);
    eeprom_update_float(EE_ADDR_DELTA, delta);
    eeprom_update_byte(EE_ADDR_FF_MODE, feedforward_mode);
    eeprom_update_byte(EE_ADDR_FF_FIXED, feedforward_fixed);
    eeprom_update_float(EE_ADDR_FF_COEFF, feedforward_coeff);
    eeprom_update_dword(EE_ADDR_MAGIC, eeprom_magic_value);
}

void load_default(){
    Kp = 0.05;
    Ki = 0.0001;
    Kd = 0.0001;
    rpm_setpoint = 19200; // = 300rpm * 64
    delta = 1.0;
    feedforward_fixed = 190;
    feedforward_coeff = 100.0;
    feedforward_mode = FEEDFORWARD_PROPORTIONAL;
}

void load_eeprom(){
    Kp = eeprom_read_float(EE_ADDR_KP);
    Ki = eeprom_read_float(EE_ADDR_KI);
    Kd = eeprom_read_float(EE_ADDR_KD);
    rpm_setpoint = eeprom_read_word(EE_ADDR_SETPOINT);
    delta = eeprom_read_float(EE_ADDR_DELTA);
    feedforward_fixed = eeprom_read_byte(EE_ADDR_FF_FIXED);
    feedforward_coeff = eeprom_read_float(EE_ADDR_FF_COEFF);
    feedforward_mode = eeprom_read_byte(EE_ADDR_FF_MODE);
}

void show_pid(){    
    fprintf(&USBSerialStream, "Setpoint=%u (%.2frpm, %.2frps)\r\nKp=%.5f, Ki=%.5f, Kd=%.5f [delta=%.5f]\r\n",
        rpm_setpoint,
        (double)rpm_setpoint/64,
        (double)rpm_setpoint/3840,
        Kp, Ki, Kd,
        delta);
}


void show_display(){
    switch (display_mode)
    {
        case DISPLAY_VALUES:
            fprintf(&USBSerialStream, "Display Values [rpm, error, output before clamping]\r\n");
            break;
        case DISPLAY_GRAPH:
            fprintf(&USBSerialStream, "Display Graph (not implemented yet)\r\n");
            break;
        default:
            fprintf(&USBSerialStream, "Display None\r\n");
            break;
    }

}

void show_feedforward(){
    fprintf(&USBSerialStream, "Feedforward fixed = %u\r\n", feedforward_fixed);
    fprintf(&USBSerialStream, "Feedforward proportional = %u/%.2f = %.2f\r\n", rpm_setpoint, feedforward_coeff, (double)rpm_setpoint/feedforward_coeff);
    fprintf(&USBSerialStream, "Feedforward current mode: ");
    switch (feedforward_mode)
    {
        case FEEDFORWARD_FIXED:
            fprintf(&USBSerialStream, "FIXED\r\n");
            break;
        case FEEDFORWARD_PROPORTIONAL:
            fprintf(&USBSerialStream, "PROPORTIONAL\r\n");
            break;
        default:
            fprintf(&USBSerialStream, "NONE\r\n");
            break;
    }
}

void show_motor(){
    if (mot_should_run){
        fprintf(&USBSerialStream, "Motor ON\r\n");
    } else {
        fprintf(&USBSerialStream, "Motor OFF\r\n");
    }
}

void eeprom_init(){
    if (eeprom_read_dword(EE_ADDR_MAGIC) != eeprom_magic_value){
        load_default();
    } else {
        load_eeprom();
    }
}

int main(void)
{
	SetupHardware();
    eeprom_init();
    motor_pwm = 190;

    RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
    RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

	// Create a regular character stream for the interface so that it can be used with the stdio.h functions
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	GlobalInterruptEnable();

	for (;;)
	{
        if (setup_mode){
            int16_t c = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
            if (!(c < 0)){
                switch(c){
                    case 'L': // reset
                        fprintf(&USBSerialStream, "Load default values\r\n");
                        load_default();
                    case 'a':
                        show_pid();
                        show_feedforward();
                        show_motor();
                        show_display();
                        break;
                    //PID coef adjust
                    case 'p':
                        Kp -= delta;
                        if (Kp < 0){
                            Kp = 0;
                        }
                        show_pid();
                        break;
                    case 'P':
                        Kp += delta;
                        show_pid();
                        break;
                    case 'i':
                        Ki -= delta;
                        if (Ki < 0){
                            Ki = 0;
                        }
                        show_pid();
                        break;
                    case 'I':
                        Ki += delta;
                        show_pid();
                        break;
                    case 'd':
                        Kd -= delta;
                        if (Kd < 0){
                            Kd = 0;
                        }
                        show_pid();
                        break;
                    case 'D':
                        Kd += delta;
                        show_pid();
                        break;
                    case '+':
                        delta *= 10.0;
                        show_pid();
                        break;
                    case '-':
                        delta /= 10.0;
                        show_pid();
                        break;
                    case 'r':
                        rpm_setpoint -= 64;
                        show_pid();
                        break;
                    case 'R':
                        rpm_setpoint += 64;
                        show_pid();
                        break;

                    case '0':
                        feedforward_mode = FEEDFORWARD_NONE;
                        show_feedforward();
                        break;
                    case '1':
                        feedforward_mode = FEEDFORWARD_FIXED;
                        show_feedforward();
                        break;
                    case '2':
                        feedforward_mode = FEEDFORWARD_PROPORTIONAL;
                        show_feedforward();
                        break;
                        
                    case 'f':
                        feedforward_fixed -= 1;
                        show_feedforward();
                        break;
                    case 'F':
                        feedforward_fixed += 1;
                        show_feedforward();
                        break;

                    case 'c':
                        feedforward_coeff -= delta;
                        show_feedforward();
                        break;
                    case 'C':
                        feedforward_coeff += delta;
                        show_feedforward();
                        break;

                    case 's': // save
                        save_eeprom();
                        fprintf(&USBSerialStream, "Values saved to EEPROM\r\n");
                        break;
                    
                    case 'm':
                        mot_should_run = 0;
                        show_motor();
                        break;
                    case 'M':
                        mot_should_run = 1;
                        show_motor();
                        break;
                    case 'g':
                        display_mode = DISPLAY_NONE;
                        show_display();
                        break;
                    case 'G':
                        display_mode = DISPLAY_GRAPH;
                        show_display();
                        break;
                    case 'v':
                        display_mode = DISPLAY_VALUES;
                        show_display();
                        break;


                    case ' ': // to make the motor spin whatever the computations
                        motor_pwm = 200;
                        break;
                }
            }


        } else {
            //Copy data from USB to the LDS buffer, if there is room.
            if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
            {
                int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
                if (!(ReceivedByte < 0)){
                    RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
                }
            }

            //Send data from the LDS to USB
            uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
            if (BufferCount)
            {
                Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

                if (Endpoint_IsINReady())
                {
                    // Never send more than one bank size less one byte to the host at a time, so that we don't block
                    // while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening
                    uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));
                    while (BytesToSend--)
                    {
                        if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
                        RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
                        {
                            break;
                        }

                        // Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
                        RingBuffer_Remove(&USARTtoUSB_Buffer);
                    }
                }
            }
            
            // Load the next byte from the USART transmit buffer into the USART
            if (!(RingBuffer_IsEmpty(&USBtoUSART_Buffer))){
                Serial_SendByte(RingBuffer_Remove(&USBtoUSART_Buffer));
            }
        }
        
		
        uint16_t dt = 1; // init just to avoid potential warning

        cli();
        if (nlds_rpm_updated_get()){
            rpm = nlds_rpm_get();
            nlds_rpm_updated_clear();
            has_new_rpm_value = 1;
            dt = micros_reset();
        }
        sei();

        // tx_led_toggle(); // TODO flash the led when stuff happens
        
        // motor control
        if (has_new_rpm_value && rpm!=last_rpm){
            has_new_rpm_value = 0;
            double err = (double)rpm_setpoint - rpm;

            double fdt = (double)dt/1000.0; // get back to something closer to 1 to avoid orders of magnitude of difference in the coeffs
            
            err_acc += err;

            double output = Kp * err + Kd * (last_err-err)/fdt + Ki * err_acc*fdt;
            
            switch(feedforward_mode){
                case FEEDFORWARD_FIXED:
                    output += feedforward_fixed;
                    break;
                case FEEDFORWARD_PROPORTIONAL:
                    output += (double)rpm_setpoint / feedforward_coeff; 
                    break;
                case FEEDFORWARD_NONE:
                default:
                    break;
            }
            
            last_err = err;
            last_rpm = rpm;

            if (setup_mode){
                switch(display_mode){
                    case DISPLAY_GRAPH:
                        // display on 80 columns the output and setpoint

                        //TODO
                        
                        break;
                    case DISPLAY_VALUES:
                        fprintf(&USBSerialStream, "%u %f %f\r\n", rpm, err, output);
                        break;
                    case DISPLAY_NONE:
                    default:
                        break;

                }
            }

            if (output > 255){
                output = 255;
            } else if (output < 0){
                output = 0;
            }

            motor_pwm = (uint8_t) output;

        }
        if ( mot_should_run ){
            apply_motor_pwm(motor_pwm);
        } else {
            apply_motor_pwm(0);
        }

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}


void usart_init(){ // Usart at 115200, 8N1
    cli();
    UCSR1A = (1<<U2X1); // x2
    UBRR1L = 16; // cf datasheet p175
    
    // enable TX, RX and RX interrupt
    UCSR1B =  (1<<RXCIE1) | (1 << RXEN1) | (1 << TXEN1); 
    sei();
}

void motor_init(){
    // Setup motor PWM (PD5/OC0B), inverting logic, phase correct (mode 1)
    TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (0 << WGM01) | (1 << WGM00);
    TCCR0B = (0 << WGM02) | (1 << CS02) | (0 << CS01) | (0 << CS00);
    apply_motor_pwm(0);
    bitSet(DDRB, 7);
}

void apply_motor_pwm(uint8_t val){
    OCR0A = val;
}

void tx_led_init(){
    tx_led_off();
    bitSet(DDRD, 0);
}
void tx_led_on(){
    bitClear(PORTD, 0);
}
void tx_led_off(){
    bitSet(PORTD, 0);
}
void tx_led_toggle(){
    bitSet(PIND, 0);
}

void SetupHardware(void)
{
	// Disable watchdog if enabled by bootloader/fuses
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

    clock_prescale_set(clock_div_1);

    micros_init();
    USB_Init();
    usart_init();
    tx_led_init();
    motor_init();
}


ISR(USART1_RX_vect)
{
    uint8_t c = UDR1;
    if (USB_DeviceState == DEVICE_STATE_Configured){
        RingBuffer_Insert(&USARTtoUSB_Buffer, c);
    }
    sei(); // once incoming data has been read, we can release the interrupts because nlds_parse should not be too long, and other timers need to be able to tick
    
    nlds_parse(c);
}


void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo){
    // we don't allow changing the baud rate or any other details, things are already set up the right way to talk to the LDS.

    // If the baudrate is at this special (and unlikely) value, it means that we want to trigger the bootloader
    if (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 1200){
        needs_bootload = true;
        setup_mode = true;
    } else if ( CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 9600 || CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 1337){
        setup_mode = true;
    } else {
        setup_mode = false;
    }   
}

// *********************  Soft reset/bootloader functionality *******************************************
// The bootloader can be ran by opening the serial port at 1200bps. Upon closing it, the reset process will be initiated
// and two seconds later, the board will re-enumerate as a DFU bootloader.

// adapted from http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=1008792#1008792
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo){
    static bool PreviousDTRState = false;
    bool        CurrentDTRState  = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

    mot_should_run = CurrentDTRState;

    /* Check how the DTR line has been asserted */
    if (PreviousDTRState && !(CurrentDTRState) ){
        // PreviousDTRState == True AND CurrentDTRState == False
        // Host application has Disconnected from the COM port
        
        if (needs_bootload){
            apply_motor_pwm(0);
            Jump_To_Reset(true);
        }
    }
    PreviousDTRState = CurrentDTRState;
}
// ******************************************************************************************************


