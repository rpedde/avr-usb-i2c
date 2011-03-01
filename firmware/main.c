#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <string.h>
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include <avr/eeprom.h>
#include "usbdrv.h"
#include "i2c.h"
#include "timerx8.h"

#include <mpusb/mpusb.h>


#define FIRMWARE_MAJOR 2
#define FIRMWARE_MINOR 0

typedef signed char int8;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define EP_SIZE 64


#define VENDOR_RQ_WRITE_BUFFER 0x00
#define VENDOR_RQ_READ_BUFFER  0x01

static uchar buffer[64];
static uint8_t buffer_len;
static uchar currentPosition, bytesRemaining;

#define REQUEST(what) ((usb_cmd_##what##_t *)&buffer)
#define RESPONSE(what) ((usb_response_##what##_t *)&buffer)

uchar usbFunctionWrite(uchar *data, uchar len)
{
    uchar i;
    uchar counter;
    uint8_t temp;

    if(len > bytesRemaining)                // if this is the last incomplete chunk
        len = bytesRemaining;               // limit to the amount we can store
    bytesRemaining -= len;
    for(i = 0; i < len; i++)
        buffer[currentPosition++] = data[i];

    if(bytesRemaining == 0) {
        switch(buffer[0]) {
        case CMD_READ_EEDATA:
            RESPONSE(read_eedata)->addr = REQUEST(read_eedata)->addr;
            RESPONSE(read_eedata)->value = eeprom_read_byte((uint8_t*)(uint16_t)REQUEST(read_eedata)->addr);

            buffer_len = sizeof(usb_response_read_eedata_t);
            break;
        case CMD_WRITE_EEDATA:
            RESPONSE(write_eedata)->result = 1;
            eeprom_write_byte((uint8_t*)(uint16_t)REQUEST(write_eedata)->addr, REQUEST(write_eedata)->value);
            buffer_len = sizeof(usb_response_write_eedata_t);
            break;
        case CMD_READ_VERSION:
            RESPONSE(read_version)->version_major = FIRMWARE_MAJOR;
            RESPONSE(read_version)->version_minor = FIRMWARE_MINOR;
            buffer_len = sizeof(usb_response_read_version_t);;
            break;
        case CMD_BOARD_TYPE:
            RESPONSE(board_type)->board_type = BOARD_TYPE_I2C;
            temp = eeprom_read_byte((uint8_t*)1);
            if((temp == 0) || (temp == 255))
                temp = 9;

            RESPONSE(board_type)->serial = temp;  // read from EEPROM address 1

#ifdef __AVR_ATmega88__
            RESPONSE(board_type)->proc_type = PROCESSOR_TYPE_A88;
#elif defined __AVR_ATmega168__
            RESPONSE(board_type)->proc_type = PROCESSOR_TYPE_A168;
#else
#warn "UNKNOWN PROC TYPE!"
            RESPONSE(board_type)->proc_type = PROCESSOR_TYPE_UNKNOWN;
#endif

            RESPONSE(board_type)->mhz = F_CPU/1000000;
            buffer_len = sizeof(usb_response_board_type_t);
            break;
        case CMD_BD_POWER_STATE:
            buffer_len = 0;
            break;
        case CMD_BD_POWER_INFO:
            buffer_len = 0;
            break;
        case CMD_I2C_READ:
            // 0 - I2C_READ
            // 1 - DATA_LEN
            // 2 - DEV
            // 3 - ADDR
            // 4 - BYTES_TO_READ
            counter = REQUEST(i2c_read)->read_len;
            RESPONSE(i2c_read)->result = counter;
            buffer_len = REQUEST(i2c_read)->read_len + 1;

            if((i2cMasterSendNI(REQUEST(i2c_read)->device << 1, 1, &REQUEST(i2c_read)->address) == I2C_OK) &&
               (i2cMasterReceiveNI(REQUEST(i2c_read)->device << 1, counter, RESPONSE(i2c_read)->data) == I2C_OK)) {
                RESPONSE(i2c_read)->result = 1;
            } else {
                RESPONSE(i2c_read)->result = 0;
                *RESPONSE(i2c_read)->data = I2C_E_NODEV;
            }

            break;

        case CMD_I2C_WRITE:
            // 0 - I2C_WRITE
            // 1 - DATA_LEN
            // 2 - DEV
            // 3 - ADDR
            // 4 - BYTES_TO_READ

            buffer_len = sizeof(usb_response_i2c_write_t);

            counter = REQUEST(i2c_write)->len - 2;
            RESPONSE(i2c_write)->result = counter;

            if(i2cMasterSendNI(REQUEST(i2c_write)->device << 1, counter + 1, &REQUEST(i2c_write)->address) == I2C_OK) {
                RESPONSE(i2c_write)->result = 1;
            } else {
                RESPONSE(i2c_write)->result = 0;
                RESPONSE(i2c_write)->extended_result = I2C_E_NODEV;
            }
            break;

        case CMD_RESET:
            break;
        }
    }

    return bytesRemaining == 0;             // return 1 if we have all data
}


usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    usbRequest_t    *rq = (void *)data;
    usbMsgLen_t len;

    switch(rq->bRequest) {
    case VENDOR_RQ_WRITE_BUFFER:
        currentPosition = 0;                // initialize position index
        bytesRemaining = rq->wLength.word;  // store the amount of data requested
        if(bytesRemaining > sizeof(buffer)) // limit to buffer size
            bytesRemaining = sizeof(buffer);
        return USB_NO_MSG;        // tell driver to use usbFunctionWrite()

    case VENDOR_RQ_READ_BUFFER:
        len = 64;                           // we return up to 64 bytes
        if(len > rq->wLength.word)          // if the host requests less than we have
            len = rq->wLength.word;         // return only the amount requested
        usbMsgPtr = buffer;                 // tell driver where the buffer starts
        return len;                         // tell driver how many bytes to send
    }

    return 0;   /* default for not implemented requests: return no data back to host */
}


/* ------------------------------------------------------------------------- */

int main(void) {
    uchar   i;

    DDRC &= (~0x30);

    _delay_ms(20);

    DDRC |= (1U << 3);
    PORTC |= (1U << 3);

    i2cInit();
    i2cSetBitrate(10);

    _delay_ms(20);

    PORTC ^= (1U << 3);

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    wdt_disable();

    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    for(;;) {                /* main event loop */
        wdt_reset();
        usbPoll();
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
