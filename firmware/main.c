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

#define CMD_READ_VERSION   0x00
#define CMD_READ_EEDATA    0x01
#define CMD_WRITE_EEDATA   0x02
#define CMD_BOARD_TYPE     0x30
#define CMD_BD_POWER_INFO  0x31
#define CMD_BD_POWER_STATE 0x32
#define CMD_I2C_READ       0x40
#define CMD_I2C_WRITE      0x41
#define CMD_RESET          0xFF

#define I2C_E_SUCCESS      0x00
#define I2C_E_NODEV        0x01
#define I2C_E_NOACK        0x02
#define I2C_E_TIMEOUT      0x03
#define I2C_E_OTHER        0x04

#define VENDOR_RQ_WRITE_BUFFER 0x00
#define VENDOR_RQ_READ_BUFFER  0x01

static uchar buffer[64];
static uint8_t buffer_len;
static uchar currentPosition, bytesRemaining;

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
            buffer[1] = buffer[2];

            buffer[0] = eeprom_read_byte((uint8_t*)(uint16_t)buffer[1]);
            buffer_len = 2;
            break;
        case CMD_WRITE_EEDATA:
            buffer[0] = 1; // true or false  [2] is offset, [3] is value
            eeprom_write_byte((uint8_t*)(uint16_t)buffer[2], buffer[3]);
            buffer_len = 4;
            break;
        case CMD_READ_VERSION:
            buffer[0] = FIRMWARE_MAJOR;
            buffer[1] = FIRMWARE_MINOR;
            buffer_len = 2;
            break;
        case CMD_BOARD_TYPE:
            buffer[0] = BOARD_TYPE_I2C;
            temp = eeprom_read_byte((uint8_t*)1);
            if((temp == 0) || (temp == 255))
                temp = 9;

            buffer[1] = temp;  // read from EEPROM address 1

#ifdef __AVR_ATmega88__
            buffer[2] = PROCESSOR_TYPE_A88;
#elif defined __AVR_ATmega168__
            buffer[2] = PROCESSOR_TYPE_A168;
#else
#warn "UNKNOWN PROC TYPE!"
            buffer[2] = PROCESSOR_TYPE_UNKNOWN;
#endif

            buffer[3] = F_CPU/1000000;
            buffer_len = 4;
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
            counter = buffer[4];
            buffer[0] = counter;
            buffer_len = buffer[4] + 1;

            if((i2cMasterSendNI(buffer[2] << 1, 1, &buffer[3]) == I2C_OK) &&
               (i2cMasterReceiveNI(buffer[2] << 1, counter, &buffer[1]) == I2C_OK)) {
                    buffer[0] = 1;
            } else {
                buffer[0] = 0;
                buffer[1] = I2C_E_NODEV;
            }

            break;

        case CMD_I2C_WRITE:
            // 0 - I2C_WRITE
            // 1 - DATA_LEN
            // 2 - DEV
            // 3 - ADDR
            // 4 - BYTES_TO_READ

            buffer_len = 2;
            counter = buffer[1] - 2;
            buffer[0] = counter;

            if(i2cMasterSendNI(buffer[2] << 1, counter + 1, &buffer[3]) == I2C_OK) {
                buffer[0] = 1;
            } else {
                buffer[0] = 0;
                buffer[1] = I2C_E_NODEV;
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
