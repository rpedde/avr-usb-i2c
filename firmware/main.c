#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "i2cmaster.h"

typedef signed char int8;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define DEBUG_ADDR 16

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

int i2c_send(uint8_t device, uint8_t addr, uint8_t byte);
int i2c_send_cmd(uint8_t device, uint8_t byte);

void usb_recv(uint8_t data, uint8_t len) {
}

static uchar buffer[64];
static uint8_t buffer_len;
static uchar currentPosition, bytesRemaining;

uchar usbFunctionWrite(uchar *data, uchar len)
{
    uchar i;
    uchar counter;

    if(len > bytesRemaining)                // if this is the last incomplete chunk
        len = bytesRemaining;               // limit to the amount we can store
    bytesRemaining -= len;
    for(i = 0; i < len; i++)
        buffer[currentPosition++] = data[i];

    if(bytesRemaining == 0) {
        switch(buffer[0]) {
        case CMD_READ_EEDATA:
            buffer[1] = buffer[2];
            buffer[0] = 0xea; // should be read value of offset buffer[1]
            buffer_len = 2;
            break;
        case CMD_WRITE_EEDATA:
            buffer[0] = 1; // true or false  [2] is offset, [3] is value
            buffer_len = 4;
            break;
        case CMD_READ_VERSION:
            buffer[0] = 2;
            buffer[1] = 0;
            buffer_len = 2;
            break;
        case CMD_BOARD_TYPE:
            buffer[0] = 2;  // i2c
            buffer[1] = 9;  // serial_no (should be from eeprom)
            buffer[2] = 2;  // avr ATmega168
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

            if(i2c_start((buffer[2] << 1) + I2C_WRITE)) {
                // probably noack.
                i2c_stop();
                buffer[0] = 0;
                buffer[1] = I2C_E_NODEV;
                break;
            }

            if(i2c_write(buffer[3])) {
                i2c_stop();
                buffer[0] = 0;
                buffer[1] = I2C_E_NOACK;
                break;
            }

            if(i2c_start((buffer[2] << 1) + I2C_READ)) {
                i2c_stop();
                buffer[0] = 0;
                buffer[1] = I2C_E_NODEV;
                break;
            }

            while(counter) {
                i = 1 + buffer[0] - counter;

                if(counter > 1) {
                    buffer[i] = i2c_readAck();
                } else {
                    buffer[i] = i2c_readNak();
                }

                counter--;
            }

            i2c_stop();
            buffer[0] = 1;
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

            if(i2c_start((buffer[2] << 1) + I2C_WRITE)) {
                // probably noack.
                i2c_stop();
                buffer[0] = 0;
                buffer[1] = I2C_E_NODEV;
                break;
            }

            if(i2c_write(buffer[3])) {
                i2c_stop();
                buffer[0] = 0;
                buffer[1] = I2C_E_NOACK;
                break;
            }

            while(counter) {
                i = 4 + buffer[0] - counter;
                if(i2c_write(buffer[i])) {
                    buffer[0] = 0;
                    buffer[1] = I2C_E_NOACK;
                }
                counter--;
            }

            i2c_stop();

            buffer[0] = 1;
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

int i2c_send_cmd(uint8_t device, uint8_t byte) {
    i2c_start_wait(device + I2C_WRITE);
    if(!i2c_write(0x41)) {
        _delay_ms(10);
        if(!i2c_write(byte)) {;
            _delay_ms(10);
        }
    }

    i2c_stop();
    _delay_ms(10);

    return 0;
}

int i2c_send(uint8_t device, uint8_t addr, uint8_t byte) {
    i2c_start_wait(device + I2C_WRITE);
    if(!i2c_write(addr)) {
        _delay_ms(10);
        if(!i2c_write(0x30 | (byte >> 4))) {
            _delay_ms(10);
            if(!i2c_write(0x30 | (byte & 0x0F))) {
                _delay_ms(10);
                i2c_write(' ');
            }
        }
    }

    _delay_ms(10);
    i2c_stop();
    _delay_ms(10);

    return 0;
}

/* ------------------------------------------------------------------------- */

int main(void) {
    uchar   i;

    DDRD |= (1U << 1);
    PORTD |= (1U << 1);
    i2c_init();
    _delay_ms(10);
    i2c_send_cmd(DEBUG_ADDR, 1);
    PORTD ^= (1U << 1);

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    wdt_disable();

    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    odDebugInit();
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */
    for(;;) {                /* main event loop */
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        wdt_reset();
        usbPoll();
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
