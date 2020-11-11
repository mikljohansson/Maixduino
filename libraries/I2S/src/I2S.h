#ifndef _MAIXDUINO_I2S_H_
#define _MAIXDUINO_I2S_H_

#include <Arduino.h>
#include <Stream.h>
#include <fpioa.h>
#include <dmac.h>
#include <i2s.h>
#include <pins_arduino.h>

#include "I2SDoubleBuffer.h"

typedef enum {
    I2S_PHILIPS_MODE,
    I2S_RIGHT_JUSTIFIED_MODE,
    I2S_LEFT_JUSTIFIED_MODE
} i2s_mode_t;

class I2SClass : public Stream {
public:
    I2SClass(i2s_device_number_t device_num, uint8_t sd, uint8_t bck, uint8_t ws);

    // the SCK and FS pins are driven as outputs using the sample rate
    int begin(int mode, long sampleRate, int bitsPerSample);
    // the SCK and FS pins are inputs, other side controls sample rate
    int begin(int mode, int bitsPerSample);
    void end();

    // from Stream
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

    // from Print
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t* buffer, size_t size);

    virtual int availableForWrite();

    int read(void* buffer, size_t size);

    size_t write(int32_t sample);
    size_t write(const void* buffer, size_t size);

    void onTransmit(void(*)(void));
    void onReceive(void(*)(void));

private:
    int begin(i2s_transmit_t transmit, int mode, long sampleRate, int bitsPerSample);

    void tryDmaTransmit();
    void tryDmaReceive();
    void onTransferComplete();

    static int onTransferCompleteHandler(void *ctx);

private:
    i2s_device_number_t _i2s_num;
    uint8_t _sdPin;
    uint8_t _sckPin;
    uint8_t _fsPin;

    i2s_transmit_t _transmit;
    uint8_t _bitsPerSample;

    I2SDoubleBuffer _doubleBuffer;

    dmac_channel_number_t _dmaChannel;
    volatile bool _dmaTransferInProgress;

    void (*_onTransmit)(void);
    void (*_onReceive)(void);
};

#ifdef I2S_DA
extern I2SClass I2S;
#else
#error "I2S is not supported on your board!"
#endif

#endif //_MAIXDUINO_I2S_H_