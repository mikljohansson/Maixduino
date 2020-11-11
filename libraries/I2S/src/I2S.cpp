#include "I2S.h"
#include "pins_arduino.h"

I2SClass::I2SClass(i2s_device_number_t device_num, uint8_t sd, uint8_t bck, uint8_t ws)
    : _i2s_num(device_num), _sdPin(sd), _sckPin(bck), _fsPin(ws), 
    _transmit((i2s_transmit_t)-1), _bitsPerSample(0), 
    _dmaChannel(DMAC_CHANNEL0), _dmaTransferInProgress(false),
    _onTransmit(0), _onReceive(0) {}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample) {
    return begin(I2S_TRANSMITTER, mode, sampleRate, bitsPerSample);
}

int I2SClass::begin(int mode, int bitsPerSample) {
    return begin(I2S_RECEIVER, mode, 0, bitsPerSample);
}

int I2SClass::begin(i2s_transmit_t transmit, int mode, long sampleRate, int bitsPerSample) {
    i2s_work_mode_t i2s_mode;
    i2s_word_length_t i2s_resolution;
    i2s_word_select_cycles_t i2s_cycles;

    switch (mode) {
        case I2S_PHILIPS_MODE:
            i2s_mode = STANDARD_MODE;
            break;

        case I2S_RIGHT_JUSTIFIED_MODE:
            i2s_mode = RIGHT_JUSTIFYING_MODE;
            break;

        case I2S_LEFT_JUSTIFIED_MODE:
            i2s_mode = LEFT_JUSTIFYING_MODE;
            break;

        default:
            // invalid mode
            return 0;
    }

    switch (bitsPerSample) {
        case 12:
            i2s_resolution = RESOLUTION_12_BIT;
            i2s_cycles = SCLK_CYCLES_16;
            break;

        case 16:
            i2s_resolution = RESOLUTION_16_BIT;
            i2s_cycles = SCLK_CYCLES_16;
            break;

        case 20:
            i2s_resolution = RESOLUTION_20_BIT;
            i2s_cycles = SCLK_CYCLES_32;
            break;

        case 24:
            i2s_resolution = RESOLUTION_24_BIT;
            i2s_cycles = SCLK_CYCLES_32;
            break;

        case 32:
            i2s_resolution = RESOLUTION_32_BIT;
            i2s_cycles = SCLK_CYCLES_32;
            break;

        default:
            // invalid bits per sample
            return 0;
    }

    // Initialize I2S device and enable 1 channel with left/right
    switch (transmit) {
        case I2S_TRANSMITTER:
            fpioa_set_function(_sdPin, FUNC_I2S0_OUT_D0);
            fpioa_set_function(_sckPin, FUNC_I2S0_SCLK);
            fpioa_set_function(_fsPin, FUNC_I2S0_WS);
            
            i2s_init(_i2s_num, transmit, 0x3);
            i2s_tx_channel_config(_i2s_num, I2S_CHANNEL_0, i2s_resolution, i2s_cycles, TRIGGER_LEVEL_4, i2s_mode);
            break;

        case I2S_RECEIVER:
            fpioa_set_function(_sdPin, FUNC_I2S0_IN_D0);
            fpioa_set_function(_sckPin, FUNC_I2S0_SCLK);
            fpioa_set_function(_fsPin, FUNC_I2S0_WS);

            i2s_init(_i2s_num, transmit, 0x3);
            i2s_rx_channel_config(_i2s_num, I2S_CHANNEL_0, i2s_resolution, i2s_cycles, TRIGGER_LEVEL_4, i2s_mode);
            break;

        default:
            // invalid transmit mode
            return 0;
    }
    _transmit = transmit;
    _bitsPerSample = bitsPerSample;

    // Setup clock
    if (sampleRate) {
        i2s_set_sample_rate(_i2s_num, sampleRate);
    }

    // Divide the 32 bit samples into left and right channels
    if (bitsPerSample <= 16 && i2s_set_dma_divide_16(_i2s_num, 1)) {
        return 0;
    }

    // Receive callbacks when DMA transfers complete
    dmac_channel_enable(_dmaChannel);
    dmac_set_irq(_dmaChannel, &I2SClass::onTransferCompleteHandler, this, 0);
    
    return 1;
}

// copied from Kendryte SDK i2s.c
static void i2s_disable_block(i2s_device_number_t device_num, i2s_transmit_t rxtx_mode) {
    irer_t u_irer;
    iter_t u_iter;

    if (rxtx_mode == I2S_RECEIVER)     {
        u_irer.reg_data = readl(&i2s[device_num]->irer);
        u_irer.irer.rxen = 0;
        writel(u_irer.reg_data, &i2s[device_num]->irer);
        /* Receiver block disable */
    }
    else     {
        u_iter.reg_data = readl(&i2s[device_num]->iter);
        u_iter.iter.txen = 0;
        writel(u_iter.reg_data, &i2s[device_num]->iter);
        /* Transmitter block disable */
    }
}

void I2SClass::end() {
    // Disable DMA channel
    dmac_wait_done(_dmaChannel);
    dmac_disable_channel_interrupt(_dmaChannel);
    dmac_channel_disable(_dmaChannel);

    // Disable I2S device
    i2s_disable_block(_i2s_num, I2S_TRANSMITTER);
    i2s_disable_block(_i2s_num, I2S_RECEIVER);

    // Set the pins back to input mode
    pinMode(_sdPin, INPUT);
    pinMode(_fsPin, INPUT);
    pinMode(_sckPin, INPUT);
}

int I2SClass::available() {
    if (_transmit != I2S_RECEIVER) {
        return 0;
    }

    tryDmaReceive();
    return _doubleBuffer.available();
}

int I2SClass::read() {
    if (_transmit != I2S_RECEIVER) {
        return 0;
    }

    tryDmaReceive();

    if (_bitsPerSample <= 16) {
        int16_t sample = 0;
        _doubleBuffer.read(&sample, sizeof(sample));
        return sample;
    }
    
    int32_t sample = 0;
    _doubleBuffer.read(&sample, sizeof(sample));
    return sample;
}

int I2SClass::peek() {
    if (_transmit != I2S_RECEIVER) {
        return 0;
    }

    tryDmaReceive();
    
    if (_bitsPerSample <= 16) {
        int16_t sample = 0;
        _doubleBuffer.peek(&sample, sizeof(sample));
        return sample;
    }
    
    int32_t sample = 0;
    _doubleBuffer.peek(&sample, sizeof(sample));
    return sample;
}

void I2SClass::flush() {
    // do nothing, writes are DMA triggered
}

size_t I2SClass::write(uint8_t data) {
    return write((int32_t)data);
}

size_t I2SClass::write(const uint8_t* buffer, size_t size) {
    return write((const void*)buffer, size);
}

int I2SClass::availableForWrite() {
    return _doubleBuffer.availableForWrite();
}

int I2SClass::read(void* buffer, size_t size) {
    tryDmaReceive();
    return _doubleBuffer.read(buffer, size);
}

int i2s_send_data(i2s_device_number_t device_num, i2s_channel_num_t channel_num, const uint8_t *pcm, size_t buf_len,
                  size_t single_length)
{
    isr_t u_isr;
    uint32_t left_buffer = 0;
    uint32_t right_buffer = 0;
    uint32_t i = 0;
    uint32_t j = 0;
    if(channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
        return -1;

    buf_len = buf_len / (single_length / 8) / 2; /* sample num */
    readl(&i2s[device_num]->channel[channel_num].tor);
    /* read clear overrun flag */

    for(j = 0; j < buf_len;)
    {
        u_isr.reg_data = readl(&i2s[device_num]->channel[channel_num].isr);
        if(u_isr.isr.txfe == 1)
        {
            switch(single_length)
            {
                case 16:
                    left_buffer = ((uint16_t *)pcm)[i++];
                    right_buffer = ((uint16_t *)pcm)[i++];
                    break;
                case 24:
                    left_buffer = 0;
                    left_buffer |= pcm[i++];
                    left_buffer |= pcm[i++] << 8;
                    left_buffer |= pcm[i++] << 16;
                    right_buffer = 0;
                    right_buffer |= pcm[i++];
                    right_buffer |= pcm[i++] << 8;
                    right_buffer |= pcm[i++] << 16;
                    break;
                case 32:
                    left_buffer = ((uint32_t *)pcm)[i++];
                    right_buffer = ((uint32_t *)pcm)[i++];
                    break;
                default:
                    left_buffer = pcm[i++];
                    right_buffer = pcm[i++];
                    break;
            }
            writel(left_buffer, &i2s[device_num]->channel[channel_num].left_rxtx);
            writel(right_buffer, &i2s[device_num]->channel[channel_num].right_rxtx);
            j++;
        }
    }
    return 0;
}

size_t I2SClass::write(int32_t sample) {
    uint16_t pcm[2];
    pcm[0] = sample;
    pcm[1] = sample;
    i2s_send_data(_i2s_num, I2S_CHANNEL_0, (uint8_t *)pcm, sizeof(pcm), 16);
}

size_t I2SClass::write(const void* buffer, size_t size) {
    if (_transmit != I2S_TRANSMITTER) {
        return 0;
    }
    
    size_t written_bytes = _doubleBuffer.write(buffer, size);
    tryDmaTransmit();
    return written_bytes;
}

void I2SClass::onTransmit(void(*function)(void)) {
    _onTransmit = function;
}

void I2SClass::onReceive(void(*function)(void)) {
    _onReceive = function;
}

void I2SClass::tryDmaTransmit() {
    // Avoid starting another transfer until the previous one has completed and there's output available
    if (!_dmaTransferInProgress && _doubleBuffer.available()) {
        // output is available to transfer, start the DMA process for the current buffer
        _dmaTransferInProgress = true;
        i2s_send_data_dma(_i2s_num, _doubleBuffer.data(), _doubleBuffer.available(), _dmaChannel);

        // swap to the next user buffer for writes
        _doubleBuffer.swap();
    }
}

void I2SClass::tryDmaReceive() {
    // Avoid starting another transfer until the previous one has completed and the user has read all the previous data
    if (!_dmaTransferInProgress && _doubleBuffer.available() == 0) {
        // the user has read all the current input, start the DMA process to fill the current buffer again
        _dmaTransferInProgress = true;
        i2s_receive_data_dma(_i2s_num, (uint32_t *)_doubleBuffer.data(), _doubleBuffer.availableForWrite(), _dmaChannel);

        // swap to the next buffer that has previously been filled, so that the user can read it
        _doubleBuffer.swap(_doubleBuffer.availableForWrite());
    }
}

void I2SClass::onTransferComplete(void) {
    _dmaTransferInProgress = false;

    if (_transmit == I2S_TRANSMITTER) {
        // transmit complete, try starting another transfer
        tryDmaTransmit();

        // call the users transmit callback if provided
        if (_onTransmit) {
            _onTransmit();
        }
    }
    else if (_transmit == I2S_RECEIVER) {
        // receive complete, try starting another transfer
        tryDmaReceive();

        // call the users receveive callback if provided
        if (_onReceive) {
            _onReceive();
        }
    }
}

int I2SClass::onTransferCompleteHandler(void *ctx) {
    ((I2SClass *)ctx)->onTransferComplete();
    return 0;
}

// Create Arduino's default I2S device using board specific pins
I2SClass I2S = I2SClass(I2S_DEVICE_0, I2S_DA, I2S_BCK, I2S_WS);
