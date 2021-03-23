
#include "Sipeed_OV2640.h"

//////////// HAL ///////////////
#include "sysctl.h"
#include "fpioa.h"
#include "dvp.h"
#include "sleep.h"
#include "ov2640_regs.h"
#include "stdlib.h"
#include "utils.h"
#include "plic.h"
#include "math.h"
#include "iomem.h"
#include "Arduino.h" // millis

#if FIX_CACHE
#define DVP_MALLOC	iomem_malloc
#define DVP_FREE	iomem_free
#else
#define DVP_MALLOC	malloc
#define DVP_FREE	free
#endif

volatile static uint8_t g_dvp_finish_flag = 0;

#define IM_MAX(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define IM_MIN(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define IM_DIV(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a / _b) : 0; })
#define IM_MOD(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a % _b) : 0; })


typedef enum {
    ACTIVE_LOW,
    ACTIVE_HIGH,
    ACTIVE_BINOCULAR,
} polarity_t;

#define DCMI_RESET_LOW()      dvp->cmos_cfg &= ~DVP_CMOS_RESET
#define DCMI_RESET_HIGH()     dvp->cmos_cfg |= DVP_CMOS_RESET
#define DCMI_PWDN_LOW()       dvp->cmos_cfg |= DVP_CMOS_POWER_DOWN
#define DCMI_PWDN_HIGH()      dvp->cmos_cfg &= ~DVP_CMOS_POWER_DOWN

/*
 * Image sizes
 */
#define CIF_WIDTH	352
#define CIF_HEIGHT	288

#define HD_720_WIDTH	1280
#define HD_720_HEIGHT	720

#define HD_1080_WIDTH	1920
#define HD_1080_HEIGHT	1080

#define QCIF_WIDTH	176
#define QCIF_HEIGHT	144

#define QQCIF_WIDTH	88
#define QQCIF_HEIGHT	72

#define QQVGA_WIDTH	160
#define QQVGA_HEIGHT	120

#define HQVGA_WIDTH	    240
#define HQVGA_HEIGHT	160

#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240

#define SVGA_WIDTH	800
#define SVGA_HEIGHT	600

#define SXGA_WIDTH	1280
#define SXGA_HEIGHT	1024

#define VGA_WIDTH	640
#define VGA_HEIGHT	480

#define UXGA_WIDTH	1600
#define UXGA_HEIGHT	1200

#define XGA_WIDTH	1024
#define XGA_HEIGHT	768

/*
 * Registers settings
 */

#define ENDMARKER { 0x00, 0x00 }

static const uint8_t ov2640_init_regs[][2] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ 0x2c,   0xff },
	{ 0x2e,   0xdf },
	{ BANK_SEL, BANK_SEL_SENS },
	{ 0x3c,   0x32 },
	{ CLKRC,  CLKRC_DIV_SET(1) },
	{ COM2,   COM2_OCAP_Nx_SET(3) },
	{ REG04,  REG04_DEF | REG04_HREF_EN },
	{ COM8,   COM8_DEF | COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN },
	{ COM9,   COM9_AGC_GAIN_8x | 0x08},
	{ 0x2c,   0x0c },
	{ 0x33,   0x78 },
	{ 0x3a,   0x33 },
	{ 0x3b,   0xfb },
	{ 0x3e,   0x00 },
	{ 0x43,   0x11 },
	{ 0x16,   0x10 },
	{ 0x39,   0x02 },
	{ 0x35,   0x88 },
	{ 0x22,   0x0a },
	{ 0x37,   0x40 },
	{ 0x23,   0x00 },
	{ ARCOM2, 0xa0 },
	{ 0x06,   0x02 },
	{ 0x06,   0x88 },
	{ 0x07,   0xc0 },
	{ 0x0d,   0xb7 },
	{ 0x0e,   0x01 },
	{ 0x4c,   0x00 },
	{ 0x4a,   0x81 },
	{ 0x21,   0x99 },
	{ AEW,    0x40 },
	{ AEB,    0x38 },
	{ VV,     VV_HIGH_TH_SET(0x08) | VV_LOW_TH_SET(0x02) },
	{ 0x5c,   0x00 },
	{ 0x63,   0x00 },
	{ FLL,    0x22 },
	{ COM3,   0x38 | COM3_BAND_AUTO },
	{ REG5D,  0x55 },
	{ REG5E,  0x7d },
	{ REG5F,  0x7d },
	{ REG60,  0x55 },
	{ HISTO_LOW,   0x70 },
	{ HISTO_HIGH,  0x80 },
	{ 0x7c,   0x05 },
	{ 0x20,   0x80 },
	{ 0x28,   0x30 },
	{ 0x6c,   0x00 },
	{ 0x6d,   0x80 },
	{ 0x6e,   0x00 },
	{ 0x70,   0x02 },
	{ 0x71,   0x94 },
	{ 0x73,   0xc1 },
	{ 0x3d,   0x34 },
	{ COM7,   COM7_RES_UXGA | COM7_ZOOM_EN },
	{ REG5A,  BD50_MAX_AEC_STEP_SET(6)
		   | BD60_MAX_AEC_STEP_SET(8) },		/* 0x57 */
	{ COM25,  COM25_50HZ_BANDING_AEC_MSBS_SET(0x0bb)
		   | COM25_60HZ_BANDING_AEC_MSBS_SET(0x09c) },	/* 0x00 */
	{ BD50,   BD50_50HZ_BANDING_AEC_LSBS_SET(0x0bb) },	/* 0xbb */
	{ BD60,   BD60_60HZ_BANDING_AEC_LSBS_SET(0x09c) },	/* 0x9c */
	{ BANK_SEL,  BANK_SEL_DSP },
	{ 0xe5,   0x7f },
	{ MC_BIST,  MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL },
	{ 0x41,   0x24 },
	{ RESET,  RESET_JPEG | RESET_DVP },
	{ 0x76,   0xff },
	{ 0x33,   0xa0 },
	{ 0x42,   0x20 },
	{ 0x43,   0x18 },
	{ 0x4c,   0x00 },
	{ CTRL3,  CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10 },
	{ 0x88,   0x3f },
	{ 0xd7,   0x03 },
	{ 0xd9,   0x10 },
	{ R_DVP_SP,  R_DVP_SP_AUTO_MODE | 0x2 },
	{ 0xc8,   0x08 },
	{ 0xc9,   0x80 },
	{ BPADDR, 0x00 },
	{ BPDATA, 0x00 },
	{ BPADDR, 0x03 },
	{ BPDATA, 0x48 },
	{ BPDATA, 0x48 },
	{ BPADDR, 0x08 },
	{ BPDATA, 0x20 },
	{ BPDATA, 0x10 },
	{ BPDATA, 0x0e },
	{ 0x90,   0x00 },
	{ 0x91,   0x0e },
	{ 0x91,   0x1a },
	{ 0x91,   0x31 },
	{ 0x91,   0x5a },
	{ 0x91,   0x69 },
	{ 0x91,   0x75 },
	{ 0x91,   0x7e },
	{ 0x91,   0x88 },
	{ 0x91,   0x8f },
	{ 0x91,   0x96 },
	{ 0x91,   0xa3 },
	{ 0x91,   0xaf },
	{ 0x91,   0xc4 },
	{ 0x91,   0xd7 },
	{ 0x91,   0xe8 },
	{ 0x91,   0x20 },
	{ 0x92,   0x00 },
	{ 0x93,   0x06 },
	{ 0x93,   0xe3 },
	{ 0x93,   0x03 },
	{ 0x93,   0x03 },
	{ 0x93,   0x00 },
	{ 0x93,   0x02 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x96,   0x00 },
	{ 0x97,   0x08 },
	{ 0x97,   0x19 },
	{ 0x97,   0x02 },
	{ 0x97,   0x0c },
	{ 0x97,   0x24 },
	{ 0x97,   0x30 },
	{ 0x97,   0x28 },
	{ 0x97,   0x26 },
	{ 0x97,   0x02 },
	{ 0x97,   0x98 },
	{ 0x97,   0x80 },
	{ 0x97,   0x00 },
	{ 0x97,   0x00 },
	{ 0xa4,   0x00 },
	{ 0xa8,   0x00 },
	{ 0xc5,   0x11 },
	{ 0xc6,   0x51 },
	{ 0xbf,   0x80 },
	{ 0xc7,   0x10 },	/* simple AWB */
	{ 0xb6,   0x66 },
	{ 0xb8,   0xA5 },
	{ 0xb7,   0x64 },
	{ 0xb9,   0x7C },
	{ 0xb3,   0xaf },
	{ 0xb4,   0x97 },
	{ 0xb5,   0xFF },
	{ 0xb0,   0xC5 },
	{ 0xb1,   0x94 },
	{ 0xb2,   0x0f },
	{ 0xc4,   0x5c },
	{ 0xa6,   0x00 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x1b },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0x7f,   0x00 },
	{ 0xe5,   0x1f },
	{ 0xe1,   0x77 },
	{ 0xdd,   0x7f },
	{ CTRL0,  CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN },
	ENDMARKER,
};

/*
 * Register settings for window size
 * The preamble, setup the internal DSP to input an UXGA (1600x1200) image.
 * Then the different zooming configurations will setup the output image size.
 */
static const uint8_t ov2640_size_change_preamble_regs[][2] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ RESET, RESET_DVP },
	{ SIZEL, SIZEL_HSIZE8_11_SET(UXGA_WIDTH) |
		 SIZEL_HSIZE8_SET(UXGA_WIDTH) |
		 SIZEL_VSIZE8_SET(UXGA_HEIGHT) },
	{ HSIZE8, HSIZE8_SET(UXGA_WIDTH) },
	{ VSIZE8, VSIZE8_SET(UXGA_HEIGHT) },
	{ CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN |
		 CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
	{ HSIZE, HSIZE_SET(UXGA_WIDTH) },
	{ VSIZE, VSIZE_SET(UXGA_HEIGHT) },
	{ XOFFL, XOFFL_SET(0) },
	{ YOFFL, YOFFL_SET(0) },
	{ VHYX, VHYX_HSIZE_SET(UXGA_WIDTH) | VHYX_VSIZE_SET(UXGA_HEIGHT) |
		VHYX_XOFF_SET(0) | VHYX_YOFF_SET(0)},
	{ TEST, TEST_HSIZE_SET(UXGA_WIDTH) },
	ENDMARKER,
};

#define PER_SIZE_REG_SEQ(x, y, v_div, h_div, pclk_div)	\
	{ CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(v_div) |	\
		 CTRLI_H_DIV_SET(h_div)},		\
	{ ZMOW, ZMOW_OUTW_SET(x) },			\
	{ ZMOH, ZMOH_OUTH_SET(y) },			\
	{ ZMHH, ZMHH_OUTW_SET(x) | ZMHH_OUTH_SET(y) },	\
	{ R_DVP_SP, pclk_div },				\
	{ RESET, 0x00}

static const uint8_t ov2640_qcif_regs[][2] = {
	PER_SIZE_REG_SEQ(QCIF_WIDTH, QCIF_HEIGHT, 3, 3, 4),
	ENDMARKER,
};

static const uint8_t ov2640_qqvga_regs[][2] = {
	PER_SIZE_REG_SEQ(QQVGA_WIDTH, QQVGA_HEIGHT, 3, 3, 4),
	ENDMARKER,
};

static const uint8_t ov2640_hqvga_regs[][2] = {
	PER_SIZE_REG_SEQ(HQVGA_WIDTH, HQVGA_HEIGHT, 2, 2, 4),
	ENDMARKER,
};

static const uint8_t ov2640_qvga_regs[][2] = {
	PER_SIZE_REG_SEQ(QVGA_WIDTH, QVGA_HEIGHT, 2, 2, 4),
	ENDMARKER,
};

static const uint8_t ov2640_cif_regs[][2] = {
	PER_SIZE_REG_SEQ(CIF_WIDTH, CIF_HEIGHT, 2, 2, 8),
	ENDMARKER,
};

static const uint8_t ov2640_vga_regs[][2] = {
	PER_SIZE_REG_SEQ(VGA_WIDTH, VGA_HEIGHT, 0, 0, 2),
	ENDMARKER,
};

static const uint8_t ov2640_svga_regs[][2] = {
	PER_SIZE_REG_SEQ(SVGA_WIDTH, SVGA_HEIGHT, 1, 1, 2),
	ENDMARKER,
};

static const uint8_t ov2640_xga_regs[][2] = {
	PER_SIZE_REG_SEQ(XGA_WIDTH, XGA_HEIGHT, 0, 0, 2),
	{ CTRLI,    0x00},
	ENDMARKER,
};

static const uint8_t ov2640_sxga_regs[][2] = {
	PER_SIZE_REG_SEQ(SXGA_WIDTH, SXGA_HEIGHT, 0, 0, 2),
	{ CTRLI,    0x00},
	{ R_DVP_SP, 2 | R_DVP_SP_AUTO_MODE },
	ENDMARKER,
};

static const uint8_t ov2640_uxga_regs[][2] = {
	PER_SIZE_REG_SEQ(UXGA_WIDTH, UXGA_HEIGHT, 0, 0, 0),
	{ CTRLI,    0x00},
	{ R_DVP_SP, 0 | R_DVP_SP_AUTO_MODE },
	ENDMARKER,
};

static const uint8_t yuv422_regs[][2] = {
        { BANK_SEL, BANK_SEL_DSP },
    	{ R_BYPASS, R_BYPASS_USE_DSP },
        { RESET,   RESET_DVP},
        { 0xD7,     0x01 },
        { IMAGE_MODE, IMAGE_MODE_YUV422 },
        { 0xE1,     0x67 },
        { RESET,    0x00 },
        {0, 0},
};

static const uint8_t rgb565_regs[][2] = {
        { BANK_SEL,   BANK_SEL_DSP },
    	{ R_BYPASS, R_BYPASS_USE_DSP },
        { RESET,      RESET_DVP},
        { 0xD7,       0x03},
        { IMAGE_MODE, IMAGE_MODE_RGB565 },
        { 0xE1,       0x77 },
        { RESET,      0x00 },
        {0,           0},
};

static const uint8_t jpeg_regs[][2] = {
        { BANK_SEL, BANK_SEL_DSP },
    	{ R_BYPASS, R_BYPASS_USE_DSP },
        { RESET,   RESET_DVP},
        { IMAGE_MODE, IMAGE_MODE_JPEG_EN|IMAGE_MODE_RGB565 },
        { 0xD7,     0x03 },
        { 0xE1,     0x77 },
        { QS,       0x0C },
        { RESET,    0x00 },
        {0, 0},
};

#define NUM_BRIGHTNESS_LEVELS (5)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS + 1][5] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA },
    { 0x00, 0x04, 0x09, 0x00, 0x00 }, /* -2 */
    { 0x00, 0x04, 0x09, 0x10, 0x00 }, /* -1 */
    { 0x00, 0x04, 0x09, 0x20, 0x00 }, /*  0 */
    { 0x00, 0x04, 0x09, 0x30, 0x00 }, /* +1 */
    { 0x00, 0x04, 0x09, 0x40, 0x00 }, /* +2 */
};

#define NUM_CONTRAST_LEVELS (5)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS + 1][7] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA, BPDATA, BPDATA },
    { 0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06 }, /* -2 */
    { 0x00, 0x04, 0x07, 0x20, 0x1c, 0x2a, 0x06 }, /* -1 */
    { 0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06 }, /*  0 */
    { 0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06 }, /* +1 */
    { 0x00, 0x04, 0x07, 0x20, 0x28, 0x0c, 0x06 }, /* +2 */
};

#define NUM_SATURATION_LEVELS (5)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS + 1][5] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA },
    { 0x00, 0x02, 0x03, 0x28, 0x28 }, /* -2 */
    { 0x00, 0x02, 0x03, 0x38, 0x38 }, /* -1 */
    { 0x00, 0x02, 0x03, 0x48, 0x48 }, /*  0 */
    { 0x00, 0x02, 0x03, 0x58, 0x58 }, /* +1 */
    { 0x00, 0x02, 0x03, 0x58, 0x58 }, /* +2 */
};



Sipeed_OV2640::Sipeed_OV2640(framesize_t frameSize, pixformat_t pixFormat)
:Camera(frameSize, pixFormat),
_dataBuffer(NULL), _aiBuffer(NULL),
_resetPoliraty(ACTIVE_HIGH), _pwdnPoliraty(ACTIVE_HIGH),
_slaveAddr(0x00),
_id(0)
{
    configASSERT(pixFormat == PIXFORMAT_RGB565 || pixFormat==PIXFORMAT_YUV422);
}


Sipeed_OV2640::~Sipeed_OV2640()
{
    end();
}

bool Sipeed_OV2640::begin()
{
    return begin(false);
}

bool Sipeed_OV2640::begin(bool binocular)
{
    if(_dataBuffer)
        DVP_FREE(_dataBuffer);
    if(_aiBuffer)
        DVP_FREE(_aiBuffer);
    _dataBuffer = (uint8_t*)DVP_MALLOC(_width*_height*2); //RGB565
    if(!_dataBuffer)
    {
        _width = 0;
        _height = 0;
        return false;
    }
    _aiBuffer = (uint8_t*)DVP_MALLOC(_width*_height*3);   //RGB888
    if(!_aiBuffer)
    {
        _width = 0;
        _height = 0;
        DVP_FREE(_dataBuffer);
        return false;
    }

    if(!reset(binocular))
        return false;
    
    if(binocular) {
        // Configure sensor 0
        shutdown(true);
        if (!configure())
            return false;

        // Configure sensor 1
        shutdown(false);
        if (!configure())
            return false;
    }
    else if (!configure())
        return false;

    return true;
}

bool Sipeed_OV2640::configure() 
{
    if(ov2640_set_framesize(_frameSize, _width, _height) < 0)
        return false; 
    if(ov2640_set_pixformat(_pixFormat) < 0)
        return false;
    if(ov2640_set_hmirror(_invert) < 0)
        return false;
    if(ov2640_set_vflip(_flip) < 0)
        return false;
    return true;
}

void Sipeed_OV2640::end()
{
    if(_dataBuffer)
        DVP_FREE(_dataBuffer);
    if(_aiBuffer)
        DVP_FREE(_aiBuffer);
    _dataBuffer = nullptr;
    _aiBuffer   = nullptr;
}

bool Sipeed_OV2640::reset(bool binocular)
{
    if(dvpInit() != 0)
        return false;
    
    if(binocular)
    {
        // Reset sensor 0
        shutdown(true);
        DCMI_RESET_LOW();
        delay(10);
        DCMI_RESET_HIGH();
        delay(10);
        if(ov2640_reset() != 0)
            return false;

        // Reset sensor 1
        shutdown(false);
        delay(10);
        DCMI_RESET_LOW();
        delay(10);
        DCMI_RESET_HIGH();
        delay(10);
        if(ov2640_reset() != 0)
            return false;
    }
    else {
        if(ov2640_reset() != 0)
            return false;
    }
    
    if(dvpInitIrq() != 0)
        return false;
    return true;
}

bool Sipeed_OV2640::run(bool run)
{
	if(run)
	{
		dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
		plic_irq_enable(IRQN_DVP_INTERRUPT);
		dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);
	}
	else{
		plic_irq_disable(IRQN_DVP_INTERRUPT);
		dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
		dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
	}
    return true;
}

int Sipeed_OV2640::id()
{
    return _id;
}

/**
 * @return pixels 
 *         If pixels format is RGB565: return RGB565 pixels with every uint16_t one pixel, e.g. RED: 0xF800
 */
uint8_t* Sipeed_OV2640::snapshot()
{
    if ( sensor_snapshot() != 0)
        return nullptr;
    return _dataBuffer;
}

void Sipeed_OV2640::setRotation(uint8_t rotation)
{
    //FIXME
}

void Sipeed_OV2640::setInvert(bool invert)
{
    _invert = invert;
}

void Sipeed_OV2640::setFlip(bool flip)
{
    _flip = flip;
}

void Sipeed_OV2640::shutdown(bool enable)
{
    if (enable)
    {
        DCMI_PWDN_HIGH();
    }
    else
    {
        DCMI_PWDN_LOW();
    }

    delay(10);
}

int Sipeed_OV2640::dvpInit(uint32_t freq)
{
    // just support RGB565 and YUV442 on k210
    configASSERT(_pixFormat==PIXFORMAT_RGB565 || _pixFormat==PIXFORMAT_YUV422);
    _freq  = freq;

	fpioa_set_function(47, FUNC_CMOS_PCLK);
	fpioa_set_function(46, FUNC_CMOS_XCLK);
	fpioa_set_function(45, FUNC_CMOS_HREF);
	fpioa_set_function(44, FUNC_CMOS_PWDN);
	fpioa_set_function(43, FUNC_CMOS_VSYNC);
	fpioa_set_function(42, FUNC_CMOS_RST);
	fpioa_set_function(41, FUNC_SCCB_SCLK);
	fpioa_set_function(40, FUNC_SCCB_SDA);

    /* Do a power cycle */
    DCMI_PWDN_HIGH();
    msleep(10);

    DCMI_PWDN_LOW();
    msleep(10);

    // Initialize the camera bus, 8bit reg
    dvp_init(8);
	 // Initialize dvp interface
	dvp_set_xclk_rate(freq);
	dvp->cmos_cfg |= DVP_CMOS_CLK_DIV(3) | DVP_CMOS_CLK_ENABLE;
	dvp_enable_burst();
	dvp_disable_auto();
	dvp_set_output_enable(DVP_OUTPUT_AI, 1);	//enable to AI
	dvp_set_output_enable(DVP_OUTPUT_DISPLAY, 1);	//enable to lcd
    if( _pixFormat == PIXFORMAT_YUV422)
        dvp_set_image_format(DVP_CFG_YUV_FORMAT);
    else
	    dvp_set_image_format(DVP_CFG_RGB_FORMAT);
	dvp_set_image_size(_width, _height);	//set QVGA default
	dvp_set_ai_addr( (uint32_t)((long)_aiBuffer), (uint32_t)((long)(_aiBuffer+_width*_height)), (uint32_t)((long)(_aiBuffer+_width*_height*2)));
	dvp_set_display_addr( (uint32_t)((long)_dataBuffer) );

    if(0 == sensor_ov_detect()){//find ov sensor
        // printf("find ov sensor\n");
    }
    else if(0 == sensro_gc_detect()){//find gc0328 sensor
        // printf("find gc3028\n");
    }

    return 0;
}

int Sipeed_OV2640::cambus_scan()
{

	uint16_t manuf_id = 0;
	uint16_t device_id = 0;
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
		cambus_read_id(addr ,&manuf_id,&device_id);
		if(0xffff != device_id)
		{
			return addr ;
		}
    }
    return 0;
}

int Sipeed_OV2640::cambus_read_id(uint8_t addr,uint16_t *manuf_id, uint16_t *device_id)
{
	dvp_sccb_send_data(addr, 0xFF, 0x01);
	*manuf_id = (dvp_sccb_receive_data(addr, 0x1C) << 8) | dvp_sccb_receive_data(addr, 0x1D);
	*device_id = (dvp_sccb_receive_data(addr, 0x0A) << 8) | dvp_sccb_receive_data(addr, 0x0B);
	return 0;
}

int Sipeed_OV2640::cambus_scan_gc0328(void)
{
    dvp_sccb_send_data(GC0328_ADDR, 0xFE, 0x00);
    uint8_t id = dvp_sccb_receive_data(GC0328_ADDR, 0xf0);
    if (id != 0x9d)
    {
        // printf("error gc0328 detect, ret id is 0x%x\r\n", id);
        return 0;
    }
    return id;
}

int Sipeed_OV2640::cambus_readb(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data)
{

    int ret = 0;
	*reg_data = dvp_sccb_receive_data(slv_addr, reg_addr);

	if(0xff == *reg_data)
		ret = -1;

    return ret;

}


int Sipeed_OV2640::cambus_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)
{

	dvp_sccb_send_data(slv_addr,reg_addr,reg_data);
	return 0;
}

int Sipeed_OV2640::cambus_readw(uint8_t slv_addr, uint8_t reg_addr, uint16_t *reg_data)
{
    return 0;
}

int Sipeed_OV2640::cambus_writew(uint8_t slv_addr, uint8_t reg_addr, uint16_t reg_data)
{
    return 0;
}

int Sipeed_OV2640::cambus_readw2(uint8_t slv_addr, uint16_t reg_addr, uint16_t *reg_data)
{
    return 0;
}

int Sipeed_OV2640::cambus_writew2(uint8_t slv_addr, uint16_t reg_addr, uint16_t reg_data)
{
    return 0;
}



int Sipeed_OV2640::sensor_ov_detect()
{
    /* Reset the sensor */
    DCMI_RESET_HIGH();
    msleep(10);

    DCMI_RESET_LOW();
    msleep(10);

    /* Probe the ov sensor */
    _slaveAddr = cambus_scan();
    if (_slaveAddr == 0) {
        /* Sensor has been held in reset,
           so the reset line is active low */
        _resetPoliraty = ACTIVE_LOW;

        /* Pull the sensor out of the reset state,systick_sleep() */
        DCMI_RESET_HIGH();
        msleep(10);

        /* Probe again to set the slave addr */
        _slaveAddr = cambus_scan();
        if (_slaveAddr == 0) {
            _pwdnPoliraty = ACTIVE_LOW;

            DCMI_PWDN_HIGH();
            msleep(10);

            _slaveAddr = cambus_scan();
            if (_slaveAddr == 0) {
                _resetPoliraty = ACTIVE_HIGH;

                DCMI_RESET_LOW();
                msleep(10);

                _slaveAddr = cambus_scan();
                if(_slaveAddr == 0) {
                    //should do something?
                    return -2;
                }
            }
        }
    }

    // Clear sensor chip ID.
    _id = 0;

    if (_slaveAddr == LEPTON_ID) {
        _id = LEPTON_ID;
		/*set LEPTON xclk rate*/
		/*lepton_init*/
    } else {
        // Read ON semi sensor ID.
        cambus_readb(_slaveAddr, ON_CHIP_ID, &_id);
        if (_id == MT9V034_ID) {
			/*set MT9V034 xclk rate*/
			/*mt9v034_init*/
        } else { // Read OV sensor ID.
            cambus_readb(_slaveAddr, OV_CHIP_ID, &_id);
            // Initialize sensor struct.
            switch (_id) {
                case OV9650_ID:
					/*ov9650_init*/
                    break;
                case OV2640_ID:
                    // printf("detect ov2640, id:%x\n", _slaveAddr);
                    break;
                case OV7725_ID:
					/*ov7725_init*/
                    break;
                default:
                    // Sensor is not supported.
                    return -3;
            }
        }
    }

    // if (init_ret != 0 ) {
    //     // Sensor init failed.
    //     return -4;
    // }
    return 0;
}

int Sipeed_OV2640::sensro_gc_detect()
{
    DCMI_PWDN_HIGH();//enable gc0328 要恢复 normal 工作模式，需将 PWDN pin 接入低电平即可，同时写入初始化寄存器即可
    DCMI_RESET_LOW();//reset gc3028
    msleep(10);
    DCMI_RESET_HIGH();
    msleep(10);
    uint8_t id = cambus_scan_gc0328();
    if(0 == id)
    {
        return -3;
    }
    else
    {
        // printf("[MAIXPY]: gc0328 id = %x\n",id); 
        _slaveAddr = GC0328_ADDR;
        _id = id;
    }
    return 0;
}

#ifdef __cplusplus
extern "C" {
#endif

static int sensor_irq(void *ctx)
{
	if (dvp_get_interrupt(DVP_STS_FRAME_FINISH)) {	//frame end
		dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
		g_dvp_finish_flag = 1;
	} else {	//frame start
        if(g_dvp_finish_flag == 0)  //only we finish the convert, do transmit again
            dvp_start_convert();	//so we need deal img ontime, or skip one framebefore next
		dvp_clear_interrupt(DVP_STS_FRAME_START);
	}

	return 0;
}

#ifdef __cplusplus
}
#endif



int Sipeed_OV2640::dvpInitIrq()
{
	dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
	plic_set_priority(IRQN_DVP_INTERRUPT, 2);
    /* set irq handle */
	plic_irq_register(IRQN_DVP_INTERRUPT, sensor_irq, (void*)NULL);

	plic_irq_disable(IRQN_DVP_INTERRUPT);
	dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
	dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);

	return 0;
}




int Sipeed_OV2640::ov2640_reset()
{
    int i=0;
    const uint8_t (*regs)[2];

    /* Reset all registers */
    cambus_writeb(_slaveAddr, BANK_SEL, BANK_SEL_SENS);
    cambus_writeb(_slaveAddr, COM7, COM7_SRST);

    /* delay n ms */
    msleep(10);

    i = 0;
    regs = ov2640_init_regs;
    /* Write initial regsiters */
    while (regs[i][0]) {
        cambus_writeb(_slaveAddr, regs[i][0], regs[i][1]);
        i++;
    }

    return 0;
}

int Sipeed_OV2640::ov2640_read_reg(uint8_t reg_addr)
{
    uint8_t reg_data;
    if (cambus_readb(_slaveAddr, reg_addr, &reg_data) != 0) {
        return -1;
    }
    return reg_data;
}

int Sipeed_OV2640::ov2640_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    return cambus_writeb(_slaveAddr, reg_addr, reg_data);
}


int Sipeed_OV2640::ov2640_set_pixformat(pixformat_t pixformat)
{
    int i=0;
    const uint8_t (*regs)[2]=NULL;

    /* read pixel format reg */
    switch (pixformat) {
        case PIXFORMAT_RGB565:
            regs = rgb565_regs;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
            regs = yuv422_regs;
            break;
        case PIXFORMAT_JPEG:
            regs = jpeg_regs;

            break;
        default:
            return -1;
    }

    /* Write initial regsiters */
    while (regs[i][0]) {
        cambus_writeb(_slaveAddr, regs[i][0], regs[i][1]);
        i++;
    }
    switch (pixformat) {
        case PIXFORMAT_RGB565:
			dvp_set_image_format(DVP_CFG_RGB_FORMAT);
            break;
        case PIXFORMAT_YUV422:
            dvp_set_image_format(DVP_CFG_YUV_FORMAT);
            break;
        case PIXFORMAT_GRAYSCALE:
			dvp_set_image_format(DVP_CFG_Y_FORMAT);
            break;
        case PIXFORMAT_JPEG:
			dvp_set_image_format(DVP_CFG_RGB_FORMAT);
            break;
        default:
            return -1;
    }
    /* delay n ms */
    msleep(30);
    return 0;
}

int Sipeed_OV2640::ov2640_set_framesize(framesize_t framesize, int w, int h)
{
    int ret=0;
    const uint8_t (*regs)[2];

    switch (framesize) {
        case FRAMESIZE_QCIF:
            regs = ov2640_qcif_regs;
            break;

        case FRAMESIZE_QQVGA:
            regs = ov2640_qqvga_regs;
            break;

        case FRAMESIZE_HQVGA:
            regs = ov2640_hqvga_regs;
            break;

        case FRAMESIZE_QVGA:
            regs = ov2640_qvga_regs;
            break;

        case FRAMESIZE_CIF:
            regs = ov2640_cif_regs;
            break;

        case FRAMESIZE_VGA:
            regs = ov2640_vga_regs;
            break;

        case FRAMESIZE_SVGA:
            regs = ov2640_svga_regs;
            break;

        case FRAMESIZE_XGA:
            regs = ov2640_xga_regs;
            break;

        case FRAMESIZE_SXGA:
            regs = ov2640_sxga_regs;
            break;

        case FRAMESIZE_UXGA:
            regs = ov2640_uxga_regs;
            break;

        default:
            return -1;
    }

    /* Write DSP preamble regsiters */
    for (int i = 0; ov2640_size_change_preamble_regs[i][0]; i++) {
        ret |= cambus_writeb(_slaveAddr, ov2640_size_change_preamble_regs[i][0], ov2640_size_change_preamble_regs[i][1]);
    }

    /* Write DSP input regsiters */
    for (int i = 0; regs[i][0]; i++) {
        ret |= cambus_writeb(_slaveAddr, regs[i][0], regs[i][1]);
    }

    /* delay n ms */
    msleep(30);
	dvp_set_image_size(w, h);
    return ret;
}

int Sipeed_OV2640::ov2640_set_framerate(framerate_t framerate)
{
    return 0;
}

int Sipeed_OV2640::ov2640_set_contrast(int level)
{
    int ret=0;

    level += (NUM_CONTRAST_LEVELS / 2 + 1);
    if (level < 0 || level > NUM_CONTRAST_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, BANK_SEL_DSP);

    /* Write contrast registers */
    for (int i=0; i<sizeof(contrast_regs[0])/sizeof(contrast_regs[0][0]); i++) {
        ret |= cambus_writeb(_slaveAddr, contrast_regs[0][i], contrast_regs[level][i]);
    }

    return ret;
}

int Sipeed_OV2640::ov2640_set_brightness(int level)
{
    int ret=0;

    level += (NUM_BRIGHTNESS_LEVELS / 2 + 1);
    if (level < 0 || level > NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, BANK_SEL_DSP);

    /* Write brightness registers */
    for (int i=0; i<sizeof(brightness_regs[0])/sizeof(brightness_regs[0][0]); i++) {
        ret |= cambus_writeb(_slaveAddr, brightness_regs[0][i], brightness_regs[level][i]);
    }

    return ret;
}

int Sipeed_OV2640::ov2640_set_saturation(int level)
{
    int ret=0;

    level += (NUM_SATURATION_LEVELS / 2 + 1);
    if (level < 0 || level > NUM_SATURATION_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, BANK_SEL_DSP);

    /* Write contrast registers */
    for (int i=0; i<sizeof(saturation_regs[0])/sizeof(saturation_regs[0][0]); i++) {
        ret |= cambus_writeb(_slaveAddr, saturation_regs[0][i], saturation_regs[level][i]);
    }

    return ret;
}

int Sipeed_OV2640::ov2640_set_gainceiling( gainceiling_t gainceiling)
{
    int ret =0;

    /* Switch to SENSOR register bank */
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, BANK_SEL_SENS);

    /* Write gain ceiling register */
    ret |= cambus_writeb(_slaveAddr, COM9, COM9_AGC_SET(gainceiling));

    return ret;
}

int Sipeed_OV2640::ov2640_set_quality(int qs)
{
    int ret=0;

    /* Switch to DSP register bank */
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, BANK_SEL_DSP);

    /* Write QS register */
    ret |= cambus_writeb(_slaveAddr, QS, qs);

    return ret;
}

int Sipeed_OV2640::ov2640_set_colorbar(int enable)
{
    uint8_t reg;
    /* Switch to SENSOR register bank */
    int ret = cambus_writeb(_slaveAddr, BANK_SEL, BANK_SEL_SENS);

    /* Update COM7 */
    ret |= cambus_readb(_slaveAddr, COM7, &reg);

    if (enable) {
        reg |= COM7_COLOR_BAR_TEST;
    } else {
        reg &= ~COM7_COLOR_BAR_TEST;
    }

    return cambus_writeb(_slaveAddr, COM7, reg) | ret;
}

int Sipeed_OV2640::ov2640_set_auto_gain(int enable, float gain_db, float gain_db_ceiling)
{
   uint8_t reg;
   int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
   ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENS);
   ret |= cambus_readb(_slaveAddr, COM8, &reg);
   ret |= cambus_writeb(_slaveAddr, COM8, (reg & (~COM8_AGC_EN)) | ((enable != 0) ? COM8_AGC_EN : 0));

   if ((enable == 0) && (!isnanf(gain_db)) && (!isinff(gain_db))) {
       float gain = IM_MAX(IM_MIN(expf((gain_db / 20.0) * log(10.0)), 32.0), 1.0);

       int gain_temp = roundf(log2(IM_MAX(gain / 2.0, 1.0)));
       int gain_hi = 0xF >> (4 - gain_temp);
       int gain_lo = IM_MIN(roundf(((gain / (1 << gain_temp)) - 1.0) * 16.0), 15);

       ret |= cambus_writeb(_slaveAddr, GAIN, (gain_hi << 4) | (gain_lo << 0));
   } else if ((enable != 0) && (!isnanf(gain_db_ceiling)) && (!isinff(gain_db_ceiling))) {
       float gain_ceiling = IM_MAX(IM_MIN(expf((gain_db_ceiling / 20.0) * log(10.0)), 128.0), 2.0);

       ret |= cambus_readb(_slaveAddr, COM9, &reg);
       ret |= cambus_writeb(_slaveAddr, COM9, (reg & 0x1F) | (((int)ceilf(log2(gain_ceiling)) - 1) << 5));
   }

   return ret;
}

int Sipeed_OV2640::ov2640_get_gain_db(float *gain_db)
{
    uint8_t reg, gain;
    int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENS);
    ret |= cambus_readb(_slaveAddr, COM8, &reg);

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= cambus_writeb(_slaveAddr, COM8, reg & (~COM8_AGC_EN));
    // }
    // DISABLED

    ret |= cambus_readb(_slaveAddr, GAIN, &gain);

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= cambus_writeb(_slaveAddr, COM8, reg | COM8_AGC_EN);
    // }
    // DISABLED

    int hi_gain = 1 << (((gain >> 7) & 1) + ((gain >> 6) & 1) + ((gain >> 5) & 1) + ((gain >> 4) & 1));
    float lo_gain = 1.0 + (((gain >> 0) & 0xF) / 16.0);
    *gain_db = 20.0 * (log(hi_gain * lo_gain) / log(10.0));

    return ret;
}

int Sipeed_OV2640::ov2640_set_auto_exposure(int enable, int exposure_us)
{
    uint8_t reg;
    int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENS);
    ret |= cambus_readb(_slaveAddr, COM8, &reg);
    ret |= cambus_writeb(_slaveAddr, COM8, COM8_SET_AEC(reg, (enable != 0)));

    if ((enable == 0) && (exposure_us >= 0)) {
        ret |= cambus_readb(_slaveAddr, COM7, &reg);
        int t_line = 0;

        if (COM7_GET_RES(reg) == COM7_RES_UXGA) t_line = 1600 + 322;
        if (COM7_GET_RES(reg) == COM7_RES_SVGA) t_line = 800 + 390;
        if (COM7_GET_RES(reg) == COM7_RES_CIF) t_line = 400 + 195;

        ret |= cambus_readb(_slaveAddr, CLKRC, &reg);
        int pll_mult = (reg & R_DVP_SP_AUTO_MODE) ? 2 : 1;
        int clk_rc = ((reg & R_DVP_SP_DVP_MASK) + 1) * 2;

        ret |= cambus_readb(_slaveAddr, BANK_SEL, &reg);
        ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg & (~BANK_SEL_SENS));
        ret |= cambus_readb(_slaveAddr, IMAGE_MODE, &reg);
        int t_pclk = 0;

        if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_YUV422) t_pclk = 2;
        if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RAW10) t_pclk = 1;
        if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RGB565) t_pclk = 2;

        int exposure = IM_MAX(IM_MIN(((exposure_us*(((_freq/clk_rc)*pll_mult)/1000000))/t_pclk)/t_line,0xFFFF),0x0000);

        ret |= cambus_readb(_slaveAddr, BANK_SEL, &reg);
        ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENS);

        ret |= cambus_readb(_slaveAddr, REG04, &reg);
        ret |= cambus_writeb(_slaveAddr, REG04, (reg & 0xFC) | ((exposure >> 0) & 0x3));

        ret |= cambus_readb(_slaveAddr, AEC, &reg);
        ret |= cambus_writeb(_slaveAddr, AEC, (reg & 0x00) | ((exposure >> 2) & 0xFF));

        ret |= cambus_readb(_slaveAddr, REG04, &reg);
        ret |= cambus_writeb(_slaveAddr, REG04, (reg & 0xC0) | ((exposure >> 10) & 0x3F));
    }

    return ret;
}

int Sipeed_OV2640::ov2640_get_exposure_us(int *exposure_us)
{
    uint8_t reg, aec_10, aec_92, aec_1510;
    int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENS);
    ret |= cambus_readb(_slaveAddr, COM8, &reg);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= cambus_writeb(_slaveAddr, COM8, reg & (~COM8_AEC_EN));
    // }
    // DISABLED

    ret |= cambus_readb(_slaveAddr, REG04, &aec_10);
    ret |= cambus_readb(_slaveAddr, AEC, &aec_92);
    ret |= cambus_readb(_slaveAddr, REG45, &aec_1510);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= cambus_writeb(_slaveAddr, COM8, reg | COM8_AEC_EN);
    // }
    // DISABLED

    ret |= cambus_readb(_slaveAddr, COM7, &reg);
    int t_line = 0;

    if (COM7_GET_RES(reg) == COM7_RES_UXGA) t_line = 1600 + 322;
    if (COM7_GET_RES(reg) == COM7_RES_SVGA) t_line = 800 + 390;
    if (COM7_GET_RES(reg) == COM7_RES_CIF) t_line = 400 + 195;

    ret |= cambus_readb(_slaveAddr, CLKRC, &reg);
    int pll_mult = (reg & R_DVP_SP_AUTO_MODE) ? 2 : 1;
    int clk_rc = ((reg & R_DVP_SP_DVP_MASK) + 1) * 2;

    ret |= cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg & (~BANK_SEL_SENS));
    ret |= cambus_readb(_slaveAddr, IMAGE_MODE, &reg);
    int t_pclk = 0;

    if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_YUV422) t_pclk = 2;
    if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RAW10) t_pclk = 1;
    if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RGB565) t_pclk = 2;

    uint16_t exposure = ((aec_1510 & 0x3F) << 10) + ((aec_92 & 0xFF) << 2) + ((aec_10 & 0x3) << 0);
    *exposure_us = (exposure*t_line*t_pclk)/(((_freq/clk_rc)*pll_mult)/1000000);

    return ret;
}

int Sipeed_OV2640::ov2640_set_auto_whitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    uint8_t reg;
    int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg & (~BANK_SEL_SENS));
    ret |= cambus_readb(_slaveAddr, CTRL1, &reg);
    ret |= cambus_writeb(_slaveAddr, CTRL1, (reg & (~CTRL1_AWB)) | ((enable != 0) ? CTRL1_AWB : 0));

    if ((enable == 0) && (!isnanf(r_gain_db)) && (!isnanf(g_gain_db)) && (!isnanf(b_gain_db))
                      && (!isinff(r_gain_db)) && (!isinff(g_gain_db)) && (!isinff(b_gain_db))) {
    }

    return ret;
}

int Sipeed_OV2640::ov2640_get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    uint8_t reg;
    int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg & (~BANK_SEL_SENS));
    ret |= cambus_readb(_slaveAddr, CTRL1, &reg);

    // DISABLED
    // if (reg & CTRL1_AWB) {
    //     ret |= cambus_writeb(_slaveAddr, CTRL1, reg & (~CTRL1_AWB));
    // }
    // DISABLED

    // DISABLED
    // if (reg & CTRL1_AWB) {
    //     ret |= cambus_writeb(_slaveAddr, CTRL1, reg | CTRL1_AWB);
    // }
    // DISABLED

    return ret;
}

int Sipeed_OV2640::ov2640_set_hmirror(int enable)
{
    uint8_t reg;
    int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENS);
    ret |= cambus_readb(_slaveAddr, REG04, &reg);

    if (enable) {
        reg |= REG04_HFLIP_IMG;
    } else {
        reg &= ~REG04_HFLIP_IMG;
    }

    ret |= cambus_writeb(_slaveAddr, REG04, reg);

    return ret;
}

int Sipeed_OV2640::ov2640_set_vflip(int enable)
{
    uint8_t reg;
    int ret = cambus_readb(_slaveAddr, BANK_SEL, &reg);
    ret |= cambus_writeb(_slaveAddr, BANK_SEL, reg | BANK_SEL_SENS);
    ret |= cambus_readb(_slaveAddr, REG04, &reg);

    if (enable) {
        reg |= REG04_VFLIP_IMG;
        reg |= REG04_VREF_EN;
    } else {
        reg &= ~REG04_VFLIP_IMG;
        reg &= ~REG04_VREF_EN;
    }

    ret |= cambus_writeb(_slaveAddr, REG04, reg);

    return ret;
}

int Sipeed_OV2640::reverse_u32pixel(uint32_t* addr,uint32_t length)
{
  if(NULL == addr)
    return -1;

  uint32_t data;
  uint32_t* pend = addr+length;
  for(;addr<pend;addr++)
  {
	  data = *(addr);
	  *(addr) = ((data & 0x000000FF) << 24) | ((data & 0x0000FF00) << 8) | 
                ((data & 0x00FF0000) >> 8) | ((data & 0xFF000000) >> 24) ;
  }  //1.7ms
  
  
  return 0;
}


int Sipeed_OV2640::sensor_snapshot( )
{	
    //wait for new frame
    g_dvp_finish_flag = 0;
    uint32_t start =  millis();
    while (g_dvp_finish_flag == 0)
    {
        usleep(50);
        if(millis() - start > 300)//wait for 300ms
            return -1;
    }
    reverse_u32pixel((uint32_t*)_dataBuffer, _width*_height/2);
    return 0;
}

