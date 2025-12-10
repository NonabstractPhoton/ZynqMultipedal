// XPAR
#include <xparameters.h>

// Generic IO
#include "xil_printf.h"
#include <stdio.h>
#include <xil_io.h>
#include <xil_printf.h>
#include "xgpio.h"

// Libraries
#include <stdio.h>
#include <sleep.h>
#include "xiicps.h"
#include <xparameters.h>
#include "xuartps.h"
#include "stdlib.h"
#include <math.h>


// Custom
#include "encoder.h"
#include "oled.h"
#include "audio.h"

// IPs
#include "lms_pcore_addr.h"
#include "xnco.h"

// Functions
void Setup();
void AllocMemory();
void FreeMemory();
void OLEDGreet();
void UpdateStates();
void UpdateOLED();
void ApplyEffects();
unsigned char gpio_init();
void nco_init();
void circular_read(u32* lbuffer, u32*rbuffer, u32* readIdxPtr, u32 bufferSize, u32* lValPtr, u32* rValPtr);
void circular_write(u32* lbuffer, u32* rbuffer, u32* writeIdxPtr, u32 bufferSize, u32 lVal, u32 rVal);

// FX Functions

void Distortion();
void Compressor();
void Delay();
void Flanger();

/* ---------------------------------------------------------------------------- *
 * 						Redefinitions from xparameters.h 						*
 * ---------------------------------------------------------------------------- */
#define NCO_ID XPAR_NCO_0_DEVICE_ID

#define LMS_LOC XPAR_LMS_PCORE_0_BASEADDR
#define LMS_X LMS_LOC + x_k__Data_lms_pcore
#define LMS_D LMS_LOC + d_k__Data_lms_pcore
#define LMS_E LMS_LOC + e_k__Data_lms_pcore
#define LMS_STROBE LMS_LOC + IPCore_Strobe_lms_pcore

#define UART_BASEADDR XPAR_PS7_UART_1_BASEADDR

#define BUTTON_SWITCH_BASE XPAR_GPIO_1_BASEADDR
#define BUTTON_SWITCH_ID XPAR_GPIO_1_DEVICE_ID
#define AUDIO_ENABLE_ID XPAR_AXI_GPIO_0_DEVICE_ID

/* ---------------------------------------------------------------------------- *
 * 							Define GPIO Channels								*
 * ---------------------------------------------------------------------------- */
#define BUTTON_CHANNEL 1
#define SWITCH_CHANNEL 2

/* ---------------------------------------------------------------------------- *
 * 							FX Defines
 * ---------------------------------------------------------------------------- */

#define DC_OFFSET 2147483648.0
#define INV_DC_OFFSET 4.6566128730773926e-10 // 1.0 / 2147483648.0

#define EFFECT_COUNT 8
#define SLIDER_STATES 20
#define SLIDER_MAX 19// SLIDER_STATES -1

#define DISTORTION_MAX_GAIN 3.95
#define DISTORTION_MAX_WET_MIX .010

#define COMPRESSOR_MAX_VAL .75
#define COMPRESSOR_TARGET_DB -6.02059991328
#define COMPRESSOR_ATTACK .9997
#define COMPRESSOR_RELEASE .9999
// Conversion factor for 20 * log10(x) to 8.68589 * log(x)
#define DB_PER_NATURAL_LOG 8.685889638069818
// Conversion factor for exp(x / 20 * ln(10)) to exp(x * 0.11512925)
#define INV_DB_PER_NATURAL_LOG 0.11512925464970228
#define MIN_LEVEL 1e-10

#define DELAY_MAX_FIFO_SAMPLES 48000
#define FLANGER_MAX_FIFO_SAMPLES 4800

typedef void (*FXFunc)();

/* ---------------------------------------------------------------------------- *
 * 							Global Variables									*
 * ---------------------------------------------------------------------------- */
extern XIicPs Iic;
extern XGpio Gpio; // Gpio instance for buttons and switches
extern XGpio Gpio_audio_enable; // GPIO instance for digital mute
extern XNco Nco;
