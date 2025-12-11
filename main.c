#include "main.h"

#define DISTORTION_IDX 0
#define DELAY_IDX 3
#define FLANGER_IDX 2

PmodENC encoder;
PmodOLED oled;
XIicPs Iic;
XGpio Gpio; // Gpio instance for buttons and switches
XGpio Gpio_audio_enable; // GPIO instance for digital mute
XNco Nco;

u8 inSlider = 0;
int menuState = 0;
u8 sliderVals[EFFECT_COUNT];
u8 fxVal;
char labels[EFFECT_COUNT][16] = {"Distortion", "Bitcrusher", "Flanger", "SuperFlanger", "SineMult", "TBD5", "TBD6", "TBD7"};
FXFunc effectFuncs[EFFECT_COUNT] = {Distortion, Bitcrusher, Flanger, SuperFlanger, SineMult, Distortion, Distortion,Distortion};

u32* delayFIFOLeft;
u32* delayFIFORight;
u32 delayFIFOSize;
u32 delayFIFOReadIdx;
u32 delayFIFOWriteIdx;
u8 full;

u32* flangerFIFOLeft;
u32* flangerFIFORight;
u32 flangerFIFOSize;
u32 flangerFIFOReadIdx;
u32 flangerFIFOWriteIdx;

double distortionGains[SLIDER_STATES];
double invDistortionGains[SLIDER_STATES];
double distortionWetMixes[SLIDER_STATES];

u32 bitcrushVals[SLIDER_STATES];

char scrollText[] = "<<Scroll>>";
char selectText[] = ">>Select<<";

u8* pat;

u32 encState; u32 pastEncState;
int diff, i;

u32  in_left, in_right;
double temp_left, temp_right;
double fp_left, fp_right;


int main(void)
{
	Setup();

	while(XGpio_DiscreteRead(&Gpio, BUTTON_CHANNEL)==0)
	{
		UpdateStates();

		UpdateOLED();

		pastEncState = ENC_getState(&encoder);

		while (!XUartPs_IsReceiveData(UART_BASEADDR))
		{

			// Read audio input from codec
			in_left = Xil_In32(I2S_DATA_RX_L_REG);
			in_right = Xil_In32(I2S_DATA_RX_R_REG);
			if (ENC_switchOn(pastEncState))
				ApplyEffects();
			/*
				// Loud noise prevention
				if (in_left & (1 << 30))
					in_left = DC_OFFSET;
				if (in_right & (1 << 30))
					in_right = DC_OFFSET;
			*/


			// Write audio input to codec
			Xil_Out32(I2S_DATA_TX_L_REG, in_left);
			Xil_Out32(I2S_DATA_TX_R_REG, in_right);

			temp_left = INV_DC_OFFSET * ((double)in_left - DC_OFFSET);
			encState = ENC_getState(&encoder);

			if (pastEncState != encState)
				break;
		}

	}
	/*
	while (!(ENC_buttonPressed(encState)))
	{
		encState = ENC_getState(&encoder);
		xil_printf("%d\n\r", encState);
		ProcessSignal();
		usleep(200000);
	}*/
	xil_printf("\n\rCleaning up\n\r");
	FreeMemory();
	CleanupOLED(&oled);

}

void UpdateStates()
{
	// Click Handling
	if (ENC_buttonPressed(encState) && !ENC_buttonPressed(pastEncState))
		inSlider = ~inSlider;

	diff = ENC_getRotation(encState, pastEncState);

	//Rotation Handling
	if (inSlider)
	{
		sliderVals[menuState] += (s8)diff;

		// if unsigned overflow or passed boundary
		if (sliderVals[menuState] > SLIDER_MAX)
		{
			if (diff > 0)
			{
					sliderVals[menuState] = SLIDER_MAX;
			}
			else if (diff < 0)
			{
					sliderVals[menuState] = 0;
			}

		}

		if (menuState == DELAY_IDX)
		{
			memset(delayFIFOLeft, 0, delayFIFOSize);
			memset(delayFIFORight, 0, delayFIFOSize);

			delayFIFOSize = (u32)(((float)sliderVals[DELAY_IDX])/SLIDER_MAX * DELAY_MAX_FIFO_SAMPLES);


			delayFIFOReadIdx = delayFIFOSize / 2;
			delayFIFOWriteIdx = 0;

		}
		else if (menuState == FLANGER_IDX)
		{
			memset(flangerFIFOLeft, 0, flangerFIFOSize);
			memset(flangerFIFORight, 0, flangerFIFOSize);

			flangerFIFOSize = (u32)(((float)sliderVals[FLANGER_IDX])/SLIDER_MAX * FLANGER_MAX_FIFO_SAMPLES);


			flangerFIFOReadIdx = flangerFIFOSize / 2;
			flangerFIFOWriteIdx = 0;

		}
	}

	else
	{
		menuState += diff;

		if (diff > 0)
		{
			while (menuState > EFFECT_COUNT -1)
				menuState -= EFFECT_COUNT;
		}

		else if (diff < 0)
		{
			while (menuState < 0)
				menuState += EFFECT_COUNT;
		}
	}
}

void UpdateOLED()
{
	OLED_ClearBuffer(&oled);
	OLED_SetCharUpdate(&oled, 1);

	if (inSlider)
	{
		OLED_SetCursor(&oled, 2, 0);
		OLED_PutString(&oled, labels[menuState]);

		OLED_SetCharUpdate(&oled, 0);

		OLED_MoveTo(&oled, 27/2, 16);
		OLED_FillRect(&oled, 27/2 +sliderVals[menuState]*100/SLIDER_STATES, 31);
		OLED_DrawRect(&oled, 27/2 + 100, 31);

	}
	else
	{
		OLED_SetCursor(&oled, 0, 1);
		OLED_PutString(&oled, labels[menuState]);
		OLED_SetCursor(&oled, 6, 2);
		OLED_PutString(&oled, scrollText);
		OLED_SetCursor(&oled, 6, 3);
		OLED_PutString(&oled, selectText);
	}

	OLED_Update(&oled);
}


void ApplyEffects()
{
	for (i = 0; i < EFFECT_COUNT; i++)
	{
		if (sliderVals[i] != 0)
		{
			effectFuncs[i]();
		}
	}

}

void Distortion()
{
	fxVal = sliderVals[i];

	temp_left = pcm2double(in_left);
	temp_right = pcm2double(in_right);

	temp_left = tanh(distortionGains[i] * temp_left) * invDistortionGains[fxVal] * distortionWetMixes[fxVal];
	temp_right = tanh(distortionGains[i] * temp_right) * invDistortionGains[fxVal] * distortionWetMixes[fxVal];

	in_left = (u32)(temp_left * DC_OFFSET + DC_OFFSET + in_left * (1-distortionWetMixes[fxVal]));
	in_right = (u32)(temp_right * DC_OFFSET + DC_OFFSET + in_right * (1-distortionWetMixes[fxVal]));

}

void Bitcrusher()
{
	const u32 bitcrush = bitcrushVals[sliderVals[i]];
	in_left = (u32)(((u64)in_left / bitcrush) * bitcrush);
	in_right = (u32)(((u64)in_right / bitcrush) * bitcrush);


}



void circular_read(u32* lbuffer, u32*rbuffer, u32* readIdxPtr, u32 bufferSize, u32* lValPtr, u32* rValPtr)
{
    *lValPtr = lbuffer[*readIdxPtr];
    *rValPtr = rbuffer[*readIdxPtr];
    (*readIdxPtr)++;
    if (*readIdxPtr >= bufferSize)
        *readIdxPtr = 0;  // Changed from -= bufferSize
}

void circular_write(u32* lbuffer, u32* rbuffer, u32* writeIdxPtr, u32 bufferSize, u32 lVal, u32 rVal)
{
    lbuffer[*writeIdxPtr] = lVal;
    rbuffer[*writeIdxPtr] = rVal;
    (*writeIdxPtr)++;
    if (*writeIdxPtr >= bufferSize)
        *writeIdxPtr = 0;  // Changed from -= bufferSize
}

void SuperFlanger()
{
	if (!full)
	{
		circular_write(delayFIFOLeft, delayFIFORight, &delayFIFOWriteIdx, delayFIFOSize,in_left,in_right);
		if (delayFIFOWriteIdx == 0)
			full = 1;
	}
	else
	{
		circular_read(delayFIFOLeft, delayFIFORight, &delayFIFOReadIdx, delayFIFOSize,&in_left,&in_right);
		if (delayFIFOReadIdx == 0)
			full = 0;
	}
}

void Flanger()
{
	const double initial_left = in_left;
	const double initial_right = in_right;

	circular_write(flangerFIFOLeft, flangerFIFORight, &flangerFIFOWriteIdx, flangerFIFOSize,in_left,in_right);
	circular_read(flangerFIFOLeft, flangerFIFORight, &flangerFIFOReadIdx, flangerFIFOSize,&in_left,&in_right);

	in_left += initial_left;
	in_right += initial_right;

}

void SineMult()
{
	XNco_Set_step_size_V(&Nco, sliderVals[i]);

	/* Receive sinusoidal sample from NCO core */
	const u32 nco_out = XNco_Get_sine_sample_V(&Nco);

	in_left = double2pcm(pcm2double(in_left)*pcm2double(nco_out));
	in_right = double2pcm(pcm2double(in_right)*pcm2double(nco_out));

}

void Setup()
{
	xil_printf("\n\rRunning Setup\n\r");

	// POPULATE LUT
	for (i = 0; i < SLIDER_STATES; i++)
	{
		double ratio = (((double)i)/SLIDER_MAX);
		distortionGains[i] = ratio*(DISTORTION_MAX_GAIN -1) + 1;
		invDistortionGains[i] = 1/distortionGains[i];
		distortionWetMixes[i] = ratio*DISTORTION_MAX_WET_MIX;

		bitcrushVals[i] = (u32)(ratio*MAX_BITCRUSH);
	}

	xil_printf("Lookup Tables Generated\n\r");

	AllocMemory();

	xil_printf("Delay Memory Allocated");

	SetupOLED(&oled);
	SetupEncoder(&encoder);
	pastEncState = ENC_getState(&encoder);

	xil_printf("OLED and ENC configured\n\r");

	//Configure the IIC data structure
	IicConfig(XPAR_XIICPS_0_DEVICE_ID);
	//Configure the Audio Codec's PLL
	AudioPllConfig();

	xil_printf("SSM2603 configured\n\r");

	/* Initialise GPIO and NCO peripherals */
	nco_init(&Nco);
	gpio_init();

	xil_printf("NCO peripheral configured\r\n");

	XGpio_DiscreteWrite(&Gpio_audio_enable, 1, 1);

	xil_printf("Audio Enabled\r\n");

	OLEDGreet();

}

void OLEDGreet()
{

	pat = OLED_GetStdPattern(1);
	OLED_SetFillPattern(&oled, pat);
	// Turn automatic updating off
	OLED_SetCharUpdate(&oled, 0);

	// Draw a rectangle over writing then slide the rectangle down slowly
	// displaying all writing
	for (int irow = 0; irow < OledRowMax; irow++)
	{
		OLED_ClearBuffer(&oled);
		OLED_SetCursor(&oled, 0, 0);
		OLED_PutString(&oled, "MultiPedal");
		OLED_SetCursor(&oled, 0, 1);
		OLED_PutString(&oled, "made by");
		OLED_SetCursor(&oled, 0, 2);
		OLED_PutString(&oled, "Atharv Thakur");

		OLED_MoveTo(&oled, 0, irow);
		OLED_FillRect(&oled, 127, 31);
		OLED_MoveTo(&oled, 0, irow);
		OLED_LineTo(&oled, 127, irow);
		OLED_Update(&oled);
		usleep(20000);
	 }
	pat = OLED_GetStdPattern(1);
	OLED_SetFillPattern(&oled, pat);
	sleep(1);

}

void AllocMemory()
{
	delayFIFOLeft = (u32*)malloc(DELAY_MAX_FIFO_SAMPLES * sizeof(u32));
	delayFIFORight = (u32*)malloc(DELAY_MAX_FIFO_SAMPLES * sizeof(u32));

	flangerFIFOLeft = (u32*)malloc(FLANGER_MAX_FIFO_SAMPLES * sizeof(u32));
	flangerFIFORight = (u32*)malloc(FLANGER_MAX_FIFO_SAMPLES * sizeof(u32));
}

void FreeMemory()
{
	free(flangerFIFOLeft);
	free(flangerFIFORight);
	free(flangerFIFOLeft);
	free(flangerFIFORight);
}


unsigned char gpio_init()
{
	int Status;

	Status = XGpio_Initialize(&Gpio, BUTTON_SWITCH_ID);
	if(Status != XST_SUCCESS) return XST_FAILURE;
	Status = XGpio_Initialize(&Gpio_audio_enable, AUDIO_ENABLE_ID);
	if(Status != XST_SUCCESS) return XST_FAILURE;

	XGpio_SetDataDirection(&Gpio_audio_enable, 1, 0x00);
	XGpio_SetDataDirection(&Gpio, SWITCH_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&Gpio, BUTTON_CHANNEL, 0xFF);

	return XST_SUCCESS;
}

/* ---------------------------------------------------------------------------- *
 * 								nco_initi()									*
 * ---------------------------------------------------------------------------- *
 * Initialises the NCO driver by looking up the configuration in the config
 * table and then initialising it.
 * ---------------------------------------------------------------------------- */
void nco_init(void *InstancePtr){
	XNco_Config *cfgPtr;
	int status;

	/* Initialise the NCO driver so that it's ready to use */

	// Look up the configuration in the config table
	cfgPtr = XNco_LookupConfig(NCO_ID);
	if (!cfgPtr) {
		print("ERROR: Lookup of NCO configuration failed.\n\r");
	}

	// Initialise the NCO driver configuration
	status = XNco_CfgInitialize(InstancePtr, cfgPtr);
	if (status != XST_SUCCESS) {
		print("ERROR: Could not initialise NCO.\n\r");
	}
}

inline double pcm2double(u32 val)
{
	return ((double)val - DC_OFFSET) * INV_DC_OFFSET;
}

inline u32 double2pcm(double val)
{
	return (u32)(val*DC_OFFSET + DC_OFFSET);
}
