
// CORE Code

#include "encoder.h"
#include "xparameters.h"

void SetupEncoder(PmodENC* InstancePtr) {
   ENC_begin(InstancePtr, XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR);
}

// DEMO Code

#if false

#include "sleep.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xil_printf.h"

void DemoInitialize();
void DemoRun();

PmodENC myDevice;

int main(void) {
   SetupEncoder(&myDevice);
   DemoRun();
   return 0;
}




void DemoRun() {
   u32 state, laststate; // Comparing current and previous state to detect edges
   int ticks = 0;        // on GPIO pins

   xil_printf("Running PmodENC Demo\n\r");
   laststate = ENC_getState(&myDevice);

   while (1) {
      state = ENC_getState(&myDevice);

      if (ENC_switchOn(state)) // Using switch as an enable for counter
         ticks += ENC_getRotation(state, laststate);
      else
         ticks = 0;

      // Only print on button posedge
      if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate)) {
         if (ENC_switchOn(state))
            xil_printf("ticks = %d\n\r", ticks);
         else
            xil_printf("tick counter disabled\r\n");
      }
      laststate = state;
      usleep(1000);
   }
}
#endif
