/*
Elisabeth Mark
400522900 - marke3
COMPENG 2DX3
Project Deliverable 2
Submitted: March 31, 2025
*/


#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

//ENUMS 
typedef enum {
	CW,
	CCW
} RotDirection;

typedef enum {
	STEP_OFF,
	STEP_ON
} StepState;

typedef enum {
	SCAN_OFF,
	SCAN_ON
} ScanState;

typedef struct {
	RotDirection dir;
	StepState step_state;
	ScanState scan_state;
} StatusParameters;

// Global variables used as flags that are manipulated by the interrupts
volatile StatusParameters measurementSys = {
		.dir = CW,
		.step_state = STEP_OFF,
		.scan_state = SCAN_OFF,
};

/*
Initialize Port H pins for stepper motor output
*/
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};         
	GPIO_PORTH_DIR_R = 0b00001111;       
  GPIO_PORTH_DEN_R = 0b00001111;  
	return;
}

// Give clock to Port J and initalize as input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
		GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 & PJ0 input 
		GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
		GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								// Configure PJ1 & PJ0 as GPIO 
		GPIO_PORTJ_AMSEL_R &= ~0x03;											// Disable analog functionality on PJ1 & PJ0	
		GPIO_PORTJ_PUR_R |= 0x03;													// Enable weak pull up resistor
}

/*
Initialize Port M, pin 0 as an output pin, this is for showing the bus frequency
*/
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; // activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; // allow for clock to stabilize
    GPIO_PORTM_DIR_R = 0b00000001;
    GPIO_PORTM_DEN_R = 0b00000001; // Enable pins
	return;
}

/*
Initialize I2C 
*/
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3

		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
        
}

// Enable interrupts
void EnableInt(void)
{    
	__asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    
	__asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    
	__asm("    wfi\n");
}

// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
		GPIO_PORTJ_IS_R = 0;     						// (Step 1) PJ1 & PJ0 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;    						//     			PJ1 & PJ0 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;    						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x03;      			// 					PJ1 & PJ0 interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x03;      				// 					Arm interrupt on PJ1 & PJ0 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;            // (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000; 					// (Step 4) Set interrupt priority to 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}

// Interrupt handler for GPIO J
void GPIOJ_IRQHandler(void){	        
    //"ready mode" to scan
    if((GPIO_PORTJ_DATA_R&0x01) == 0) { //Button PJ0 enables scanning and transmitting
			measurementSys.scan_state = !measurementSys.scan_state; 					 // Toggle scanning
			ToggleLED4(1);												// toggle light when ready to scan
			GPIO_PORTJ_ICR_R = 0x01;     					 // Acknowledge flag by setting proper bit in ICR register
    }
		
		//moving mode
		if((GPIO_PORTJ_DATA_R&0x02) == 0) { 		 //Button PJ1 enables stepper motor rotation
			measurementSys.step_state = !measurementSys.step_state; 										 // Toggle rotation flag
			GPIO_PORTJ_ICR_R = 0x02;     					 // Acknowledge flag by setting proper bit in ICR register
    }
    return;
}

// Controls the rotation of the stepper motor. 
// input number of full steps
void rotationControl(int direction, int delay){
	// CCW
	if(direction == CCW){
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1ms(delay);

	}
	
	// CW
	else if(direction == CW){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1ms(delay);

	}

}

/*
* Called every 5.625 degrees to poll the sensor for a distance measurement. Verifies ToF status over I2C, then reads data. 
* Transmits data to the PC over UART. 
*/
void scanBegin(int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus, int depth, int count){		
		
		SysTick_Wait10ms(10); 
			//wait until data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						FlashLED3(1); //measurement status flash
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			//read sensor data
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					// distance
			status = VL53L1X_GetSignalRate(dev, &SignalRate);
			status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			status = VL53L1X_GetSpadNb(dev, &SpadNum);

			status = VL53L1X_ClearInterrupt(dev); // clear the interrupt
			//cw rotation
			int indicationOfStepperMotorPos = count;
			
			//initially, the depth going cw is even, therefore ccw is odd
			if (measurementSys.dir == CCW) { //if you put a protractor on the motor, a particular point would be the same angle regardless of it rotating cw or ccw
				indicationOfStepperMotorPos = 512-count;
			}
			// transmit data
			FlashLED1(1); // UART TX Flash
			//the indicationOfStepperMotorPos is the "angle" sent to matlab where it is actually converted to an angle
			sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, indicationOfStepperMotorPos, depth, SpadNum);
			UART_printf(printf_buffer);
			SysTick_Wait10ms(5);
		
}


/*
* Loop that controls scanning and stepper rotation. Flashes and LED and scans every 5.625 degrees. Upon completing a rotation, it resets the count and swaps direction.
*/
void measurementControl(int steps, int delay, int count, int depth,int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus ){
	while(1){
		if (measurementSys.step_state == STEP_ON){
			//do step (4 motor phases)
			rotationControl(measurementSys.dir, delay);
			count++;
		}
		
		// Take a reading every 5.625 degrees. 360 deg / 512 steps = 0.703125 deg/step. 5.625 deg/0.703125 deg/step = 8 steps. 
		// Therefore, every multiple of 8 steps will be a multiple of 5.625 degrees.
		if (count % 8 == 0 && measurementSys.step_state == STEP_ON && count != 0){
			//take reading if count is a multiple pf 8 (motor went 5.625 deg)
			if (measurementSys.scan_state){
				FlashLED4(1); //additional status light
				
				scanBegin(status, dev, dataReady, Distance, SignalRate, AmbientRate, SpadNum, RangeStatus, depth, count); // Take reading
			}
		}

		// swap direction and iterate the depth
		//done full rotation
		if (count > 512){
			//stop moving
				measurementSys.step_state = STEP_OFF;
			//reset count
				count = 0;
			//switch dir so the wires dont get tangled
				measurementSys.dir = !measurementSys.dir;
			//add an "x" value/new plane indication
				depth += 1;
		}
	}
}


/*
* Function to validate the microcontroller's bus speed. Probing PM0 with an osillyscope will produce a waveform of with period 2 ms.
*/
void busFreqTest(){
	while (1){
		GPIO_PORTM_DATA_R ^= 0b00000001;
		SysTick_Wait1ms(1);
	}
}

/*
* Main function.
*/
int main(void) {
  uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
	int status=0;
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	// Initialize clock, ports, LEDs, GPIOS, and interrupts
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	PortH_Init();
	
	// Bus speed demo
	//comment and uncomment code
	/*
		PortM_Init();
		busFreqTest();
	*/
	

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	//choose this method of measurement
	//chose this one since 1  = short (for indoor/low light from 0 to 1.3m), 2 = long (max 4m).. hallway in y dir is 4m ish
  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */ , how long the sensor spents per measuremnt... longer budget = more accurate results = slower refresh rates
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

	status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	// Initialize variables for stepper motor and measurements
	int steps = 512;
	int delay = 5;
	int count = 0;
	int depth = 0;
	
	while (1){ // Loop infinitely
			if (measurementSys.step_state == STEP_ON &&measurementSys.scan_state == SCAN_ON){ 
				// If the following flag is set we can begin rotating the stepper motor a full rotation
				measurementControl(steps, delay, count, depth,status, dev, dataReady, Distance, SignalRate, AmbientRate, SpadNum, RangeStatus);
			}
	}
	VL53L1X_StopRanging(dev);
  
}