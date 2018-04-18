/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 //  Thanarajasegaran Subramaniam for Dr. Yevgen Biletskiy, PEng
//	 This code is protected until 11: 59PM at June 30th, 2017 as is required, while the code is under review by 
//	 the Department of Electrical and Computer Engineering.

//   Use of code allowed after the date, with prior approval from creator.

//Automated Tracking Controller to track the movement of Sunlight throughout the day. This program utilizes the use of sensors, 
//as this project is a proof of concept for tracking controllers and automation. Currently, the system is a horizontal motion only
// and a vertical line of motion can be added if a second stepper motor can be added. 4 sensors are currently included in the code, but sensors
// 2&3 are commented. The system has a cut off value set at 45000, if this cannot be found, the system will assume that there isnt a sufficient 
//light source
//
//
//Thanarajasegaran Subramaniam -All rights reserved.

#include "MK64F12.h"
#include <stdio.h>
#include <string.h>

void init(); 			

int ADC_Convert(void); 
int ADC_Convert2(void);
int ADC_Convert3(void);
int ADC_Convert4(void);

int main(void)
{	init();							// Begin initialization of initialization code
	
	int convert; 					// These integers will hold the conversion values of light values found by the sensors
	int convert2;
	int convert3;
	int convert4;
	
	while(1){
	convert=ADC_Convert();			// check light
	convert2=ADC_Convert2();
	convert3=ADC_Convert3();
	convert4=ADC_Convert4();
	delay_shortest(); 				// sensor has a slow response time, need to delay and recheck
	convert=ADC_Convert();
	convert2=ADC_Convert2();
	convert3=ADC_Convert3();
	convert4=ADC_Convert4();

	
	while(!(convert>=45000 && convert4>=45000)){ // Cutoff Value for the light sensors, anything less than this and the light is not sufficient to continue

	convert=ADC_Convert(); 			// Keep checking, values may change in between
	convert2=ADC_Convert2();
	convert3=ADC_Convert3();
	convert4=ADC_Convert4();
	
	
    // If sensor one has detected more Sunlight, go into the while loop
	
	if(convert>convert4){			// && convert>convert3 && convert>convert2){ (Enable this if you want 4 sensor functionality)
	
    while(convert <= 45000){ 		//Look for the position where this value of the light sensor can be reached, move towards sensor that has most light
		GPIOB_PCOR = 0x00000800; 	// clear the GPIO set to move the motor towards sensor 4, if it happened already
		GPIOB_PSOR = 0x00000400; 	//keep moving motor towards sensor 1
		
		convert=ADC_Convert(); 		// Recheck adc values
		convert2=ADC_Convert2();
		convert3=ADC_Convert3();
		convert4=ADC_Convert4();
		
		if(convert4>convert)		// || convert3>convert ||convert2>convert) (Enable this if you want 4 sensor functionality) 
			break; 					// Break out of while if another sensor registers a higher value

}GPIOB_PCOR = 0x00000400;  			// Stop moving motor towards Sensor 1
	}
	
	// Code for sensors 2 & 3 listed below (not used in this system, but working, this will need a 2nd motor for full functionality)
	
	/*else if(convert2>convert && convert2>convert3 && convert2>convert4){
		while(convert2 <=45000){
			GPIOB_PCOR = 0x00000400;
			GPIOB_PSOR = 0x00000800;
			convert=ADC_Convert();
			convert2=ADC_Convert2();
			convert3=ADC_Convert3();
			convert4=ADC_Convert4();
			if(convert4>convert2 || convert3>convert2 || convert>convert2)
				break;
		}GPIOB_PCOR = 0x00000800;
	}

	else if(convert3>convert && convert3>convert2 && convert3>convert4){
		while(convert3 <=45000){
				GPIOB_PCOR = 0x00000800;
				GPIOB_PSOR = 0x00000400;
				convert=ADC_Convert();
				convert2=ADC_Convert2();
				convert3=ADC_Convert3();
				convert4=ADC_Convert4();
				if(convert4>convert3 || convert2>convert3 || convert>convert3)
					break;
	}GPIOB_PCOR = 0x00000400;
	}*/


	else if(convert4>convert){			// && convert4>convert2 && convert4>convert3){  Enable rest of converts for 4 sensor functionality
		while(convert4 <= 45000){ 		//Look for the position where this value of the light sensor can be reached, move towards sensor with most light
		
		GPIOB_PCOR = 0x00000400;		// Clear the GPIO for motor movement towards sensor 1
		GPIOB_PSOR = 0x00000800;		//Start moving towards sensor 4
		
		convert = ADC_Convert(); 		// Recheck values
		convert2=ADC_Convert2();
		convert3=ADC_Convert3();
		convert4 = ADC_Convert4();
		
		if(convert>convert4)			// || convert2>convert4 || convert3>convert4) (Enable rest of converts for 4 sensor functionality)
			break; 						// Break out of this loop if sensor 4 finds a stronger light source
}GPIOB_PCOR = 0x00000800;
	}

}

	}}



void init() 
{
	SIM_SCGC5 |=SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTB_MASK;
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART1_MASK;// Clock for UART
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	SIM_SCGC6 |= SIM_SCGC6_DAC0_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK; 	//Enable the FTM0 clock, this is not used in this project.
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /*Enable the PORTD clock*/
	SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK;
	SIM_SCGC2 |= SIM_SCGC2_DAC1_MASK;
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK ;



	PORTB_PCR17 |= PORT_PCR_MUX(3); //UART0 Transmit, UART Transmit and Receive is not used in this program
	PORTB_PCR16 |= PORT_PCR_MUX(3); //UART0 Receive

	PORTC_PCR3 |= PORT_PCR_MUX(1); //Motor

	/*UART0 Stuff- unused in this code, but can be renabled if you want to display something in Putty
	
	UART0_C2 |=0x0;// Initialization of transmit and receive
	UART0_C1 |=0x0;// UART0 for 8 bits

	UART0_BDH=0;
	UART0_BDL=0x88;

	UART0_C2 |= UART_C2_TE_MASK;
	UART0_C2 |= UART_C2_RE_MASK;*/
	
	//Just ADC things
	ADC_Init(); // INitialize ADC 
	
	// GPIO Things
	PORTB_PCR10|=PORT_PCR_MUX(1); // direction control for left motor
	PORTB_PCR11|=PORT_PCR_MUX(1); // direction control for right motor
	GPIOB_PDDR=0x00000c00; // Set this pin as output
	


	//DAC Enable and DAC Reference
	DAC0_C0 = 0xF0;
	DAC0_C1 = 0x00;
}

void ADC_Init() 
{   // PTB2
	SIM_SCGC6|= SIM_SCGC6_ADC0_MASK;   // Enable clock for ADC0
	SIM_SCGC5|= SIM_SCGC5_PORTC_MASK;   //Enable clock for PORT c
	// Configuration
	ADC0_CFG1 = 0b101100; // ADIV=01, ADLSMP=0, Mode=11, ADICLK=0
	ADC0_CFG2 = 0b000000; // MUXSEL=0, ADHSC=0
	ADC0_SC1A = 0b01110; // AIEN=0, DIFF=0, ADCH=01100 (AD ch 12)
	ADC0_SC2 = 0b0000000; // ADACT=1, ADTRG=0, ACFE=0, DMAEN=0, REFSEL=00
	ADC0_SC3 = 0b000000; // CAL=0, ADCO=0
}

int ADC_Convert(void) 
{	// Conversion
	int convert;
	ADC0_SC1A = 0b01110; // AIEN=0, DIFF=0, ADCH=01110 (AD ch)
	while (!(ADC0_SC1A & 0x80)); // exits the loop when ADC0_SC1A=0x8e
	convert = ADC0_RA; // COCO
	return convert;
}

int ADC_Convert2(void) 
{	// Conversion
	int convert2;
	ADC0_SC1A = 0b01100; // AIEN=0, DIFF=0, ADCH=01100 (AD ch)
	while (!(ADC0_SC1A & 0x80)); // exits the loop when ADC0_SC1A=0x8e
	convert2 = ADC0_RA; // COCO
	return convert2;
}

int ADC_Convert3(void) 
{	// Conversion
	int convert3;
	ADC0_SC1A = 0b01101; // AIEN=0, DIFF=0, ADCH=01101 (AD ch))
	while (!(ADC0_SC1A & 0x80)); // exits the loop when ADC0_SC1A=0x8e
	convert3 = ADC0_RA; // COCO
	return convert3;
}

int ADC_Convert4(void) 
{	// Conversion
	int convert4;
	ADC0_SC1A = 0b01111; // AIEN=0, DIFF=0, ADCH=01111 (AD ch)
	while (!(ADC0_SC1A & 0x80)); // exits the loop when ADC0_SC1A=0x8e
	convert4 = ADC0_RA; // COCO
	return convert4;
}

void delay_shortest()
{
	int c=1,d=1;
	for (c=1;c<=32767;c++){
 		for (d=1;d<=10;d++){}
}}

}


