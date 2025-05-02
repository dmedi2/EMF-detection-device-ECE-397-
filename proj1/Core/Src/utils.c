/*
 * utils.c
 *
 *  Created on: Feb 28, 2025
 *      Author: citylion
 */
#include "main.h"
#include <stdio.h>
#include "display.h"
#include "text.h"
#include "gps.h"
#include "stm32g4xx_hal.h"
#include "stdlib.h"
#include <string.h>
#include <stdarg.h> //for va_list var arg functions

/*


#include "display.h"
#include "gps.h"
#include "text.h"
#include "utils.h"

 */

static unsigned int counter = 0;

static _Bool adcError = 0;

static unsigned int adcRaw = -1;
static float adcVolt = -1;

static float rxPwr = -1;

static unsigned short adcDatCounter = 0;
//static short historyLength = 10;
static float rxPwrHistory[300];

static float rxAvg = -1;

static void dispJammed(){
	origin_set(0,0);
	writestr("GPS - NO LOCK");
}

static void dispGPS(void)
{
    char buf[32];
    origin_set(0, 0);
    snprintf(buf, sizeof(buf), "%.3f %.3f", getLat(), getLon());
    writestr(buf);
}

static void dispRx(void)
{
    char buf[24];
    origin_set(0, 24);
    snprintf(buf, sizeof(buf), "PWR RX: %.2f DBM", rxAvg);
    writestr(buf);
}

static void dispMsg(){
	if(rxAvg > -8){//rxAvg > -8dBm "high" power signal
		char buf[24];
		origin_set(0,42);
		snprintf(buf,sizeof(buf), "DRONE ECM PRESENT");
		writestr(buf);
	}
}



static void pollAdc(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	adcRaw = HAL_ADC_GetValue(&hadc1);
	HAL_StatusTypeDef status = HAL_ADC_Stop(&hadc1);
	if(status == HAL_OK){
		adcVolt = 3.3 * ((float) adcRaw / (float) 4096);
		//original formula using excel "ATTEN (6dB) FILT (2.4GHz)"
		//rxPwr = -43.22*adcVolt + 47.53;

		//new formula using excel "f = 2400 MHz" and offset 6
		rxPwr = -57.218*adcVolt + (48.282 + 6);
		adcError = 0;
	}
	else{
		adcError = 1;
	}

}

static void measureRF(){
	pollAdc();
	int arrSize = sizeof(rxPwrHistory) / sizeof(rxPwrHistory[0]);

	if(adcDatCounter >= arrSize){
		adcDatCounter=0;
	}

	rxPwrHistory[adcDatCounter] = rxPwr;
	adcDatCounter++;
}

//low pass digital filtering algorithm
static void calcRFAvg(){
	int arrSize = sizeof(rxPwrHistory) / sizeof(rxPwrHistory[0]);
	float a = 0;
	for(int i=0; i<arrSize; i++){
		a += rxPwrHistory[i];
	}
	rxAvg = a/arrSize;
}




void maincall(void){

	counter++;
	//write to the pointer, giving us any message received on uart

	//every 100 calls we will update the gps display
	if(counter%100 == 0){
		calcRFAvg();

		buffer_clear();

		if(gps_isJammed()){
			dispJammed();
		}
		else{
			dispGPS();
		}

		dispMsg();
		dispRx();//display received rf power
		display_update();
	}
	else if(counter % 10212 == 0){
		display_clear();
	}
	measureRF();
}











