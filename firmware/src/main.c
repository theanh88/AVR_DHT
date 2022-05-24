/*
 * main.c
 *
 *  Created on: Apr 30, 2022
 *      Author: Group 3
 */
#define F_CPU 7372800UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/iom128.h>
#include "DHT.h"
#include "lcd.h"

// Output Port pin duty cycle
#define PORT_MOTOR_DUTY      PORTE
#define DDR_MOTOR_DUTY       DDRE
#define BIT_MOTOR_DUTY        4

// Output Port pin1 motor
#define PORT_MOTOR_1      PORTE
#define DDR_MOTOR_1       DDRE
#define BIT_MOTOR_1         3

// Output Port pin2 motor
#define PORT_MOTOR_2      PORTE
#define DDR_MOTOR_2       DDRE
#define BIT_MOTOR_2         5

// Define frequency
#define PWM_MAX_DUTY_CYCLE 0x3FF

void PWM_vInit(void) {
	/*
	 Start Timer 1 with clock prescaler CLK/8 and phase correct
	 10-bit PWM mode. Output on PB6 (OC1B). Resolution is 1.09 us.
	 Frequency is 450 Hz.
	 */

	TCCR3A = (0<<COM3A1)|(0<<COM3A0)|(1<<COM3B1)|(0<<COM3B0)
				|(0<<COM3C1)|(0<<COM3C0)|(1<<WGM31)|(1 << WGM30);

	TCCR3B = (0<<ICNC3)|(0<<ICES3)|(0<<WGM33)|(0<<WGM32)
				|(0<<CS32)|(1<<CS31)|(0<<CS30);

	// Reset counter
	TCNT3 = 0;//


	// Set duty cycle to 0%
	OCR3B = 0;

}

void PWM_vSetDutyCycle(uint16_t u16DutyCycle) {
	// Clip parameter to maximum value
	if (u16DutyCycle > PWM_MAX_DUTY_CYCLE) {
		u16DutyCycle = PWM_MAX_DUTY_CYCLE;
	}
	OCR3B = u16DutyCycle;
}
void Start(){
	//start motor
	PORT_MOTOR_1 |= (1 << BIT_MOTOR_1);
	PORT_MOTOR_2 &= ~(1 << BIT_MOTOR_2);
}

void Stop()
{
	PORT_MOTOR_1 &= ~(1 << BIT_MOTOR_1);
	PORT_MOTOR_2 &= ~(1 << BIT_MOTOR_2);
}
void init_IO()
{

    DDR_MOTOR_DUTY |= (1 << BIT_MOTOR_DUTY);
    DDR_MOTOR_1 |= (1 << BIT_MOTOR_1);
    DDR_MOTOR_2 |= (1 << BIT_MOTOR_2);
}
int main(void)
{
	// Initialise port IO
	init_LCD();

	// Initialise port IO
	init_IO();

	// Initialise PWM
	PWM_vInit();

	// Set duty cycle
	PWM_vSetDutyCycle(1000);

	//Variables
	double temperature;
	double humidity;

	//Setup
	DHT_Setup();

	//Loop
	while (1)
	{
		clr_LCD();
		//Read from sensor
		DHT_Read(&temperature, &humidity);

		//Check status
		switch (DHT_GetStatus())
		{
			case (DHT_Ok):
				//Print temperature
				//print(temperature[0]);
				move_LCD(1, 1);
				printf_LCD("Temp: %0.1f ", temperature);

				//Print humidity
				//print(humidity[0]);
				move_LCD(2, 1);
				printf_LCD("Rh: %0.1f ", humidity);
				if(temperature > 27)
				{
						Start();
				}
				else
				{
						Stop();
				}

				break;
			case (DHT_Error_Checksum):
				//Do something
				printf_LCD("Error_Checksum");
				break;
			case (DHT_Error_Timeout):
				//Do something else
				printf_LCD("Error_Timeout");
				break;
			case (DHT_Error_Temperature):
				//Do something
				printf_LCD("Error_Temperature");
				break;
			case (DHT_Error_Humidity):
				//Do something else
				printf_LCD("Error_Humidity");
				break;
		}

		_delay_ms(500);
	}

	return 0;
}

