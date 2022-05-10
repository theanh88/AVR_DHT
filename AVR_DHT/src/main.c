/*
 * main.c
 *
 *  Created on: Apr 30, 2022
 *      Author: Group 3
 */
#include <avr/io.h>
#include <util/delay.h>
#include "DHT.h"
#include "lcd.h"

int main(void)
{
	init_LCD();

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
				printf_LCD("Temp: %d .doC", temperature);

				//Print humidity
				//print(humidity[0]);
				move_LCD(2, 1);
				printf_LCD("Rh: %d .%", humidity);

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

		//Sensor needs 1-2s to stabilize its readings
		_delay_ms(1000);
	}

	return 0;
}

