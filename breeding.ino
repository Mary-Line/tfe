#include "breeding.h"
#include<stdio.h>

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SDAPIN  20
    #define SCLPIN  21
    #define RSTPIN  7
    //#define SERIAL SerialUSB
#else
    #define SDAPIN  A4
    #define SCLPIN  A5
    #define RSTPIN  2
    //#define SERIAL Serial
#endif

SHT35 sensor(SCLPIN);
Multi_Channel_Relay relay;
Water_Level water_level;

int serialspeed = 9600;

void setup() {
  SERIAL_DB.begin(serialspeed);
  delay(10);
  if (sensor.init()) {
    SERIAL_DB.println("sensor init failed!!!");
  }
  delay(1000);
}

void loop() 
{
	fct_check_parameters();
	fct_read_from_serial_port();
	//fct_check_water_level();
	delay(2000);
}

void fct_check_parameters()
{
	u16 value = 0;
	u8 data[6] = {0};
	float temp, hum;
	int water = 0;
	String message ="";
	delay(1000);
	if (NO_ERROR  != sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum))
	{
		SERIAL_DB.println("read temp failed!!");
	}	
	else
	{
		message = message + "{\"humidity\": ";
		message = message + hum;
		message = message + ", \"temperature\": ";
		message = message + temp;
		//message = message + "}";	
	}
	
	water_level.begin();
	water = water_level.check_water_level();
	message = message + ", \"level\": ";
	message = message + water;
	message = message + "}";
	SERIAL_DB.println(message);
}

void fct_read_from_serial_port()
{	
	String str_from_serial;
	
	int channel_relay;
	int channel_port_temp;
	int channel_port_temp_status;
	int channel_port_hum;
	int channel_port_hum_status;
	
	if (SERIAL_DB.available()) 
	{
    	str_from_serial = SERIAL_DB.readStringUntil('\n');
		channel_relay = (str_from_serial.substring(0,2)).toInt();
		channel_port_temp = (str_from_serial.substring(3,4)).toInt();
		channel_port_temp_status = (str_from_serial.substring(5,6)).toInt();
		channel_port_hum = (str_from_serial.substring(7,8)).toInt();
		channel_port_hum_status = (str_from_serial.substring(9,10)).toInt();
		
		/*SERIAL_DB.println(str_from_serial);
		SERIAL_DB.println(channel_relay);
		SERIAL_DB.println(channel_port_temp);
		SERIAL_DB.println(channel_port_temp_status);
		SERIAL_DB.println(channel_port_hum);
		SERIAL_DB.println(channel_port_hum_status);*/
		
		relay.begin(channel_relay);
		if (channel_port_temp_status == 0)
		{
			relay.turn_off_channel(channel_port_temp);
		}
		else
		{
			relay.turn_on_channel(channel_port_temp);
		}
		if (channel_port_hum_status == 0)
		{
			relay.turn_off_channel(channel_port_hum);
		}
		else
		{
			relay.turn_on_channel(channel_port_hum);
		}
	}
}



