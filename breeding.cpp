#include "breeding.h"

/*Control SHT35 device */
/*-------------------- */

SHT35::SHT35(u8 scl_pin, u8 IIC_ADDR) {
    set_iic_addr(IIC_ADDR);
    set_scl_pin(scl_pin);
    CLK_STRCH_STAT = CLK_STRETCH_DISABLE;
}

err_t SHT35::init() {
    err_t ret = NO_ERROR;
    IIC_begin();
    ret = soft_reset();
    return ret;
}

err_t SHT35::soft_reset() {
    err_t ret = NO_ERROR;
    ret = send_command(CMD_SOFT_RST);
    return ret;
}

err_t SHT35::read_meas_data_single_shot(u16 cfg_cmd, float* temp, float* hum) {
    err_t ret = NO_ERROR;
    u8 data[6] = {0};
    u16 temp_hex = 0, hum_hex = 0;
    CHECK_RESULT(ret, send_command(cfg_cmd));
    CHECK_RESULT(ret, read_bytes(data, sizeof(data), CLK_STRCH_STAT));

    temp_hex = (data[0] << 8) | data[1];
    hum_hex = (data[3] << 8) | data[4];

    *temp = get_temp(temp_hex);
    *hum = get_hum(hum_hex);

    return ret;
}

float SHT35::get_temp(u16 temp) {
    return (temp / 65535.00) * 175 - 45;
}

float SHT35::get_hum(u16 hum) {
    return (hum / 65535.0) * 100.0;
}

u16 SHT35::temp_to_hex(float temp) {
    return (u16)((temp + 45) * 65535.0 / 175);
}

u16 SHT35::hum_to_hex(float hum) {
    return (u16)(hum / 100.0 * 65535);
}


/******************************************************STATUS REG**************************************************/
/******************************************************STATUS REG**************************************************/

err_t SHT35::read_reg_status(u16* value) {
    err_t ret = NO_ERROR;
    *value = 0;
    u8 stat[3] = {0};
    CHECK_RESULT(ret, send_command(CMD_READ_SREG));
    CHECK_RESULT(ret, request_bytes(stat, sizeof(stat)));
    *value |= (u16)stat[0] << 8;
    *value |= stat[1];
    return ret;
}

err_t SHT35::heaterStatus(u16 status, bool stat) {
    stat = ((status >> 13) & 0x01);
    return NO_ERROR;
}

err_t SHT35::heaterStatus(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((status >> 13) & 0x01);
    return ret;
}
/****************************************************/

err_t SHT35::reset_check(u16 status, bool stat) {
    stat = ((stat >> 4) & 0x01);
    return NO_ERROR;
}

err_t SHT35::reset_check(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((stat >> 4) & 0x01);
    return ret;
}
/****************************************************/

err_t SHT35::cmd_excu_stat(u16 status, bool stat) {
    stat = ((stat >> 1) & 0x01);
    return NO_ERROR;
}

err_t SHT35::cmd_excu_stat(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((stat >> 1) & 0x01);
    return ret;
}
/****************************************************/
err_t SHT35::last_write_checksum(u16 status, bool stat) {
    stat = ((status >> 0) & 0x01);
    return NO_ERROR;
}
err_t SHT35::last_write_checksum(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((stat >> 0) & 0x01);
    return ret;
}

/***********************************************************************************************/
/**************************************EXEC COMMAND*********************************************/

err_t SHT35::change_heater_status(bool stat) {
    err_t ret = NO_ERROR;

    if (stat) {
        ret = send_command(CMD_HEATER_ON);
    } else {
        ret = send_command(CMD_HEATER_OFF);
    }

    return ret;
}

/***********************************************************************************************/
/*****************************************IIC OPRT**********************************************/
u8 SHT_IIC_OPRTS::crc8(const u8* data, int len) {

    const u8 POLYNOMIAL = 0x31;
    u8 crc = 0xFF;

    for (int j = len; j; --j) {
        crc ^= *data++;

        for (int i = 8; i; --i) {
            crc = (crc & 0x80)
                  ? (crc << 1) ^ POLYNOMIAL
                  : (crc << 1);
        }
    }
    return crc;
}

err_t SHT_IIC_OPRTS::send_command(u16 cmd) {
    s32 ret = 0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write((cmd >> 8) & 0xFF);
    Wire.write(cmd & 0xFF);
    ret = Wire.endTransmission();
    if (!ret) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}


err_t SHT_IIC_OPRTS::I2C_write_bytes(u16 cmd, u8* data, u32 len) {
    u8 crc = 0;
    s32 ret = 0;
    crc = crc8(data, len);

    Wire.beginTransmission(_IIC_ADDR);
    Wire.write((cmd >> 8) & 0xFF);
    Wire.write(cmd & 0xFF);
    //Wire.beginTransmission(_IIC_ADDR);
    for (int i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    Wire.write(crc);
    ret = Wire.endTransmission();
    if (!ret) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}

err_t SHT_IIC_OPRTS::request_bytes(u8* data, u16 data_len) {
    err_t ret = NO_ERROR;
    u32 time_out_count = 0;
    Wire.requestFrom(_IIC_ADDR, data_len);
    while (data_len != Wire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    for (int i = 0; i < data_len; i++) {
        data[i] = Wire.read();
    }
    return NO_ERROR;
}

/*SHT3X device is different from other general IIC device.*/
err_t SHT_IIC_OPRTS::read_bytes(u8* data, u32 data_len, clk_skch_t clk_strch_stat) {
    err_t ret = NO_ERROR;
    u32 time_out_count = 0;
    if (clk_strch_stat == CLK_STRETCH_ENABLE) {
        while (0 == digitalRead(SCK_PIN)) {
            yield();
        }
    } else {
        Wire.beginTransmission(_IIC_ADDR);
        while (Wire.endTransmission() == NACK_ON_ADDR) {
            Wire.beginTransmission(_IIC_ADDR);
        }
    }

    Wire.requestFrom(_IIC_ADDR, data_len);
    while (data_len != Wire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    for (int i = 0; i < data_len; i++) {
        data[i] = Wire.read();
    }
    return NO_ERROR;
}


void SHT_IIC_OPRTS::set_scl_pin(u8 scl) {
    SCK_PIN = scl;
}

/** @brief change the I2C address from default.
    @param IIC_ADDR: I2C address to be set
 * */
void SHT_IIC_OPRTS::set_iic_addr(u8 IIC_ADDR) {
    _IIC_ADDR = IIC_ADDR;
}

/*Control relay_board device */
/*-------------------------- */

Multi_Channel_Relay::Multi_Channel_Relay() {

}

void Multi_Channel_Relay::begin(int address) {
    Wire.begin();
    channel_state = 0;
    _i2cAddr = address;

}

uint8_t Multi_Channel_Relay::getFirmwareVersion(void) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(CMD_READ_FIRMWARE_VER);
    Wire.endTransmission();

    Wire.requestFrom(_i2cAddr, 1);
    //while(!Wire.available());
    return Wire.read();
}

void Multi_Channel_Relay::changeI2CAddress(uint8_t old_addr, uint8_t new_addr) {
    Wire.beginTransmission(old_addr);
    Wire.write(CMD_SAVE_I2C_ADDR);
    Wire.write(new_addr);
    Wire.endTransmission();

    _i2cAddr = new_addr;
}


uint8_t Multi_Channel_Relay::getChannelState(void) {
    return channel_state;
}

void Multi_Channel_Relay::channelCtrl(uint8_t state) {
    channel_state = state;

    Wire.beginTransmission(_i2cAddr);
    Wire.write(CMD_CHANNEL_CTRL);
    Wire.write(channel_state);
    Wire.endTransmission();
}

void Multi_Channel_Relay::turn_on_channel(uint8_t channel) {
    channel_state |= (1 << (channel - 1));

    Wire.beginTransmission(_i2cAddr);
    Wire.write(CMD_CHANNEL_CTRL);
    Wire.write(channel_state);
    Wire.endTransmission();
}

void Multi_Channel_Relay::turn_off_channel(uint8_t channel) {
    channel_state &= ~(1 << (channel - 1));

    Wire.beginTransmission(_i2cAddr);
    Wire.write(CMD_CHANNEL_CTRL);
    Wire.write(channel_state);
    Wire.endTransmission();
}

uint8_t Multi_Channel_Relay::scanI2CDevice(void) {
    byte error = 0, address = 0, result = 0;
    int nDevices;

    SERIAL_DB.println("Scanning...");

    nDevices = 0;
    for (address = 1; address <= 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            result = address;
            SERIAL_DB.print("I2C device found at address 0x");
            if (address < 16) {
                SERIAL_DB.print("0");
            }
            SERIAL_DB.print(address, HEX);
            SERIAL_DB.println("  !");

            nDevices++;
        } else if (error == 4) {
            SERIAL_DB.print("Unknown error at address 0x");
            if (address < 16) {
                SERIAL_DB.print("0");
            }
            SERIAL_DB.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        SERIAL_DB.println("No I2C devices found\n");
        result = 0x00;
    } else {
        SERIAL_DB.print("Found ");
        SERIAL_DB.print(nDevices);
        SERIAL_DB.print(" devices\n");
        if (nDevices != 1) {
            result = 0x00;
        }
    }

    return result;
}

/*Control water level device */
/*-------------------------- */

Water_Level::Water_Level() {

}

void Water_Level::begin() 
{
    Wire.begin();
    int _i2cAddr_high = ATTINY1_HIGH_ADDR; //address_high;
	int _i2cAddr_low = ATTINY2_LOW_ADDR; //address_low;
}

int Water_Level::check_water_level(void)
{
	int sensorvalue_min = 250;
	int sensorvalue_max = 255;
	
	int low_count = 0;
	int high_count = 0;
	
	int waterlevel = 0;
	
	while (1)
	{
		uint32_t touch_val = 0;
		uint8_t trig_section = 0;
		low_count = 0;
		high_count = 0;
		getLow8SectionValue();
		getHigh12SectionValue();

		for (int i = 0; i < 8; i++)
		{
			//SERIAL_DB.print(low_data[i]);
			//SERIAL_DB.print(".");
			if (low_data[i] >= sensorvalue_min && low_data[i] <= sensorvalue_max)
			{
				low_count++;
			}
			if (low_count == 8)
			{
			//SERIAL_DB.print("      ");
			//SERIAL_DB.print("PASS");
			}
		}
		for (int i = 0; i < 12; i++)
		{
			if (high_data[i] >= sensorvalue_min && high_data[i] <= sensorvalue_max)
			{
				high_count++;
			}
			if (high_count == 12)
			{
			//        SERIAL_DB.print("      ");
			//SERIAL_DB.print("PASS");
			}
		} 
		for (int i = 0 ; i < 8; i++) 
		{
			if (low_data[i] > THRESHOLD) 
			{
				touch_val |= 1 << i;
			}
		}
		
		for (int i = 0 ; i < 12; i++) 
		{
			if (high_data[i] > THRESHOLD) 
			{
				touch_val |= (uint32_t)1 << (8 + i);
			}
		}
		while (touch_val & 0x01)
		{
			trig_section++;
			touch_val >>= 1;
		}
		return waterlevel = (trig_section * 5);
	}
}
	
void Water_Level::getHigh12SectionValue(void)
	{
	  memset(high_data, 0, sizeof(high_data));
	  Wire.requestFrom(0x78, 12);
	  while (12 != Wire.available());
	 
	  for (int i = 0; i < 12; i++) {
		high_data[i] = Wire.read();
	  }
	  delay(10);
	}
	 
void Water_Level::getLow8SectionValue(void)
	{
	  memset(low_data, 0, sizeof(low_data));
	  Wire.requestFrom(0x77, 8);
	  while (8 != Wire.available());
	 
	  for (int i = 0; i < 8 ; i++) 
	  {
		low_data[i] = Wire.read(); // receive a byte as character
	  }
	  //SERIAL_DB.println("here " + low_data[i]);
	  delay(10);
	}
