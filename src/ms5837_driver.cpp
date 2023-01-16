#include "ms5837_driver/ms5837_driver.hpp"

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

#include <math.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <bitset>


MS5837Driver::~MS5837Driver() {
    if (fd_ != -1) {
        close(fd_);
    }
}

bool MS5837Driver::init() {
    fd_ = open(port_.c_str(), O_RDWR);

    // Unable to open I2C port
    if (fd_ < 0) {
        std::cerr << "Unable to open I2C port " << port_ << "!" << std::endl;
        return false;
    }

    // Unable to set the adress
	if (ioctl(fd_, I2C_SLAVE, MS5837_ADDR) < 0) {
        std::cerr << "Unable to set the adress!" << std::endl;
        return false;
    }

    // Reset the sensor
    i2c_smbus_write_byte(fd_, MS5837_RESET);

	// Wait for reset to complete
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// Read calibration values and CRC
    u_int8_t buffer[2];
    for (uint8_t i=0 ; i<7 ; ++i) {
        // PROM read
        int32_t len = i2c_smbus_read_i2c_block_data(fd_, MS5837_PROM_READ + 2*i, sizeof(buffer), buffer);
        if (len != sizeof(buffer)) {
            std::cerr << "Error during calibration coefficient read!" << std::endl;
            std::cerr << "Request length of " << sizeof(buffer) << " bytes does not match the length of read bytes " << len << std::endl;
            return false;
        }

        // PROM storage
		C[i] = (buffer[0] << 8) | buffer[1];
	}
    

	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	if ( crcCalculated == crcRead ) {
		return true; // Initialization success
	}
    else {
        std::bitset<4> crcR(crcRead);
        std::bitset<4> crcC(crcCalculated);
        std::cerr << "Error of CRC check!" << std::endl;
        std::cerr << "Read " << crcR << " Calculated " << crcC << std::endl;
	    return false; // CRC fail
    }
}

void MS5837Driver::setFluidDensity(float density) {
	fluidDensity = density;
}

bool MS5837Driver::read_data() {
    int32_t len;
    uint8_t buffer[3];

	// Request D1 conversion
    i2c_smbus_write_byte(fd_, MS5837_CONVERT_D1_8192);

	std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Max conversion time per datasheet

    len = i2c_smbus_read_i2c_block_data(fd_, MS5837_ADC_READ, sizeof(buffer), buffer);
    if (len != 3) {
        std::cerr << "Error in D1 request!" << std::endl;
        std::cerr << "Reading " << len << " instead of 3 bytes!" << std::endl;
        return false;
    }
	D1 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[0];

    // Request D2 conversion
    i2c_smbus_write_byte(fd_, MS5837_CONVERT_D2_8192);

	std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Max conversion time per datasheet

    len = i2c_smbus_read_i2c_block_data(fd_, MS5837_ADC_READ, sizeof(buffer), buffer);
    if (len != 3) {
        std::cerr << "Error in D2 request!" << std::endl;
        std::cerr << "Reading " << len << " instead of 3 bytes!" << std::endl;
        return false;
    }
	D2 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[0];

	calculate();

    return true;
}

void MS5837Driver::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	
	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;  
	int32_t Ti = 0;    
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
	
	// Terms called
	dT = D2-uint32_t(C[5])*256l;
    SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
    OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
    P = (D1*SENS/(2097152l)-OFF)/(8192l);
	
	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
	
	//Second order compensation
    if((TEMP/100)<20){         //Low temp
        Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
        OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
        SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
        if((TEMP/100)<-15){    //Very low temp
            OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
            SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
        }
    }
    else if((TEMP/100)>=20){    //High temp
        Ti = 2*(dT*dT)/(137438953472LL);
        OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
        SENSi = 0;
    }
	
	OFF2 = OFF-OFFi; // Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	TEMP = (TEMP-Ti);
	
    P = (((D1*SENS2)/2097152l-OFF2)/8192l);
}

float MS5837Driver::pressure(float conversion) {
    return P*conversion/10.0f;
}

float MS5837Driver::temperature() {
	return TEMP/100.0f;
}

float MS5837Driver::depth() {
	return (pressure(MS5837Driver::Pa)-101300)/(fluidDensity*9.80665);
}

float MS5837Driver::altitude() {
	return (1-std::pow((pressure()/1013.25),.190284))*145366.45*.3048;
}

uint8_t MS5837Driver::crc4(uint16_t n_prom[]) const {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for (uint8_t i=0; i<16; ++i) {
		if (i%2 == 1) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for (uint8_t n_bit=8; n_bit>0; --n_bit) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return (n_rem ^ 0x00);
}

void MS5837Driver::show_PROM() const {
    std::cout << "PROM" << std::endl;
    for (uint8_t i=0 ; i<7 ; ++i) {
        // PROM show
        std::bitset<16> b(C[i]);
        std::cout << b << std::endl;
	}
    std::bitset<4> crcR(C[0] >> 12);
    std::bitset<4> crcC(crc4(C));
    std::cerr << "CrcRead " << crcR << " CrcCalculated " << crcC << std::endl;
}
