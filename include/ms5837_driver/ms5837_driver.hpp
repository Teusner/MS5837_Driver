#pragma once

#include <string>

class MS5837Driver {
    public:

        MS5837Driver(std::string port): port_(port) {};

        ~MS5837Driver();

        bool init();

        void show_PROM();

        /** The read from I2C takes up to 40 ms, so use sparingly if possible.
         */
        bool read_data();

        /** Pressure returned in mbar or mbar*conversion rate.
         */
        float pressure(float conversion = 1.0f);

        /** Temperature returned in deg C.
         */
        float temperature();


        /** Provide the density of the working fluid in kg/m^3. Default is for 
         * seawater. Should be 997 for freshwater.
         */
        void setFluidDensity(float density);

        /** Depth returned in meters (valid for operation in incompressible
         *  liquids only. Uses density that is set for fresh or seawater.
         */
        float depth();

        /** Altitude returned in meters (valid for operation in air only).
         */
        float altitude();

    private :
        // Sensor I2C constants
        static constexpr uint8_t MS5837_ADDR = 0x76;  
        static constexpr uint8_t MS5837_RESET = 0x1E;
        static constexpr uint8_t MS5837_ADC_READ = 0x00;
        static constexpr uint8_t MS5837_PROM_READ = 0xA0;
        static constexpr uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
        static constexpr uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

        static constexpr float Pa = 100.0f;
        static constexpr float bar = 0.001f;
        static constexpr float mbar = 1.0f;

        // I2C dev port e.g. "/dev/i2c-1"
        std::string port_;

        // File descriptor associated to the I2C port
        int fd_;

        // Calibration coefficents
        uint16_t C[7];

        uint32_t D1, D2;
        int32_t TEMP;
        int32_t P;
        int port;

        float fluidDensity=1000.;

        /** Performs calculations per the sensor data sheet for conversion and
         *  second order compensation.
         */
        void calculate();

        uint8_t crc4(uint16_t n_prom[]);
};