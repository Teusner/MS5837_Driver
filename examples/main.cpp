#include <ms5837_driver/ms5837_driver.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

int main() {
    MS5837Driver driver("/dev/i2c-1");
    bool init = driver.init();

    if (!init) {
        std::cerr << "Unable to init the driver at port " << "/dev/i2c-1" << std::endl;
        return -1;
    }

    driver.show_PROM();

    for (uint8_t i=0; i<100; ++i) {
        bool read = driver.read_data();

        if (read) {
            float p = driver.pressure();
            float t = driver.temperature();
            float d = driver.depth();
            float a = driver.altitude();
            std::cout << "Pressure " << p << " Temperature " << t << " Depth " << d << " Altitude " << a << std::endl;
        }
        else {
            std::cerr << "Problem while reading data" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}