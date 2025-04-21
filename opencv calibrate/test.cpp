#include <iostream>
#include <unistd.h>
#include <wiringPiI2C.h>
#include <cmath>
#include <cstdlib>

#define PCA9685_ADDR 0x40
#define MODE1        0x00
#define PRESCALE     0xFE
#define LED0_ON_L    0x06

const int servo_channel = 0;
const int min_val = 205;
const int max_val = 410;

int pca9685_fd;

void setPWMFreq(int freq) {
    float prescaleval = 25000000.0;  // 25MHz internal oscillator
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    int prescale = std::floor(prescaleval + 0.5);

    int oldmode = wiringPiI2CReadReg8(pca9685_fd, MODE1);
    int sleep = (oldmode & 0x7F) | 0x10;  // sleep mode
    wiringPiI2CWriteReg8(pca9685_fd, MODE1, sleep);
    wiringPiI2CWriteReg8(pca9685_fd, PRESCALE, prescale);
    wiringPiI2CWriteReg8(pca9685_fd, MODE1, oldmode);
    usleep(5000);
    wiringPiI2CWriteReg8(pca9685_fd, MODE1, oldmode | 0xa1);  // restart + auto-increment
}

void setPWM(int channel, int on, int off) {
    wiringPiI2CWriteReg8(pca9685_fd, LED0_ON_L + 4 * channel, on & 0xFF);
    wiringPiI2CWriteReg8(pca9685_fd, LED0_ON_L + 4 * channel + 1, on >> 8);
    wiringPiI2CWriteReg8(pca9685_fd, LED0_ON_L + 4 * channel + 2, off & 0xFF);
    wiringPiI2CWriteReg8(pca9685_fd, LED0_ON_L + 4 * channel + 3, off >> 8);
}

int angleToPWM(int angle) {
    angle = std::max(0, std::min(180, angle));
    return min_val + (angle * (max_val - min_val)) / 180;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <angle (0–180)>" << std::endl;
        return 1;
    }

    int angle = std::atoi(argv[1]);
    if (angle < 0 || angle > 180) {
        std::cerr << "Invalid angle. Must be between 0 and 180." << std::endl;
        return 1;
    }

    pca9685_fd = wiringPiI2CSetup(PCA9685_ADDR);
    if (pca9685_fd < 0) {
        std::cerr << "Failed to connect to PCA9685 at 0x" 
                  << std::hex << PCA9685_ADDR << std::endl;
        return 1;
    }

    setPWMFreq(50);  // Set to 50 Hz for servos

    int pwm = angleToPWM(angle);
    std::cout << "Setting servo to " << angle << "°, PWM: " << pwm << std::endl;
    setPWM(servo_channel, 0, pwm);

    return 0;
}
