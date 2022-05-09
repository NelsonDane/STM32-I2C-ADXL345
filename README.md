# STM32-I2C-ADXL345
Project to read xyz data from an ADXL345 accelerometer via LL I2C, printing the values out to USART2.

### Notes
To change pin configurations, open ADXL_I2C.ioc using CubeMX, then regenerate the code.
Tested with Keil on the STM32f446re microcontroller with an ADXL345. Wiring is commented in the code as well as in wiring.jpg
