# aht10_driver
Imx6ull (Linux) I2C driver for AHT10 sensor
    Linux device driver for AHT10 temperature sensor

### Compiling the module for Linux (arm32 imx6ull)
#### modifi path of Compiling KERNELDIR in Makefile
KERNELDIR := /home/xxx

make

### Compile the device tree overlay
make dtbs

### Compile the demo app
cd app

make

