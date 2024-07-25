# BMP280(or HW611)for nrf52832
 drive program, easy use function and simple demo
 The main program and the function's definition is in \app\hw611\hw611.c, and decleartion is in \app\hw611\hw611.h
 You just need to "include hw611.h" in your project
 And \app\demo.c is a reference for using this program
 You should copy the whole sdk_config.h or at least the "nrf_driver" part, it enable the twi(i2c device) to make sure this program can run
