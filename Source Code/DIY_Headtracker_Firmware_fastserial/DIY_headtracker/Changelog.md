* sensor timer moved from timer0 to timer2 - so we can use millis()!
* odd slow Wire library changed to fast I2C library
* I2C library altered to speed up and reduce size
* added full compass calibration with hard-iron and soft-iron corrections
* addede Four Order filter to accel data
* all code refactored to fit all the above
* calculations optimized so now measure & calculations uses only 4ms
* gyro filter changed from 98Hz to 20Hz
