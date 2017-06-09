open-headtracker versions (http://www.rcgroups.com/forums/showthread.php?t=1677559&page=207)

 all founded forks of original DIY_Headtracker ( in DIY_Headtracker_firmware_orig )


DIY_headtracker-8mhz - port to 8mhz Arduino (as is)
and others for different sensors (as is)

main one:
DIY_Headtracker_firmware_fastserial - changed to FastSerial library with printf_P, uses ATAN2 (no more NAN!), add noise filter and some cleanup

* sensor timer moved from timer0 to timer2 - so we can use millis()!
* odd slow Wire library changed to fast I2C library
* I2C library altered to speed up and reduce size
* added full compass calibration with hard-iron and soft-iron corrections
* addede Four Order filter to accel data
* all code refactored to fit all the above
* calculations optimized so now measure & calculations uses only 4ms
* gyro filter changed from 98Hz to 20Hz


 - It works!

