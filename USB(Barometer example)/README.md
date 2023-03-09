Sending Data via USB with a working example of barometer(interrupt based)

NOTES:
1. Same ioc as 'SensorStick_Integrated' zip file (3/1/2023) just 2 change. Go to 'USB_DEVICE' and set to 'COMMUNICATION DEVICE CLASS (VIRTUAL PORT COM)' and PC13 set to EXTI13 
2. '-u_printf_float' flag added in
3. Using PC13 for barometer interrupt, currently unused by other sensors
