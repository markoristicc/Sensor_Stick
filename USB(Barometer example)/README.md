Sending Data via USB with a working example of barometer

NOTES:
1. Same ioc as 'SensorStick_Integrated' zip file (3/1/2023) just 1 change. Go to 'USB_DEVICE' and set to 'COMMUNICATION DEVICE CLASS (VIRTUAL PORT COM)'
2. In Project properties, go to 'C/C++ Build' then in 'Settings' go to MCU GCC Assembler, MCU GCC Compiler, and MCU GCC Linker and add in '-u_printf_float' flag in 'Command section
3. All code added in main file has comments around it saying 'ADDED IN'
