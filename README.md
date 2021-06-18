# Introduction
In this project, we will be using STM32 Nucleo-64 development board to display different colour of lightning at the output RGB LEDs depending on the range of ambient temperature captured from the input temperature sensor. The details of the detected temperature value and the colour printing on the led will be printed on the output LCD screen. 
# Hardware and Software needed
Hardware:
- STM32 Nucleo-64 development board (we are using STM32F411RE)
- 2x16 LCD display
- RGB LEDs 
- 110ohms Resistors
- 10k potentiometer
- LM35 temperature sensor

Software:
- Integrated Development Environment: STM32CubeMX
- Keil uVision

Steps for firmware development
1. Multithreading 
Real-time operating system (RTOS) that run on our microcontroller STM32F411RE allow us to execute multiple tasks concurrently. We adopted multi-threading of CMSIS RTOS in reading real time ambient temperature every 100 milliseconds, displaying colour on RGB LEDs every 4 seconds, and displaying real time temperature condition on LCD screen every 4 seconds without affecting one another. 
