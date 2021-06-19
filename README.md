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

![image](https://user-images.githubusercontent.com/82261395/122631081-67b1b400-d0fb-11eb-865a-f7c9f88acc03.png)

Software:
- STM32CubeMX IDE
- Keil uVision

# Configuration steps
This session can be divided into 4 main parts as configuration needs to be done separately at multithreading, ADC of temperature from LM35 temperature sensors, PWM control on RGB LEDs, and LCD display using STM32CubeMX IDE. 

- Multithreading
1. Step 1: In the Categories tab on the left, go to Middleware and select FREERTOS. Change the interface from disable to CMSIS RTOS V2. 
2. Step 2: Go to Tasks and Queues tab, we create three tasks/threads which are readtemp, printled and printlcd.
3. Step 3: Change the priority of readtemp as osPriorityNormal while printled and printlcd as osPriorityBelowNormal, we will discuss in the session steps for firmware development. 
4. Step 4: Renamed the entry function as ReadTemp, PrintLED and PrintLCD as it is just a function that is get called to start a task or thread. 

![image](https://user-images.githubusercontent.com/82261395/122635558-419a0d00-d117-11eb-9f11-6db1010cc11e.png)

5. Step 5: Go to System Core and select SYS. Change the timebase source to become other timer (TIM5), this is because STM32 HAL API use the systick timer to determine how much time has passed since restart. HAL required the timer at higher priority to keep the time accurate while scheduler in FREERTOS required the timebase at lower priority. In this case, we use timer TIM5 for HAL (higher priority) and leave Systick for FREERTOS. 

![image](https://user-images.githubusercontent.com/82261395/122635598-71491500-d117-11eb-9959-d3c0dded7a62.png)

- ADC on temperature from LM35 temperature sensors
1. Step 1: In the Categories tab on the left, go to Analog and select ADC1. 
2. Step 2: Enable the IN0 which corresponds to output pin PA0 to configure it as an ADC pin
3. Step 3: Set the resolution value to 12 bits (15 ADC clock cycles) which specify the ADC values that we'll be getting ranges from 0 to 2^12.

![image](https://user-images.githubusercontent.com/82261395/122635837-a30eab80-d118-11eb-8632-472ff0fd57e0.png)

- PWM control on RGB LEDs
1. Step 1: In the Categories tab on the left, go to Timer configuration. There are 4 built in timers. At the core, timer acts as a counter and each timer affect different set of pins. In this case, we select TIM1 as it controls pin PA8, PA9 and PA10. 
2. Step 2: Set the clock source as internal clock source as we want the counter to increment once every single time of the system clock tick. 
3. Step 3: Set channel 1, 2 and 3 to become PWM Generation CH1, CH2 and CH3 respectively. This step allows the generation of PWM signal to drive 3 output pins as these 3 pins will control the operation of RGB LEDs.

![image](https://user-images.githubusercontent.com/82261395/122635452-9721ea00-d116-11eb-8feb-cce5df40d3f2.png)

4. Step 4: Set the prescaler and counter value to become 1538 and 255 repspectively as we will discuss it lately in the steps for firmware development.

![image](https://user-images.githubusercontent.com/82261395/122635483-cdf80000-d116-11eb-99cb-68a6bd03d2cc.png)

5. Step 5: We can observe the output pins PA8, PA9 and PA10 are enabled and labeled as TIM1_CH1, TIM1_CH2 and TIM1_CH3 respectively. 

![image](https://user-images.githubusercontent.com/82261395/122635401-5033f480-d116-11eb-9203-b3d51025c520.png)

- LCD display 
1. Step 1: In the Pinout view, enable and declare pin PB0, PB1, PB2, PB4, PB5, PB6 and PB7 as GPIO output pins.
2. Step 2: Label these GPIO output pins as RS, RW, EN, D4, D5, D6 and D7 respectively for better pin function recognition. 

![image](https://user-images.githubusercontent.com/82261395/122635805-78245780-d118-11eb-8370-fdb7489f8159.png)

3. Step 3. Add on two additional files which is lcd.c and lcd.h, that is the libaries for the lcd where we can use the function directly to control the lcd module. 

![image](https://user-images.githubusercontent.com/82261395/122635688-f9c7b580-d117-11eb-8d4b-991be24d9f30.png)

# Steps for firmware development
Multithreading
- Real-time operating system (RTOS) that run on our microcontroller STM32F411RE allow us to execute multiple tasks concurrently. The scheduler or OsKernel is told to handle three threads in this case which are ReadTemp, PrintLED and PrintLCD. Besides, preemption is enabled to allow the scheduler to stop task from running to run another task of higher priority. In the configuration pane, thread ReadTemp is set to OsPriorityNormal while PrintLED and PrintLCD is set to OsPriorityBelowNormal. We want the operation of temperature reading to take place first because LED and LCD printing only can be done when the data read from the temperature sensor is ready. eading real time ambient temperature every 100 milliseconds, displaying colour on RGB LEDs every 4 seconds, and displaying real time temperature condition on LCD screen every 4 seconds without affecting one another. 
