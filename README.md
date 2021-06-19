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
1. Step 1: 
2. Step 2:
3. Step 3:
4. Step 4:

- ADC of temperature from LM35 temperature sensors
1. Step 1:
2. Step 2:
3. Step 3:
4. Step 4:

- PWM control on RGB LEDs
1. Step 1: Go to select Timer configuration pane on the left, there are 4 built in timers. At the core, timer acts as a counter and each timer affect different set of pins. In this case, we select TIM1 as it controls pin PA8, PA9 and PA10. 
2. Step 2: Set the clock source as internal clock source as we want the counter to increment once every single time of the system clock tick. 
3. Step 3: Set channel 1, 2 and 3 to become PWM Generation CH1, CH2 and CH3 respectively. This step allows the generation of PWM signal to drive 3 output pins as these 3 pins will control the operation of RGB LEDs.   
4. Step 4: Set the prescaler and counter value to become 1538 and 255 repspectively as we will discuss it lately in the steps for firmware development.
5. Step 5: We can observe the output pins PA8, PA9 and PA10 are enabled and labeled as TIM1_CH1, TIM1_CH2 and TIM1_CH3 respectively. 

- LCD display 
1.
2.
3.
4.


# Steps for firmware development
Multithreading
- Real-time operating system (RTOS) that run on our microcontroller STM32F411RE allow us to execute multiple tasks concurrently. The scheduler or OsKernel is told to handle three threads in this case which are ReadTemp, PrintLED and PrintLCD. Besides, preemption is enabled to allow the scheduler to stop task from running to run another task of higher priority. In the configuration pane, thread ReadTemp is set to OsPriorityNormal while PrintLED and PrintLCD is set to OsPriorityBelowNormal. We want the operation of temperature reading to take place first because LED and LCD printing only can be done when the data read from the temperature sensor is ready. eading real time ambient temperature every 100 milliseconds, displaying colour on RGB LEDs every 4 seconds, and displaying real time temperature condition on LCD screen every 4 seconds without affecting one another. 
