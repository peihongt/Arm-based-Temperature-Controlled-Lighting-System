# Introduction
A smart temperature controlled lighting system is developed where the color tone of the RGB led will change automatically and gradually as the ambient temperature changes as to give people more comfortable feeling. We will be using STM32 Nucleo-64 development board to produce PWM signals to display different colours of lightning using RGB LEDs depending on the ambient temperature captured by temperature sensor. The detected temperature value and the colour to be displayed by the led will be printed on the LCD screen. 

# Hardware and Software used
Hardware:
1. STM32 Nucleo-64 development board (we are using STM32F411RE)
2. 2x16 LCD display
3. RGB LEDs 
4. 110ohms Resistors
5. 10k potentiometer
6. LM35 temperature sensor
7. Jumpers
8. Breadboard

![image](https://user-images.githubusercontent.com/82261395/122662419-5cc35600-d1c5-11eb-8507-87b3a4c5d5b3.png)

Software:
1. STM32CubeMX
- We use STM32CubeMX in this project as it is  a graphical tools that allows a very easy configuration of STM32 microcontroller and microprocessor. It also helps to generate the corresponding initialization C++ code for ARM Cortex-M core (we are using ARM Cortex M-4 core), through a step by step process. The first step is to select the model of STM32 microcontroller (we are using series STM32F411RE). The second step is to configure microprocessor in various peripheral supported, in our project we would be configuring peripheral RTOS, GPIO, TIMER and ADC. After configuration done, we need a toolchains to compile the code as STM32CubeMX just helped to configure the microprocessor easily and generate code initilization.

![image](https://user-images.githubusercontent.com/82261395/122660734-131f3f00-d1b6-11eb-8f00-92efe6d4fc2e.png)

2. Keil uVision
- Keil uVision is an integrated development environment (IDE) as it integrates the tools needed to develop embedded applications. After we done the configuration in STM32CubeMX, it generates code initialization for the peripherals that we enabled to Keil uVision IDE. In Keil uVision, is where we actually develop the main functions of the program, build and compile the code using C++ compiler. The last step would be load the code to our STM32 Nucleo Board. 

# Configuration steps using STM32CubeMX
This segment is divided into 4 main parts which are multithreading, configurations on ADC for temperature from LM35 temperature sensors, PWM control for RGB LEDs, and LCD display using STM32CubeMX IDE. The configuration for the necessary I/O pins will be explained as well.

1. Multithreading
- Step 1: In the Categories tab on the left, go to Middleware and select FREERTOS. Change the interface from disable to CMSIS RTOS V2. 
- Step 2: Go to Tasks and Queues tab, create three tasks/threads which are readtemp, printled and printlcd.
- Step 3: Change the priority of readtemp as osPriorityNormal while printled and printlcd as osPriorityBelowNormal, we will discuss in the session steps for firmware development. 
- Step 4: Renamed the entry function as ReadTemp, PrintLED and PrintLCD as it is just a function that is get called to start a task or thread. 

![image](https://user-images.githubusercontent.com/82261395/122635558-419a0d00-d117-11eb-9f11-6db1010cc11e.png)

- Step 5: Go to System Core and select SYS. Change the timebase source to become other timer (TIM5), this is because STM32 HAL API use the systick timer to determine how much time has passed since restart. HAL required the timer at higher priority to keep the time accurate while scheduler in FREERTOS required the timebase at lower priority. In this case, we use timer TIM5 for HAL (higher priority) and leave Systick for FREERTOS. 

![image](https://user-images.githubusercontent.com/82261395/122635598-71491500-d117-11eb-9959-d3c0dded7a62.png)

2. ADC conversion on input analog temperature value
- Step 1: In the Categories tab on the left, go to Analog and select ADC1. 
- Step 2: Enable the IN0 which corresponds to output pin PA0 to configure it as an ADC pin
- Step 3: Set the resolution value to 12 bits (15 ADC clock cycles) which specify the ADC values that we'll be getting ranges from 0 to 2^12 which is 4095 and change the sampling time to 15 clock cycles.

![image](https://user-images.githubusercontent.com/82261395/122635837-a30eab80-d118-11eb-8632-472ff0fd57e0.png)

3. PWM control on RGB LEDs
- Step 1: In the Categories tab on the left, go to Timer configuration. There are 4 built in timers. At the core, timer acts as a counter and each timer affect different set of pins. In this case, we select TIM1 as it controls pin PA8, PA9 and PA10. 
- Step 2: Set the clock source as internal clock source as we want the counter to increment once every single time of the system clock tick. 
- Step 3: Set channel 1, 2 and 3 to become PWM Generation CH1, CH2 and CH3 respectively. This step allows the generation of PWM signal to drive 3 output pins as these 3 pins will control the operation of RGB LEDs.

![image](https://user-images.githubusercontent.com/82261395/122635452-9721ea00-d116-11eb-8feb-cce5df40d3f2.png)

- Step 4: Set the prescaler and counter value to become 1538 and 255 repspectively as we will discuss it lately in the steps for firmware development.

![image](https://user-images.githubusercontent.com/82261395/122635483-cdf80000-d116-11eb-99cb-68a6bd03d2cc.png)

- Step 5: We can observe the output pins PA8, PA9 and PA10 are enabled and labeled as TIM1_CH1, TIM1_CH2 and TIM1_CH3 respectively. 

![image](https://user-images.githubusercontent.com/82261395/122635401-5033f480-d116-11eb-9203-b3d51025c520.png)

4. LCD display 
- Step 1: In the Pinout view, enable and declare pin PB0, PB1, PB2, PB4, PB5, PB6 and PB7 as GPIO output pins.
- Step 2: Label these GPIO output pins as RS, RW, EN, D4, D5, D6 and D7 respectively for better pin function recognition. 

![image](https://user-images.githubusercontent.com/82261395/122635805-78245780-d118-11eb-8370-fdb7489f8159.png)

- Step 3. At Keil, nclude the additional libraries for the lcd, which are lcd.c and lcd.h, that is the libaries for the lcd where we can use the function directly to control the lcd module. 

![image](https://user-images.githubusercontent.com/82261395/122635688-f9c7b580-d117-11eb-8d4b-991be24d9f30.png)

# Steps for firmware development
First Step: Defines Multiple Threads
- Real-time operating system (RTOS) that run on our microcontroller STM32F411RE allow us to execute multiple tasks concurrently. The scheduler or OsKernel is told to handle three threads which are ReadTemp, PrintLED and PrintLCD. 

![image](https://user-images.githubusercontent.com/82261395/122644963-5e036d00-d14a-11eb-94e0-0061c56bfa10.png)

- In this case, preemption is enabled to allow the scheduler to stop task from running to run another task of higher priority. In the configuration pane, thread ReadTemp is set to OsPriorityNormal while PrintLED and PrintLCD is set to OsPriorityBelowNormal. We want the operation of temperature reading to take place first because LED and LCD printing only can be done when the data read from the temperature sensor is ready. From the Figure below, real time ambient temperature is read every 100 milliseconds, new RGB LEDs colour will be updated every 4 seconds, and displaying real time temperature condition on LCD screen every 4 seconds without affecting one another. At 0s, scheduler execute ReadTemp thread first as it is set to higher priority. After the execution of ReadTemp thread finish, it is followed by PrintLED and PrintLCD thread. PrintLED and PrintLCD thread will be executed in round-robin as they are having same priority level. Same concept is repeated every 4 seconds. 

![image](https://user-images.githubusercontent.com/82261395/122637649-7364a100-d122-11eb-96f3-955c93b553af.png)

Second Step: ReadTemp Thread Setup
- The entire process flow start with reading of ambient temperature from the surrounding every 100milliseconds. LM35 temperature sensor we used is an analog sensor that converts the surrounding temperature to a proportional analog voltage. The information or signal that we obtained from LM35 temperature is in the form of analog signal. In this case,  our microcontroller STM32F411RE is embedded with ADC converter that managed to derive the equivalent temperature value in digital format. In the ADC configuration step, the resolution value is set to 12 bits meaning this is the sample steps for our ADC, the converted ADC values or integer values that we get will be in the range from 0 to 2^12-1 = 4095. Then, the value will be input to the Kalman Filter function to further stabilize the temperature value and return the optimized value.

![image](https://user-images.githubusercontent.com/82261395/122646456-d15cad00-d151-11eb-9e6d-3ffba70bd137.png)

Third Step: PrintLED Thread Setup
- As we know that RGB stands for Red, Green and Blue and these LEDs can emit all these three colours at the same time. Basically each colour have a range of intensity from 0 to 255 which depends on the voltage provided to the respective pin. We combine these intensities (0 to 255) of these three colours to produce 16 million (256x256x256) different colors. In order to vary the voltage across these pins, we need to use PWM or timer to vary the duty cycle. In the configuration steps, we use TIM1 as clock source to provide PWM signal to drive three colour pins. The TIMER in our Nucleo64 board with model STM32F411RE maximum running at 100Mhz. We use prescalar register and auto reload register (ARR) to vary the duty cycle.  Duty cycle is the percentage of the total time that the signal is in HIGH state in one cycle. Since we want to configure the timer in such a way that 100% duty cycle is equivalent to 255 range of intensity, we take clock rate divide by 255. Next, we set ARR to 255 because we want the timer to count up to 255 in one cycle. Hence, the prescalar register will be set to the value of 100M/255^2 = 1538.
- The stabilized temperature value from Kalman filter will be used to determine next colour to display on the RGB LEDs. A new colour will be updated on RGB LEDs every 4 seconds. If the 
1. Kalman temperature > 27 : RGB LEDs print BLUE,
2. 27 <= Kalman temperature > 26 : RGB LEDs print GREEN
3. 26 <= Kalman temperature > 25 : RGB LEDs print YELLOW
4. Kalman temperature <= 25 : RGB LEDs print RED
- When a different range of temperature is detected, the colour of RGB LEDs with be changing gradually from previous colour to the new colour which represents a new detected temperature range. 

![image](https://user-images.githubusercontent.com/82261395/122647812-9c078d80-d158-11eb-8877-c90b92cd83e9.png)

Fourth Step: PrintLCD Thread Setup
- As explained in the first step, PrintLCD thread and PrintLED thread is assigned with same priority level which is OsPriorityBelowNormal. They will be executed by round-robin scheduling. Basically, this thread allows the printing of real time temperature value and the colour it will be changing to on the LCD screen. The new temperature value and the colour information will be updated every 4 seconds. If the 
1. Kalman temperature > 27 : LCD print the text, BLUE and current temperature
2. 27 <= Kalman temperature > 26 : LCD print the text, GREEN and current temperature
3. 26 <= Kalman temperature > 25 : LCD print the text, YELLOW and current temperature
4. Kalman temperature <= 25 : LCD print the text, RED and current temperature

![image](https://user-images.githubusercontent.com/82261395/122648447-f1916980-d15b-11eb-8344-6c3c182d9bbd.png)

# Steps for hardware development
1. Prepare all the hardware required as listed in the software/tools setup session.
2. RGB LEDs setup
- The setup of RGB LEDs on STM32 Nucleo Board is shown as figure below. We applied 3 additional 220 ohm resistors on each RGB pins. 

![image](https://user-images.githubusercontent.com/82261395/122651046-e264e880-d168-11eb-97ee-a466c5c57206.png)

- By referring to the RGB LEDs datasheets, the current needed to drive each RGB pin is 20mA. The output GPIO pins of our STM32 Nucleo Board is supplying up to 5V. In mathematical calculation, apply formula V/I = R. For RED pin: (5V-2V)/20mA = 150 ohm, for GREEN pin: (5V-3V)/20mA = 100 ohm, for BLUE pin: (5V-2V)/20mA = 100 ohm. The ideal resistor value to maximize each RGB pins performance in terms of luminous intensity is 150 ohm, 100 ohm and 100 ohm respectively. In our project, the objective that we are using 220 ohm for each RGB instead of the ideal resistor value is due to a reason. The maximum current that can be afford by each RGB led pin is 20mA, if we use the resistor value for the absolute maximum current, we do not guarantee that the supply voltage is always 5V. Hence, we use higher resistor value to prevent the uncertainty occurred and that would damage the RGB leds and the STM32 Nucleo Board if come to the worst case. We can simply avoid it by using higher resistor value and sacrifice the luminous intensity.

![image](https://user-images.githubusercontent.com/82261395/122661314-968f5f00-d1bb-11eb-9ffc-a4f71d639ce4.png)

3. LCD screen setup
The setup of LCD screen on STM32 Nucleo Board is shown as figure below. We could see that other than LCD screen and STM32 Nucleo board, there is an additional potentiometer. The pin Vo of the LCD act as a control pin to adjust the contrast of LCD. The middle variable output pin of the potentiometer is connected to Vo pin of the LCD to provide an adjustable variable voltage from 0V to 5V for adjusting the LCD contrast, the left pin connected to 5V Vdd while the right pin connected to the ground. 

![image](https://user-images.githubusercontent.com/82261395/122649886-afb7f180-d162-11eb-860a-a9e29563f523.png)

4. LM35 Temperature sensor setup
The setup of LM35 temperature sensor on STM32 Nucleo Board is shown as below. We could see that the middle analog output pin is connected to the analog input pin on STM32 Nucleo Board. The analog output signal from LM35 is transmitted to analog input pin A0 on Nucleo Board to perform analog to digital conversion, the vdd pin connects to 3.3v on the board, while the right pin which is ground connects to the GND pin.

![image](https://user-images.githubusercontent.com/82261395/122661947-f12bb980-d1c1-11eb-916e-9892372e33de.png)

# Conclusion
In this project, we have learned about ultilizing multiple thread functions of the CMSIS RTOS to enable the concurrent execution of tasks such as the detection of real time ambient temperature, gradual change of led colour from one to another depends on the real time temperature and displaying the current temperature value and colour as text on the lcd. This project has demonstrated how a rather cheap and accessible microcontroller like the stm32 series can be integrated in to the everyday life of people. 
