
# SMART-AUTOMATED-AIR-POLLUTION-DETECTION-AND--MONITORING-SYSTEM-IN-VEHICLES

A brief description of what this project does and who it's for
## Abstract
Every vehicle has its own emission of gases, but the 
problem occurs when the emission is beyond the standardized values. 
The primary reason for this breach of emission level being the 
incomplete combustion of fuel supplied to the engine which is due to 
the improper maintenance of vehicles. This emission from vehicles 
cannot be completely avoided, but it definitely can be controlled.
The aim of the project is to monitor and control the pollutants in the 
vehicle by using the pollution control circuit. This pollution control 
circuit consists of various sensors like smoke sensor, temperature 
sensor and GSM, kind of devices, and all of them are integrated and 
connected to a Controller. It is a real time work where a demo 
application has been made in which ARDUINO microcontroller is 
used and a controller board is made where all these devices get 
integrated and work accordingly. The vehicle is controlled by this 
circuit. When a vehicle attains certain threshold pollution level then 
the engine gets automatically switched off and an SMS is generated 
and sent to the pre-defined number stored in the memory through the 
GSM module. This paper demonstrates an effective utilization of 
technology by which we save our environment by controlling the 
pollution of vehicles.
## List of Equipments required

### Sensors
- MQ135 – Air quality sensor.
- MQ7 – Carbon Monoxoide sensor.
- DHT – Temperature and Humidity sensor. 
### Microcontroller
- Arduino UNO
- Arduino GSM module
- Arduino GPS module
### Others
- Power supply
- Relay
- LCD display
## Working Principle 
The smoke detector detects carbon and 
gives it to the Microcontroller to check the maximum 
percentage of carbon content in the smoke released by vehicles. 
Temperature sensor can be used to sense the temperature in the 
vehicle. So the controller checks the percentage of carbon and 
temperature, if it exceeds the threshold level the system gets 
triggered and the engine comes to hault state and then it sends 
SMS about this to the nearby pollution control office through 
GSM.

  
## Screenshots

![image](https://drive.google.com/uc?export=view&id=1sEYZxLhZrhIiiSEStniddXqVrxQOmx-P)

### MQ135 Air Quality Sensor
Air quality sensor for detecting a wide range of gases, including 
NH3, NOx, alcohol, benzene, smoke and CO2. Ideal for use in 
office or factory. MQ135 gas sensor has high sensitivity to 
Ammonia, Sulfide and Benze steam, also sensitive to smoke and 
other harmful gases. It is with low cost and particularly suitable 
for Air quality monitoring application.

### MQ7 – Carbon Monoxoide sensor
This is a simple-touse Carbon Monoxide (CO) sensor, suitable for sensing CO 
concentrations in the air. The MQ-7 can detect CO-gas 
concentrations anywhere from 20 to 2000ppm.
This sensor has a high sensitivity and fast response time. The 
sensor's output is an analog resistance. The drive circuit is very 
simple; all you need to do is power the heater coil with 5V, add 
a load resistance, and connect the output to an ADC.

### DHT – Temperature and Humidity sensor
The DHT11 is a 
commonly used Temperature and humidity sensor. The 
sensor comes with a dedicated NTC to measure temperature and 
an 8-bit microcontroller to output the values of temperature and 
humidity as serial data. The sensor is also factory calibrated and 
hence easy to interface with other microcontrollers.

### Arduino board 
Arduino is an open-source platform used for 
building electronics projects. Arduino consists of both a
physical programmable circuit board (often referred to as 
a microcontroller) and a piece of software, or IDE (Integrated 
Development Environment) that runs on your computer, used 
to write and upload computer code to the physical board.

### Arduino GSM module
A GSM Module is basically a GSM 
Modem (like SIM 900) connected to a PCB with different types 
of output taken from the board – say TTL Output (for Arduino, 
8051 and other microcontrollers) and Output to interface 
directly with a PC (personal computer).

## Project Overview
![image](https://drive.google.com/uc?export=view&id=1d55BEj4RenYQCWeDJygyqfsTqeV61ufM)

## Circuit diagram
![image](https://drive.google.com/uc?export=view&id=1NZTvCKqK2ZcBRqz2QmPM38h3NUSJg4Y5)

### Advantages
- The designed smart intelligent environmental 
system monitors the pollutants produced by the 
vehicles. 
- Warn the vehicle owners to control the 
pollution.
- The air pollution agencies can able to analyze 
the data and also detect the vehicle registration 
numbers that causes more pollution in the 
atmosphere
- Low cost, simple to operate and is easily 
inserted in any locations.

### Disadvantages
- A SIM card is must for sending the SMS to vehicle 
owner. 
- Short period of one day time is given to the vehicle 
owner for making tuning of the engine.

## Conclusions
This whole paper mainly focuses on two 
things. The First thing is the concept of detecting the level of 
Pollution and indicating it to the driver. There is an increase in 
the level of Pollution over the last couple of decades, leading to 
several Environmental problems. There will be a huge 
population, who do not take the pollution from their vehicles 
seriously, which has already resulted in several environmental 
problems such Ozone layer depletion and so on. So, this system 
will be highly beneficial is curbing this problem. The second 
reason is that this system will be one of the greatest 
improvements in technology to keep the Environment free 
from vehicular emission and bring it to a halt if the Pollution 
level is more than the Standards mentioned by the 
Government. The fact that this system is just an add-on, as it 
does not change the configuration of the engine by any means, 
will make it easier to employ this system in the existing 
vehicles. The same concept can also be extended to industries.



## References

[Circuitdigest](https://circuitdigest.com/microcontroller-projects/iot-air-pollution-monitoring-using-arduino)

  