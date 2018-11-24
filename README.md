# Sim800l Nokia
Utilizing functions of Sim800l module beyond call and sms.

Features:
* Request a text file from a URL and display the result on screen
* Show network operator info (name, signal strength, network date & network time) 
* Show your approximate Location a.k.a LBS information (date, time, latitude, and longitude)
* Save (to eeprom) and retrive last saved location info
* Upload your location info to a JSON server (jsonbin.io)
* Scheduled location uploads (every 6 hours by default)
* Show weather info of your current location (using OpenWeatherMap API & LBS)
* Serial debugging a.k.a. SerialPassthrough
* Autosleep when idle after a specific time (30 senconds by default)
* Advanced Sleep & Wake-up functions to save power (consumption ≈ 6 mW during deep sleep)
* Simple menu to navigate 
* Optimized SRAM usage (more than 50% is available for local variables)

Future features:
* FM radio tuner?


Two external libraries are required: "Adafruit_GFX" and "Adafruit_PCD8544"


Needed hardware:
* Arduino Leonardo/Micro/Pro Micro compatible board (Atmega32u4)
* Sim800l GSM breakout board (5v tolerant)
* Sim card with data plan
* Nokia 3310/5110 LCD (PCD8544)
* Solderless breadboard
* 3 Push buttons
* Jump wires
* Optional: NPN transistor and a resistor to control GND pin of the Sim800l module (to save power)

![alt text](https://raw.githubusercontent.com/HA4ever37/Sim800l-Nokia/master/Sim800l_bb.png)
![alt text](https://github.com/HA4ever37/Sim800l/blob/master/Atmega32u4+PCD8544+Sim800L.jpg?raw=true)
[alt text](https://raw.githubusercontent.com/HA4ever37/Sim800l-Nokia/master/power.jpg)
