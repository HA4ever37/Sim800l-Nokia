# Sim800l Nokia
Utilizing functions of Sim800l module beyond calls and sms using any Atmega32u4/Atmega328p board and Nokia 5110 (PCD8544) display

Features:
* Request a text file from a URL and display the result on screen (support both Http and Https)
* Show network operator info (name, signal strength, network date & network time) 
* Show your approximate location a.k.a LBS information (date, time, latitude, and longitude). Please note that the date and time are based on GMT time
* Save (to eeprom) and retrive last saved location info (date, time, latitude, and longitude)
* Upload your location info to a JSON server (jsonbin.io) including:
  * Google Maps link of your location
  * GMT date and time of the upload entry
* Scheduled location uploads (every 6 hours by default)
* Show weather info of your current location (using OpenWeatherMap API & LBS) including:
  * Current temperature + Min & Max temp (in Celsius)
  * Humitiy level (in percentage)
  * Wind speed (in Meters per Second)
* Serial debugging a.k.a. SerialPassthrough (Atmega32u4 version only)
* Autosleep when idle after a specific time period (30 senconds by default)
* Advanced sleep & wake up functions to minimize power consumption during sleep
* Simple menu to navigate (Nokia Style)
* Optimized SRAM usage (more than 50% is available for local variables)
* Backlight control

Future features:
* FM radio tuner?


Two external libraries are required: "Adafruit_GFX" and "Adafruit_PCD8544"


Needed hardware:
* Arduino Leonardo/Micro/Pro Micro compatible board (Atmega32u4) or Arduino Pro Mini (Atmega328p)
* Sim800l GSM breakout board (5v tolerant)
* Sim card with data plan
* Nokia 3310/5110 LCD (PCD8544)
* Solderless breadboard
* 3 Push buttons
* Jump wires
* Optional: NPN transistor and a resistor to control GND pin of the Sim800l module (to save power)

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/ckMDemmqBLY/0.jpg)](http://www.youtube.com/watch?v=ckMDemmqBLY)
![alt text](https://raw.githubusercontent.com/HA4ever37/Sim800l-Nokia/master/Sim800l_bb.png)
![alt text](https://github.com/HA4ever37/Sim800l/blob/master/Atmega32u4+PCD8544+Sim800L.jpg?raw=true)
