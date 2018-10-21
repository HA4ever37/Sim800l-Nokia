# Sim800l
Utilizing functions of Sim800L module beyond call and sms.

Features:
* Request a text file from a URL and display the result on screen
* Show network operator info (name, signal strength, network date & network time) 
* Show current Location (LBS) information
* Save (to eeprom) and retrive last saved location info
* Upload location info to a JSON server (jsonbin.io)
* Scheduled location uploads
* Autosleep when idle after specific time (30 senconds by default)
* Advanced Sleep & Wake-up functions to save power
* Simple menu to navigate 
* Optimized SRAM usage

Future features:
* Get last location info from a JSON server 
* FM radio tuner?


Two external libraries are required: "Adafruit_GFX" and "Adafruit_PCD8544"


Needed hardware:
* Arduino Leonardo/Micro/Pro Micro compatible board (Atmega32u4)
* Sim800L GSM breakout board (5v tolerant)
* Sim card with data plan
* Nokia 3310/5110 LCD (PCD8544)
* Solderless breadboard
* 3 Push buttons
* Jump wires
* Optional: NPN transistor to control GND pin of the Sim800l module  


![alt text](https://github.com/HA4ever37/Sim800l/blob/master/Atmega32u4+PCD8544+Sim800L.jpg?raw=true)
