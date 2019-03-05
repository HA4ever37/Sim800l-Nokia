#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define lcdBL 5         // LCD backlight pin
#define resetPin 6      // Reset pin of Sim800L
#define pwrPin A0       // controling an NPN transistor that provides 0V to GND pin of Sim800L
#define btnEnt 2
#define btnUp 3
#define btnDwn 7
#define MENU_ROW_HEIGHT 11
#define LCD_ROWS 5
#define RXLED 17
#define TXLED 30
#define COUNTER 2522   // auto upload sleep counter will make the device sleep for 6 hours (COUNTER * 1.07 * 8 seconds)

Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 4);

unsigned long startMillis, currentMillis;
const unsigned long period PROGMEM = 30000; // The duration in milliseconds before the microcontroller goes to sleep
bool GPRSCon, isItSleep, exitBool;
byte menuPos, menuScreen, markerPos, menuStartAt;
const char* const menu[14] PROGMEM  = {"File Request", "Network Info", "Weather Info", "Location Info", "Save Location",
                                       "Last Saved" , "Upload Loc", "Auto Upload", "Connect", "Disconnect",
                                       "Light Switch", "Power Down", "Reset Sim800L", "Debug Mode"
                                      };
byte MENU_LENGTH =  sizeof(menu) / sizeof(menu[0]);

void setup() {
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDwn, INPUT_PULLUP);
  pinMode(btnEnt, INPUT_PULLUP);
  pinMode(lcdBL, OUTPUT);
  pinMode(resetPin, OUTPUT);
  pinMode(pwrPin, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  digitalWrite(resetPin, HIGH);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, HIGH);
  digitalWrite(resetPin, HIGH);
  digitalWrite(pwrPin, HIGH);
  digitalWrite(lcdBL, LOW);
  display.begin();
  display.setRotation(2);     // My screen is flipped upside down! :/
  display.clearDisplay();
  delay(4000);  // You may increase this if the duration isn't enough
  while (!checkSim800()) {
    display.clearDisplay();
    display.println(F("Sim800L is \nturned off or \nnot responding"));
    display.display();
    delay(2000);
    resetSim800();
  }
  waitToReg();
  showMenu();
  startMillis = millis();
}

void loop() {
  currentMillis = millis();
  if (isButtonDown(btnDwn) == true) {
    if (menuPos < MENU_LENGTH - 1) {
      menuPos++;
      if (menuPos - menuStartAt > 3 )
        menuStartAt++;
      showMenu();
    }
    delay(100);
    startMillis = currentMillis;
  }
  if (isButtonDown(btnUp) == true) {
    if (menuPos > 0) {
      menuPos--;
      if (menuPos - menuStartAt < 0 && menuStartAt != 0)
        menuStartAt--;
      showMenu();
    }
    delay(100);
    startMillis = currentMillis;
  }
  if (isButtonDown(btnEnt) == true) {
    if (menuPos == 0) {
      const String s = openURL(F("raw.githubusercontent.com/HA4ever37/Sim800l/master/Sim800.txt"), true); // Change the URL to your text file link
      if (s == "ERROR" || s == "")  {
        display.clearDisplay();
        display.println(F("Bad request! \ntry again"));
        display.display();
        delay(2000);
      }
      else {
        s = s.substring(s.indexOf(F("\r")) + 2, s.lastIndexOf(F("OK")));
        s = s.substring(s.indexOf(F("\r")) + 2, s.length() - 1);
        display.clearDisplay();
        digitalWrite(lcdBL, LOW);
        /*for (byte i = 0; i < s.length(); i++) {
          //Serial.print(s.charAt(i));
          display.print(s.charAt(i));
          }*/
        display.print(s);
        display.display();
        digitalWrite(lcdBL, HIGH);
        delay(500);
        digitalWrite(lcdBL, LOW);
        while (!isButtonDown(btnEnt) && !isButtonDown(btnUp) && !isButtonDown(btnDwn));
      }
      showMenu();
    }
    else if (menuPos == 1) {
      netInfo();
      showMenu();
    }
    else if (menuPos == 2) {
      const String s = openURL(locInfo(3), false);
      char split = '"';
      int pos = s.indexOf(F("description\":\"")) + 14;
      String weather = s.substring(pos, s.indexOf(split, pos));
      weather[0] = toupper(weather[0]);
      display.clearDisplay();
      display.println(weather);
      split = ',';
      pos = s.indexOf(F("temp\":")) + 6;
      display.print(F("Temp:"));
      display.print(s.substring(pos, s.indexOf(split, pos)));
      display.println(F(" C"));
      pos = s.indexOf(F("p_min\":")) + 7;
      display.print(F("Min:"));
      display.print(s.substring(pos, s.indexOf(split, pos)));
      display.println(F(" C"));
      split = '}';
      pos = s.indexOf(F("p_max\":")) + 7;
      display.print(F("Max:"));
      display.print(s.substring(pos, s.indexOf(split, pos)));
      display.println(F(" C"));
      split = ',';
      pos = s.indexOf(F("humidity\":")) + 10;
      display.print(F("Humidity:"));
      display.print(s.substring(pos, s.indexOf(split, pos)));
      display.println(F("%"));
      pos = s.indexOf(F("speed\":")) + 7;
      display.print(F("Wind:"));
      display.print(s.substring(pos, s.indexOf(split, pos)));
      //display.print(s.substring(s.indexOf(F("speed\":")) + 7, s.indexOf(F(",\"deg"))));
      display.print(F(" m/s"));
      display.display();
      while (!isButtonDown(btnEnt) && !isButtonDown(btnUp) && !isButtonDown(btnDwn));
      showMenu();
    }
    else if (menuPos == 3) {
      locInfo(0);
      showMenu();
    }
    else if (menuPos == 4) {
      locInfo(2);
      showMenu();
    }
    else if (menuPos == 5) {
      readEeprom();
      showMenu();
    }
    else if (menuPos == 6) {
      locInfo(1);
      showMenu();
    }
    else if (menuPos == 7)
      autoUp();
    else if (menuPos == 8) {
      connectGPRS();
      showMenu();
    }
    else if (menuPos == 9) {
      disConnectGPRS();
      showMenu();
    }
    else if (menuPos == 10)
      toggle();
    else if (menuPos == 11)
      pwrDown();
    else if (menuPos == 12) {
      resetSim800();
      showMenu();
    }
    else if (menuPos == 13) {
      debugMode();
      showMenu();
    }
    delay(100);
    startMillis = currentMillis = millis();
  }
  if (currentMillis - startMillis >= period)
    pwrDown();
}

void waitToReg() {
  display.clearDisplay();
  display.println(F("Waiting to \nregister sim \ncard"));
  display.display();
  do {
    Serial1.print(F("AT+COPS?\r"));
  } while (Serial1.readString().indexOf(F("+COPS: 0,0,\"")) == -1);
}

String openURL(String string, bool ssl) {
  while (!GPRSCon)
    connectGPRS();
  display.clearDisplay();
  Serial1.print(F("AT+HTTPINIT\r"));
  display.println(F("HTTP \nIntialization\n"));
  display.display();
  if (Serial1.readString().indexOf(F("OK")) == -1)
    return;
  Serial1.print(F("AT+HTTPPARA=\"CID\",1\r"));
  Serial1.readString();
  display.print(F("Sending URL\nrequest"));
  display.display();
  Serial1.print(F("AT+HTTPPARA=\"URL\",\""));
  Serial1.print(string + "\"\r");
  if (Serial1.readString().indexOf(F("OK")) == -1)
    return;
  display.print(Serial1.readString());
  display.display();
  display.clearDisplay();
  Serial1.print(F("AT+HTTPPARA=\"REDIR\",1\r"));
  Serial1.readString();
  if (ssl) {
    Serial1.print(F("AT+HTTPSSL=1\r"));
    Serial1.readString();
  }
  else {
    Serial1.print(F("AT+HTTPSSL=0\r"));
    Serial1.readString();
  }
  Serial1.print(F("AT+HTTPACTION=0\r"));
  Serial1.readString();
  while (!Serial1.available());
  Serial1.readString();
  if (Serial1.readString().indexOf(F(",200,")) != -1)
    return;
  display.print(F("Downloading \ndata"));
  display.display();
  Serial1.print(F("AT+HTTPREAD\r"));
  string = Serial1.readString();
  string.trim();
  Serial1.print(F("AT+HTTPTERM\r"));
  Serial1.readString();
  return string;
}

String locInfo(byte save) {
  while (!GPRSCon)
    connectGPRS();
  display.clearDisplay();
  display.println(F("Getting\nlocation info\n"));
  display.display();
  Serial1.print(F("AT+CIPGSMLOC=1,1\r"));
  Serial1.readString();
  while (!Serial1.available());
  String s = Serial1.readString();
  if (s.indexOf(F(",")) == -1) {
    display.println(F("Failed to \nget info!"));
    display.display();
    delay(2000);
  }
  else {
    s = s.substring(s.indexOf(F(",")) + 1, s.indexOf(F("OK")));
    s.trim();
    const String data[4];
    for (byte i = 0; i < 4; i++) {
      data[i] = s.substring(0, s.indexOf(F(",")));
      s = s.substring(s.indexOf(F(",")) + 1, s.length());
    }
    if (save == 0) {
      display.clearDisplay();
      display.print(F("Dt:"));
      display.println(data[2]);
      display.print(F("Time:"));
      display.println(data[3]);
      display.println(F("Latitude: "));
      display.println(data[1]);
      display.println(F("Longitude: "));
      display.println(data[0]);
      display.display();
      while (!isButtonDown(btnEnt) && !isButtonDown(btnUp) && !isButtonDown(btnDwn));
    }
    else if (save == 1) {
      display.print(F("Connecing to\nupload server"));
      display.display();
      display.clearDisplay();
      s = "{\"Date\": \"" + data[2] + "\", \"Time\": \"" + data[3] + "\", \"Location link\": \"www.google.com/maps?q=" + data[1] + "," + data[0] + "\"}";
      Serial1.print(F("AT+HTTPINIT\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPPARA=\"CID\",1\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPSSL=0 \r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPPARA=\"URL\",\"api.jsonbin.io/b\"\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r"));
      Serial1.readString();
      // WARNING!!!  YOU MUST CHANGE the secret-key otherwise your location info will be sent to my JSON account!
      Serial1.print(F("AT+HTTPPARA=\"USERDATA\",\"secret-key: $2a$10$/4cwS1j8JzAgdbYKEDbeM.x19a0UM5C612PtEvoBv.hqtGagcY.DG\\r\\nprivate: true\"\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPDATA="));
      Serial1.print(String(s.length()) + ",2000\r");
      Serial1.readString();
      Serial1.print(s + "\r");
      Serial1.readString();
      Serial1.print(F("AT+HTTPACTION=1\r"));
      Serial1.readString();
      while (!Serial1.available());
      s = Serial1.readString();
      if (s.indexOf(F(",200,")) == -1 ) {
        display.print(F("Failed to\nupload info!"));
        display.display();
      }
      else {
        display.println(F("Location \nuploaded!"));
        display.display();
      }
      Serial1.print(F("AT+HTTPREAD\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPTERM\r"));
      Serial1.readString();
    }
    else if (save == 2) {
      writeEeprom("Dt:" + data[2] + "\nTime:" + data[3] + "\nLongitude:\n" + data[0] + "\nLatitude:\n" + data[1]);
      display.println(F("Info saved \nto Eeprom!:"));
      display.display();
      delay(2000);
    }
    else if (save == 3) {
      s = F("api.openweathermap.org/data/2.5/weather?lon=");
      s += data[0];
      s += F("&lat=");
      s += data[1];
      s += F("&units=metric&appid=0a3456488cb52d167293ee9ca1f00539"); // Please change the App id to your API key
      return s;
    }
  }
  s = "";
  return s;
}

void netInfo() {
  if (isItSleep) {
    wakeUp();
    isItSleep = false;
  }
  display.clearDisplay();
  display.println(F("Getting\nnetwork info"));
  display.display();
  const String data[2];
  String network;
  do {
    Serial1.print(F("AT+COPS?\r"));
    network = Serial1.readString();
    //Serial.println(network);
  } while (network.indexOf(F("+COPS: 0,0,\"")) == -1);
  network = network.substring(network.lastIndexOf(F(",\"")) + 2, network.lastIndexOf(F("\"")));
  exitBool = false;
  attachInterrupt(digitalPinToInterrupt(btnEnt), exitLoop, FALLING);
  while (!exitBool) {
    Serial1.print(F("AT+CSQ\r"));
    String quality = Serial1.readString();
    quality = quality.substring(quality.indexOf(F(": ")) + 2, quality.indexOf(F(",")));
    if (quality.toInt() < 10)
      quality = "Poor " + quality;
    else if (quality.toInt() < 15)
      quality = "Fair " + quality;
    else if (quality.toInt() < 20)
      quality = "Good " + quality;
    else
      quality = "Excellent " + quality;
    Serial1.print(F("AT+CCLK?\r"));
    const String s = Serial1.readString();
    char am_pm[] = "AM";
    byte hrs = 12;
    data[0] = s.substring(s.indexOf(F("\"")) + 1, s.indexOf(F(",")));
    data[1] = s.substring(s.indexOf(F(",")) + 1, s.indexOf(F("-")));
    if (data[1].substring(0, 2).toInt() > hrs) {
      am_pm[0] = 'P';
      hrs = data[1].substring(0, 2).toInt() - hrs;
    }
    else if (data[1].substring(0, 2).toInt() == hrs)
      am_pm[0] = 'P';
    else if (data[1].substring(0, 2).toInt() == 0);
    else if (data[1].substring(0, 2).toInt() < hrs)
      hrs = data[1].substring(0, 2).toInt();
    display.clearDisplay();
    display.println(network);
    display.println(F("Sig. Strength:"));
    display.println(quality);
    display.println(F("Date & Time:"));
    display.print(F("20"));
    display.println(data[0]);
    display.println(String(hrs) + data[1].substring(2, data[1].length() ) + " " + am_pm);
    display.display();
  }
}

void exitLoop() {
  if (isButtonDown(btnEnt)) {
    detachInterrupt(btnEnt);
    exitBool = true;
  }
}

bool checkGPRS() {
  Serial1.print(F("AT+SAPBR=2,1\r"));
  if (Serial1.readString().indexOf(F("+SAPBR: 1,1,")) != -1)
    return true;
  return false;
}

void connectGPRS() {
  if (isItSleep) {
    wakeUp();
    isItSleep = false;
  }
  if (checkGPRS()) {
    display.clearDisplay();
    display.println(F("Already \nconnected!"));
    display.display();
    delay(2000);
  }
  else {
    display.clearDisplay();
    display.println(F("Initializing \nSim800L\n"));
    display.display();
    Serial1.print(F("AT+CSCLK=0\r"));
    while (!Serial1.available());
    Serial1.readString();
    Serial1.print(F("AT+SAPBR=3,1,\"APN\",\"pwg\"\r"));   // Need to be changed to your APN
    Serial1.readString();
    display.println(F("Trying to\nconnect to the\naccess point"));
    display.display();
    display.clearDisplay();
    Serial1.print(F("ATE0\r"));
    Serial1.readString();
    Serial1.print(F("AT+SAPBR=1,1\r"));
    while (!Serial1.available());
    if (Serial1.readString().indexOf(F("OK")) != -1) {
      display.print(F("Connected!"));
      GPRSCon = true;
    }
    else {
      display.print(F("Failed to \nconnect!"));
      display.display();
      delay(1000);
      resetSim800();
      waitToReg();
    }
    Serial1.print(F("ATE1\r"));
    Serial1.readString();
  }
}

void disConnectGPRS() {
  if (!checkGPRS()) {
    display.clearDisplay();
    display.println(F("Already \ndisconnected!"));
    display.display();
    delay(2000);
  }
  else {
    display.clearDisplay();
    display.print(F("Disconnecting \nGPRS"));
    display.display();
    Serial1.print(F("AT+SAPBR=0,1\r"));
    Serial1.readString();
    delay(100);
    Serial1.print(F("AT+CSCLK=2\r"));
    Serial1.readString();
    GPRSCon = false;
  }
}

void readEeprom() {
  String s;
  for (int i = 0; EEPROM.read(i) != 0 && i < EEPROM.length(); i++)
    s += (char)EEPROM.read(i);
  display.clearDisplay();
  display.print(s);
  display.display();
  delay(100);
  while (!isButtonDown(btnEnt) && !isButtonDown(btnUp) && !isButtonDown(btnDwn));
}

void writeEeprom(String s) {
  for (int i = 0; i < s.length() && i < EEPROM.length(); i++) {
    EEPROM.update(i, s.charAt(i));
  }
}

void resetSim800() {
  if (isItSleep) {
    wakeUp();
    isItSleep = false;
  }
  else {
    display.clearDisplay();
    display.println(F("Restarting"));
    display.display();
    digitalWrite(resetPin, LOW);
    delay(100);
    digitalWrite(resetPin, HIGH);
    GPRSCon = false;
    delay(2000);
    waitToReg();
  }
}

void autoUp() {
  display.clearDisplay();
  display.println(F("To disable \nautoupload\nplease restart\nthe device"));
  display.display();
  delay(3000);
  while (true) {
    if (isItSleep)
      connectGPRS();
    locInfo(1);
    Serial1.print(F("AT+CPOWD=0\r"));
    Serial1.readString();
    digitalWrite(lcdBL, HIGH);  // keep the LCD backlight off during sleep time and wakeups to reduce power consumption
    display.clearDisplay();
    display.display();
    display.command( PCD8544_FUNCTIONSET | PCD8544_POWERDOWN);
    digitalWrite(pwrPin, LOW);
    for (int i = 0; i < COUNTER; i++)
      myWatchdogEnable (0b100001);  // 8 seconds
    //myWatchdogEnable (0b100000);  // 4
    sleep_disable();
    power_all_enable();
    display.begin();
    isItSleep = true;
  }
}

void toggle() {
  if (isButtonDown(btnEnt))
    digitalWrite(lcdBL, !digitalRead(lcdBL));
}

bool isButtonDown(byte pin) {
  if (digitalRead(pin) == LOW) {
    delay(30);
    if (digitalRead(pin) == LOW)
      return true;
    return false;
  }
  return false;
}

void showMenu() {
  for (byte i = menuStartAt; i < (menuStartAt + LCD_ROWS); i++) {
    byte markerY = (i - menuStartAt) * MENU_ROW_HEIGHT;
    if (i == menuPos) {
      display.setTextColor(WHITE, BLACK);
      display.fillRect(0, markerY, display.width(), MENU_ROW_HEIGHT, BLACK);
    }
    else {
      display.setTextColor(BLACK, WHITE);
      display.fillRect(0, markerY, display.width(), MENU_ROW_HEIGHT, WHITE);
    }
    if (i >= MENU_LENGTH)
      continue;
    display.setCursor(3, markerY + 2);
    display.print((char*)pgm_read_word(&(menu[i])));
  }
  display.display();
}

bool checkSim800() {
  Serial1.begin(9600);
  while (Serial1.available() > 0)
    Serial1.read();
  Serial1.print(F("AT\r"));
  delay(100);
  if (Serial1.available() > 0) {
    Serial1.read();
    return true;
  }
  else
    return false;
}

void wakeUp() {
  digitalWrite(resetPin, HIGH);
  digitalWrite(pwrPin, HIGH);
  display.clearDisplay();
  display.println(F("Waking up \nSim800L"));
  display.display();
  delay(4000);
  while (!checkSim800()) {
    display.clearDisplay();
    display.println(F("Sim800L is \nturned off or \nnot responding"));
    display.display();
    delay(2000);
    resetSim800();
  }
  waitToReg();
}

void pwrDown() {
  delay(250);   //debouncing
  isItSleep = true;
  GPRSCon = false;
  digitalWrite(lcdBL, HIGH);
  display.clearDisplay();
  display.display();
  display.command( PCD8544_FUNCTIONSET | PCD8544_POWERDOWN);
  digitalWrite(pwrPin, LOW);
  attachInterrupt(digitalPinToInterrupt(btnEnt), pinInterrupt, RISING);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  savePower();
  sleep_mode();
  sleep_disable();
  power_all_enable();
  MENU_LENGTH =  13;
  display.begin();
  digitalWrite(lcdBL, LOW);
  startMillis = currentMillis = millis();
  showMenu();
}

void debugMode() {
  Serial.begin(9600);
  display.clearDisplay();
  display.println(F("Press enter to\nexit debug \nmode"));
  display.display();
  exitBool = false;
  delay(250);   //debouncing
  attachInterrupt(digitalPinToInterrupt(btnEnt), exitLoop, FALLING);
  while (!exitBool) {
    if (Serial.available()) {
      Serial1.write(Serial.read());
    }
    if (Serial1.available()) {
      Serial.write(Serial1.read());
    }
  }
}

void pinInterrupt(void) {
  detachInterrupt(btnEnt);
}

void ledTx( boolean on) {
  if (on)  {
    pinMode( LED_BUILTIN_TX, OUTPUT);    // pin as output.
    digitalWrite( LED_BUILTIN_TX, LOW);  // pin low
  }
  else {
    pinMode( LED_BUILTIN_TX, INPUT);
  }
}

void ledRx( boolean on) {
  if (on) {
    pinMode( LED_BUILTIN_RX, OUTPUT);
    digitalWrite( LED_BUILTIN_RX, LOW);
  }
  else {
    pinMode( LED_BUILTIN_RX, INPUT);
  }
}

ISR(WDT_vect) {
  wdt_disable();
}

void myWatchdogEnable(const byte interval) {
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  wdt_reset();
  savePower();
  sleep_mode();            // now goes to Sleep and waits for the interrupt
}

void savePower() {
  ADCSRA = 0;
  power_adc_disable();
  ACSR |= (1 << ACD); // disable Analog comparator, saves 4 uA
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1 << AIN1D) | (1 << AIN0D);
  power_usart0_disable();
  power_usart1_disable();
  power_spi_disable();
  power_twi_disable();
  power_timer0_disable();  // Do not disable if you need millis()!!!
  power_timer1_disable();
  power_timer3_disable();
  PRR1 |= (uint8_t)(1 << 4);  //PRTIM4
  power_usart1_disable();
  // Disable the USB interface
  USBCON &= ~(1 << USBE);
  // Disable the VBUS transition enable bit
  USBCON &= ~(1 << VBUSTE);
  // Disable the VUSB pad
  USBCON &= ~(1 << OTGPADE);
  // Freeze the USB clock
  USBCON &= ~(1 << FRZCLK);
  // Disable USB pad regulator
  UHWCON &= ~(1 << UVREGE);
  // Clear the IVBUS Transition Interrupt flag
  USBINT &= ~(1 << VBUSTI);
  // Physically detact USB (by disconnecting internal pull-ups on D+ and D-)
  UDCON |= (1 << DETACH);
  power_usb_disable();  // Keep it here, after the USB power down
  // Disable functions
  power_all_disable();
}
