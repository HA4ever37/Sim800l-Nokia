#include <EEPROM.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define resetPin 6      // Reset pin for Sim800L
#define lcdBL 5         // LCD backlight pin
#define btnEnt 2
#define btnUp 3
#define btnDwn 7
#define MENU_LENGTH 12
#define MENU_ROW_HEIGHT 11
#define LCD_ROWS  5
#define RXLED 17
#define TXLED 30

Adafruit_PCD8544 display = Adafruit_PCD8544(15, 16, 9, 8, 4);

unsigned long startMillis, currentMillis;
const unsigned long period = 30000; // The value is a number in milliseconds
bool GPRSCon, isItSleep, exitBool;
byte menuPos = 0;
byte menuScreen = 0;
byte markerPos = 0;
byte menuStartAt = 0;
const String menu[12] = {"URL Request", "Network Info", "Location Info", "Connect", "Disconnect", "Light Switch",
                         "Power Down", "Sleep 24 Scnd", "Reset Sim800L", "Send Location" , "Save Location", "Last Saved"
                        };

void setup() {
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDwn, INPUT_PULLUP);
  pinMode(btnEnt, INPUT_PULLUP);
  pinMode(lcdBL, OUTPUT);
  pinMode(resetPin, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  digitalWrite(resetPin, HIGH);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, HIGH);
  digitalWrite(resetPin, HIGH);
  digitalWrite(lcdBL, LOW);
  display.begin();
  display.setContrast(50);
  display.setRotation(2);     // My screen is flipped! :/
  display.clearDisplay();
  Serial1.begin(9600);
  //Serial.begin(9600);
  //while (!Serial);
  delay(8000);  // You may increase this if 8 seconds isn't enough
  while (!checkSim800()) {
    display.clearDisplay();
    display.println(F("Sim800L is \nturned off or \nnot responding"));
    display.display();
    delay(2000);
    restSim800();
  }
  GPRSCon = checkGPRS();
  Serial1.print(F("ATE0\r"));
  Serial1.readString();
  showMenu();
  startMillis = millis();
}

void loop() {
  currentMillis = millis();
  if (isButtonDown(btnDwn) == true) {
    if (menuPos < MENU_LENGTH - 1) {
      menuPos++;
      if (menuPos > 3)
        menuStartAt++;
      showMenu();
    }
    delay(100);
    startMillis = currentMillis;
  }
  if (isButtonDown(btnUp) == true) {
    if (menuPos > 0) {
      menuPos--;
      if (menuStartAt > 0)
        menuStartAt--;
      showMenu();
    }
    delay(100);
    startMillis = currentMillis;
  }
  if (isButtonDown(btnEnt) == true) {
    if (menuPos == 0) {
      String s = openURL(F("raw.githubusercontent.com/HA4ever37/Sim800l/master/Sim800.txt")); // Change the URL to your text file link
      if (s == "ERROR" || s == "")  {
        display.clearDisplay();
        display.println(F("Bad request! \ntry again"));
        display.display();
        delay(2000);
      }
      else {
        display.clearDisplay();
        digitalWrite(lcdBL, LOW);
        /*display.print(s);
          for (byte i = 0; i < s.length(); i++) {
          //Serial.print(s.charAt(i));
          display.print(s.charAt(i));
          }
        */
        display.print(s);
        display.display();
        digitalWrite(lcdBL, HIGH);
        delay(500);
        digitalWrite(lcdBL, LOW);
        while (!isButtonDown(btnEnt) && !isButtonDown(btnUp) && !isButtonDown(btnDwn));
        display.clearDisplay();
        display.display();
      }
    }
    else if (menuPos == 1)
      netInfo();
    else if (menuPos == 2)
      locInfo(0);
    else if (menuPos == 3)
      connectGPRS();
    else if (menuPos == 4)
      disConnectGPRS();
    else if (menuPos == 5)
      toggle();
    else if (menuPos == 6)
      pwrDown();
    else if (menuPos == 7)
      sleep24();
    else if (menuPos == 8)
      restSim800();
    else if (menuPos == 9)
      locInfo(1);
    else if (menuPos == 10)
      locInfo(2);
    else if (menuPos == 11)
      readEeprom();
    showMenu();
    delay(100);
    startMillis = currentMillis = millis();;
  }
  if (currentMillis - startMillis >= period)
    pwrDown();
}

String openURL(String string) {
  while (!GPRSCon)
    connectGPRS();
  Serial1.print(F("AT+HTTPINIT\r"));
  display.clearDisplay();
  display.print(F("HTTP \nIntialization"));
  display.display();
  display.print(Serial1.readString());
  display.display();
  Serial1.print(F("AT+HTTPPARA=\"CID\",1\r"));
  Serial1.readString();
  display.print(F("Sending URL\nrequest"));
  display.display();
  Serial1.print(F("AT+HTTPPARA=\"URL\",\""));
  Serial1.print(string + "\"\r");
  if (Serial1.readString().equals(F("ERROR")))
    return "ERROR";
  display.print(Serial1.readString());
  display.display();
  display.clearDisplay();
  Serial1.print(F("AT+HTTPPARA =\"REDIR\",1\r"));
  Serial1.readString();
  Serial1.print(F("AT+HTTPSSL=1 \r"));
  Serial1.readString();
  Serial1.print(F("AT+HTTPACTION=0\r"));
  Serial1.readString();
  while (!Serial1.available());
  Serial1.readString();
  if (Serial1.readString().indexOf(",200,") != -1) {
    display.println(F("Request \failed!"));
    display.display();
    delay(500);
    return "ERROR";
  }
  else {
    display.println(F("Request \naccepted!\n"));
    display.display();
    while (Serial1.available() > 0)
      Serial1.read();
    display.print(F("Downloading \ndata"));
    display.display();
    Serial1.write("AT+HTTPREAD\r");
    string = Serial1.readString();
    string.trim();
    Serial1.print(F("AT+HTTPTERM\r"));
    Serial1.readString();
    string = string.substring(string.indexOf("\r") + 2, string.lastIndexOf("OK"));
    return string;
  }
}

void locInfo(byte save) {
  while (!GPRSCon)
    connectGPRS();
  display.clearDisplay();
  display.println(F("Getting\nlocation info"));
  display.display();
  Serial1.print(F("AT+CIPGSMLOC=1,1\r"));
  while (!Serial1.available());
  String s = Serial1.readString();
  if (s.indexOf(",") == -1) {
    display.println(F("Failed to \nget info!"));
    display.display();
    delay(2000);
  }
  else {
    s = s.substring(s.indexOf(",") + 1, s.indexOf("OK"));
    s.trim();
    String data[4];
    for (byte i = 0; i < 4; i++) {
      data[i] = s.substring(0, s.indexOf(","));
      s = s.substring(s.indexOf(",") + 1, s.length());
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
      s = "{\"Date\": \"" + data[2] + "\", \"Time\": \"" + data[3] + "\", \"Location link\": \"www.google.com/maps?q=" + data[1] + "," + data[0] + "\"}";
      s.trim();
      Serial1.print(F("AT+HTTPINIT\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPPARA = \"CID\",1\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPSSL=0 \r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPPARA=\"URL\",\"api.jsonbin.io/b\"\r"));
      Serial1.readString();
      Serial1.print(F("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r"));
      Serial1.readString();
      // YOU MUST CHANGE the secret-key otherwise your location data will be sent to my JSON account!
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
      if (s.indexOf(",200,") == -1 ) {
        display.print(F("Failed to\nupload info!"));
        display.display();
        delay(1000);
      }
      else {
        display.println(F("Location \nuploaded!"));
        display.display();
        delay(1000);
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
  }
}
void netInfo() {
  if (isItSleep) {
    wakeUp();
    isItSleep = false;
  }
  display.clearDisplay();
  display.println(F("Getting\nnetwork info"));
  display.display();
  String data[2], network;
  do {
    Serial1.print(F("AT+COPS?\r"));
    network = Serial1.readString();
    //Serial.println(network);
  } while (network.indexOf("+COPS: 0,0,\"") == -1);
  network = network.substring(network.indexOf("0,\"") + 3, network.lastIndexOf("\""));
  exitBool = false;
  attachInterrupt(digitalPinToInterrupt(btnEnt), exitLoop, FALLING);
  while (!exitBool) {
    Serial1.print(F("AT+CSQ\r"));
    String quality = Serial1.readString();
    quality = quality.substring(quality.indexOf(": ") + 2, quality.indexOf(","));
    //Serial.println(quality);
    if (quality.toInt() < 10)
      quality = "Poor " + quality;
    else if (quality.toInt() < 15)
      quality = "Fair " + quality;
    if (quality.toInt() < 20)
      quality = "Good " + quality;
    else
      quality = "Excellent " + quality;
    Serial1.print(F("AT+CCLK?\r"));
    String s = Serial1.readString();
    char am_pm[] = "AM";
    byte hrs = 12;
    data[0] = s.substring(s.indexOf("\"") + 1, s.indexOf(","));
    data[1] = s.substring(s.indexOf(",") + 1, s.indexOf("-"));
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
    display.println("  20" + data[0]);;
    display.println("  " + String(hrs) + data[1].substring(2, data[1].length() ) + " " + am_pm);
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
  if (Serial1.readString().indexOf("+SAPBR: 1,1,") != -1)
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
    Serial1.print(F("AT + CSCLK = 0\r"));
    delay(100);
    if (Serial1.available() > 0) {
      Serial1.readString();
      Serial1.print(F("AT + SAPBR = 3, 1, \"APN\",\"freedompop.foggmobile.com\"\r")); // Need to be changed to your APN
      Serial1.readString();
      display.println(F("Trying to\nconnect to the\naccess point"));
      display.display();
      display.clearDisplay();
      Serial1.print(F("AT+SAPBR=1,1\r"));
      while (!Serial1.available());
      if (Serial1.readString().indexOf("OK") != -1) {
        display.print(F("Connected!"));
        GPRSCon = true;
      }
      else {
        display.print(F("Failed to \nconnect!"));
        display.display();
        delay(1000);
        restSim800();
      }
    }
    else {
      display.print(F("Failed to \nconnect!"));
      display.display();
      delay(1000);
      restSim800();
    }
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
    display.print(F("Disconnect \nGPRS"));
    display.display();
    Serial1.print(F("AT+SAPBR=0,1\r"));
    display.print(Serial1.readString());
    display.display();
    //Serial1.readString();
    delay(100);
    Serial1.print(F("AT+CSCLK=2\r"));
    display.clearDisplay();
    display.print(Serial1.readString());
    display.display();
    GPRSCon = false;
  }
}

void readEeprom() {
  String s;
  for (int i = 0; i < EEPROM.length(); i++)
    s += (char)EEPROM.read(i);
  display.clearDisplay();
  display.print(s);
  display.display();
  delay(100);
  while (!isButtonDown(btnEnt) && !isButtonDown(btnUp) && !isButtonDown(btnDwn));
}

void writeEeprom(String s) {
  for (int i = 0; i < s.length(); i++) {
    EEPROM.update(i, s.charAt(i));
  }
}

void restSim800() {
  display.clearDisplay();
  display.println(F("Restarting"));
  display.display();
  Serial1.flush();
  //Serial.flush();
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  delay(10000);
  GPRSCon = false;
}

void sleep24() {
  GPRSCon = false;
  digitalWrite(lcdBL, HIGH);
  Serial1.print(F("AT+CPOWD=0\r"));
  Serial1.readString();
  display.clearDisplay();
  display.display();
  display.command( PCD8544_FUNCTIONSET | PCD8544_POWERDOWN);
  for (byte i = 0; i < 4; i++)
    myWatchdogEnable (0b100001);  // 8 seconds
  //myWatchdogEnable (0b100000);  // 4 seconds
  display.begin();
  digitalWrite(lcdBL, LOW);
  wakeUp();
}

void toggle() {
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
  startMillis = currentMillis;
  for (byte i = menuStartAt; i < (menuStartAt + LCD_ROWS); i++) {
    byte markerY = (i - menuStartAt) * MENU_ROW_HEIGHT;
    if (i == menuPos) {
      display.setTextColor(WHITE, BLACK);
      display.fillRect(0, markerY, display.width(), MENU_ROW_HEIGHT, BLACK);
    } else {
      display.setTextColor(BLACK, WHITE);
      display.fillRect(0, markerY, display.width(), MENU_ROW_HEIGHT, WHITE);
    }

    if (i >= MENU_LENGTH)
      continue;
    display.setCursor(2, markerY + 2);
    display.print(menu[i]);
  }
  display.display();
}

bool checkSim800() {
  while (Serial1.available() > 0)
    Serial1.read();
  Serial1.write("AT\r");
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
  display.clearDisplay();
  display.println(F("Waking up \nSim800L"));
  display.display();
  delay(10000);
  while (!checkSim800()) {
    display.clearDisplay();
    display.println(F("Sim800L is \nturned off or \nnot responding"));
    display.display();
    delay(2000);
    restSim800();
  }
  Serial1.print(F("ATE0\r"));
  Serial1.readString();
}

void pwrDown() {
  delay(250);
  isItSleep = true;
  GPRSCon = false;
  digitalWrite(lcdBL, HIGH);
  display.clearDisplay();
  display.display();
  display.command( PCD8544_FUNCTIONSET | PCD8544_POWERDOWN);
  digitalWrite(resetPin, LOW);
  attachInterrupt(digitalPinToInterrupt(btnEnt), pinInterrupt, RISING);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  power_adc_disable();
  power_usart0_disable();
  power_usart1_disable();
  power_spi_disable();
  power_twi_disable();
  power_usb_disable();
  sleep_mode();
  sleep_disable();
  power_all_enable();
  display.begin();
  showMenu();
  digitalWrite(lcdBL, LOW);
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

void ledRx( boolean on)
{
  if (on) {
    pinMode( LED_BUILTIN_RX, OUTPUT);
    digitalWrite( LED_BUILTIN_RX, LOW);
  }
  else {
    pinMode( LED_BUILTIN_RX, INPUT);
  }
}

ISR(WDT_vect) {
  wdt_disable();  // disable watchdog
}

void myWatchdogEnable(const byte interval) {
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_mode();            // now goes to Sleep and waits for the interrupt
}
