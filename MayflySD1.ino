//#define DEBUG
//#include <Adafruit_SleepyDog.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SD.h>
#include <CayenneLPP.h>
#include <Sodaq_PcInt.h>
#include <Sodaq_DS3231.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RAK811V2.h>
#include <LowPower.h>
#include <Adafruit_ADS1X15.h>

// ---------------- SD Download / Maintenance Mode ----------------
#define MAINT_BUTTON_PIN 21   // MayFly button on D21 (active HIGH)

bool maintenanceMode = false;

// switched power lines pin
#define SWITCHED_POWER 22 // Enable pin for switched 3.3

// Error flag and error LED pin
#define ERROR_LED 9
bool error;

// Operation Success pin
#define SUCCESS_LED 8
bool success;

// DateTime variable for the time
DateTime now;

// create a string buffer for SD card and printing info to the debugging serial port
String string_buffer;

// File object variable
File file;
// COnfiguration of battery voltage variables
int batteryPin = A6;       // on the Mayfly board, pin A6 is connected to a resistor divider on the battery input; R1 = 10 Mohm, R2 = 2.7 Mohm
int batterysenseValue = 0; // variable to store the value coming from the analogRead function
float batteryvoltage;      // the battery voltage as calculated by the formula below
// formula for battery level calculation is batteryvoltage = (3.3/1023.) * 4.7 * batterysenseValue;

// create and configure CayenneLPP buffer wit a buffer size of 51
CayenneLPP lpp(51);

// Create and configure DHT21 object and variables for temperature and humidity
#define DHTPIN 6
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);
float temperature2;
float humidity;

// Configure DS18B20 sensor and variable for temperature
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
float temperature = 0;

//ads
Adafruit_ADS1115 ads;

// configure variables for hd-38 sensor
int16_t humedadSueloRaw = 0;
int humedadSueloPorcentaje = 0;
const int16_t VALOR_SECO = 26400;   // seco
const int16_t VALOR_HUMEDO = 10000; // humedo

// declare LoRa Radio
RAK811 lora(Serial1, Serial);
bool joined_network;

// pin for the RTC alarm interrupt
int interruptPin = A7;
// a flag to indicate that device was awaken by RTC, set to true in setup to make sure measurments are executed once when loop is first executed
volatile int rtcWakeFlag = 1;


// Interrupt service routine for RTC alarm
void INT0_ISR()
{
  // set flag to indicate device was awaken by RTC
  rtcWakeFlag = 1;
}



void setup()
{
  // configure switched power, error and success pins
  pinMode(SWITCHED_POWER, OUTPUT);
  digitalWrite(SWITCHED_POWER, LOW);
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(ERROR_LED, LOW);
  pinMode(SUCCESS_LED, OUTPUT);
  digitalWrite(SUCCESS_LED, LOW);
  // pinMode(tippingBucketPin, INPUT);

  // blink success led to notify powered on state
  blink(SUCCESS_LED, 3, 500);

  // Check maintenance button (active HIGH)
  maintenanceMode = maintenanceButtonPressedAtBoot();

  if (maintenanceMode) {
  // Visual confirmation
  blink(SUCCESS_LED, 2, 200);

  Serial.begin(57600);

  // Keep sensors off
  digitalWrite(SWITCHED_POWER, LOW);

  // Init SD (no infinite loop)
  if (!SD.begin(12)) {
    Serial.println(F("ERR: SD init failed"));
    blink(ERROR_LED, 10, 200);
    while (true) {
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    }
  }
    runSdDownloadShell();  // never returns
  }

  // initialize debugging serial
  #if defined DEBUG
  Serial.begin(9600);
  #endif

  //Initializa ds18b20 sensor and check presence
  ds18b20.begin();
  if (!ds18b20.getDS18Count())
  {
    blink(ERROR_LED, 3, 500);
    #if defined DEBUG
    Serial.println(F("Primary temperature sensor error"));
    Serial.flush();
    #endif
  } 

// Initialize ADS1115
  if (!ads.begin()) {
    #if defined DEBUG
    Serial.println(F("Error al inicializar el ADS1115"));
    #endif
    blink(ERROR_LED, 5, 200);
  } else {
    ads.setGain(GAIN_ONE);
  }


// --- Boot self-test: power sensors and verify DHT21 ---
digitalWrite(SWITCHED_POWER, HIGH);

// Give the switched rail and DHT time to stabilize.
// DHT21 often needs >1s after power-up, especially with longer leads.
delay(1500);

dht.begin();

// Throw away one read (very common to be NaN right after begin/power-up)
(void)dht.readTemperature();
(void)dht.readHumidity();
delay(1200);

float t = dht.readTemperature();
float h = dht.readHumidity();

// Retry once more if needed
if (isnan(t) || isnan(h)) {
  delay(1500);
  t = dht.readTemperature();
  h = dht.readHumidity();
}

if (isnan(t) || isnan(h)) {
  blink(ERROR_LED, 3, 500);
#if defined DEBUG
  Serial.println(F("Secondary Temp/Humidity sensor error (boot self-test)"));
  Serial.flush();
#endif
}

// Power off sensors after self-test
digitalWrite(SWITCHED_POWER, LOW);

  // initialize the RAK811 serial port
  Serial1.begin(9600);
  if (lora.rk_begin())
  {
    #if defined DEBUG
    Serial.println(F("LoRa initialization OK"));
    #endif
    joined_network = true;
    blink(SUCCESS_LED, 3, 1000);
  }
  else
  {
    #if defined DEBUG
    Serial.println(F("LoRa initialization Error"));
    #endif
    joined_network = false;
    blink(ERROR_LED, 3, 1000);
  }
  //Test to see if payload limit reached for current data rate when sending data
  //lora.sendRawCommand(F("at+set_config=lora:adr:1",2000));
  //initilize SD
  while (!SD.begin(12))
  { // si no se logra inicializar la SD
    #if defined DEBUG
    Serial.println(F("No SD"));
    #endif
    digitalWrite(ERROR_LED, HIGH);
    delay(500); // espera antes de volver a intentarlo
  }
  digitalWrite(ERROR_LED, LOW);
  // adjust RTC if neccesary
  //DateTime dt (__DATE__, __TIME__);
  //rtc.setDateTime(dt); //Adjust date-time as defined 'dt' above
  // Configure the interrupt to awake the device
  pinMode(interruptPin, INPUT_PULLUP);
  PcInt::attachInterrupt(interruptPin, INT0_ISR);
  // initialize the rtc
  rtc.begin();
#if defined DEBUG
  rtc.enableInterrupts(EveryMinute);
#else
  setupNextAlarm15();
#endif
  // atach tipping bucket interrupt
 // pinMode(tippingBucketPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(tippingBucketPin), TB_ISR, FALLING);
}

void loop()
{
  //enable the watchdog timer in case the program halts
  //Watchdog.enable(8000);
  // check if the device was awaken by RTC
  if (rtcWakeFlag == 1)
  {
    // This section clears the alarm flag of the RTC
    //rtc.clearINTStatus();
    // turn the switched source on
    digitalWrite(SWITCHED_POWER, HIGH);
    // initialize DHT library
    dht.begin();
    // read DS18B20
    ds18b20.requestTemperatures();
    temperature = ds18b20.getTempCByIndex(0);
    if (temperature == DEVICE_DISCONNECTED_C)//try to read only one more time in case of an error
    {
      delay(500);
      ds18b20.requestTemperatures();
      temperature = ds18b20.getTempCByIndex(0);
    }
    //reset the watchdog timer at this point to avoid device reset
    //Watchdog.reset();
   
    // delay for DHT21 startup
    delay(500);
    // read DHT21
    temperature2 = dht.readTemperature();
    humidity = dht.readHumidity();

    //reinitialize ads1115
  
    if (!ads.begin()) {
      #if defined DEBUG
      Serial.println(F("Error al inicializar el ADS1115 tras despertar"));
      #endif
    } else {
      ads.setGain(GAIN_ONE);
    }

    // read hd38 (ads AIN0)
    humedadSueloRaw = ads.readADC_SingleEnded(0);
    // Convertir a porcentaje y asegurar que se mantiene entre 0 y 100
    humedadSueloPorcentaje = map(humedadSueloRaw, VALOR_SECO, VALOR_HUMEDO, 0, 100);
    humedadSueloPorcentaje = constrain(humedadSueloPorcentaje, 0, 100);

    
    // power sensor off as soon as it finishes reading
    digitalWrite(SWITCHED_POWER, LOW);
    string_buffer = "";
    batterysenseValue = analogRead(batteryPin);
    batteryvoltage = (3.3 / 1023.) * 4.7037 * batterysenseValue;
    now = rtc.now();
    // set wake flag to false again
    //rtcWakeFlag = 0;
    // Might be a good idea to store the files in a y/m/d format to make it easier to navigate files in SD
    string_buffer = String(now.year()-2000) + "_" + String(now.month()) + "_" + String(now.date()) + ".csv";
    // string_buffer = String(now.date()) + "_" + String(now.month()) + ".csv";
    //  section to verify if the file exists, if it doesn't then create it and generate the text file header
    //reset the watchdog timer at this point to avoid device reset
    //Watchdog.reset();
    if (!SD.exists(string_buffer))
    {
      file = SD.open(string_buffer, FILE_WRITE);
      if (file)
      {
        file.println(F("Fecha/hora,Temperatura 1,Temperatura 2,Humedad,Bateria,Humedad suelo"));
        file.close();
      }
      else{
        #if defined DEBUG
          Serial.println("Error al crear archivo");
        #endif
        error = true;}
    }
    file = SD.open(string_buffer, FILE_WRITE);
    string_buffer = "";
    now.addToString(string_buffer);
    string_buffer.concat("," + String(temperature) + "," + String(temperature2) + "," + String(humidity) + "," + String(batteryvoltage) + "," + String(humedadSueloPorcentaje));
    #if defined DEBUG
      Serial.println(string_buffer);
    #endif

    #if defined DEBUG
      Serial.println(string_buffer);
      Serial.print("Suelo RAW: "); Serial.print(humedadSueloRaw);
      Serial.print(" | Suelo %: "); Serial.println(humedadSueloPorcentaje);
    #endif

    if (file)
    {
      file.println(string_buffer);
      file.close();
      #if defined DEBUG
        Serial.println(F("Succesfully written to the SD"));
      #endif
      error = false;
    }
    else{
      #if defined DEBUG
        Serial.println("Error de escritura");
      #endif
      error = true;}
    //reset the watchdog timer at this point to avoid device reset
    //Watchdog.reset();
    if (joined_network) SendCayenne();
    #if !defined DEBUG
      setupNextAlarm15(); // Schedule the next 15-min interval
    #endif
    // update error status pin
    if (error)
    {
      digitalWrite(ERROR_LED, HIGH);
    }
    // This section clears the alarm flag of the RTC
    rtc.clearINTStatus();
    rtcWakeFlag = 0;
  }

 
  #if defined DEBUG
    Serial.println(F("Going to sleep"));
    Serial.flush();
  #endif
  //reset the watchdog timer and disable it before going to sleep mode
  //Watchdog.reset();
  //Watchdog.disable();
  // This section puts the device in deep sleep
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}


void SendCayenne()
{
  //reset the watchdog timer at this point to avoid device reset
  //Watchdog.reset();
  lora.rk_wake();
  lpp.reset();
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(3, humidity);
  lpp.addAnalogInput(10, batteryvoltage);
  lpp.addAnalogInput(11, humedadSueloPorcentaje);
  lpp.addTemperature(2, temperature2);
  //Watchdog.reset();
  if (!lora.rk_sendBytes(2, lpp.getBuffer(), lpp.getSize()))//check if te payload size is not supported by the actual Data Rate (Error code 101)
  {
    //reset the watchdog timer at this point to avoid device reset
    //Watchdog.reset();
    //Send data again without the last 4 bytes correspondig to the last temperature channel data (channel byte plus type byte plus 2 data bytes for a temperature sensor)
    lora.rk_sendBytes(2,lpp.getBuffer(), lpp.getSize() - 8);
    //reset the watchdog timer at this point to avoid device reset
    //Watchdog.reset();
  }
  lora.rk_sleep();
  //reset the watchdog timer at this point to avoid device reset
  //Watchdog.reset();
}

uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
  }
  return crc;
}

const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void b64encode3(const uint8_t in[3], int len) {
  uint8_t o0 = (in[0] & 0xFC) >> 2;
  uint8_t o1 = ((in[0] & 0x03) << 4) | (((len > 1 ? in[1] : 0) & 0xF0) >> 4);
  uint8_t o2 = (((len > 1 ? in[1] : 0) & 0x0F) << 2) | (((len > 2 ? in[2] : 0) & 0xC0) >> 6);
  uint8_t o3 = ((len > 2 ? in[2] : 0) & 0x3F);

  Serial.write(B64[o0]);
  Serial.write(B64[o1]);
  Serial.write(len > 1 ? B64[o2] : '=');
  Serial.write(len > 2 ? B64[o3] : '=');
}

void sd_list_root() {
  File root = SD.open("/");
  if (!root) { Serial.println(F("ERR: cannot open /")); return; }

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      Serial.print(entry.name());
      Serial.print('\t');
      Serial.println(entry.size());
    }
    entry.close();
  }
  root.close();
  Serial.println(F("OK"));
}

void sd_send_file(const String &path) {
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f) { Serial.println(F("ERR: nofile")); return; }

  uint32_t size = f.size();
  Serial.print(F("BEGIN "));
  Serial.print(path);
  Serial.print(' ');
  Serial.println(size);

  uint32_t crc = 0xFFFFFFFFUL;

  uint8_t buf[48]; // multiple of 3 is nice for base64
  while (true) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;

    for (int i = 0; i < n; i++) crc = crc32_update(crc, buf[i]);

    for (int i = 0; i < n; i += 3) {
      uint8_t in3[3] = {
        buf[i],
        (uint8_t)((i + 1 < n) ? buf[i + 1] : 0),
        (uint8_t)((i + 2 < n) ? buf[i + 2] : 0)
      };
      int len = (n - i >= 3) ? 3 : (n - i);
      b64encode3(in3, len);
    }
    Serial.println(); // newline per chunk
  }

  f.close();
  crc ^= 0xFFFFFFFFUL;

  Serial.print(F("END "));
  Serial.println(crc, HEX);
}

void sd_send_all_root() {
  File root = SD.open("/");
  if (!root) { Serial.println(F("ERR: cannot open /")); return; }

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      String p = String("/") + entry.name();
      entry.close();
      sd_send_file(p);
    } else {
      entry.close();
    }
  }

  root.close();
  Serial.println(F("OK"));
}

// Non-blocking-ish heartbeat: wait up to wait_ms, but return early if serial data arrives
void waitOrSerial(unsigned long wait_ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < wait_ms) {
    if (Serial.available()) return;
    delay(5);
  }
}

String readLineBlocking() {
  String s;
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        s.trim();
        if (s.length() > 0) return s;
        s = "";
      } else {
        if (s.length() < 120) s += c;
      }
    }
    // Keep CPU responsive without burning power
    delay(2);
  }
}

void runSdDownloadShell() {
  Serial.println(F("\n=== SD DOWNLOAD MODE ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  ls                -> list files in SD root"));
  Serial.println(F("  get /file.csv     -> download one file (Base64 + CRC32)"));
  Serial.println(F("  getall            -> download all files in SD root"));
  Serial.println(F("  help              -> show this help"));
  Serial.println(F("  exit              -> sleep forever (power-cycle to log)"));
  Serial.println(F("========================\n"));

  while (true) {
    // --- Maintenance heartbeat pattern (alternating LEDs) ---
    digitalWrite(SUCCESS_LED, HIGH);
    waitOrSerial(150);
    digitalWrite(SUCCESS_LED, LOW);

    digitalWrite(ERROR_LED, HIGH);
    waitOrSerial(150);
    digitalWrite(ERROR_LED, LOW);

    waitOrSerial(1500);

    // If no command, loop again (keeps heartbeat alive)
    if (!Serial.available()) continue;

    Serial.print(F("> "));
    String cmd = readLineBlocking();

    if (cmd == "ls") {
      sd_list_root();
    }
    else if (cmd == "getall") {
      sd_send_all_root();
    }
    else if (cmd.startsWith("get ")) {
      String path = cmd.substring(4);
      path.trim();
      if (path.length() == 0 || path[0] != '/') {
        Serial.println(F("ERR: usage get /filename"));
      } else {
        sd_send_file(path);
      }
    }
    else if (cmd == "help" || cmd == "?") {
      Serial.println(F("Commands: ls | get /file | getall | exit"));
    }
    else if (cmd == "exit") {
      Serial.println(F("Bye. Sleeping forever. Power-cycle to return to logging."));
      Serial.flush();
      while (true) {
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      }
    }
    else {
      Serial.println(F("ERR: badcmd (type 'help')"));
    }
  }
}



bool maintenanceButtonPressedAtBoot() {
  pinMode(MAINT_BUTTON_PIN, INPUT);  // external pulldown exists

  // Small debounce / stabilization delay
  delay(20);

  // Check if button is held
  if (digitalRead(MAINT_BUTTON_PIN) != HIGH) return false;

  // Confirm it's still pressed after a short delay
  delay(80);
  return (digitalRead(MAINT_BUTTON_PIN) == HIGH);
}

void maintenanceLedPattern() {
  digitalWrite(SUCCESS_LED, HIGH);
  delay(150);
  digitalWrite(SUCCESS_LED, LOW);

  digitalWrite(ERROR_LED, HIGH);
  delay(150);
  digitalWrite(ERROR_LED, LOW);

  delay(1500);
}

void blink(int pin, int cycles, int delay_time_ms)
{
  for (int i = 0; i < cycles; i++)
  {
    digitalWrite(pin, HIGH);
    delay(delay_time_ms);
    digitalWrite(pin, LOW);
    delay(delay_time_ms);
  }
}

void setupNextAlarm15() {
    DateTime now = rtc.now();
    uint8_t nextMinute = (now.minute() / 15 + 1) * 15;
    
    if (nextMinute >= 60) {
        nextMinute = 0; 
    }

    // --- SMART JITTER LOGIC ---
    // Seed the random generator using internal noise from an unconnected pin 
    // or the low bits of your battery reading.
    randomSeed(analogRead(A0) + analogRead(batteryPin));
    
    // Pick a random second between 0 and 55. 
    // This spreads 20 devices across nearly a full minute.
    uint8_t randomSecond = random(0, 30); 

    // Use your specific library syntax:
    // (Type, Daydate, Hour, Minute, Second)
    rtc.enableInterrupts(MATCH_MINUTES, 0, 0, nextMinute, randomSecond); 
    
    #if defined DEBUG
      Serial.print(F("Next Alarm: "));
      Serial.print(nextMinute);
      Serial.print(F(":"));
      Serial.println(randomSecond);
    #endif
}