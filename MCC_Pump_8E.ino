/*
MCC (Master Central Controller). Wemos D1 R32
Скетч центрального контроллера домашней системы дистанционного беспроводного контроля состояния
Насосно-Аккумуляторной станции воды с местной индикауией и локальным WiFi сервером состояния. 
Include:
1. MCC (Master Central Controller) 
2. RCWPS (Remote Controller for Water Pump Station)

Main components MCC:
MCC             Wemos D1 R32
Remote Control  nRF2401L
Display         LCD1602 over I2C
RTC             DS1307
SD-MMC

By Ochilnik, 2024
*/

#include "SPI.h"                      //  Подключаем библиотеки для работы с шиной SPI (nRF2401)
#include "nRF24L01.h"                 //  Подключаем библиотеки для работы с nRF2401
#include "RF24.h"                     //  Подключаем библиотеки для работы с nRF2401
#include <Wire.h>                     //  Подключаем библиотеку для работы с шиной I2C (LCD, RTC)
#include <LiquidCrystal_I2C.h>        //  Подключаем библиотеку для работы с LCD дисплеем по шине I2C
#include <driver/timer.h>             //  Подключаем библиотеку Timer from ESP-IDF
#include "uRTCLib.h"                  //  Подключаем библиотеку для работы с DS1307 по шине I2C
#include "FS.h"                       //  Подключаем библиотеку для работы с SDMMC
#include "SD_MMC.h"                   //  Подключаем библиотеку для работы с SDMMC
#include <WiFi.h>                     //  Подключаем библиотеку для работы с WiFi

#define TIMER_GROUP TIMER_GROUP_0     // Группа таймеров
#define TIMER_IDX   TIMER_0           // Индекс таймера
#define TIMER_DIVIDER 40000           // Предделитель (APB тактирование / 40000 = 2 kГц)
#define TIMER_ALARM_VALUE 240000      // Срабатывание через 30 секунд (в 0,5 милисекундах)
#define NUM_AVG 10                    // Количество циклов для вычисления среднего значения тока
#define REQ_BUF_SZ   120              // size of buffer used to capture HTTP requests
#define ADRDAY 1                      // 1 byte address currentDay in I2C AT24C32 memory (addr: 1 - 56)
#define ADRNUMON 2                    // 1 byte address numOnPump in I2C AT24C32 memory (addr: 1 - 56)
#define ADRCOUNT 3                    // 2 bytes address countOnPump in I2C AT24C32 memory (addr: 1 - 56)
#define ADRCOUNTDAY 5                 // 2 bytes address vountDayPump in I2C AT24C32 memory (addr: 1 - 56)

LiquidCrystal_I2C lcd(0x27,16,2);     // Объявляем  объект lcd, (адрес I2C = 0x27, количество столбцов = 16, количество строк = 2)
RF24 radio(13, 5);                    // Объявляем  объект radio. порты D13 - CE, D5 - CSN
String bits = "";                     // Объявляем  строковый объект bits

byte rtcModel = URTCLIB_MODEL_DS1307; // Set RTC model
uRTCLib rtc;                          //  Объявляем  объект rtc - часы

File webFile;                         // the web page file on the SD card
char HTTP_req[REQ_BUF_SZ] = {0};      // buffered HTTP request stored as null terminated string
char req_index = 0;                   // index into HTTP_req buffer

const char *ssid = "SSID";              // name WiFi
const char *password = "password";        // password WiFi
NetworkServer server(80);                 // Объявляем сетевой объект server


const uint64_t addr_tx = 0xAABBCCDD22LL;  // адрес коннекта на передачу
const uint64_t addr_rx = 0xAABBCCDD11LL;  // адрес коннекта на прием
const uint8_t num_ch = 119;               // номер канала
const float timeCycle = 4.19424;          // One cycle time is Arduino UNO counter

bool voltageOn;                           // Voltage is ON to pump
bool commandOnPump;                       // command to ON pump from RCWPS (Remote Controller for Water Pump Station)
bool commandOffPump;                      // command to OFF pump from RCWPS (Remote Controller for Water Pump Station)
bool commandBlock;                        // command to Block pump from RCWPS (Remote Controller for Water Pump Station)
bool stateAlarm, alarmMCC, alarmRCWPS, stateOnPump, prevStateOnPump, stateBlock;  // Current States signals from RCWPS
bool trigerBlock = 0, prevBlock = 0;      // trigger flags
bool tMenu, prevMenu;                     // touch button Menu, previos state button Menu
bool codErrMCC[8];                        // code of errors from MCC
bool codErrRCWPS[8];                      // code of errors from RCWPS
volatile bool backLight = false;          // Global state LCD light
volatile bool flagLight = false;          // Global flag state LCD light
volatile bool flagCount = false;          // Global flag enable counters from rtc
volatile bool flagPulse = false;          // Global flag is pulse from rtc.sq 1Hz
bool webOn, webOff, webBlock;             // command ON, OFF, Block from web client
bool pumpON, pumpOFF;                     // summ commands from MCC & WebClient

//uint8_t ramAdr = 1;
uint8_t data_tx[5] = {0};                 // Array data to transmit
uint8_t data_rx[15] = {0};                // Array data to recieve
uint8_t data_tx_previos[5] = {0};         // Array data that transmit last time
uint8_t numOnPump;                        // number of pump starts
uint8_t currentDay;                       // today
uint8_t numScreen = 1;                    // number current screen on Display
const uint8_t lcdScreens = 3;             // number screens lcd Display           
uint16_t year;                             // current year
uint8_t month;                            // current month
uint8_t day;                              // current day
uint8_t hour;                             // current hour
uint8_t minute;                           // current minute
uint8_t dayHour;                          // hours in day counter
uint8_t dayMinute;                        // minutes in day counter
uint8_t daySecond;                        // seconds in day counter
uint8_t tHour;                            // hours in last counter
uint8_t tMinute;                          // minutes in last counter
uint8_t tSecond;                          // seconds in last counter
uint8_t lowByte;                          // Младший байт
uint8_t highByte;                         // Старший байт

uint16_t countOnPump;                     // time last start
uint16_t countDayPump;                    // pump operating time for the whole day
const uint16_t maxOnPump = 600;           // max time pump is working at once
const uint16_t maxDayPump = 3600;         // max time pump is working at day

int timeToOff = 0;                        // Оставшееся время до отключения насоса
int humidity;                             // Humiduty from RCWPS (Remote Controller for Water Pump Station)
int pressure;

float currentRMS;                         // RMS current value
float temperature;                        // Temperatures from RCWPS (Remote Controller for Water Pump Station)
const float maxCurrent = 2.95;            // max RMS current value
const float minCurrent = 2.5;             // min RMS current value
const float maxTemp = 30;                 // max temperatures
float avgCurrent;                         // average currentRMS

/* Error code codErrRCWPS
    0 - If output onPump is ON (inverse) and no voltage
    1 - If output blockPump is ON (inverse) and voltage on
    2
    3
    4
    5
    6
    7

  Error code codErrMCC
    0 - one switching time is longer than 10 minutes
    1 - working time per day longer than 60 minutes
    2 - the consumed current is above the nominal value
    3 - the consumed current is below the nominal value
    4 - the temperature is above 30C
    5 - 
    6
    7
*/
    

/* TX Control words structures
BYTE 0    0     1     2     3     4     5     6     7
  Pin     Do17  Di33  Di32  Di27      
  Name    alarm onM   offM  block
BYTE 1    0     1     2     3     4     5     6     7
  Pin     
  Name    


/* RX Control words structures
BYTE 0    0     1     2     3     4     5     6     7
  Pin     
  Name    UM    onM   offM  block
BYTE 1    0     1     2     3     4     5     6     7
  Pin     
  Name    alarm onM   block
BYTE 2,3
  Pin     
  Name    int IM
BYTE 4,5
  Pin     
  Name    P 
BYTE 6,7
  Pin     
  Name    int temperature
BYTE 8
  Pin     
  Name    int relative_humidity
BYTE 9
  Pin     -
  Name    codErrRCWPS[8]                      // Array of errors codes from RCWPS    
BYTE 10
  Pin     -
  Name    uint8_t rest cycles Timer1  
*/

// Pinout configuration
uint8_t buttonMenu = 4;         // button menu on is pressed
uint8_t buttonOnPumpPin = 27;   // button pump on is pressed
uint8_t buttonOffPumpPin = 32;  // button pump off is pressed
uint8_t buttonBlockPin = 33;    // triger blocking pump is on
uint8_t ledOnPumpPin = 16;      // led pump is on
uint8_t ledAlarmPin = 17;       // led alarm is on
uint8_t ledBlockPin = 25;       // led blocking pump is on
uint8_t buzzerPin = 26;         // buzzer alarm is on
uint8_t rtcSQPin = 35;          // rtc wave 1Hz input

// Выключение подсветки дисплея. функция прерывания от таймера (ISR). IRAM_ATTR sence is load function to RAM
void IRAM_ATTR onTimer(void* arg){
  timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_IDX); // Очистка флага прерывания таймера
  // код для обработки прерывания
  backLight = false;                //  Выключаем подсветку LCD дисплея
  flagLight = true;                 // flag indicate that backLight is changed
  //timer_group_enable_alarm_in_isr(TIMER_GROUP, TIMER_IDX);  // Если авто-перезагрузка выключена, сбрасываем таймер
  timer_pause(TIMER_GROUP, TIMER_IDX);  // Остановить таймер
  timer_set_alarm(TIMER_GROUP, TIMER_IDX, TIMER_ALARM_EN);  // Once triggered, the alarm is disabled automatically and needs to be re-enabled to trigger again.
}

// function interrupt counters operating time from external wave 1Hz
void IRAM_ATTR counterOn(){
  countOnPump++;
  flagPulse = true;
}

void setup(){
// ******************************** Settings INPUT/OUTPUT ***********************
  //pinMode(buttonMenu, INPUT_PULLUP);      //  button menu
  pinMode(buttonOnPumpPin, INPUT_PULLUP);   // button pump on
  pinMode(buttonOffPumpPin, INPUT_PULLUP);  // button pump off
  pinMode(buttonBlockPin, INPUT_PULLUP);    // button blocking pump
  pinMode(rtcSQPin, INPUT);                 // SQ wave from RTC
  pinMode(ledOnPumpPin, OUTPUT);            // led pump is on
  pinMode(ledAlarmPin, OUTPUT);             // led signal attention
  pinMode(ledBlockPin, OUTPUT);             // led blocking pump is on
  pinMode(buzzerPin, OUTPUT);               // sound signal attention

// ******************************** Settings Serial port **************************
  Serial.begin(115200);

// **************************** Settings Radio Interface *****************************
  Serial.println("Reciver/Transmitter ON");
  radio.begin();                    // инициализация радиомодуля
  delay(2000);
  //radio.setPayloadSize(7);        // Установить статичный размер блока данных пользователя в байтах (0-32)
  radio.setDataRate(RF24_250KBPS);  // Установить скорость передачи данных RF24_1MBPS или RF24_2MBPS или RF24_250KBPS
  /*switch(radio.getDataRate()){                            //  Проверка установленной скорости radio.getDataRate()
    case RF24_1MBPS   : Serial.println("setDataRate  1 MBPS"); break; // Если полученное значение RF24_1MBPS
    case RF24_2MBPS   : Serial.println("setDataRate  2 MBPS"); break; // Если полученное значение RF24_2MBPS
    case RF24_250KBPS : Serial.println("setDataRate  250 KBPS"); break;
  } */
  radio.setCRCLength(RF24_CRC_8);   // Установить размер CRC 8 bit или 16 bit
  radio.setChannel(num_ch);         // установка номера канала
  radio.setAutoAck(true);           // подтверждение приема
  radio.setPALevel(RF24_PA_MAX);    // уровень питания усилителя RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
 
  //-----------------------Reciever part-----------------------
  //radio.enableDynamicPayloads();  //включить поддержку динамического размера полезной нагрузки на приемнике
  radio.openReadingPipe(1, addr_rx); // открыть коннектор на прием
  //closeReadingPipe();             // Закрыть коннект открытый ранее для прослушивания (приёма данных).
  //radio.enableAckPayload();       // разрешить отсылку данных в ответ на входящий сигнал
  radio.startListening();           // перевод модуль в режим работы приемника

  //---------------------Transmitter part--------------------------
  radio.setRetries(5, 15);          // Указать время ожидания = (число+1) * 250 мкс и количество попыток отправки данных от 0 до 15
  radio.openWritingPipe(addr_tx);   // открыть коннект на отправку
  //radio.stopListening();          // перевод модуля в режим работы передатчика
  //radio.powerUp();                // режим полной нагрузки
  //radio.powerDown();              // режим пониженного потребления

// ************************* Settings LCD on I2C *************************
  lcd.init();                       //  Инициируем работу с LCD дисплеем
  //lcd.backlight();                  //  Включаем подсветку LCD дисплея
  //lcd.setCursor(0, 0);              //  Устанавливаем курсор в позицию (0 столбец, 0 строка)

// ********************** Settings backlight timer LCD **********************
  timer_config_t timeconf = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = TIMER_DIVIDER
  };
  //timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0);
  timer_init(TIMER_GROUP, TIMER_IDX, &timeconf);
  timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, TIMER_ALARM_VALUE); // Установка значения сигнализации
  timer_enable_intr(TIMER_GROUP, TIMER_IDX);  // Включить прерывания
  timer_isr_register(TIMER_GROUP, TIMER_IDX, onTimer, NULL, ESP_INTR_FLAG_IRAM, NULL); // Привязать обработчик прерывания
  //timer_start(TIMER_GROUP, TIMER_IDX);  // Запустить таймер
  
// *********************** Settings Real Time clock DS1307 ********************
  URTCLIB_WIRE.begin();
  rtc.set_rtc_address(0x68);
	rtc.set_model(rtcModel);  // set RTC Model
	rtc.refresh();  // refresh data from RTC HW in RTC class object so flags like rtc.lostPower(), rtc.getEOSCFlag(), etc, can get populated
  // Only use once, then disable
	// RTCLib::set(byte second, byte minute, byte hour (0-23:24-hour mode only), byte dayOfWeek (Sun = 1, Sat = 7), byte dayOfMonth (1-12), byte month, byte year)
  //rtc.set(0, 9, 12, 2, 7, 1, 25);
  rtc.sqwgSetMode(URTCLIB_SQWG_1H); // Setting SQWG/INT output
  // currentDay = rtc.day();           // Set current day
  delay(10);

// ************************** Read stored data *********************************
  currentDay = rtc.ramRead(ADRDAY);     // Read currentDay in I2C AT24C32 memory (addr: 1 - 56)
  numOnPump = rtc.ramRead(ADRNUMON);     // Read numOnPump in I2C AT24C32 memory (addr: 1 - 56)
  countOnPump = (rtc.ramRead(ADRCOUNT+1) << 8) | rtc.ramRead(ADRCOUNT);  // Read countOnPump in I2C AT24C32 memory (addr: 1 - 56)
  countDayPump = (rtc.ramRead(ADRCOUNTDAY+1) << 8) | rtc.ramRead(ADRCOUNTDAY);    // Read countDayPump in I2C AT24C32 memory (addr: 1 - 56)

// ****************************** Settings SD-MMC card ***************************************
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");    // !!!!!!!!!!!!!!!
    return;
  }
  /*Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDayHourC) {
    Serial.println("SDayHourC");
  } else {
    Serial.println("UNKNOWN");
  }*/
  Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));
  // check for index.html file
  if (!SD_MMC.exists("/index.html")) {
      Serial.println("ERROR - Can't find index.html file!");        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      return;  // can't find index file
  }
  delay(10);

// **************************** Settings Wi-Fi ************************************
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);               //  Network.begin(mac, ip);  // initialize Network device
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");

  // Print connections parameters
  /*Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.println(WiFi.getHostname());
  Serial.print("ESP Mac Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMinuteask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());*/
   
  server.begin();                           // start to listen for clients

}

void loop(){
  // refresh and read data from RTC HW in RTC class object so flags like rtc.lostPower(), rtc.getEOSCFlag(), etc, can get populated
	rtc.refresh();
  year = rtc.year();
  month = rtc.month();
  day = rtc.day();
  hour = rtc.hour();
  minute = rtc.minute();
  dayHour = countDayPump/60/60;
  dayMinute = (countDayPump - dayHour * 60)/60;
  daySecond = countDayPump%60;
  tHour = countOnPump/60/60;
  tMinute = (countOnPump - tHour * 60)/60;
  tSecond = countOnPump%60;
  
//********************** Reciever part **************************************************88
  if(radio.available()){                       // Если в буфере имеются принятые данные, то ...
    radio.read(&data_rx, sizeof(data_rx));     // Читаем данные из буфера в массив myData указывая сколько всего байт может поместиться в массив.  
    
    //RX BYTE 0
    voltageOn = (data_rx[0] >> 0) & 0x01;       // Bit 0. Remote signal that voltage is on to Pump
    commandOnPump = (data_rx[0] >> 1) & 0x01;   // Bit 1. Remote Command to On Pump 
    commandOffPump = (data_rx[0] >> 2) & 0x01;  // Bit 2. Remote Command to Off Pump
    commandBlock = (data_rx[0] >> 3) & 0x01;    // Bit 3. Remote Command to Block work Pump
    
    //RX BYTE 1
    alarmRCWPS = (data_rx[1] >> 0) & 0x01;      // Bit 0. State Alarm
    stateOnPump = (data_rx[1] >> 1) & 0x01;     // Bit 1. State On Relay to ON Pump 
    stateBlock = (data_rx[1] >> 2) & 0x01;      // Bit 2. State On Relay to Block work Pump
    
    //RX BYTES 2 - 3. Motors current
    uint16_t current_raw = (data_rx[2] << 8) | data_rx[3]; // Объединяем два байта
    currentRMS = current_raw / 100.0;              // Преобразуем обратно в float
    
    //RX BYTES 4 - 5. Reserved for 15Ai - Waters pressure

    //RX BYTES 6 - 8. Temperature and humidity
    uint16_t temperature_raw = (data_rx[6] << 8) | data_rx[7]; // Объединяем два байта
    temperature = temperature_raw / 10.0;              // Преобразуем обратно в float
    humidity = data_rx[8];
    
    //RX BYTE 9. Array of errors codes from RCWPS
    for (int i = 0; i < 8; i++) {
      codErrRCWPS[i] = data_rx[9] & (1 << i); // Проверить бит на позиции i и сохранить в массив
    }

    //RX BYTE 10. Rest time to off pump
    timeToOff = static_cast<int>(timeCycle * data_rx[10]);
  }

//*********************** Control part - LED *********************************
  // Control LED pump state. Green
  if(voltageOn){
    digitalWrite(ledOnPumpPin, HIGH);
  } else {
    digitalWrite(ledOnPumpPin, LOW);
  }
  // Control LED pump block state. Red
  if(stateBlock){
    digitalWrite(ledBlockPin, HIGH);
  } else {
    digitalWrite(ledBlockPin, LOW);
  }
  // Control LED alarm state. Yellow
  if(stateAlarm){
    digitalWrite(ledAlarmPin, HIGH);
  } else {
    digitalWrite(ledAlarmPin, LOW);
  }

//******************************** LCD ****************************************
  // Backlight LCD after MENU button pressing
  //Serial.println(touchouRead(buttonMenu));
  if(touchRead(buttonMenu) <= 25) {tMenu = true;} else {tMenu = false;}
  //if(!digitalRead(buttonMenu) == true && !backLight && !flagLight){
  if(tMenu && !backLight && !flagLight){
    lcd.setBacklight(1);                  //  Включаем подсветку LCD дисплея
    backLight = true;
    timer_start(TIMER_GROUP, TIMER_IDX);  // Запустить таймер
  }
  if (!tMenu && !backLight && flagLight) {
    lcd.setBacklight(0);                  //  Выключаем подсветку LCD дисплея
    flagLight = false;
  }
  
  // switch to around LCD Screens after MENU button pressing
  //if(!digitalRead(buttonMenu) && backLight && prevMenu){    // ver for button Menu
  if(tMenu && backLight && prevMenu){                         // ver for touch Menu
    numScreen++;
    if(numScreen > lcdScreens){
      numScreen = 1;
    }
  }
  //prevMenu = digitalRead(buttonMenu);
  prevMenu = tMenu;

//********************** Alarms ***********************************
  stateAlarm = alarmMCC || alarmRCWPS;
  // Check max time working pump
  if(countOnPump > maxOnPump){
    codErrMCC[0] = true;
  } else {
    codErrMCC[0] = false;
  }
  // Check max day time working pump
  if(countDayPump > maxDayPump){
    codErrMCC[1] = true;
  } else {
    codErrMCC[1] = false;
  }
  
  // Calculate average current
  avgCurrent = calcAverage(currentRMS);
  // Check max working rms current
  if(avgCurrent > maxCurrent){
    codErrMCC[2] = true;
  } else {
    codErrMCC[2] = false;
  }
  // Check min working rms curren t
  if(voltageOn && avgCurrent < minCurrent){
    codErrMCC[3] = true;
  } else {
    codErrMCC[3] = false;
  }
  // Check max working temperatures
  if(temperature > maxTemp){
    codErrMCC[4] = true;
  } else {
    codErrMCC[4] = false;
  }
  /*// Test errors messages
  for(uint8_t i=0; i<8; i++){
    codErrMCC[i] = true;
  }*/
  
  // Set flag alarmMCC if in array codErrMCC error
  alarmMCC = false;
  for (uint8_t i = 0; i < 8; i++) {
    alarmMCC = alarmMCC || codErrMCC[i];  // Выполняем логическое ИЛИ
    if (alarmMCC) break;                  // Если результат уже true, можно прервать цикл
  }

// ************* Counters working time ****************************
  // ON interrupt from 1Hz signal
  if(voltageOn && !prevStateOnPump && !flagCount){
    attachInterrupt(digitalPinToInterrupt(rtcSQPin), counterOn, FALLING);
    flagCount = true;
    countOnPump = 0;
    numOnPump++;
    prevStateOnPump = true;
    rtc.ramWrite(ADRNUMON, numOnPump);   // Write numOnPump in I2C AT24C32 memory (addr: 1 - 56) 
  }
  // OFF interrupt from 1Hz signal
  if(!voltageOn && !stateOnPump && prevStateOnPump && flagCount){
    detachInterrupt(digitalPinToInterrupt(rtcSQPin));
    flagCount = false;
    prevStateOnPump = false;
    lowByte = countOnPump & 0xFF;           // Младший байт
    highByte = (countOnPump >> 8) & 0xFF;   // Старший байт
    rtc.ramWrite(ADRCOUNT, lowByte);        // Write countOnPump in I2C AT24C32 memory (addr: 1 - 56)
    rtc.ramWrite(ADRCOUNT+1, highByte);     // Write countOnPump in I2C AT24C32 memory (addr: 1 - 56)
    lowByte = countDayPump & 0xFF;          // Младший байт
    highByte = (countDayPump >> 8) & 0xFF;  // Старший байт
    rtc.ramWrite(ADRCOUNTDAY, lowByte);          // Write countDayPump in I2C AT24C32 memory (addr: 1 - 56)
    rtc.ramWrite(ADRCOUNTDAY+1, highByte);       // Write countDayPump in I2C AT24C32 memory (addr: 1 - 56)
  }
  // continue interrupt handling 1Hz counter
  if(flagPulse){
    if(currentDay == rtc.day()){
      countDayPump++;
    } else {
      countDayPump = 0;
      numOnPump = 0;
      currentDay = rtc.day();
      rtc.ramWrite(ADRDAY, currentDay);   // Reset numOnPump in I2C AT24C32 memory (addr: 1 - 56)
      rtc.ramWrite(ADRNUMON, 0);   // Reset numOnPump in I2C AT24C32 memory (addr: 1 - 56)
      //rtc.ramWrite(ADRCOUNT, 0);   // Reset countOnPump in I2C AT24C32 memory (addr: 1 - 56)
      rtc.ramWrite(ADRCOUNTDAY, 0);    // Reset countDayPump in I2C AT24C32 memory (addr: 1 - 56)
      rtc.ramWrite(ADRCOUNTDAY+1, 0);    // Reset countDayPump in I2C AT24C32 memory (addr: 1 - 56)
    }
    flagPulse = false;
  }

//******************** Serial print variables *******************
  toSerial();

//******************** LCD print ********************************
  toDisplay();

//******************** Web Server *******************************
  NetworkClient client = server.accept();       // try to get client

  if (client) {                                 // got client?
      boolean currentLineIsBlank = true;
      while (client.connected()) {
          if (client.available()) {             // client data available to read
              char c = client.read();           // read 1 byte (character) from client
              // limit the size of the stored received HTTP request
              // buffer first part of HTTP request in HTTP_req array (string)
              // leave last element in array as 0 to null terminate string (REQ_BUF_SZ - 1)
              if (req_index < (REQ_BUF_SZ - 1)) {
                  HTTP_req[req_index] = c;      // save HTTP request character
                  req_index++;
              }
              // last line of client request is blank and ends with \n
              // respond to client only after last line received
              if (c == '\n' && currentLineIsBlank) {
                  // send a standard http response header
                  client.println("HTTP/1.1 200 OK");
                  // remainder of header follows below, depending on if
                  // web page or XML page is requested
                  // Ajax request - send XML file
                  if (StrContains(HTTP_req, "ajax_inputs")) {
                      // send rest of HTTP header
                      client.println("Content-Type: text/xml");
                      client.println("Connection: keep-alive");
                      client.println();
                      webCommand();                      
                      XML_response(client);                       // send XML file containing input states
                  } else {                                        // web page request
                      // send rest of HTTP header
                      client.println("Content-Type: text/html");
                      client.println("Connection: keep-alive");
                      client.println();
                      // send web page
                      webFile = SD_MMC.open("/index.html");            // open web page file
                      if (webFile) {
                          while(webFile.available()) {
                              client.write(webFile.read());       // send web page to client
                          }
                          webFile.close();
                      }
                  }
                  // display received HTTP request on serial port
                  Serial.print(HTTP_req);
                  // reset buffer index and all buffer elements to 0
                  req_index = 0;
                  StrClear(HTTP_req, REQ_BUF_SZ);
                  break;
              }
              // every line of text received from the client ends with \r\n
              if (c == '\n') {
                  // last character on line of received text
                  // starting new line with next character read
                  currentLineIsBlank = true;
              } else if (c != '\r') {
                  // a text character was received from client
                  currentLineIsBlank = false;
              }
          } // end if (client.available())
      } // end while (client.connected())
      delay(1);      // give the web browser time to receive the data
      client.stop(); // close the connection
  } // end if (client)  

//******************* Control part - Buttons  ****************************
  // command ON Pump
  pumpON = !digitalRead(buttonOnPumpPin) || webOn;
  // command OFF Pump
  pumpOFF = !digitalRead(buttonOffPumpPin) || webOff;

  // Triger that switch by pressing BLOCK button
  if((!digitalRead(buttonBlockPin) == true && !digitalRead(buttonBlockPin) != prevBlock && trigerBlock == false) ||
    (webBlock == true && webBlock != prevBlock && trigerBlock == false)) {
    trigerBlock = true;
  } else if((!digitalRead(buttonBlockPin) == true && !digitalRead(buttonBlockPin) != prevBlock && trigerBlock == true) ||
    (webBlock == true && webBlock != prevBlock && trigerBlock == true)){
    trigerBlock = false;
  }
  prevBlock = !digitalRead(buttonBlockPin) || webBlock;

//******************** Transmitter part *************************
  // Prepare data
  //TX BYTE 0
  data_tx[0] =  (trigerBlock << 3) | (pumpOFF << 2) | (pumpON << 1) | alarmMCC; 
  //TX BYTE 1 
  //TX BYTE 2
  //TX BYTE 3
  //TX BYTE 4
  
  // Transmit data if not equal previous
  if(!dataAreEqual(data_tx, data_tx_previos, sizeof(data_tx))) {  
    radio.stopListening();                       // Выключаем приемник, завершаем прослушивание открытых труб.
    radio.write(&data_tx, sizeof(data_tx));      // Отправляем данные из массива myData указывая сколько байт массива мы хотим отправить.
    
    // SerialPrint tx data. Генерация строки битов
    Serial.println("      TX");
    bits = "";
    for (int i = 7; i >= 0; i--) {
        bits += String((data_tx[0] >> i) & 1);  // Сдвиг вправо и побитовая операция AND
    }
    Serial.println("      Inputs = " + bits);
    Serial.println();

    radio.startListening();                      // Включаем приемник
  }

  // Copy current station to previous station
  for(uint8_t i = 0; i < sizeof(data_tx); i++) {
    data_tx_previos[i] = data_tx[i];
  }
  delay(500);
} // end loop



// Resolve commands from web client
void webCommand(void){
  // Start
    if (StrContains(HTTP_req, "START")) {
      webOn = true;
    } else {
      webOn = false;
    }
  // Stop
    if (StrContains(HTTP_req, "STOP")) {
      webOff = true;
    } else {
      webOff = false;
    }
  // Block
    if (StrContains(HTTP_req, "BLOCK_WORK")) {
      webBlock = true;
    } else {
      webBlock = false;
    }
  // Set Time
    if (StrContains(HTTP_req, "second=")) {
      setTimeFromHttp(HTTP_req);
    }
} // end webCommand()

// send the XML file with analog values, switch status
void XML_response(NetworkClient cl){
    cl.println("<?xml version=\"1.0\" ?>");
    cl.println("<root>");
   
    cl.print("<data>");
    cl.print(addZero(String(day), 2) + "-" + addZero(String(month), 2) + "-20" + addZero(String(year), 2));
    cl.println("</data>"); 
  
    cl.print("<time>");
    cl.print(addZero(String(hour), 2) + ":" + addZero(String(minute), 2));
    cl.println("</time>");

    cl.print("<temperature>");
    cl.print(String(temperature));
    cl.println("</temperature>");

    cl.print("<humidity>");
    cl.print(String(humidity));
    cl.println("</humidity>");

    cl.print("<voltage>");
    cl.print(voltageOn == true ? "ON" : "OFF");
    cl.println("</voltage>");

    cl.print("<rmsCurrent>");
    cl.print(String(currentRMS));
    cl.println("</rmsCurrent>");

    cl.print("<pressure>");
    cl.print(String(pressure));
    cl.println("</pressure>");

    cl.print("<lastOnTime>");
    cl.print(addZero(String(tHour), 2) + "h " + addZero(String(tMinute), 2) + "m " + addZero(String(tSecond), 2) + "s");
    cl.println("</lastOnTime>");

    cl.print("<timePerDay>");
    cl.print(addZero(String(dayHour), 2) + "h " + addZero(String(dayMinute), 2) + "m " + addZero(String(daySecond), 2) + "s");
    cl.println("</timePerDay>");

    cl.print("<startsPerDay>");
    cl.print(String(numOnPump));
    cl.println("</startsPerDay>");
    
    cl.print("<timeToOff>");
    cl.print(addZero(String(timeToOff), 3) + "s");
    cl.println("</timeToOff>");

    cl.print("<block>");
    cl.print(String(stateBlock == true ? "ON" : "OFF"));
    cl.println("</block>");
    
    cl.print("<alarm>");
    cl.print(String(stateAlarm == true ? "ON" : "OFF"));
    cl.println("</alarm>");
    
    cl.print("<relay>");
    cl.print(String(stateOnPump == true ? "ON" : "OFF"));
    cl.println("</relay>");

    cl.println("</root>");
} // end XML_response()

// Function set time to DS1307 from client http request string  
void setTimeFromHttp(String http_req) {
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t dayOfWeek;
  uint8_t dayOfMonth;
  uint8_t month;
  uint8_t year;
  
  int paramStart = http_req.indexOf('?');
  if (paramStart == -1) {
    Serial.println("Ошибка: строка запроса не найдена!");
    return;
  }

  // Извлекаем параметры
  String params = http_req.substring(paramStart + 1);
  int paramEnd = params.indexOf(' ');
  if (paramEnd != -1) {
    params = params.substring(0, paramEnd);
  }

  // Разбиваем параметры по '&'
  while (params.length() > 0) {
    int eqIndex = params.indexOf('=');
    int ampIndex = params.indexOf('&');

    if (eqIndex != -1) {
      String key = params.substring(0, eqIndex);
      String value = (ampIndex != -1) ? params.substring(eqIndex + 1, ampIndex) : params.substring(eqIndex + 1);

      if (key == "second") second = value.toInt();
      else if (key == "minute") minute = value.toInt();
      else if (key == "hour") hour = value.toInt();
      else if (key == "dayOfWeek") dayOfWeek = value.toInt();
      else if (key == "dayOfMonth") dayOfMonth = value.toInt();
      else if (key == "month") month = value.toInt();
      else if (key == "year") year = value.toInt();

      if (ampIndex == -1) break; // Если больше нет параметров, выходим из цикла
      params = params.substring(ampIndex + 1); // Убираем обработанный параметр
    } else {
      break;
    }
  }
  // Выводим полученные значения
  Serial.println("Параметры времени:");
  Serial.print("Second: "); Serial.println(second);
  Serial.print("Minute: "); Serial.println(minute);
  Serial.print("Hour: "); Serial.println(hour);
  Serial.print("Day of Week: "); Serial.println(dayOfWeek);
  Serial.print("Day of Month: "); Serial.println(dayOfMonth);
  Serial.print("Month: "); Serial.println(month);
  Serial.print("Year: "); Serial.println(year);

  // Устанавливаем время на модуле DS1307
  // RTCLib::set(byte second, byte minute, byte hour (0-23:24-hour mode only), byte dayOfWeek (Sun = 1, Sat = 7), byte dayOfMonth (1-12), byte month, byte year)
  rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
  Serial.println("Время успешно установлено!");
  Serial.println();
} // end setTimeFromHttp()

// Функция дополнения текста нулями
String addZero(String value, uint8_t width) {
    String formattedValue = value;

    // Добавление нулей, если длина строки меньше требуемой ширины
    while (formattedValue.length() < width) {
        formattedValue = "0" + formattedValue;
    }

    // Ограничение длины строки (на случай ошибок)
    if (formattedValue.length() > width) {
        formattedValue = formattedValue.substring(0, width);
    }
    return formattedValue;
} // end addZero()

// sets every element of str to 0 (clears array)
void StrClear(char *str, char length)
{
    for (int i = 0; i < length; i++) {
        str[i] = 0;
    }
}

// searches for the string sfind in the string str
// returns 1 if string found
// returns 0 if string not found
char StrContains(const char *str, const char *sfind)
{
    char found = 0;
    char index = 0;
    char len;

    len = strlen(str);
    
    if (strlen(sfind) > len) {
        return 0;
    }
    while (index < len) {
        if (str[index] == sfind[found]) {
            found++;
            if (strlen(sfind) == found) {
                return 1;
            }
        }
        else {
            found = 0;
        }
        index++;
    }
    return 0;
} // end StrContains()

// function calculate average value from several (NUM_AVG)
float calcAverage(float current) {
  static float values[NUM_AVG] = {0};       // Буфер для хранения последних значений
  static int index = 0;                   // Текущий индекс в буфере
  static int count = 0;                   // Количество заполненных элементов (до 10)
  //int current = round(10 * val);          // convert to int

  // Записываем новое значение в буфер
  values[index] = current;
  index = (index + 1) % NUM_AVG; // Перемещаем индекс по кольцу

  // Увеличиваем счетчик заполненных элементов, но не больше NUM_VALUES
  if (count < NUM_AVG) {
    count++;
  }

  // Вычисляем среднее значение
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += values[i];
  }
  // static_cast<float>(sum) / count / 10;  // convert to float
  return sum / count;
} // end calcAverage()

// function out information on LCD display. use global variables
void toDisplay() {
    // Previous state of variables
    static bool pv;         // voltageOn
    static bool par;        // alarmRCWPS
    static bool pam;        // alarmMCC
    static bool psop;       // stateOnPump
    static bool psb;        // stateBlock
    static uint8_t pnd;     // number screen on display
    static uint8_t phour;     // Hour
    static uint8_t pminute;      // Minute
    static uint8_t pnop;    // numOnPump
    static uint8_t ph;      // humidity
    static uint16_t pcop;   // countOnPump
    static uint16_t pcdp;   // countDayPump
    static uint8_t ptto;    // timeToOff
    static float pi;        // currentRMS
    static float pt;        // temperatures

    static uint8_t iNmsg;   // number error message to screen
    static uint8_t iTshow;  // counter times to show error message

    char* errMsgRCWPS[8] = {                      // mesages of errors from MCC
      " RelayON noVolt",
      " BlockON VoltON",
      "",
      "",
      "",
      "",
      "",
      ""
    };
  
    char errMsgMCC[8][16];  // 8 сообщений, каждое длиной до 16 символов
    // Преобразуем сообщения в массивы char
    snprintf(errMsgMCC[0], sizeof(errMsgMCC[0]), "Last ON > %dayMinute", maxOnPump / 60);
    snprintf(errMsgMCC[1], sizeof(errMsgMCC[1]), "Day ON > %dayMinute", maxDayPump / 60);
    snprintf(errMsgMCC[2], sizeof(errMsgMCC[2]), "I=%.2fA > %.2f", avgCurrent, maxCurrent);
    snprintf(errMsgMCC[3], sizeof(errMsgMCC[3]), "I=%.2fA < %.2f", avgCurrent, minCurrent);
    snprintf(errMsgMCC[4], sizeof(errMsgMCC[4]), "T=%.2fC > %.2f", temperature, maxTemp);
    strcpy(errMsgMCC[5], "");
    strcpy(errMsgMCC[6], "");
    strcpy(errMsgMCC[7], "");
  
  // redraw display only if parameters change
  if(par != alarmRCWPS || pam != alarmMCC || pnd != numScreen ||
    phour != hour || pminute != minute || pnop != numOnPump || ph != humidity ||
    pcop != countOnPump || pcdp != countDayPump ||
    ptto != timeToOff || pi != currentRMS || pt != temperature) {
  
    if(numScreen == 1) {
        // 1st string - Clock   
        lcd.clear();                      //  Чистим дисплей
        lcd.setCursor(0, 0);              //  Устанавливаем курсор в позицию (0 столбец, 0 строка)
        lcdPrintZeros(String(hour), 0, 2, 0);   //  lcd.print(u == 1 ? "ON" : "OFF");               
        lcd.setCursor(2, 0);
        lcd.print(":");
        lcdPrintZeros(String(minute), 3, 2, 0);
        // 1st string - Timer Day
        lcd.setCursor(6, 0);
        lcd.print("D");        
        lcdPrintZeros(String(dayHour), 7, 2, 0);   // Hours from countOnDay
        lcd.setCursor(9, 0);         
        lcd.print("h");               
        lcdPrintZeros(String(dayMinute), 10, 2, 0);  // Minutes from countOnDay
        lcd.setCursor(12, 0);
        lcd.print("m");         
        lcdPrintZeros(String(daySecond), 13, 2, 0);  // Seconds from countOnDay
        lcd.setCursor(15, 0); 
        lcd.print("s");
        // 2nd string - Rest Time Pump is On
        lcd.setCursor(0, 1);
        lcd.print("R");           
        lcdPrintZeros(String(timeToOff), 1, 3, 1);  // Rest time pump is on
        lcd.setCursor(4, 1);
        lcd.print("s");
        // 2nd string - number of starts
        lcd.setCursor(6, 1);
        lcd.print("N");
        lcd.setCursor(7, 1);
        lcd.print(numOnPump);  
        // 2nd string - Time the last of pump start 
        lcdPrintZeros(String(tMinute), 10, 2, 1);  // Time the last of pump start
        lcd.setCursor(12, 1);
        lcd.print("m");
        lcdPrintZeros(String(tSecond), 13, 2, 1);
        lcd.setCursor(15, 1);
        lcd.print("s");
    }
    if(numScreen == 2) {
        // 1st string - Clock   
        lcd.clear();                      //  Чистим дисплей
        lcd.setCursor(0, 0);              //  Устанавливаем курсор в позицию (0 столбец, 0 строка)
        lcdPrintZeros(String(hour), 0, 2, 0);   //  lcd.print(u == 1 ? "ON" : "OFF");               
        lcd.setCursor(2, 0);
        lcd.print(":");
        lcdPrintZeros(String(minute), 3, 2, 0);
        // 1st string - Temperature      
        lcdPrintZeros(String(temperature), 6, 4, 0);  // Temperatures
        lcd.setCursor(10, 0);         
        lcd.print("C");                  
        lcdPrintZeros(String(humidity), 13, 2, 0); // Humidity
        lcd.setCursor(15, 0);
        lcd.print("%");      
        // 2nd string - RMS Current
        lcd.setCursor(0, 1);
        lcd.print("I");
        lcd.setCursor(1, 1);
        lcd.print(currentRMS);
        // 2nd string - number of starts
        lcd.setCursor(6, 1);
        lcd.print("N");
        lcd.setCursor(7, 1);
        lcd.print(numOnPump);         // Number of pump starts
        // 2nd string - Time the last of pump start
        lcdPrintZeros(String(tMinute), 10, 2, 1);  // Time the last of pump start
        lcd.setCursor(12, 1);
        lcd.print("m");
        lcdPrintZeros(String(tSecond), 13, 2, 1);
        lcd.setCursor(15, 1);
        lcd.print("s");
    }
  }  
    if(numScreen == 3) {
        // output error message RCWPS on 1st string, MCC on 2nd string every 4 loop
        if(iTshow % 4 == 0){  
          lcd.clear();                      //  Чистим дисплей
          lcd.setCursor(0, 0);              
          lcd.print(iNmsg);
          if(codErrRCWPS[iNmsg] == true){lcd.print(errMsgRCWPS[iNmsg]);} else {lcd.print(" RCWPS is OK");}
          lcd.setCursor(0, 1);            
          if(codErrMCC[iNmsg] == true){lcd.print(errMsgMCC[iNmsg]);} else {lcd.print(" MCC is OK");}
          iNmsg = (iNmsg + 1) % 5;            // counter number error message, to zero each 8   
        }
        iTshow++;                           // counter show times error message
    }
  
  pv = voltageOn;
  par = alarmRCWPS;
  pam = alarmMCC;
  psop = stateOnPump;
  psb = stateBlock;
  pnd = numScreen;    
  phour = hour;            
  pminute = minute;              
  pnop = numOnPump;
  ph = humidity;
  pcop = countOnPump;
  pcdp = countDayPump;
  ptto = timeToOff;
  pi = currentRMS;
  pt = temperature;
  
} // end toDisplay()

// Функция вывода текста с выравниванием по правому краю
void lcdRightPrint(String text, uint8_t colB, uint8_t colW, uint8_t row) {
    int padding = colW - text.length(); // Количество отступов
    if (padding < 0) padding = 0;    // Защита от отрицательных значений

    lcd.setCursor((colB + padding), row);     // Установка курсора на нужное место
    lcd.print(text);                 // Печать текста
}

// Функция вывода текста с дополнением нулями
void lcdPrintZeros(String value, uint8_t colB, uint8_t width, uint8_t row) {
    String formattedValue = value;

    // Добавление нулей, если длина строки меньше требуемой ширины
    while (formattedValue.length() < width) {
        formattedValue = "0" + formattedValue;
    }

    // Ограничение длины строки (на случай ошибок)
    if (formattedValue.length() > width) {
        formattedValue = formattedValue.substring(0, width);
    }
    lcd.setCursor(colB, row);
    lcd.print(formattedValue);
} // end lcdPrintZeros()


void toSerial(){
  // ***** SerialPrint rx data. Генерация строки битов
  Serial.println("RX");
  //RX BYTE 0. Buttons
  bits = "";
  for (int i = 7; i >= 0; i--) {
      bits += String((data_rx[0] >> i) & 1);  // Сдвиг вправо и побитовая операция AND
  }
  Serial.println("Inputs = " + bits);  
  //RX BYTE 1. States
  bits = "";
  for (int i = 7; i >= 0; i--) {
      bits += String((data_rx[1] >> i) & 1);  // Сдвиг вправо и побитовая операция AND
  }
  Serial.println("Outputs = " + bits);
  //RX BYTES 9. Errors
  bits = "";
  for (int i = 7; i >= 0; i--) {
      bits += String((data_rx[9] >> i) & 1);  // Сдвиг вправо и побитовая операция AND
  }
  Serial.println("ErrorsRCWPS = " + bits);
  bits = "";
  for (int i = 0; i < 8; ++i) {
        bits += (codErrMCC[i] ? '1' : '0');
    }
  Serial.println("ErrorsMCC = " + bits);
  //RX BYTES 2, 3. MS Current
  Serial.println("RMS Current: " + String(currentRMS) + "A");
  Serial.println("average Current: " + String(avgCurrent) + "A");
  //RX BYTES 4, 5. Pressure
  //RX BYTES 6, 7. Temperature
  Serial.println("Temperature: " + String(temperature) + "°C");
  //RX BYTES 8. Humidity
  Serial.println("Humidity: " + String(humidity) + "%");
  //RX BYTES 10. Time to Off
  Serial.println("Time to Off: " + String(timeToOff) + "s");
  Serial.println();
  //Date & Time
  Serial.print("RTC DateTime: ");
	Serial.print(rtc.day());
	Serial.print('/');
	Serial.print(rtc.month());
	Serial.print('/');
	Serial.print(rtc.year());
	Serial.print(' ');
	Serial.print(rtc.hour());
	Serial.print(':');
	Serial.print(rtc.minute());
	Serial.print(':');
	Serial.print(rtc.second());
  Serial.println();
  
  Serial.print("ON numbers: ");
  Serial.println(numOnPump);
  Serial.print("ON time: ");
  Serial.println(countOnPump);
  Serial.print("ON day time: ");
  Serial.print(countDayPump/60);
  Serial.print("m ");
  Serial.print(countDayPump%60);
  Serial.println("s");
  Serial.println();
} // end toSerial()

bool isNumber(String str) {
  if (str.length() == 0) return false;
  for (int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (!isDigit(c)) return false;
  }
  return true;
}

bool dataAreEqual(uint8_t arr1[], uint8_t arr2[], size_t size) {
  for (size_t i = 0; i < size; i++) {
    if (arr1[i] != arr2[i]) {
      return false;
    }
  }
  return true;
}
