/*
RCWPS (Remote Controller for Water Pump Station). Arduino UNO
Скетч удаленного контроллера домашней системы дистанционного беспроводного контроля состояния
Насосно-Аккумуляторной станции воды с местной индикауией и управлением. 
Include:
1. MCC (Master Central Controller) 
2. RCWPS (Remote Controller for Water Pump Station)

Main components RCWPS:
RCWPS           Arduino UNO
Remote Control  nRF2401L
Temperature     DHT11
3xSensor buttons
2xRelay output
HV input
CT analog input

By Ochilnik, 2024
*/

#include "SPI.h"                          //  Подключаем библиотеки для работы с шиной SPI (nRF2401)
#include "nRF24L01.h"                     //  Подключаем библиотеки для работы с nRF2401
#include "RF24.h"                         //  Подключаем библиотеки для работы с nRF2401
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "EmonLib.h"                      // Include Emon Library. AC current measurement

#define DHTPIN 17                         // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11                  // DHT 11

RF24 radio(9, 10);                        // порты D9, D10: CE, CSN  For Arduino UNO
DHT_Unified dht(DHTPIN, DHTTYPE);
String bits = "";
EnergyMonitor emon1;                      // Create an instance

const uint64_t addr_tx = 0xAABBCCDD11LL;  // адрес коннектора на передачу
const uint64_t addr_rx = 0xAABBCCDD22LL;  // адрес коннектора на прием
const uint8_t num_ch = 119;               // номер канала

bool commandAlarm;                        // command from MCC - Alarm
bool commandOnPump;                       // command from MCC - Pump ON
bool commandOffPump;                      // command from MCC - Pump OFF
bool commandBlock;                        // command from MCC - Block Pump
bool stateAlarm, alarmMCC, alarmRCWPS;    // Current States signals from RCWPS
bool codErrRCWPS[8] = {0};                // Array of errors codes from RCWPS

uint8_t data_tx[15] = {0};                // Array data to transmit
uint8_t data_rx[5] = {0};                 // Array data to recieve
uint8_t data_tx_previos[15] = {0};        // Array data that transmit last time
uint8_t cycleTimer = 14;                  // Количество циклов таймера до нужного времени 60s / 4,19424s = 14,3
uint8_t cycle = cycleTimer;               // Текущее Количество циклов таймера 
//uint8_t codErrMCC;                        // code of errors from MCC

float currentCalibration = 27.0;
float currentRMS;

unsigned long measurement_timestamp = 0;
unsigned long interval = 4000ul;  // Интервал измерений DTH в миллисекундах

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



/* RX Control words structures
BYTE 0    0     1     2     3     4     5     6     7
  Pin           
  Name    alarm onM   offM  block
BYTE 1    0     1     2     3     4     5     6     7
  Pin     
  Name    


  TX Control words structures
BYTE 0    0     1     2     3     4     5     6     7
  Pin     2Di   4Di   5Di   3Di
  Name    UM    onM   offM  block
BYTE 1    0     1     2     3     4     5     6     7
  Pin     8Do   7Do   6Do
  Name    alarm onM   block
BYTE 2,3
  Pin     14Ai
  Name    int IM
BYTE 4,5
  Pin     15Ai
  Name    P 
BYTE 6,7
  Pin     17Ai
  Name    int temperature
BYTE 8
  Pin     17Ai
  Name    int relative_humidity
BYTE 9
  Pin     -
  Name    codErrRCWPS[8]                      // Array of errors codes from RCWPS  
BYTE 10
  Pin     -
  Name    uint8_t rest cycles Timer1  
*/

//***** PINOUT *****
uint8_t voltagePumpPin = 2; // Voltage is supplied to the motor pump 
uint8_t buttonOnPumpPin = 4; // button pump on is pressed
uint8_t buttonOffPumpPin = 5; // button pump off is pressed
uint8_t buttonBlockPin = 3; // triger blocking pump is on
uint8_t blockPumpPin = 6; // command blocking pump
uint8_t onPumpPin = 7; // command pump on
uint8_t alarmPin = 8; // signal attention
uint8_t currentPumpPin = 14; // current is consumed motor pump 


void setup(){
// ***** Settings INPUT/OUTPUT 
  pinMode(voltagePumpPin, INPUT); // Voltage is supplied to the pump motor
  pinMode(buttonOnPumpPin, INPUT); // button pump on is pressed
  pinMode(buttonOffPumpPin, INPUT); // button pump off is pressed
  pinMode(buttonBlockPin, INPUT); // triger blocking pump is on
  pinMode(blockPumpPin, OUTPUT); // command pump off
  digitalWrite(blockPumpPin, !LOW);
  pinMode(onPumpPin, OUTPUT); // command pump on
  digitalWrite(onPumpPin, !LOW);
  pinMode(alarmPin, OUTPUT); // signal attention

// ***** Settings Serial port
  Serial.begin(115200);

// ***** Settings Timer1
  noInterrupts();                       // отключаем все прерывания
  TCCR1A = 0;                           //  управляющие регистры таймера/счетчика
  TCCR1B = 0;
  TCNT1 = 0;                            // регистр для установки заранее загружаемого значения
  TIMSK1 |= (1 << TOIE1);               // enable timer overflow interrupt ISR
  interrupts();                         // разрешаем все прерывания

// ***** Settings Radio Interface
  Serial.println("Reciver/Transmitter ON");
  radio.begin(); // инициализация радиомодуля
  delay(2000);
  //radio.setPayloadSize(7);        // Установить статичный размер блока данных пользователя в байтах (0-32)
  radio.setDataRate(RF24_250KBPS);  // Установить скорость передачи данных RF24_1MBPS или RF24_2MBPS или RF24_250KBPS
  /*switch(radio.getDataRate()){                            //  Проверка установленной скорости radio.getDataRate()
    case RF24_1MBPS   : Serial.println("setDataRate  1 MBPS"); break; // Если полученное значение RF24_1MBPS
    case RF24_2MBPS   : Serial.println("setDataRate  2 MBPS"); break; // Если полученное значение RF24_2MBPS
    case RF24_250KBPS : Serial.println("setDataRate  250 KBPS"); break;
  }*/
  radio.setCRCLength(RF24_CRC_8);   // Установить размер CRC 8 bit или 16 bit
  radio.setChannel(num_ch);         // установка номера канала
  radio.setAutoAck(true);           // подтверждение приема
  radio.setPALevel(RF24_PA_MAX);    // уровень питания усилителя RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
 
  //-----Reciever part--
  //radio.enableDynamicPayloads();  //включить поддержку динамического размера полезной нагрузки на приемнике
  radio.openReadingPipe(1, addr_rx); // открыть коннектор на прием
  //closeReadingPipe();             // Закрыть трубу открытую ранее для прослушивания (приёма данных).
  //radio.enableAckPayload();       // разрешить отсылку данных в ответ на входящий сигнал
  radio.startListening();           // перевод модуль в режим работы приемника

  //-----Transmitter part--
  radio.setRetries(5, 15);         // Указать время ожидания = (число+1) * 250 мкс и количество попыток отправки данных от 0 до 15
  radio.openWritingPipe(addr_tx);   // открыть коннект на отправку
  //radio.stopListening();          // перевод модуля в режим работы передатчика
  //radio.powerUp();                // режим полной нагрузки
  //radio.powerDown();              // режим пониженного потребления

//***** Initialize DHT11 device.
  dht.begin();

//***** Initialize class device CT
  emon1.current(currentPumpPin, currentCalibration);             // Current: input pin, calibration.
}

void startPump(){
  digitalWrite(onPumpPin, !HIGH);        // Start Pump (inverse)
  startTimer1();                         // start Timer1 on 63s = cycleTimer(14) * 4,19424s
}

void stopPump(){
  digitalWrite(onPumpPin, !LOW);        // Stop Pump (inverse)
  cycle = cycleTimer;                   // Reset cycles
  TCNT1 = 0;                            // Reset Timer1
  stopTimer1();  
}

void blockPump(){
  stopPump();
  digitalWrite(blockPumpPin, !HIGH);    // Disconnect Pump from power
  //stopTimer1();
}

void unblockPump(){
  digitalWrite(blockPumpPin, !LOW);     // Connect Pump to power
  //startTimer1();
}

void startTimer1(){
  TCCR1B |= (1 << CS10)|(1 << CS12);    // Start timer 1 with prescaler 1024
}

void stopTimer1(){
  TCCR1B &= ~(1 << CS10 | 1 << CS11 | 1 << CS12);    // Stop Timer1 
}


ISR(TIMER1_OVF_vect) {  // процедура обработки прерывания переполнения счетчика
  if(cycle>0){
    //TCNT1 = value;        // preload timer
    cycle--;
  } else {
    //digitalWrite(ledPin, LOW);  // выключаем светодиод
    //cycle = cycleTimer;
    stopPump();
  }
}



void loop(){
  
//*********** Control part - Commands and Buttons 
  if(digitalRead(onPumpPin) && digitalRead(blockPumpPin) && (commandOnPump || digitalRead(buttonOnPumpPin))) {
    startPump();
  }
  if(!digitalRead(onPumpPin) && digitalRead(blockPumpPin) && (commandOffPump || digitalRead(buttonOffPumpPin))) {
    stopPump();
  }
  if(digitalRead(blockPumpPin) && (commandBlock || digitalRead(buttonBlockPin))) {
    blockPump();
  }
  if(!digitalRead(blockPumpPin) && (!commandBlock && !digitalRead(buttonBlockPin))) {
    unblockPump();
  }

//****** Alarms
  for (int i = 0; i < 8; i++) {                                    // reset array of alarms
  codErrRCWPS[i] = false;
  }
  if(!digitalRead(onPumpPin) && !digitalRead(voltagePumpPin)){     // If output onPump is ON (inverse) and no voltage
    codErrRCWPS[0] = true;                                         // set error code
  } else {
    codErrRCWPS[0] = false;                                        // reset error code
  }
  if(!digitalRead(blockPumpPin) && digitalRead(voltagePumpPin)){  // If output blockPump is ON (inverse) and voltage on
    codErrRCWPS[1] = true;                                         // set error code
  } else {
    codErrRCWPS[1] = false;                                        // set error code
  }
  // Control Alarm LED
  if(isError() || alarmMCC){
    digitalWrite(alarmPin, HIGH);  
  } else {
    digitalWrite(alarmPin, LOW);
  }

//****** Reciever part
  if(radio.available()){                       // Если в буфере имеются принятые данные, то ...
    radio.read(&data_rx, sizeof(data_rx));     // Читаем данные из буфера в массив myData указывая сколько всего байт может поместиться в массив.  
    //RX BYTE 0
    alarmMCC = (data_rx[0] >> 0) & 0x01; // Bit 0. Remote Command to Alarm
    commandOnPump = (data_rx[0] >> 1) & 0x01; // Bit 1. Remote Command to On Pump 
    commandOffPump = (data_rx[0] >> 2) & 0x01; // Bit 2. Remote Command to Off Pump
    commandBlock = (data_rx[0] >> 3) & 0x01; // Bit 3. Remote Command to Block work Pump
    //RX BYTE 1
    //
  }

//****** Transmitter part
  // Prepare data
  //TX BYTE 0
  data_tx[0] =  (digitalRead(buttonBlockPin) << 3) | (digitalRead(buttonOffPumpPin) << 2) | 
                (digitalRead(buttonOnPumpPin) << 1) | digitalRead(voltagePumpPin); 
                        
  //TX BYTE 1
  data_tx[1] =  (!digitalRead(blockPumpPin) << 2) | 
                (!digitalRead(onPumpPin) << 1) | alarmRCWPS;

  //TX BYTES 2 - 3. Motors current
  currentRMS = emon1.calcIrms(1480);  // Calculate Irms only
  int packI = (int)round(currentRMS * 100);
  // Storage Pump Current (2 байта)
  data_tx[2] = (packI >> 8) & 0xFF; // Старший байт температуры
  data_tx[3] = packI & 0xFF;        // Младший байт температуры

  //TX BYTES 4 - 5. Reserved for 15Ai - Waters pressure

  //TX BYTES 6 - 8 
  if (millis() - measurement_timestamp >= interval) {
    sensors_event_t event;
    // Get temperature event and print its value.    
    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature)) {
      int packTemp = (int)round(event.temperature * 10);
      // Сохраняем температуру (2 байта)
      data_tx[6] = (packTemp >> 8) & 0xFF; // Старший байт температуры
      data_tx[7] = packTemp & 0xFF;        // Младший байт температуры
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity)) {
      data_tx[8] = (int)round(event.relative_humidity);
      // Сохраняем влажность (1 байт)
      //data_tx[8] = (packHum >> 8) & 0xFF;  // Старший байт влажности
      //data_tx[9] = packHum & 0xFF;         // Младший байт влажности
    
    }
    measurement_timestamp = millis(); // Обновление метки времени    
  }

  //TX BYTE 9. Error code RCWPS
  data_tx[9] = 0;
  for (int i = 0; i < 8; i++) {
    if (codErrRCWPS[i]) {
      data_tx[9] |= (1 << i); // Установить бит на соответствующей позиции
    }
  }

  //TX BYTE 10. Rest time to OFF Pump
  data_tx[10] = cycle;



  // Transmit data if not equal previous
  if(!dataAreEqual(data_tx, data_tx_previos, sizeof(data_tx))) {  
    radio.stopListening();                       // Выключаем приемник, завершаем прослушивание открытых труб.
    radio.write(&data_tx, sizeof(data_tx));      // Отправляем данные из массива myData указывая сколько байт массива мы хотим отправить.    
    toSerialTX();
    radio.startListening();                      // Включаем приемник, начинаем прослушивать открытые коннекты.
  }
  // Print data to serial port
  Serial.println("RX");
  Serial.println("Count = " + String(data_rx[0]));
  Serial.println("Start = " + String(data_rx[1]));
  Serial.println("Stop = " + String(data_rx[2]));
  Serial.println();
  
  // Copy current station to previous station
  for (uint8_t i = 0; i < sizeof(data_tx); i++) {
    data_tx_previos[i] = data_tx[i];
  }
  delay(300);
}

void toSerialTX(){
  Serial.println("      TX");
    // Генерация строки битов
    bits = "";
    for (int i = 7; i >= 0; i--) {
        bits += String((data_tx[0] >> i) & 1);  // Сдвиг вправо и побитовая операция AND
    }
    Serial.println("      Inputs = " + bits);
    bits = "";
    for (int i = 7; i >= 0; i--) {
        bits += String((data_tx[1] >> i) & 1);  // Сдвиг вправо и побитовая операция AND
    }
    Serial.println("      Outputs = " + bits);
    bits = "";
    for (int i = 7; i >= 0; i--) {
        bits += String((data_tx[9] >> i) & 1);  // Сдвиг вправо и побитовая операция AND
    }
    Serial.println("      Errors = " + bits);
    Serial.print(F("      Current RMS: "));
    uint16_t current_raw = (data_tx[2] << 8) | data_tx[3]; // Объединяем два байта
    float showI = current_raw / 100.0;              // Преобразуем обратно в float
    Serial.print(showI);
    Serial.println(F("A"));
    Serial.print(F("      Temperature: "));
    uint16_t temperature_raw = (data_tx[6] << 8) | data_tx[7]; // Объединяем два байта
    float showT = temperature_raw / 10.0;              // Преобразуем обратно в float
    Serial.print(showT);
    Serial.println(F("°C"));
    Serial.print(F("      Humidity: "));
    //uint16_t humidity_raw = (data_tx[8] << 8) | data_tx[9];    // Объединяем два байта
    //float showH = humidity_raw / 100.0;                    // Преобразуем обратно в float
    Serial.print(String(data_tx[8]));
    Serial.println(F("%"));
    Serial.println("      Cycles = " + String(data_tx[10]));
}

bool isError() {
  for(uint8_t i=0; i < 8; i++){
    if(codErrRCWPS[i] == true){
      alarmRCWPS = true;
      return true;  
    }
  }
  alarmRCWPS = false;
  return false;
}

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

