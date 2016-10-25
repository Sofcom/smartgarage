// Original http://arduino.ru/forum/proekty/kontrol-vlazhnosti-podvala-arduino-pro-mini
// for Arduino Pro Mini 328

#pragma GCC optimize ("-Os")
//#pragma pack(push, 1)     // выравнивание по одному байту ????????

#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <leOS.h>      // Шедуллер задач
#include <dht.h>       // Дачик влажности и температуры
#include "Ucglib.h"    // ВНИМАНИЕ использовать библиотеку не познее 1.01 справка https://code.google.com/p/ucglib/wiki/
#include "rusFont.h"   // Русские шрифты
#include "nRF24L01.h"  // Беcпроводной модуль надо использовать библиотеку http://tmrh20.github.io/RF24
#include "RF24.h"      // Беcпроводной модуль используются не стандартные функции https://github.com/TMRh20

// - ОПЦИИ -------------------------------
//#define DEBUG                                   // Отладочную  информацию в ком порт посылает  
//#define DEMO                                    // Признак демонстрации - данные с датчиков генерятся рандом
#define BEEP                                      // Использовать пищалку
#define RADIO                                     // Признак использования радио модуля
#define VERSION "Ver. 0.66 11/09/15"              // Текущая версия
#define ID             0x21                       // уникально Идентификатор устройства - старшие 4 бита, вторые (младшие) 4 бита серийный номер устройства
#define LABEL "Подпол"  // "Подпол" "Подвал"      // Место установки блока - отоображается как заголовок колонки в таблице. Не более 6 символов а то не влезает

// Макросы для работы с портами  скорость и место
#define SetOutput(port,bit)       DDR ## port |= _BV(bit)
#define SetInput(port,bit)        DDR ## port &= ~_BV(bit)
#define SetBit(port,bit)          PORT ## port |= _BV(bit)
#define ClearBit(port,bit)        PORT ## port &= ~_BV(bit)
#define WritePort(port,bit,value) PORT ## port = (PORT ## port & ~_BV(bit)) | ((value & 1) << bit)
#define ReadPort(port,bit)        (PIN ## port >> bit) & 1
#define PullUp(port,bit)          { SetInput(port,bit); SetBit(port,bit); }
#define Release(port,bit)         { SetInput(port,bit); ClearBit(port,bit); }
// Мои макросы
#define MOTOR_BIT                 0            // бит мотора в packet.flags
#define HEAT_BIT                  1            // бит калорифера в packet.flags
#define ABS_H_BIT                 2            // бит калорифера в packet.flags
#define MODE_BIT                  5            // первый бит режима в packet.flags
//#define MASK_MODE_BITS            0xe0         // маска для выделения номера режима в packet.flags
#define SET_MODE_BITS             packet.flags |= (settingRAM.mode<<MODE_BIT)  // запись номера режима в packet.flags
#define FLAG_ABS_H_ON             packet.flags |= (1<<ABS_H_BIT)   // бит ABS_H_BIT установить в 1
#define FLAG_ABS_H_OFF            packet.flags &= ~(1<<ABS_H_BIT)  // бит ABS_H_BIT установить в 0
#define FLAG_ABS_H_CHECK          packet.flags & (1<<ABS_H_BIT)    // бит ABS_H_BIT проверить на 1

#define FLAG_MOTOR_ON             packet.flags |= (1<<MOTOR_BIT)   // бит мотора установить в 1
#define FLAG_MOTOR_OFF            packet.flags &= ~(1<<MOTOR_BIT)  // бит мотора установить в 0
#define FLAG_MOTOR_CHECK          packet.flags & (1<<MOTOR_BIT)    // бит мотора проверить на 1
#define MOTOR_ON                  { WritePort(C,0,HIGH); FLAG_MOTOR_ON;  }   // включить мотор
#define MOTOR_OFF                 { WritePort(C,0,LOW) ; FLAG_MOTOR_OFF; }   // выключить мотор

#define FLAG_HEAT_ON              packet.flags |= (1<<HEAT_BIT)   // бит калорифера установить в 1
#define FLAG_HEAT_OFF             packet.flags &= ~(1<<HEAT_BIT)  // бит калорифера установить в 0
#define FLAG_HEAT_CHECK           packet.flags & (1<<HEAT_BIT)    // бит калорифера проверить на 1
#define HEAT_ON                   { WritePort(C,2,HIGH); FLAG_HEAT_ON;  }   // включить калорифер
#define HEAT_OFF                  { WritePort(C,2,LOW); FLAG_HEAT_OFF; }   // выключить калорифер


// - КОНСТАНТЫ --------------------------------------
#define dH_OFF          10                      // Гистерезис абсолютной влажности в сотых грамма на куб
#define dT_OFF          10                      // Гистерезис температуры в сотых градуса
#define TEMP_LOW       150                      // Температура подвала критическая - может быть замораживание (в сотых градуса) - система выключается и включается нагреватель
// СИСТЕМАТИЧЕСКИЕ ОШИБКИ ДАТЧИКОВ для каждого ID свой. ОШИБКИ ДОБАВЛЯЮТСЯ!!
#if ID == 0x21
    #define TOUT_ERR      -40                  // Ошибка уличного датчика температуры в сотых долях градуса
    #define TIN_ERR       -40                  // Ошибка домового датчика температуры в сотых долях градуса
    #define HOUT_ERR      -30                  // Ошибка уличного датчика влажности в сотых долях %
    #define HIN_ERR       +30                  // Ошибка домового датчика влажности в сотых долях %
#elif ID == 0x22
    #define TOUT_ERR      -12                  // Ошибка уличного датчика температуры в сотых долях градуса
    #define TIN_ERR       -10                  // Ошибка домового датчика температуры в сотых долях градуса
    #define HOUT_ERR      -250                 // Ошибка уличного датчика влажности в сотых долях %
    #define HIN_ERR       +250                 // Ошибка домового датчика влажности в сотых долях %
#else 
    #define TOUT_ERR      0                    // Ошибка уличного датчика температуры в сотых долях градуса
    #define TIN_ERR       0                    // Ошибка домового датчика температуры в сотых долях градуса
    #define HOUT_ERR      0                    // Ошибка уличного датчика влажности в сотых долях %
    #define HIN_ERR       0                    // Ошибка домового датчика влажности в сотых долях %
#endif

// - ВРЕМЕНА ---------------------------------------
#ifdef DEMO                                     // Для демо все быстрее и случайным образом
    #define NUM_SAMPLES      2                  // Число усреднений измерений датчика
    #define TIME_SCAN_SENSOR 1000               // Время опроса датчиков мсек, для демки быстрее
    #define TIME_PRINT_CHART 4000               // Время вывода точки графика мсек, для демки быстрее
    #define TIME_HOUR        50000              // Число мсек в часе, для демки быстрее   
#else   
   #define NUM_SAMPLES      10                  // Число усреднений измерений датчика
   #define TIME_SCAN_SENSOR 3000                // Время опроса датчиков мсек
   #define TIME_PRINT_CHART 300000              // Время вывода точки графика мсек
   #define TIME_HOUR        3600000             // Число мсек в часе
#endif

#define LONG_KEY      2000                      // Длительное нажатие кнопки мсек, появляется Экран инфо
#define SHORT_KEY     100                       // Короткое нажатие кнопки более мсек
#define NRF24_CHANEL  100                       // Номер канала nrf24

//  НОГИ к которым прицеплена переферия (SPI используется для TFT и NRF24 - 11,12,13)
#define PIN_HEAT      16                         // Ножка куда повешен калорифер A2 (port C2)
#ifdef BEEP
    #define PIN_BEEP  15                         // Ножка куда повешена пищалка A1 (port C1)
#endif
#define PIN_RELAY     14                         // Ножка на которую повешено реле (SSR) вентилятора - аналоговый вход A0 через резистор 470 ом (port C0)
#define PIN_CS        10                         // TFT дисплей spi
#define PIN_CD        9                          // TFT дисплей spi
#define PIN_RESET     8                          // TFT дисплей spi
#define PIN_CE        7                          // nrf24 ce
#define PIN_CSN       6                          // nrf24 cs
#define PIN_DHT22a    5                          // Первый датчик DHT22   IN  ДОМ
#define PIN_DHT22b    4                          // Второй датчик DHT22   OUT УЛИЦА
#define PIN_KEY       3                          // Кнопка, повешена на прерывание, что бы ресурсов не тратить (port D3)
#define PIN_IRQ_NRF24 2                          // Ножка куда заведено прерывание от NRF24 (пока не используется)

// Настройки
#define NUM_SETTING    7                        // Число вариантов настроек 
#define BLOCK_OFF      0                        // Выключено (вариант настроек)
#define HOOD_ON        1                        // Режим вытяжки (вариант настроек)
#define COOLING        2                        // Режим охлаждение (вариант настроек)

// АЦП ----------------------------------------
const long ConstADC=1126400;                    // Калибровка встроенного АЦП (встроенный ИОН) по умолчанию 1126400 дальше измеряем питание и смотрим на дисплей    

Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/PIN_CD, /*cs=*/PIN_CS, /*reset=*/PIN_RESET); // Аппаратный SPI на дисплей ILI9341
leOS myOS;                                      // многозадачность
dht DHT;                                        // Датчики
bool infoScreen=false;                          // Признак отображениея иформационного экрана  1 - на экран ничего не выводится кроме информационного экрана
bool flagKey=false;                             // Флаг нажатия клавиши
bool pressKey=false;                            // Флаг необходимости обработки кнопки
unsigned long time_key=0;                       // Время нажатия копки
byte last_error=100;                            // Предыдущая ошибка чтения датчиков

 struct type_setting_eeprom                     // Структура для сохранения данных в eeprom
 {
     byte mode = 0;                             // Какой режим работы блока
     unsigned long hour_unit=0;                 // мото часы блок измеряется в интервалах вывода данных = NUM_SAMPLES*TIME_SCAN_SENSOR
     unsigned long hour_motor=0;                // мото часы мотора измеряется в интервалах вывода данных = NUM_SAMPLES*TIME_SCAN_SENSOR
     unsigned long hour_heat=0;                 // мото часы нагревателя измеряется в интервалах вывода данных = NUM_SAMPLES*TIME_SCAN_SENSOR
  }; 
volatile type_setting_eeprom settingRAM;        // Рабочая копия счетчиков в памяти
type_setting_eeprom settingEEPROM EEMEM;        // Копия счетчиков в eeprom - туда пишем 

// Пакет передаваемый, используется также для хранения результатов. 
 struct type_packet_NRF24   // Версия 2.4!! адаптация для stm32 Структура передаваемого пакета 32 байта - 32 максимум
    {
        byte id=ID;                             // Идентификатор типа устройства - старшие 4 бита, вторые (младшие) 4 бита серийный номер устройства
        byte DHT_error;                         // Ошибка разряды: 0-1 первый датчик (00-ок) 2-3 второй датчик (00-ок) 4 - радиоканал     
        int16_t   tOut=-500,tIn=500;            // Текущие температуры в сотых градуса !!! место экономим
        uint16_t  absHOut=123,absHIn=123;       // Абсолютные влажности в сотых грамма на м*3 !!! место экономим
        uint16_t  relHOut=123,relHIn=123;       // Относительные влажности сотых процента !!! место экономим
        uint8_t   flags=0x00;                   // байт флагов  
                                                // 0 бит - мотор включен/выключен 
                                                // 1 бит - нагреватель включен/выключен
                                                // 2 бит -[1 - dH_min задается в сотых грамма на м*3] [0 - dH_min задается в ДЕСЯТЫХ процента от absHIn]
                                                // 3 4 - пока пусто
                                                // 5-7 - номер настройки = settingRAM.mode до 8 настроек, надо передавать, что бы на приемнике восстановить
        uint8_t   dH_min;                       // Порог включения вентилятора по РАЗНИЦЕ абсолютной влажности в сотых грамма на м*3 или в ДЕСЯТЫХ % см flags:2
        uint8_t   T_min;                        // Порог выключения вентилятора по температуре в ДЕСЯТЫХ долях градуса, только положительные значения
        uint8_t   count=0;                      // циклический счетчик отправленных пакетов нужен что бы на приемнике проверять качество связи
        char note[14] = LABEL;                  // Примечание не более 13 байт + "0" байт Русские буквы в два раза меньше т.к. UTF-8
    } packet; 
    
struct type_sensors                             // структура для усреднения измерений
{
       int  num=0;                              // сколько отсчетов уже сложили не болле NUM_SAMPLES
       long  sum_tOut=0,sum_tIn=0;              // Сумма для усреднения Текущие температуры в сотых градуса !!! место экономим
       long  sum_relHOut=0,sum_relHIn=0;        // Сумма для усреднения Относительные влажности сотых процента !!! место экономим
       int  tOut=-5000,tIn=5000;                // Текущие температуры в сотых градуса !!! место экономим
       int  absHOut=55555,absHIn=55555;         // Абсолютные влажности в сотых грамма на м*3 !!! место экономим
       int  relHOut=55555,relHIn=55555;         // Относительные влажности сотых процента !!! место экономим
} sensors;
    
// Массивы для графиков
uint8_t tOutChart[120];
uint8_t tInChart[120];
uint8_t absHOutChart[120];
uint8_t absHInChart[120];
uint8_t posChart=0;       // Позиция в массиве графиков - начало вывода от 0 до 120-1
uint8_t TimeChart=0;      // Время до вывода очередной точки на график. 
bool ChartMotor=false;    // Признак работы мотора во время интервала графика если мотор был включен на любое время (даже одно измерение) то на графике фон меняется в этом месте
bool ChartHeat=false;     // Признак работы нагревателя во время интервала графика если нагреватель был включен на любое время (даже одно измерение) то на графике фон меняется в этом месте

#ifdef  RADIO    // Радио модуль NRF42l  
   RF24 radio(PIN_CE, PIN_CSN);  // определение управляющих ног
   bool send_packet_ok=false;    // признак удачной отправки последнего пакета 
   unsigned long err_ask=0;      // число не полученных ответов
#endif 
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// ПРОГРАММА
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
#ifdef  DEBUG  
   Serial.begin(9600); 
   Serial.println(F("DEBUG MODE")); 
   #ifdef  BEEP  
     Serial.println(F("BEEP ON")); 
   #else         
     Serial.println(F("BEEP OFF"));
   #endif
   #ifdef  RADIO 
     Serial.println(F("RADIO ON")); 
   #else         
     Serial.println(F("RADIO OFF"));
   #endif
#endif

reset_sum();

#ifdef  RADIO    // Радио модуль NRF42l  первичная настройка
   radio.begin();
   radio.setDataRate(RF24_1MBPS);         //  выбор скорости RF24_250KBPS RF24_1MBPS RF24_2MBPS
   radio.setPALevel(RF24_PA_MAX);           // выходная мощность передатчика
   radio.setChannel(NRF24_CHANEL);          //тут установка канала
   radio.setCRCLength(RF24_CRC_16);         // использовать контрольную сумму в 16 бит
   radio.setAutoAck(true);                  // выключить аппаратное потверждение
// radio.enableDynamicPayloads();           // разрешить Dynamic Payloads
// radio.enableAckPayload();                // разрешить AckPayload
  radio.setRetries(50,10);                  // Количество пауза и количество повторов 
  radio.openWritingPipe(0xF0F0F0F0E1LL);    // передатчик
  radio.openReadingPipe(1,0xF0F0F0F0D2LL);  // приемник
  radio.startListening();
#endif
  #ifdef BEEP
     SetOutput(C,1);                      //  Настройка ноги для динамика
     WritePort(C,1,LOW);
  #endif
  SetInput(D,3);                        //  Включена кнопка
  WritePort(D,3,HIGH);
  SetOutput(C,0);                       //  Подключить Реле
  WritePort(C,0,LOW);
  SetOutput(C,2);                       //  Подключить Калорифер
  WritePort(C,2,LOW);
  
  pinMode(PIN_DHT22a, OUTPUT);          //  Датчик DHT22 #1 
  digitalWrite(PIN_DHT22a, HIGH);  
  pinMode(PIN_DHT22b, OUTPUT);          //  Датчик DHT22 #2
  digitalWrite(PIN_DHT22b, HIGH);  
  
  reset_ili9341();                      // сброс дисплея
  readEeprom();                         // Прочитать настройки 
  byte i=ReadPort(D,3);
  if (i==0)                  // Если при включении нажата кнопка то стираем Eeprom
  {   settingRAM.mode=0;
      settingRAM.hour_unit=0;
      settingRAM.hour_motor=0;
      settingRAM.hour_heat=0;
      ucg.setColor(255, 255, 255);
      print_StrXY(10,50,F("Сброс настроек и счетчиков"));
      for(i=0;i<3;i++)
        {
         delay(1000);
         ucg.print(F(" ."));
        }
      writeEeprom();       // Запись в EEPROM  
      delay(1000);
      ucg.clearScreen();
  } 
//  settingRAM.mode=4; 
  wdt_enable(WDTO_8S); // Сторожевой таймер Для тестов не рекомендуется устанавливать значение менее 8 сек.
  // Запуск задач по таймеру
  myOS.begin();
  myOS.addTask(measurement,TIME_SCAN_SENSOR);    // Измерение 
  attachInterrupt(1, scanKey, CHANGE);           // КНОПКА Прерывания по обоим фронтам
  print_static();                                // распечатать таблицу
  Setting();                                     // Применить настройки
//  printChart();
  measurement();                                 // Считать данные
  #ifdef BEEP
     beep(100);
     delay(300);
     beep(500);
  #endif
resetKey();
}

void loop()
{
 wdt_reset();                        // Сбросить сторожевой таймер
// Обработка нажатия кнопки
if (pressKey==true)                 // Кнопка была нажата и отпущена
{
// myOS.pauseTask(measurement);                                 // Остановить задачи
 if (flagKey!=false)                                          // Обработчик кнопки
 if ((time_key>=LONG_KEY)&&(infoScreen!=true))                // Длительное нажатие кнопки и информационный экран не показан
   printInfo();                                               // Вывод информационного экрана
 else 
   if (time_key >= SHORT_KEY)                                  // Короткое нажатие кнопки все остальное игнорируется
     { 
        #ifdef BEEP
          beep(30);                                            // Звук нажатия на клавишу
        #endif
       if (infoScreen==true)  clearInfo();                    // если информационный экран показан то стереть его
       else {if (settingRAM.mode >= NUM_SETTING) settingRAM.mode=0; //  Кольцевой счетчик настроек
               else settingRAM.mode++;                      // В противном случае следующая настройка
               Setting(); }  
     }
  resetKey();                                                //  Сброс состояния кнопки
//  myOS.restartTask(measurement);                             // Запустить задачи
  }
}

void resetKey(void)  // Сброс состяния кнопки
{
  flagKey=false;                                             
// time_key=millis();                                                
  pressKey=false;
}  
void print_static()  // Печать статической картинки 
{  
  int i;
   cli();
  // Заголовок 
  ucg.setColor(0, 0, 180);  // 
  ucg.drawBox(0, 0, 320-1, 23);
  ucg.setColor(250, 250, 250);
  ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
  print_StrXY(2,19,F("ОСУШИТЕЛЬ ID: 0x"));
  ucg.print( hex(packet.id >> 4));
  ucg.print( hex(packet.id&0x0f));
   
  #ifdef DEMO
    ucg.print(F(" demo"));
  #endif
  
  // Таблица для данных
  ucg.setColor(0, 200, 0);
  ucg.drawHLine(0,25,320-1);
  ucg.drawHLine(0,25+23*1,320-1);
  ucg.drawHLine(0,25+23*2,320-1);
  ucg.drawHLine(0,25+23*3,320-1);
  ucg.drawHLine(0,25+23*4,320-1);
  ucg.drawVLine(200-4,25,24+23*3);
  ucg.drawVLine(260,25,24+23*3);

  // Заголовки таблиц
  ucg.setColor(255, 255, 0);
  // В зависимости от id разные надписи - привязка местоположения блока к ID
  
//  print_StrXY(180+30-9,25+0+18,F(LABEL));
  ucg.setPrintPos(180+30-9,25+0+18); 
  ucg.print(packet.note); 
  
  print_StrXY(250+20,25+0+18,F("Улица"));
  print_StrXY(0,25+23*1+18,F("Температура градусы C")); 
  print_StrXY(0,25+23*2+18,F("Относительная влаж. %")); 
  print_StrXY(0,25+23*3+18,F("Абсолют. влаж. г/м*3")); 

  // Графики
  ucg.setColor(210, 210, 210);
//  ucg.drawHLine(1,240-2,130);
//  ucg.drawVLine(1,135,105);
//  ucg.drawHLine(10+154,240-2,130);
//  ucg.drawVLine(10+154,135,105);
  print_StrXY(10,135+0,F("Температура")); 
  print_StrXY(20+154,135+0,F("Абс. влажность")); 
   
  // надписи на графиках
  print_StrXY(128,154,F("+20")); 
  print_StrXY(135,194,F("0")); 
  print_StrXY(128,233,F("-20"));
  
  print_StrXY(296,164,F("15"));
  print_StrXY(296,194,F("10"));
  print_StrXY(296,223,F("5"));
  
  // Горизонтальная шкала по часам
//  ucg.setColor(255, 255, 0);
  for(i=0;i<=120;i=i+12)
    {
      ucg.drawPixel(4+i,239);
      ucg.drawPixel(4+i,238);
      ucg.drawPixel(167+i,239);
      ucg.drawPixel(167+i,238);
     } 
   sei();
}

void print_status(void) // Печать панели статуса Значки на статус панели
{
  if (infoScreen==true) return;        // если отображен информационный экран то ничего не выводим  
   cli();
 // 1. печать ошибки чтения датчиков
   print_error_DHT();
 // 2. Признак включения мотора или нагревателя
  if (FLAG_MOTOR_CHECK)         ucg.setColor(0, 240, 0); 
  else if (FLAG_HEAT_CHECK)     ucg.setColor(240, 0, 0);
  else                          ucg.setColor(0, 40, 0);
  ucg.drawBox(290-32, 5, 14, 14);
  
  #ifdef  RADIO 
  // 3. Признак удачной передачи информации по радиоканалу - получение квитанции
      if (send_packet_ok==true)  ucg.setColor(0, 240, 0); 
      else                       ucg.setColor(0, 40, 0);
      ucg.setPrintDir(3);
      ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
      print_StrXY(290-40,20,F(">>")); 
      ucg.setPrintDir(0);
      ucg.setFontMode(UCG_FONT_MODE_SOLID);
  #endif
  sei();
}  

void print_error_DHT(void) // Печать ошибки чтения датчиков выводится при каждом чтении датчика
{
  if (infoScreen==true) return;        // если отображен информационный экран то ничего не выводим  
 // 1. печать ошибки чтения датчиков
  if (packet.DHT_error!=last_error)        // если статус ошибки поменялся то надо вывести если нет то не выводим - экономия время и нет мерцания
  {
      cli();
      last_error=packet.DHT_error; 
      ucg.setColor(0, 0, 180);         // Сначала стереть
      ucg.drawBox(290, 0, 26, 18);
      ucg.setPrintPos(280,18); 
      ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
      if (packet.DHT_error>0) 
        { 
        ucg.setColor(255, 100, 100); 
        print_StrXY(280,19,F("0x"));
        ucg.print( hex(packet.DHT_error >> 4));
        ucg.print( hex(packet.DHT_error & 0x0f)); 
        }
      else  { ucg.setColor(200, 240, 0);   ucg.print(F(" ok ")); } 
     sei(); 
   }   
} 
//  вывод на экран данных (то что меняется)
void print_data()
{ 
  // Статистика по моточасам, время ведется в интервалах вывода данных = NUM_SAMPLES*TIME_SCAN_SENSOR а потом пересчитывается в часы при выводе.
settingRAM.hour_unit++;
if (FLAG_MOTOR_CHECK) settingRAM.hour_motor++;  // если мотор включен
if (FLAG_HEAT_CHECK) settingRAM.hour_heat++;    // если нагреватель включен

 if (infoScreen==true) return;                  // если отображен информационный экран то ничего не выводим  
  cli();
 // Печать значений 
  ucg.setFontMode(UCG_FONT_MODE_SOLID);
  ucg.setColor(250, 0, 100);  // Цвет ДОМА
  print_floatXY(200+0,25+23*1+18,((float)packet.tIn)/100);
  print_floatXY(200+0,25+23*2+18,((float)packet.relHIn)/100);
  print_floatXY(200+0,25+23*3+18,((float)packet.absHIn)/100);
  ucg.setColor(0, 250, 100);  // Цвет УЛИЦЫ
  print_floatXY(260+4,25+23*1+18,((float)packet.tOut)/100);
  print_floatXY(260+6,25+23*2+18,((float)packet.relHOut)/100);
  print_floatXY(260+6,25+23*3+18,((float)packet.absHOut)/100);
  sei();
}  

// Печать графика на экране, добавляется одна точка и график сдвигается 
void printChart() 
{
byte i,x=0;
uint8_t tmp;

// Работаем через кольцевой буфер
// Добавить новую точку в кольцевой буфер
     // Температура в доме. диапазон -25 . . . +25 растягиваем на 100 точек
     if (packet.tIn<=-2500) tInChart[posChart]=0;           // Если температура меньше -25 то округляем до -25
     else  if (packet.tIn>=2500)  tInChart[posChart]=100-1;   // Если температура больше 25  то округляем до 25
      else tInChart[posChart]=(packet.tIn+2500)/50;    // внутри -25...+25 растягиваем в два раза
   
    if (ChartMotor==true) tInChart[posChart]|=0x80;     // Признак включения мотора- старший бит в 1 - цвет фона на графике меняется
     ChartMotor=false;

     // Температура на улице. диапазон -25 . . . +25 растягиваем на 100 точек
     if (packet.tOut<=-2500) tOutChart[posChart]=0;         // Если температура меньше -25 то округляем до -25
     else  if (packet.tOut>=2500)  tOutChart[posChart]=100-1; // Если температура больше 25  то округляем до 25
      else tOutChart[posChart]=(packet.tOut+2500)/50;  // внутри -25...+25 растягиваем в два раза

    if (ChartHeat==true) tOutChart[posChart]|=0x80;     // Признак включения нагревателя- старший бит в 1 - цвет фона на графике меняется
     ChartHeat=false;

     // Абсолютная влажность в доме диапазон от 0 до 20 грамм на кубометр, растягиваем на 100 точек
     if (packet.absHIn>=2000) absHInChart[posChart]=100-1;
     else absHInChart[posChart]=packet.absHIn/20;   // внутри 0...20 растягиваем в пять  раз в сотых % по этому делем не на 100 а на 20

     // Абсолютная влажность на улицу диапазон от 0 до 20 грамм на кубометр, растягиваем на 100 точек
     if (packet.absHOut>=2000) absHOutChart[posChart]=100-1;
     else absHOutChart[posChart]=packet.absHOut/20;   // внутри 0...20 растягиваем в пять раз,  в сотых % по этому делем не на 100 а на 20
     
  if (infoScreen==false)                 // если отображен информационный экран то ничего не выводим
   {
   cli();  
   for(i=0;i<120;i++)    // График слева на право
     { 
     // Вычислить координаты текущей точки x в кольцевом буфере. Изменяются от 0 до 120-1
     if (posChart<i) x=120+posChart-i; else x=posChart-i;
    
     // нарисовать фон в зависимости от статуса мотора
     if  (tOutChart[x]>=0x80) ucg.setColor(90, 60, 0);     // Нагреватель был ключен - бледно желтый
     else if (tInChart[x]>=0x80) ucg.setColor(0, 60, 90);  // Мотор был ключен - бледно синий
     else                     ucg.setColor(0, 0, 0);       // все выключено
      
     ucg.drawVLine(5+120-i,237-100,100); 
     ucg.drawVLine(5+120-i+162,237-100,100);  
     
     ucg.setColor(180, 180, 180);  
     if (i%5==0) // Пунктирные линии графика
     {
       ucg.drawPixel(5+120-i,236-10-1);
       ucg.drawPixel(5+120-i,236-50-1);
       ucg.drawPixel(5+120-i,236-90-1);
       
       ucg.drawPixel(5+120-i+162,236-25-1);
       ucg.drawPixel(5+120-i+162,236-50-1);
       ucg.drawPixel(5+120-i+162,236-75-1);  
     } 
     
     // Вывести новую точку
     tmp=tInChart[x]&0x7f;   // Отбросить старший разряд - признак включения мотора
     if ((tmp==0)||(tmp==100))   ucg.setColor(255, 255, 255); else ucg.setColor(255, 100, 100); 
     ucg.drawPixel(5+120-i,236-tmp);

     tmp=tOutChart[x]&0x7f;   // Отбросить старший разряд - признак включения калорифера   
     if ((tmp==0) || (tmp==100)) ucg.setColor(255, 255, 255); else ucg.setColor(100, 255, 100); 
     ucg.drawPixel(5+120-i,236-tmp);
     
     if (absHInChart[x]==100) ucg.setColor(255, 255, 255); else ucg.setColor(255, 100, 100); 
     ucg.drawPixel(5+120-i+162,236-absHInChart[x]);
 
     if (absHOutChart[x]==100) ucg.setColor(255, 255, 255); else ucg.setColor(100, 255, 100); 
     ucg.drawPixel(5+120-i+162,236-absHOutChart[x]);
       }
     sei(); 
   } 
 if (posChart<120-1) posChart++; else posChart=0;            // Изменили положение в буфере и Замкнули буфер
} 

// ---ПЕРЕДАЧА ДАННЫХ ЧЕРЕЗ РАДИОМОДУЛЬ -----------------------------
#ifdef  RADIO    // Радио модуль NRF42l
void send_packet()
{       radio.powerUp();
        radio.stopListening();     // Остановить приемник
        delay(2);
    //    radio.openWritingPipe(0xF0F0F0F0E1LL);    // передатчик
        cli();
          //  send_packet_ok = radio.write(&packet,sizeof(packet));
          radio.writeBlocking(&packet,sizeof(packet),200);  // Writes 1 payload to the buffers
      send_packet_ok=radio.txStandBy();
        sei(); 
         if (send_packet_ok!=true) err_ask++;            // квитанция не получена добавить счетчик
         #ifdef BEEP
           if (send_packet_ok==true) beep(30);           // Пакет передан успешно
         #endif
         #ifdef  DEBUG  
           if (send_packet_ok==true)  Serial.println(F("Packet sending ++++++++++"));
           else                       Serial.println(F("Packet NOT sending -----------"));
         #endif   
        radio.startListening();    // Включить приемник
        packet.count++;            // при переполнении сам сбросится
     //   if (packet.count>255) packet.count=0;
 }  
#endif 
// Чтение датчика возвращает код ошибки:
// DHTLIB_OK                   0
// DHTLIB_ERROR_CHECKSUM       1
// DHTLIB_ERROR_TIMEOUT        2
// DHTLIB_ERROR_CONNECT        3
// DHTLIB_ERROR_ACK_L          4
// DHTLIB_ERROR_ACK_H          5
byte readDHT(byte pin)
{
//delay(5);
cli();
  byte err=-1*DHT.read22(pin); // Чтение датчика
sei();  
return err; 
} 

// Измерение и обработка данных чтение датчиков -----------------------------------------
void measurement()
{ 
 myOS.pauseTask(measurement);        // Обязательно здесь, а то датчики плохо читаются мешает leos
 wdt_reset();                        // Сбросить сторожевой таймер
 
 packet.DHT_error=readDHT(PIN_DHT22a);   // ПЕРВЫЙ ДАТЧИК ДОМ  Новый пакет, сбросить все ошибки и прочитать первый датчик
 #ifdef  DEMO
   DHT.temperature=packet.tIn/100+random(-20,30)/10.0; 
   if (DHT.temperature>20) DHT.temperature=19;
   if (DHT.temperature<-10) DHT.temperature=-9;
   DHT.humidity=packet.relHIn/100+(float)random(-5,8);
   if (DHT.humidity>96) DHT.humidity=90;
   if (DHT.humidity<1) DHT.humidity=10;
   packet.DHT_error=0; // в Демо режиме
//   DHT.temperature=3.0;
//   DHT.humidity=21.0;
  #endif  
     sensors.tIn=(int)(DHT.temperature*100.0)+TIN_ERR;  // Запомнить результаты для суммирования с учетом ошибок
     sensors.relHIn=(int)(DHT.humidity*100.0)+HIN_ERR;  
     
    #ifdef  DEBUG  
       Serial.print(F("Sensor read samples:")); Serial.println(sensors.num); 
       Serial.print(F("IN T="));Serial.print(sensors.tIn);
       Serial.print(F(" H=")); Serial.print(sensors.relHIn); 
       Serial.print(F(" error=")); Serial.println(packet.DHT_error);
    #endif
 
 packet.DHT_error=packet.DHT_error+16*readDHT(PIN_DHT22b);// ВТОРОЙ ДАТЧИК УЛИЦА  ошибки в старшие четыре бита
 #ifdef  DEMO
   DHT.temperature=packet.tOut/100+random(-20,30)/10.0; 
   if (DHT.temperature>30) DHT.temperature=19;
   if (DHT.temperature<-30) DHT.temperature=-9;
   DHT.humidity=packet.relHOut/100+(float)random(-5,8);
   if (DHT.humidity>96) DHT.humidity=90;
   if (DHT.humidity<1)  DHT.humidity=10;
   packet.DHT_error=0;      // в Демо режиме
// DHT.temperature=-10.0;
// DHT.humidity=40.0;
 #endif  
     sensors.tOut=(int)(DHT.temperature*100.0)+TOUT_ERR;  // Запомнить результаты для суммирования с учетом ошибок
     sensors.relHOut=(int)(DHT.humidity*100.0)+HOUT_ERR;
 
    #ifdef  DEBUG  
       Serial.print(F("OUT T="));Serial.print(sensors.tOut);
       Serial.print(F(" H=")); Serial.print(sensors.relHOut); 
       Serial.print(F(" error=")); Serial.println(packet.DHT_error);
    #endif   
 
 print_error_DHT();    // Вывод ошибки чтения датчика при каждом чтении контроль за качеством связи с датчиками
 
 if (packet.DHT_error==0)// Если чтение без ошибок у ДВУХ датчиков  копим сумму для усреднения
  {
     sensors.sum_tIn=sensors.sum_tIn+sensors.tIn;
     sensors.sum_relHIn=sensors.sum_relHIn+sensors.relHIn;
     sensors.sum_tOut=sensors.sum_tOut+sensors.tOut;
     sensors.sum_relHOut=sensors.sum_relHOut+sensors.relHOut;
     sensors.num++;
   }
 
 // набрали в сумме нужное число отсчетов рассчитываем усреднение и выводим
 if (sensors.num>=NUM_SAMPLES)  // Пора усреднять и выводить значения 
 {
//         resetKey();          // сброс кнопки почему то часто выскакивает длительное нажатие
        // вычисление средних значений
         packet.tIn=sensors.sum_tIn/NUM_SAMPLES;
         packet.relHIn=sensors.sum_relHIn/NUM_SAMPLES;
         packet.tOut=sensors.sum_tOut/NUM_SAMPLES;
         packet.relHOut=sensors.sum_relHOut/NUM_SAMPLES;
         reset_sum();       // Сброс счетчиков и сумм
         
         // вычисление абсолютной влажности
         packet.absHIn=(int)(calculationAbsH((float)(packet.tIn/100.0),(float)(packet.relHIn/100.0))*100.0);
         packet.absHOut=(int)(calculationAbsH((float)(packet.tOut/100.0),(float)(packet.relHOut/100.0))*100.0);
         
     #ifdef  DEBUG  
       Serial.println(F("Average value >>>>>>>>>>"));
       Serial.print(F("IN T="));Serial.print(packet.tIn);Serial.print(F(" H=")); Serial.print(packet.relHIn); 
       Serial.print(F(" abs H=")); Serial.println(packet.absHIn);
       Serial.print(F("OUT T="));Serial.print(packet.tOut);Serial.print(F(" H=")); Serial.print(packet.relHOut); 
       Serial.print(F(" abs H=")); Serial.println(packet.absHOut);
     #endif   
                     
         #ifdef  RADIO     // Радио модуль NRF42l 
            send_packet();   // Послать данные
         #endif
         CheckON();                                // Проверка статуса вентилятора
         print_data();                             // вывод усредненных значений 
         print_status();                           // панель состояния
         if (FLAG_MOTOR_CHECK) ChartMotor=true;    // Признак того что надо показывать включение мотора на графике
         if (FLAG_HEAT_CHECK) ChartHeat=true;      // Признак того что надо показывать включение нагревателя на графике 
         
         TimeChart++;
         if ((long)((long)TimeChart*TIME_SCAN_SENSOR*NUM_SAMPLES)>=(long)TIME_PRINT_CHART) // проврека не пора ли выводить график
            { printChart(); TimeChart=0; // Сдвиг графика и вывод новой точки
              #ifdef  DEBUG  
                 Serial.println(F("Point add chart ++++++++++++++++++++"));
              #endif  
              #ifdef BEEP
 //               beep(50);
              #endif
             } 
        }
    myOS.restartTask(measurement);     // Пустить задачи
}

// Функция переводит относительную влажность в абсолютную 
// t-температура в градусах Цельсия h-относительная влажность в процентах
float calculationAbsH(float t, float h)
{
 float temp;
 temp=pow(2.718281828,(17.67*t)/(t+243.5));
 return (6.112*temp*h*2.1674)/(273.15+t);
}

// Сканирование клавиш ------------------------------------------
void scanKey()
{  
    byte key,key1; 
    cli(); 
    key=ReadPort(D,3);                                // Прочитать кнопку 0 - нажата 
    delay(30);                                        // Задержка для подавления дребезга контактов
    key1=ReadPort(D,3);                               // читаем еще раз кнопку
    if (key!=key1) {resetKey();return;}               // если значения не равны то ложное срабатывание выходим
          
    if ((key==0)&&(flagKey==false))                   // Если кнопка была нажата запомнить время и поставить флаг нажатия
    {
        flagKey=true;                                 // Кнопка нажата  ждем обратного фронта
        time_key=millis();                            // Время нажатия запомнили
     }
    if ((key==1)&&(flagKey==true))                    // Если кнопка была отжата 
    {
         time_key=millis()-time_key;                  // Рассчитать время нажатия
         pressKey=true;                               // Поставить признак обработки кнопки
    }
   sei();
 }

// Проверка статуса вытяжки, не пора ли переключится
void CheckON(void)
{
int tmp=0;
// 0.  Проверить замораживание подвала КАЛОРИФЕР 
if (packet.tIn<=TEMP_LOW) { MOTOR_OFF; HEAT_ON; return;}   // Контроль от промораживания подвала по идеи здесь надо включать калорифер
if ((FLAG_HEAT_CHECK)&&(packet.tIn>TEMP_LOW+dT_OFF)) HEAT_OFF;    // Выключить калорифер

// 1. Режимы не зависящие от влажности и температуры ВЫСШИЙ приоритет
if ((settingRAM.mode==BLOCK_OFF)&&(~FLAG_MOTOR_CHECK))  return;
if ((settingRAM.mode==BLOCK_OFF)&&(FLAG_MOTOR_CHECK))  { MOTOR_OFF ; return;}
if ((settingRAM.mode==HOOD_ON )&&(FLAG_MOTOR_CHECK))   return;
if ((settingRAM.mode==HOOD_ON )&&(~FLAG_MOTOR_CHECK))  { MOTOR_ON  ; return;}

// 2. Режим охлаждения (второй приоритет) температура внутри больше 10 градусов темература снаружи меньше на 2 градуса чем внутри, на влажность не смотрим
if (settingRAM.mode==COOLING)          // Режим охлаждение
  {
    if ((~FLAG_MOTOR_CHECK)&&(packet.tIn>(packet.T_min*10))&&((packet.tIn-packet.tOut)>packet.dH_min)) // dH_min используется не штатно для температуры
       {MOTOR_ON; return;}            // мотор выключен, температура выше установленной и снаружи температура ниже на 2 градуса  то ВКЛЮЧЕНИЕ мотора
    if ((FLAG_MOTOR_CHECK)&&(packet.tIn<=packet.tOut))   
       {MOTOR_OFF; return;}        // мотор включен и темература внутри ниже наружней то ВЫКЛЮЧЕННИЕ мотора
   return;                             // изменений нет выходим    
  } 
// 3. В режиме осушения - проверка на достижение минимальной температуры помещения в режиме осушения - СРОЧНО ВЫКЛЮЧИТЬ  третий приоритет
if (packet.tIn<=(packet.T_min*10))
   {
     if (~FLAG_MOTOR_CHECK)   return;      // Мотор уже выключен, выходим
     else  { MOTOR_OFF; return;}           // выключить и выйти
   } 
   
// 4. Режимы зависящие от температуры и влажности низший приоритет (что осталось)
// Расчитываем разность срабатывания по влажности
if (FLAG_ABS_H_CHECK)   tmp=packet.dH_min;                 // Если режимы используют абсолютную разность в сотых грамма на куб
else tmp=(int)(packet.absHIn*(packet.dH_min/10)/100);      // Если режимы используют ДЕСЯТЫЕ % от абсолютной разности внутренней температуры 

 if ((~FLAG_MOTOR_CHECK)&&(packet.tIn>(packet.T_min*10))&&((packet.absHIn-tmp)>packet.absHOut)) 
        {MOTOR_ON; return;}        // мотор выключен, темература выше критической, абс влажность с наружи меньше  то ВКЛЮЧЕНИЕ мотора
 if ((FLAG_MOTOR_CHECK)&&((packet.tIn<=(tmp+dT_OFF))||(packet.absHIn<(packet.absHOut+dH_OFF)))) 
        {MOTOR_OFF; return;}       // мотор включен и темература ниже критической или абс влажность внутри ниже  то ВЫКЛЮЧЕННИЕ мотора
} 
 

// Вывод информации о настройках и сохрание индекса настроек в eeprom 
// ---------------------------------
void Setting()
{
 // Настройка
  cli();
  ucg.setColor(0, 100, 255);
  ucg.setFontMode(UCG_FONT_MODE_SOLID);
  ucg.setPrintPos(0,25+0+18); 
  switch (settingRAM.mode)
        {
        case  BLOCK_OFF: ucg.print(F("Выключено              ")); packet.dH_min=255; packet.T_min=255; FLAG_ABS_H_ON;  break; 
        case  HOOD_ON:   ucg.print(F("Режим вытяжки         "));  packet.dH_min=0;   packet.T_min=0;   FLAG_ABS_H_ON;  break; 
        case  COOLING:   ucg.print(F("Охлаждение T>10 dT>2"));    packet.dH_min=200; packet.T_min=100; FLAG_ABS_H_ON;  break; // dH_min используется не штатно для температуры     
        case  3:         ucg.print(F("Осушение T>+3 dH>0.2 "));   packet.dH_min=20;  packet.T_min=30;  FLAG_ABS_H_ON;  break;
        case  4:         ucg.print(F("Осушение T>+3 dH>4% "));    packet.dH_min=40;  packet.T_min=30;  FLAG_ABS_H_OFF; break;
        case  5:         ucg.print(F("Осушение T>+4 dH>0.3 "));   packet.dH_min=30;  packet.T_min=40;  FLAG_ABS_H_ON;  break;
        case  6:         ucg.print(F("Осушение T>+4 dH>8%  "));   packet.dH_min=80;  packet.T_min=40;  FLAG_ABS_H_OFF; break;
        case  7:         ucg.print(F("Осушение T>+5 dH>0.7 "));   packet.dH_min=70;  packet.T_min=50;  FLAG_ABS_H_ON;  break;
        } 
 writeEeprom();       // Запись в EEPROM  новых настроек
 SET_MODE_BITS;       // в пакет для передачи добавили смену режима
 MOTOR_OFF;           // Поменяли настройки - отключить мотор, пусть заново настройки сработают если потребуется
 CheckON();           // Возможно надо включить мотор
 print_status();      // Показать панель состояния (смена настроек)
// resetKey();
 sei();   
}

// Вывод float  с двумя десятичными знаком в координаты x y // для экономии места
void print_floatXY(int x,int y, float v)
{
 ucg.setPrintPos(x,y);
 ucg.print(v,2);
 ucg.print(F("  ")); // Стереть хвост от предыдущего числа
} 

// Вывод строки константы в координаты x y // для экономии места
void print_StrXY(int x,int y, const __FlashStringHelper* b)
{
 ucg.setPrintPos(x,y);
 ucg.print(b);
} 

void printInfo() // Окно с информацией о блоке, появляется при длительном нажатии на кнопку
{
  infoScreen=true;
  cli();
  ucg.setColor(250, 250, 250);  // 
  ucg.drawBox(10, 10, 320-1-20, 240-1-20);
  ucg.setColor(0, 50, 250);
  ucg.drawFrame(10+5, 10+5, 320-1-20-10, 240-1-20-10);
  
  ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
  ucg.setColor(0, 150, 10);
  print_StrXY(35,18+15,F("ОСУШИТЕЛЬ на Arduino Pro Mini"));
  
  ucg.setColor(0, 50, 50);
  print_StrXY(10+10,15+17*2,F("1 Напряжение питания В.")); 
  print_floatXY(10+230,15+17*2,readVcc()/1000.0);
 
  print_StrXY(10+10,15+17*3,F("2 Температура блока гр.")); 
  print_floatXY(10+230,15+17*3,GetTemp());
 
  print_StrXY(10+10,15+17*4,F("3 Свободная память байт")); 
  ucg.setPrintPos(10+230,15+17*4); 
  ucg.print(freeRam()); 
 
  print_StrXY(10+10,15+17*5,F("4 Мото часы блока")); 
  ucg.setPrintPos(10+230,15+17*5); 
  ucg.print(settingRAM.hour_unit/(TIME_HOUR/(NUM_SAMPLES*TIME_SCAN_SENSOR))); 
  
  print_StrXY(10+10,15+17*6,F("5 Мото часы вентилятора")); 
  ucg.setPrintPos(10+230,15+17*6); 
  ucg.print(settingRAM.hour_motor/(TIME_HOUR/(NUM_SAMPLES*TIME_SCAN_SENSOR))); 
  
  print_StrXY(10+10,15+17*7,F("6 Мото часы нагревателя")); 
  ucg.setPrintPos(10+230,15+17*7); 
  ucg.print(settingRAM.hour_heat/(TIME_HOUR/(NUM_SAMPLES*TIME_SCAN_SENSOR)));  
 
  print_StrXY(10+10,15+17*8,F("7 Канал NRF24l01+")); 
  ucg.setPrintPos(10+230,15+17*8); 
  ucg.print(NRF24_CHANEL); 
  
  print_StrXY(10+10,15+17*9,F("8 Гистерезис T/absH   ")); 
  ucg.print((float)dT_OFF/100.0);
  ucg.print(F("/"));
  ucg.print((float)dH_OFF/100.0);
  
  print_StrXY(10+10,15+17*10,F("9 ERR Т/Н in:")); 
  ucg.print(TIN_ERR); 
  ucg.print(F("/")); 
  ucg.print(HIN_ERR); 
  ucg.print(F(" out:"));
  ucg.print(TOUT_ERR); 
  ucg.print(F("/")); 
  ucg.print(HOUT_ERR); 
 
  ucg.setColor(0, 0, 150);
  print_StrXY(10+10,16+17*11,F("СБРОС - Вкл. при нажатой кнопке.")); 
  
  ucg.setColor(250,80,80);
  print_StrXY(10+10,20+21+18*10,F(VERSION)); 
  #ifdef  RADIO     // Радио модуль NRF42l вывести число ошибок ask
     ucg.setColor(0, 150, 10);
     ucg.print(F(" Ask err:"));
     ucg.print(err_ask);
  #endif
  sei();
  
  #ifdef BEEP
   beep(40);
  #endif
//  resetKey();
} 

void clearInfo()  // Стереть информационный экран
{
      infoScreen=false;
//      resetKey();
      last_error=100;         // Признак обновления ошибки
      cli();
      ucg.setColor(0, 0, 0);  // залить черным
      ucg.drawBox(10, 10, 320-1-20, 240-1-20);
      print_static();
      Setting();  
      print_data();                            
      printChart();
//      resetKey();
      sei();
} 
// Чтение свободной памяти --------------------------------------------------------------------
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
// Чтение внутреннего датчика температуры ---------------------------------------
double GetTemp(void)
{
  unsigned int wADC;
  double t;
  sei();  // Должны быть разрешены прерывания
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  
  delay(20);           
  ADCSRA |= _BV(ADSC);  
  while (bit_is_set(ADCSRA,ADSC));
  wADC = ADCW;
  t = (wADC - 324.31 ) / 1.22;
  return (t); 
}
// Чтение напряжения питания ----------------------------------------------
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = ConstADC / result; // Back-calculate AVcc in mV
  return result;
} 
// Запись счетчиков в Eeprom --------------------------------------------------
void writeEeprom()
{ 
cli(); 
  eeprom_write_block((const void*)&settingRAM, (void*) &settingEEPROM, sizeof(settingRAM)); 
sei();
}
// Чтение счетчиков из Eeprom --------------------------------------------------
void readEeprom()
{
cli(); 
   eeprom_read_block((void*)&settingRAM, (const void*) &settingEEPROM, sizeof(settingRAM)); 
   if ((settingRAM.mode>(NUM_SETTING-1))||(settingRAM.mode<0)) settingRAM.mode=0;        // гарантированно попадаем в диапазон
sei();
}

void reset_sum()  // Сброс счетчиков накоплений
{
sensors.num=0;  // Рассчитать величину усреднения
sensors.sum_tOut=0;
sensors.sum_tIn=0;
sensors.sum_relHOut=0;
sensors.sum_relHIn=0;
}

char hex(byte x)  // Функция для вывода в hex
{
   if(x >= 0 && x <= 9 ) return (char)(x + '0');
   else      return (char)('a'+x-10);
}

bool reset_ili9341(void)
{
  pinMode(PIN_RESET, OUTPUT);                    // Сброс дисплея сигнал активным является LOW
  digitalWrite(PIN_RESET, LOW);  
  delay(100);
  digitalWrite(PIN_RESET, HIGH);  
  // Дисплей
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.setFont(my14x10rus);  
//   ucg.setRotate90();
  ucg.setRotate270();
  ucg.clearScreen();
}

#ifdef BEEP  
  void beep(int x)  // Пищать х мсек
  {
    WritePort(C,1,HIGH);
    delay(x);
    WritePort(C,1,LOW);
  } 
#endif
