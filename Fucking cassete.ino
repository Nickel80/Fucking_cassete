#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>


#define TIMEOUT_MOTOR 10000

//Пин управления направлением вращения мотора
const int motor_dir=3;
#define MOTOR_DIR_PD 3
//Пин управления скоростью мотора
const int motor_pwm=5;
#define MOTOR_PWM_PD 5

//Пин датчика открытия
const int open_sensor=9;
#define OPEN_SENSOR_PB 0
//Пин датчика закрытия
const int close_sensor=8;
#define CLOSE_SENSOR_PB 1

//Переменная для отметки времени
long time_stamp=0;

//Массив UID коссеты, предполагается первый байт - тип, остальные порядковая сквозная нумерация
const uint32_t UID[4]={0x01,0x02,0x03,0x07};

//Пин старшего бита номера слота
const int h_pin=2;
#define H_PIN_PD 2
//Пин второго бита номера слота
const int hl_pin=4;
#define HL_PIN_PD 4
//Пин третьего бита номера слота
const int lh_pin=6;
#define LH_PIN_PD 6
//Пин четвертого бита номера слота
const int ll_pin=7;
#define LL_PIN_PD 7

//Массив ответа на запрос
uint8_t answer_long[5]={};

//Функция включения мотора, (скорость, направление)
void motor_on(uint8_t mot_pow, uint8_t mot_dir){
    digitalWrite(motor_dir,mot_dir);
    analogWrite(motor_pwm,mot_pow);
}

//Функция резкой остановки мотора (направление, должно быть противоположно текущему)
void motor_stop(uint8_t mot_dir){
    digitalWrite(motor_dir,mot_dir);
    delay(1);
    analogWrite(motor_pwm,0);
}

int check_sensor(int sensor_num){
    int sensor_undreb=0;
    for (int i=0;i<3;i++){
        if (digitalRead(sensor_num)==0){
            sensor_undreb++;
        };
        delay(1);
    }
    if (sensor_undreb==3){
        return 0;
    }
    else {
        return 1;
    }

    }
//Функция открытия шторки
uint8_t open_curtain(void){
    long time_stamp=0;
    //делаем отметку времени
    time_stamp=millis();
    //Включаем мотор
    motor_on(180,1);
    //Пока не закончился таймаут обрабатываем датчик открытия
    while ((millis()-time_stamp)<TIMEOUT_MOTOR){
    //Если датчик открытия сработал
    if (check_sensor(open_sensor)==0){
        //задержка для антидребезга
        delay(1);
        //Останавливаем мотор
        motor_stop(0);
        //Возвращаем удачный код завершения
        return 1;
    }
    //Сброс WatchDog
    wdt_reset();
    }
    //Если таймаут закончился, останавливаем мотор и возвращаем 2
    motor_stop(0);
    return 2;
};

//Функция закрытия шторки
uint8_t close_curtain(void){
long time_stamp=0;
    //делаем отметку времени
    time_stamp=millis();
    //Включаем мотор
    motor_on(180,0);
    //Пока не закончился таймаут обрабатываем датчик закрытия
    while ((millis()-time_stamp)<TIMEOUT_MOTOR){
    //Если датчик закрытия сработал
    if (check_sensor(close_sensor)==0){
        //задержка для антидребезга
        delay(1);
        //Останавливаем мотор
        motor_stop(1);
        //Возвращаем удачный код завершения
        return 1;
    }
    //Сброс WatchDog
    wdt_reset();
    }
    //Если таймаут закончился, останавливаем мотор и возвращаем 2
    motor_stop(1);
    return 2;
};

//Обработчик получения запроса по i2c
void receiveEvent(int howMany)
{
  //wdt_enable(WDTO_2S);
  // переменная типа чар для чтения команды по I2C от мастера
  char c=' ';
  //пока есть данные для приема считываем и тут же обрабатываем
  while (Wire.available()) // loop through all but the last
  {
        c = Wire.read(); // receive byte as a character
//в зависимости от команды записываем ответ в answer_long[], если команду можно обработать находясь в прерывании,
//обрабатываем ее сразу, и записываем в answer_long[4] команду в вернем регистре (значит обработана), если
//команду в прерывании обрабатывать нежелательно записываем в anwer_long[4] команду в нижнем регистре для обработки в цикле

//init
  if (c=='i'){
    for(int i=0;i<4;i++){
    answer_long[i]=UID[i];
  }
    answer_long[4]='I';
  }
  //status
  if (c=='s'){
    answer_long[0]=digitalRead(open_sensor);
    answer_long[1]=digitalRead(close_sensor);
    answer_long[2]=0;
    answer_long[3]=0;
    answer_long[4]='S';
  }
  //open
  if(c=='o'){
    answer_long[0]=0;
    answer_long[1]=0;
    answer_long[2]=0;
    answer_long[3]=0;
    answer_long[4]='o';
  }
  //close
  if(c=='c'){
    answer_long[0]=0;
    answer_long[1]=0;
    answer_long[2]=0;
    answer_long[3]=0;
    answer_long[4]='c';
  }
  //test
  if(c=='t'){
    answer_long[0]=0;
    answer_long[1]=0;
    answer_long[2]=0;
    answer_long[3]=0;
    answer_long[4]='t';
  }
  }
  //wdt_reset();
}

//Обработчик запроса на чтение от мастера по I2C
void requestEvent(void)
{
  //  wdt_enable(WDTO_1S);
  //  for(int i=0;i<10;i++){
 // Wire.write(answer[i]);
   // }
   //Serial.println("Request");
  // for(int i=0;i<4;i++){

  //Просто отправляем текущще значение массива answer_long[]
   Wire.write(answer_long,5);
  // wdt_reset();
//}
}

//Функция получения адреса слота, в который установлена кассета
uint8_t take_slot_number(int h_p,int hl_p,int lh_p,int ll_p){
  uint8_t slot_number=0;
  //считываем состояние старшего бита
  //slot_number=digitalRead(h_p);
  //и смещаем влево
  //slot_number=slot_number<<1;
  //и т.д.
  slot_number+=digitalRead(hl_p);
  slot_number=slot_number<<1;
  slot_number+=digitalRead(lh_p);
  slot_number=slot_number<<1;
  slot_number+=digitalRead(ll_p);
  //Прибавляем к занчению 0b00001000, чтобы адрес оказался в допустимом для протокола I2C диапазоне
  slot_number=slot_number+0b00001000;
  return slot_number;
}

void setup() {
//Serial.begin(9600);
delay(1000);
//На всякия случай включаем WatchDog
wdt_enable(WDTO_2S);
  pinMode(motor_dir,OUTPUT);
  pinMode(motor_pwm,OUTPUT);
  pinMode(h_pin,INPUT_PULLUP);
  pinMode(hl_pin,INPUT_PULLUP);
  pinMode(lh_pin,INPUT_PULLUP);
  pinMode(ll_pin,INPUT_PULLUP);
  pinMode(open_sensor,INPUT_PULLUP);
  pinMode(close_sensor,INPUT_PULLUP);
  delay(500);
  //Инициализируем slave на адресе, полученном за счет считывания перемычек слота
  Wire.begin(take_slot_number(h_pin,hl_pin,lh_pin,ll_pin));
  //Устанавливаем обработчики запроса и ответа
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
//Сброс WatchDoga
wdt_reset();
//На всякий случай принудительно разрешаем прерывания
sei();
}

void loop() {
//На всякия случай включаем WatchDog
  wdt_enable(WDTO_8S);

//Отладочное действие по посылке answer_long в ком-порт
 // for (int i=0;i<5;i++){
  //Serial.print(answer_long[i]);
 // }
//  Serial.println(' ');

//Обработка в цикле команд, которые нежелательно обрабатывать в прерывании
//Обработка команды open
  if (answer_long[4]=='o'){
        //Записываем в answer_long[0] результат выполнения команд open
    answer_long[0]=open_curtain();
        //Записываем в answer_long[4] команду в верхнем регистре, что означает ее выполнение
    answer_long[4]='O';
  }
  //Далее все тоже самое
  if (answer_long[4]=='c'){
    answer_long[0]=close_curtain();
    answer_long[4]='C';
  }
  if (answer_long[4]=='t'){
    answer_long[0]=open_curtain();
    wdt_reset();
    answer_long[1]=close_curtain();
    answer_long[4]='T';
  }

//Сброс WatchDog
  wdt_reset();
}





