#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>


#define TIMEOUT_MOTOR 10000

//��� ���������� ������������ �������� ������
const int motor_dir=3;
#define MOTOR_DIR_PD 3
//��� ���������� ��������� ������
const int motor_pwm=5;
#define MOTOR_PWM_PD 5

//��� ������� ��������
const int open_sensor=9;
#define OPEN_SENSOR_PB 0
//��� ������� ��������
const int close_sensor=8;
#define CLOSE_SENSOR_PB 1

//���������� ��� ������� �������
long time_stamp=0;

//������ UID �������, �������������� ������ ���� - ���, ��������� ���������� �������� ���������
const uint32_t UID[4]={0x01,0x02,0x03,0x07};

//��� �������� ���� ������ �����
const int h_pin=2;
#define H_PIN_PD 2
//��� ������� ���� ������ �����
const int hl_pin=4;
#define HL_PIN_PD 4
//��� �������� ���� ������ �����
const int lh_pin=6;
#define LH_PIN_PD 6
//��� ���������� ���� ������ �����
const int ll_pin=7;
#define LL_PIN_PD 7

//������ ������ �� ������
uint8_t answer_long[5]={};

//������� ��������� ������, (��������, �����������)
void motor_on(uint8_t mot_pow, uint8_t mot_dir){
    digitalWrite(motor_dir,mot_dir);
    analogWrite(motor_pwm,mot_pow);
}

//������� ������ ��������� ������ (�����������, ������ ���� �������������� ��������)
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
//������� �������� ������
uint8_t open_curtain(void){
    long time_stamp=0;
    //������ ������� �������
    time_stamp=millis();
    //�������� �����
    motor_on(180,1);
    //���� �� ���������� ������� ������������ ������ ��������
    while ((millis()-time_stamp)<TIMEOUT_MOTOR){
    //���� ������ �������� ��������
    if (check_sensor(open_sensor)==0){
        //�������� ��� ������������
        delay(1);
        //������������� �����
        motor_stop(0);
        //���������� ������� ��� ����������
        return 1;
    }
    //����� WatchDog
    wdt_reset();
    }
    //���� ������� ����������, ������������� ����� � ���������� 2
    motor_stop(0);
    return 2;
};

//������� �������� ������
uint8_t close_curtain(void){
long time_stamp=0;
    //������ ������� �������
    time_stamp=millis();
    //�������� �����
    motor_on(180,0);
    //���� �� ���������� ������� ������������ ������ ��������
    while ((millis()-time_stamp)<TIMEOUT_MOTOR){
    //���� ������ �������� ��������
    if (check_sensor(close_sensor)==0){
        //�������� ��� ������������
        delay(1);
        //������������� �����
        motor_stop(1);
        //���������� ������� ��� ����������
        return 1;
    }
    //����� WatchDog
    wdt_reset();
    }
    //���� ������� ����������, ������������� ����� � ���������� 2
    motor_stop(1);
    return 2;
};

//���������� ��������� ������� �� i2c
void receiveEvent(int howMany)
{
  //wdt_enable(WDTO_2S);
  // ���������� ���� ��� ��� ������ ������� �� I2C �� �������
  char c=' ';
  //���� ���� ������ ��� ������ ��������� � ��� �� ������������
  while (Wire.available()) // loop through all but the last
  {
        c = Wire.read(); // receive byte as a character
//� ����������� �� ������� ���������� ����� � answer_long[], ���� ������� ����� ���������� �������� � ����������,
//������������ �� �����, � ���������� � answer_long[4] ������� � ������ �������� (������ ����������), ����
//������� � ���������� ������������ ������������ ���������� � anwer_long[4] ������� � ������ �������� ��� ��������� � �����

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

//���������� ������� �� ������ �� ������� �� I2C
void requestEvent(void)
{
  //  wdt_enable(WDTO_1S);
  //  for(int i=0;i<10;i++){
 // Wire.write(answer[i]);
   // }
   //Serial.println("Request");
  // for(int i=0;i<4;i++){

  //������ ���������� ������� �������� ������� answer_long[]
   Wire.write(answer_long,5);
  // wdt_reset();
//}
}

//������� ��������� ������ �����, � ������� ����������� �������
uint8_t take_slot_number(int h_p,int hl_p,int lh_p,int ll_p){
  uint8_t slot_number=0;
  //��������� ��������� �������� ����
  //slot_number=digitalRead(h_p);
  //� ������� �����
  //slot_number=slot_number<<1;
  //� �.�.
  slot_number+=digitalRead(hl_p);
  slot_number=slot_number<<1;
  slot_number+=digitalRead(lh_p);
  slot_number=slot_number<<1;
  slot_number+=digitalRead(ll_p);
  //���������� � �������� 0b00001000, ����� ����� �������� � ���������� ��� ��������� I2C ���������
  slot_number=slot_number+0b00001000;
  return slot_number;
}

void setup() {
//Serial.begin(9600);
delay(1000);
//�� ������ ������ �������� WatchDog
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
  //�������������� slave �� ������, ���������� �� ���� ���������� ��������� �����
  Wire.begin(take_slot_number(h_pin,hl_pin,lh_pin,ll_pin));
  //������������� ����������� ������� � ������
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
//����� WatchDoga
wdt_reset();
//�� ������ ������ ������������� ��������� ����������
sei();
}

void loop() {
//�� ������ ������ �������� WatchDog
  wdt_enable(WDTO_8S);

//���������� �������� �� ������� answer_long � ���-����
 // for (int i=0;i<5;i++){
  //Serial.print(answer_long[i]);
 // }
//  Serial.println(' ');

//��������� � ����� ������, ������� ������������ ������������ � ����������
//��������� ������� open
  if (answer_long[4]=='o'){
        //���������� � answer_long[0] ��������� ���������� ������ open
    answer_long[0]=open_curtain();
        //���������� � answer_long[4] ������� � ������� ��������, ��� �������� �� ����������
    answer_long[4]='O';
  }
  //����� ��� ���� �����
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

//����� WatchDog
  wdt_reset();
}





