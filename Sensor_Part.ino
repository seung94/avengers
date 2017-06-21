/* 작성자 : 장원준, 박승화
 *  sensor part
 *  AXM address : 3000
 */
#include <MsTimer2.h>
#include<SoftwareSerial.h>
SoftwareSerial uno(2, 3);

#define PIN_A 0 //적외선 A0
#define PIN_B 1 //적외선 A1
#define PIN_C 2 //적외선 B0
#define PIN_D 3 // 적외선 B1
#define CDSPIN 5 //조도센서
#define PASS_SPEED 3000 //첫 센서에서 객체를 감지 후 다음 센서에서 객체가 감지되기까지 기다릴 시간(단위:ms)
#define ADR_TX  "7000" //가로등 통신모듈 주소 1
#define ADR_TX2 "6000" //가로등 통신모듈 주소 2
#define CMD_04 "04" //명령어 04 조도값 전송
#define MAX_ARR 9  //부분 배열 크기
#define DISTPTP 9 //적외선센서 A0와 A1 양 센서간 거리(cm)
#define DISTPTP2 9 //적외선센서 B0와 B1 양 센서간 거리 
#define CDSPERIOD 1800000 //조도센서값 보내는 주기(단위 : ms)
#define COR_A0 5   //A0 passing 보정값. 적외선센서가 값을 읽어 오는 오차가 심하기 때문에 보정값이 필요하다.
#define COR_A1 3  //A1 passing 보정값
#define COR_B0 4 //B0 passing 보정값
#define COR_B1 7 //B1 passing 보정값

void Send_Car(int num); //통과한 객체의 속도를 가로등에 전송하는 함수
void Send_Cds();  //조도값을 가로등에 전송하는 함수
void calc_pkt(int val);  //int값을 받아서 char형으로 변환해 배열에 저장시켜준다.
double distanceavg(int pinnum, int MAX); //오차를 극복하기 위해 적외선센서로 감지한 거리의 평균값을 추출해줌. MAX가 오를수록 정확도향상
double check_speed(double distance, int time); //속도 = (거리/시간) 임을 이용해 초속을 반환하는 함수
int checkDist(); //적외선센서 A부분.루프에서 계속 돌아갈 함수. 위의 함수들을 이용해 객체 감지 후 실질적으로 패킷 전송 명령을 내릴 함수.
int checkDist2(); //상동. 적외선센서 B부분
void check_cds(); //조도를 측정할 함수
long unsigned int waiting_time; //현재시간+PASS_SPEED를 넣어서 기다릴 시간을 측정. 센서 A부분
long unsigned int waiting_time2; //상동. 센서B부분


/*  보낼 주소, 보낼 데이터, 보낼 명령 정의 
 *   계산할 변수 정의
 */
String adr_tx;  //주소를 저장할 스트링변수
String data_tx;  //데이터값을 저장할 스트링변수
String cmd_tx;  //명령어 번호를 저장할 스트링변수
String pkt_tx;  //주소,커맨드,데이터를 합쳐서 전송될 스트링변수
String last_arr; //패킷 계산을 도와주기 위한 임시 저장 변수 
double distance_A = 0; //Cm로 계산된 거리값을 저장해두기위해 변수를 선언합니다.
double distance_B = 0;
double distance_C = 0; //Cm로 계산된 거리값을 저장해두기위해 변수를 선언합니다.
double distance_D = 0;
double volt_A, volt_B, volt_C, volt_D;  //distanceavg 함수의 리턴값을 받는 변수
double passing_A, passing_B,passing_C, passing_D; // 객체가 지나갔음을 판단하는 기준이 되는 변수
unsigned long prev_time=0, prev_time2 = 0; //객체가 첫 적외선센서를 통과 했을 때의 시간값을 받을 변수
unsigned long current_time = 0, current_time2;  //객체가 두번째 적외선 센서를 통과 했을 때의 시간 값을 받을 변수
double speed, speed2;  //객체가 지나간 속도
double garbageValue,garbageValue2;  //새나 먼지 등의 쓰레기값을 읽는 경우를 줄이기 위한 변수. 적외선센서와 너무 가까우면 쓰레기 값으로 인지
int valLed; //cdsPin값의 범위는 0~1024, led범위는 0~255
boolean sensorA = false;  // flag
boolean sensorB = false;  // flag
boolean sensorC = false;  //flag
boolean sensorD= false;  //flag

void setup() {
  Serial.begin(115200);
  uno.begin(115200);
  Serial.print("Loading...");

  /* 객체가 지나갔음을 판단할 기준을 설정하는 부분 */
  passing_A = distanceavg(PIN_A, 1000);
  passing_A = ((27.61 / (passing_A - 0.1696)) * 1000) - COR_A0;
  passing_C = distanceavg(PIN_C, 1000);
  passing_C = ((27.61 / (passing_C - 0.1696)) * 1000) - COR_B0;
  garbageValue = (passing_A / 2);
  garbageValue2 = (passing_C / 2);
  Serial.print("..");
  passing_B = distanceavg(PIN_B, 1000);
  passing_B = ((27.61 / (passing_B - 0.1696)) * 1000) - COR_A1;
  passing_D = distanceavg(PIN_B, 1000);
  passing_D = ((27.61 / (passing_D - 0.1696)) * 1000) - COR_B1;

  /* 조도 값을 측정해 전송하는 부분 */
  Serial.print("..");
  check_cds();
  Send_Cds();
  Serial.println(last_arr);
  MsTimer2::set(CDSPERIOD, check_cds); //30분마다 조도체크
  MsTimer2::set(CDSPERIOD, Send_Cds);
  MsTimer2::start();

  /* 초기 상태 시리얼 모니터에 출력 */
  Serial.println("..!!");
  Serial.print("passing_A : ");
  Serial.println(passing_A);
  Serial.print("passing_B : ");
  Serial.println(passing_B);
  Serial.print("passing_C : ");
  Serial.println(passing_C);
  Serial.print("passing_D : ");
  Serial.println(passing_D);
  Serial.println("Start!!");
  Serial.println(" ");
}

void loop() {
  checkDist();
  checkDist2();
}


/* 각 가로등에 조도값을 전송하는 함수 */
void Send_Cds()
{
  adr_tx = String(ADR_TX);
  cmd_tx = String(CMD_04);
  calc_pkt(valLed);
  data_tx = last_arr;
  pkt_tx = adr_tx + cmd_tx + data_tx;
  uno.print(pkt_tx);
  
  delay(20);  //패킷 구별위해 delay 필요
  
  adr_tx = String(ADR_TX2);
  cmd_tx = String(CMD_04);
  calc_pkt(valLed);
  data_tx = last_arr;
  pkt_tx = adr_tx + cmd_tx + data_tx;
  uno.print(pkt_tx);
}

/* 차가 지나간 속도값을 전송하는 함수 */
void Send_Car(int num)
{
  if(num == 0){
  adr_tx = String(ADR_TX);
  cmd_tx = String(CMD_04);
  calc_pkt(floor(speed));
  last_arr[0] = '0';
  data_tx = last_arr;
  pkt_tx = adr_tx + cmd_tx + data_tx;
  uno.print(pkt_tx); 
  Serial.print(pkt_tx);
  }
  else{
  adr_tx = String(ADR_TX2);
  cmd_tx = String(CMD_04);
  calc_pkt(floor(speed2));
  last_arr[0] = '0';
  data_tx = last_arr;
  pkt_tx = adr_tx + cmd_tx + data_tx;
  uno.print(pkt_tx); 
  Serial.print(pkt_tx);
  }
}

/* int로 받은 데이터값을 char화 시켜서 패킷으로 전송시킬 수 있게 도와주는 함수 */
void calc_pkt(int val){
  char arr[MAX_ARR];
  itoa(val,arr,10);
  String arr2 = arr;
  char arr3[MAX_ARR];
  int i;
  arr3[0] = '1';
  for(i=1; i< MAX_ARR -1 - arr2.length(); i++)
  {
    arr3[i] = '0';
  }
  arr3[i] = '\0';
  last_arr = arr3 + arr2;
}

/* 거리 값을 평균을 내서 반환하는 함수 */
double distanceavg(int pinnum, int MAX) {
  long double temp = 0;
  int i;
  for (i = 0; i < MAX; i++) {
    temp = temp + map(analogRead(pinnum), 0, 1023, 0, 5000);
  }
  temp = (temp / MAX);

  return abs(temp);
}

/* 두 센서 사이의 거리와 지나가는 데 걸린 시간값을 받아 속도를 초속 기준으로 반환하는 함수 */
double check_speed(double distance, int time){
   return (distance/time) * 1000;   //초속 반환
}

/* 위의 각 함수들을 이용해 A지점에서의 객체를 감지하고 객체가 정상적으로 지나가면 속도를 전송하게 하는 함수 */
int checkDist()
{
  
  volt_A = distanceavg(PIN_A, 5); //오차를 줄이기 위한 평균 추출함수사용
  distance_A = (27.61 / (volt_A - 0.1696)) * 1000; //거리값을 cm로 변환하는 계산공식 적용

  /* 센서와 너무 가깝지 않은 거리에서 첫 번째 센서에서 객체를 감지하면 */
  if ((garbageValue < distance_A) && (distance_A < passing_A)){  
    if(sensorA == false){ //flag가 내려가 있었어야 작동. sensorA에서 감지가 되지 않았던 상태였어야 새로 감지를 하는 것이다
      sensorA = true;   
    
      
    prev_time = millis();

    waiting_time = prev_time + PASS_SPEED;
    Serial.println("A에서 객체 감지");
    }
  }
    /* 센서 B, 즉 적외선 A1에서의 감지 */
    volt_B = distanceavg(PIN_B, 5);
    distance_B = (27.61 / (volt_B - 0.1696)) * 1000;
    
    /* 센서 B에서 객체를 감지하고 이전에 이미 A에서 감지된 상태였으면 정상적으로 객체가 passing 한 것 */
    if ((distance_B < passing_B) && (sensorA == true))  
      {
      sensorB = true;
      current_time = millis();
      speed = check_speed(DISTPTP,current_time - prev_time);
      Serial.println("B에서 객체 감지");
      Serial.println("A,B가 정상적으로 지나감");
      Send_Car(0);
      Serial.print("속도 : ");
      Serial.print(speed);
      Serial.println("cm/s");
      Serial.print("걸린시간 : ");
      Serial.print((current_time - prev_time)/1000.0);
      Serial.println("초");
      delay(100);
      sensorA = false;
      return 0;
      }
      /*B에서 객체가 감지되지 전에 기다릴 시간이 초과되버리면 A에서 값을 잘못 읽은것으로 판단하므로 상태를 되돌림*/
     else if(millis() > waiting_time && sensorA == true){
          Serial.println("B가 감지되지 않음");
          sensorB = false;
          sensorA = false;
     }
  
}

/* 위의 각 함수들을 이용해 B지점에서의 객체를 감지하고 객체가 정상적으로 지나가면 속도를 전송하게 하는 함수 */
int checkDist2()
{
  
  volt_C = distanceavg(PIN_C, 5); //오차를 줄이기 위한 평균 추출함수사용
  distance_C = (27.61 / (volt_C - 0.1696)) * 1000; //거리값을 cm으로 변환하는 계산공식 적용

 /* 센서와 너무 가깝지 않은 거리에서 첫 번째 센서에서 객체를 감지하면 */
  if ((garbageValue2 < distance_C) && (distance_C < passing_C)){
    if(sensorC == false){
      sensorC = true;  
    
      
    prev_time2 = millis();

    waiting_time2 = prev_time2 + PASS_SPEED;
    Serial.println("C에서 객체 감지");
    }
  }
  
  /* 센서 D, 즉 적외선 B1에서의 감지 */
    volt_D = distanceavg(PIN_D, 5);
    distance_D = (27.61 / (volt_D - 0.1696)) * 1000;

    /* 센서 D에서 객체를 감지하고 이전에 이미 C에서 감지된 상태였으면 정상적으로 객체가 passing 한 것 */
    if ((distance_D < passing_D) && (sensorC == true))
      {
      sensorD = true;
      current_time2 = millis();
      speed2 = check_speed(DISTPTP2,current_time2 - prev_time2);
      Serial.println("D에서 객체 감지");
      Serial.println("C,D가 정상적으로 지나감");
      Send_Car(1);
      Serial.print("속도 : ");
      Serial.print(speed2);
      Serial.println("cm/s");
      Serial.print("걸린시간 : ");
      Serial.print((current_time2 - prev_time2)/1000.0);
      Serial.println("초");
      delay(100);
      sensorC = false;
      return 0;
      }

     /* D에서 객체가 감지되지 전에 기다릴 시간이 초과되버리면 C에서 값을 잘못 읽은것으로 판단하므로 상태를 되돌림*/
     else if(millis() > waiting_time2 && sensorC == true){
          Serial.println("D가 감지되지 않음");
          sensorD = false;
          sensorC = false;
     }
  
}

/* 조도를 측정한다. 조도 센서도 가끔 값이 현저히 낮게 나오는 오류가 있기 때문에 여러번 측정해서 가장 높은 값 사용 */
void check_cds()
{
  int a=0, b;
  for(int i=0;i<5;i++){
    b=a;
    a=(analogRead(CDSPIN))/4;
    valLed = max(a,b);
  }
}

