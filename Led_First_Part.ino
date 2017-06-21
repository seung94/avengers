/* 작성자 : 박승화
 *  led first part
 *  AXM address : 7000
 *  Sector : A, B
   */
#include <SoftwareSerial.h> //axm100통신을 위한 헤더파일
#include<MsTimer2.h> //timer()함수를 사용하기 위한 헤더파일

SoftwareSerial uno(2, 3); //Axm100과 통신을 위한 함수 (tx, rx)

#define MAX_SPEED 3500
#define CURRENT 0.04 //40mA사용

//주소
#define ADR_POWER "1000"
#define ADR_TX "1000" //받는 곳의 address

//명령어
#define CMD_0 "00" //낮
#define CMD_1 "01" //밤(기본상태)
#define CMD_2 "02" //sector 제어
#define CMD_3 "03" //전력사용량 (앞은 led 번호) 03은 led0, 13은 led1
#define CMD_4 "04" //조도 or 차량 통과

//핀 번호
#define ledPin_A1 10
#define ledPin_A2 9
#define ledPin_B1 6
#define ledPin_B2 5

//포트 번호
#define PORT_A1 "00"
#define PORT_A2 "01"
#define PORT_B1 "02"
#define PORT_B2 "03"

#define POWER_SIZE 6 //가로등 전력 사용량 사이즈
#define V_SIZE 7 //속도값 사이즈
#define CMD_FIRST 4 //packet에서 command시작 배열 번호
#define DATA_FIRST 6 //packet에서 data시작 배열 번호
#define DATA_CDS_FIRST 5 //data_rx에서 cds값이 시작되는 배열의 번호
#define MAX_PKT_SIZE 14 //packet의 사이즈
#define NUM_SECTOR 2//sector 수
#define NUM_LED_IN_SECTOR 2 ///한 섹터당 제어하는 led port수(한 섹터당 led는 3개이지만 제어하는 port는 2개(1개는 첫번째 led, 1개는 두번째 & 세번째 led))
#define NUM_PORT NUM_SECTOR*NUM_LED_IN_SECTOR //총 port 수
#define HIGH_VOLT 5 //led on 최대 volt값
#define LOW_VOLT 0 //led off volt값
#define PERIOD 100 //power check를 위한 주기적 시간 [ms]
#define POWER_TX_PERIOD 10000 //측정한 power값을 보내기 위한 주기적 시간[ms]
#define ON_ING_TIME_FIRST 5
#define ON_ING_TIME_SECOND 10

int waiting_time_first = 1000; //led가 켜지는데 걸리는 시간[ms]
int waiting_time_second = 1000; //led가 켜지는데 걸리는 시간[ms]
int cds_value=0; //cds의 정수형
int count_ms = 0;
int count_m = 0;
int sector_case=0;

float volt_value[NUM_PORT] = {0}; //현재 가로등 전력값
float volt_sum_value[NUM_PORT] = {0}; //각 가로등마다 사용한 전체 전력값

unsigned long interval_time; //led running time을 계산하기 위한 변수
unsigned long start_time[NUM_PORT]; //led가 켜지는 순간의 시간을 저장하기 위한 변수
unsigned long end_time; //running time시 체크할 때 시간을 저장하기 위한 변수
unsigned long running_time[NUM_PORT]; //가로등의 running time을 저장하는 변수
unsigned long temp_time; //차량이 통과한 후 켜지는 시간을 계산하기 위한 변수

boolean sector_on_off[NUM_SECTOR] = {false}; //sector의 on/off상태를 체크하기 위한 변수
boolean led_on_off[NUM_PORT] = {false}; //가로등의 on/off상태를 체크하기 위한 변수
boolean running_night = false; //밤인지를 체크하기 위한 변수
boolean passing_again = false; //차량이 통과했는지 체크하기 위한 변수
boolean interrupt = false; //차량이 통과 중에 다른 packet이 들어왔는지 체크하기 위한 변수

char pkt_rx[15]; //받은 packet 저장
char cmd_rx[3]; //받의 packet의 command 저장
char data_rx[9]; //받은 packet의 data 저장
char sector_rx[3]; //어떤 sector을 제어하는지 알려주는 변수
char cds_passing_rx; //조도값을 주는 패킷인지 차량이 통과하는 패킷인지 알려주는 변수
char cds_temp_value[4]; //받은 패킷에서 cds값을 추출하기 위한 문자형 배열
char temp[14]; //문자형으로 패킷을 전송하기 위해서 사용하는 임시 변수
char c[9]; // 숫자를 문자로 바꾸기 위해서 사용하는 변수

void Compare_Cmd_Data(); //어떤 명령어가 들어왔는지 판단하는 함수
void Compare_Sector(); // sector제어시에 어떤 sector가 제어되는지 판단하기 위해서 사용되는 함수
void Cds_or_Passing(); //조도 또는 차량의 통과를 알려주는 패킷의 데이터값을 추출하기 위한 함수
void During_On(unsigned long); //led가 on하는 시간동안 다른 packet이 들어왔는지 체크하기 위한 함수 
void LED_Night(); //밤일 때를 구동을 위한 함수
void LED_Afternoon(); //오후일 때 구동을 위한 함수
void Power_Check(); //각 led마다 power사용량을 측정하기 위한 함수(주기적으로 체크)
void Passing_Time(); //가로등 점등 시간 계산 함수
void P_TX(); //사용 전력량 전송 함수
void itoa(int); //int 형을  char 형으로 바꿔주는 함수

void setup() {
  //serial 및 axm100과의 무선통신을 위한 begin()함수
  Serial.begin(115200);
  uno.begin(115200); //axm100으로 속도(B)로 115200사용
  MsTimer2::set(PERIOD, Power_Check); //전력사용량을 측정하기 위한 timer선언
  MsTimer2::start();

   //LED(output) 선언
  pinMode(ledPin_A1, OUTPUT);
  pinMode(ledPin_A2, OUTPUT);

  pinMode(ledPin_B1, OUTPUT);
  pinMode(ledPin_B2, OUTPUT);
  
  Serial.println("RX Start!"); 
}

void loop() {
  if(uno.available()> 0) //axm100으로 데이터가 들어오는지 확인하는 if문
  {
    int i;
     uno.readBytes(pkt_rx, 14); 
     if(pkt_rx[0] != '7')
     {
        memset(pkt_rx, 0, sizeof(pkt_rx)); //배열데이터 초기화 
     }
     //buffer에 저장되어있는 14길이만큼 읽어드린다, return값은 받은 길이
     pkt_rx[14] = '\0'; //문자열의 종료를 넣어서 배열의 끝을 알려준다.
     Compare_Cmd_Data();
     memset(pkt_rx, 0, sizeof(pkt_rx)); //배열데이터 초기화 
  }
  Passing();
}

void Compare_Cmd_Data() //cmd와 data비교
{
  int i;
  for(i=CMD_FIRST; i<DATA_FIRST;++i) //명령어 insert
  {
    cmd_rx[i-CMD_FIRST] = pkt_rx[i];
  }
  cmd_rx[i-CMD_FIRST] = '\0';
  
  for(i=DATA_FIRST; i<MAX_PKT_SIZE;++i) //데어터 insert
  {
    data_rx[i-DATA_FIRST] = pkt_rx[i];
  }
  data_rx[i-DATA_FIRST] = '\0';

  if(strcmp(CMD_0, cmd_rx) == 0) //낮
  {
        LED_Afternoon();
        sector_on_off[0] = false; 
        sector_on_off[1] = false;
  }
  
  else if(strcmp(CMD_1, cmd_rx) == 0) //밤
   {
        LED_Night();
        passing_again = false;
        sector_on_off[0] = false; 
        sector_on_off[1] = false;
   }
        
  else if(strcmp(CMD_2, cmd_rx) == 0) //sector제어
  {
       Compare_Sector();
  }    
       
  else if(strcmp(CMD_4, cmd_rx) == 0) // 조도 및 차량 패싱
  {
      Cds_or_Passing();
  }
  
  else; 
}

void Compare_Sector() //sector 제어명령어가 들어올 경우, sector 비교
{
    int i;
    for(i=DATA_FIRST; i<DATA_FIRST+2;++i) //데어터 insert
    {
        sector_rx[i-DATA_FIRST] = pkt_rx[i];
    }
    sector_rx[i-DATA_FIRST] = '\0';
//    Serial.print("sector : ");
    switch(sector_rx[1])
    {
      case '0' :
        sector_on_off[0] = true; //sector 제어를 표시       
        analogWrite(ledPin_A1, 255);
        start_time[0] = millis();
        volt_value[0] = HIGH_VOLT; //5V
        led_on_off[0] = true;
        
        analogWrite(ledPin_A2, 255);
        start_time[1] = millis(); 
        volt_value[1] = HIGH_VOLT;
        led_on_off[1] = true;
//        Serial.println("A");
        break;
        
      case '1' :
        sector_on_off[1] = true;
        
        analogWrite(ledPin_B1, 255);
        start_time[2] = millis();
        volt_value[2] = HIGH_VOLT; //5V
        led_on_off[2] = true;
        analogWrite(ledPin_B2, 255);
        start_time[3] = millis();
        volt_value[3] = HIGH_VOLT; //5V
        led_on_off[3] = true;
//        Serial.println("B");
        break;
    }
}
void Passing()
{
  int i,j;
  unsigned long waiting_time_A, waiting_time_B;
  if(running_night == true)
  {
    interrupt = false; //init
    while(passing_again)
    {
      passing_again = false; //init

      //LED ON
      start_time[0] = millis(); //led A1 on
      volt_value[0] = HIGH_VOLT;
      led_on_off[0] = true;
      analogWrite(ledPin_A1, 255);
      temp_time = start_time[0] + waiting_time_first;
      passing_again = false;
      During_On(temp_time); //led를 on하는 시간을 기다리면서 interrupt되는지를 기다림
      if(interrupt == true) //secotr제어나 차량 passing 이외의 packet이 도착하면 while문을 빠져나옴
      {
        break; //낮 or 밤(기본상태) 입력시
      }
        
      start_time[1] = millis(); //led A2 on
      volt_value[1] = HIGH_VOLT;
      led_on_off[1] = true;
      analogWrite(ledPin_A2, 255);
      temp_time = start_time[1] + waiting_time_second;
      During_On(temp_time); 
      if(interrupt == true)
      {
        break;
      }
      
      start_time[2] = millis(); //led B1 on
      volt_value[2] = HIGH_VOLT;
      led_on_off[2] = true;
      analogWrite(ledPin_B1, 255);
      temp_time = start_time[2] + waiting_time_first;
      During_On(temp_time);
      if(interrupt == true)
      {
        break;
      }

      start_time[3] = millis(); //led B2 on
      volt_value[3] = HIGH_VOLT;
      led_on_off[3] = true;
      analogWrite(ledPin_B2, 255);
      temp_time = start_time[3] + waiting_time_second;
      During_On(temp_time);
      if(passing_again == true) //차량이 passing할 경우 while문의 첫번째 줄로 이동하여 다시 led on을 한다.
      {
         continue; 
      }
      else if(interrupt == true)
      {
          break;
      }


      //LED OFF
      sector_case = 0; //Init
      for(i=0; i<NUM_SECTOR; ++i) //sector가 몇개 제어 중인지를 check
      {
        if(sector_on_off[i] == true)
            sector_case++;
      }

      //sector 제어가 없을 경우
      if(sector_case == 0) 
      {
        //sector A1
        start_time[0] = millis();
        temp_time = start_time[0] + waiting_time_first * ON_ING_TIME_FIRST;
        During_On(temp_time);
        if(passing_again == true ) 
        {
            continue;
        }
        else if(interrupt == true)
        {
            break;
        }
        else if(sector_on_off[0] == false)
        {
           start_time[0] = millis();
           volt_value[0] = (float)cds_value/255 * HIGH_VOLT ;
           led_on_off[0] = true;
           analogWrite(ledPin_A1, cds_value);
        }
       
        //sector A2
        start_time[1] = millis();
        temp_time = start_time[1] + waiting_time_first *ON_ING_TIME_SECOND;
        During_On(temp_time);
        if(passing_again == true)
        {
            continue;
        }
        else if(interrupt == true)
        {
            break;
        }
        else if(sector_on_off[0] == false)
        {
           start_time[1] = millis();
           volt_value[1] = LOW_VOLT ;
           led_on_off[1] = false;
           analogWrite(ledPin_A2, 0);
        }  
       
        //sector B1
        start_time[2] = millis();
        temp_time = start_time[2] + waiting_time_first * ON_ING_TIME_FIRST;
        During_On(temp_time);
        if(passing_again == true)// || interrupt == true)//(interrupt == true)//passing_again == true)
        {
           continue;
         }
        else if(interrupt == true)
        {
          break;
        }
        else if(sector_on_off[1] == false)
        {
           start_time[2] = millis();
           volt_value[2] = (float)cds_value/255 * HIGH_VOLT ;
           led_on_off[2] = true;
           analogWrite(ledPin_B1, cds_value);  
        }
         
        //sector B2
        start_time[3] = millis();
        temp_time = start_time[3] + waiting_time_first *ON_ING_TIME_SECOND;
        During_On(temp_time);
        if(passing_again == true )
        {
            continue;
        }
        else if(interrupt == true)
        {
            break;
        }
        else if(sector_on_off[1] == false)
       {
           start_time[3] = millis();
           volt_value[3] = LOW_VOLT ;
           led_on_off[3] = false;
           analogWrite(ledPin_B2, 0);
       }
     }  

      //sector 1개 제어 중일 경우
      else if(sector_case == 1) 
      {
         if(sector_on_off[0] == true ) //sector A제어 중
         {
           waiting_time_A = millis(); //Aector가 제어 중이기 때문에 해당 시간만큼 기다리기 위해서 사용(1번)
           temp_time = waiting_time_A + waiting_time_first;
           During_On(temp_time);
           if(passing_again == true)
           {
              continue;
           }
           else if(interrupt == true)
           {
            break;
           }
            
          waiting_time_A = millis(); //Aector가 제어 중이기 때문에 해당 시간만큼 기다리기 위해서 사용(1번, 총 2번(2개의 port사용))
          temp_time = waiting_time_A + waiting_time_second;
          During_On(temp_time);
         if(passing_again == true)
         {
             continue;
         }
         else if(interrupt == true)
         {
            break;
         }
            
         start_time[2] = millis();
         temp_time = start_time[2] + waiting_time_first;
         During_On(temp_time);
         if(passing_again == true)
         {
            continue;
         }
         else if(interrupt == true)
         {
            break;
         }
         else if(sector_on_off[1] == false)
         {
           start_time[2] = millis();
           volt_value[2] = (float)cds_value/255 * HIGH_VOLT ;
           led_on_off[2] = true;
           analogWrite(ledPin_B1, cds_value);
         }
          
         start_time[3] = millis();
         temp_time = start_time[3] + waiting_time_first;
         During_On(temp_time);
        if(passing_again == true)
         {
              continue;
         }
         else if(interrupt == true)
         {
            break;
         }
         else if(sector_on_off[1] == false)
        {
           start_time[3] = millis();
           volt_value[3] = LOW_VOLT ;
           led_on_off[3] = false;
           analogWrite(ledPin_B2, 0);
        }
      }
        
      else if(sector_on_off[1] == true) //sector B 제어 중일 경우
      {
           start_time[0] = millis();
           temp_time = start_time[0] + waiting_time_first;
           During_On(temp_time);
           if(passing_again == true)
           {
              continue;
           }
           else if(interrupt == true)
           {
            break;
           }
          else if(sector_on_off[0] == false)
          {
              start_time[0] = millis();
              volt_value[0] = (float)cds_value/255 * HIGH_VOLT ;
              led_on_off[0] = true;
              analogWrite(ledPin_A1, cds_value);
          }
        
          start_time[1] = millis();
          temp_time = start_time[1] + waiting_time_first;
          During_On(temp_time);
          if(passing_again == true)
          {
              continue;
          }
          else if(interrupt == true)
          {
             break;
          }
          else if(sector_on_off[0] == false)
         {
           start_time[1] = millis();
           volt_value[1] = LOW_VOLT ;
           led_on_off[1] = false;
           analogWrite(ledPin_A2, 0);
         }
         
         waiting_time_B = millis(); //Aector가 제어 중이기 때문에 해당 시간만큼 기다리기 위해서 사용(1번)
         temp_time = waiting_time_B + waiting_time_first;
         During_On(temp_time);
        if(passing_again == true)
        {
            continue;
        }
        else if(interrupt == true)
        {
            break;
        }  
  
        waiting_time_B = millis(); //Aector가 제어 중이기 때문에 해당 시간만큼 기다리기 위해서 사용(1번, 총 2번(2개의 port사용))
        temp_time = waiting_time_B + waiting_time_second;
        During_On(temp_time);
       if(passing_again == true)
       {
            continue;
       }
       else if(interrupt == true)
       {
            break;
       }
      }
    }
    else //sector 모두 제어중일 때
    {
       break;
    }
   } 
  }
 }
 
void Cds_or_Passing()//조도 또는 차량의 통과를 알려주는 패킷의 데이터값을 추출하기 위한 함수 
{
  int i;
    if(data_rx[0] == '0') //차량패싱
    {
      passing_again = true;
      Passing_Time();
    }
    else if(data_rx[0]=='1') //조도값
    {     
      for(i=DATA_CDS_FIRST; i<DATA_CDS_FIRST+3;++i)
      {
        cds_temp_value[i-DATA_CDS_FIRST] = data_rx[i]; 
      }
      cds_temp_value[i] = '\0';
      cds_value = atoi(cds_temp_value);
      if(led_on_off[0] == true && sector_on_off[0] == false)
        analogWrite(ledPin_A1, cds_value);
      if(led_on_off[1] == true && sector_on_off[0] == false)
        analogWrite(ledPin_A2, cds_value);
      if(led_on_off[2] == true && sector_on_off[1] == false)
        analogWrite(ledPin_B1, cds_value);
      if(led_on_off[3] == true && sector_on_off[1] == false)
        analogWrite(ledPin_B1, cds_value);  
    }
}

void LED_Afternoon() //led all off
{
  int i;
  running_night = false;
  analogWrite(ledPin_A1, 0);
  analogWrite(ledPin_A2, 0);
  analogWrite(ledPin_B1, 0);
  analogWrite(ledPin_B2, 0);

  for(i=0; i<NUM_PORT;++i)
  {
    volt_value[i] = LOW_VOLT; //0V
    led_on_off[i] = false;
  }
}

void LED_Night()
{
  running_night = true;
  analogWrite(ledPin_A1, cds_value);
  start_time[0] = millis();
  volt_value[0] = (float)cds_value/255 * HIGH_VOLT;
  led_on_off[0] = true;
  
  analogWrite(ledPin_B1, cds_value);
  start_time[2] = millis();
  volt_value[2] = (float)cds_value/255 * HIGH_VOLT;
  led_on_off[2] = true;

  analogWrite(ledPin_A2, 0);
  volt_value[1] = LOW_VOLT;
  led_on_off[1] = false;

  analogWrite(ledPin_B2, 0);
  volt_value[3] = LOW_VOLT;
  led_on_off[3] = false;
  
}

void During_On(unsigned long temp_t) //led가 켜지는 시간까지 다른 packet을 받았는지 체크하기 위한 함수
{
  interrupt = false;
  while(temp_t > millis())
  {
    if(uno.available()> 0) //axm100으로 데이터가 들어오는지 확인하는 if문
    {
      uno.readBytes(pkt_rx, 14); 
     if(pkt_rx[0] != '7')
     {
        memset(pkt_rx, 0, sizeof(pkt_rx)); //배열데이터 초기화 
     }
     
     //buffer에 저장되어있는 14길이만큼 읽어드린다, return값은 받은 길이
     pkt_rx[14] = '\0'; //문자열의 종료를 넣어서 배열의 끝을 알려준다.
     if(pkt_rx[5]=='4' &&  pkt_rx[7] == '0') //passing
     {
        Passing_Time(); //차량의 속도를 계산에서 led 켜지는 속도를 설정
         passing_again = true;
     }
      else
      { 
        Compare_Cmd_Data();
        if(strcmp(CMD_0, cmd_rx) == 0) 
          interrupt = true;
        else if(strcmp(CMD_2, cmd_rx) == 0)
        {
          while(temp_t > millis()); //waiting을 주기 위해서 while문 사용
        }
         break;
      }
    }
  }
}

void Power_Check()
{
  int i;
  end_time = millis();
  for(i=0; i<NUM_PORT; i++)
  {
    interval_time = end_time - start_time[i]; //[msec]
    interval_time /= 1000; // [sec]
     volt_sum_value[i] += interval_time * volt_value[i] * CURRENT;// [W*sec]
  }
  count_ms++; //PERIOD[ms]마다 counting
  if(count_ms % (1000/PERIOD) == 0 ) //msec to sec
  {
     count_ms = 0; //init
     count_m++;
     
     //1분경과마다 1분동안 가로등마다 사용된 소비 전력량을 제어부분에 전송
    if(count_m % 60 == 0) //sec to min
    {
       count_m = 0;
      P_TX();
      //packet전송 및 power reset
    }
  }
}

void itoa(int number)
{
  int i, j;
  int radix = 10; //진수
  int deg = 1;
  int cnt = 0;
  //char c[9];
  while(1) //자리수 알기 위해서 사용
  {
    if((number/deg) > 0)
      cnt++;
    else
      break;
    deg *= radix;
  }
 // Serial.print("자리수 : ");
  //Serial.println(cnt);
  deg /= radix; //자리수보다 한자리 높게 카운팅 됨
  for(i=0; i<(POWER_SIZE - cnt) ;i++)
  {
    c[i] = '0';
  }
  for(j=0; j< cnt; j++)
  {
    c[i++] = number/deg + '0'; //가장 큰 자리의 수부터 뽑음
    number -= ((number/deg) * deg); //뽑은 자리수의 수를 없앰
    deg /= radix; //자리수를 줄임
  }
  c[i] = '\0';
}

void P_TX()
{
  int i;
  String adr_power = String(ADR_POWER);
  String cmd_power = String(CMD_3);
  String data_power;
  String port_power;
  String pkt_power;
  char data_temp[9];
  for(i=0; i<NUM_PORT;i++)
      {
//        Serial.print("led");
//        Serial.print(i);
//        Serial.print(" : ");
//        Serial.println(volt_sum_value[i]);
        itoa((int)volt_sum_value[i]); //
        start_time[i] = millis(); //reset for preventing overflow of data
        volt_sum_value[i] = 0; //reset for preventing Overflow
        data_power = c; //c는 volt_sum_value값의 char형변환한 값
        pkt_power = adr_power + cmd_power;
        if(i==0) //A1
          port_power = String(PORT_A1);
        else if(i==1)
          port_power = String(PORT_A2);
        else if(i==2)
          port_power = String(PORT_B1);
        else if(i==3)
          port_power = String(PORT_B2);
        pkt_power += port_power + data_power;
        
        uno.print(pkt_power);
        
      }

      //Init volt_sum_value 
      for(i=0; i<NUM_PORT;i++)
      {
        volt_sum_value[i] = 0;
      }

}
void Passing_Time()
{
  char temp_time[8]; //최대속도의 빠르기는 5자리
  int p_time;
  float operating_time;
  int i;
  p_time = atoi(data_rx); //차량의 속도
  operating_time = MAX_SPEED/p_time; //가로등이 켜지는 시간

  waiting_time_first = operating_time; //가로등 1개
  waiting_time_second = operating_time * 2; //가로등 2개
}

