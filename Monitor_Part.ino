/* 작성자 : 박승화
 * monitor part
 * AXM address : 3000
 */
#include <MsTimer2.h>
#include <SoftwareSerial.h>

#define DISTINCT_PACKET 100
#define CMD_SIZE 2
#define ADR_SIZE 4
#define ADR_TX_1 "7000" //LED 1의 ADDRESS
#define ADR_TX_2 "6000" //LED 2의 ADDRESS

#define CMD_0 "00" //낮
#define CMD_1 "01" //밤(기본상태)
#define CMD_2 "02" //sector 제어
#define CMD_3_0 "03"  //전력사용량 led1 "7000"
#define CMD_3_1 "13" //전력사용량 led2 "6000"
#define CMD_4 "04"  //조도 또는 차량 통과

#define PORT_A1 "00"
#define PORT_A2 "01"
#define PORT_B1 "02"
#define PORT_B2 "03"

#define DATA_SIZE 6
#define NUM_PORT 4
SoftwareSerial uno(2, 3); //tx,rx

String pkt_tx_1; //led1
String pkt_tx_2; //led2
String adr_tx_1 = String(ADR_TX_1); //led1
String adr_tx_2 = String(ADR_TX_2); //led2
String cmd_tx; //command
String data_tx; //data

char data_rx[9];
char num_rx[3]; //led 번호
char pkt_rx[15];
char sel;

float sum_power_1[NUM_PORT] ={0};
float sum_power_2[NUM_PORT] ={0};
void Menu();
void Select(int );
void Put_Data();
void Print_Power();
void setup() {

  Serial.begin(115200);
  uno.begin(115200);

  memset(pkt_rx, 0, sizeof(pkt_rx)); //Init
  Serial.println(" Start!");
  Menu();
}

void loop() {
 
  if(Serial.available() > 0)
  {
    sel = Serial.read();
    Select(sel);
    Menu();
  }
  
  if(uno.available() > 0) //가로등 전력 사용량 측정
  {
    uno.readBytes(pkt_rx, 14);
    pkt_rx[14] = '\0';
    
    Put_Data();
    memset(pkt_rx, 0, sizeof(pkt_rx));
  }
}

void Put_Data()
{
  int i, j;
  String adr_rx;
  String cmd_rx;
  char temp_adr[5];
  char temp_cmd[3];
  for(i=0; i<ADR_SIZE;i++)
    temp_adr[i] = pkt_rx[i];
  temp_adr[i] = '\0';
  
  j=0;
  for(;i<6; i++)
    temp_cmd[j++] = pkt_rx[i];
   temp_cmd[j] = '\0';
   
  if(strcmp(CMD_3_0, temp_cmd) == 0) //LED1
  {
    i=6; // data는 packet에서 6번째부터 시작
    num_rx[0] = pkt_rx[i++];
    num_rx[1] = pkt_rx[i++];
    num_rx[2] = '\0';

    for(j=0;j<DATA_SIZE; j++)
      data_rx[j] = pkt_rx[i++];
    data_rx[j] = '\0';

    if(strcmp(PORT_A1, num_rx) == 0) //port A1
      sum_power_1[0] += atoi(data_rx);
    else if(strcmp(PORT_A2, num_rx) == 0) //port A1
      sum_power_1[1] += atoi(data_rx);
   if(strcmp(PORT_B1, num_rx) == 0) //port A1
      sum_power_1[2] += atoi(data_rx);
    else if(strcmp(PORT_B2, num_rx) == 0) //port A1
      sum_power_1[3] += atoi(data_rx); 
  }
  else if(strcmp(CMD_3_1, temp_cmd) == 0) //led2
  {
    i=6; // data는 packet에서 6번째부터 시작
    num_rx[0] = pkt_rx[i++];
    num_rx[1] = pkt_rx[i++];
    num_rx[2] = '\0';

    for(j=0;j<DATA_SIZE; j++)
      data_rx[j] = pkt_rx[i++];
    data_rx[j] = '\0';

    if(strcmp(PORT_A1, num_rx) == 0) //port A1
      sum_power_2[0] += atoi(data_rx);
    else if(strcmp(PORT_A2, num_rx) == 0) //port A1
      sum_power_2[1] += atoi(data_rx);
    if(strcmp(PORT_B1, num_rx) == 0) //port A1
      sum_power_2[2] += atoi(data_rx);
    else if(strcmp(PORT_B2, num_rx) == 0) //port A1
      sum_power_2[3] += atoi(data_rx);
  }
}

void Menu()
{
  Serial.println("*******Menu******");
  Serial.println("1. 낮");
  Serial.println("2. 밤");
  Serial.println("3. A제어");
  Serial.println("4. B제어");
  Serial.println("5. C제어");

//  8 번과 9번은 테스트용
//  Serial.println("8. 적외선센서(차량통과");
//  Serial.println("9. 조도값 전송");
  Serial.println("0. 전력사용량 출력");
}
void Select(int sel)
{
  int i;
  switch(sel)
  {
    case '1' : //낮
      cmd_tx = String(CMD_0);  //command : 0x00
      data_tx = String("00000000");
      pkt_tx_1 = adr_tx_1 + cmd_tx + data_tx;

      uno.print(pkt_tx_1);
      delay(DISTINCT_PACKET);  // 들어오는 패킷을 구분하기 위함
      pkt_tx_2 = adr_tx_2 + cmd_tx + data_tx;
      uno.print(pkt_tx_2);
      break;
      
    case '2' : //밤
      cmd_tx = String(CMD_1); //command : 0x10
      data_tx = String("00000000");
      pkt_tx_1 = adr_tx_1 + cmd_tx + data_tx;

      uno.print(pkt_tx_1);
      delay(DISTINCT_PACKET);
      pkt_tx_2 = adr_tx_2 + cmd_tx + data_tx;
      uno.print(pkt_tx_2);
      break;
      
    case '3' : //A제어
      cmd_tx = String(CMD_2);  //command : 0x02
      data_tx = String("00000000");  //data 앞 두자리 : sector 번호(A : 00)
      pkt_tx_1 = adr_tx_1 + cmd_tx + data_tx;

      uno.print(pkt_tx_1);
      delay(DISTINCT_PACKET);
      pkt_tx_2 = adr_tx_2 + cmd_tx + data_tx;
      uno.print(pkt_tx_2);
      break;
      
    case '4' : //B제어
      cmd_tx = String(CMD_2);  //command : 0x02
      data_tx = String("01000000");  //data 앞 두자리 : sector 번호(B : 01)
      pkt_tx_1 = adr_tx_1 + cmd_tx + data_tx;

      uno.print(pkt_tx_1);
      delay(DISTINCT_PACKET);
      pkt_tx_2 = adr_tx_2 + cmd_tx + data_tx;
      uno.print(pkt_tx_2);
      break;
      
   case '5' : //C제어
      cmd_tx = String(CMD_2);  //command : 0x02
      data_tx = String("02000000");  //data 앞 두자리 : sector 번호(C : 02)
      pkt_tx_1 = adr_tx_1 + cmd_tx + data_tx;

      uno.print(pkt_tx_1);
      delay(DISTINCT_PACKET);
      pkt_tx_2 = adr_tx_2 + cmd_tx + data_tx;
      uno.print(pkt_tx_2);
      break;

    case '8' : //적외선 센싱(차량이패싱하면) 패킷 전송
      cmd_tx = String(CMD_4);  //command : 0x40
      data_tx = String("00000222");  
      pkt_tx_1 = adr_tx_1 + cmd_tx + data_tx;

      uno.print(pkt_tx_1);
      delay(DISTINCT_PACKET);
      pkt_tx_2 = adr_tx_2 + cmd_tx + data_tx;
      uno.print(pkt_tx_2);
      break;
      
   case '9' : //조도값
      cmd_tx = String(CMD_4);  //command : 0x40
      data_tx = String("10000032");  //조도값으로 뒤의 세자리(0~255)까지 사용, 0:0V, 255: 5V
                                      //조도와 적외선 센싱의 커멘드를 같게 하기 위해서 데이터의 MSB(최상위비트)를 1로 설정
      pkt_tx_1 = adr_tx_1 + cmd_tx + data_tx;

      uno.print(pkt_tx_1);
      delay(DISTINCT_PACKET);
      pkt_tx_2 = adr_tx_2 + cmd_tx + data_tx;
      uno.print(pkt_tx_2);
      break;  
    case '0' : //전력사용량
      Print_Power();
      break;
  }
}

void Print_Power()
{
  int i;
  for(i=0; i<NUM_PORT; i++)
      {
        Serial.println("*****led_1***** ");
        if(i==0)
          Serial.print("A1 : ");
        else if(i==1)
          Serial.print("A2 : "); 
        else if(i==2)
          Serial.print("B1 : "); 
        else 
          Serial.print("B2 : "); 
        Serial.println(sum_power_1[i]); 
      }

   Serial.println("");
   Serial.println("");
   
    for(i=0; i<NUM_PORT; i++)
      {
        Serial.println("*****led_2***** ");
        if(i==0)
          Serial.print("A1 : ");
        else if(i==1)
          Serial.print("A2 : "); 
        else if(i==2)
          Serial.print("B1 : "); 
        else 
          Serial.print("B2 : "); 
        Serial.println(sum_power_2[i]); 
      }
}
