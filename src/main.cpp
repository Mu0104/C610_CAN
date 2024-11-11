#include <M5Unified.h>
#include <mcp_can.h>
#include <SPI.h>


//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 1                   // Timer Interrupt Period
#define VSPI_CLOCK_PIN 18
#define VSPI_MOSI_PIN 23
#define VSPI_MISO_PIN 38

// CAN
long unsigned int canId;
unsigned char len = 0;
unsigned char data[8];
unsigned char datas[8];
unsigned char datar[8];
#define CAN_INT 35 //32
MCP_CAN CAN(27);    //33 

int cnt = 0;
int TYPE = 0; // Standard Format
int DLC = 8;
byte sndStat;

byte angle_H;
byte angle_L;
byte velocity_H;
byte velocity_L;
byte torque_H;
byte torque_L;

int angle_buff;
float angle;
int velocity_buff;
int velocity;
int torque_buff;
int torque;
float angle2;
int velocity2;
int torque2;

float gain_P = 1.4;
float gain_I = 0.1;
float gain_D = 0.1;
float send_current;

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer10;
int iTimer50;
int iTimer100;


unsigned int brake_mode = 2;

void init_can(void);
void torqueControl(int trq);
void RecieveBrake(void);
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);
void velocity_control(float Vf, float Vi);

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Lcd.setTextSize(2);
  init_can();
  M5.Lcd.println("C610 Test");   
  delay(100);
  brake_mode = 2;
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 
   
}

void loop()
{
  timerInterrupt();

}

// Timer Interrupt
//------------------------------------------------------------------//
void timerInterrupt(void) {

  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    RecieveBrake();

    iTimer10++;
    //10ms timerInterrupt
    switch(iTimer10){
      case 1:     
        torqueControl(1000);
        //velocity_control(1000,velocity);
        break;
      
      case 2:     

        break;

      case 10:

        iTimer10 = 0;
        break;
    }

    iTimer50++;
    //50ms timerInterrupt
    switch(iTimer50){
      case 10:
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println(velocity);
        M5.Lcd.println(torque);
        break;
      
      case 50:
        iTimer50 = 0;
        break;
    }
      


  }
}

void init_can(void){
  CAN.begin(SPI,MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
}
void torqueControl(int trq)
{
  
  canId = 0x200;
  if (trq > 3000)
    trq = 3000;
  if (trq < -3000)
    trq = -3000;

  switch (brake_mode)
  {
  case 0:
    data[0] = 0 >> 8 & 0xFF;
    data[1] = 0 & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    sndStat = CAN.sendMsgBuf(0x200, TYPE, DLC, data);
    break;

  case 1:
    data[0] = -3000 >> 8 & 0xFF;
    data[1] = -3000 & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    sndStat = CAN.sendMsgBuf(0x200, TYPE, DLC, data);
    break;

  case 2:
    data[0] = trq >> 8 & 0xFF;
    data[1] = trq & 0xFF;
    data[2] = 500 >> 8 & 0xFF;
    data[3] = 500 & 0xFF;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    sndStat = CAN.sendMsgBuf(0x200, TYPE, DLC, data);
    break;
  }
  if (sndStat == CAN_OK){
  }
  else{
  }
  
}
void RecieveBrake(void)
{
  if(!digitalRead(CAN_INT))
  {
    CAN.readMsgBuf(&canId, &len, data);

    if(canId==0x201){
      for (int i = 0; i < 8; i++)
      {
        switch (i)
        {
          case 0:
            angle_H = (char)data[i];
            break;
          case 1:
            angle_L = (char)data[i];
            break;
          case 2:
            velocity_H = (char)data[i];
            break;
          case 3:
            velocity_L = (char)data[i];
            break;
          case 4:
            torque_H = (char)data[i];
            break;
          case 5:
            torque_L = (char)data[i];
            break;
          case 6:
            break;
          case 7:
            break;
          
        }
      }
      angle_buff = ((angle_H << 8 & 0xFF00) | (angle_L & 0xFF));
      angle = (float)angle_buff / 8191 * 360;
      velocity_buff = ((velocity_H << 8 & 0xFF00) | (velocity_L & 0xFF));
      if (velocity_buff >= 32768)
      {
        velocity = velocity_buff - 65535;
      }
      else
      {
        velocity = velocity_buff;
      }
      torque_buff = ((torque_H << 8 & 0xFF00) | (torque_L & 0xFF));
      if (torque_buff >= 32768)
      {
        torque = torque_buff - 65535;
      }
      else
      {
        torque = torque_buff;
      }

    }
    
  }
}
// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void velocity_control(float Vf, float Vi){
  float integral = 0;
  float last_err = 0;
  unsigned long last_micros = 0;

  float dX = Vf - Vi;  
  float P = gain_P * dX;
  unsigned long current_micros = micros();            
  float dt = ((float)(current_micros - last_micros))
              / 1000000.0; 
  
  integral += dX * dt;                              
  float I = gain_I * integral;                       
  float diff = (dX - last_err) / dt;                
  float D = gain_D * diff;                           
  send_current = P + I + D;

  if(send_current > 10000){
    send_current = 10000;
  }else if(send_current < -10000){
    send_current = -10000;
  }
  torqueControl((int)send_current);
  last_err = dX;                                    
  last_micros = current_micros;  

}