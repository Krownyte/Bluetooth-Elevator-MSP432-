//------------See AP.c for details of hardware connections to CC2650--------------------
//------------See LaunchPad.c for details of switches and LEDs--------------------------

#include <stdint.h>
#include<stdbool.h>
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/UART1.h"
#include "../inc/SSD1306.h"
#include "../inc/Motor.h"
//#include "../inc/Motor.c"
#include "msp.h"

#define LA 13635
#define LAS 12870
#define LB 12148
#define LC 11466
#define LCS 10822
#define LD 10215
#define LDS 9641
#define LE 9100
#define LF 8589
#define LFS 8107
#define LG 7652
#define LGS 7223
#define MA 6817
#define MAS 6435
#define MB 6073
#define MC 5732
#define MCS 5411
#define MD 5107
#define MDS 4820
#define ME 4550
#define MF 4294
#define MFS 4053
#define MG 3826
#define MGS 3611
#define HA 3408

#define quarter 396 * 2
#define half quarter * 2

uint8_t  BT_ByteData;      // 8-bit user data from the phone
uint16_t wasInterrupt = 0;
volatile uint16_t vcommand;

 //Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
//Interrupt on falling edge (on touch)
void FlameInt_Init(void)
{
    // write this as part of Lab 5
P6DIR &= ~BIT2;
P6REN |= BIT2;
P6OUT |= BIT2;

P6DIR |= BIT3;
P6OUT &= ~BIT3;

P6IE |= BIT2;
P6IES &= ~BIT2;
P6IFG &= ~BIT2;
NVIC -> ISER[1] = 0x100;

}

// triggered on touch, falling edge
void PORT6_IRQHandler(void){
   wasInterrupt = 1;
   P6IFG &= 0x00;
}
// ********OutValue**********
// Debugging dump of a data value to virtual serial port to PC
// data shown as 1 to 8 hexadecimal characters
// Inputs:  response (number returned by last AP call)
// Outputs: none
void ValueOut(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}

void playNote(uint16_t note, uint16_t length){
    TA1CTL |= 0x0010;
    TA1CCR0 = note;
    TA1CCR1 = note / 2;
    Clock_Delay1ms(length);
    TA1CTL &= ~0x0010;
    Clock_Delay1ms(50);
}

void MoveRobot (uint16_t command) {
// this function calls the appropriate functions to stop, move forward, move backward, turn right,
// or turn left according to the command received from the BLE
    if(command == 1)
         {
        vcommand = 1;
           SSD1306_Clear();
                         SSD1306_SetCursor(0,0);
                         SSD1306_OutString("First Floor");
                         SSD1306_OutChar(CR);
           playNote(LF,quarter);
           P2OUT &= ~0x07;
           P2OUT |= 0x01;
           Clock_Delay1ms(200);
         }
         if(command == 2)
         {
             vcommand = 2;
           SSD1306_Clear();
             SSD1306_SetCursor(0,0);
             SSD1306_OutString("Second Floor");
             SSD1306_OutChar(CR);
           playNote(LF,quarter);
           playNote(LF,quarter);
           P2OUT &= ~0x07;
           P2OUT |= 0x02;
           Clock_Delay1ms(200);
         }
         if(command == 3)
         {
             vcommand = 3;
           SSD1306_Clear();
             SSD1306_SetCursor(0,0);
             SSD1306_OutString("Third Floor");
             SSD1306_OutChar(CR);
           playNote(LF,quarter);
           playNote(LF,quarter);
           playNote(LF,quarter);
           P2OUT &= ~0x07;
           P2OUT |= 0x04;
           Clock_Delay1ms(200);
         }
         if(command == 4)
         {
             vcommand = 4;
           SSD1306_Clear();
             SSD1306_SetCursor(0,0);
             SSD1306_OutString("Ground Floor");
             SSD1306_OutChar(CR);
           playNote(LD,quarter);
           P2OUT &= ~0x07;
           P2OUT |= 0x05;
           Clock_Delay1ms(200);
         }
         if(command == 0)
         {
             vcommand = 0;
           Clock_Delay1ms(200);
         }
         return vcommand;
}




void WriteByteData(void){ // called on a SNP Characteristic Write Indication on characteristic ByteData
  MoveRobot(BT_ByteData);   // send command to robot
  ValueOut("\n\rWrite BLE_ByteData=",BT_ByteData);
}


int main(void){
  volatile int r;

  DisableInterrupts();
  Clock_Init48MHz();
  UART0_Init();
  TimerInit();
  MotorInit();
  FlameInt_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  EnableInterrupts();
  UART0_OutString("\n\rApplication Processor - MSP432-CC2650\n\r");
  r = AP_Init();
  AP_GetStatus();  // optional
  AP_GetVersion(); // optional
  AP_AddService(0xFFF0);
  //------------------------
  BT_ByteData = 0;  // write parameter from the phone will be used to control command
  AP_AddCharacteristic(0xFFF1,1,&BT_ByteData,0x02,0x08,"commandData",0,&WriteByteData);

  //------------------------

  AP_RegisterService();
  AP_StartAdvertisementJacki();
  AP_GetStatus(); // optional
  enum motor_states {First, Second, Third, Ground, OFF} state,prevState;//start state
   state = Ground;
   prevState != OFF;               //used to know when the state has changed
   volatile uint16_t stateTimer = 0;       //used to stay in a state
   bool isNewState = true;           //true when the state has switched
  while(1){

    AP_BackgroundProcess();  // handle incoming SNP frames
    volatile uint16_t prevcommand;
    if (vcommand != prevcommand)
    {
        prevcommand = vcommand;
        if(vcommand == 1)
        {
            state = First;

        }
        if(vcommand == 2)
        {
            state = Second;

        }
        if(vcommand == 3)
        {
            state = Third;

        }
        if(vcommand == 4)
        {
            state = Ground;

        }
        if(vcommand == 0)
        {
            state = OFF;
        }

    }


    // then if command == 3
    // then state = ?
    isNewState = (state != prevState);
               prevState = state;
    switch (state) {

                case First:
                    if(stateTimer < 10)
                     {
                       stateTimer++;
                       Motor_Forward(10999,10999);
                          // playNote(LE,quarter),playNote(LG,quarter),playNote(LB,quarter);
                          // playNote(MF,quarter);
                          // playNote(ME, quarter/2);

                     }
                    else if(stateTimer == 10)
                    {
                        MotorStop();
                    }
                    else
                    {
                        stateTimer--;
                        Motor_Backward(10999,10999);
                    }
                      break;
                case Second:
                   if(stateTimer < 20)
                    {
                      stateTimer++;
                      Motor_Forward(10999,10999);
                    }
                   else if(stateTimer == 20)
                   {
                       MotorStop();
                   }
                   else
                   {
                       stateTimer--;
                       Motor_Backward(10999,10999);
                   }
                    break;
                case Third:
                    if(stateTimer < 30)
                     {
                       stateTimer++;
                      Motor_Forward(10999,10999);
                     }
                    else if(stateTimer == 30)
                    {
                        MotorStop();
                    }
                    else
                    {
                        stateTimer--;
                        Motor_Backward(10999,10999);
                    }
                    break;
                case Ground:
                    if(stateTimer == 0)
                    {
                        MotorStop();
                    }
                    else if(stateTimer > 0)
                    {
                        stateTimer--;
                        Motor_Backward(10999,10999);
                    }
                        break;


    default: state = OFF;
    }//switch
    if(1 == wasInterrupt)
    {
        SSD1306_Clear();
          SSD1306_SetCursor(0,0);
          SSD1306_OutString("FIRE!!!");
          SSD1306_OutChar(CR);

        wasInterrupt = 0;
        state = Ground;
        playNote(LA,quarter);
        playNote(LA,quarter);



    }
    Clock_Delay1ms(100);
  }    //while(1)
}
