#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.c"
#include "utils/uartstdio.c"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "driverlib/systick.h"

/************ SEVEN SEGMENT DISPLAY (SSD)****************/

int ssdnum[16]={0b11111100,0b01100000,0b11011010,0b11110010,                      //binary equivalent of SSD
                0b01100110,0b10110110,0b10111110,0b11100000,                     //binary equivalent of SSD
                0b11111110,0b11110110,0b11101110,0b00111110,                    //binary equivalent of SSD        (0 TO F)
                0b10011100,0b01111010,0b10011110,0b10001110};                  //binary equivalent of SSD


uint32_t ssdDig[4]={GPIO_PIN_7,GPIO_PIN_6,GPIO_PIN_5,GPIO_PIN_4};                // for SSD digit selection


/************ CONSTANTS AND DEFINES ****************/




#define SLAVE_ADDRESS 0x51

void InitConsole(void);                                                       //Initializes and configures UART Communication
void InitI2C(void);                                                          //Initialize and configure i2c
void SSDInit(void);                                                         //Initialize and configure ports for SSD and switches
void InterruptInit(void);

void Display(void);                                                              //SSD display
void UARTDisplay(void);
void Setting_Display(void) ;                                                    //display during setting

void TotalRead(void);                                                         //complete read operation
void TotalWrite(void);                                                       //complete write operation
void TotalWriteAlarm(void);                                  //write alarm

void time_set(void);                                        //time set ISR (incrementer also)
void alarm_set(void);                                      //alarm set ISR



void BCDtoDEC_sec(int );                            //BCD to Decimal conversion of sec      {can be done using a single function also}
void BCDtoDEC_min(int );                            //BCD to Decimal conversion of min
void BCDtoDEC_hrs(int );                           //BCD to Decimal conversion of hours
void BCDtoDEC_days(int );                         //BCD to Decimal conversion of days
void BCDtoDEC_weekdays(int );                    //BCD to Decimal conversion of weekdays
void BCDtoDEC_cent_month(int );                 //BCD to Decimal conversion of cent_month
void BCDtoDEC_yrs(int );                       //BCD to Decimal conversion of year

unsigned long DataRx[7]={0,0,0,0,0,0,0};              //received data
unsigned long DataTx[4]={0,0,0,0};                   //setting data to be send
unsigned long DataTx_al[5]={0,0,0,0,0};              //setting alarm

int sel=0;                                                                   //selecting particular SSD
int temp[4]={0,0,0,0};                                                      //stores the value to be printed
int temp_set[4]={0,0,0,0};                                                 // for setting_dispaly()
int CountSwitch1=0;                                                       //no of time sw1 is pressed (shifting the SSD)
int CountSwitch3=0;

int hrs_set=0;                                                            //used in time set
int min_set=0;                                                           //

int alarm_min_set=0;
int alarm_hrs_set=0;



int sec=0;
int min=0;
int hrs=0;
int days=0;
int weekdays=0;
int cent=0;
int month=0;
int year=0;


/************ MAIN *************************************************************/

 int main(void) {

    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //Configuring system clock to 16MHz

    InitConsole();                                                                        //initialize uart0 for display
    InitI2C();                                                                           //initialize and configure I2C0
    SSDInit();                                                                          //SSD initialize
    InterruptInit();


    IntEnable(INT_GPIOF);
    IntEnable(INT_GPIOB);
    IntMasterEnable();


while(1){

    switch(sel){
    case  0:
                CountSwitch1=0;                                                  //reset switch press counter here
                CountSwitch3=0;

                TotalRead();


                BCDtoDEC_sec(DataRx[0]);                                        //decode after receiving
                BCDtoDEC_min(DataRx[1] );
                BCDtoDEC_hrs( DataRx[2]);
                BCDtoDEC_days(DataRx[3] );
                BCDtoDEC_weekdays(DataRx[4] );
                BCDtoDEC_cent_month(DataRx[5]);
                BCDtoDEC_yrs(DataRx[6]);

                temp[0]=min%10;                                               //first digit
                temp[1]=min/10;                                              //second digit
                temp[2]=hrs%10;                                             //third digit
                temp[3]=hrs/10;                                            //fourth digit
                Display();                                                //SSD display
               break;
    case  1:
               //SetTime();
               Setting_Display();

               break;
    default:
               CountSwitch1=0;                                     //reset switch press counter here
               CountSwitch3=0;

               TotalRead();

               BCDtoDEC_sec(DataRx[0]);
               BCDtoDEC_min(DataRx[1] );
               BCDtoDEC_hrs( DataRx[2]);
               BCDtoDEC_days(DataRx[3] );
               BCDtoDEC_weekdays(DataRx[4] );
               BCDtoDEC_cent_month(DataRx[5]);
               BCDtoDEC_yrs(DataRx[6]);

               temp[0]=min%10;
               temp[1]=min/10;
               temp[2]=hrs%10;
               temp[3]=hrs/10;

               Display();                                          //SSD display
                }

             UARTDisplay();
     }

return 0;

  }

void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                                        //Enable PORT A Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);                                       //Enable UART0 Module

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);                     //Configure PA0 and PA1 as UART Pins
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);                             //Configure UART0 Module clock source
    UARTStdioConfig(0, 115200, 16000000);                                        //Configure the UART0 Module (115200 ; CLock : 16MHz)
}

void InitI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);                                    //Enable I2C0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                                  //Enable GPIO B Module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_3 |GPIO_PIN_2);                      //Initialize PB2 and PB3 as I2C Pins
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE,GPIO_PIN_2);                              //Initialize PB2 as SCL

}

void BCDtoDEC_sec(int data)
{
  int unit=data & 0x0F;
  int tens=(data>>4) & 0x07;

  sec= (tens*10)+unit;

}

void BCDtoDEC_min(int data)
{
  int unit=data & 0x0F;
  int tens=(data>>4) & 0x07;

  min= (tens*10)+unit;

}

void BCDtoDEC_hrs(int data)
{
  int unit=data & 0x0F;
  int tens=(data>>4) & 0x03;

  hrs= (tens*10)+unit;

}

void BCDtoDEC_days(int data)
{
  int unit=data & 0x0F;
  int tens=(data>>4) & 0x03;

  days= (tens*10)+unit;

}

void BCDtoDEC_weekdays(int data)
{
  int unit=data & 0x07;


  weekdays= unit;

}

void BCDtoDEC_cent_month(int data)
{
  int unit=data & 0x0F;
  int tens=(data>>4) & 0x01;

  month= (tens*10)+unit;

}

void BCDtoDEC_yrs(int data)
{
  int unit=data & 0x0F;
  int tens=(data>>4) & 0x07;

  year= (tens*10)+unit;

}

int DECtoBCD(int temp1,int temp2)
{
return (temp2<<4)+temp1;
}

//
void SSDInit()
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);


    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,0x0F);                                 //PORT D as output (tiva c to SSD)
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,0xF0);                                //PORT C as output (SSD digit selection)
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,0xF0);                               //PORT A as output (tiva c to SSD)

}

void InterruptInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                           //rest ports are already enabled

/************ switch3 ****************and ************ alarm interrupt pin ****************/

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);        //  alarm interrupt

    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);       //pull up config. for switch3(ALARM)
    GPIOIntRegister(GPIO_PORTB_BASE,alarm_set);                                                //time set switch1

    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_5);
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_4);                                                  //previous interrupt clear
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);                             //configuring interrupt
    GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_4);                                               //interrupt enabled



    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_LOW_LEVEL);                             //configuring interrupt
    GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_5);


    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_6);                                    //alarm buzzer


/************ switch1 and switch2 ****************/

    HWREG(GPIO_PORTF_BASE|GPIO_O_LOCK)=GPIO_LOCK_KEY;                                    //Unlocking the user switch 2
    HWREG(GPIO_PORTF_BASE|GPIO_O_CR)=GPIO_PIN_0;                                        // to increment digits
    HWREG(GPIO_PORTF_BASE|GPIO_O_LOCK)=0;                                              //

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);                  //LEDs output
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_3);                         //green led initially

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);                               //switches as input
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);      //pull up config. for switch1
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);      //pull up config. for switch2

    GPIOIntRegister(GPIO_PORTF_BASE,time_set);                                                 //time set switch1
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4);                                                 //previous interrupt clear


    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0);                                                    //previous interrupt clear

    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);                               //configuring interrupt
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4);                                                 //interrupt enabled

    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);                             //configuring interrupt
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_0);                                               //interrupt enabled


}

/************ DISPLAY ON SSD ****************///(MULTIPLEXED)
void Display()
{
    int i;
    for(i=0;i<4;i++)
       {
           if(i==2){                                                //displaying point also
           GPIOPinWrite(GPIO_PORTD_BASE,0X0F,ssdnum[temp[i]]+1);
           GPIOPinWrite(GPIO_PORTA_BASE,0XF0,ssdnum[temp[i]]+1);
            }
           else{
               GPIOPinWrite(GPIO_PORTD_BASE,0X0F,ssdnum[temp[i]]);
               GPIOPinWrite(GPIO_PORTA_BASE,0XF0,ssdnum[temp[i]]);
              }
           GPIOPinWrite(GPIO_PORTC_BASE,0XF0,ssdDig[i]);
           SysCtlDelay(15000);

           GPIOPinWrite(GPIO_PORTC_BASE,0XF0,0);

        }

}

void Setting_Display()                                             //another display just to avoid to much interdependencies
{

    int i;
    for(i=0;i<4;i++)
       {
           if(i==2){                                                //displaying point also
           GPIOPinWrite(GPIO_PORTD_BASE,0X0F,ssdnum[temp_set[i]]+1);
           GPIOPinWrite(GPIO_PORTA_BASE,0XF0,ssdnum[temp_set[i]]+1);
            }
           else{
               GPIOPinWrite(GPIO_PORTD_BASE,0X0F,ssdnum[temp_set[i]]);
               GPIOPinWrite(GPIO_PORTA_BASE,0XF0,ssdnum[temp_set[i]]);
              }
           GPIOPinWrite(GPIO_PORTC_BASE,0XF0,ssdDig[i]);

           if(CountSwitch1==i+1)                     //this digit will appear more bright(selected digit)
           SysCtlDelay(150000);
          else if(CountSwitch3==i+1)
           SysCtlDelay(150000);
           else
           SysCtlDelay(15000);

           GPIOPinWrite(GPIO_PORTC_BASE,0XF0,0);

        }

}

void UARTDisplay()
{
    if(hrs<10)  UARTprintf("0");
    UARTprintf("%d",hrs);
    UARTprintf(":");

    if(min<10)  UARTprintf("0");
    UARTprintf("%d",min);
    UARTprintf(":");

    if(sec<10)  UARTprintf("0");
    UARTprintf("%d",sec);
    UARTprintf("  ");

    if(days<10)  UARTprintf("0");
    UARTprintf("%d",days);
    UARTprintf("/");

    if(month<10)  UARTprintf("0");
    UARTprintf("%d",month);
    UARTprintf("/");

    if(year<10)  UARTprintf("0");
    UARTprintf("%d",year);
    UARTprintf("\n");

}

void TotalRead()
{
    int index=0;                                                   //index for data array


    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    while(I2CMasterBusBusy(I2C0_BASE))                        //used to see if bus is free (can be used to check if bus not working also)
    {}                                                       //
    I2CMasterDataPut(I2C0_BASE, 0x02);                       // Send 0x02 (Register Address) to the RTC
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE))
                      {
                      }

    while(index<7){

    if(index==0){

                  I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);                //Send the Slave Address in Read Mode

                  while(I2CMasterBusBusy(I2C0_BASE))                                  //used to see if bus is free
                     {}                                                              // if we remove a wire then tm4c will wait here
                                                                                    //(can be used to check if bus not working also)
                  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START );
                  while(I2CMasterBusy(I2C0_BASE))
                      {
                      }
                  DataRx[index] = I2CMasterDataGet(I2C0_BASE);   // Read the data
                  index++;
                 }

    else if(index<6){

                 I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);//can also use I2C_MASTER_CMD_SINGLE_RECEIVE on every read
                 while(I2CMasterBusy(I2C0_BASE))
                         {
                         }
                    DataRx[index] = I2CMasterDataGet(I2C0_BASE);   // Read the data
                    index++;
                     }

    else if(index==6)
                    {

                   I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                   while(I2CMasterBusy(I2C0_BASE))
                         {
                         }
                   DataRx[index] = I2CMasterDataGet(I2C0_BASE);   // Read the data
                    index++;
                     }
      }
}

/************(time setting and incrementer) ISR****************/

void time_set()
{
    uint32_t status=0;

    status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,status);


    if( (status & GPIO_INT_PIN_4) == GPIO_INT_PIN_4){
                                                                     //Then there was a pin4 interrupt
           int i;
           for(i=0;i<=5000;i++);                                   //Provide a small delay to provide for bouncing

              if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0)       //If the button is still pressed
                {

                    if(CountSwitch1==0)
                    {sel=1;                                        //sel=1 (setting mode)
                    CountSwitch1++;                               //count the switch press
                    temp_set[0]=0;
                    temp_set[1]=0;                               //set all digits to be displayed to "ZERO"
                    temp_set[2]=0;
                    temp_set[3]=0;
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_1);       //red LED on TM4C launchpad
                    }

             else if(CountSwitch1<4 && CountSwitch1!=0 )
                    CountSwitch1++;

             else if(CountSwitch1==4)
                    {CountSwitch1=0;
                     sel=0;
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_3);

                    min_set=DECtoBCD(temp_set[0],temp_set[1]);
                    hrs_set=DECtoBCD(temp_set[2],temp_set[3]);
                    TotalWrite();                                             //after decoding, write the time
                    }

                }
    }

    /************ incrementer pin ****************/

    if( (status & GPIO_INT_PIN_0) == GPIO_INT_PIN_0){
                                                                //Then there was a pin5 interrupt
        int i;
        for(i=0;i<=2000;i++);                                 //Provide a small delay to provide for bouncing

                  if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)==0)   //If the button is still pressed
                   {

                       if(CountSwitch1==1 ||CountSwitch3==1)            //if CountSwitch==1 then increment unit digit of min.
                        {
                           if(temp_set[0]<9)
                           {temp_set[0]++;}
                            else
                           {temp_set[0]=0;}
                         }

                       if(CountSwitch1==2||CountSwitch3==2)               //for tens digit
                        {
                           if(temp_set[1]<9)
                           {temp_set[1]++;}
                           else
                           {temp_set[1]=0;}
                         }

                       if(CountSwitch1==3||CountSwitch3==3)
                        {
                           if(temp_set[2]<9)
                           {temp_set[2]++;}
                           else
                           {temp_set[2]=0;}
                          }

                      if(CountSwitch1==4||CountSwitch3==4)
                       {
                           if(temp_set[3]<9)
                           {temp_set[3]++;}
                          else
                           {temp_set[3]=0;}
                       }


                   }

    }
}

/************write on slave (time to set)****************/

void TotalWrite()
{


    DataTx[0]=0x02;                       //address of "seconds" sent first to set address register
    DataTx[1]=0;
    DataTx[2]=min_set;
    DataTx[3]=hrs_set;


    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);    //Setup Clock to the I2C0 Module
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);     //Provide the Slave Address of the RTC in the Write Mode


    I2CMasterDataPut(I2C0_BASE, DataTx[0]);                             //Put the data to be sent in the buffer
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);      //Start sending
    while(I2CMasterBusy(I2C0_BASE))                                   //Wait until the transaction is complete
    {
    }


    int index;
    //Do the same for all the other data bytes
    for(index = 1; index <3 ; index++)
      {



           I2CMasterDataPut(I2C0_BASE, DataTx[index]);
           I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
           while(I2CMasterBusy(I2C0_BASE))
           {
         }

       }

    I2CMasterDataPut(I2C0_BASE, DataTx[3]);
       I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
       while(I2CMasterBusy(I2C0_BASE))
     {
     }

    I2CMasterDisable(I2C0_BASE);

}

/************alarm setting ISR and alarm interrupt from RTC handler****************/

void alarm_set()
{
    uint32_t status=0;

    status = GPIOIntStatus(GPIO_PORTB_BASE,true);
    GPIOIntClear(GPIO_PORTB_BASE,status);


    if( (status & GPIO_INT_PIN_4) == GPIO_INT_PIN_4){
      //Then there was a pin4 interrupt
        int i;
        for(i=0;i<=10000;i++);       //Provide a small delay to provide for bouncing

           if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4)==0) //If the button is still pressed
             {

                 if(CountSwitch3==0)
                 {sel=1;

                 GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);         //stop the buzzer on press of alarm set switch

                 CountSwitch3++;
                 temp_set[0]=0;
                 temp_set[1]=0;
                 temp_set[2]=0;
                 temp_set[3]=0;
                 GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_1);
                 }

          else if(CountSwitch3<4 && CountSwitch3!=0 )
                 CountSwitch3++;

          else if(CountSwitch3==4)
                 {CountSwitch3=0;
                  sel=0;
                 GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_3);
                 alarm_min_set=DECtoBCD(temp_set[0],temp_set[1]);
                 alarm_hrs_set=DECtoBCD(temp_set[2],temp_set[3]);
                 TotalWriteAlarm();
                   }

             }
       }


 //ISR OF alarm interrupt from RTC

    if( (status & GPIO_INT_PIN_5) == GPIO_INT_PIN_5){
                        //Then there was a pin5 interrupt

           if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5)==0)               //alarm pin level (LOW)
             {
               unsigned long control=0x01;                                //address of control register2 in RTC
               unsigned long clear=0x06;                                  //data for clearing the interrupt flag

               GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);        //start the buzzer

               I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);      //Setup Clock to the I2C0 Module
               I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);      //Provide the Slave Address of the RTC in the Write Mode


               I2CMasterDataPut(I2C0_BASE,control );                            //Put the data to be sent in the buffer
               I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);   //Start sending
               while(I2CMasterBusy(I2C0_BASE))                                //Wait until the transaction is complete
                    {
                    }
                I2CMasterDataPut(I2C0_BASE,clear );                                  //Put the data to be sent in the buffer
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);      //Start sending
                while(I2CMasterBusy(I2C0_BASE))                                    //Wait until the transaction is complete
                                   {
                                   }

               I2CMasterDisable(I2C0_BASE);

             }
       }

    }

/************write on slave (alarm to set)****************/

void TotalWriteAlarm()
{


        // Initialize the array to be sent
        DataTx_al[0]=0x09;                                          //min_alarm register address
        DataTx_al[1]=alarm_min_set & 0b01111111;                    //min_alarm
        DataTx_al[2]=alarm_hrs_set & 0b01111111;                    //hrs_alarm

        DataTx_al[3]=0x01;                                         //control register 2 address
        DataTx_al[4]=0x0E;                                         //data to activate the alarm interrupt

        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);    //Setup Clock to the I2C0 Module
        I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);     //Provide the Slave Address of the RTC in the Write Mode


        I2CMasterDataPut(I2C0_BASE, DataTx_al[0]);                         //Put the data to be sent in the buffer
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);     //Start sending
        while(I2CMasterBusy(I2C0_BASE))                                  //Wait until the transaction is complete
            {
            }
        I2CMasterDataPut(I2C0_BASE, DataTx_al[1]);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(I2C0_BASE))
            {
            }
         I2CMasterDataPut(I2C0_BASE, DataTx_al[2]);
         I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
         while(I2CMasterBusy(I2C0_BASE))
           {
           }

         I2CMasterDataPut(I2C0_BASE, DataTx_al[3]);                            //Put the data to be sent in the buffer
         I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);        //Start sending
         while(I2CMasterBusy(I2C0_BASE))                                     //Wait until the transaction is complete
              {
              }

         I2CMasterDataPut(I2C0_BASE, DataTx_al[4]);                                   //Put the data to be sent in the buffer
         I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);               //Start sending
         while(I2CMasterBusy(I2C0_BASE))                                             //Wait until the transaction is complete
               {
               }
        I2CMasterDisable(I2C0_BASE);



}

