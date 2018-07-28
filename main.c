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

/************ CONSTANTS AND DEFINES ****************/




#define SLAVE_ADDRESS 0x51

void InitConsole(void);     //Initializes and configures UART Communication
void InitI2C(void);         //Initialize and configure i2c

void BCDtoDEC_sec(int );       //BCD to Decimal conversion of sec
void BCDtoDEC_min(int );       //BCD to Decimal conversion of min
void BCDtoDEC_hrs(int );       //BCD to Decimal conversion of hours
void BCDtoDEC_days(int );       //BCD to Decimal conversion of days
void BCDtoDEC_weekdays(int );       //BCD to Decimal conversion of weekdays
void BCDtoDEC_cent_month(int );       //BCD to Decimal conversion of cent_month
void BCDtoDEC_yrs(int );       //BCD to Decimal conversion of year

unsigned long DataRx[7];

int sec=0;
int min=0;
int hrs=0;
int days=0;
int weekdays=0;
int cent=0;
int month=0;
int year=0;

int main(void) {

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);   //Configure System Clock
    InitConsole();                         //initialize uart0 for display
    InitI2C();                             //initialize and configure I2C0


while(1){

    int index=0;                            //index for data array

    label:

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    I2CMasterDataPut(I2C0_BASE, 0x02);      // Send 0x02 (Register Address) to the RTC
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE))
                      {
                      }

    while(index<7){

    if(index==0){
                  int old_sec=DataRx[0];
                  I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);   //Send the Slave Address in Read Mode
                  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE );
                  while(I2CMasterBusy(I2C0_BASE))
                      {
                      }
                  DataRx[index] = I2CMasterDataGet(I2C0_BASE);   // Read the data

                  while(old_sec==DataRx[0])      //if sec does not change goto "label" (i.e. wait for 1 sec)
                   {goto label;}                 //using this method, all variables every second

                  index++;
                 }

    else if(index<6){

                   I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
                   while(I2CMasterBusy(I2C0_BASE))
                         {
                         }
                    DataRx[index] = I2CMasterDataGet(I2C0_BASE);   // Read the data
                    index++;
                     }

    else if(index==6)
                    {

                   I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
                   while(I2CMasterBusy(I2C0_BASE))
                         {
                         }
                   DataRx[index] = I2CMasterDataGet(I2C0_BASE);   // Read the data
                    index++;
                     }



           }

     BCDtoDEC_sec(DataRx[0]);
     BCDtoDEC_min(DataRx[1] );
     BCDtoDEC_hrs( DataRx[2]);
     BCDtoDEC_days(DataRx[3] );
     BCDtoDEC_weekdays(DataRx[4] );
     BCDtoDEC_cent_month(DataRx[5]);
     BCDtoDEC_yrs(DataRx[6]);

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
    //SysCtlDelay(18000000);

       }



  }


void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);            //Enable PORT A Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);              //Enable UART0 Module

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);      //Configure PA0 and PA1 as UART Pins
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);       //Configure UART0 Module clock source as the Precision Internal OSCillator
    UARTStdioConfig(0, 115200, 16000000);                   //Configure the UART0 Module (115200 ; CLock : 16MHz)
}

void InitI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);     //Enable I2C0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    //Enable GPIO B Module

    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);   //Initialize PB2 and PB3 as I2C Pins
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE,GPIO_PIN_2);  //Initialize PB2 as SCL

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

