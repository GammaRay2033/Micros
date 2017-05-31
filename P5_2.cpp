#include <mraa.h>
#include <stdio.h>
#include <cstdlib>
#include "kbhit.h"

#define LCD 0x3E
#define RGB 0x62

void initLCD();
void initRGB();
void setRGB(int R, int G, int B);
void writeLCD(char str);
void clrLCD();

char c='a';
char str[5];
int count=0;
int value = false;
uint8_t rx_tx[2];
mraa_i2c_context i2c;
mraa_gpio_context Tout;
mraa_gpio_context PBEvent;

int main(void){
  mraa_init();
  i2c = mraa_i2c_init(0);
  Tout = mraa_gpio_init(13);
  PBEvent = mraa_gpio_init(5);
  mraa_gpio_dir(Tout, MRAA_GPIO_OUT);
  mraa_gpio_dir(PBEvent, MRAA_GPIO_IN);

  initLCD();
  initRGB();
  setRGB(255,0,0);
	
  puts("Press 's' to start the counter and 'r' to reset");
  sprintf(str, " %i ", count);
  puts(str);
  clrLCD();
  writeLCD(str[0]);
  writeLCD(str[1]);
  writeLCD(str[2]);
	
  while(true){
    mraa_gpio_write(Tout,0);
    while((c!='s')&&(c!='r')&&(c!='x')){
      if(kbhit()){
        c=readch();
      }
    }
	  
    do{
      c='a';
      while(!value){
        value = mraa_gpio_read(PBEvent);
	if(kbhit()){
	  c=readch();
	    if((c=='r')||(c=='x')){
	      break;
	    }
	}
      }
      value = false;
      count++;
      usleep(150000);
      if(kbhit()){
        c=readch();
      }
      system("reset");
      sprintf(str, " %i ", count);
      puts(str);
      clrLCD();
      writeLCD(str[0]);
      writeLCD(str[1]);
      writeLCD(str[2]);
    }while((c!='r')&&(count<=9)&&(c!='x'));
	  
    if(c=='r'){
      mraa_gpio_write(Tout,0);
      count = 0;
      system("reset");
      sprintf(str, " %i ", count);
      puts(str);
      clrLCD();
      writeLCD(str[0]);
      writeLCD(str[1]);
      writeLCD(str[2]);
    }
	  
    if(count>=10){
      mraa_gpio_write(Tout,1);
      count = 0;
      c='a';
      do{    
        if(kbhit()){
           c=readch();
        }
      }while(c!='r');
      system("reset");
      sprintf(str, " %i ", count);
      puts(str);
      clrLCD();
      writeLCD(str[0]);
      writeLCD(str[1]);
      writeLCD(str[2]);
      c='a';
    }
    
    if(c=='x'){
      return 0;
    }
  }
return 0;
}

void initLCD(){
  mraa_i2c_address(i2c,LCD);
  rx_tx[0] = 0x80;
  rx_tx[1] = 0x3C;
  mraa_i2c_write(i2c,rx_tx,2);
  usleep(4000);
  mraa_i2c_write(i2c,rx_tx,2);
  usleep(150);
  mraa_i2c_write(i2c,rx_tx,2);
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[1] = 0x0E;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[1] = 0x01;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[1] = 0x06;
  mraa_i2c_write(i2c,rx_tx,2);
}

void initRGB(){
  mraa_i2c_address(i2c,RGB);
  rx_tx[0] = 0x00;
  rx_tx[1] = 0x00;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[0] = 0x08;
  rx_tx[1] = 0xFF;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[0] = 0x01;
  rx_tx[1] = 0x00;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[0] = 0x04;
  rx_tx[1] = 0xFF;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[0] = 0x03;
  rx_tx[1] = 0xFF;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[0] = 0x02;
  rx_tx[1] = 0xFF;
  mraa_i2c_write(i2c,rx_tx,2);
}

void writeLCD(char str){
  mraa_i2c_address(i2c,LCD);
  rx_tx[0] = 0x40;
  rx_tx[1] = str;
  mraa_i2c_write(i2c,rx_tx,2);
  usleep(1000);
}

void clrLCD(){
  mraa_i2c_address(i2c,LCD);
  rx_tx[0] = 0x80;
  rx_tx[1] = 0x01;
  mraa_i2c_write(i2c,rx_tx,2);
}

void setRGB(int R, int G, int B){
  mraa_i2c_address(i2c,RGB);
  rx_tx[0] = 0x04;
  rx_tx[1] = R;
  mraa_i2c_write(i2c,rx_tx,2);    
  rx_tx[0] = 0x03;
  rx_tx[1] = G;
  mraa_i2c_write(i2c,rx_tx,2);
  rx_tx[0] = 0x02;
  rx_tx[1] = B;
  mraa_i2c_write(i2c,rx_tx,2);
}
