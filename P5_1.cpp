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

char c;
char str[5];
float timer=5.0;
int value = false;
uint8_t rx_tx[2];
mraa_i2c_context i2c;
mraa_gpio_context Tout;
mraa_gpio_context PBStart;

int main(void){
  mraa_init();
  i2c = mraa_i2c_init(0);
  Tout = mraa_gpio_init(13);
  PBStart = mraa_gpio_init(5);
  mraa_gpio_dir(Tout, MRAA_GPIO_OUT);
  mraa_gpio_dir(PBStart, MRAA_GPIO_IN);

  initLCD();
  initRGB();
  setRGB(255,0,0);

  while(true){
    mraa_gpio_write(Tout,0);
    while((!value)||(c!='x')){
      value = mraa_gpio_read(PBStart);
      if(kbhit()){
        c=readch();
      }
    }
    puts("Press 's' to stop the count and 'r' to reset");
    do{
      sprintf(str, " %0.2f", timer);
      clrLCD();
      writeLCD(str[0]);
      writeLCD(str[1]);
      writeLCD(str[2]);
      writeLCD(str[3]);
      sleep(1.0);
      timer -= 1.0;
      if(kbhit()){
        c=readch();
      }
    }while((c!='s')&&(c!='r')&&(timer>=0)&&(c!='x'));
    system("reset");
    if(timer<=0){
      mraa_gpio_write(Tout,1);
      timer = 5.0;
      do{
        if(kbhit()){
          c=readch();
	}
      }while((c!='r')&&(c!='x'));
    }
    if(c=='s'){
      c='a';
      do{
        if(kbhit()){
          c=readch();
        }
      }while((c!='s')&&(c!='x'));
    }
    if(c=='r'){
      mraa_gpio_write(Tout,0);
      timer = 5.0;
    }
    if(c=='x'){
      return 0;
    }
    c='a';
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
