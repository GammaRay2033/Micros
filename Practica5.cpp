#include <mraa.h>
#include <stdio.h>
#include <cstdlib>

#define LCD 0x3E
#define RGB 0x62

char c;
uint8_t rx_tx[2];
mraa_i2c_context i2c;

int main(void){
  mraa_init();
  i2c = mraa_i2c_init(0);

  initLCD();
  initRGB();
  setRGB(255,255,255);
  

  puts("Enter text '.' to exit");

  do{
    system("/bin/stty raw");
    c=getchar();
    system("/bin/stty -raw");
  }while(c!='s');
  printf("\n");
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

