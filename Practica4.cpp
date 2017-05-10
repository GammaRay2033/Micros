#include <mraa.h>
#include <stdio.h>

int main(void)
{
int opc;

float temperature;
float hummidity;
float light;
uint8_t temp[3];
uint8_t hum[3];
uint8_t setup[2];

mraa_i2c_context i2c;
mraa_init();
i2c = mraa_i2c_init(0);
mraa_i2c_address(i2c,0x40);
	
while(1){
	printf("Seleccionar opciones \n\n");
	printf("1. Sensor de Temperatura \n");
	printf("2. Sensor de Humedad \n");
	printf("3. Sensor de Luminisidad \n\n");

	printf("Introduzca su opcion: ");
	scanf("%d", &opc);
	
	switch(opc){

		case 1: for(int i=1;i<10;i++){
			mraa_i2c_address(i2c,0x40);
			setup[0] = 0x03;
			setup[1] = 0x11;
			mraa_i2c_write(i2c,setup,2);
			usleep(1);
			do{
				mraa_i2c_read_bytes_data(i2c,0x00,temp,3);
			}while(temp[0]==0);
			temperature=((temp[1]<<6)|(temp[2>>2]))/32.0-50.0;
			temp[0]=0x00;
			temp[1]=0x00;
			temp[2]=0x00;
			sleep(0.5);
			}
			printf("\nTemperatura = %.2f grados centrigrados \n\n", temperature);
			break;

		case 2: for(int i=1;i<10;i++){
			mraa_i2c_address(i2c,0x40);
			setup[0] = 0x03;
			setup[1] = 0x01;
			mraa_i2c_write(i2c,setup,2);
			usleep(1);
			do{
				mraa_i2c_read_bytes_data(i2c,0x00,hum,3);
			}while(hum[0]==0);
			hummidity=(((hum[1]<<4)|(hum[2]>>4))/16.0)-24.0;
			hum[0]=0x00;
			hum[1]=0x00;
			hum[2]=0x00;
			sleep(0.5);
		        }
		        printf("\nHumedad relativa = %.2f%% \n\n", hummidity);
			break;

		case 3: mraa_aio_context adc_a0;
			adc_a0 = mraa_aio_init(0);
			if(adc_a0 == NULL){
				return 1;
			}
			light = mraa_aio_read_float(adc_a0)*100.0;
			printf("\nNivel de luz = %.2f%% \n\n",light);
			mraa_aio_close(adc_a0);
			break;
		
		default: printf("\nOpcion invalida\n\n");
			 break;

	}
}
return 0;
}

