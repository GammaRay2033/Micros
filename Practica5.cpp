#include <mraa.h>
#include <stdio.h>
#include <cstdlib>

int main(void){
	char c;
	puts("Enter text '.' to exit");
	do{
		system("/bin/stty raw");
		c=getchar();
		system("/bin/stty -raw");
		sleep(1);
	}while(c!='.');
	printf("\n");
	return 0;
}

