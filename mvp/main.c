#include "lib/stm32f0xx_rcc.h"
#include "lib/stm32f0xx_gpio.h"

#include <stdbool.h>

#define STACK_TOP 0x20000800

//Declarations
void nmi_handler(void);
void hardfault_handler(void);
int main(void);

// Define the vector table
unsigned int * myvectors[4]
   __attribute__ ((section("vectors")))= {
   	(unsigned int *)	STACK_TOP,	// stack pointer
   	(unsigned int *) 	main,		// code entry point
   	(unsigned int *)	nmi_handler,		// NMI handler (not really)
   	(unsigned int *)	hardfault_handler		// hard fault handler (let's hope not)
};

void assert_param(bool x)
{
}

int main(void)
{
    while(1);
}
void nmi_handler(void)
{
	return ;
}

void hardfault_handler(void)
{
	return ;
}
