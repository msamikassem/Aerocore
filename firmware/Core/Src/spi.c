#include "stm32f4xx.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>

#define CR1_Maseter	(1U<<2)
#define CR1_SSM		(1U<<9)
#define CR1_SSI		(1U<<8)
#define CR1_SPE     (1u<<6)
#define SR_TXE		(1U<<1)
#define SR_BUSY		(1U<<7)
#define SR_RXNE		(1U<<0)



void spi_init(uint8_t prescaler, bool clk_idle_low, bool first_edge_capture, bool MSB_First, bool eight_bit_frame)
{

	//Provide CLK to SPI1 peripheral (through AHB1)
	RCC->APB2ENR |= (1U<<12);

	//set Baud rate control (prescaler) to the chosen value
	SPI1->CR1 |= (prescaler<<3);

	//Set the Clock polarity (Default is HIGH/LOW)
	SPI1->CR1 &= ~(1U<<1); //set to zero (reset)
	if (!clk_idle_low)
	{
		SPI1->CR1 |= (1U<<1); //1 when idle
	}



	//Set the Clock Phase (capture edge)
	SPI1->CR1 &= ~(1U<<0);
	if (!first_edge_capture)
	{
		SPI1->CR1 |= (1U<<0);
	}


	//Set LSBFIRST (LSB/MSB as first bit during transmission)
	if (!MSB_First)
	{
		SPI1->CR1 |= (1U<<7);
	}


	//Set Data frame format (DFF) to 8/16 bit
	if (!eight_bit_frame)
	{
		SPI1->CR1 |= (1U<<11);
	}

	//Set MCU to Master
		SPI1->CR1 |= CR1_Maseter;

	//set communication to Full-duplex
	SPI1->CR1 &= ~(1U<<10);

	//Force Masters NSS pin to HIGH
		//Enable Software slave management (SSM)
	SPI1->CR1 |= CR1_SSM;
		//Select HIGH as NSS value (SSI)
	SPI1->CR1 |= CR1_SSI;

	//Enable SPI
	SPI1->CR1 |= CR1_SPE;

}

void spi_transmit(uint8_t *data, uint32_t size)
{
	uint8_t temp;

	 for (uint8_t i = 0; i < size; i++)
	{
		//wait until TXE (Data transfered) is set
		 while (!(SPI1->SR & SR_TXE)){}

		 //Place the contents of the data address into Data Register to be transmitted
		 SPI1->DR = *data++;
	}

	 //wait for BUSY flag to reset (Communication is done)
	 while (SPI1->SR & SR_BUSY) {}
	 //Clear OVR (for later use by receive function)
	 //OVR indicates an over write occurred
	 temp = SPI1->DR;
	 temp = SPI1->SR;
}

void spi_receive(uint8_t *data,uint32_t size)
{
	while(size)
	{
		//Transmit signal to receive signal
		SPI1->DR=0;

		//wait for RXNE flag (Data arrived) to be set
		while (!(SPI1->SR & SR_RXNE)){}

		//Read data from data register and save it at data address
		*data++ = (SPI1->DR);
		size--;
	}
}




