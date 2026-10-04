
#ifndef INC_SPI_H_
#define INC_SPI_H_
#include <stdbool.h>
#include <stdint.h>

void spi_init(uint8_t prescaler, bool clk_idle_low, bool first_edge_capture, bool MSB_First, bool eight_bit_frame);
void spi_transmit(uint8_t *data, uint32_t size);
void spi_receive(uint8_t *data,uint32_t size);

#endif /* INC_SPI_H_ */
