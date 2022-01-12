/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

/* Configure non-secure peripherals. */

void nrf_spu_periph_set(uint16_t id, uint32_t flags);
void nrf_spu_periph_clear(uint16_t id, uint32_t flags);
void nrf_spu_gpio_set(uint16_t id);
void nrf_spu_gpio_clear(uint16_t id);

void nrf_gpio_dir_set(uint16_t pin);
void nrf_gpio_dir_clear(uint16_t pin);
void nrf_gpio_out_set(uint16_t pin);
void nrf_gpio_out_clear(uint16_t pin);
void nrf_gpio_toggle(uint16_t pin);

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/
int main(void) {
  int i, j;

  for ( i = 2; i < 10; i++ )
  {
    nrf_gpio_dir_set(i);
  }

  j = 0;

  do {
    nrf_gpio_out_set( j + 2 );
    for (i = 0; i < 10000; i++) {
      printf("Hello World %d!\n", i);
    }

    nrf_gpio_toggle( j + 2 );
    j = ( j + 1 ) % 4;
  } while (1);
}

void nrf_gpio_dir_set(uint16_t pin)
{
  NRF_P0_NS->DIRSET |= (1 << pin);
}

void nrf_gpio_dir_clear(uint16_t pin)
{
  NRF_P0_NS->DIRCLR |= (1 << pin);
}

void nrf_gpio_out_set(uint16_t pin)
{
  NRF_P0_NS->OUTSET |= (1 << pin);
}

void nrf_gpio_out_clear(uint16_t pin)
{
  NRF_P0_NS->OUTCLR |= (1 << pin);
}

void nrf_gpio_toggle(uint16_t pin)
{
  uint32_t gpio;

  gpio = NRF_P0_NS->OUT;
  gpio ^= (1 << pin);
  NRF_P0_NS->OUT = gpio;
}

/*************************** End of file ****************************/
