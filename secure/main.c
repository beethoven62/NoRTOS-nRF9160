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
#include <arm_cmse.h>
#include <nrf.h>

/* Start address of non-secure application. */
#define mainNONSECURE_APP_START_ADDRESS		( 0x00010000U )
#define mainNONSECURE_RAM_START_ADDRESS         ( 0x20008000U )
#define RAM_ADDRESS_MASK                        ( 0x0fffffffU )
#define SPU_FLASHREGION_SIZE                    ( 0x8000U )
#define SPU_RAMREGION_SIZE                      ( 0x2000U )
#define SPU_NUM_REGIONS                         ( 0x20U )

/* typedef for non-secure Reset Handler. */
typedef void ( *NonSecureResetHandler_t )( void ) __attribute__( ( cmse_nonsecure_call ) );
/*-----------------------------------------------------------*/

/* Configure non-secure regions. */
void ConfigNonSecure( void );
void nrf_spu_flashregion_set(uint16_t region, uint32_t flags);
void nrf_spu_flashregion_clear(uint16_t region, uint32_t flags);
void nrf_spu_ramregion_set(uint16_t region, uint32_t flags);
void nrf_spu_ramregion_clear(uint16_t region, uint32_t flags);
void nrf_spu_periph_set(uint16_t id, uint32_t flags);
void nrf_spu_periph_clear(uint16_t id, uint32_t flags);
void nrf_spu_gpio_set(uint16_t id);
void nrf_spu_gpio_clear(uint16_t id);

/* Boot into the non-secure code. */
void BootNonSecure( uint32_t ulNonSecureStartAddress );
/*-----------------------------------------------------------*/

/* Secure main() */
int main( void )
{
      /* Configure non-secure regions. */
      ConfigNonSecure();

      /* Boot the non-secure code. */
      BootNonSecure( mainNONSECURE_APP_START_ADDRESS );

      /* Non-secure software does not return, this code is not executed. */
      for( ; ; )
      {
              /* Should not reach here. */
      }
}
/*-----------------------------------------------------------*/

void BootNonSecure( uint32_t ulNonSecureStartAddress )
{
      NonSecureResetHandler_t pxNonSecureResetHandler;

      /* Main Stack Pointer value for the non-secure side is the first entry in
       * the non-secure vector table. Read the first entry and assign the same to
       * the non-secure main stack pointer(MSP_NS). */
      __TZ_set_MSP_NS( *( ( uint32_t * )( ulNonSecureStartAddress ) ) );

      /* Non secure Reset Handler is the second entry in the non-secure vector
       * table. Read the non-secure reset handler.
       */
      pxNonSecureResetHandler = ( NonSecureResetHandler_t )( * ( ( uint32_t * ) ( ( ulNonSecureStartAddress ) + 4U ) ) );

      /* Start non-secure software application by jumping to the non-secure Reset
       * Handler. */
      pxNonSecureResetHandler();
}
/*-----------------------------------------------------------*/

void ConfigNonSecure( void )
{
  uint16_t rgn;

  for ( rgn = mainNONSECURE_APP_START_ADDRESS / SPU_FLASHREGION_SIZE; rgn < SPU_NUM_REGIONS; rgn++ )
  {
    nrf_spu_flashregion_clear(rgn, SPU_FLASHREGION_PERM_SECATTR_Msk);
   }
  for ( rgn = ( mainNONSECURE_RAM_START_ADDRESS & RAM_ADDRESS_MASK ) / SPU_RAMREGION_SIZE; rgn < SPU_NUM_REGIONS; rgn++ )
  {
    nrf_spu_ramregion_clear(rgn, SPU_RAMREGION_PERM_SECATTR_Msk);
  }

  rgn = (( uint32_t )NRF_P0_NS >> 12) & 0x7f;
  nrf_spu_periph_clear(rgn, SPU_PERIPHID_PERM_SECATTR_Msk);

  for ( rgn = 2; rgn < 10; rgn++ )
  {
    nrf_spu_gpio_clear(rgn);
  }
}

void nrf_spu_flashregion_set(uint16_t region, uint32_t flags)
{
  NRF_SPU_S->FLASHREGION[region].PERM |= flags;
}

void nrf_spu_flashregion_clear(uint16_t region, uint32_t flags)
{
  NRF_SPU_S->FLASHREGION[region].PERM &= ~flags;
}

void nrf_spu_ramregion_set(uint16_t region, uint32_t flags)
{
  NRF_SPU_S->RAMREGION[region].PERM |= flags;
}

void nrf_spu_ramregion_clear(uint16_t region, uint32_t flags)
{
  NRF_SPU_S->RAMREGION[region].PERM &= ~flags;
}

void nrf_spu_gpio_set(uint16_t id)
{
  NRF_SPU_S->GPIOPORT[0].PERM |= (1 << id);
}

void nrf_spu_gpio_clear(uint16_t id)
{
  NRF_SPU_S->GPIOPORT[0].PERM &= ~(1 << id);
}

void nrf_spu_periph_set(uint16_t id, uint32_t flags)
{
  NRF_SPU_S->PERIPHID[id].PERM |= flags;
}

void nrf_spu_periph_clear(uint16_t id, uint32_t flags)
{
  NRF_SPU_S->PERIPHID[id].PERM &= ~flags;
}

/*************************** End of file ****************************/
