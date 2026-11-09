#include "multiMode.hpp"
#include "multiModePio.hpp"
#include "./../gbaLinkLayer.hpp"

#include <zephyr/kernel.h>

#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/drivers/pinctrl.h>

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void configureMaster();

static void configureSlave();

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Interface
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void multiMode_selectMode(enum MultiMode mode)
{
    switch(mode)
    {

    case MASTER: configureMaster(); break;
    case SLAVE: configureSlave(); break;
    case DISABLED: link_disable(); break;
      break;
    }
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void configureMaster()
{
    link_disable();
    link_configureProgram(
        RPI_PICO_PIO_GET_PROGRAM(pio_master), 
        RPI_PICO_PIO_GET_WRAP(pio_master), 
        RPI_PICO_PIO_GET_WRAP_TARGET(pio_master)
    );
    
	link_enable();
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void configureSlave(void)
{
    link_disable();
    link_configureProgram(
        RPI_PICO_PIO_GET_PROGRAM(pio_slave),
        RPI_PICO_PIO_GET_WRAP(pio_slave),
        RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave)
    );

	link_enable();
}