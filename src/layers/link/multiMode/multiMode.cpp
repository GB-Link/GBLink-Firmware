#include "multiMode.hpp"
#include "multiModePio.hpp"
#include "./../linkLayer.hpp"

#include <zephyr/kernel.h>

#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void configureMaster();

static void configureSlave();

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Interface
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void multiMode_selectMode(enum LinkMode mode)
{
    switch(mode)
    {
    case MASTER: configureMaster(); break;
    case SLAVE: configureSlave(); break;
    case DISABLED: link_disable(); break;
    }
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

// SD is read on both pins and driven only on the pin the GBA answered on, so there is one program per role and
// no cable type to pick.
static void configureMaster()
{
    link_disable();

    link_configureProgram(
        RPI_PICO_PIO_GET_PROGRAM(pio_master_multi),
        RPI_PICO_PIO_GET_WRAP(pio_master_multi),
        RPI_PICO_PIO_GET_WRAP_TARGET(pio_master_multi),
        MASTER
    );

    link_enable();
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void configureSlave(void)
{
    link_disable();

    link_configureProgram(
        RPI_PICO_PIO_GET_PROGRAM(pio_slave_multi),
        RPI_PICO_PIO_GET_WRAP(pio_slave_multi),
        RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_multi),
        SLAVE
    );

    link_enable();
}
