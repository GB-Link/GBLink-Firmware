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

    enum CableType cableType = link_getSelectedCableType();
    link_configureCableType(cableType);
    switch (cableType)
    {
        case GBC:
            link_configureProgram(
                RPI_PICO_PIO_GET_PROGRAM(pio_master_gbc), 
                RPI_PICO_PIO_GET_WRAP(pio_master_gbc), 
                RPI_PICO_PIO_GET_WRAP_TARGET(pio_master_gbc)
            );
            break;
            
        case GBA:
            link_configureProgram(
                RPI_PICO_PIO_GET_PROGRAM(pio_master_gba), 
                RPI_PICO_PIO_GET_WRAP(pio_master_gba), 
                RPI_PICO_PIO_GET_WRAP_TARGET(pio_master_gba)
            );
    }
    
	link_enable();
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void configureSlave(void)
{
    link_disable();

    enum CableType cableType = link_getSelectedCableType();
    link_configureCableType(cableType);

    switch (cableType)
    {
        case GBC:
            link_configureProgram(
                RPI_PICO_PIO_GET_PROGRAM(pio_slave_gbc), 
                RPI_PICO_PIO_GET_WRAP(pio_slave_gbc), 
                RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gbc)
            );
            break;

        case GBA:
            link_configureProgram(
                RPI_PICO_PIO_GET_PROGRAM(pio_slave_gba), 
                RPI_PICO_PIO_GET_WRAP(pio_slave_gba), 
                RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gba)
            );
    }
    
	link_enable();

    link_setPioPinDirs(link_getPin(SC), PIN_DIR_IN);
    link_setPioPinDirs(link_getPin(SI), PIN_DIR_IN);
    link_setPioPinDirs(link_getPin(SO), PIN_DOR_OUT);
}