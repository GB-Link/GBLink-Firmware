#include "ereader.hpp"
#include "ereaderPio.hpp"
#include "../gbaLinkLayer.hpp"

#include <zephyr/kernel.h>

#include "hardware/gpio.h"

#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/drivers/pinctrl.h>

static void configureEreaderSlavePio(void);

static void releaseInactiveSdPin(void);

static void assertBothSdPartnerGpio(void);

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

/* Undo the SIO overrides the e-reader flows leave behind (SD pins driven high
 * for partner presence, SO forced low as e-reader child) so later modes start
 * from cable-idle pin state; the PIO paths only reclaim the pins they use. */
void ereader_releasePartnerPins(void)
{
    uint32_t SO_pin = link_getPin(SO);
    uint32_t sd_gba = link_getPin(SD_GBA);
    uint32_t sd_gbc = link_getPin(SD_GBC);
    gpio_init(SO_pin); gpio_set_dir(SO_pin, GPIO_IN); gpio_disable_pulls(SO_pin);
    gpio_init(sd_gba); gpio_set_dir(sd_gba, GPIO_IN); gpio_disable_pulls(sd_gba);
    gpio_init(sd_gbc); gpio_set_dir(sd_gbc, GPIO_IN); gpio_disable_pulls(sd_gbc);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void ereader_configurePartnerPresence(void)
{
    link_disable();
    uint32_t SO_pin = link_getPin(SO);
    uint32_t SC_pin = link_getPin(SC);
    uint32_t SI_pin = link_getPin(SI);
    gpio_init(SC_pin); gpio_pull_up(SC_pin); gpio_set_dir(SC_pin, GPIO_IN);
    gpio_init(SI_pin); gpio_pull_up(SI_pin); gpio_set_dir(SI_pin, GPIO_IN);
    gpio_init(SO_pin); gpio_set_dir(SO_pin, GPIO_OUT); gpio_put(SO_pin, 0);
    assertBothSdPartnerGpio();
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void ereader_configureEreaderSlave(void)
{
    link_disable();

    uint32_t SC_pin = link_getPin(SC);
    uint32_t SI_pin = link_getPin(SI);
    uint32_t SO_pin = link_getPin(SO);

    gpio_init(SC_pin); gpio_pull_up(SC_pin); gpio_set_dir(SC_pin, GPIO_IN);
    gpio_init(SI_pin); gpio_pull_up(SI_pin); gpio_set_dir(SI_pin, GPIO_IN);
    gpio_init(SO_pin); gpio_set_dir(SO_pin, GPIO_OUT); gpio_put(SO_pin, 0);
    releaseInactiveSdPin();

    configureEreaderSlavePio();
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void configureEreaderSlavePio(void)
{
    link_disable();

    enum CableType cableType = link_getSelectedCableType();
    link_configureCableType(cableType);

    switch (cableType)
    {
        case GBC:
            link_configureProgram(
                RPI_PICO_PIO_GET_PROGRAM(pio_slave_ereader_gbc), 
                RPI_PICO_PIO_GET_WRAP(pio_slave_ereader_gbc), 
                RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_ereader_gbc)
            );
            break;

        case GBA:
            link_configureProgram(
                RPI_PICO_PIO_GET_PROGRAM(pio_slave_ereader_gba), 
                RPI_PICO_PIO_GET_WRAP(pio_slave_ereader_gba), 
                RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_ereader_gba)
            );
    }

    gpio_init(link_getPin(SO));
    gpio_set_dir(link_getPin(SO), GPIO_OUT);
    gpio_put(link_getPin(SO), 0);

    link_setPioPinDirs(link_getPin(SC), PIN_DIR_IN);
    link_setPioPinDirs(link_getPin(SI), PIN_DIR_IN);

    link_enable();
}

static void assertBothSdPartnerGpio(void)
{
    uint32_t sd_gba = link_getPin(SD_GBA);
    uint32_t sd_gbc = link_getPin(SD_GBC);
    gpio_init(sd_gba); gpio_set_dir(sd_gba, GPIO_OUT); gpio_put(sd_gba, 1);
    gpio_init(sd_gbc); gpio_set_dir(sd_gbc, GPIO_OUT); gpio_put(sd_gbc, 1);
}


static void releaseInactiveSdPin(void)
{
    const bool gbc = link_getSelectedCableType() == GBC;

    const uint32_t inactive = gbc ? link_getPin(SD_GBC) : link_getPin(SD_GBA);
        
    gpio_init(inactive);
    gpio_set_dir(inactive, GPIO_IN);
    gpio_disable_pulls(inactive);
}