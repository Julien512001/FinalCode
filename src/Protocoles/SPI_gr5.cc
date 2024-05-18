#include "../../include/Protocoles/SPI_gr5.h"


/*! \brief send data in the SPI channel
 *  
 *  \param[in] addr SPI address of the device
*/

void init_spi(CtrlStruct *cvs)
{
    int spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, SPI_FLAGS);
    if (spi_handle < 0) {
        fprintf(stderr, "Impossible d'ouvrir le canal SPI\n");
    }

    cvs->spi->spi_handle = spi_handle;
}

void send_spi(CtrlStruct *cvs, uint32_t message, uint8_t addr) {
    int spi_handle = cvs->spi->spi_handle;

    // Séparation du message en octets
    char txData[] = {
        0x80 | addr,            // Premier byte: adresse du registre SPI
        (message >> 24) & 0xFF, // Deuxième byte (le plus significatif)
        (message >> 16) & 0xFF, // Troisième byte
        (message >> 8) & 0xFF,  // Quatrième byte
        message & 0xFF          // Cinquième byte (le moins significatif)
    };

    char rxData[5]; // 5 octets de données de réception

    spiXfer(spi_handle, txData, rxData, sizeof(txData));
}

void reveive_spi(CtrlStruct *cvs, int addr)
{

    int spi_handle = cvs->spi->spi_handle;


    char txData[] = {addr, 0x00, 0x00, 0x00, 0x00};
    char rxData[5];

    spiXfer(spi_handle, txData, rxData, sizeof(txData));

    int rx[] = {(int) rxData[1],(int) rxData[2],(int) rxData[3],(int) rxData[4]};
    cvs->spi->rxData = rx;
}

void free_spi(CtrlStruct *cvs)
{
    int spi_closing = spiClose(cvs->spi->spi_handle);
    if (spi_closing != 0) {
        fprintf(stderr, "Echec de fermeture du canal SPI\n");
    }

    free(cvs->spi);
}
