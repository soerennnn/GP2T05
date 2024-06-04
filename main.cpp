#define F_CPU 16000000

#define MISO PB3// connected to SDI on LIS3DH
#define MOSI PB2// connected to SDO on LIS3DH
#define SCK PB1  // connected to SCK on LIS3DH
#define CS PB0   // connected to CS on LIS3DH

#include <avr/io.h>
#include <util/delay.h>
#include <UART.h>

// Funktion zur Initialisierung der SPI-Schnittstelle
void SPI_MasterInit(void) {
    // MOSI, SCK und CS als Ausgang konfigurieren
    DDRB |= (1 << MOSI) | (1 << SCK) | (1 << CS);
    // MISO als Eingang konfigurieren
    DDRB &= ~(1 << MISO);

    // SPI Einstellungen: SPI enable(SPE), Master-modus, SCK auf logisch „1“, Übertragung auf fallende Flanke, 1MHz Datenrate
    
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR0);
    
    SPCR &= ~(1<<SPR1); // SPR1 = 0 

    

    
}



void SPI_MasterTransmit(char data) {
    // Start der Übertragung
    SPDR = data; // Daten in das SPI Data Register schreiben
    // Warten, bis die Übertragung abgeschlossen ist
    while (!(SPSR & (1 << SPIF))); // Warten auf das SPI Interrupt Flag
}

uint8_t SPI_MasterReceive(void)
{
    /* send empty Byte */
    SPI_MasterTransmit(0x00);
    /* Return Value = SPI Data register */
    // while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

int main(void) {
    // UART Initialisierung mit Baudrate 9600 UBRR0 = 103
    UART0_INIT(103);

    // SPI Initialisierung
    SPI_MasterInit();

    uint8_t datax;

    while(1) {
        // Byte über UART empfangen
        datax = UART0_receive();

        PORTB &= ~(1 << CS);
        // Byte über SPI senden
        SPI_MasterTransmit(datax);

        // Byte über SPI empfangen
        SPI_MasterReceive();
        PORTB |= (1 << CS);
        
        datax = SPDR;

        // Empfangenes Byte über UART senden
        UART0_send(datax);

        _delay_ms(500); // Verzögerung
    }
}