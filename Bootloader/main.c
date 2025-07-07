// Copyright 2011 Emilie Gillet.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// -----------------------------------------------------------------------------
//
// Bootloader supporting MIDI SysEx update.
//
// Caveat: assumes the firmware flashing is always done from first to last
// block, in increasing order. Random access flashing is not supported!

// #include <avr/boot.h>
// #include <avr/interrupt.h>
#include <avr/io.h>
// #include <avr/pgmspace.h>

#define SPM_PAGESIZE 64

FUSES = {
	.WDTCFG = 0x00, // WDTCFG {PERIOD=OFF, WINDOW=OFF}
	.BODCFG = 0xE0, // BODCFG {SLEEP=DIS, ACTIVE=DIS, SAMPFREQ=1KHz, LVL=BODLEVEL7}
	.OSCCFG = 0x01, // OSCCFG {FREQSEL=16MHZ, OSCLOCK=CLEAR}
	.TCD0CFG = 0x00, // TCD0CFG {CMPA=CLEAR, CMPB=CLEAR, CMPC=CLEAR, CMPD=CLEAR, CMPAEN=CLEAR, CMPBEN=CLEAR, CMPCEN=CLEAR, CMPDEN=CLEAR}
	.SYSCFG0 = 0xF6, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=UPDI, CRCSRC=NOCRC}
	.SYSCFG1 = 0x07, // SYSCFG1 {SUT=64MS}
	.APPEND = 0x07, // APPEND {APPEND=User range:  0x0 - 0xFF}
	.BOOTEND = 0x03, // BOOTEND {BOOTEND=User range:  0x0 - 0xFF}
};

enum SysExReceptionState {
  MATCHING_HEADER = 0,
  MATCHING_OLD_HEADER = 1,
  READING_COMMAND = 2,
  READING_DATA = 3,
};

uint16_t page = 0;
uint8_t rx_buffer[SPM_PAGESIZE + 1];

void (*main_entry_point)(void) = 0x0000;

void init()
{
    // Configure the prescaler
	CPU_CCP = CCP_IOREG_gc;							// Allow the protected register CLKCTRL being programmed
    // CLKCTRL.MCLKCTRLB = 7;
	CLKCTRL.MCLKCTRLB = (1<<CLKCTRL_PEN_bp) | (1<<CLKCTRL_PDIV_0_bp) | (1<<CLKCTRL_PDIV_1_bp);	// Enable the Clock-Prescaler, Divide by 16, clk_per is now 1MHz
    
    // Configure the UART
	USART0.CTRLB |= (1<<USART_RXEN_bp);		// Enable USART0 Receiver
    USART0.CTRLA |= (1<<USART_RXCIE_bp);    // Enable USART0 Receiver Interrupt
	USART0.CTRLC = (3<<USART_CHSIZE_gp);    // MIDI has 1 Start bit, 8 data bits, no parity, 1 stop bit (8-N-1)	
	USART0.BAUD = 128;                     // 31.250 Hz
}

void write_status_leds(uint8_t pattern) {
    // Insert here code for outputing an 8 bits value to LEDs.
}

uint8_t bootloader_active() 
{
    // Check if the upper button is pressed
    if( PORTB.IN & (1<<PIN0_bp) )
    {
        return(0);
    }
    else
    {
        return(1);
    }
}

inline void write_buffer_to_flash() 
{
    uint16_t i;
    const uint8_t* p = rx_buffer;
    // eeprom_busy_wait();
  
    // boot_page_erase(page);
    // boot_spm_busy_wait();

    for (i = 0; i < SPM_PAGESIZE; i += 2) 
    {
        uint16_t w = *p++;
        w |= (*p++) << 8;
        // boot_page_fill(page + i, w);
    }

    // boot_page_write(page);
    // boot_spm_busy_wait();
    // boot_rww_enable();
}

const uint8_t sysex_header[] = {
  0xf0,  // <SysEx>
  0x00, 0x21, 0x02,  // Manufacturer ID for Mutable instruments.
  0x02,  // Product ID for "any other project".
};


void midi_rx_loop()
{

    uint8_t byte;
    uint16_t bytes_read = 0;
    uint16_t rx_buffer_index;
    uint8_t state = MATCHING_HEADER;
    uint8_t checksum;
    uint8_t sysex_commands[2];
    uint8_t current_led = 1;
    uint8_t status = 0;
    uint8_t progress_counter = 0;
    
    page = 0;
    write_status_leds(0x55);
    
    while (1) 
    {
        
        while (!(USART0.STATUS & USART_RXCIF_bm) )
        {
            ; // Do nothing
        }

        byte = USART0.RXDATAL;
        
        // In case we see a realtime message in the stream, safely ignore it.
        //if (byte > 0xf0 && byte != 0xf7) 
        //{
        //    continue;
        //}
        // write_status_leds(status);
        
        switch (state) 
        {
            case MATCHING_HEADER:
                // Check if the sysex_header is correct. If yes,
                // go to READING_COMMAND
                if (byte == sysex_header[bytes_read]) 
                {
                    ++bytes_read;
                    if ( bytes_read == sizeof (sysex_header) ) 
                    {
                        bytes_read = 0;
                        state = READING_COMMAND;
                    }
                }
                else 
                {
                    bytes_read = 0;
                }
                break;

            case READING_COMMAND:
                // Read the command bytes. Don't interpret.
                // If two bytes have been read, go to 
                // READING_DATA
                if (byte < 0x80) 
                {
                    sysex_commands[bytes_read++] = byte;
                    if (bytes_read == 2) 
                    {
                        bytes_read = 0;
                        rx_buffer_index = 0;
                        checksum = 0;
                        state = READING_DATA;
                    }
                }
                else 
                {
                    state = MATCHING_HEADER;
                    current_led = 1;
                    status = 0;
                    bytes_read = 0;
                }
                break;

            case READING_DATA:
                if (byte < 0x80) 
                {
                    // De-Nibblize: Odd
                    if (bytes_read & 1) 
                    {
                        rx_buffer[rx_buffer_index] |= byte & 0xf;
                        if (rx_buffer_index < SPM_PAGESIZE) 
                        {
                            checksum += rx_buffer[rx_buffer_index];
                        }
                        ++rx_buffer_index;
                    }
                    else 
                    // Even
                    {
                        rx_buffer[rx_buffer_index] = (byte << 4);
                    }
                    ++bytes_read;
                }
                else 
                if (byte == 0xf7) 
                {
                    // 7F 00 is the reset-command
                    if ( sysex_commands[0] == 0x7f &&
                         sysex_commands[1] == 0x00 &&
                         bytes_read == 0) 
                    {
                        // Return, when back in main() go straight to the 
                        // main program.
                        return;
                    }
                    else 
                    // 7E 00 is the update command
                    if (rx_buffer_index == SPM_PAGESIZE + 1 &&
                        sysex_commands[0] == 0x7e &&
                        sysex_commands[1] == 0x00 &&
                        rx_buffer[rx_buffer_index - 1] == checksum) 
                    {
                        // Block write.
                        // write_buffer_to_flash();
                        // page += SPM_PAGESIZE;
                        ++progress_counter;
                        if (progress_counter == 32) 
                        {
                            status |= current_led;
                            current_led <<= 1;
                            if (current_led == 0) 
                            {
                                current_led = 1;
                                status = 0;
                            }
                            progress_counter = 0;
                        }
                        status ^= current_led;
                    }
                    else 
                    {
                        current_led = 1;
                        status = 0;
                    }
                    state = MATCHING_HEADER;
                    bytes_read = 0;
                }
                break;
        }
    }
}

int main(void) {
    // uint8_t watchdog_status = MCUSR;
    // MCUSR = 0;
    // WDTCSR |= _BV(WDCE) | _BV(WDE);
    // WDTCSR = 0;

    init();
    midi_rx_loop();
    if (bootloader_active()) 
    {
        midi_rx_loop();
    }
    main_entry_point();
}
