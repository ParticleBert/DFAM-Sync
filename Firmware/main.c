/*
 * File:   main.c
 * Author: C. Wiesner (cw@midimonster.de)
 *
 * Created on 13. Juli 2024, 14:28
 */
#include <avr/io.h>

#define DEBUG_ON        PORTB.OUTSET = (1<<PIN1_bp)
#define DEBUG_OFF       PORTB.OUTCLR = (1<<PIN1_bp)
#define SYNC_ON         PORTA.OUTSET = (1<<PIN5_bp)
#define SYNC_OFF        PORTA.OUTCLR = (1<<PIN5_bp)
#define RUN_STOP_ON     PORTA.OUTSET = (1<<PIN6_bp)
#define RUN_STOP_OFF    PORTA.OUTCLR = (1<<PIN6_bp)
#define LED_ON          PORTA.OUTSET = (1<<PIN7_bp)
#define LED_OFF         PORTA.OUTCLR = (1<<PIN7_bp)

#define MIDI_CLOCK      0xF8
#define MIDI_START      0xFA
#define MIDI_CONTINUE   0xFB
#define MIDI_STOP       0xFC

#define D9_ON   do { PORTA.DIRSET = (1<<PIN4_bp) | (1<<PIN1_bp); \
                     PORTA.OUTSET = (1<<PIN4_bp); \
                     PORTA.OUTCLR = (1<<PIN1_bp); } while(0)

#define D9_OFF  do { PORTA.DIRCLR = (1<<PIN4_bp) | (1<<PIN1_bp); } while(0)

// #define MIDITICKS_RELOAD 05		// 05: Normal Tempo
								// 11: Halved Tempo

// Clock Division Modes
const uint8_t division_factors[8] = {
    24,  // Mode 0: 1/4  note
    12,  // Mode 1: 1/8  note
     6,  // Mode 2: 1/16 note <- Standard
     3,  // Mode 3: 1/32 note
     2,  // Mode 4: 1/32 triplet
     4,  // Mode 5: 1/16 triplet
     8,  // Mode 6: 1/8  triplet
    16   // Mode 7: 1/4  triplet
};

const uint16_t pattern_lengths[8] = {
    192, // Mode 0
     96, // Mode 1
     48, // Mode 2 <- Standard
     24, // Mode 3
     16, // Mode 4
     32, // Mode 5
     64, // Mode 6
    128  // Mode 7
};

enum division_state_enum
{
	DIV_RUNNING,		// Normal laufend
	DIV_PENDING,		// User hat Änderung angefordert
	DIV_WAIT_FOR_STEP_0	// Master-Takt ist durch, warte auf DFAM Step 0
} division_state;

uint8_t pending_mode; 	// Der Modus, zu dem gewechselt werden soll

// Division mode variables
uint8_t current_mode = 2;  // Start at mode 2 (1/16 notes, 1:1 ratio)
								
// _bm = Bit Mask       Bsp: #define TCA_SINGLE_PERBV_bm  0x01  /* Period Buffer Valid bit mask.
// _bp = Bit Position   Bsp: #define TCA_SINGLE_PERBV_bp  0  /* Period Buffer Valid bit position.
// _gm = Group Mask     Bsp: #define CLKCTRL_CLKSEL_gm  0x03
// _gp = Group Position Bsp: #define CLKCTRL_CLKSEL_gp  0

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

LOCKBITS = 0xC5; // {LB=NOLOCK}

struct buttons_struct
{
    uint8_t up_prev;
    uint8_t up_act;
    uint8_t up_pressed;
    
    uint8_t dn_prev;
    uint8_t dn_act;
    uint8_t dn_pressed;
    
    uint8_t st_prev;
    uint8_t st_act;
    uint8_t st_pressed;
} button;

struct ctr_struct
{
    uint8_t miditicks;		// Countdown für nächsten Trigger
	uint8_t dfam_step;		// NEU: Aktueller DFAM Step (0-7)
	uint8_t master_clock;	// NEU: Zählt MIDI-Clocks im Haupttakt (0-47)
} ctr;

enum state_enum 
{
    INIT, HANDLE_START_REQ, HANDLE_CONT_REQ, RUN, STOP
} state, state_old;

enum keystate_enum
{
	TIMING, NUDGE
} keystate;

void HandleButtons(void)
{
	// AUTO		SW_AUTO		PB0
	// FAST		SW_DEC		PA2
	// SLOW		SW_INC		PA3
	
	static uint8_t blink_state = 0;
	
    if(TCA0.SINGLE.INTFLAGS & (1<<TCA_SINGLE_OVF_bp) )
    {
        // 10ms have elapsed
        
        // Handle UP-Button (Upper, SW_AUTO)
        if(PORTB.IN & (1<<PIN0_bp))
        {
            button.up_act = 1;
        }
        else
        {
            button.up_act = 0;
        }
        if( (button.up_prev == 1) && (button.up_act == 0) )
        {
            button.up_pressed = 1;
        }
        button.up_prev = button.up_act;
		
		// Handle Status-Button (Middle, SW_DEC)
		if(PORTA.IN & (1<<PIN2_bp))
		{
			button.st_act = 1;
		}
		else
		{
			button.st_act = 0;
		}
		if( (button.st_prev == 1) && (button.st_act == 0) )
		{
			button.st_pressed = 1;
		}
		button.st_prev = button.st_act;
        
        // Handle DOWN-Button (Lower, SW_INC)
        if(PORTA.IN & (1<<PIN3_bp))
        {
            button.dn_act = 1;
        }
        else
        {
            button.dn_act = 0;
        }
        if( (button.dn_prev == 1) && (button.dn_act == 0) )
        {
            button.dn_pressed = 1;
        }
        button.dn_prev = button.dn_act;
        
		// Blink LED if change is pending
		if(division_state == DIV_PENDING || division_state == DIV_WAIT_FOR_STEP_0)
		{
			blink_state = !blink_state;
			if(blink_state)
				D9_ON;
			else
				D9_OFF;
		}
		else
		{
			D9_OFF;
		}
		
        // Clear the Interrupt flag
        TCA0.SINGLE.INTFLAGS |= (1<<TCA_SINGLE_OVF_bp);
    }
}

void SendOnePulse(void)
{
    TCB0.CCMP = 120; // 120µs = 0,12ms
    TCB0.CNT = 0;
    SYNC_ON;
    TCB0.CTRLA |= (1<<TCB_ENABLE_bp); // Enables TCB0

    while(!(TCB0.INTFLAGS & (TCB_CAPT_bm))); // Waits for the capture interrupt

    TCB0.INTFLAGS |= (1<<TCB_CAPT_bm); // Clears the Interruptflag
    SYNC_OFF;
    TCB0.CCMP = 100; // 100µs = 0,1ms
    TCB0.CNT = 0;

    while(!(TCB0.INTFLAGS & (TCB_CAPT_bm))); // Waits for the capture interrupt
    
    TCB0.INTFLAGS |= (1<<TCB_CAPT_bm); // Clears the Interruptflag       
}

void HandlePulse(void)
{
    // Check if Timer / Counter B (set to one shot) is already finished.
    if(TCB0.INTFLAGS & (TCB_CAPT_bm))
    {
        LED_OFF; // Switch the LED-Port off
        SYNC_OFF; // Switch the SYNC-Port off
        TCB0.INTFLAGS |= (1<<TCB_CAPT_bm); // Clear the Interruptflag
        TCB0.CTRLA &= ~(1<<TCB_ENABLE_bp); // Disable TCB
    }
}

uint8_t HandleMIDI_IN(void)
{      
    if( USART0.STATUS & USART_RXCIF_bm ) // If data is available
    {
        uint8_t rx_data = USART0.RXDATAL; // Read the data
        
        if
        (   (rx_data == MIDI_CLOCK) | // If it is interesting for us..
            (rx_data == MIDI_START) |
            (rx_data == MIDI_CONTINUE) |
            (rx_data == MIDI_STOP) )
        {
            return rx_data; // Return it
        }
        else
        {
            return 0; // If not, return 0
        }
    }
    else
    {
        return 0;
    }
}

int main(void) 
{
  	// Configure the prescaler
	CCP = 0xD8;							// Allow the protected register CLKCTRL being programmed
	CLKCTRL.MCLKCTRLB = (1<<CLKCTRL_PEN_bp) | (3<<CLKCTRL_PDIV_gp);			// Enable the Clock-Prescaler, Divide by 16, clk_per is now 1MHz
    
    // Configure the UART
	USART0.CTRLB |= (1<<USART_RXEN_bp);		// Enable USART0 Receiver
    USART0.CTRLB |= (1<<USART_TXEN_bp);     // Enable USART0 Transmitter
    USART0.CTRLA |= (1<<USART_RXCIE_bp);    // Enable USART0 Receiver Interrupt
	USART0.CTRLC = (3<<USART_CHSIZE_gp);    // MIDI has 1 Start bit, 8 data bits, no parity, 1 stop bit (8-N-1)	
	USART0.BAUD = 128;                      // 31.250Hz
    
    // Configure Pins as Output
    PORTA.DIRSET |= (1<<PIN5_bp);           // PA5 (3) is Sync-Out
    PORTA.DIRSET |= (1<<PIN6_bp);           // PA6 is RUN / STOP
    PORTA.DIRSET |= (1<<PIN7_bp);           // PA7 (5) is MIDI LED
    PORTB.DIRSET |= (1<<PIN1_bp);           // PB1 (8) is the debug output   
	PORTB.DIRSET |= (1<<PIN2_bp);			// PB2 (7) is MIDI output
  
    // Configure the Timer / Counter A
    TCA0.SINGLE.CTRLA |= (1<<TCA_SINGLE_ENABLE_bp); // Enable the peripheral
    // TCA0.SINGLE.INTCTRL |= (1<<TCA_SINGLE_OVF_bp); // Enable the overflow interrupt
    TCA0.SINGLE.PER = 10000; // 10ms
  
    // Initialize variables

    state = INIT;
    
    SYNC_OFF;
    // Configure the Timer / Counter B
    TCB0.CTRLB |= TCB_CNTMODE_1_bm | TCB_CNTMODE_2_bm; // 0x06: Single Shot Mode
    TCB0.CCMP = 10000;  // 10ms
    // TCB0.CTRLA |= (1<<TCB_ENABLE_bp); // Enables TCB0
  
    RUN_STOP_ON;
	for(uint8_t i=0; i<7; i++)
	{
		SendOnePulse(); // Send 7 pulses
	}
    
    while (1)
    {
        // Local variables
        uint8_t midi_in = 0;

        // 1: Update pressed buttons
        HandleButtons();
        
		// 2: Handle the select-Button
		if(button.st_pressed)
		{
			switch(keystate)
			{
				case NUDGE:
					keystate = TIMING;
					break;
				case TIMING:
					keystate = NUDGE;
					break;
			}
			button.st_pressed = 0;
		}
		
        // 3: Update MIDI Input
        midi_in = HandleMIDI_IN();
        
        if(midi_in)
        {
            state_old = state; // Backup state
            switch(state)
            {
                case INIT:
                    if(midi_in == MIDI_START)
                    {
                        state = HANDLE_START_REQ;
                    }
                    break;
                case HANDLE_START_REQ:
                    if(midi_in == MIDI_CLOCK)
                    {
                        ctr.miditicks = division_factors[current_mode] - 1;  // -1 weil du sofort dekrementierst
						ctr.dfam_step = 0;
						ctr.master_clock = 0;	// Neu: Master clock startet bei 0
                        LED_ON;
                        SYNC_ON;
                        TCB0.CCMP = 10000;
                        TCB0.CNT = 0;
                        TCB0.CTRLA |= (1<<TCB_ENABLE_bp);                        
                        state = RUN;
                    }
                    break;
                case HANDLE_CONT_REQ:
                    if(midi_in == MIDI_CLOCK)
                    {
                        state = RUN;
                    }
                    break;
                case STOP:
                    if(midi_in == MIDI_START)
                    {
						// Berechne wie viele Pulses bis Step 0
						uint8_t pulses_to_send = ((8 - ctr.dfam_step) % 8) - 1;
                        
						// Sende die restlichen Pulses
						while(pulses_to_send > 0)
                        {
                            SendOnePulse();
                            pulses_to_send--;
                        }
						
						ctr.dfam_step = 7;	// Jetzt auf Step 0
                        state = HANDLE_START_REQ;
                    }
                    if(midi_in == MIDI_CONTINUE)
                    {
                        state = HANDLE_CONT_REQ;
                    }
                    break;
                case RUN:
                    if(midi_in == MIDI_CLOCK)
                    {
						ctr.master_clock++;
						
						// Prüfe ob Umschalt-Zeitpunkt erreicht ist
						uint8_t should_check_switch = 0;
						
						// Bestimme ob wir in/zu Triolen wechseln
						uint8_t involves_triplets = (current_mode >= 4) || (pending_mode >= 4);
						
						if(involves_triplets)
						{
							// Triolen: Bei jeder Viertelnote (12, 24, 36, 48)
							if(ctr.master_clock >= 48)
							{
								ctr.master_clock = 0;
								should_check_switch = 1;
							}
							else if ((ctr.master_clock == 12) || (ctr.master_clock == 24) || (ctr.master_clock == 36) ||(ctr.master_clock == 48))
							{
								should_check_switch = 1;
							}
						}
						else
						{
							// Nur normale Noten: Bei Taktende (48)
							if(ctr.master_clock >= 48)
							{
								ctr.master_clock >= 0;
								should_check_switch = 1;
							}
						}
						
						// Wenn Zeitpunkt erreicht und Umschaltung pending
						if(should_check_switch && division_state == DIV_PENDING)
						{
							division_state = DIV_WAIT_FOR_STEP_0;
						}						
						
                        if(ctr.miditicks==0)
                        {
							// Send the pulse to ADV / CLOCK
                            LED_ON;
                            SYNC_ON;
                            TCB0.CCMP = 10000;
                            TCB0.CNT = 0;
                            TCB0.CTRLA |= (1<<TCB_ENABLE_bp);
							
							// Neu: DFAM Step hochzählen
							ctr.dfam_step++;
							if(ctr.dfam_step >= 8)
							{
								ctr.dfam_step = 0;
								
								// DFAm ist auf Step 0. Prüfe, ob wir warten
								if(division_state == DIV_WAIT_FOR_STEP_0)
								{
									// JETZT umschalten
									current_mode = pending_mode;
									division_state = DIV_RUNNING;
									D9_OFF;
								}
							}
							
                            ctr.miditicks = division_factors[current_mode] - 1;
                        }
						else
						{
							if(keystate == NUDGE)
							{
								// NUDGE Mode: Tempo adjustment
								
								// UP pressed: Speed up (don't subtract)
								if(button.up_pressed)
								{
									button.up_pressed = 0; // Reset Flag
									++ctr.miditicks;
								}
								// Normal: Subtract
								--ctr.miditicks;

								// DOWN pressed: Slow down (subtrackt twice)
								if(ctr.miditicks > 1)
								{
									if(button.dn_pressed)
									{
										--ctr.miditicks;
										button.dn_pressed = 0;
									}
								}
							}
							else // keystate == TIMING
							{
								// TIMING Mode: Division mode change
                
								// UP pressed: Smaller division (faster notes)
								if(button.up_pressed)
								{
									button.up_pressed = 0;
									if(current_mode < 7)
									{
										pending_mode = current_mode + 1;
										division_state = DIV_PENDING;
										// Nicht sofort umschalten
									}
								}
                
								// DOWN pressed: Larger division (slower notes)
								if(button.dn_pressed)
								{
									button.dn_pressed = 0;
									if(current_mode > 0)
									{
										pending_mode = current_mode - 1;
										division_state = DIV_PENDING;
										// Nicht sofort umschalten
									}
								}
                
							// In TIMING mode, still decrement normally
							--ctr.miditicks;
							}								
						}
					}
                    if(midi_in == MIDI_STOP)
                    {
                        state = STOP;
                    }
                    break;
                default:
                    break;
            }
        }
         
        // 3: If a LED / Sync Pulse has been set, check for the 10ms-flag
        // to turn it off.
        HandlePulse();
    }
}