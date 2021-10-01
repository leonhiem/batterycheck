/*
 * Project: Battery Tester
 * Author : LeonH <L.Hiemstra@gmail.com>
 *
 * Build:   AVR Studio 4.15 (and 4.13)
 *          avr-gcc (WinAVR 20081205) 4.3.2
 *
 * Config:  ATtiny24; CPU=8000000 Hz; optimize=-Os
 *
 * Date:    1st release: DEC-2009
 * Change:  Sat Jul  7 18:15:38 CEST 2012
 *          Fri Nov  1 15:55:26 ICT 2013:  runtime for 3cell NiCd 700mAh
 *
 */

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>


/*
 * USI TWI definitions:
 */
// TWI commands
#define TWI_CMD_VREQ  		0xa5
#define TWI_CMD_TREQ  		0xa6
#define TWI_CMD_READALL		0xa7
#define TWI_CMD_RESET           0xa8


#define TWI_RX_BUFFER_SIZE  (8)
#define TWI_RX_BUFFER_MASK ( TWI_RX_BUFFER_SIZE - 1 )
#define TWI_TX_BUFFER_SIZE  (8)
#define TWI_TX_BUFFER_MASK ( TWI_TX_BUFFER_SIZE - 1 )

#define USI_SLAVE_CHECK_ADDRESS                (0x00)
#define USI_SLAVE_SEND_DATA                    (0x01)
#define USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA (0x02)
#define USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   (0x03)
#define USI_SLAVE_REQUEST_DATA                 (0x04)
#define USI_SLAVE_GET_DATA_AND_SEND_ACK        (0x05)

#define DDR_USI             DDRA
#define PORT_USI            PORTA
#define PIN_USI             PINA
#define PORT_USI_SDA        PORTA6
#define PORT_USI_SCL        PORTA4
#define PIN_USI_SDA         PINA6
#define PIN_USI_SCL         PINA4
#define USI_START_COND_INT  USISIF

/*
 * These are the macros for USI-TWI:
 */
#define SET_USI_TO_SEND_ACK( ) \
{ \
  /* prepare ACK */ \
  USIDR = 0; \
  /* set SDA as output */ \
  DDR_USI |= ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC )| \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_READ_ACK( ) \
{ \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* prepare ACK */ \
  USIDR = 0; \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | \
       ( 1 << USIDC ) | \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
} 

 
#define SET_USI_TO_TWI_START_CONDITION_MODE( ) \
{ \
  USICR = \
       /* enable Start Condition Interrupt, disable Overflow Interrupt */ \
       ( 1 << USISIE ) | ( 0 << USIOIE ) | \
       /* set USI in Two-wire mode, no USI Counter overflow hold */ \
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
       /* Shift Register Clock Source = External, positive edge */ \
       /* 4-Bit Counter Source = external, both edges */ \
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
       /* no toggle clock-port pin */ \
       ( 0 << USITC ); \
  USISR = \
        /* clear all interrupt flags, except Start Cond */ \
        ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
        ( 1 << USIDC ) | ( 0x0 << USICNT0 ); \
}
#define SET_USI_TO_SEND_DATA( ) \
{ \
  /* set SDA as output */ \
  DDR_USI |=  ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    =  \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
} 

#define SET_USI_TO_READ_DATA( ) \
{ \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    = \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | ( 1 << USIDC ) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
} 


/*
 * This variable sits in the EEPROM:
 */
uint8_t EE_slaveAddress EEMEM = 0x2c; // TWI slave address
uint8_t EE_osccal       EEMEM;


/*
 * Global variables system-wide
 */
static uint8_t TWI_slaveAddress;
static volatile uint8_t USI_TWI_Overflow_State; 

 
static uint8_t TWI_RxBuf[TWI_RX_BUFFER_SIZE];
static volatile uint8_t TWI_RxHead;
static volatile uint8_t TWI_RxTail;
 
static uint8_t TWI_TxBuf[TWI_TX_BUFFER_SIZE];
static volatile uint8_t TWI_TxHead;
static volatile uint8_t TWI_TxTail;
 
//uint8_t osccal_tmp;
uint8_t subs;
uint8_t usi_timeout;
uint8_t keylock;
uint16_t runtime;
uint16_t systimeout;

uint8_t adcsequence;

uint16_t adc1_vbatt;
uint16_t adc2_isense;
uint16_t vbatt_track;
uint16_t iref;

uint8_t sysstate;
uint8_t batt_condition;

/*
 * State defenitions
 */
#define LED_RED   PB1
#define LED_GREEN PB0

#define TIME5MIN    300
#define TIMETRACK   500
#define CHARGE_TIMEOUT 8500        // 2.36 hours (not suppose to run in this timeout but if dV,dt fails..)
#define DISCHARGE_THRESHOLD 4320   // 1 hour + 12min   >= means good battery  >= 600mAh
                                   //                  <  means bad battery   <  600mAh

#define FINISHED     0
#define NOT_FINISHED 1
#define ERROR        -1

#define STATE_READY      					0
#define STATE_discharge_1					1
#define STATE_WAIT_COOL_AFTER_discharge_1	 6 
#define STATE_charge_1						2
#define STATE_WAIT_COOL_AFTER_charge_1		 7
#define STATE_discharge_2					3
#define STATE_WAIT_COOL_AFTER_discharge_2	 8
#define STATE_charge_2						4
#define STATE_ERROR                         5
#define STATE_BATT_good						9
#define STATE_BATT_bad						10
#define STATE_calibrate                     11

#define cIREF_CHARGE       766 // 500mA
#define cIREF_CHARGE_HL    763
#define cIREF_CHARGE_HH    769
//#define cIREF_CHARGE       950 // 850mA
//#define cIREF_CHARGE_HL    947
//#define cIREF_CHARGE_HH    953



#define cIREF_DISCHARGE     255 // 500mA
#define cIREF_DISCHARGE_HL  252
#define cIREF_DISCHARGE_HH  258
//#define cIREF_DISCHARGE     70 // 850mA
//#define cIREF_DISCHARGE_HL  67
//#define cIREF_DISCHARGE_HH  73

/* ADC defenitions */
#define cStartAdc    ((1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(0<<ADPS1)|(0<<ADPS0))
#define cRestartAdc  ((cStartAdc)|(1<<ADSC))
#define cAdcV        0 

/*
 * Subroutine declarations:
 */
static void display_state(void);
static uint8_t timed_out(void);
static int8_t process_discharge(void);
static int8_t process_charge(void);
static uint8_t which_button_pressed(void);
static void USI_TWI_Slave_Initialise( void );
static int8_t USI_TWI_Transmit_Byte( uint8_t );
static uint8_t USI_TWI_Receive_Byte( void );
static uint8_t USI_TWI_Data_In_Receive_Buffer( void );
static void reset_runtime(void);


/*
 * Insterrupt service routines:
 */
ISR(USI_STR_vect)
{     
 // Set default starting conditions for new TWI package
     USI_TWI_Overflow_State = USI_SLAVE_CHECK_ADDRESS;
     DDR_USI  &= ~(1<<PORT_USI_SDA); // Set SDA as input

	while (
       // SCL his high
       (PIN_USI & (1<<PIN_USI_SCL)) &&
       // and SDA is low
       !((PIN_USI & (1<<PIN_USI_SDA)))
  	);
     
	if (!(PIN_USI & (1<<PIN_USI_SDA))) {

    	// a Stop Condition did not occur
    	USICR =
         // keep Start Condition Interrupt enabled to detect RESTART
         ( 1 << USISIE ) |
         // enable Overflow Interrupt
         ( 1 << USIOIE ) |
         // set USI in Two-wire mode, hold SCL low on USI Counter overflow
         ( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
         // Shift Register Clock Source = External, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  	} else {

    	// a Stop Condition did occur
    	USICR =
         // enable Start Condition Interrupt
         ( 1 << USISIE ) |
         // disable Overflow Interrupt
         ( 0 << USIOIE ) |
         // set USI in Two-wire mode, no USI Counter overflow hold
         ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
         // Shift Register Clock Source = external, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  	} // end if

	USISR =
       // clear interrupt flags - resetting the Start Condition Flag will
       // release SCL
       ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
       ( 1 << USIPF ) |( 1 << USIDC ) |
       // set USI to sample 8 bits (count 16 external SCL pin toggles)
       ( 0x0 << USICNT0);
}
 

ISR(USI_OVF_vect)
{
   switch (USI_TWI_Overflow_State) {
     // ---------- Address mode ----------
     // Check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI.
     case USI_SLAVE_CHECK_ADDRESS:
       if ((USIDR == 0) || (( USIDR&0xfe ) == TWI_slaveAddress)) {
         if ( USIDR & 0x01 ) {
           USI_TWI_Overflow_State = USI_SLAVE_SEND_DATA;
         } else {
           USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
		 }
         SET_USI_TO_SEND_ACK();
       } else {
         SET_USI_TO_TWI_START_CONDITION_MODE();
       }
       break;
 
     // ----- Master write data mode ------
     // Check reply and goto USI_SLAVE_SEND_DATA if OK, else reset USI.
     case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
       if ( USIDR ) // If NACK, the master does not want more data.
       {
         SET_USI_TO_TWI_START_CONDITION_MODE();
         return;
       }
       // From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK
 
     // Copy data from buffer to USIDR and set USI to shift byte. Next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
     case USI_SLAVE_SEND_DATA:
 
       // Get data from Buffer       
       if (TWI_TxHead != TWI_TxTail) {
         TWI_TxTail = ( TWI_TxTail + 1 ) & TWI_TX_BUFFER_MASK;
         USIDR = TWI_TxBuf[TWI_TxTail];
       }
       else // If the buffer is empty then:
       {
           SET_USI_TO_TWI_START_CONDITION_MODE();
           return;
       }
       USI_TWI_Overflow_State = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
       SET_USI_TO_SEND_DATA();
       break;
 
     // Set USI to sample reply from master. Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
     case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
       USI_TWI_Overflow_State = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
       SET_USI_TO_READ_ACK();
       break;
 
     // ----- Master read data mode ------
     // Set USI to sample data from master. Next USI_SLAVE_GET_DATA_AND_SEND_ACK.
     case USI_SLAVE_REQUEST_DATA:
       USI_TWI_Overflow_State = USI_SLAVE_GET_DATA_AND_SEND_ACK;
       SET_USI_TO_READ_DATA();
       break;
 
     // Copy data from USIDR and send ACK. Next USI_SLAVE_REQUEST_DATA
     case USI_SLAVE_GET_DATA_AND_SEND_ACK:
       // Put data into Buffer
       
       TWI_RxHead = ( TWI_RxHead + 1 ) & TWI_RX_BUFFER_MASK;
       TWI_RxBuf[TWI_RxHead] = USIDR;
 
       USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
       SET_USI_TO_SEND_ACK();
       break;
   }
}



/*
 * - Update runtime
 * - Restart an ADC conversion
 */
ISR(TIM1_COMPA_vect)
{   	
    subs++;
    if(subs >= 10) {    // == 1 second
        subs=0;
        runtime+=1;
	if(keylock!=0) { keylock--; }
        if(usi_timeout>0) usi_timeout--;
    }	
}



ISR(ADC_vect)
{
    uint8_t muxtmp;
    uint16_t sample;

    muxtmp = ADMUX;
    muxtmp &= 0xf;
        
    sample = (uint16_t)ADCL;
    sample |= ((uint16_t)ADCH)<<8;     

    if(muxtmp == 1) {
		adc1_vbatt = sample;
		adcsequence=2;
    } else if(muxtmp == 2) {       

		adc2_isense = sample;

		if(sysstate == STATE_charge_1 || sysstate == STATE_charge_2) {
			// turn OFF boost, turn ON buck 	
	        TCCR0A=0x83;
			OCR0B=0; // for boost PWM

			/* Current regulation: */	
			
			if(iref > 0) {						
				if(adc2_isense < cIREF_CHARGE_HL) {
				//if(adc2_isense < iref) {
					if(OCR0A < 240) {
						OCR0A++;
					}
				} else if(adc2_isense > cIREF_CHARGE_HH) {
				//} else if(adc2_isense > iref) {
					if(OCR0A > 0) {
						OCR0A--;
					}
				}			
			}
			   
		} else if(sysstate == STATE_discharge_1 || sysstate == STATE_discharge_2) {
			// turn ON boost, turn OFF buck 
	    	TCCR0A=0x23;
        	OCR0A=0; // for buck PWM

			/* Current regulation: */
			if(iref > 0) {
				if(adc2_isense < cIREF_DISCHARGE_HL) {
				//if(adc2_isense < iref) {					
					if(OCR0B > 0) {
						OCR0B--;
					}
					
				} else if(adc2_isense > cIREF_DISCHARGE_HH) {
				//} else if(adc2_isense > iref) {
					if(OCR0B < 240) {
						OCR0B++;
					}
				}
			}

		} else {
			// for now turn OFF buck and boost channels
    		TCCR0A=0x03;
        	OCR0A=0; // for buck PWM
	    	OCR0B=0; // for boost PWM
		}  
		
		adcsequence=1;
    } 

    /*
     * Start ADC
     */
    
    // Init ADC
    ADMUX = cAdcV; 
    ADMUX |= adcsequence;
    ADCSRA = cRestartAdc; // Restart the ADC again   
}





/*
 * Main loop
 */
int main() 
{
    uint8_t button;
    uint8_t cmd;	

    // init portB
    PORTB = 0x08;
    DDRB = 0x07; 

    // init portA
    PORTA = 0x71; 
    DDRA = 0x80;

    keylock=0;
/*
	if(which_button_pressed()!=3) {
	    OSCCAL = eeprom_read_byte(&EE_osccal);		
    } else {
		OSCCAL = 0x54;
	}
*/
    adcsequence=1;
    iref=0;

    // Reset the runtime    
    usi_timeout=0;
    subs=0;
    runtime=0;
	
    sysstate = STATE_READY;
    batt_condition=0;
	
    // init timers
    // timer0 (PWM buck & boost) fast PWM	
    TCCR0B=0x01; // prescaling=0

    // for now turn OFF buck and boost channels	
    TCCR0A=0x03;
    //OCR0A=0; // for buck PWM
    //OCR0B=0; // for boost PWM

    //TIMSK0 = 0;


    // timer1 (ADC restart + runtime)
    TCCR1A = 0x03; // fast PWM mode 0-OCR1A
    TCCR1B = 0x1c; // prescaler /256
    //TCCR1C = 0x00;

    // 8000000 / 256 = 31250Hz.  / 31250 = 10Hz interrupt
#define DIV_FACTOR_T1 3125

    OCR1AH = (uint8_t)(DIV_FACTOR_T1 >> 8);
    OCR1AL = (uint8_t)(DIV_FACTOR_T1 & 0xFF); 

    // enable interrupts for timer 1
    TIMSK1=0x02;


    // Turn off the analog comperator to safe power:
    ACSR |= (1<<ACD);

    /*
     * Start ADC
     */
    
    // Init ADC
    ADMUX = cAdcV; 
    ADMUX |= adcsequence;
    ADCSRA = cRestartAdc; // Restart the ADC again  
		
    // Read our own TWI slave address from EEPROM address 0x00
    TWI_slaveAddress = eeprom_read_byte(&EE_slaveAddress); 

    // Init USI
    USI_TWI_Slave_Initialise();

    //wdt_enable(WDTO_2S); // enable watchdog
    WDTCSR = (1<<WDE)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0);

    /* enable interrupts: */	
    sei();		  

    /* Main loop: */
    while (1) {
    	int8_t sreg,status=0;				
        uint8_t data[8],i;

        wdt_reset();

		if( USI_TWI_Data_In_Receive_Buffer() ) {
                PORTB |= ((1<<LED_RED) | (1<<LED_GREEN) );
         	cmd = USI_TWI_Receive_Byte();			
         	switch(cmd) {
                        // consider: set state, reset
				case TWI_CMD_READALL:
                                        data[0]=TWI_slaveAddress;
                                        data[1]=sysstate;
					sreg=SREG;
					cli(); // disable interrupts
					data[2] = (uint8_t)(runtime>>8); 
					data[3] = (uint8_t)(runtime&0xff); 
					data[4] = (uint8_t)(adc1_vbatt>>8); 
					data[5] = (uint8_t)(adc1_vbatt&0xff); 
					data[6] = (uint8_t)(adc2_isense>>8); 
					data[7] = (uint8_t)(adc2_isense&0xff); 
					SREG=sreg; // sei()        			
					break;
                                case TWI_CMD_RESET:
                                        while(1); // hangup myself, let watchdog reset us
				default:
					break;
			}	
                        for(i=0;i<8;i++) {
			    if(USI_TWI_Transmit_Byte(data[i]) < 0) break;
                        }
     	        }

		switch(sysstate) {
			case STATE_READY:
				button=	which_button_pressed();			
				if(button==1) {	// do: discharge5min-cool-charge-cool-discharge-cool-charge
                                        reset_runtime();
					systimeout=runtime+TIME5MIN;
					sysstate = STATE_discharge_1;			
                                } else if(button==2) {	// do: one charge only
                                        reset_runtime();
					vbatt_track=0;
					sysstate = STATE_charge_2;
					systimeout=runtime+TIMETRACK;
				}
				break;
		 	case STATE_WAIT_COOL_AFTER_charge_1:
				if(timed_out() || which_button_pressed()==2) {
                                        reset_runtime();
					sysstate = STATE_discharge_2;
				}
				break;         	
                        case STATE_WAIT_COOL_AFTER_discharge_1:
				if(timed_out() || which_button_pressed()==2) {
                                        reset_runtime();
					vbatt_track=0;
					sysstate = STATE_charge_1;				
					systimeout=runtime+TIMETRACK;
				}
				break;
		 	case STATE_WAIT_COOL_AFTER_discharge_2:
				if(timed_out() || which_button_pressed()==2) {
                                        reset_runtime();
					vbatt_track=0;
					sysstate = STATE_charge_2;
					systimeout=runtime+TIMETRACK;
				}
				break;
		 	case STATE_charge_1:
				if((status=process_charge()) == FINISHED ||
				    which_button_pressed()==2) {

			    	sysstate=STATE_WAIT_COOL_AFTER_charge_1;
					systimeout=runtime+TIME5MIN;
				}
				break;
		 	case STATE_charge_2:
				if((status=process_charge()) == FINISHED ||
				    which_button_pressed()==2) {

			    	sysstate=batt_condition;					
				}
				break;			
		 	case STATE_discharge_1:
				if((status=process_discharge()) == FINISHED ||
                                    timed_out() || which_button_pressed()==2) {

					sysstate=STATE_WAIT_COOL_AFTER_discharge_1;
					systimeout=runtime+TIME5MIN;
				}
				break;
		 	case STATE_discharge_2:
				if((status=process_discharge()) == FINISHED ||
				    which_button_pressed()==2) {

					sysstate=STATE_WAIT_COOL_AFTER_discharge_2;
					systimeout=runtime+TIME5MIN;
				}
				break;
		 	default:
                                reset_runtime();
				if(which_button_pressed()==1) {
					sysstate = STATE_READY;
				}
				break;			
                }
		if(status<0) {
		    status=0;
                    sysstate=STATE_ERROR;
		}
                if(runtime==0xffff) { // runtime overflow after 45.5 hours: restart
                    reset_runtime();
                    sysstate = STATE_READY;
                }
		display_state();    
	}
    return 0;
}

uint8_t which_button_pressed(void)
{		
    if((PINA & (1<<PA5)) == 0 && keylock==0) {		// do all 
		keylock=2;   
		return 1;
    } else if( (PINA & (1<<PA0)) == 0 && keylock==0) {	// skip (old: do only one charge)
		keylock=2;
		return 2;
    } else {	    
		return 0;
    }
}

void reset_runtime(void)
{
    uint8_t sreg;
    sreg=SREG;
    cli(); // disable interrupts
    runtime=0;
    SREG=sreg; // sei()        			
}

/*
 * Subroutines here:
 */
int8_t process_discharge(void)
{
    int8_t status=NOT_FINISHED;
    uint8_t sreg;
    uint16_t tmp_adc;

    #define VBATT_EMPTY 350 // 2V

    sreg=SREG;
    cli(); // disable interrupts
    iref=cIREF_DISCHARGE; // this actually starts the PWM
    tmp_adc = adc1_vbatt;

    if(runtime >= DISCHARGE_THRESHOLD) {
        batt_condition = STATE_BATT_good;
    } else {
        batt_condition = STATE_BATT_bad;
    }
    SREG=sreg; // sei()

    // check Vbatt
    if(tmp_adc < 200) {
        status = ERROR; // error: no battery inserted
    } else if(tmp_adc < VBATT_EMPTY) {		
        status = FINISHED;
        iref=0;

        // for now turn OFF buck and boost channels
        TCCR0A=0x03;
        OCR0A=0; // for buck PWM
        OCR0B=0; // for boost PWM		
    } else {
        status = NOT_FINISHED;
    }
    return status;
}

int8_t process_charge(void)
{
    int8_t status=NOT_FINISHED;
    uint8_t sreg;
    uint16_t tmp_adc;

    sreg=SREG;
    cli(); // disable interrupts

    // this actually starts the PWM:
    //iref=700; // approx 180mA
    //iref=800; // approx 270mA
    //iref=900; // approx 370mA
    //iref=950;  // aprox 420mA
    iref=cIREF_CHARGE;

    tmp_adc = adc1_vbatt;
    SREG=sreg; // sei()

    // check Vbatt
    if(tmp_adc > 800) { // 820) { // 4 Volt
        status = ERROR;    // error: no battery (clamping against zener diode)
    } else {	
        sreg=SREG;
        cli(); // disable interrupts
        if(runtime >= CHARGE_TIMEOUT) {
            status = FINISHED;
        }
        if(runtime >= systimeout) { 
            systimeout=runtime+TIMETRACK;
            SREG=sreg; // sei()
	
            // 5V / 1023 is 5mV

            if(vbatt_track > 0) {
                if(tmp_adc < (vbatt_track-4)) { // -4 means 20mV delta
                    status = FINISHED;					
                }
            }
            vbatt_track=tmp_adc;

            if(status == FINISHED) {
                iref=0;

                // for now turn OFF buck and boost channels
                TCCR0A=0x03;
                OCR0A=0; // for buck PWM
                OCR0B=0; // for boost PWM
                vbatt_track=0;
            }
        } else {
            SREG=sreg; // sei()		 	    
        }
    }
    return status;
}

uint8_t timed_out(void)
{
    uint8_t sreg;
	sreg=SREG;
    cli(); // disable interrupts
    if(runtime >= systimeout) { 
        SREG=sreg; // sei()
		return 1;
    } else {
	    SREG=sreg; // sei()
		return 0;    
	}
}



void display_state(void)
{	
    PORTB &= ~( (1<<LED_RED) | (1<<LED_GREEN) );

    switch(sysstate) {
      case STATE_READY: //0
          PORTB |= ( (1<<LED_GREEN) );
          break;
      case STATE_charge_1: //2
      case STATE_charge_2: //4
          if(subs&0x04) {
              PORTB |= (1<<LED_RED);
          }
          break;
      case STATE_discharge_1: //1
      case STATE_discharge_2: //3
          PORTB |= ( (1<<LED_GREEN) | (1<<LED_RED) );
          break;
      case STATE_ERROR: //5
      case STATE_calibrate: //11
          if(subs==0) {
	      PORTB |= ( (1<<LED_GREEN) | (1<<LED_RED) );
	  }
          break;
      case STATE_BATT_bad://10
          if(subs&0x01) {
              PORTB |= (1<<LED_RED);
          }
          break;
      case STATE_BATT_good://9
          if(subs&0x01) {
              PORTB |= (1<<LED_GREEN);
          }
          break;
      default:
	//} else if(sysstate > 5 && sysstate <9) { // waiting for cool (blink green)		      
          if(subs&0x04) {
              PORTB |= (1<<LED_GREEN);
          }
          break;
    }
}

void Flush_TWI_Buffers(void)
{
    TWI_RxTail = 0;
    TWI_RxHead = 0;
    TWI_TxTail = 0;
    TWI_TxHead = 0;
}
 
//********** USI_TWI functions **********//
 
void USI_TWI_Slave_Initialise( void )
{
   Flush_TWI_Buffers();   
 
   // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
  // low when a start condition is detected or a counter overflow (only
  // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
  // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

  // Set SCL and SDA as output
  DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );

  // set SCL high
  PORT_USI |= ( 1 << PORT_USI_SCL );

  // set SDA high
  PORT_USI |= ( 1 << PORT_USI_SDA );

  // Set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  USICR =
       // enable Start Condition Interrupt
       ( 1 << USISIE ) |
       // disable Overflow Interrupt
       ( 0 << USIOIE ) |
       // set USI in Two-wire mode, no USI Counter overflow hold
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       // Shift Register Clock Source = external, positive edge
       // 4-Bit Counter Source = external, both edges
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       // no toggle clock-port pin
       ( 0 << USITC );

  // clear all interrupt flags and reset overflow counter

  USISR = ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );                                               // Clear all flags and reset overflow counter
}

int8_t USI_TWI_Transmit_Byte( uint8_t data )
{
     uint8_t tmphead;
 
     usi_timeout=5;
     tmphead = ( TWI_TxHead + 1 ) & TWI_TX_BUFFER_MASK;         // Calculate buffer index.
     while ( tmphead == TWI_TxTail && usi_timeout>0);           // Wait for free space in buffer.
     if(usi_timeout==0) return -1;
     TWI_TxBuf[tmphead] = data;                                 // Store data in buffer.
     TWI_TxHead = tmphead;                                      // Store new index.
     return 0;
}
 
uint8_t USI_TWI_Receive_Byte( void )
{         
     usi_timeout=5;
     while ( TWI_RxHead == TWI_RxTail && usi_timeout>0);
     if(usi_timeout==0) return 0;
      
     TWI_RxTail = ( TWI_RxTail + 1 ) & TWI_RX_BUFFER_MASK;        // Calculate buffer index
     return TWI_RxBuf[TWI_RxTail];                                // Return data from the buffer.
}
 
uint8_t USI_TWI_Data_In_Receive_Buffer( void )
{     
     return ( TWI_RxHead != TWI_RxTail );                 // Return 0 (FALSE) if the receive buffer is empty.
}
