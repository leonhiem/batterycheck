/*
 * Project:     Master batterytester
 * Hardware:    ATmega16 @ Mainboard_V2 (NOV-2009)
 * Author:      L.Hiemstra@gmail.com
 * last change: JUN-2012
 */


#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define F_OSC 4000000
#define F_CPU F_OSC
#include "delay.h"
#include "uart.h"
#include "twi.h"




/* ADC defenitions */
#define cStartAdc    ((1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0))
#define cRestartAdc  ((cStartAdc)|(1<<ADSC))
#define cAdcV        (1<<REFS0) 


uint8_t subsadc;
uint8_t subs;
uint8_t twi_timeout;
uint32_t uptime;

uint8_t sysstate;

uint8_t data[10];




/*
 * - Update uptime
 * - Restart an ADC conversion
 */
ISR(TIMER2_COMP_vect)
{
	uint8_t adcflag;
	uint8_t muxtmp;

	adcflag=0;
	subsadc++;
	if(subsadc == 8) { adcflag=1; subsadc=0; }

        subs++;
        if(subs >= 125) {    // == 1 second
                subs=0;
                uptime+=1;
                //if(twi_timeout>0) twi_timeout--;
        }

	if(adcflag) {
		// Switch ADC channel by changing ADCMUX (no pref)
		muxtmp = ADMUX;
		muxtmp++;
		muxtmp &= 0x7;   // wrap around if > 7
                if(muxtmp >= 4) { muxtmp=0; }
		muxtmp |= cAdcV; // set ADC voltage (basic setting)
		ADMUX = muxtmp;  // update ADC mux

		ADCSRA = cRestartAdc; // Restart the ADC again
	}
}


/*
 * - Run the schedular
 */
ISR(TIMER0_COMP_vect)
{
	// not in use anymore
}


/*
 * - Read the ADC sample
 */
ISR(ADC_vect)
{
	uint8_t muxtmp;
	uint8_t lsb,msb;
	uint16_t sample;
	
	muxtmp = ADMUX;
	muxtmp &= 0x7;

	lsb = ADCL; // read the low part
	msb = ADCH; // read the high part

	sample = (uint16_t)lsb;
	sample |= ((uint16_t)msb)<<8;

	if(muxtmp < 4) {
	    //adc[muxtmp] = sample;
        }
}


/*
 * Do all the startup-time peripheral initializations.
 */
static void ioinit(void)
{

    // Init ADC
    ADMUX = cAdcV;
    ADCSRA = cStartAdc;


    // init timers
    // timer0 (schedular)
    TCCR0 = 0xb; // compare | set TC0 prescaler to 64
    OCR0 = 0x64; // 100 Hz

    // timer1 (PWM channels OC1A and OC1B)
    TCCR1A = 0xa2;
    TCCR1B = 0x19; // max prescaler frequency; top=ICR1 // about 15kHz

    ICR1H = 0; // use ICR register as a top value (0xf is max value)
    ICR1L = 0xff;

    TCNT1H = 0;
    TCNT1L = 0;

    OCR1AH = 0;
    OCR1AL = 1; // preset heater 1 at 0V

    OCR1BH = 0;
    OCR1BL = 1; // preset heater 2 at 0V

    // timer 2 (ADC restart)
    TCCR2 = 0xe; // compare | TC0 Prescaler divider to 256
    OCR2 = 0x7c; // 125 Hz

    // interrupt enable for timers 0 and 2
    TIMSK = 0x82;



    // init IO ports
    // init portA
    DDRA = 0xf0; // bits 0..3 input; bits 4..7 output
    asm volatile ("nop");
    PORTA = 0x0c; 

    // init portB
    DDRB  = 0x0f; // bits 0..3 output; bits 4..7 input
    asm volatile ("nop");
    PORTB = 0xf0; // activate pullups

    // init portC
    // Alarm2=bit7
    // Alarm1=bit6
    // Alarm0=bit5
    DDRC = 0xff;  // all bits are output
    asm volatile ("nop");
    PORTC = 0x1f;

    // init portD
    DDRD = 0xbe;  
    asm volatile ("nop");
    PORTD = 0xff; // activate pullup for RxD


    // raise the alarms for bootup
    PORTC |= (1<<PC7) | (1<<PC6) | (1<<PC5);

    // Wait until the display device is safely powered up
    delay_ms(300); // power-up-delay

    // unraise the alarms for bootup
    PORTC &= ~( (1<<PC7) | (1<<PC6) | (1<<PC5) );

    delay_ms(100); // power-up-delay


    uart_init();
    twi_init();
}


void uart_switch2listen(void);
uint8_t uart_is_busy(void);
void uart_send_HK_packet(uint8_t dest);
void uart_send_time_packet(uint8_t,uint8_t);
uint8_t bcd_inc_to_6(uint8_t *hour);
uint8_t bcd_inc_to_24(uint8_t *hour) ;
uint8_t bcd_inc_to_60(uint8_t *hour) ;
uint8_t bcd_inc(uint8_t *hour);
void process_nrhours(void);
void process_nrminutes(void);
uint8_t rtc_bcd2bin(uint8_t *rtc, uint8_t size);
uint8_t conv2bcd (uint8_t x);
uint8_t bcd_test(uint8_t bcd);
int8_t open_file(char *fname);
int8_t write2file(char *str);
void close_file(char *fname);


FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(void)
{
    uint8_t i;    
    uint8_t rtc[7], rtc_old=0xff;
    int fileok=0;
    
    // This is required on the ATmega16 to get PORTC working:
    MCUCSR = 0x80;
    MCUCSR = 0x80;

    /*
     * Set initial sysstate to pessimistic start:
     * - booting
     * - curr tubeset = 0
     * - curr humidity = 0
     * - tubesensor absent
     * - ambient sensor absent
     */
    sysstate = 0xea; // booting
    
    ioinit();

    stdout = stdin = &uart_str;

    // Reset the uptime
    twi_timeout=0;
    subsadc=0;
    subs=0;
    uptime=0UL;

#define cSEEPROM_addr_Hours 0
#define cSEEPROM_addr_RTC   5

#define cSEEPROM_slave_addr 0xa0
#define cRTC_slave_addr     0xd0
#define TWI_slaveAddress    0x10

#define TWI_CMD_VREQ  		0xa5
#define TWI_CMD_TREQ  		0xa6
#define TWI_CMD_READALL 	0xa7
#define TWI_CMD_RESET           0xa8


    sei();      /* enable interrupts */

    delay_ms(1000); 		
    wdt_reset();

    // read current time
    if(twi_read_rtc(7,rtc,cRTC_slave_addr,0,1) != 0) {
        // check the CH bit in the SECONDS register
        // if this bit is a '1' then the RTC is not running: reset it then
        if((rtc[0]&0x80)) {
            rtc[0] &= 0x7f; // reset CH bit
            if(twi_write_rtc(7,rtc,cRTC_slave_addr,0,1) < 0) {
                twi_write_rtc(7,rtc,cRTC_slave_addr,0,1); // retry once
            }
        }
    }

    PORTB &= ~( (1<<PB1) | (1<<PB2) | (1<<PB3) );
    
    for(;;) {   // main loop		
        uint8_t sla,a,errorstate;
        uint16_t runtime,vbatt,isense;
        double vbattf;
        char buf[80];
        char filename[14]="YYMMDDHH.txt";
        char *sysstate_str=NULL;


        while(bit_is_clear(PINA,PA2)) { // while jumper is in
            int s[7];
            uint8_t srtc[7],i;
            char *buf_ptr;

            wdt_disable(); // disable watchdog
            PORTB |= (1<<PB3);
            fprintf(stdout, "\nSet date (yyyy-mm-dd hh:mm:ss) (24h format):");
            if (fgets(buf, sizeof buf - 1, stdin) == NULL)
                break;
            fprintf(stdout, "Received:%s\n",buf);

            for(i=0;i<7;i++) { s[i]=0; }

            buf_ptr = buf;
            s[6] =atoi(buf_ptr); // year
            buf_ptr+=5;
            s[5]  =atoi(buf_ptr);// mon
            buf_ptr+=3;
            s[4]  =atoi(buf_ptr);// day
            buf_ptr+=3;
            s[2] =atoi(buf_ptr); // hour
            buf_ptr+=3;
            s[1]  =atoi(buf_ptr); // min
            buf_ptr+=3;
            s[0]  =atoi(buf_ptr); // sec

                srtc[6]=conv2bcd((uint8_t)(s[6]-2000));
                srtc[5]=conv2bcd((uint8_t)(s[5]));
                srtc[4]=conv2bcd((uint8_t)(s[4]));
                srtc[3]=0; // dont care about day of week
                srtc[2]=conv2bcd((uint8_t)(s[2]));
                srtc[1]=conv2bcd((uint8_t)(s[1]));
                srtc[0]=conv2bcd((uint8_t)(s[0]));

            fprintf(stdout, 
                    "Writing to RTC: 20%02x-%02x-%02x %02x:%02x:%02x\n",
                    srtc[6],srtc[5],srtc[4],srtc[2],srtc[1],srtc[0]);

            twi_write_rtc(7,srtc,cRTC_slave_addr,0,1);
        }
        PORTB &= ~(1<<PB3);
        wdt_enable(WDTO_2S); // enable watchdog
        wdt_reset();
        errorstate=0;
        if(twi_read_rtc(7,rtc,cRTC_slave_addr,0,1) >= 0) {
            if(rtc[2] != rtc_old && rtc_old!=0xff && fileok==1) { 
                close_file(filename); fileok=0; 
            }
            rtc_old=rtc[2];
            sprintf(filename, "%02x%02x%02x%02x.txt",rtc[6],rtc[5],rtc[4],rtc[2]); 

            if(bit_is_set(PINA,PA3) && fileok==0) {
                fileok=open_file(filename);
            }

            for(a=0;a<28;a++) {
                sla=TWI_slaveAddress+(a*2);

                for(i=0;i<10;i++) data[i]=0; 

                PORTB |= (1<<PB2);
                if(twi_write_one(sla,TWI_CMD_READALL) >= 0) {
                    delay_ms(100);
        
                    wdt_reset();

                    if(twi_read_small(8,data,sla) >= 0) {

                        runtime=((uint16_t)data[2]<<8) | ((uint16_t)data[3] &0xff);
                        vbatt=((uint16_t)data[4]<<8) | ((uint16_t)data[5] &0xff);
                        isense=((uint16_t)data[6]<<8) | ((uint16_t)data[7] &0xff);
                        vbattf=(((double)vbatt) * 5.0);
                        vbattf=vbattf / 1023.0;

                        switch(data[1]) {
                            case 0:
                               sysstate_str="ready";
                               break;
                            case 1:
                               sysstate_str="dis_1";
                               break;
                            case 2:
                               sysstate_str="cha_1";
                               break;
                            case 3:
                               sysstate_str="dis_2";
                               break;
                            case 4:
                               sysstate_str="cha_2";
                               break;
                            case 5:
                               sysstate_str="error";
                               break;
                            case 6:
                            case 7:
                            case 8:
                               sysstate_str="cool ";
                               break;
                            case 9:
                               sysstate_str="good ";
                               break;
                            case 10:
                               sysstate_str="bad  ";
                               break;
                            case 11:
                               sysstate_str="calib";
                               break;
                        }

                        delay_ms(50);
                        if(twi_read_rtc(7,rtc,cRTC_slave_addr,0,1) >= 0) {

                            sprintf(buf,
                            "%x-%02x-%02x %02x:%02x:%02x 0x%02x %s %05d %.02f %d\n",
                            0x2000+(uint16_t)rtc[6],rtc[5],rtc[4],rtc[2],rtc[1],rtc[0],
                            data[0],sysstate_str,runtime,vbattf,isense);

                            if(fileok==1) {
                                write2file(buf);
                            } else {
                                fprintf(stdout, "%s",buf);
                            }
                            PORTB &= ~(1<<PB2);

                            delay_ms(500); 		
                            wdt_reset();

                            if(fileok==1 && bit_is_clear(PINA,PA3)) { 
                                close_file(filename); fileok=0; 
                            }
                        } else { errorstate=1; }
                    } else { errorstate=2; }
                } else { errorstate=3; }


                if(errorstate) { goto need_reset; }
            }
        } else { errorstate=4; }

        if(errorstate) { goto need_reset; }
    }
need_reset:
    while(1); // reset myself by watchdog

    return 0;
}

int8_t open_file(char *fname)
{
    char buf[32];
    // Sync first
    
    uart_RTS(0); // take module out of reset
    wdt_reset();
    delay_ms(1000); 		
    wdt_reset();
    delay_ms(1000); 		
    wdt_reset();
    uart_getchars(buf,32); // flush
    wdt_reset();
    uart_getchars(buf,32); // flush
    wdt_reset();
    uart_getchars(buf,32); // flush
    wdt_reset();

    fprintf(stdout,"E\r");
    uart_getchars(buf,32); // flush
    wdt_reset();
    delay_ms(500); 		
    wdt_reset();
    fprintf(stdout,"e\r");
    uart_getchars(buf,32); // flush
    wdt_reset();
    delay_ms(500); 		
    wdt_reset();
    uart_getchars(buf,32); // flush
    wdt_reset();

    fprintf(stdout,"\r");
    uart_getchars(buf,32); // flush
    wdt_reset();
    if(buf[0]=='N') {
        return -1; // no drive
    }
    uart_getchars(buf,32); // flush
    wdt_reset();
    delay_ms(500); 		
    wdt_reset();

    fprintf(stdout,"OPW %s\r",fname);
    uart_getchars(buf,32); // flush
    wdt_reset();

    PORTB |= (1<<PB1);

    return 1;
}

int8_t write2file(char *str)
{
    char buf[90];
    int len=strlen(str);
    char *bufptr=buf;
    int i;

    *bufptr++='W';
    *bufptr++='R';
    *bufptr++='F';
    *bufptr++=' ';
    *bufptr++=0;
    *bufptr++=0;
    *bufptr++=0;
    *bufptr++=len;
    *bufptr++='\r'; 
    memcpy((void *)bufptr,(void *)str,len);
    bufptr+=len;
    *bufptr++='\r';
    *bufptr++=0;

    bufptr=buf;
    len=10+len;
    wdt_reset();
    for(i=0;i<len;i++) {
        uart_putchar_raw(*bufptr++);
    }
    delay_ms(500); 		
    wdt_reset();

    uart_getchars(buf,90); // flush
    wdt_reset();
    return 0;
}

void close_file(char *fname)
{
    char buf[32];
    wdt_reset();
    delay_ms(700); 		
    wdt_reset();
    fprintf(stdout,"CLF %s\r",fname);
    delay_ms(1000); 		
    wdt_reset();
    uart_getchars(buf,32); // flush
    wdt_reset();
    PORTB &= ~(1<<PB1);
}


/*
 * returns 1 if carry, otherwise 0
 */
uint8_t bcd_inc(uint8_t *hour) 
{
	uint8_t tmp = *hour;

	if(tmp >= 0x99) {
		*hour=0;
		return 1;
	} else {
		tmp &= 0xf;
		if(tmp < 0x9) {
			*hour+=1;
			return 0;
		} else {
			*hour+=7;
			return 0;
		}
	}
}

/*
 * returns 1 if carry, otherwise 0
 */
uint8_t bcd_inc_to_6(uint8_t *hour) 
{
	if(*hour < 5) {
		*hour+=1;
		return 0;
	} else {
		*hour=0;
		return 1;
	}
}

/*
 * returns 1 if carry, otherwise 0
 */
uint8_t bcd_inc_to_60(uint8_t *hour) 
{
        uint8_t tmp = *hour;

        if(tmp >= 0x59) {
                *hour=0;
                return 1;
        } else {
                tmp &= 0xf;
                if(tmp < 0x9) {
                        *hour+=1;
                        return 0;
                } else {
                        *hour+=7;
                        return 0;
                }
        }
}

/*
 * returns 1 if carry, otherwise 0
 */
uint8_t bcd_inc_to_24(uint8_t *hour) 
{
        uint8_t tmp = *hour;

        if(tmp >= 0x23) {
                *hour=0;
                return 1;
        } else {
                tmp &= 0xf;
                if(tmp < 0x9) {
                        *hour+=1;
                        return 0;
                } else {
                        *hour+=7;
                        return 0;
                }
        }
}

uint8_t bcd_test(uint8_t bcd)
{
	uint8_t tmp=bcd;
	tmp &= 0xf;
	if(tmp > 9) return 1; // not bcd !

	tmp=bcd;
	tmp >>= 4;
	tmp &= 0xf;
	if(tmp > 9) return 1; // not bcd !

	return 0; // ok
}

uint8_t conv2bcd (uint8_t x)
{
        uint8_t p = 0;
        uint8_t c = 0;
        uint8_t i;

        for (i = x; i != 0; i = i / 10) {
                p |= (i % 10) << c;
                c += 4;
        }
        return p;
}

