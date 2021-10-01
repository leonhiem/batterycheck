/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * Stdio demo, UART implementation
 *
 * $Id: uart.c 1008 2005-12-28 21:38:59Z joerg_wunsch $
 */


#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>

#define F_OSC F_CPU

#include "delay.h"

#include "uart.h"


/*
 * Initialize the UART to 9600 Bd, tx/rx, 8N1.
 */
void
uart_init(void)
{
//  UBRRL = (F_CPU / (16UL * UART_BAUD)) - 1;
 // UCSRB = _BV(TXEN) | _BV(RXEN); /* tx/rx enable */
#define UBRR_VAL  (F_CPU/16/UART_BAUD - 1)
  UBRRH = (uint8_t)(UBRR_VAL >> 8);
  UBRRL = (uint8_t)(UBRR_VAL & 0xFF);

  PORTD &= ~(1<<PD2); // uart in listening mode


//  UCSRB = (1<<UDRIE)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE); 
//  UCSRC=(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
  UCSRB = _BV(TXEN) | _BV(RXEN); /* tx/rx enable */


}

/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
int
uart_putchar(char c, FILE *stream)
{
/*
  if(bit_is_set(PINA,PA2)) {
      while(bit_is_clear(PIND,PD6)); // wait until ready
  }
*/
  if (c == '\a')
    {
      fputs("*ring*\n", stderr);
      return 0;
    }

  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSRA, UDRE);
  UDR = c;

  return 0;
}

int uart_putchar_raw(char c)
{
  loop_until_bit_is_set(UCSRA, UDRE);
  UDR = c;
  return 0;
}

/*
 * Receive a character from the UART Rx.
 *
 * This features a simple line-editor that allows to delete and
 * re-edit the characters entered, until either CR or NL is entered.
 * Printable characters entered will be echoed using uart_putchar().
 *
 * Editing characters:
 *
 * . \b (BS) or \177 (DEL) delete the previous character
 * . ^u kills the entire input buffer
 * . ^w deletes the previous word
 * . ^r sends a CR, and then reprints the buffer
 * . \t will be replaced by a single space
 *
 * All other control characters will be ignored.
 *
 * The internal line buffer is RX_BUFSIZE (80) characters long, which
 * includes the terminating \n (but no terminating \0).  If the buffer
 * is full (i. e., at RX_BUFSIZE-1 characters in order to keep space for
 * the trailing \n), any further input attempts will send a \a to
 * uart_putchar() (BEL character), although line editing is still
 * allowed.
 *
 * Input errors while talking to the UART will cause an immediate
 * return of -1 (error indication).  Notably, this will be caused by a
 * framing error (e. g. serial line "break" condition), by an input
 * overrun, and by a parity error (if parity was enabled and automatic
 * parity recognition is supported by hardware).
 *
 * Successive calls to uart_getchar() will be satisfied from the
 * internal buffer until that buffer is emptied again.
 */
int
uart_getchar(FILE *stream)
{
  uint8_t c;
  char *cp, *cp2;
  static char b[RX_BUFSIZE];
  static char *rxp;


  if (rxp == 0)
    for (cp = b;;)
      {
	loop_until_bit_is_set(UCSRA, RXC);
	if (UCSRA & _BV(FE)) {
	  return _FDEV_EOF;
        }
	if (UCSRA & _BV(DOR)) {
	  return _FDEV_ERR;
        }
	c = UDR;


	/* behaviour similar to Unix stty ICRNL */
	if (c == '\r')
	  c = '\n';
	if (c == '\n')
	  {
	    *cp = c;
	    uart_putchar(c, stream);
	    rxp = b;
	    break;
	  }
	else if (c == '\t')
	  c = ' ';

	if ((c >= (uint8_t)' ' && c <= (uint8_t)'\x7e') ||
	    c >= (uint8_t)'\xa0')
	  {
	    if (cp == b + RX_BUFSIZE - 1)
	      uart_putchar('\a', stream);
	    else
	      {
		*cp++ = c;
		uart_putchar(c, stream);
	      }
	    continue;
	  }

	switch (c)
	  {
	  case 'c' & 0x1f:
	    return -1;

	  case '\b':
	  case '\x7f':
	    if (cp > b)
	      {
		uart_putchar('\b', stream);
		uart_putchar(' ', stream);
		uart_putchar('\b', stream);
		cp--;
	      }
	    break;

	  case 'r' & 0x1f:
	    uart_putchar('\r', stream);
	    for (cp2 = b; cp2 < cp; cp2++)
	      uart_putchar(*cp2, stream);
	    break;

	  case 'u' & 0x1f:
	    while (cp > b)
	      {
		uart_putchar('\b', stream);
		uart_putchar(' ', stream);
		uart_putchar('\b', stream);
		cp--;
	      }
	    break;

	  case 'w' & 0x1f:
	    while (cp > b && cp[-1] != ' ')
	      {
		uart_putchar('\b', stream);
		uart_putchar(' ', stream);
		uart_putchar('\b', stream);
		cp--;
	      }
	    break;
	  }
      }

  c = *rxp++;
  if (c == '\n')
    rxp = 0;

  return c;
}

int uart_getchars(char *buf, int len)
{
  char *cp=buf;
  char c;
  int i,stat=0;
  uint16_t timeout;


  for(i=0;i<len;i++) {
      //loop_until_bit_is_set(UCSRA, RXC);
      timeout=1000;
      while(bit_is_clear(UCSRA, RXC)) {
          timeout--;
          if(timeout==0) { goto quit; }
          delay_ms(1);
      }

      if (UCSRA & _BV(FE)) {
          stat= _FDEV_EOF;
          goto quit;
      }
      if (UCSRA & _BV(DOR)) {
          stat= _FDEV_ERR;
          goto quit;
      }
      c = UDR;
      stat++;
      *cp++=c;


      if (c == '\r') break;
  }
quit:
  return stat;
}

void uart_RTS(int state)
{
    if(state==0) {
       PORTD &= ~(1<<PD7); // ready to receive
    } else {
       PORTD |= (1<<PD7); // not ready to receive
    }
}

