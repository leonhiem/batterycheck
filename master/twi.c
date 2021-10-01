#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <util/twi.h>		/* Note [1] */


/*
 * Maximal number of iterations to wait for a device to respond for a
 * selection.  Should be large enough to allow for a pending write to
 * complete, but low enough to properly abort an infinite loop in case
 * a slave is broken or not present at all.  With 100 kHz TWI clock,
 * transfering the start condition and SLA+R/W packet takes about 10
 * µs.  The longest write period is supposed to not exceed ~ 10 ms.
 * Thus, normal operation should not require more than 100 iterations
 * to get the device to respond to a selection.
 */
#define MAX_ITER	200
#define PAGE_SIZE 8


/*
 * Saved TWI status register, for error messages only.  We need to
 * save it in a variable, since the datasheet only guarantees the TWSR
 * register to have valid contents while the TWINT bit in TWCR is set.
 */
uint8_t twst;
extern uint8_t twi_timeout;

void twi_init(void)
{
    /* Initialize TWI */
    TWBR = 0xff; 
    //TWSR = 0x3;
    //TWSR = 0x2; 
    TWSR = 0x1; 
}




int twi_write_one(uint8_t slave_addr, uint8_t data)
{
    uint8_t n = 0;
    int rv = 0;

restart:
    if (n++ >= MAX_ITER) return -1;

begin:

    /* Note [15] */
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_REP_START:		/* OK, but should not happen */
       case TW_START:
         break;

       case TW_MT_ARB_LOST:
         goto begin;

       default:
         return -1;		/* error: not in start condition */
				/* NB: do /not/ send stop condition */
    }

    /* send SLA+W */
    TWDR = slave_addr;
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_MT_SLA_ACK:
         break;

       case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
         goto restart;

       case TW_MT_ARB_LOST:	/* re-arbitrate */
         goto begin;

       default:
         goto error;		/* must send stop condition */
    }

    

    TWDR = data; 
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transm */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
      case TW_MT_DATA_ACK:
        break;

      case TW_MT_DATA_NACK:
        goto quit;

      case TW_MT_ARB_LOST:
        goto begin;

      default:
        goto error;		/* must send stop condition */
    }
    
quit:
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

    return rv;

error:
    rv = -1;
    goto quit;
}



int twi_read_small( int len, uint8_t *buf, uint8_t slave_addr)
{
    uint8_t twcr;
    int rv = 0;


begin:
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_REP_START:          /* OK, but should not happen */
       case TW_START:
         break;

       case TW_MT_ARB_LOST:        /* Note [9] */
         goto begin;

       default:
         return -1;                /* error: not in start condition */
                                   /* NB: do /not/ send stop condition */
    }

    /* send SLA+R */
    TWDR = slave_addr | 1;
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
      case TW_MR_SLA_ACK:
        break;

      case TW_MR_SLA_NACK:
        goto quit;

      case TW_MR_ARB_LOST:
        goto begin;

      default:
        goto error;
    }


    for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) /* Note [13] */;
         len > 0;
         len--)
    {
        if (len == 1) twcr = _BV(TWINT) | _BV(TWEN); /* send NAK this time */

        TWCR = twcr;		/* clear int to start transmission */
        twi_timeout=3;
        while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
        switch ((twst = TW_STATUS)) {
  	  case TW_MR_DATA_NACK:
	    len = 0;		/* force end of loop */
	    /* FALLTHROUGH */
	  case TW_MR_DATA_ACK:
	    *buf++ = TWDR;
	    rv++;
	    break;

	  default:
	    goto error;
	}
    }
quit:

    /* Note [14] */
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

    return rv;

error:
    rv = -1;
    goto quit;
}

int twi_read_rtc( int len, uint8_t *buf, uint8_t slave_addr, uint16_t addr,
              uint8_t addr_size)
{
    uint8_t sla, twcr, n = 0;
    int rv = 0;

  /* patch high bits of EEPROM address into SLA */
  //sla = TWI_SLA_24CXX | (((eeaddr >> 8) & 0x07) << 1);

    sla=slave_addr;

restart:
    if (n++ >= MAX_ITER) return -1;

begin:
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_REP_START:          /* OK, but should not happen */
       case TW_START:
         break;

       case TW_MT_ARB_LOST:        /* Note [9] */
         goto begin;

       default:
         return -1;                /* error: not in start condition */
                                   /* NB: do /not/ send stop condition */
    }

    /* send SLA+W */
    TWDR = sla | TW_WRITE;
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_MT_SLA_ACK:
         break;

       case TW_MT_SLA_NACK:        /* nack during select: device busy writing */
                                   /* Note [11] */
         goto restart;

       case TW_MT_ARB_LOST:        /* re-arbitrate */
         goto begin;

       default:
         goto error;               /* must send stop condition */
    }
    if(addr_size == 2) {           /* 16 bit address, so send MSB first */
        TWDR = (uint8_t)(addr >> 8);
        TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transm */
        twi_timeout=3;
        while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
        switch ((twst = TW_STATUS)) {
          case TW_MT_DATA_ACK:
            break;

          case TW_MT_DATA_NACK:
            goto quit;
      
          case TW_MT_ARB_LOST:
            goto begin;

          default:
            goto error;               /* must send stop condition */
          }
    }

    TWDR = (uint8_t)(addr & 0xff);    /* LSB part of address */
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transm */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
      case TW_MT_DATA_ACK:
        break;

      case TW_MT_DATA_NACK:
        goto quit;
    
      case TW_MT_ARB_LOST:
        goto begin;

      default:
        goto error;               /* must send stop condition */
    }



    /*
     * Next cycle(s): master receiver mode
     */
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send rep. start condition */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
      case TW_START:              /* OK, but should not happen */
      case TW_REP_START:
        break;

      case TW_MT_ARB_LOST:
        goto begin;

      default:
        goto error;
    }


    /* send SLA+R */
    TWDR = sla | TW_READ;
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
      case TW_MR_SLA_ACK:
        break;

      case TW_MR_SLA_NACK:
        goto quit;

      case TW_MR_ARB_LOST:
        goto begin;

      default:
        goto error;
    }


    for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) /* Note [13] */;
         len > 0;
         len--)
    {
        if (len == 1) twcr = _BV(TWINT) | _BV(TWEN); /* send NAK this time */

        TWCR = twcr;            /* clear int to start transmission */
        twi_timeout=3;
        while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
        switch ((twst = TW_STATUS)) {
          case TW_MR_DATA_NACK:
            len = 0;            /* force end of loop */
            /* FALLTHROUGH */
          case TW_MR_DATA_ACK:
            *buf++ = TWDR;
            rv++;
            break;

          default:
            goto error;
        }
    }
quit:

    /* Note [14] */
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

    return rv;

error:
    rv = -1;
    goto quit;
}


int twi_write_rtc(int len, uint8_t *buf, uint8_t slave_addr, uint16_t addr,
              uint8_t addr_size)
{
    uint8_t sla, n = 0;
    int rv = 0;
    uint16_t endaddr;

    if (addr + len < (addr | (PAGE_SIZE - 1)))
      endaddr = addr + len;
    else
      endaddr = (addr | (PAGE_SIZE - 1)) + 1;
    len = endaddr - addr;

  /* patch high bits of EEPROM address into SLA */
  // sla = TWI_SLA_24CXX | (((eeaddr >> 8) & 0x07) << 1);
  sla = slave_addr;

restart:
    if (n++ >= MAX_ITER) return -1;

begin:

    /* Note [15] */
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_REP_START:               /* OK, but should not happen */
       case TW_START:
         break;

       case TW_MT_ARB_LOST:
         goto begin;

       default:
         return -1;             /* error: not in start condition */
                                /* NB: do /not/ send stop condition */
    }

    /* send SLA+W */
    TWDR = sla | TW_WRITE;
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
       case TW_MT_SLA_ACK:
         break;

       case TW_MT_SLA_NACK:     /* nack during select: device busy writing */
         goto restart;

       case TW_MT_ARB_LOST:     /* re-arbitrate */
         goto begin;

       default:
         goto error;            /* must send stop condition */
    }

    if(addr_size == 2) {           /* 16 bit address, so send MSB first */
        TWDR = (uint8_t)(addr >> 8);
        TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transm */
        twi_timeout=3;
        while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
        switch ((twst = TW_STATUS)) {
          case TW_MT_DATA_ACK:
            break;

          case TW_MT_DATA_NACK:
            goto quit;

          case TW_MT_ARB_LOST:
            goto begin;

          default:
            goto error;         /* must send stop condition */
        }
    }

    TWDR = (uint8_t)(addr & 0xff); /* LSB part of address */
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transm */
    twi_timeout=3;
    while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
    switch ((twst = TW_STATUS)) {
      case TW_MT_DATA_ACK:
        break;

      case TW_MT_DATA_NACK:
        goto quit;

      case TW_MT_ARB_LOST:
        goto begin;

      default:
        goto error;             /* must send stop condition */
    }


    for (; len > 0; len--) {
        TWDR = *buf++;

        TWCR = _BV(TWINT) | _BV(TWEN); /* start transmission */
        twi_timeout=3;
        while ((TWCR & _BV(TWINT)) == 0) { if(twi_timeout==0) { goto error; } } /* wait for transmission */
        switch ((twst = TW_STATUS)) {
          case TW_MT_DATA_NACK:
            goto error;         /* device write protected -- Note [16] */

          case TW_MT_DATA_ACK:
            rv++;
            break;

          default:
            goto error;
        }
    }
quit:
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

    return rv;

error:
    rv = -1;
    goto quit;
}

