#ifndef _TWI_H_
 #define _TWI_H_

void twi_init(void);
int twi_write_one(uint8_t slave_addr, uint8_t data);
int twi_read_small( int len, uint8_t *buf, uint8_t slave_addr);
int twi_read_rtc( int len, uint8_t *buf, uint8_t slave_addr, uint16_t addr,
              uint8_t addr_size);
int twi_write_rtc(int len, uint8_t *buf, uint8_t slave_addr, uint16_t addr,
              uint8_t addr_size);



#endif //_TWI_H_


