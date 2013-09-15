/*************************************************** 
  This is a library for the Radioduino

  Designed specifically to work with the Radioduino
  ----> http://www.


  author: Heye Everts (heye.everts.1@gmail.com) 
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/


#include "rfm73.h"
#include <WProgram.h>
#include <util/delay.h>

// this is new
void (*rfm73::user_onReceive)(void);


///////////////////////////////////////////////////////////////////////////////
// Register initialization values and command macros //
///////////////////////////////////////////////////////////////////////////////
 
//************ Address definition commands
const uint8_t PROGMEM rfm73_cmd_adrRX0[] = { (0x20|0x0A), 0x34,0x43,0x10,0x10,0x01};
const uint8_t PROGMEM rfm73_cmd_adrTX[]  = { (0x20|0x10), 0x34,0x43,0x10,0x10,0x01};
const uint8_t PROGMEM rfm73_cmd_adrRX1[] = { (0x20|0x0B), 0x35,0x43,0x10,0x10,0x02};
 
//************ Bank0 register initialization commands
 const uint8_t PROGMEM rfm73_bank0Init[][2] = {
  // address data
  { (0x20|0x00), 0x0F }, //Disable CRC ,CRC=1byte, POWER UP, TX
  { (0x20|0x01), 0x3F }, //Enable auto acknowledgement data pipe0-5
  { (0x20|0x02), 0x3F }, //Enable RX Addresses pipe0-5
  { (0x20|0x03), 0x03 }, //RX/TX address field width 5byte
  { (0x20|0x04), 0x08 }, //x = 250 ms = 4000ms, y = 15 tries
  { (0x20|0x05), 0x17 }, //channel = 0x17
  //{ (0x20|0x06), 0x3F }, //air data rate-2M,out power 5dbm,setup LNA gain high (0dBM)
  { (0x20|0x06), 0x0F }, //init register 6 for rfm73
  { (0x20|0x07), 0x07 }, //
  { (0x20|0x08), 0x00 }, //
  { (0x20|0x09), 0x00 }, //
  { (0x20|0x0C), 0xc3 }, //LSB Addr pipe 2
  { (0x20|0x0D), 0xc4 }, //LSB Addr pipe 3
  { (0x20|0x0E), 0xc5 }, //LSB Addr pipe 4
  { (0x20|0x0F), 0xc6 }, //LSB Addr pipe 5
  { (0x20|0x11), 0x20 }, //Payload len pipe0
  { (0x20|0x12), 0x20 }, //Payload len pipe0
  { (0x20|0x13), 0x20 }, //Payload len pipe0
  { (0x20|0x14), 0x20 }, //Payload len pipe0
  { (0x20|0x15), 0x20 }, //Payload len pipe0
  { (0x20|0x16), 0x20 }, //Payload len pipe0
  { (0x20|0x17), 0x20 }, //Payload len pipe0
  { (0x20|0x1C), 0x3F }, //Enable dynamic payload legth data pipe0-5
  { (0x20|0x1D), 0x07 } //Enables Dynamic Payload Length,Enables Payload with ACK
};
 
//************ Bank1 register initialization commands
 const uint8_t PROGMEM rfm73_bank1Init[][5] = {
  // address data
  { (0x20|0x00), 0x40, 0x4B, 0x01, 0xE2 },
  { (0x20|0x01), 0xC0, 0x4B, 0x00, 0x00 },
  { (0x20|0x02), 0xD0, 0xFC, 0x8C, 0x02 },
  { (0x20|0x03), 0x99, 0x00, 0x39, 0x41 },
  //{ (0x20|0x04), 0xb9, 0x9E, 0x86, 0x0B }, // b9? f9?
  { (0x20|0x04), 0xD9, 0x96, 0x82, 0x1B }, //new register for rfm73 b9? d9?
  //{ (0x20|0x04), 0x1B, 0x82, 0x96, 0xD9 }, //new register for rfm73 b9? d9?
  //{ (0x20|0x05), 0x24, 0x06, 0x7F, 0xA6 },	//original
  //{ (0x20|0x05), 0x28, 0x02, 0x7F, 0xA6 },	//bank1 register 5 = rssi settings = -96dbm
  { (0x20|0x05), 0x3C, 0x02, 0x7F, 0xA6 },	//bank1 register 5 = rssi settings = -71dbm - best
  { (0x20|0x06), 0x00, 0x00, 0x00, 0x00 },
  { (0x20|0x07), 0x00, 0x00, 0x00, 0x00 },
  { (0x20|0x08), 0x00, 0x00, 0x00, 0x00 },
  { (0x20|0x09), 0x00, 0x00, 0x00, 0x00 },
  { (0x20|0x0a), 0x00, 0x00, 0x00, 0x00 },
  { (0x20|0x0b), 0x00, 0x00, 0x00, 0x00 },
  { (0x20|0x0C), 0x00, 0x12, 0x73, 0x00 },
  //{ (0x20|0x0D), 0x36, 0xb4, 0x80, 0x00 }
  { (0x20|0x0D), 0x46, 0xb4, 0x80, 0x00 }
};
 
//************ Bank1 register 14 initialization commands
 const uint8_t PROGMEM rfm73_bank1R0EInit[] = {
  // address Data...
  (0x20|0x0E), 0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF
};
 
//************ other commands: { <command>, <data>, ... }
const uint8_t PROGMEM rfm73_cmd_switch_cfg[] = { 0x50, 0x53 }; // switch Register Bank
const uint8_t PROGMEM rfm73_cmd_flush_rx[] = { 0xe2, 0x00 }; // flush RX FIFO
const uint8_t PROGMEM rfm73_cmd_flush_tx[] = { 0xe1, 0x00 }; // flush TX FIFO
const uint8_t PROGMEM rfm73_cmd_activate[] = { 0x50, 0x73 }; // Activation command
const uint8_t PROGMEM rfm73_cmd_tog1[]={ (0x20|0x04), 0xd9 | 0x06, 0x9e, 0x86, 0x0b }; //assosciated with set1[4]!
const uint8_t PROGMEM rfm73_cmd_tog2[]={ (0x20|0x04), 0xd9 & ~0x06, 0x9e, 0x86, 0x0b}; //assosciated with set1[4]!
 




///////////////////////////////////////////////////////////////////////////////
// SPI access //
///////////////////////////////////////////////////////////////////////////////


void rfm73::initSPI(uint8_t cs, uint8_t clk_div) {
  // setup SPI Port directions
  setPinMode(SCK, OUTPUT);
  setPinMode(MOSI, OUTPUT);
  setPinMode(MISO, INPUT);
  setPinMode(cs, OUTPUT);

  // setup SPI port states
  setPinState(MOSI, LOW);
  setPinState(SCK, LOW);
  setPinState(cs, HIGH);

  // init SPI
  SPCR = (1<<SPE)|(1<<MSTR);

  // det clock divider
  spiSetClockDivider(clk_div);

  // remember cs-pin
  pinCS = cs;
}




///////////////////////////////////////////////////////////////////////////////
// rfm73 initialization //
///////////////////////////////////////////////////////////////////////////////


void rfm73::initHardware(uint8_t ce, uint8_t irq){
  // setup RFM Port directions
  pinCE = ce;
  setPinMode(ce, OUTPUT);
  setPinState(ce, LOW);

  if (irq != -1) 
    setPinMode(irq, INPUT);
  
}


void rfm73::initRegisters() {

  // init bank 0 registers
  selectBank(0);

  // !! The last two regs in the bank0Init list will be handled later
  for (int i = 0; i < 20; i++)
    writeRegVal(pgm_read_byte(&rfm73_bank0Init[i][0]), pgm_read_byte(&rfm73_bank0Init[i][1]));

  // init address registers in bank 0
  writeRegPgmBuf((uint8_t *)rfm73_cmd_adrRX0, sizeof(rfm73_cmd_adrRX0));
  writeRegPgmBuf((uint8_t *)rfm73_cmd_adrRX1, sizeof(rfm73_cmd_adrRX1));
  writeRegPgmBuf((uint8_t *)rfm73_cmd_adrTX, sizeof(rfm73_cmd_adrTX));

  // activate Feature register
  if(!readRegVal(rfm73_REG_FEATURE))
    writeRegPgmBuf((uint8_t *)rfm73_cmd_activate, sizeof(rfm73_cmd_activate));

  // now set Registers 1D and 1C
  writeRegVal(pgm_read_byte(&rfm73_bank0Init[22][0]), pgm_read_byte(&rfm73_bank0Init[22][1]));
  writeRegVal(pgm_read_byte(&rfm73_bank0Init[21][0]), pgm_read_byte(&rfm73_bank0Init[21][1]));

  // init bank 1 registers
  selectBank(1);

  for (int i=0; i < 14; i++)
    writeRegPgmBuf((uint8_t *)rfm73_bank1Init[i], sizeof(rfm73_bank1Init[i]));

  // set ramp curve
  writeRegPgmBuf((uint8_t *)rfm73_bank1R0EInit, sizeof(rfm73_bank1R0EInit));

  // do we have to toggle some bits here like in the example code?
  writeRegPgmBuf((uint8_t *)rfm73_cmd_tog1, sizeof(rfm73_cmd_tog1));
  writeRegPgmBuf((uint8_t *)rfm73_cmd_tog2, sizeof(rfm73_cmd_tog2));


  delayMs(rfm73_END_INIT_WAIT_MS);

  //Check the ChipID
  if (readRegVal(0x08) != 0x63) 
    debug(rfm73_DEBUG_WRONG_CHIP_ID);

  selectBank(0);
  setModeRX();
}




void rfm73::Begin(uint8_t ce) {
  Begin(ce, -1, SS, RFM77_DEFAULT_SPI_CLOCK_DIV);
}

void rfm73::Begin(uint8_t ce, uint8_t irq) {
  Begin(ce, irq, SS, RFM77_DEFAULT_SPI_CLOCK_DIV);
}

void rfm73::Begin(uint8_t ce, uint8_t irq, uint8_t cs) {
  Begin(ce, irq, cs, RFM77_DEFAULT_SPI_CLOCK_DIV);
}

void rfm73::Begin(uint8_t ce, uint8_t irq, uint8_t cs, uint8_t clk_div) {
  initHardware(ce, irq);
  initSPI(cs, clk_div);
  delayMs(rfm73_BEGIN_INIT_WAIT_MS);
  initRegisters();
}


///////////////////////////////////////////////////////////////////////////////
// rfm73 Control //
///////////////////////////////////////////////////////////////////////////////


void rfm73::selectBank(uint8_t bank) {
  uint8_t tmp = readRegVal(0x07) & 0x80;
  if(bank) {
    if(!tmp)
      writeRegPgmBuf((uint8_t *)rfm73_cmd_switch_cfg, sizeof(rfm73_cmd_switch_cfg));
  } 
  else {
    if(tmp)
      writeRegPgmBuf((uint8_t *)rfm73_cmd_switch_cfg, sizeof(rfm73_cmd_switch_cfg));
  }
}


void rfm73::setMode(uint8_t mode) {
  if (mode)
    setModeRX();
  else
    setModeTX();
}


void rfm73::setModeRX(void)
{
  uint8_t val;

  writeRegPgmBuf((uint8_t *)rfm73_cmd_flush_rx, sizeof(rfm73_cmd_flush_rx)); // Flush RX FIFO
  val = readRegVal(rfm73_REG_STATUS); // Read Status
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_STATUS, val); // Reset IRQ bits
  setPinState(pinCE, LOW); // RFM chip disable
  // set PRIM_RX bit to 1
  val=readRegVal(rfm73_REG_CONFIG);
  val |= rfm73_PIN_PRIM_RX;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_CONFIG, val);
  setPinState(pinCE, HIGH); // RFM chip enable
}

void rfm73::setModeTX(void)
{
  uint8_t val;

  writeRegPgmBuf((uint8_t *)rfm73_cmd_flush_tx, sizeof(rfm73_cmd_flush_tx)); // Flush TX FIFO
  setPinState(pinCE, LOW); // RFM chip disable
  // set PRIM_RX bit to 0
  val=readRegVal(rfm73_REG_CONFIG);
  val &= ~rfm73_PIN_PRIM_RX;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_CONFIG, val);
  setPinState(pinCE, HIGH); // RFM chip enable
}



uint8_t rfm73::configRxPipe(uint8_t pipe_nr, uint8_t * adr, uint8_t plLen, uint8_t en_aa) {

  uint8_t tmp;
  uint8_t nr = pipe_nr -1;
  
  if(plLen > 32 || nr > 5 || en_aa > 1)
	return 0;

  // write address
  if(nr<2)	// full length for rx pipe 0 an 1
    writeRegCmdBuf(rfm73_CMD_WRITE_REG | (rfm73_REG_RX_ADDR_P0 + nr), adr, sizeof(adr));
  else // only LSB for pipes 2..5
    writeRegVal(rfm73_CMD_WRITE_REG | (rfm73_REG_RX_ADDR_P0 + nr), adr[0]); //ODO:check this
  
  // static
  if (plLen) {
    // set payload len
    writeRegVal(rfm73_CMD_WRITE_REG | (rfm73_REG_RX_PW_P0 + nr), plLen);
	// set EN_AA bit
	tmp = readRegVal(rfm73_REG_EN_AA);
	if (en_aa)
		tmp |= 1 << nr;
	else
		tmp &= ~(1 << nr);
	writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_EN_AA, tmp);
	// clear DPL bit
	tmp = readRegVal(rfm73_REG_DYNPD);
	tmp &= ~(1 << nr);
	writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_DYNPD, tmp);	
	// set Enable pipe bit
	enableRxPipe(nr);
  }
  // dynamic
  else {
    // set payload len to default
    writeRegVal(rfm73_CMD_WRITE_REG | (rfm73_REG_RX_PW_P0 + nr), 0x20);
	// set EN_AA bit
	tmp = readRegVal(rfm73_REG_EN_AA);
	tmp |= 1 << nr;
	writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_EN_AA, tmp);
	// set DPL bit
	tmp = readRegVal(rfm73_REG_DYNPD);
	tmp |= 1 << nr;
	writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_DYNPD, tmp);
	// set Enable pipe bit
	enableRxPipe(nr);
  }
  return 1;
}


void rfm73::enableRxPipe(uint8_t pipe_nr) {
  uint8_t nr = pipe_nr - 1;
  if (nr > 5) return;
  uint8_t tmp;
  // set Enable pipe bit
  tmp = readRegVal(rfm73_REG_EN_RXADDR);
  tmp |= 1 << nr;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_EN_RXADDR, tmp);
}

void rfm73::disableRxPipe(uint8_t pipe_nr) {
  uint8_t nr = pipe_nr - 1;
  if (nr > 5) return;
  uint8_t tmp;
  // set Enable pipe bit
  tmp = readRegVal(rfm73_REG_EN_RXADDR);
  tmp &= ~(1 << nr);
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_EN_RXADDR, tmp);

}


void rfm73::configTxPipe(uint8_t * adr, uint8_t pltype) {
  // write TX address
  writeRegCmdBuf(rfm73_CMD_WRITE_REG | rfm73_REG_TX_ADDR, adr, sizeof(adr));
  // write RX0 address
  writeRegCmdBuf(rfm73_CMD_WRITE_REG | rfm73_REG_RX_ADDR_P0, adr, sizeof(adr));
  // set static or dynamic payload
  uint8_t tmp;
  tmp = readRegVal(rfm73_REG_DYNPD);
  if(pltype == TX_DPL) // dynamic
	tmp |= 1;
  else  
    tmp &= ~(1 << 0);
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_DYNPD, tmp);
}


void rfm73::configCRC(uint8_t crc) {
//  setPinState(pinCE, LOW); // RFM chip disable
  uint8_t tmp = readRegVal(rfm73_REG_CONFIG);
  //reset crc state
  tmp &= 0xF3;
  if (crc == CRC1)
    tmp |= 0x08;
  else if (crc == CRC2)
    tmp |= 0x0C;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_CONFIG, tmp);
//  setPinState(pinCE, HIGH); // RFM chip enable
}


void rfm73::configARD(uint8_t ard) {
  if (ard > 0x0f) return;
  uint8_t tmp = readRegVal(rfm73_REG_SETUP_RETR);
  tmp &= ((ard << 4) | 0x0F);
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_SETUP_RETR, tmp);
}

void rfm73::configARC(uint8_t arc) {
  if (arc > 0x0f) return;
  uint8_t tmp = readRegVal(rfm73_REG_SETUP_RETR);
  tmp &= (arc | 0xF0);
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_SETUP_RETR, tmp);
}

/*void rfm73::configSpeed(uint8_t speed) {
  if (speed > 2 || speed < 1) return;
  uint8_t tmp = readRegVal(rfm73_REG_RF_SETUP);
  tmp &= 0xF7;
  tmp |= speed << 3;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_RF_SETUP, tmp);
}*/
/***
*
*	speed 0 = 1Mbps
*	1 = 2MBps
*	2 = 250Kbps
*	3 = 2Mbps(default)
*
***/
void rfm73::configSpeed(uint8_t speed) {
  if (speed > 3) return;
  uint8_t tmp = readRegVal(rfm73_REG_RF_SETUP);
  tmp &= 0xD7;
  switch(speed){
  	case 0:
  		tmp |= 0b00000000;
  	break;
  	case 1:
  		tmp |= 0b00001000;
  	break;
  	case 2:
  		tmp |= 0b00100000;
  	break;
  	case 3:
  		tmp |= 0b00101000;
  	break;
  }  
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_RF_SETUP, tmp);
}

void rfm73::configLnaGain(uint8_t gain) {
  if (gain > 1) return;
  uint8_t tmp = readRegVal(rfm73_REG_RF_SETUP);
  tmp &= 0xFE;
  tmp |= gain;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_RF_SETUP, tmp);
}


void rfm73::configRfPower(uint8_t pwr) {
  if (pwr > 3) return;
  uint8_t tmp = readRegVal(rfm73_REG_RF_SETUP);
  tmp &= 0xF9;
  tmp |= pwr << 1;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_RF_SETUP, tmp);
}

void rfm73::confAddrWidth(uint8_t width) {
  if (width < 3 || width > 5) return;
  uint8_t tmp = readRegVal(rfm73_REG_SETUP_AW);
  tmp &= ( 0xF0 | (width - 2 ));
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_SETUP_AW, tmp);  
}

void rfm73::setPower(uint8_t pwr) {
  if (pwr > 1) return;
//  setPinState(pinCE, LOW); // RFM chip disable
  uint8_t tmp = readRegVal(rfm73_REG_CONFIG);
  tmp &= (0xFD | (pwr << 1));
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_CONFIG, tmp);
//  setPinState(pinCE, HIGH); // RFM chip enable
}

void rfm73::confIRQ(uint8_t irq_pin, uint8_t reflectTX_DS, uint8_t reflectRX_DR, uint8_t reflectMAX_RT) {
  if (irq_pin != -1) 
    setPinMode(irq_pin, INPUT);
//  setPinState(pinCE, LOW); // RFM chip disable
  uint8_t tmp = readRegVal(rfm73_REG_CONFIG) & 0x8F;
  tmp|= ((reflectTX_DS & 0x01) ^ 0x01) << 6;
  tmp|= ((reflectRX_DR & 0x01) ^ 0x01) << 5;
  tmp|= ((reflectMAX_RT & 0x01) ^ 0x01) << 4;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_CONFIG, tmp);
//  setPinState(pinCE, HIGH); // RFM chip enable
}

void rfm73::cliAll() {
  uint8_t tmp = readRegVal(rfm73_REG_STATUS) | 0x70;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_STATUS, tmp);
}


void rfm73::cliRxDr() {
  uint8_t tmp = readRegVal(rfm73_REG_STATUS) | 0x40 & 0xCF;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_STATUS, tmp);
}

void rfm73::cliTxDs() {
  uint8_t tmp = readRegVal(rfm73_REG_STATUS) | 0x20 & 0xAF;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_STATUS, tmp);
}


void rfm73::cliTimeout() {
  uint8_t tmp = readRegVal(rfm73_REG_STATUS) | 0x10 & 0x9F;
  writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_STATUS, tmp);
}


void rfm73::flushTxFIFO() {
  writeRegPgmBuf((uint8_t *)rfm73_cmd_flush_tx, sizeof(rfm73_cmd_flush_tx)); // Flush TX FIFO
}

void rfm73::flushRxFIFO() {
  writeRegPgmBuf((uint8_t *)rfm73_cmd_flush_rx, sizeof(rfm73_cmd_flush_rx)); // Flush RX FIFO
}
  
 

///////////////////////////////////////////////////////////////////////////////
// rfm73 getter //
///////////////////////////////////////////////////////////////////////////////

uint8_t rfm73::getMode(void) {
  return readRegVal(rfm73_REG_CONFIG) & rfm73_PIN_PRIM_RX;
}

uint8_t rfm73::getCarrierDetect(void)
{
  return readRegVal(rfm73_REG_CD);
}

void rfm73::setChannel(uint8_t cnum)
{
  writeRegVal( rfm73_CMD_WRITE_REG | rfm73_REG_RF_CH, cnum);
}

uint8_t rfm73::getChannel(void) {
  return readRegVal(rfm73_REG_RF_CH);
}

uint8_t rfm73::getPLC(void) {
  return readRegVal(rfm73_REG_OBSERVE_TX) >> 4 & 0x0F;
}

uint8_t rfm73::getARC(void){
  return readRegVal(rfm73_REG_OBSERVE_TX) & 0x0F;
}

uint8_t rfm73::rxDataReceived() {
  uint8_t status = readRegVal(rfm73_REG_STATUS);
  if(status & rfm73_IRQ_STATUS_RX_DR) {
	return ((status & 0x0E) >> 1) + 1;
  }
    
}

uint8_t rfm73::txDataSent() {
  return readRegVal(rfm73_REG_STATUS) & rfm73_IRQ_STATUS_TX_DS;
}

uint8_t rfm73::txTimeout() {
  return readRegVal(rfm73_REG_STATUS) & rfm73_IRQ_STATUS_MAX_RT;
}

uint8_t rfm73::txFIFOFull() {
  return readRegVal(rfm73_REG_FIFO_STATUS) & rfm73_FIFO_STATUS_TX_FULL;
}

uint8_t rfm73::txFIFOEmpty() {
  return readRegVal(rfm73_REG_FIFO_STATUS) & rfm73_FIFO_STATUS_TX_EMPTY;
}

uint8_t rfm73::rxFIFOFull() {
  return readRegVal(rfm73_REG_FIFO_STATUS) & rfm73_FIFO_STATUS_RX_FULL;
}

uint8_t rfm73::rxFIFOEmpty() {
  return readRegVal(rfm73_REG_FIFO_STATUS) & rfm73_FIFO_STATUS_RX_EMPTY;
}



///////////////////////////////////////////////////////////////////////////////
// rfm73 Communication //
///////////////////////////////////////////////////////////////////////////////


uint8_t rfm73::readRegVal(uint8_t cmd) {
  uint8_t res;
  setPinState(pinCS, LOW);
  delayMs(rfm73_CS_DELAY);
  transmitSPI(cmd);
  res=transmitSPI(0);
  setPinState(pinCS, HIGH);
  delayMs(rfm73_CS_DELAY);
  return res;
}


void rfm73::readRegBuf(uint8_t reg, uint8_t * buf, uint8_t len) {
  uint8_t status, byte_ctr;
  setPinState(pinCS, LOW);
  delayMs(rfm73_CS_DELAY);
  status = transmitSPI(reg); // Select register to write, and read status UINT8
  for(byte_ctr = 0; byte_ctr < len; byte_ctr++)
    buf[byte_ctr] = transmitSPI(0); // Perform SPI_RW to read UINT8 from rfm73
  setPinState(pinCS, HIGH);
  delayMs(rfm73_CS_DELAY);
}


uint8_t rfm73::writeRegVal(uint8_t cmd, uint8_t val) {
  setPinState(pinCS, LOW);
  delayMs(rfm73_CS_DELAY);
  transmitSPI(cmd);
  transmitSPI(val);
  setPinState(pinCS, HIGH);
  delayMs(rfm73_CS_DELAY);
}


uint8_t rfm73::writeRegPgmBuf(uint8_t * cmdbuf, uint8_t len) {
  setPinState(pinCS, LOW);
  delayMs(rfm73_CS_DELAY);
  while(len--) {
    transmitSPI(pgm_read_byte(cmdbuf++));
  }
  setPinState(pinCS, HIGH);
  delayMs(rfm73_CS_DELAY);
}


uint8_t rfm73::writeRegCmdBuf(uint8_t cmd, uint8_t * buf, uint8_t len) {
  setPinState(pinCS, LOW);
  delayMs(rfm73_CS_DELAY);
  transmitSPI(cmd);
  while(len--) {
    transmitSPI(*(buf++));
  }
  setPinState(pinCS, HIGH);
  delayMs(rfm73_CS_DELAY);
}




uint8_t rfm73::sendAckPayload(uint8_t * payload, uint8_t len) {
  sendPayload(payload, len, -1);
}


uint8_t rfm73::sendPayload(uint8_t * payload, uint8_t len) {
  sendPayload(payload, len, NO_ACK);
}


uint8_t rfm73::sendPayload(uint8_t * payload, uint8_t len, uint8_t toAck)
{
  // check TX_FIFO
  uint8_t status;
  status = readRegVal(rfm73_REG_FIFO_STATUS); 
  if (status & rfm73_FIFO_STATUS_TX_FULL) {
    debug(rfm73_DEBUG_FIFO_FULL);
    return 0;
  }
  // send payload
  setPinState(pinCS, LOW);
  delayMs(rfm73_CS_DELAY);
  if(toAck == -1)
    transmitSPI(rfm73_CMD_W_ACK_PAYLOAD);
  else if (toAck == 0)
    transmitSPI(rfm73_CMD_W_TX_PAYLOAD_NOACK);
  else
    transmitSPI(rfm73_CMD_WR_TX_PLOAD);
  while(len--) {
    transmitSPI(*(payload++));
  }
  setPinState(pinCS, HIGH);
  delayMs(rfm73_CS_DELAY);

  return 1;
}


uint8_t rfm73::receivePayload(uint8_t *payload)
{
  uint8_t len;
  // check RX_FIFO
  uint8_t status;
  status = readRegVal(rfm73_REG_STATUS);
  if (status & rfm73_IRQ_STATUS_RX_DR) { // RX_DR
    //while(1) {
      uint8_t fifo_sta;
      len = readRegVal(rfm73_CMD_RX_PL_WID); // Payload width
      readRegBuf(rfm73_CMD_RD_RX_PLOAD, payload, len);
      fifo_sta = readRegVal(rfm73_REG_FIFO_STATUS);
      //if (fifo_sta & rfm73_FIFO_STATUS_RX_EMPTY) break; // read until RX_FIFO empty
    //}
	if (fifo_sta & rfm73_FIFO_STATUS_RX_EMPTY) {
	status|= 0x40 & 0xCF; // clear status bit rx_dr
    writeRegVal(rfm73_CMD_WRITE_REG | rfm73_REG_STATUS, status); 
	}
    return len;
  }
  else
    return 0;
}




///////////////////////////////////////////////////////////////////////////////
// rfm73 debug //
///////////////////////////////////////////////////////////////////////////////


void rfm73::debug(uint8_t token) {
	// to be done
} 



///////////////////////////////////////////////////////////////////////////////
// rfm73 potential inline funcs
///////////////////////////////////////////////////////////////////////////////


uint8_t rfm73::transmitSPI(uint8_t val) {
  SPDR = val;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

void rfm73::setPinState(uint8_t pin, uint8_t state) {
  // Arduino specific implementation. change if not on Arduino
  digitalWrite(pin, state);
}


void rfm73::setPinMode(uint8_t pin, uint8_t mode) {
  // Arduino specific implementation. change if not on Arduino
  pinMode(pin, mode);
}

void rfm73::delayMs(uint8_t ms) {
  // Arduino specific implementation. change if not on Arduino
  if (ms)delay(ms);
}

void rfm73::spiSetClockDivider(uint8_t rate) {
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}

///////////////////////////////////////////////////////////////////////////////
// this is the new shit
///////////////////////////////////////////////////////////////////////////////
/***
*
*  void Begin()
*  sets default parameters used by the megaRF 2.4G
*
***/
void rfm73::Begin(void){  
  // rfm73 setup
  Begin(8, 2);
  confAddrWidth(3);
  configRfPower(3); //max power
  configLnaGain(1); //gain
  //default mode is rx mode
  setMode(MODE_PRX);
}


/***
*
*	void tick() 
*	checks if there is a complete transmission and calls the users callback function
*
***/
void rfm73::tick()
{
  byte len = receivePayload(rcvBuffer);
  if(len<32) rcvBuffer[len] = '\0';
  //new packet if len > 0 
  if (len!=0) {
    packetLength = (uint8_t)len;
    //Serial.print("recieved: ");      
    //Serial.println((char*)rcv_payload);

    /*
    if(rcv_payload[0] == ID_TELEMETRY) {
    }
    */

		// don't bother if user hasn't registered a callback
 		if(!user_onReceive){
    		
  		}
  		else{
			user_onReceive();
		}
  }
  return;
}





/***
*	
*	sets function called when receive transmission complete
*
***/
void rfm73::onReceive( void (*function)(void) )
{
  user_onReceive = function;
}






/***
*
*	uint8_t* getBuffer(void)
*	return pointer to the first element of the buffer
*
***/
uint8_t* rfm73::getRcvBuffer(void){
	return rcvBuffer;
}



/***
*
*	uint8_t getByte(uint8_t)
*	returns the specified byte from the package buffer
*
***/
uint8_t rfm73::getRcvByte(uint8_t byte){
	return rcvBuffer[byte];
}


/***
*
*	void send(char*) : function to enqueue a packet for transmission
*	returns: 0x02 (error) 0x03 (occupied) 0x80 (enqueued)	
*
***//*
bool rfm73::send(char* s)
{
	uint8_t* ptr=(uint8_t*)s;	
	uint8_t len=0;	
	
	//get the length of char s 
	while (*ptr)
    {   // so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" 
        ptr++;
        len++;
    }
    
    return send(s, len);
}*/


uint8_t rfm73::getPacketLength(void){
  return packetLength;
}


/***
*
*	boolean send(char*) : function to enqueue a packet for transmission
*	returns: true, false
*
***/
void rfm73::send(uint8_t* s, uint8_t len)
{

  //switch to tx mode
  cliTxDs();
  setMode(MODE_PTX);    
  
  if (txTimeout()) {
    flushTxFIFO();
    cliTimeout();
  }
  /*
  if(rfm.sendPayload(cmd_buf, len, WITH_ACK) == 1){
    Serial.println("send");
  }*//*
  if(sendPayload(s, len) == 1){
    Serial.println("sent");
  }
  else{
    Serial.println("send Err");  
  }*/
  
  sendPayload(s, len);  
  
  //wait for packet to be sent
  _delay_us(SEND_DELAY);
  
  //switch to rx mode
  cliRxDr();
  setMode(MODE_PRX);    
}



// preinstantiate object

rfm73 RFM;