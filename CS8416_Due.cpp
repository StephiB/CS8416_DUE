/**************************************************************************/
//	Library for CS8416 SPI Mode on Arduino Due
/**************************************************************************/

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <SPI.h>
#include <CS8416_Due.h>

CS8416::CS8416(uint8_t cs) {
	_cs   = cs;
}

/**************************************************************************/
//	Start device
/**************************************************************************/
bool CS8416::begin() {
	uint8_t revision;
	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);
	SPI.begin();
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1); //Data is clocked in on the rising edge of CCLK and out on the falling edge. See Datasheet 12.1 SPI Mode
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, CS8416_ID_VERSION, SPI_LAST);
	delayMicroseconds(20);
	revision = SPI.transfer(_cs, CS8416_READ_REG);
	if (revision != 0) {
		return true;
	}
	else {
		return false;
	}
}
/**************************************************************************/
//	Initiate: Run, I²S, 16bit, de-emphasis filter auto-select on,
//	GPIO0 = TX, GPIO1 = INT, GPIO2 = 96kHz
/**************************************************************************/
void CS8416::initiate() {
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, (INCR & 0x00), SPI_CONTINUE);
	SPI.transfer(_cs, 0x00, SPI_CONTINUE);
	SPI.transfer(_cs, 0x00, SPI_CONTINUE);
	SPI.transfer(_cs, ((CTRL2_EMPH_CNTL0 & 0x40) | (CTRL2_GPO0_SEL & 0x0B)), SPI_CONTINUE);
	SPI.transfer(_cs, ((CTRL3_GPO1SEL & 0x20) | (CTRL3_GPO2SEL & 0x08)), SPI_CONTINUE);
	SPI.transfer(_cs, CTRL4_RUN, SPI_CONTINUE);
	SPI.transfer(_cs, FRMT_I2S | (FRMT_SORES & 0x20), SPI_CONTINUE);
	SPI.transfer(_cs, ERRMASK_UNLOCK, SPI_CONTINUE);
	SPI.transfer(_cs, INT_RERRM, SPI_CONTINUE);
	SPI.transfer(_cs, INT_RERR1, SPI_CONTINUE);
	SPI.transfer(_cs, INT_RERR0, SPI_LAST);
}
/**************************************************************************/
//	Writes 8-bits to the specified destination register
/**************************************************************************/
void CS8416::writeRegister(uint8_t reg, uint8_t value) {
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, reg, SPI_CONTINUE);
	SPI.transfer(_cs, value, SPI_LAST);
}
/**************************************************************************/
//	Reads 8-bits from the specified register
/**************************************************************************/
uint8_t CS8416::readRegister(uint8_t reg) {
	uint8_t value;
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, reg, SPI_LAST);
	delayMicroseconds(20);
	value = SPI.transfer(_cs, CS8416_READ_REG);
	return value;
}
/**************************************************************************/
//	Writes multiple bytes
/**************************************************************************/
bool CS8416::writeBytes(int startAddr, const byte* array, int numBytes) {
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, startAddr, SPI_CONTINUE);
	for (uint8_t i = 0; i < numBytes; i++) {
		if (i == numBytes) {
			SPI.transfer(_cs, array[i], SPI_LAST);
		}
		else {
			SPI.transfer(_cs, array[i], SPI_CONTINUE);
		}
	}
	return true;
}
/**************************************************************************/
//	Reads multiple bytes
/**************************************************************************/
bool CS8416::readBytes(int startAddr, byte array[], int numBytes) {
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, startAddr, SPI_LAST);
	delayMicroseconds(20);
	SPI.transfer(_cs, CS8416_READ_REG, SPI_CONTINUE);
	for (uint8_t i = 0; i < numBytes; i++) {
		if (i == numBytes) {
			array[i] = SPI.transfer(_cs, 0x00, SPI_LAST);
		}
		else {
			array[i] = SPI.transfer(_cs, 0x00, SPI_CONTINUE);
		}
	}
	return true;
}
/**************************************************************************/
//	Read Q-subcode block 
/**************************************************************************/
void CS8416::readQsubcodes(byte array[10]) {
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.setBitOrder(_cs, LSBFIRST); //Each byte is LSB first with respect to the 80 Q-subcode bits.
	SPI.transfer(_cs, CS8416_Q_CH_SUB_07_00, SPI_LAST);
	delayMicroseconds(20);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.transfer(_cs, CS8416_READ_REG, SPI_CONTINUE);
	SPI.setBitOrder(_cs, LSBFIRST);
	array[1] = SPI.transfer(_cs, CS8416_Q_CH_SUB_07_00, SPI_CONTINUE);  //   (R)   Control
	array[2] = SPI.transfer(_cs, CS8416_Q_CH_SUB_15_08, SPI_CONTINUE);  //   (R)   Track
	array[3] = SPI.transfer(_cs, CS8416_Q_CH_SUB_23_16, SPI_CONTINUE);  //   (R)   Index
	array[4] = SPI.transfer(_cs, CS8416_Q_CH_SUB_31_24, SPI_CONTINUE);  //   (R)   Minute
	array[5] = SPI.transfer(_cs, CS8416_Q_CH_SUB_39_32, SPI_CONTINUE);  //   (R)   Second
	array[6] = SPI.transfer(_cs, CS8416_Q_CH_SUB_47_40, SPI_CONTINUE);  //   (R)   Frame
	array[7] = SPI.transfer(_cs, CS8416_Q_CH_SUB_55_48, SPI_CONTINUE);  //   (R)   Zero
	array[8] = SPI.transfer(_cs, CS8416_Q_CH_SUB_63_56, SPI_CONTINUE);  //   (R)   ABS Minute
	array[9] = SPI.transfer(_cs, CS8416_Q_CH_SUB_71_64, SPI_CONTINUE);  //   (R)   ABS Second
	array[10] = SPI.transfer(_cs, CS8416_Q_CH_SUB_79_72, SPI_LAST);     //   (R)   ABS Frame
}
/**************************************************************************/
//	Change used input and source for TX
/**************************************************************************/
void CS8416::changeInput(uint8_t num) {
	uint8_t tx = num;
	uint8_t rx = num << 3;
	muteOutput(true);
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, CS8416_CTRL4, SPI_CONTINUE);
	SPI.transfer(_cs, (CTRL4_RUN | (CTRL4_RXSEL & rx ) | (CTRL4_TXSEL & tx)), SPI_LAST);
	muteOutput(false);
}
/**************************************************************************/
//	Automatic clock switching
/**************************************************************************/
void CS8416::clockSwitch(bool cls) {
	uint8_t h;
	if (!cls){
		h = 0;
	}
	else{
		h = 1;
	}
	uint8_t switchbit = h << 7;
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, CS8416_CTRL1, SPI_CONTINUE);
	SPI.transfer(_cs, (CTRL1_SWCLK & switchbit), SPI_LAST);
}
/**************************************************************************/
//	Mute
/**************************************************************************/
void CS8416::muteOutput(bool mto) {
	uint8_t h;
	if (!mto){
		h = 0;
	}
	else{
		h = 1;
	}
	uint8_t mutebit = h << 6;
	SPI.setClockDivider(_cs, 21);
	SPI.setBitOrder(_cs, MSBFIRST);
	SPI.setDataMode(_cs, SPI_MODE1);
	SPI.transfer(_cs, CS8416_WRITE_REG, SPI_CONTINUE);
	SPI.transfer(_cs, CS8416_CTRL1, SPI_CONTINUE);
	SPI.transfer(_cs, (CTRL1_MUTESAO & mutebit), SPI_LAST);
}
/**************************************************************************/
//	Read the device ID (can be used to check connection)
/**************************************************************************/
uint8_t CS8416::getDeviceID(void) {
	return readRegister(CS8416_ID_VERSION);
}
/**************************************************************************/
//
/**************************************************************************/
