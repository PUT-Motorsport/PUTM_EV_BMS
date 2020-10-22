/*
 * bms_1.c
 *
 * Author : Maksymilian Jaruga
 */ 

#include <avr/io.h>
#include <string.h>
#include "./can.h"

#define F_CPU 16000000UL

#include <util/delay.h>
#include <avr/interrupt.h>

const can_filter_t filtersetup = {0,0,{0}};

//	timers
volatile uint64_t timerCounter = 0;
volatile uint64_t timerCounter_2 = 0;
volatile uint64_t timerCounter_3 = 0;

float temperatureMap[29][2] = {
	//    ltc value ,  temperature *C
	48656 , -20 ,
	45988 , -15 ,
	44280 , -10 ,
	42292 , -5  ,
	40026 , 0 ,
	37515 , 5 ,
	34796 , 10  ,
	31955 , 15  ,
	29043 , 20  ,
	26150 , 25  ,
	23325 , 30  ,
	20658 , 35  ,
	18165 , 40  ,
	15899  , 45  ,
	13842  , 50  ,
	12011  , 55  ,
	10407  , 60  ,
	9012  , 65  ,
	7793  , 70  ,
	6729  , 75  ,
	5840  , 80  ,
	5056  , 85  ,
	4393  , 90  ,
	3818  , 95  ,
	3330  , 100 ,
	2894  , 105 ,
	2528  , 110 ,
	2214  , 115
};

uint8_t ltcConfig[6] = {0xFC, (1874 & 0xff), (1874>>4)|(2625<<4), (2625>>4), 0, 0};
uint8_t acuState=0;
uint16_t cellValues[10];
uint16_t cellValuesSum;
uint16_t cellValuesLow;
uint16_t cellValuesHigh;
uint16_t tempValues[10];
uint16_t tempValues_2[10];
uint16_t tempValuesAvr;
uint16_t tempValuesHigh;
uint16_t reference2_value;
uint16_t reference2_value_2;
uint8_t canFlag=0;
float tempValuesFloat[10];

ISR(TIMER0_COMP_vect)
{
	TCNT0 = 0;
	
	timerCounter_3++;
	if((timerCounter_3%20)==0){
		canFlag=1;
	}
	
	
	if((cellValues[0] < 35000 || cellValues[0] > 42300) || (cellValues[1] < 35000 || cellValues[1] > 42300) || (cellValues[3] < 35000 || cellValues[3] > 42300) || (cellValues[4] < 35000 || cellValues[4] > 42300)){
		timerCounter++;
	}
	
	if((cellValues[0] >= 35000 && cellValues[0] <= 42300) && (cellValues[1] >= 35000 && cellValues[1] <= 42300) && (cellValues[3] >= 35000 && cellValues[3] <= 42300) && (cellValues[4] >= 35000 && cellValues[4] <= 42300)){
		acuState = 0;
		timerCounter = 0;
	}
	
	if(tempValuesFloat[1] > 50 || tempValuesFloat[2] > 50 || tempValuesFloat[3] > 50 || tempValuesFloat[4] > 50){
		timerCounter_2++;
	}
	
	if(tempValuesFloat[1] <= 50 && tempValuesFloat[2] <= 50 && tempValuesFloat[3] <= 50 && tempValuesFloat[4] <= 50){
		acuState = 0;
		timerCounter_2 = 0;
	}
	
	if(timerCounter_3>3000 && (tempValuesFloat[1] <= 40 && tempValuesFloat[2] <= 40 && tempValuesFloat[3] <= 40 && tempValuesFloat[4] <= 40) && ((cellValues[0] >= 36000 && cellValues[0] <= 42300) && (cellValues[1] >= 36000 && cellValues[1] <= 42300) && (cellValues[3] >= 36000 && cellValues[3] <= 42300) && (cellValues[4] >= 36000 && cellValues[4] <= 42300))){
		acuState = 0;
		PORTE |= 1 << DDE2;
	}
	
	if(timerCounter >= 479){
		if((cellValues[0] < 35000 || cellValues[0] > 42300)){
			if(cellValues[0] < 35000){
				acuState = 0b1001;
				}else{
				acuState = 0b10001;
			}
		}
		if((cellValues[1] < 35000 || cellValues[1] > 42300)){
			if(cellValues[1] < 35000){
				acuState = 0b1011;
				}else{
				acuState |= 0b10011;
			}
		}
		if((cellValues[2] < 35000 || cellValues[2] > 42300)){
			if(cellValues[2] < 35000){
				acuState |= 0b1101;
				}else{
				acuState |= 0b10101;
			}
		}
		if((cellValues[3] < 35000 || cellValues[3] > 42300)){
			if(cellValues[3] < 35000){
				acuState |= 0b1111;
				}else{
				acuState |= 0b10111;
			}
		}
	}
	
	if(timerCounter >= 500){
		PORTE = 0 << DDE2;
	}
	
	if(timerCounter_2 >= 479){
	if(tempValuesFloat[1] > 50){
		acuState = 0b100001;
	}
	if(tempValuesFloat[2] > 50){
		acuState |= 0b100011;
	}
	if(tempValuesFloat[3] > 50){
		acuState |= 0b100101;
	}
	if(tempValuesFloat[4] > 50){
		acuState |= 0b100111;
	}
	}
	
	if(timerCounter_2 >= 500){
		PORTE = 0 << DDE2;
	}
	
}

void TickTimerInit(){
	// fcpu 16 MHz
	// preskaler 64
	// comp 249
	// 16 000 000 / (64 * (249+1)) = 16000 ---> 1ms
	TCCR0A = 1<<CS1 | 1 << CS0;
	TCNT0 = 0;
	OCR0A = 249;
	TIFR0 = 0b11; // clearing interrupt flags
	TIMSK0 = 1<<OCIE0A;
}

void d_ms(int n) {
	while(n--) {
		_delay_ms(1);
	}
}

uint16_t pec15Table[256];
uint16_t CRC15_POLY = 0x4599;
void init_PEC15_Table()
{
	uint16_t remainder;
	for (int i = 0; i < 256; i++)
	{
		remainder = i << 7;
		for (int bit = 8; bit > 0; --bit)
		{
			if (remainder & 0x4000)
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC15_POLY);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}
		pec15Table[i] = remainder&0xFFFF;
	}
}

uint16_t pec15(char *data , int len)
{
	uint16_t remainder,address;
	remainder = 16;//PEC seed
	for (int i = 0; i < len; i++)
	{
		address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
		remainder = (remainder << 8 ) ^ pec15Table[address];
	}
	return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}

void SPI_MasterInit(void)
{
  /* Set MOSI and SCK output, all others input */
  DDRB = (1<<DDB2) | (1<<DDB1) | (1<<DDB5) | (1<<DDB0) | (1<<DDB6);
  DDRE = (1<<DDE2);
  /* Enable SPI, Master, set clock rate fck/16 */
  SPSR = (0<<SPI2X);
  SPCR = (1<<SPE) | (0<<DORD) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA) | (0<<SPR1) | (1<<SPR0);
}

void SPI_Transmit(int8_t  data)  // Byte to be written to SPI port
{
  SPDR = data;                  //! 1) Start the SPI transfer
  while (!(SPSR & (1<<SPIF)));  //! 2) Wait until transfer complete
}

int8_t SPI_Read(int8_t  data) //!The data byte to be written
{
  SPDR = data;                  //! 1) Start the SPI transfer
  while (!(SPSR & (1<<SPIF))); //! 2) Wait until transfer complete
  return SPDR;                  //! 3) Return the data read
}

void SPI_TransmitArray(uint8_t data[],  //Array of bytes to be written on the SPI port
uint8_t len                          // Option: Number of bytes to be written on the SPI port
)
{
  for (uint8_t i = 0; i < len; i++)
  {
    SPI_Transmit((int8_t)data[i]);
  }
}

void SPI_TransmitRead(uint8_t *tx_Data,//array of data to be written on SPI port
uint8_t tx_len, //length of the tx and rx data array
uint8_t *rx_data//Input: array that will store the data read by the SPI port
)
{
  for (uint8_t i = 0; i < tx_len; i++)
  {
    rx_data[i] = (uint8_t)SPI_Read(tx_Data[i]);
  }

}

void LTC_Wakeup()
{
	uint8_t tab[2] = {0xFF};

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 2);
	PORTB |= 1<<DDB0;
	
}

void LTC_StartCellADC()
{
	uint8_t tab[12];
	uint16_t pec;
	uint16_t cmd = (1<<15) | 0x01;
	// configuration
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;
	
	tab[4] = ltcConfig[0];
	tab[5] = ltcConfig[1];
	tab[6] = ltcConfig[2];
	tab[7] = ltcConfig[3];
	tab[8] = ltcConfig[4];
	tab[9] = ltcConfig[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	LTC_Wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 12);
	PORTB |= 1<<DDB0;

	// adc conversion
	memset(tab, 0, 12);

	cmd = 0b1001100000 | (0b00 << 7);
	tab[0] = cmd>>8;
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//LTC_Wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 4);
	PORTB |= 1<<DDB0;

}

void LTC_GetValuesADC()
{
	uint8_t tab[100], rx_tab[100];
	uint16_t pec;
	
	// read cell voltage group A
	uint16_t cmd = (1<<15) | 0b100;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	LTC_Wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB |= 1<<DDB0;

	cellValues[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	cellValues[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cellValues[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8); //short circuit
	
	// read cell voltage group B
	cmd = (1<<15) | 0b110;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//LTC_Wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB ^= 1<<DDB0;

	cellValues[3] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	cellValues[4] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cellValues[5] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8); //short circuit
	
	cellValuesSum  = cellValues[0] / 1000;
	cellValuesLow  = cellValues[0];
	cellValuesHigh = cellValues[0];
	
	//cell calculations
	for(int i = 1; i < 5; i++)
	{
		if(i!=2){
			cellValuesSum = cellValuesSum + cellValues[i] / 1000;
			if(cellValuesLow > cellValues[i]){
				cellValuesLow = cellValues[i];
			}
			if(cellValuesHigh < cellValues[i]){
				cellValuesHigh = cellValues[i];
			}
		}
	}
	cellValuesSum  = cellValuesSum;
	cellValuesHigh = cellValuesHigh / 1000;
	cellValuesLow  = cellValuesLow  / 1000;
	
}

float tempCalculate(uint16_t ltc_value)
{
	float retval = 0.0;
	for(int i = 1; i < 28; i++)
	{
		if(ltc_value >= (uint16_t)temperatureMap[i][0])
		{
			// approximation
			retval = temperatureMap[i][1] - 5.0 * ((float)ltc_value-temperatureMap[i][0]) / (temperatureMap[i-1][0] - temperatureMap[i][0]);
			break;
		}
	}
	return retval;
}

void LTC_StartTempValues()
{
	uint8_t tab[12];
	uint16_t pec;
	uint16_t cmd = (1<<15) | 0x01;
	// configuration
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;
	
	tab[4] = ltcConfig[0];
	tab[5] = ltcConfig[1];
	tab[6] = ltcConfig[2];
	tab[7] = ltcConfig[3];
	tab[8] = ltcConfig[4];
	tab[9] = ltcConfig[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	LTC_Wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 12);
	PORTB |= 1<<DDB0;

	// adc conversion
	memset(tab, 0, 12);

	cmd = 0b10001100000 | (0b00 << 7);
	tab[0] = cmd>>8;
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//LTC_Wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 12);
	PORTB |= 1<<DDB0;
}


void LTC_GetTempValues()
{
	uint8_t tab[100], rx_tab[100];
	uint16_t pec;

	// read gpio voltage group A
	memset(tab, 0, 12);
	tab[0] = 0;
	tab[1] = 0b1100;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	LTC_Wakeup();
	
	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,20,rx_tab);
	PORTB |= 1<<DDB0;

	tempValues[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	tempValues[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	tempValues[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);

	// read cell voltage group B
	memset(tab, 0, 12);
	tab[0] = 0;
	tab[1] = 0b1110;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//LTC_Wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,20,rx_tab);
	PORTB |= 1<<DDB0;

	tempValues[3] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	tempValues[4] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	reference2_value = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);
	
	tempValuesAvr = 0;

	//temp calculations
	for(int i = 1; i < 5; i++)
	{
		tempValuesFloat[i] = tempCalculate(tempValues[i]);
		
		tempValuesAvr = tempValuesAvr + tempValuesFloat[i];
	}
	tempValuesAvr = tempValuesAvr/4;
	
	tempValuesHigh = tempValuesFloat[1];
	for(int i = 2; i < 5; i++){
		if(tempValuesHigh < tempValuesFloat[i]){
			tempValuesHigh = tempValuesFloat[i];
		}
	}
}

int main(void)
{
	init_PEC15_Table();
	SPI_MasterInit();
	TickTimerInit();
	sei();
	
	can_init(BITRATE_500_KBPS);
	can_set_filter(1, &filtersetup);
	PORTE &= 0 << DDE2; //power in car is disabled -> will be enabled 3sec after program launch
	
    while (1) 
    {
			//should "if" with "PORTE DDE2" be here?
			LTC_StartCellADC();
			d_ms(20);
			LTC_GetValuesADC();
			
			LTC_StartTempValues();
			d_ms(20);
			LTC_GetTempValues();
			
			if(canFlag == 1){
				canFlag = 0;
				static can_t txMessage;
				
				txMessage.id = 0x0B;
				txMessage.flags.rtr = 0;
				txMessage.length = 8;
				
				txMessage.data[0] = cellValuesSum;
				txMessage.data[1] = acuState;
				txMessage.data[2] = tempValuesAvr;
				txMessage.data[3] = tempValuesHigh;
				txMessage.data[4] = cellValues[0];
				txMessage.data[5] = cellValues[1];
				txMessage.data[6] = cellValues[3];
				txMessage.data[7] = cellValues[4];
				
				can_send_message(&txMessage);
			}
    }
}

