/*
 * bms_1.c
 *
 * Created: 09/25/2020 20:46:55
 * Author : Maks
 */ 

#include <avr/io.h>
#include <string.h>

#define F_CPU 16000000UL

#include <util/delay.h>
#include <avr/interrupt.h>

//	stan licznika timera
volatile uint64_t timer_counter = 0;

/*
float temperature_map[29][2] = {
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
};*/

uint8_t ltc_config[6] = {0xFC, (1874 & 0xff), (1874>>4)|(2625<<4), (2625>>4), 0, 0};
uint16_t cell_values[10];
uint16_t temp_values[10];

ISR(TIMER0_COMP_vect)
{
	TCNT0 = 0;
	//timer_counter++;
	
		
	
	
	//if(timer_counter >= DRS_MAX_TYPE_CHANGE_TICKS){
	//	DRS_type_change_possible = 0;
	//}
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
  DDRB = (1<<DDB2)|(1<<DDB1)|(1<<DDB5)|(1<<DDB0)|(1<<DDB6);
  DDRE = (1<<DDE2);
  /* Enable SPI, Master, set clock rate fck/16 */
  SPSR = (0<<SPI2X);
  SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(0<<SPR1)|(1<<SPR0);
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
//int transmitDelay
)
{
  for (uint8_t i = 0; i < len; i++)
  {
    SPI_Transmit((int8_t)data[i]);
    //d_ms(transmitDelay);
  }
}
void SPI_TransmitRead(uint8_t *tx_Data,//array of data to be written on SPI port
uint8_t tx_len, //length of the tx data array
uint8_t *rx_data//Input: array that will store the data read by the SPI port
//uint8_t rx_len, //Option: number of bytes to be read from the SPI port
//int transmitDelay
)
{
  /*for (uint8_t i = 0; i < tx_len; i++)
  {
    SPI_Transmit(tx_Data[i]);
    d_ms(transmitDelay);
  }*/

  for (uint8_t i = 0; i < tx_len; i++)
  {
    rx_data[i] = (uint8_t)SPI_Read(tx_Data[i]);
    //d_ms(transmitDelay);
  }

}

void ltc_wakeup()
{
	uint8_t tab[2] = {0xFF};

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 2);
	PORTB|=1<<DDB0;
	//digitalWrite(SPI_CS_PIN, SET);
	
}

void ltc_start_cell_adc()
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
	//
	tab[4] = ltc_config[0];//0xFC | 0b000; 
	tab[5] = ltc_config[1];
	tab[6] = ltc_config[2];
	tab[7] = ltc_config[3];
	tab[8] = ltc_config[4];
	tab[9] = ltc_config[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 12);
	//digitalWrite(SPI_CS_PIN, SET);
	PORTB|=1<<DDB0;

	// adc conversion
	memset(tab, 0, 12);

	cmd = 0b1001100000 | (0b00 << 7);
	tab[0] = cmd>>8;
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 4);
	PORTB|=1<<DDB0;
	//digitalWrite(SPI_CS_PIN, SET);

}

void ltc_get_adc_values()
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

	ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB|=1<<DDB0;
	//digitalWrite(SPI_CS_PIN, SET);

	cell_values[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	cell_values[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cell_values[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8); //zwarty
	
	// read cell voltage group B
	/*cmd = (1<<15) | 0b110;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//ltc_wakeup();

	PORTB^=1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB^=1<<DDB0;

	cell_values[3] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8); //zwarty ?
	cell_values[4] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cell_values[5] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);
*/
}

void ltc_start_temp_adc()
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
	//
	tab[4] = ltc_config[0]; //0xFC | 0b000;
	tab[5] = ltc_config[1];
	tab[6] = ltc_config[2];
	tab[7] = ltc_config[3];
	tab[8] = ltc_config[4];
	tab[9] = ltc_config[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 12);
	PORTB|=1<<DDB0;
	//digitalWrite(SPI_CS_PIN, SET);

	// adc conversion
	memset(tab, 0, 12);

	cmd = 0b10001100000 | (0b00 << 7);
	tab[0] = cmd>>8;
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//ltc_wakeup();

	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 12);
	PORTB|=1<<DDB0;
}

void ltc_get_temp_values()
{
	uint8_t tab[100], rx_tab[100];
	uint16_t pec;
	uint16_t cmd = (1<<15) | 0b1100;

	// read gpio voltage group A
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	ltc_wakeup();

	PORTB^=1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB|=1<<DDB0;

	temp_values[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	temp_values[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	temp_values[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);
	
	// read gpio voltage group B
	cmd = (1<<15) | 0b1110;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	ltc_wakeup();

	PORTB^=1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB|=1<<DDB0;

	temp_values[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	temp_values[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	temp_values[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);
	
	
}

int main(void)
{
	init_PEC15_Table();
	SPI_MasterInit();
	TickTimerInit();
	/*
	uint8_t tab[12];
	uint8_t tab_r[12];
	uint16_t cmd=(1<<15)|(0b101100);
	ltc_wakeup();
	
	tab[0]=(cmd>>8);
	tab[1]=cmd;
	uint16_t pec=pec15(tab,2);
	tab[2]=(pec>>8);
	tab[3]=pec;
	
	PORTB^=1<<DDB0;
	SPI_TransmitRead(tab, 12, tab_r);
	PORTB|=1<<DDB0;
	
	uint64_t sid;
	memcpy(&sid,&tab_r[4],6);
	if(sid!=0x0){
		PORTB |= 1 << DDB6;
	}*/
	
    /* Replace with your application code */
    while (1) 
    {
		PORTB ^= 1 << DDB5;
		d_ms(500);
		
			/*
			PORTB^=1<<DDB0;
			SPI_TransmitRead(tab, 12, tab_r);
			PORTB|=1<<DDB0;
			
			uint64_t sid;
			memcpy(&sid,&tab_r[4],6);
			if(sid!=0xFFFFFFFFFFFF){
				PORTB ^= 1 << DDB6;
			}*/
			
			ltc_start_cell_adc();
			d_ms(20);
			ltc_get_adc_values();
    }
}

