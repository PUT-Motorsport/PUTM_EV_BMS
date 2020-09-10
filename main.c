#include <avr/io.h>
#define F_CPU 16000000
#define SPI_CS_PIN 8


#define RESET 0
#define SET 1

float temperature_map[29][2] = {
	//    ltc value ,  temperature *C
	27190 , -20 ,
	26380 , -15 ,
	25400 , -10 ,
	24260 , -5  ,
	22960 , 0 ,
	21520 , 5 ,
	19960 , 10  ,
	18330 , 15  ,
	16660 , 20  ,
	15000 , 25  ,
	13380 , 30  ,
	11850 , 35  ,
	10420 , 40  ,
	9120  , 45  ,
	7940  , 50  ,
	6890  , 55  ,
	5970  , 60  ,
	5170  , 65  ,
	4470  , 70  ,
	3860  , 75  ,
	3350  , 80  ,
	2900  , 85  ,
	2520  , 90  ,
	2190  , 95  ,
	1910  , 100 ,
	1660  , 105 ,
	1450  , 110 ,
	1270  , 115

};

uint8_t ltc_config[6] = {0xFC, (1874 & 0xff), (1874>>4)|(2625<<4), (2625>>4), 0, 0};
uint16_t cell_values[10];

uint16_t temp_values[5];
float temp_values_float[5];

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
	DDRB = (1<<DDB2)|(1<<DDB1)|(1<<DDB5)|(1<<DDB0);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPSR = (0<<SPI2X);
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(0<<SPR1)|(1<<SPR0);
}
void SPI_Transmit(int8_t  data)  // Byte to be written to SPI port
{
	SPDR = data;                  //! 1) Start the SPI transfer
	while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complete
}

int8_t SPI_Read(int8_t  data) //!The data byte to be written
{
	SPDR = data;                  //! 1) Start the SPI transfer
	while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complete
	return SPDR;                  //! 3) Return the data read
}

void SPI_TransmitArray(uint8_t data[], //Array of bytes to be written on the SPI port
uint8_t len, // Option: Number of bytes to be written on the SPI port
int transmitDelay)
{
	for (uint8_t i = 0; i < len; i++)
	{
		SPI_Transmit((int8_t)data[i]);
		d_ms(transmitDelay);
	}
}
void SPI_TransmitRead(uint8_t tx_Data[],//array of data to be written on SPI port
uint8_t tx_len, //length of the tx data arry
uint8_t *rx_data,//Input: array that will store the data read by the SPI port
uint8_t rx_len, //Option: number of bytes to be read from the SPI port
int transmitDelay
)
{
	for (uint8_t i = 0; i < tx_len; i++)
	{
		SPI_Transmit(tx_Data[i]);
		d_ms(transmitDelay);
	}

	for (uint8_t i = 0; i < rx_len; i++)
	{
		rx_data[i] = (uint8_t)SPI_Read(0xFF);
		d_ms(transmitDelay);
	}

}

void ltc_wakeup()
{
	uint8_t tab[2] = {0xFF};

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 2, 1);
	//digitalWrite(SPI_CS_PIN, SET);
	PORTB^=1<<DDB0;
}

void ltc_start_cell_adc()
{
	uint8_t tab[12];
	uint16_t pec;
	// configuration
	tab[0] = 0x00;
	tab[1] = 0x01;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;
	//
	tab[4] = ltc_config[0];//0xFC | 0b000; //ltc_config[0];
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
	SPI_TransmitArray(tab, 12, 100);
	//digitalWrite(SPI_CS_PIN, SET);
	PORTB^=1<<DDB0;

	// adc conversion
	memset(tab, 0, 12);

	uint16_t cmd = 0b1001100000 | (0b00 << 7);
	tab[0] = cmd>>8;
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 4, 100);
	PORTB^=1<<DDB0;
	//digitalWrite(SPI_CS_PIN, SET);

}

void ltc_get_adc_values()
{
	uint8_t tab[100], rx_tab[100];
	uint16_t pec;

	// read cell voltage group A
	memset(tab, 0, 12);
	tab[0] = 0;
	tab[1] = 0b100;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab,12,100);
	PORTB^=1<<DDB0;
	//digitalWrite(SPI_CS_PIN, SET);

	cell_values[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	cell_values[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cell_values[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);
}

void ltc_start_temp_adc()
{
	uint8_t tab[12];
	uint16_t pec;
	// configuration
	tab[0] = 0x00;
	tab[1] = 0x01;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;
	//
	tab[4] = 0xFC | 0b000; //ltc_config[0];
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
	SPI_TransmitArray(tab, 12, 100);
	//digitalWrite(SPI_CS_PIN, SET);
	PORTB^=1<<DDB0;

	// adc conversion
	memset(tab, 0, 12);

	uint16_t cmd = 0b10001100000 | (0b00 << 7);
	tab[0] = cmd>>8;
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitArray(tab, 12, 100);
	//digitalWrite(SPI_CS_PIN, SET);
	PORTB^=1<<DDB0;
}

void ltc_get_temp_values()
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

	ltc_wakeup();

	//digitalWrite(SPI_CS_PIN, RESET);
	PORTB^=1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab,12,100);
	PORTB^=1<<DDB0;
	//digitalWrite(SPI_CS_PIN, SET);

	temp_values[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	temp_values[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	temp_values[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);
}

float temperature_calculate(uint16_t ltc_value)
{
	float retval = 0.0;
	for(int i = 1; i < 28; i++)
	{
		if(ltc_value >= (uint16_t)temperature_map[i][0])
		{
			// approximation
			retval = temperature_map[i][1] - 5.0 * ((float)ltc_value-temperature_map[i][0])/(temperature_map[i-1][0] - temperature_map[i][0]);
			break;
		}
	}
	return retval;
}


int main(void){
	// put your setup code here, to run once:

	SPI_MasterInit();
	init_PEC15_Table();
	//DDRB = 0b10111111;

	// put your main code here, to run repeatedly:
	while(1){
		
		//PORTB ^= 0b10111111;
		PORTB ^= 1 << DDB5;
		d_ms(500);
		//PORTB ^= 0b10111111;

		//_delay_ms(500);
		ltc_start_cell_adc();
		d_ms(200);
		
		ltc_get_adc_values();

		ltc_start_temp_adc();
		
		d_ms(200);
		ltc_get_temp_values();
	}

	

}
