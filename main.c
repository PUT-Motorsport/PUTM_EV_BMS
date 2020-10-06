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
volatile uint64_t timer_counter = 0;
volatile uint64_t timer_counter_2 = 0;
volatile uint64_t timer_counter_3 = 0;

/*
float temperature_map_2[29][2] = {
	//		ltc value ,  temperature *C
	27190	,	-20	,
	26380	,	-15	,
	25400	,	-10	,
	24260	,	-5	,
	22960	,	0	,
	21520	,	5	,
	19960	,	10	,
	18330	,	15	,
	16660	,	20	,
	15000	,	25	,
	13380	,	30	,
	11850	,	35	,
	10420	,	40	,
	9120	,	45	,
	7940	,	50	,
	6890	,	55	,
	5970	,	60	,
	5170	,	65	,
	4470	,	70	,
	3860	,	75	,
	3350	,	80	,
	2900	,	85	,
	2520	,	90	,
	2190	,	95	,
	1910	,	100	,
	1660	,	105	,
	1450	,	110	,
	1270	,	115

};
*/

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
};

uint8_t ltc_config[6] = {0xFC, (1874 & 0xff), (1874>>4)|(2625<<4), (2625>>4), 0, 0};
uint8_t acu_state[8]={0,0,0,0,0,0,0,0};
uint8_t cell_values[10];
uint8_t cell_values_sum;
uint8_t cell_values_low;
uint8_t cell_values_high;
uint8_t temp_values[10];
uint8_t temp_values_2[10];
uint8_t temp_values_avr;
uint8_t temp_values_high;
uint8_t reference2_value;
uint8_t reference2_value_2;
uint8_t can_flag=0;
float temp_values_float[10];
float temp_values_2_float[10];

ISR(TIMER0_COMP_vect)
{
	TCNT0 = 0;
	//timer_counter++;
	
	
	if((cell_values[0]<35000 || cell_values[0]>42300) || (cell_values[1]<35000 || cell_values[1]>42300) || (cell_values[3]<35000 || cell_values[3]>42300) || (cell_values[4]<35000 || cell_values[4]>42300)){
		acu_state|=0b1;
		if((cell_values[0]<35000 || cell_values[0]>42300)){
			if(cell_values[0]<35000){
				acu_state|=0b1000;
			}else{
				acu_state|=0b10000;
			}
		}
		if((cell_values[1]<35000 || cell_values[1]>42300)){
			acu_state|=0b10;
			if(cell_values[1]<35000){
				acu_state|=0b1000;
				}else{
				acu_state|=0b10000;
			}
		}
		if((cell_values[2]<35000 || cell_values[2]>42300)){
			acu_state|=0b100;
			if(cell_values[2]<35000){
				acu_state|=0b1000;
				}else{
				acu_state|=0b10000;
			}
		}
		if((cell_values[3]<35000 || cell_values[3]>42300)){
			acu_state|=0b110;
			if(cell_values[3]<35000){
				acu_state|=0b1000;
				}else{
				acu_state|=0b10000;
			}
		}
		
		timer_counter++;
	}
	
	if((cell_values[0]>=42000 && cell_values[0]<=42300) && (cell_values[1]>=35000 && cell_values[1]<=42300) && (cell_values[3]>=35000 && cell_values[3]<=42300) && (cell_values[4]>=35000 && cell_values[4]<=42300)){
		acu_state&=0b0;
		timer_counter = 0;
	}
	
	if(temp_values_float[1]>50 || temp_values_float[2]>50 || temp_values_float[3]>50 || temp_values_float[4]>50){
		acu_state|=0b1;
		if(temp_values_float[1]>50){
			acu_state|=0b100000;
		}
		if(temp_values_float[2]>50){
			acu_state|=0b10;
			acu_state|=0b100000;
		}
		if(temp_values_float[3]>50){
			acu_state|=0b100;
			acu_state|=0b100000;
		}
		if(temp_values_float[4]>50){
			acu_state|=0b110;
			acu_state|=0b100000;
		}
		timer_counter_2=timer_counter_2+1;
	}
	
	if(temp_values_float[1]<=50 && temp_values_float[2]<=50 && temp_values_float[3]<=50 && temp_values_float[4]<=50){
		acu_state&=0b0;
		timer_counter_2 = 0;
	}
	
	if(timer_counter >= 1000){
		PORTE &= 0 << DDE2;
	}
	
	if(timer_counter_2 >= 1000){
		PORTE &= 0 << DDE2;
	}
	
}

ISR(TIMER0_COMP_vect)
{
	timer_counter_3++;
	if((timer_counter_3%20)==0){
		can_flag=1;
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

void TickTimerInit_2(){
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

void ltc_wakeup()
{
	uint8_t tab[2] = {0xFF};

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 2);
	PORTB |= 1<<DDB0;
	
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
	
	tab[4] = ltc_config[0];
	tab[5] = ltc_config[1];
	tab[6] = ltc_config[2];
	tab[7] = ltc_config[3];
	tab[8] = ltc_config[4];
	tab[9] = ltc_config[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	ltc_wakeup();

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

	//ltc_wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 4);
	PORTB |= 1<<DDB0;

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

	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB |= 1<<DDB0;

	cell_values[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	cell_values[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cell_values[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8); //zwarty
	
	// read cell voltage group B
	cmd = (1<<15) | 0b110;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//ltc_wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,12,rx_tab);
	PORTB ^= 1<<DDB0;

	cell_values[3] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	cell_values[4] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cell_values[5] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8); //zwarty
	cell_values_sum = cell_values[0];
	cell_values_low = cell_values[0];
	cell_values_high = cell_values[0];
	
	for(int i = 1; i < 5; i++)
	{
		if(i!=2){
			cell_values_sum = cell_values_sum+cell_values[i];
			if(cell_values_low > cell_values[i]){
				cell_values_low = cell_values[i];
			}
			if(cell_values_high < cell_values[i]){
				cell_values_high = cell_values[i];
			}
		}
	}
	
	
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
	
	tab[4] = ltc_config[0];
	tab[5] = ltc_config[1];
	tab[6] = ltc_config[2];
	tab[7] = ltc_config[3];
	tab[8] = ltc_config[4];
	tab[9] = ltc_config[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	ltc_wakeup();

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

	//ltc_wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitArray(tab, 12);
	PORTB |= 1<<DDB0;
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
	
	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,20,rx_tab);
	PORTB |= 1<<DDB0;

	temp_values[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	temp_values[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	temp_values[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);

	/*temp_values_2[0] = (uint16_t)rx_tab[12] | (((uint16_t)rx_tab[13])<<8);
	temp_values_2[1] = (uint16_t)rx_tab[14] | (((uint16_t)rx_tab[15])<<8);
	temp_values_2[2] = (uint16_t)rx_tab[16] | (((uint16_t)rx_tab[17])<<8);*/

	// read cell voltage group B
	memset(tab, 0, 12);
	tab[0] = 0;
	tab[1] = 0b1110;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	//ltc_wakeup();

	PORTB ^= 1<<DDB0;
	SPI_TransmitRead(tab,20,rx_tab);
	PORTB |= 1<<DDB0;

	temp_values[3] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	temp_values[4] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	reference2_value = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);
	
	temp_values_avr = 0;

	/*temp_values_2[3] = (uint16_t)rx_tab[12] | (((uint16_t)rx_tab[13])<<8);
	temp_values_2[4] = (uint16_t)rx_tab[14] | (((uint16_t)rx_tab[15])<<8);
	reference2_value_2 = (uint16_t)rx_tab[16] | (((uint16_t)rx_tab[17])<<8);*/
	
	for(int i = 1; i < 5; i++)
	{
		temp_values_float[i] = temperature_calculate(temp_values[i]);
		temp_values_2_float[i] = temperature_calculate(temp_values_2[i]);
		
		temp_values_avr = temp_values_avr + temp_values_float[i];
	}
	temp_values_avr = temp_values_avr/4;
	
	temp_values_high=temp_values_float[1];
	for(int i = 2; i < 5; i++){
		if(temp_values_high < temp_values_float[i]){
			temp_values_high = temp_values_float[i];
		}
	}
}

int main(void)
{
	//PORTE |= 1 << DDE2;
	init_PEC15_Table();
	SPI_MasterInit();
	TickTimerInit();
	TickTimerInit_2();
	sei();
	
	can_init(BITRATE_500_KBPS);
	can_set_filter(1, &filtersetup);
	
	PORTE |= 1 << DDE2;
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
			
			ltc_start_temp_adc();
			d_ms(20);
			ltc_get_temp_values();
			
			
			if(temp_values_float[4]>30){
			PORTB ^= 1 << DDB6;
			}
    }
}

