/*
 * The connections
 * I2C1_SCL - PB08
 * I2C1_SDA - PB09
 *RTC(IN1307) DS1307
 */
//---------------------------header----------------------
#include "stm32f4xx.h"
#include <stdio.h>

    					/*0x00   0x01 0x02 0x03 0x04 0x05 0x06 */
 char timeDateToSet[15] = {55, 59, 10, 3, 9, 12, 22, 0};
    						/*55:58:10 3(thu), 26(date), 9(sep) 2017*/
                                /* 2017 September 26, Tuesday, 10:58:55 */
char timeDateReadback[15];
char set_time[15];
unsigned char numaric_value[20];
unsigned char numaric_value[20];
int rtc_data[15];

char buffer[15];
//---------------------------define-------------------------
#define DS1307_ADDR_SEC 		0x00
#define DS1307_ADDR_MINIUTE 	0x01
#define DS1307_ADDR_HRS 		0x02
#define DS1307_ADDR_DAY 		0x03
#define DS1307_ADDR_DATE 		0x04
#define DS1307_ADDR_MONTH 		0x05
#define DS1307_ADDR_YEAR 		0x06


#define SLAVE_ADDR 0xD0			//0b1101000		//0xD0	//0x68    /* 1101 000.    DS1337 */
#define DS1307_SLAVE_ADDR 0b1101000 	//0xD0   //0x68
#define DS1307_CH_DATA 	 0x00                /*~(1<<7)*/

// Define Time Modes
#define AM_Time          0
#define PM_Time          1
#define TwentyFourHoursMode  2
// Define days
#define Sunday 		1
#define Monday 		2
#define Tuesday 	3
#define Wednesday 	4
#define Thursday 	5
#define Friday 		6
#define Saturday 	7



//void delayMs(int n);
 void delay(uint32_t  x);
 uint8_t bcd_to_binary(uint8_t value);
 void RTC_BCD_data(char *bin);
//-----------------------------------------I2C-----------------------
void I2C1_init(void);
void I2C1_burstWrite(char saddr, char maddr, int n, char* data);
void I2C1_burstRead(char saddr, char maddr, int n, char* data);
void uart2_init();
void uart2_write(unsigned char data);
void uart_string_write(char *ptr_str);

//------------------------------------converts function-----------
void deci_2_bcd_conver(char *ptr);
//void bcd_2_deci_conver(int *ptr);
//void bcd_2_deci_conver(char *ptr);
void split_num_conditinal(int digit);
void digit_split(unsigned int p);
int decimalToBCD(int decimal);
int BCDtoDecimal(int BCD);

//------------------------------------LCD Fun-----------------------
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_pin_init(void);
void LCD_disp_init();

void lcd_send_string (char *str);
//---------------------------------
void Get_Time();
//------------------------------Time-structure---------------------
typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME time;




int main(void)
{

    I2C1_init();
    uart2_init();
    LCD_pin_init();
    LCD_disp_init();

 //   uart_string_write("start");
    delay(5);
//    LCD_data('m');
    lcd_send_string("RTC:");
  //  deci_2_bcd_conver(timeDateToSet);
  //  I2C1_burstWrite(SLAVE_ADDR, 0x00, 7, set_time);


    delay(100);

    while (1)
    {
    	LCD_command(0x80);

    	Get_Time();		//reading RTC data

    	 sprintf (buffer,"%02d:%02d:%02d", time.hour, time.minutes, time.seconds);
    	// delay(1);
    	 lcd_send_string(buffer);
    	// delay(1);

    	 sprintf (buffer,"%02d-%02d-20%02d", time.dayofmonth, time.month, time.year);
    	// delay(1);
    	  LCD_command(0xC0);
    	 lcd_send_string(buffer);


/*    	 for(int i=0;i<8;i++)
    	 {
    		 digit_split(rtc_data[i]);
    		 //split_num_conditinal(rtc_data[i]);
    	 }
    	*/
  /*  	 while(1)
   // 	 {
  //  		 for(int i=0;i<19;i++)
    		 {
    		 uart2_write(numaric_value[i]);
    		 delay(10);
    		 }
    	 }
    */
    }

}

void I2C1_init(void)
{
		RCC->AHB1ENR |=  2;                     /* Enable GPIOB clock */
	    RCC->APB1ENR |=  0x00200000;	        /* Enable I2C1 clock */

	    /* configure PB8, PB9 pins for I2C1 */
	    GPIOB->AFR[1]   &= ~0x000000FF;         /* PB8, PB9 I2C1 SCL, SDA */
	    GPIOB->AFR[1]   |=  0x00000044;
	    GPIOB->MODER    &= ~0x000F0000;         /* PB8, PB9 use alternate function */
	    GPIOB->MODER    |=  0x000A0000;
	    GPIOB->OTYPER   |=  0x00000300;         /* output open-drain */
	    GPIOB->PUPDR    &= ~0x000F0000;         /* with pull-ups */
	    GPIOB->PUPDR    |=  0x00050000;

	    I2C1->CR1       =   0x8000;             /* software reset I2C1 */
	    I2C1->CR1       &= ~0x8000;             /* out of reset */
	    I2C1->CR2       =   0x0010;             /* peripheral clock is 16 MHz */
	    I2C1->CCR       =   80;                 /* standard mode, 100kHz clock */
	    I2C1->TRISE     =   17;                 /* maximum rise time */
	    I2C1->CR1       |=  0x0001;             /* enable I2C1 module */
}

/* this funtion writes multiple bytes of data to the memory location maddr of
 * a device with I2C slave device address saddr.
 */
void I2C1_burstWrite(char saddr, char maddr, int n, char* data)
{
    int i;
    volatile int tmp;

    while (I2C1->SR2 & 2);                  /* wait until bus not busy */
    I2C1->CR1 &= ~0x800;                    /* disable POS */
    I2C1->CR1 |= 0x100;                     /* generate start */
    while (!(I2C1->SR1 & 1));               /* wait until start flag is set */
    I2C1->DR = saddr /*<< 1*/;                  /* transmit slave address */
    while (!(I2C1->SR1 & 0x2));               /* wait until addr flag is set */
    tmp = I2C1->SR2;                        /* clear addr flag */
    while (!(I2C1->SR1 & 0x80));            /* wait until data register empty */
    I2C1->DR = maddr;                       /* send memory address */

    /* write all the data */
    for (i = 0; i < n; i++) {
        while (!(I2C1->SR1 & 0x80));        /* wait until data register empty */
        I2C1->DR = *data++;                 /* transmit memory address */
    }

    while (!(I2C1->SR1 & 4));               /* wait until transfer finished */
    I2C1->CR1 |= 0x200;                     /* generate stop */
}

void I2C1_burstRead(char saddr, char maddr, int n, char* data)
{
    volatile int tmp;

    while (I2C1->SR2 & 2);                  /* wait until bus not busy */
    I2C1->CR1 &= ~0x800;                    /* disable POS */
    I2C1->CR1 |= 0x100;                     /* generate start */
    while (!(I2C1->SR1 & 1));               /* wait until start flag is set */
    I2C1->DR = saddr /*<< 1*/;                  /* transmit slave address + Write */
    while (!(I2C1->SR1 & 2));               /* wait until addr flag is set */
    tmp = I2C1->SR2;                        /* clear addr flag */
    while (!(I2C1->SR1 & 0x80));            /* wait until transmitter empty */
    I2C1->DR = maddr;                       /* send memory address */
    while (!(I2C1->SR1 & 0x80));            /* wait until transmitter empty */

    I2C1->CR1 |= 0x100;                     /* generate restart */
    while (!(I2C1->SR1 & 1));               /* wait until start flag is set */
    I2C1->DR = saddr /*<< 1*/ | 1;              /* transmit slave address + Read */
    while (!(I2C1->SR1 & 2));               /* wait until addr flag is set */
    tmp = I2C1->SR2;                        /* clear addr flag */
    I2C1->CR1 |=  0x0400;                   /* Enable Acknowledge */

    while(n > 0)
    {
        /* One byte left */
        if(n == 1)
        {
            I2C1->CR1 &= ~(0x400);          /* Disable Acknowledge */
            I2C1->CR1 |= 0x200;             /* Generate Stop */
            while (!(I2C1->SR1 & 0x40));    /* Wait for RXNE flag set */
            *data++ = I2C1->DR;             /* Read data from DR */
            break;
        }
        else
        {
            while (!(I2C1->SR1 & 0x40));    /* Wait until RXNE flag is set */
            (*data++) = I2C1->DR;           /* Read data from DR */
            n--;
        }
    }
}

void Get_Time()
{
	//uint8_t get_time[7];
	I2C1_burstRead(SLAVE_ADDR, 0x00, 7, timeDateReadback);

	time.seconds = BCDtoDecimal(timeDateReadback[0]);		////conversion back
	time.minutes = BCDtoDecimal(timeDateReadback[1]);
	time.hour = 	BCDtoDecimal(timeDateReadback[2]);
	time.dayofweek = BCDtoDecimal(timeDateReadback[3]);
	time.dayofmonth = BCDtoDecimal(timeDateReadback[4]);
	time.month = BCDtoDecimal(timeDateReadback[5]);
	time.year = BCDtoDecimal(timeDateReadback[6]);
}

//-------------------------------------uart---------------------------------

void uart2_init()
{
	RCC->AHB1ENR |=0x1;			//peripheral clock enable of GPIOA
	RCC->APB1ENR |= 0x20000;	//USART2 clock enable
	GPIOA->MODER &= ~0xf0;
	GPIOA->MODER |=0xA0;		//set Alternate function mode of PA2 =TX; PA3 =RX;
	GPIOA->AFR[0] &= ~0xFF00;			//clear bit
	GPIOA->AFR[0] |=0x7700;		//configure USART2 alternate function

	/*set buadrate 9600 =16000000/9600 = 1666.666 eqals to 16667 convert hex is 0x683;	*/
	USART2->BRR =0x683;

	USART2->CR1 |= 0x8;			//TE :Transmitter enable
	USART2->CR1 |= 0x4;			//RE: Receiver enable
	USART2->CR1 |= 0x2000;		//UE: USART enable
}
void uart2_write(unsigned char data)
{
	while(!(USART2->SR & 0x80));
	USART2->DR = (data);// & 0xff);
}


void uart_string_write(char *ptr_str)
{
	int i;
	for(i=0;*(ptr_str+i)!='\0';i++)
	{
		uart2_write(*(ptr_str+i));
	}
}


void delay(uint32_t  x)
{
	for(int i=0;i<x;i++)
	{
		for(int j=0;j<3000;j++);
	}
}

int BCDtoDecimal(int BCD)
{
	return(((BCD>>4)*10) + (BCD & 0xf));
}

int decimalToBCD(int decimal)
{
	return(((decimal/10)<<4) | (decimal%10));
}

void deci_2_bcd_conver(char *ptr)
{
	int i=0;
	while(i<7)
	{
		set_time[i] = decimalToBCD(*ptr);
		i++;
		ptr++;
	}
}
/*
void bcd_2_deci_conver(char bcd)
{
	int i=0;
	while(i<7)
	{
		rtc_data[i++] = BCDtoDecimal(*ptr++);
		//i++;ptr++;
	}

}
*/
/*
void digit_split(unsigned int p)
	{

	int k,w,b=1,i=0;
		k=p;
		while(k>=10)
		{
			b=b*10;
			k=k/10;
		}
		while(b>=1)
		{
			w=p/b;
			p=p%b;
			b=b/10;
			numaric_value[i] = (w+48);
		//	uart2_write(w+48);
			delay(500);
			i++;
			//
			LCD_data(w+48);

		}
		LCD_data(' ');
	//	uart2_write(' ');
}
*/

void LCD_pin_init()
{
	//port initialization
	RCC -> AHB1ENR |= 0x06;			//enable clock for port B and C
	GPIOB -> MODER &= ~0x0000FC00;	//clear pin PB5,6,7
	GPIOB -> MODER |= 0x00005400;	//set as output PB5,6,7
	GPIOB -> ODR &= 0x0000FF1F;		// making pin PB5,6,7

	GPIOC -> MODER &= ~0x0000FFFF;
	GPIOC -> MODER |= 0x00005555; // port c pin 0-7 configured as output pin
}

void LCD_disp_init()
{

	LCD_command(0x38);//8 bit 2*16 data
	delay(100);
	LCD_command(0x06);//entry and cursur move right to left
	delay(100);
	LCD_command(0x01);//clear screen
	delay(100);
	LCD_command(0x80);//force curser to bigining of 1st row
	delay(100);
	//LCD_command(0x0f);//clear screen
	LCD_command(0x0C);	//disp on cursur off
	delay(100);

}

void LCD_command(unsigned char command)
{
	GPIOB->ODR &= 0xFF9F;//RS= 0;
	GPIOB->ODR &= 0xFFBF;//R/W = 0;

	GPIOC->ODR = command;
	GPIOB->ODR |= 0x0080;//E = 1
	delay(50);
	GPIOB->ODR &= ~0x0080;//clear E

	if(command<4)
		delay(2);
	else
		delay(1);

}

void LCD_data(char data)
{
	GPIOB->ODR |= 0x0020;//RS= 1;
	GPIOB->ODR &= 0xFFBF;//R/W = 0;
	GPIOC->ODR = data;//put data
	GPIOB->ODR |= 0x0080;//E = 1
	delay(50);
	GPIOB->ODR &= ~0x0080;//clear E
	delay(1);
}

void lcd_send_string (char *str)
{
	int i;
	for(i=0;*(str+i)!='\0';i++)
	{
		LCD_data(*(str+i));
		//delay(80);
	}
	/*
	while (*str )
	{
		LCD_data(*str++);
		delay(80);
	}
	*/
}


//----------------------------------------------------------------------
/*
void split_num_conditinal(int digit)
{
	int i=16;
	int temp = digit;
	while(digit!=0)
	{
		if(temp < 99)
		{
			numaric_value[i-1] = ((digit%10)+'0');
			digit=digit/10;
			i--;
			numaric_value[17]=0;
		}
		else
		{
			numaric_value[i] = ((digit%10)+'0');
			//printf(" %d ",(num%10));
			digit=digit/10;
			i--;
		}
		numaric_value[17] = 0;
	}
}
*/
/*
uint8_t bcd_to_binary(uint8_t value)
{
    uint8_t m,n;

     m = (uint8_t)((value>>4) * 10);
     n = value & 0x0F;
     return(m+n);
}

void RTC_BCD_data(char *bin)
{
	for(int i=0;i<3;i++)
	{
		timeDateReadback[i] = bcd_to_binary(*bin++);
	}
}
*/
