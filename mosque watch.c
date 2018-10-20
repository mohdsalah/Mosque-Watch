#define F_CPU 1000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define LCD_DataPRT PORTA
#define LCD_DataDDR DDRA
#define LCD_RS 5
#define LCD_RW 6
#define LCD_EN 7
#define LCD_SETPRT PORTB
#define LCD_SETDDRPRT DDRB


////////////// prayer's time ////////////////////
unsigned char FAJR_H=0x04;
//unsigned char FAJR_HH='0';
//unsigned char FAJR_ML='0';
unsigned char FAJR_M=0x30;

unsigned char DOHR_H=0x12;
//unsigned char DOHR_HL='2';
//unsigned char DOHR_MH='0';
unsigned char DOHR_M=0x00;

unsigned char ASR_H=0x16;
//unsigned char ASR_HH='1';
//unsigned char ASR_ML='0';
unsigned char ASR_M=0x00;

unsigned char MAG_H=0x17;
//unsigned char MAG_HL='7';
//unsigned char MAG_MH='0';
unsigned char MAG_M=0x050;

unsigned char ESHA_H=0x19;
//unsigned char ESHA_HL='9';
//unsigned char ESHA_MH='0';
unsigned char ESHA_M=0x00;



void i2c_init (void){
   TWSR = 0x00;
   TWBR = 0x47;
   TWCR = 0x04;
}

void i2c_start()
{
   TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
   while(!(TWCR & (1<<TWINT)));
}

void i2c_write(unsigned char data){
   TWDR = data;
   TWCR = (1<<TWINT)|(1<<TWEN);
   while(!(TWCR & (1<<TWINT)));
}
unsigned char i2c_read (unsigned char ackVal){
   TWCR = (1<<TWINT)|(1<<TWEN)|(ackVal<<TWEA);
   while(!(TWCR & (1<<TWINT)));
   return TWDR;
}

void i2c_stop (){
   TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
   int k;
   for (k = 0;k<100;k++);
}

void rtc_init(){
   i2c_init();   //initialize I2C module
   i2c_start();    //transmit START condition
   i2c_write(0xD0);  //address DS1307 for write
   i2c_write(0x07);  //set register pointer to 7
   i2c_write(0x00);   //set value of location 7 to 0
   i2c_stop();   //transmit STOP condition

}

void rtc_setTime(unsigned char h,unsigned char m,unsigned char s){
   i2c_start();    //transmit START condition
   i2c_write(0xD0);  //address DS1307 for write
   i2c_write(0);  //set register pointer to 0
   i2c_write(s);   //set seconds
   i2c_write(m);   //set minutes
   i2c_write(h);   //set hour
   i2c_stop();   //transmit STOP condition
}

void rtc_setDate(unsigned char y,unsigned char m,unsigned char d){
   i2c_start();    //transmit START condition
   i2c_write(0xD0);  //address DS1307 for write
   i2c_write(0x04);  //set register pointer to 0
   i2c_write(d);   //set seconds
   i2c_write(m);   //set minutes
   i2c_write(y);   //set hour
   i2c_stop();   //transmit STOP condition
}

void rtc_getTime(unsigned char *h,unsigned char *m,unsigned char *s){
   i2c_start();    //transmit START condition
   i2c_write(0xD0);  //address DS1307 for write
   i2c_write(0);  //set register pointer to 0
   i2c_stop();   //transmit STOP condition

   i2c_start();    //transmit START condition
   i2c_write(0xD1);  //address DS1307 for write
   *h = i2c_read(1);   //read second, return ACK
   *m = i2c_read(1);   //read minute, return ACK
   *s = i2c_read(0);   //read hour, return ACK
   i2c_stop();   //transmit STOP condition
}

void rtc_getDate(unsigned char *y,unsigned char *m,unsigned char *d){
   i2c_start();    //transmit START condition
   i2c_write(0xD0);  //address DS1307 for write
   i2c_write(0x04);  //set register pointer to 0
   i2c_stop();   //transmit STOP condition

   i2c_start();    //transmit START condition
   i2c_write(0xD1);  //address DS1307 for write
   *d = i2c_read(1);   //read day, return ACK
   *m = i2c_read(1);   //read month, return ACK
   *y = i2c_read(0);   //set year, return NACK
   i2c_stop();   //transmit STOP condition
}

/*char * get_ASCII(unsigned char data){
   static char r[2] ;
    r[0] = '0' + (data>>4);
    r[1]= '0' + (data & 0xF);

   return r;

   }*/


///////////////////////////////////lcd

void Send_LCD_Com(unsigned char Command){
	LCD_DataPRT=Command & 0xF0;
	LCD_SETPRT &=~(1<<LCD_RS);           /*RS=0 for command       */
	LCD_SETPRT &=~(1<<LCD_RW);           /*RW=0 for write       */
	LCD_SETPRT |=(1<<LCD_EN);            /*EN=1      */
	_delay_us(1);
	LCD_SETPRT &=~(1<<LCD_EN);            /*EN=0      */
	_delay_us(100);

	LCD_DataPRT=Command<<4;
	LCD_SETPRT |=(1<<LCD_EN);            /*EN=1      */
	_delay_us(1);
	LCD_SETPRT &=~(1<<LCD_EN);            /*EN=0      */
	_delay_us(100);

}

void LCD_Init(){
	LCD_DataDDR=0xFF;
	LCD_SETDDRPRT=0xFF;

	LCD_SETPRT &=~(1<<LCD_EN);
	_delay_us(2000);
	Send_LCD_Com(0x33);
	Send_LCD_Com(0x32);
	Send_LCD_Com(0x28);
	Send_LCD_Com(0x0C);
	Send_LCD_Com(0x01);
	_delay_us(2000);
	Send_LCD_Com(0x06);
}

void Send_DATA_TO_LCD(unsigned char data){
	LCD_DataPRT=data & 0xF0;

	LCD_SETPRT |=(1<<LCD_RS);           /*RS=0 for command       */
	LCD_SETPRT &=~(1<<LCD_RW);           /*RW=0 for write       */
	LCD_SETPRT |=(1<<LCD_EN);            /*EN=1      */
	_delay_us(1);
	LCD_SETPRT &=~(1<<LCD_EN);            /*EN=0      */
	_delay_us(100);

	LCD_DataPRT=data<<4;
	LCD_SETPRT |=(1<<LCD_EN);            /*EN=1      */
	_delay_us(1);
	LCD_SETPRT &=~(1<<LCD_EN);            /*EN=0      */
	_delay_us(100);

}


void lcd_gotoxy (unsigned char x , unsigned char y)
{
	unsigned char firstCharAdr[] = {0x80, 0xC0, 0x94 , 0xD4};
	Send_LCD_Com(firstCharAdr[y-1] + x - 1);
	_delay_us(100);
}

void LCD_Print(char* str){
	unsigned char i=0;
	while (str[i]!=0)
	{

		Send_DATA_TO_LCD(str[i]);
		i++;
	}

}
//////////////////////////////main

void show_Time(unsigned char h,unsigned char m,unsigned char s){
	unsigned sh=(s>>4)+'0';
	unsigned sl=(s & 0x0f)+'0';

	unsigned mh=(m>>4)+'0';
	unsigned ml=(m & 0x0f)+'0';

	unsigned hh=(h>>4)+'0';
	unsigned hl=(h & 0x0f)+'0';

	Send_DATA_TO_LCD(hh);
	Send_DATA_TO_LCD(hl);
	Send_DATA_TO_LCD(':');
	Send_DATA_TO_LCD(mh);
	Send_DATA_TO_LCD(ml);
	Send_DATA_TO_LCD(':');
	Send_DATA_TO_LCD(sh);
	Send_DATA_TO_LCD(sl);
}

void show_Date(unsigned char d,unsigned char mon,unsigned char y){
	unsigned dh=(d>>4)+'0';
	unsigned dl=(d & 0x0f)+'0';

	unsigned monh=(mon>>4)+'0';
	unsigned monl=(mon & 0x0f)+'0';

	unsigned yh=(y>>4)+'0';
	unsigned yl=(y & 0x0f)+'0';


	LCD_Print("20");
	Send_DATA_TO_LCD(dh);
	Send_DATA_TO_LCD(dl);
	Send_DATA_TO_LCD(':');
	Send_DATA_TO_LCD(monh);
	Send_DATA_TO_LCD(monl);
	Send_DATA_TO_LCD(':');
	Send_DATA_TO_LCD(yh);
	Send_DATA_TO_LCD(yl);

}


int next_Prayer_Time(unsigned char h,unsigned char m){
	if(h>=ESHA_H || h<=FAJR_H){
		                       if(h==FAJR_H && m<FAJR_M)  return 1;
	                           else if(h==FAJR_H && m>FAJR_M)  return 2;
							   else if(h<FAJR_H || h>ESHA_H) return 1;}

    if(h>=FAJR_H && h<=DOHR_H ){
				       if(h==DOHR_H && m<DOHR_M) return 2;
	                                 else if(h==DOHR_H && m>DOHR_M)  return 3;
									 else if(h<DOHR_H) return 2;}

    if(h>=DOHR_H && h<=ASR_H ) {
				       if(h==ASR_H && m<ASR_M) return 3;
	                               else if(h==ASR_H && m>ASR_M ) return 4;
								   else if(h<ASR_H) return 3;}

    if(h>=ASR_H && h<=MAG_H ) {
				       if(h=MAG_H && m<MAG_M) return 4;
	                                else if(h==MAG_H && m>MAG_M)  return 5;
									else if(h<MAG_H) return 4;}

    if(h>=MAG_H && h<=ESHA_H){
		                        if(h==ESHA_H && m<ESHA_M) return 5;
					if(h==ESHA_H && m>ESHA_M) return 1;
							    if(h<ESHA_H) return 5;
	                            }



}

void alaram(unsigned char h,unsigned char m){
	if(h==FAJR_H && m==FAJR_M){
	   PORTD |= 0x08;}
	else if(h==DOHR_H && m==DOHR_M) {
	   PORTD |= 0x08;}
	else if(h==ASR_H && m==ASR_M) {
	   PORTD |= 0x08;}
	else if(h==MAG_H && MAG_M) {
	   PORTD |= 0x08;}
	else if(h==ESHA_H && m==ESHA_M) {
	   PORTD |= 0x08;}
	else PORTD &= 0xF7;
}
void buzzer_config(){
   DDRD |= 0x08; //PD3 as output
   PORTD &= 0xF7; // turn off buzzer
   }

int main()
 {
   buzzer_config();

   LCD_Init();

   unsigned char s,m,h;
   unsigned char d,mon,y;

   rtc_init();
   rtc_setTime(0x18,0x59,0x55);
   rtc_setDate(0x18,0x9,0x20);




   while (1){

  rtc_getTime(&s,&m,&h);
  rtc_getDate(&d,&mon,&y);

 alaram(h,m);


lcd_gotoxy(1 , 1);
LCD_Print("TIME ");
show_Time(h,m,s);

lcd_gotoxy(1 , 2);
LCD_Print("DATE ");
show_Date(d,mon,y);



int npt;
npt=next_Prayer_Time(h,m);
lcd_gotoxy(1,3);
LCD_Print("NEXT PRAYER ");


//lcd_gotoxy(1,4);
switch (npt)
{
	case 1:
	show_Time(FAJR_H,FAJR_M,0x00);
	break;
	case 2:
	show_Time(DOHR_H,DOHR_M,0x00);
	break;
	case 3:
	show_Time(ASR_H,ASR_M,0x00);
	break;
	case 4:
	show_Time(MAG_H,MAG_M,0x00);
	break;
	case 5:
	show_Time(ESHA_H,ESHA_M,0x00);
	break;

}



   }

   return 0;
 }
