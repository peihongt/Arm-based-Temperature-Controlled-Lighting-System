#include "lcd_1602.h"
#include "main.h"
static void delay_us(uint32_t delay){
	delay*=32;
	while(delay--);
}

static void lcd_send_4bit(uint8_t data){	
	
	if(data & 0x10) d4(1); else d4(0);
	if(data & 0x20) d5(1); else d5(0);
	if(data & 0x40) d6(1); else d6(0);
	if(data & 0x80) d7(1); else d7(0);
	
}

static void lcd_send(int8_t rs,uint8_t data){
	rs(rs);rw(0);
	lcd_send_4bit(data);
	en(1);delay_us(100);en(0);
	lcd_send_4bit(data<<4);
	en(1);delay_us(100);en(0);
}

void lcd_cmd(uint8_t command){
	lcd_send(1,command);
}

void lcd_data(char c){
	lcd_send(1,(uint8_t)c);
}

void lcd_init(void){
	//bl(1);
	lcd_send(0,0x33);
  lcd_send(0,0x32);
  lcd_send(0,0x28);
  lcd_send(0,0x0C);
  lcd_send(0,0x06);
  lcd_send(0,0x01);
	HAL_Delay(2);
}

void lcd_clr(void){
    lcd_send(0,0x01);
    HAL_Delay(2);
}


void lcd_gotoxy(char x, char y){
	
    lcd_send(0,0x80+x+(y*0x40));
}

void lcd_puts(char *text){
    while(*text){
        lcd_data(*text);
        text++;
    }
}
