#include <asf.h>
#include "conf_usb.h"
#include "grideye.h"
#include <system_interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#define LED PIN_PA22
static volatile bool main_b_cdc_enable = false;

void gpio_Pin_Init(void); //deklaracja funkcji inicjalizacji pinu pod leda
void inicjalizacja(void);//inicjalizacje potrzebne dla mikrokontolrea
static bool my_flag_autorize_cdc_transfert = false;
bool my_callback_cdc_enable(void)
{
	my_flag_autorize_cdc_transfert = true;
	return true;
}
void my_callback_cdc_disable(void)
{
	my_flag_autorize_cdc_transfert = false;
}

struct i2c_master_packet wr_packet;
struct i2c_master_module i2c_master_instance;
GridEye gri;
int init = 0; //zmienna dla procedury inicjalizacji
int out = 0;  //zmienna przechowywuj¹ca wynik skanowoania matrycy
char * z; //zmienna przechowywujaca tekst dla konsoli

//odczyt danych z matrycy i sumowanie ich ///////////////////////////////////////////////////////////////////

int suma(GridEye *ge) //odczyt w int sumy oczytanego obrazu (5000 temp pokojowa 7000 cieplo)
{
	int suma = 0;
	for(int i = 0; i < 64; i++)
	{
		suma += (int)ge->image[0].data[i]; //sumowanie
	}
	return suma;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

int main (void)
{
	inicjalizacja();
	
	
	while(1)
	{	
		ge_readData(&gri);//odczyt danych 
		delay_ms(40);	
		//udi_cdc_write_buf(gri.image[0].data, 128);
		out=suma(&gri);  //odczyt sumy temperatury
		//itoa(out,z,10); //zamienia int na char
		delay_ms(40);
		if (out <= 6000) z="NO";  
		else z="ok";                //jak jest ruch to zmienna out osiaga wartosc wieksza od 6000
		
		udi_cdc_write_buf(z, 2); //zapis na konsole, port 6, predkosc 115200
		port_pin_toggle_output_level(LED); //zmiana stanu diody
		//delay_ms(40);
		
	}
}

void gpio_Pin_Init()
{
	struct port_config GPIO_LED, I2C_Pins;
	port_get_config_defaults(&GPIO_LED);
	GPIO_LED.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED, &GPIO_LED);
	port_pin_set_output_level(LED, false);
	port_get_config_defaults(&I2C_Pins);
	
}

void inicjalizacja()
{
	system_init();
	delay_init();
	delay_ms(100);
	gpio_Pin_Init();
	irq_initialize_vectors();
	cpu_irq_enable();
	udc_start();
	udc_attach();
	while(!my_flag_autorize_cdc_transfert && init < 20000000)
	{
		init++;
	}
	ge_init(&gri, &i2c_master_instance, 0); //inicjalizacja matrycy
	
	delay_ms(5);
	ge_setFPS(&gri, true);
	delay_ms(105);
	ge_readAverage(&gri);
}

