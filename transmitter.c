#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ringbuf.h"

#include "esp8266/spi_struct.h"
#include "esp8266/gpio_struct.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi.h"
#define GPIO_OUTPUT_IO_0    15
#define GPIO_OUTPUT_IO_1    5
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define buff_size (1024*2)

struct status_n{
		uint8_t RX_DR;
		uint8_t TX_DS; 
		uint8_t MAX_RT;
		uint8_t RX_P_NO; 
		uint8_t TX_FULL;
		uint8_t RX_DR_MASK;
		uint8_t TX_DS_MASK;
		uint8_t MAX_RT_MASK;
		uint8_t RX_P_NO_MASK;
		uint8_t TX_FULL_MASK;
	}STATUS_NRF;
struct status_n STATUS_NRF= {0,0,0,0,0,0x40,0x20,0x10,0x0e,0x01};
//commands
	uint16_t cmd_read = 0X00;//3bit
	uint16_t cmd_write = 0x01<<(16-11);//3 bit
	uint16_t cmd_R_RX_PAYLOAD =0x61;//8bit 
	uint16_t cmd_W_TX_PAYLOAD = 0xA0;//8bit
	uint16_t cmd_FLUSH_TX = 0xE1;//8bit
	uint16_t cmd_FLUSH_RX = 0xE2;//8bit
	uint16_t cmd_REUSE_TX_PL = 0xE3;//8bit
	uint16_t cmd_NOP  = 0xff;//8bit

	
	uint32_t addr0_def = 0x08;
	uint32_t PRIM_RX = 0x01;//address 00 8bit
	uint32_t PRIM_TX = 0x00;//address 00 8bit
	uint32_t auto_retransmit = 0x00;//address 04 8bit 
	uint32_t PWR_UP =0x02;//address 00
	uint32_t RX_PW_P0 =0x4;//address 11 payload width in bytes
	
	uint32_t addr04 =0x04<<(32-5);
	uint32_t addr00 =0x00;
	uint32_t addr11 = 0x11<<(32-5);
	uint32_t addr01 = 0x01<<(32-5);
	uint32_t addr06 = 0x06<<(32-5);//rf config
	uint32_t addr08 = 0x08<<(32-5);
	uint32_t addr_status = 0x07<<(32-5);
	uint32_t addr_FIFO = 0x17<<(32-5);




static void nrf_read(uint16_t *cmd , uint8_t cmd_len, uint32_t *addr,uint8_t addr_len,uint32_t *data,uint8_t data_len){
        spi_trans_t trans;
	memset(&trans,0x0, sizeof(trans));
   	trans.bits.val = 0;            // clear all bit
	trans.bits.miso = data_len;
        trans.miso = data;
    	trans.bits.cmd = cmd_len;
    	trans.bits.addr = addr_len;     // transmit data will use 8bit address
    	trans.addr = addr;
	trans.cmd = cmd;
	vTaskDelay(0.5);
    	spi_trans(HSPI_HOST, &trans);

	}

static void nrf_write(uint16_t *cmd , uint8_t cmd_len, uint32_t *addr,uint8_t addr_len,uint32_t *data,uint8_t data_len){
        spi_trans_t trans;
	memset(&trans,0x0, sizeof(trans));
   	trans.bits.val = 0;            // clear all bit
	trans.bits.mosi=data_len;             // One time transmit only support 64bytes
        trans.mosi = data;
    	trans.bits.cmd = cmd_len;
    	trans.bits.addr = addr_len;     // transmit data will use 8bit address
    	trans.addr = addr;
	trans.cmd = cmd;
	vTaskDelay(0.5);
    	spi_trans(HSPI_HOST, &trans);

	}

void setup_spi(){
	spi_config_t spi_config;
        // Load default interface parameters
	// CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
	spi_config.interface.val = SPI_DEFAULT_INTERFACE;
		
	spi_config.interface.cpol = 0;
	spi_config.interface.cpha = 0;
	spi_config.interface.bit_tx_order = SPI_BIT_ORDER_LSB_FIRST;
	spi_config.interface.bit_rx_order = SPI_BIT_ORDER_LSB_FIRST;
	spi_config.interface.byte_tx_order = SPI_BYTE_ORDER_LSB_FIRST;
	spi_config.interface.byte_rx_order = SPI_BYTE_ORDER_LSB_FIRST;
	spi_config.interface.mosi_en = 1;
	spi_config.interface.miso_en = 1;
	spi_config.interface.cs_en =1;

    // Load default interrupt enable
    // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    // Set SPI to master mode
    // ESP8266 Only support half-duplex
    spi_config.mode = SPI_MASTER_MODE;
    // Set the SPI clock frequency division factor
    spi_config.clk_div = SPI_2MHz_DIV;
    spi_config.event_cb = NULL;
    spi_init(HSPI_HOST, &spi_config);
}
void setup_nrf(int mode){//mode =1 for prx and 0 for ptx  now only can transmit 32 bit/4 byte at a time 
	uint16_t cmd;
	uint8_t  cmd_len ;
	uint32_t addr ;
	uint8_t  addr_len;
	uint32_t data ; 
	uint8_t data_len ;


	if(mode ==1){
	// set payload width to 4 byte
		 cmd = cmd_write;
		 cmd_len =3;
		 addr = addr11;
		 addr_len = 5;
		 data = RX_PW_P0; 
		 data_len =8;
		 
		 nrf_write(&cmd , cmd_len, &addr, addr_len, &data, data_len);
		//set power up adn rx mode
		 cmd = cmd_write;
		 cmd_len =3;
		 addr = 0x00;
		 addr_len = 5;
		 data = PRIM_RX | addr0_def |  PWR_UP ; 
		 data_len =8;

		 nrf_write(&cmd , cmd_len, &addr, addr_len, &data, data_len);
	}
	else if (mode ==0){
	// set payload width to 4 byte
		 cmd = cmd_write;
		 cmd_len =3;
		 addr = addr11;
		 //addr = 0xf0000000;
		 addr_len = 5;
		 data = RX_PW_P0; 
		 data_len =8;
		 
		 nrf_write(&cmd , cmd_len, &addr, addr_len, &data, data_len);

	// set powerup adn tx mode
	cmd = cmd_write;
	cmd_len =3;
	addr = 0x00;
	addr_len = 5;
	data = PRIM_TX | addr0_def |  PWR_UP ; 
	data_len =8;

	nrf_write(&cmd , cmd_len, &addr, addr_len, &data, data_len);
	}

}


 void flush_buffer(uint8_t buf){//if buff ==1 flush receiver buffer else if "0" flush transmitter buffer
	uint16_t cmd;
	uint8_t  cmd_len;
	uint32_t addr =0x00;
	uint8_t  addr_len=0;
	uint32_t data =0; 
	uint8_t data_len =0;

	if(buf ==1){
	cmd = 0xE2;
	cmd_len= 8;

	}
	else if(buf ==0){
	cmd = 0xE1;
	cmd_len= 8;
		
	}
	nrf_write(&cmd , cmd_len, &addr, addr_len, &data, data_len);
	//return 1;
	}



void read_status(){
	uint16_t cmd =cmd_read;
	uint8_t cmd_len =3;
	uint32_t addr = 0x07<<(32-5);
	uint8_t addr_len = 5;
	uint32_t data = 0x00; 
	uint8_t data_len =8;
	nrf_read(&cmd , cmd_len, &addr, addr_len, &data, data_len);
	STATUS_NRF.RX_DR   =  (data&STATUS_NRF.RX_DR_MASK)  !=0; //rx received 6 th bit
	STATUS_NRF.TX_DS   =  (data&STATUS_NRF.TX_DS_MASK)  !=0 ; //tx send 5  th bit
	STATUS_NRF.MAX_RT  =  (data&STATUS_NRF.MAX_RT_MASK) !=0 ; //max retry 4  th bit
	STATUS_NRF.RX_P_NO =  (data&STATUS_NRF.RX_P_NO_MASK)>>1; // payload address 3  th bit to 1 th bit
	STATUS_NRF.TX_FULL =  (data&STATUS_NRF.TX_FULL_MASK)!=0; //transmitter buffer full 0  th bit;


}


 void write_status(){
	uint16_t cmd =cmd_write;
	uint8_t cmd_len =3;
	uint32_t addr = 0x07<<(32-5);
	uint8_t addr_len = 5;
	uint8_t data_len =8;
	uint32_t data = (STATUS_NRF.RX_DR<<6) | (STATUS_NRF.TX_DS<<5)|(STATUS_NRF.MAX_RT<<4)|(STATUS_NRF.RX_P_NO<<1)|(STATUS_NRF.TX_FULL); //rx received 6 th bit
	//tx send 5  th bit
	 //max retry 4  th bit
	// payload address 3  th bit to 1 th bit
	//transmitter buffer full 0  th bit;
	nrf_write(&cmd , cmd_len, &addr, addr_len, &data, data_len);
	};


bool receive_data(uint32_t *data){
	gpio_set_level(GPIO_OUTPUT_IO_1,true);	
	read_status();		
	*data=0x00;
	if(STATUS_NRF.RX_DR>0){	
		gpio_set_level(GPIO_OUTPUT_IO_1,false);
		
		nrf_read(&cmd_R_RX_PAYLOAD , 8, 0x00, 0, data,RX_PW_P0*8);
		
		
		STATUS_NRF.RX_DR = 0x01;
		
		
		write_status();	
		flush_buffer(1);
		return 1;
		}
	return 0;
	}


bool send_data(uint32_t *data){
	uint16_t cmd = cmd_W_TX_PAYLOAD;
	uint8_t cmd_len =8;
	uint32_t addr = 0x00;
	uint8_t addr_len = 0;
	nrf_write(&cmd , cmd_len, &addr, addr_len, data, RX_PW_P0*8);
	vTaskDelay(1/ portTICK_RATE_MS);
	gpio_set_level(GPIO_OUTPUT_IO_1,false);
	gpio_set_level(GPIO_OUTPUT_IO_1,true);
	while(true){
		read_status();
		if(STATUS_NRF.TX_DS>0){
			gpio_set_level(GPIO_OUTPUT_IO_1,false);
			STATUS_NRF.TX_DS=0x1;
			write_status();
			flush_buffer(0);
			return 1;
			break;		
				}
		else if(STATUS_NRF.MAX_RT>0){
			gpio_set_level(GPIO_OUTPUT_IO_1,false);
			STATUS_NRF.MAX_RT=0x01;
			write_status();
			flush_buffer(0);
			return 0;
			break;
				}
		}
	}

static void nrf(){
	
	uint32_t data;
	
	gpio_set_level(GPIO_OUTPUT_IO_1,false);
	setup_spi();
	setup_nrf(0);//1 for rx and 0 for tx 

	 data = 0x12345678; 
	// data_len =32;
	
	while (1) 
		{
		for(uint32_t i=0 ;i<0xffffffff;i++){
		send_data(&i);// for receiving receive_data(uint32_t *data)
			if(i>0xffffff){i=0;}		
		}
		
		}
		
} 


void app_main(){

	gpio_config_t io_conf={
		.pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
		.mode =GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE

		};
	gpio_config(&io_conf);
	//gpio_set_level(GPIO_OUTPUT_IO_1,true);
	//gpio_set_level(GPIO_OUTPUT_IO_1,false);
	printf("test\r\n");
	//vTaskDelay(1000);
	//gpio_set_level(GPIO_OUTPUT_IO_1,true);
	vTaskDelay(100);
	   

	
	xTaskCreate(nrf,"test",2048,NULL,10,NULL);
	while (1) vTaskDelay(1000/ portTICK_RATE_MS);
}
