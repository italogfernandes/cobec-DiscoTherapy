#include<nrf24le1.h>
#include<nRF-SPIComands.h>
#include<hal_adc.h>
#include<hal_delay.h>

#define LED P03
#define CMDREAD 0xAA

void adc_init(){
  hal_adc_set_input_channel(HAL_ADC_INP_AIN7);                     
  hal_adc_set_reference(HAL_ADC_REF_VDD);                        
  hal_adc_set_input_mode(HAL_ADC_SINGLE);                             
  hal_adc_set_conversion_mode(HAL_ADC_SINGLE_STEP); 
  //hal_adc_set_sampling_rate(HAL_ADC_8KSPS);
	//hal_adc_set_power_down_delay(HAL_ADC_PDD_INF);
	//hal_adc_set_acq_window(HAL_ADC_AQW_36US);
  hal_adc_set_resolution(HAL_ADC_RES_12BIT);                          
  hal_adc_set_data_just(HAL_ADC_JUST_RIGHT); 	
}
 

void setup(){
	P0DIR = 0xFF; //All input
  P0DIR &= ~(1<<3); //except the LED
  LED = 1; delay_ms(500); LED = 0; delay_ms(500); LED = 1; delay_ms(500);
  rf_init(ADDR_HOST,ADDR_HOST,16,RF_DATA_RATE_2Mbps,RF_TX_POWER_0dBm);
	adc_init();
}


void main(){
  setup();
  while(1){
    if(newPayload){
			if(rx_buf[0] == CMDREAD){
				//read_and_send_values();
				hal_adc_start();                                               
				while(hal_adc_busy()); 
				tx_buf[0] = hal_adc_read_MSB();
				tx_buf[1] = hal_adc_read_LSB();
				TX_Mode_NOACK(2);
				RX_Mode();
			}
			sta = 0;
			newPayload = 0;
    }
  }
}