//programinha pra fazer media movel de um sinal que vc ja recebeu
//1000hz
//
#include<nrf24le1.h>
#include<timer0.h>
#include<nRF-SPIComands.h>
#include<hal_adc.h>
#include<hal_delay.h>

#define Aquire_Freq 1000 //frequencia recebida
#define LED P04
#define MY_SUB_ADDR 0x01
#define tamanhoDaFila 50 // 1000hz de frequencia recebida, andrei disse que a media movel pode ser de 20hz
//1000 amostras por segundo, fazendo uma media movel com 

uint16_t emg_read;
uint16_t media_movel;
uint16_t xdata FilaMedMovel[tamanhoDaFila];
uint8_t count = 0;

void read_and_send_values();
void EnqueueDequeue (uint16_t *Fila, uint16_t NovoNumero);

// Sensor id |  ... | data | ... |
void send_to_host(uint8_t data_msb,uint8_t data_lsb){
  tx_buf[0] = MY_SUB_ADDR;
	tx_buf[1] = data_msb;
	tx_buf[2] = data_lsb;
	TX_Mode_NOACK(3);
	RX_Mode();
}

void adc_init(){
  hal_adc_set_input_channel(HAL_ADC_INP_AIN7);                     
  hal_adc_set_reference(HAL_ADC_REF_VDD);                        
  hal_adc_set_input_mode(HAL_ADC_SINGLE);                             
  hal_adc_set_conversion_mode(HAL_ADC_SINGLE_STEP);               
  hal_adc_set_resolution(HAL_ADC_RES_12BIT);                          
  hal_adc_set_data_just(HAL_ADC_JUST_RIGHT);    
}
 
void setup(){
  P0DIR |= 1<<3;
  P0DIR &= ~(1<<4);
  LED = 1; delay_ms(500); LED = 0; delay_ms(500); LED = 1; delay_ms(500);
  rf_init(ADDR_HOST,ADDR_HOST,16,RF_DATA_RATE_2Mbps,RF_TX_POWER_0dBm);
	adc_init();
  setup_T0_freq(Aquire_Freq,1);//Time em +-1000
}

void main(){
  setup();
  while(1){
    
    if(timer_elapsed){
      read_and_send_values();
			timer_elapsed = 0;
    }
  }
}

void read_and_send_values(){
	hal_adc_start();                                  // Start ADconversion                                                    
	while(hal_adc_busy());                            // Wait for data ready    
	emg_read = 0;
	emg_read = hal_adc_read_LSB();                     // Read LSB of output
	emg_read |= ((uint16_t)hal_adc_read_MSB()) << 8;   // Add MSB of output
	
    //fazer media que chega a 1000hz e despaixar a 100hz
    //sessao de codigo modificada por tulio, dia 17/04
    //fazendo calculo de uma media movel de 20hz pra ser usada na transmissao 
  EnqueueDequeue(FilaMedMovel, emg_read); // adiciona o novo valor e remove o valor mais antigo 
  //ou seja, move a janela da qual sera feita a media movel
  count ++;
  //checo se count eh 10 pq quero passar de 1000hz para 100hz,
  //entao nao faz sentido caucular a media movel para 1000 amostras por segundo,
  //portanto vou calcular apenas para uma a cada 10 amostras (1000hz/10 = 100hz)
  if(count == 50)//espero count tar em 50 pra ter certeza que a fila ta cheia (nao tem zeros)
  {
		uint8_t i = 0;
    for (i=0 ; i<tamanhoDaFila ; i++)
    {//soma todos os valores da fila
      media_movel = media_movel + FilaMedMovel[i];
    }
    count = count -10;//reseta o count
    media_movel = media_movel/tamanhoDaFila;//faz a media
    send_to_host((uint8_t) (media_movel >> 8),(uint8_t) media_movel);// envia a media dos ultimos 50 valores 
  }
  
    
    //fazer media    
}

//sessao de codigo modificada por tulio, dia 17/04
//criando um EnqueueDequeue para o array
//essa funcao remove o numero do comeco da fila (numero mais velho) e insere um numero no final da fila (numero mais novo)
void EnqueueDequeue (uint16_t *Fila, uint16_t NovoNumero)
{//oq era 1,2,3,4,5,6,7,8
 //vira   2,3,4,5,6,7,8,NovoNumero
	uint8_t i=1;
  for (i = 1; i< tamanhoDaFila; i++)
  {//transforma em 2,3,4,5,6,7,8,8
    Fila[i-1] = Fila[i];
  }
  //preenche a ultima posicao com o novo numero
  Fila[tamanhoDaFila-1] = NovoNumero;
}

