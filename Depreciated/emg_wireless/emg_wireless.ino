/* ------------------------------------------------------------------------------
* FEDERAL UNIVERSITY OF UBERLANDIA
* Faculty of Electrical Engineering
* Biomedical Engineering Lab
* Uberlândia, Brazil
* ------------------------------------------------------------------------------
* Author: Italo G S Fernandes; 
* contact: italogsfernandes@gmail.com;
* URLs: www.biolab.eletrica.ufu.br
*       https://github.com/BIOLAB-UFU-BRAZIL
*       https://github.com/italogfernandes
* ------------------------------------------------------------------------------
*/
#include <nrf24le01Module.h>
//#include <DueTimer.h>
/////////////
// Defines //
/////////////
#define freq_teste 2000
unsigned long POLLING_TIMEOUT=1000000/freq_teste; // 10 milliseconds of maximum wait time. tempo medio: 385
const double sampPeriod = (1.0 / freq_teste) * 1000000;
#define LED_STATUS 13
#define UART_BAUDRATE 115200

#define CMD_READ 0xAA

//Polling and other timeouts variables
unsigned long timeout_init_time, timeout_actual_time;

//Polling and other timeouts variables
unsigned long timer_init_time, timer_actual_time;

nrf24le01Module host_nrf(2,3,4);

void setup(){
  pinMode(LED_STATUS, OUTPUT);
  Serial.begin(UART_BAUDRATE); //Communication with Sofware
  host_nrf.rf_init(host_nrf.ADDR_HOST,host_nrf.ADDR_HOST,16,RF_DATA_RATE_2Mbps,RF_TX_POWER_0dBm); //RF Communication
  Serial.print("Arduino HOST Iniciado...\n");
  //Piscadas
  digitalWrite(LED_STATUS, HIGH); delay(500);  digitalWrite(LED_STATUS, LOW); delay(500);
  digitalWrite(LED_STATUS, HIGH); delay(500);  digitalWrite(LED_STATUS, LOW); delay(500);
  //Timer3.attachInterrupt(takeReading).start(sampPeriod);
}


void loop() {
//  timer_init_time = micros() + sampPeriod;
//  while(1){
//    timer_actual_time = micros();
//    if(timer_actual_time>timer_init_time){
      takeReading();
//      timer_init_time = micros() + sampPeriod;
//    }
//  }
}

void takeReading(){
  send_rf_command(CMD_READ);
}

//////////////////////
//RF High Functions //
//////////////////////

void wait_rf_response(){
  timeout_init_time = micros() + POLLING_TIMEOUT;
  while(1){
    timeout_actual_time = micros();
    if(timeout_actual_time>timeout_init_time){
      Serial.println(0);
      break;
    }
    if(!digitalRead(host_nrf.RFIRQ)){
      host_nrf.RF_IRQ();
      if(host_nrf.newPayload){
//         Serial.print("Sensor response with: ");
//         Serial.print(POLLING_TIMEOUT - timeout_init_time + timeout_actual_time);
//         Serial.print(" microseconds.\n");
        rf_communication_handler();
      }
      break;
    }
  }
}

void rf_communication_handler(){
  //Redirects the packet
  Serial.println(host_nrf.rx_buf[0] << 8 | host_nrf.rx_buf[1]);
  host_nrf.sta = 0;
  host_nrf.newPayload = 0;
}

/////////////////////
//RF Low Functions //
/////////////////////

void send_rf_data(uint8_t *data2send, uint8_t data_len){
  uint8_t i;
  for(i = 0; i < data_len; i++){
    host_nrf.tx_buf[i] = data2send[i];
  }
  host_nrf.TX_Mode_NOACK(data_len);
}

void send_rf_command(uint8_t cmd2send){
  host_nrf.tx_buf[0] = cmd2send;
  host_nrf.TX_Mode_NOACK(1);
  wait_rf_response();
}


