#include "csma_lora.h"


long backoff_time_seq[4]= {0,0,0,0};  // 2^n * (SIFS)
long CSMA_TIMEOUT=300; //60;   // unit : s

bool TIMEOUT = 0;

void init_backoff_time_seq(){
  long sifs = SIFS;
  backoff_time_seq[0]=long(2*sifs);
  backoff_time_seq[1]=long(4*sifs);
  backoff_time_seq[2]=long(8*sifs);
  backoff_time_seq[3]=long(16*sifs);
}

void LoRa_setup() {
    init_backoff_time_seq();
  LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);

  //Low Data Rate Optim
  if (LORA_LOW_DR_OPT) {
    LoRa.writeRegister(0x26, 0x0C);
  }

}

void CAD_irq_setup(irs_func irs_f) {
  LoRa.setInterruptMode(0, LORA_IRQ_DIO0_CADDONE);
  attachInterrupt(digitalPinToInterrupt(2), irs_f, RISING);
}



bool Carrier_Sense(long timeout) {
  
  extern volatile bool interruptHappened;
  // Carrier_sense time
  long start_time = millis();
  long current_time = millis();
  bool channel_act = CH_FREE;
  LoRa.cad();
  while ((current_time - start_time) < timeout) {  
    if (interruptHappened) {
      interruptHappened = false;
      const uint8_t loraInterrupts = LoRa.readInterrupts();

      if (loraInterrupts & LORA_IRQ_FLAG_CAD_DETECTED) {
        // Channel is activity
        channel_act = CH_BUSY;
      }
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (!interruptHappened) {
          //clear interrupt flag
          LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);
        }
      }
      if (channel_act) {
        return CH_BUSY;
      }
      current_time = millis();
      LoRa.cad(); 
    }
    
  }

  return CH_FREE;
}
