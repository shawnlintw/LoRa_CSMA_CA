#include <util/atomic.h>

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

/*
      Modify LoRaCADDetectionwithInterrupt.ino for CSMA/CA demo.
      2021-03-15 17:00 Shawn Lin<ediwin2619@gmail.com>
*/

/*
   This example shows how to use interrupts to catch various LoRa events:
    - RxDone
    - TxDone
    - CADDone
    - RxTimeout
    - FHSSChangeChannel
    - CADDetected
    - ValidHeader
    - CRCError
    - PLLLock
    - ModeReady
    - ClockOutput

   This specific example goes through the process of turning on CAD detection and
   when a successful CAD was observed, put the LoRa chip to receive mode.
*/

/*
   It's possible to handle LoRa CAD (channel activity detection, page 43) the following way via interrupts:

   1. Write your interrupt service routine (ISR) which is executed when an IRQ arrives to the pin which is connected to DIO[0-5].
   2. Configure ISR in Arduino for LoRa DIO[0-5] pin: LoRa.setInterruptMode(0, LORA_IRQ_MODE_CADDONE)
   3. Start CAD: LoRa.cad()
   4. Check if valid CAD detected after an IRQ arrived : LoRa.readInterrupts() & LORA_IRQ_FLAG_CAD_DETECTED
   5. Clear IRQs to be able to restart CAD process later: LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);

   You may use all five DIOs concurrently connected to different Arduino pins (though those pins should intercept IRQs).

   Wiring of the current example: LoRa DIO0 - Arduino D3
*/

/* Copyright © 2018 Szőts Ákos <szotsaki@gmail.com> */

#define DEBUG

#define CHANNEL_FREE 0


// Device information
String dev_id = "1";
String ACK_str = "1ACK";

// testing resend period time
int period_time = 6;

// LED shows Channel activitly...
int LED_pin_low_act = 18 ; // A4 on Nano board.

// Message id
int msg_id = 0;

//TX data string
String string;

// CSMA/CA maximum retry times
int MAX_retry_times = 3;
// Carrier Sense timeout
int CS_timeout = 3;
// CSMA/CA backoff time
int backoff_time_MIN = 2;
int backoff_time_MAX = 4;
//CSMA/CA random time
long randtime;
// Data Send Retry
int retry_times = 0;

// Recv timeout in sec.
int timeout_sec = 20;

// Interrupt Service Routine (ISR)
// Must be as short as physically possible - assigning a variable should make it

static volatile bool interruptHappened = false;
static void isr_pinD2() {
  interruptHappened = true;
}

void setup() {

  pinMode(LED_pin_low_act, OUTPUT);
  digitalWrite(LED_pin_low_act, LOW);
  randomSeed(analogRead(5));
  Serial.begin(9600);
  while (!Serial)
    ;

  if (!LoRa.begin(434.5E6)) {
    Serial.println("Starting LoRa failed");
    while (1)
      ;
  }
  LoRa.setPreambleLength(8);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);


  // Enable Low Data Rate Optim
  LoRa.writeRegister(0x26, 0x0C);

  // For other constants, like FHSS change channel, CRC error, or RX timeout, see the LoRa.h header file.
  // Choose from LORA_IRQ_DIOx_ variants and use this "x" number in place of the first parameter.
  // Not all DIOx and interrupt type mixes are possible.
  LoRa.setInterruptMode(0 /* DIO0 */, LORA_IRQ_DIO0_CADDONE);

  // Launch our ISR function when LoRa interrupt happens
  attachInterrupt(digitalPinToInterrupt(2), isr_pinD2, RISING);

  // Start LoRa CAD process
  //LoRa.cad();

  Serial.println("Starting LoRa succeeded");
}

void loop() {

  msg_id += 1;
  string = String(msg_id);
  // Send data using CSMACA.
  Serial.println("Measure Start...");
  dev_class_A_upload_CSMACA(&string);
  //int randtime;
  //randtime = random(5, 10);
  //period_time = randtime;
  //delay((period_time * 1000));

}

int start_time_ms = 0;
int last_time_ms = 0;
int cad_detected_times =0;

void dev_class_A_upload_CSMACA(String *string) {

  StaticJsonBuffer<200> jsonBuffer;
  //boot_time = millis()/1000;
  JsonObject& root = jsonBuffer.createObject();
  bool ack_error = 0;
  bool channel_activity = 0;

  retry_times = 0;
  Serial.println("Measure transmit time...");
  // measure cad time;
  CAD_Measure();
  Serial.print("Transmit time is :");
  Serial.println((last_time_ms - start_time_ms));
  Serial.print("CAD detected times is :");
  Serial.println(cad_detected_times);
  //CSMA_CA_LoRa();
  start_time_ms = 0;
  last_time_ms = 0;
  cad_detected_times =0;
  
}

bool CAD_Measure() {

  int start_time = millis() / 1000;
  int current_time = millis() / 1000;

  int measure_timeout = 3;


  bool first_detected = 0;
  bool start =0;
  LoRa.cad();

  while ((current_time - start_time) < measure_timeout ) {

    if (interruptHappened) {

      interruptHappened = false;
      const uint8_t loraInterrupts = LoRa.readInterrupts();

      if (loraInterrupts & LORA_IRQ_FLAG_CAD_DETECTED) {
        if(start==0)
        {
          start_time = millis() / 1000;
         }
        start =1;
        cad_detected_times ++;
        if (first_detected == 0) {
          first_detected =1;
          start_time_ms = millis();
        }
        last_time_ms = millis();
      }

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

        if (!interruptHappened) {
          LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);
        }
      } // end ATOMIC_BLOCK segments
      LoRa.cad();
    }
    if(start==1)
      current_time = millis() / 1000;
  }



}



bool CSMA_CA_LoRa() {


  //Carrier Sense
  int start_time = millis() / 1000;
  int current_time = millis() / 1000;

  int start_time_ms = millis();
  int current_time_ms = millis();
  int measure_cad_time_ms = 0;
  LoRa.cad();
  while ( (current_time - start_time < CS_timeout) && retry_times < MAX_retry_times ) {
    bool channel_activity = 0;

    if (interruptHappened) {
      interruptHappened = false;
      const uint8_t  loraInterrupts = LoRa.readInterrupts();
      if (loraInterrupts & LORA_IRQ_FLAG_CAD_DETECTED) {

        //Channel Activity detected...
        channel_activity = 1;
#ifdef DEBUG
        Serial.println("channel is busy");
#endif

      } // end LoRa CAD detected segments.

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

        if (!interruptHappened) {
          LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);
          measure_cad_time_ms = (millis()) - start_time_ms;
          Serial.println(measure_cad_time_ms);
          start_time_ms = millis();
        }
      } // end ATOMIC_BLOCK segments
      LoRa.cad();
    } // end interruptHappened segments

    // delay a while and retry
    //    if (channel_activity)
    //    {
    //      randtime = random(backoff_time_MIN * 1000, backoff_time_MAX * 1000);
    //#ifdef DEBUG
    //      Serial.print("  Channel activity... backoff a while. " );
    //      Serial.println(randtime);
    //#endif
    //      delay((randtime));
    //      retry_times++;
    //      start_time = millis() / 1000; // re-clock carrier sense time.
    //    }
    current_time = (millis() / 1000);
  } // end carrier sense segment...


  //  if (retry_times >= MAX_retry_times) {
  //
  //    // channel is busy...
  //    // data dropout...
  //#ifdef DEBUG
  //    Serial.println("Channel is busy. Data dropout.");
  //#endif
  //    return 1;
  //  }
  //#ifdef DEBUG
  //  Serial.println("Channel is free. ready to send data");
  //#endif

  return 0;
}

//void send_data(JsonObject& root, String *string) {
//
//  String output, json_string;
//  root["Dev id"] = dev_id;
//  root["msg id"] = *string;
//  root["retry "] = retry_times;
//
//  root.printTo(json_string);
//  LoRa.beginPacket();
//  LoRa.print("01050");
//  root.printTo(LoRa);
//  LoRa.print("43524C46");
//  LoRa.endPacket();
//
//}
//
//bool Recv_mode(int timeout_seconds) {
//
//  int packetSize = 0;
//  int recv_time = 0;
//  int start_time = 0;
//  int ack_error = 0;
//
//  start_time = millis() / 1000;
//
//  while (recv_time < timeout_seconds) {
//
//    char recv_string[50];
//    packetSize = LoRa.parsePacket();
//    recv_time = (millis() / 1000) - start_time;
//
//    if (packetSize)
//    {
//      int index = 0;
//      while (LoRa.available()) {
//        recv_string[index] = LoRa.read();
//
//        if (recv_string[index] != ACK_str[index]) {
//          ack_error = 1;
//        }
//        index++;
//      }
//      if (ack_error == 0) {
//        return 0;
//      }
//      else {
//        Serial.println("ACK failed.");
//        return 1;
//      }
//
//    }
//  }
//  if (ack_error == 0)
//    Serial.println("ACK timeout");
//  return 1;
//}

/*
  void loop() {

  // Prepare TX Data
  //

  if (interruptHappened) {
    interruptHappened = false;

    const uint8_t loraInterrupts = LoRa.readInterrupts();
    if (loraInterrupts & LORA_IRQ_FLAG_CAD_DETECTED) { // Here use LORA_IRQ_FLAG_* variants from LoRa.h


  //          LoRa.parsePacket(); // Put into RXSINGLE mode
  //          // ... Process packet if there's one
  //          while(1){
  //          int packetSize = LoRa.parsePacket();
  //          if (packetSize){
  //              while(LoRa.available())
  //              {
  //                char rx = LoRa.read();
  //                Serial.print(rx,HEX);
  //              }
  //              Serial.println("");
  //              break;
  //           }
  //          }
  //          Serial.println("RX");
      digitalWrite(LED_pin_low_act, LOW);
      //delay(1000);
    }

    // Do not let the ISR function and the loop() write and read the interruptHappened variable at the same time
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      // It's possible that the ISR function had set interruptHappened to "true" while we were receiving packets.
      // Check again to be sure that we clear interrupt flags only when we received no further IRQs.
      if (!interruptHappened) {
        LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);

      }
    }

    //Serial.println(loraInterrupts);

    //Restart CAD process
    LoRa.cad();
  }
  digitalWrite(LED_pin_low_act, HIGH);
  }
*/
