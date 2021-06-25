#include<util/atomic.h>

#include<SPI.h>
#include "csma_lora.h"
#include <ArduinoJson.h>

#define DEBUG
#define RTS_CTS
#define TX_POWER  2

#define EXP1  random(15, 23)        // EXP1 : 128 bytes
#define CSMACA_EXP2  random(7,15)   // EXP2 : 128 bytes
#define CSMACA_EXP3  random(25,35)  // EXP3 : 128 bytes
#define CSMACA_EXP4  random(35,45)  // EXP4 : 128 bytes

#define recv_ack    1
#define recv_cts    0

// Maximum test_times
#define MAX_TEST  250

// Maximum payload
#define MAX_PAYLOAD 128   // It have to larger than 94


//Device information
String dev_id = "4";


String ACK_str = "ACK";
String CTS_ACK = "CTS";

// message id
int msg_id = 0;

// data drop times
int drop_times = 0;

// resend times
int retry_times = 0;

//Measure send delay time
long exceptTX_time = 0;
long realTX_time = 0;

//except send flag bool
bool except_sent_flag = 0;

// MAXIMUM RESEND TIMES
#define MAX_RETRY_TIMES   5

// index of backoff sequence...
int backoff_index = 0;

volatile bool interruptHappened = false;

static void isr_pinD2() {
  interruptHappened = true;
}

void Test_data_gen(JsonObject& data, String *string);

void setup() {
  // put your setup code here, to run once:
  randomSeed(analogRead(5));
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed");
    while (1);
  }
  LoRa_setup() ;
  LoRa.setTxPower(TX_POWER);
  CAD_irq_setup(isr_pinD2);
  ACK_str = dev_id + ACK_str;
  CTS_ACK = dev_id + CTS_ACK;
  LoRa.sleep();
  delay(2000);   // Waiting system initialize finished.
  Serial.println("LoRa device init OK...");
}

void loop() {

  // put your main code here, to run repeatedly:
  StaticJsonBuffer<270> jsonBuffer;
  JsonObject& output = jsonBuffer.createObject();

  msg_id += 1;
  if (msg_id > MAX_TEST)
  {
    LoRa.end();
    while (1);
  }
  String string;
  string = String(msg_id);
  except_sent_flag = 0;
  
  

  
  // test function here....
  lora_csma_Test(output, &string);
  LoRa.sleep();
  int randtime;
  //randtime = random(27,35);
  randtime = CSMACA_EXP4;
  for (int i = 0; i < randtime; i++)
  {
    delay(1000);
  }
  //randtime = random(7, 15);
  //delay((randtime * 1000));

}


void lora_csma_Test(JsonObject& output, String *string) {

  bool CH_CTS = false;
  bool ack_error = false;
  bool retry = false;
  retry_times = 0;


  do {
    //Test_data_gen(output, string);
    CH_CTS = csma_test();
    // If channel is free, send data.
    if (CH_CTS == true) {
#ifdef DEBUG
      Serial.println("Debug: CH is free, ready send data.");
      realTX_time = millis() / 1000;
#endif

#ifdef RTS_CTS
      Test_data_gen(output, string);
      Send_RTS();
      ack_error = Recv_mode(10, recv_cts);
      if (ack_error == 1) {
        retry = true;
        retry_times++;
#ifdef DEBUG
        Serial.println("Debug: CTS has not be received.");
#endif
        continue;
      }
#ifdef DEBUG
      Serial.println("Debug: CTS has received.");
#endif
      delay(1000); // Waiting for gateway change to recv mode.
      Send_data(output);
#else
      Test_data_gen(output, string);
      Send_data(output);
#endif
    }
    else {
      // CH_CTS == false
      // Channel is busy and Retry again..
      retry = true;
      retry_times++;
#ifdef DEBUG
      Serial.println("Debug: Collision, Retry.");
#endif
      continue;
    }
#ifdef DEBUG
    Serial.println("Debug: Waiting ACK response.");
#endif
    ack_error = Recv_mode(10, recv_ack  );
    if (ack_error == 1) {
      retry = true;
      retry_times ++;
    }
    else {
      retry = false;
    }

  } while (retry && retry_times < MAX_RETRY_TIMES);

  if (retry_times < MAX_RETRY_TIMES) {
    Serial.println("Data sent.");
  }
  else {
    drop_times++;
    Serial.println("Data dropout.");
  }

}

bool csma_test() {

  bool channel_act = CH_BUSY;
  bool data_send = false;
  long backoff_time = 0;
  long start_time_ms = 0;
  long current_time_ms = 0;
  backoff_index = 0;
  // Is channel free in DIFS ?

  long randtime;
  randtime = random(SIFS, DIFS);
  //Serial.println(randtime);
  //channel_act = Carrier_Sense(DIFS + randtime * 4);
  channel_act = Carrier_Sense(DIFS);
#ifdef DEBUG
  //set timer
  if (except_sent_flag == 0) {
    exceptTX_time = millis() / 1000;
    except_sent_flag = 1;
  }
  
#endif

#ifdef HIDDEN_TERMINAL
  channel_act= Carrier_Sense(ToAmax);
#endif

  if (channel_act == CH_FREE) {
#ifdef DEBUG
    Serial.println("Debug : ch free");
#endif
    return CLEAR_TO_SEND;
  }
  else {
    //Channel is busy and do something to avoid collision ..
#ifdef DEBUG
    Serial.println("Debug : ch busy");
#endif
    while (data_send == false && backoff_index < 4) {
      // Waiting untill CH is free.  the duration of CH free must larger than SIFS...
      start_time_ms = millis();
      do {
        channel_act = Carrier_Sense(SIFS);
        current_time_ms = millis();
      } while (channel_act == CH_BUSY && ((current_time_ms - start_time_ms) < (CSMA_TIMEOUT * 1000)));  // do utill channel is free or timeout ...
      if (current_time_ms - start_time_ms >= (CSMA_TIMEOUT * 1000)) {
#ifdef DEBUG
        Serial.println("CSMA TIMEOUT.");
#endif
        continue;
      }
      // if channel is free then start to countdown backoff time
      if (channel_act == CH_FREE) {
#ifdef DEBUG
        Serial.println("Debug: backoff.");
#endif
        backoff_time = backoff_time_gen(backoff_index);
        channel_act = backoff_process(backoff_time);
        //As channel is free, it still waiting for DIFS time.
        channel_act = Carrier_Sense(DIFS);
        if (channel_act == CH_FREE) {
          data_send = true;
        }
        else {
#ifdef DEBUG
          Serial.println("Debug : Channel is so busy that backoff timeout.");
#endif
          backoff_index ++;
        }
      }
      //      else {
      //#ifdef DEBUG
      //        Serial.println("Debug : Channel is not free in DIFS again.");
      //#endif
      //        continue;
      //        // It means Channel becomes busy...  Continue Waiting...
      //      }
    }
    if (data_send) {
      return CLEAR_TO_SEND;
    }
    else {
      return 0;   // CANNOT_SEND // Reach to Maximum backoff_index
    }
  }

}

long backoff_time_gen(int backoff_index) {

  float randfactor = random(50, 200) / 100.0;
  return (long(backoff_time_seq[backoff_index] * (randfactor)));

}

bool backoff_process(long backoff_time) {

  bool channel_act = false;
  bool channel_BUSY = false;
  long backoff_time_count = 0;
  long detect_start_time = 0;
  long detect_time = 0;
  long start_time = 0;
  long current_time = 0;


  detect_start_time = millis();
  detect_time = millis();
  while ( backoff_time_count < (backoff_time + ToAmax) && ((detect_time - detect_start_time) < CSMA_TIMEOUT * 1000 )) {

    start_time = millis();
    channel_act = Carrier_Sense(SIFS);

    if (channel_act == CH_FREE) {
      current_time = millis();
      backoff_time_count +=  (current_time - start_time) ;
      channel_BUSY=false;
    }
#ifdef HIDDEN_TERMINAL
    else {
      if (channel_BUSY == false) {
        Serial.println("backoff reset ToAmax");
        if (backoff_time_count > ToAmax) {
          backoff_time_count = backoff_time_count - ToAmax;
        }
        else {
          backoff_time_count = 0;
        }
      }
      channel_BUSY=true;
    }
#endif

    detect_time = millis();
  }
  if (((detect_time/1000) - (detect_start_time/1000)) < CSMA_TIMEOUT )
    return 0;
  else
    return 1;
}


void Send_RTS()
{
  LoRa.beginPacket();
  LoRa.print("01050");    // 5 bytes
  LoRa.print("{");        // 1 byte
  LoRa.print(dev_id);     // 1 byte
  LoRa.print("RTS");      // 3 bytes
  LoRa.print("}");        // 1 byte
  LoRa.print("43524C46"); // 8 bytes
  LoRa.endPacket();
  // 19 bytes
  Serial.print("RTS Sent : ");
  Serial.print("01050{");
  Serial.print(dev_id);
  Serial.println("RTS}4324C46");
}


void Test_data_gen(JsonObject& data, String *string) {

  String json_string;

  String test_string, payload_string;
  int payload_length = 0;
  char payload_char;

  long tx_delay_time = realTX_time - exceptTX_time;
  data["Dev id"] = dev_id;
  data["msg id"] = *string;
  data["backoff_index"] = backoff_index ;
  data["delay"] = tx_delay_time;
  data["retry "] = retry_times;
  data["drop times"] = drop_times;

  data.printTo(test_string);
  //Serial.println(test_string.length());
  payload_length = MAX_PAYLOAD - test_string.length() - 13 - 13;
  if (payload_length > 0)
  {
    for (int i = 0; i < payload_length; i++)
    {
      payload_char = 65 + (i % 26);
      payload_string.concat(payload_char);
    }
    data["payload"] = payload_string;
  }
  //data["payload"] ="ABCDEFGHIJKLMNOPQRSTU";

  data.printTo(json_string);

  //Serial.println(json_string.length());

}

void Send_data(JsonObject& output) {

  LoRa.beginPacket();
  LoRa.print("01050");    // 5 bytes
  output.printTo(LoRa);
  LoRa.print("43524C46"); // 8 bytes
  LoRa.endPacket();
  
  String json_string;
  output.printTo(json_string);
  Serial.print("Data Sent : ");
  Serial.print("01050");
  Serial.print(json_string);
  Serial.println("43524C46");
}

bool Recv_mode(long timeout_seconds, bool recv_type) {

  int packetSize = 0;
  long recv_time = 0;
  long start_time = 0;
  int ack_error = 0;

  start_time = millis() / 1000;

  while (recv_time < timeout_seconds) {

    char recv_string[50];
    packetSize = LoRa.parsePacket();
    recv_time = (millis() / 1000) - start_time;

    if (packetSize)
    {
      int index = 0;
      while (LoRa.available()) {
        recv_string[index] = LoRa.read();

        if (recv_type == recv_ack) {
          if (recv_string[index] != ACK_str[index]) {
            ack_error = 1;
          }
        }
        else if (recv_type == recv_cts) {
          if (recv_string[index] != CTS_ACK[index]) {
            ack_error = 1;
          }
        }
        index++;
      }
      if (ack_error == 0) {
        Serial.print("Receive Response :");
        print_recv_string(recv_string, &index);
        Serial.println("");
        return 0;
      }
      else {
        Serial.print("ACK failed.: ");
        print_recv_string(recv_string, &index);
        return 1;
      }

    }
  }
  if (ack_error == 0)
    Serial.println("ACK timeout");
  return 1;

}

void print_recv_string(char *recv_string, int *index)
{
  for (int i = 0; i < *index; i++)
  {
    Serial.print(*(recv_string + i));
  }
}
// for test function :
// First carrier sense and CSMA/CA op
// Send data
// Receive ack


// CSMA/CA processing :

// if channel is free in DIFS
//    then send data
// else
//    check the ch is free or waiting utill ch is free....
//    Once channel is free... then count down the backoff time
//    During count down t... if ch is busy...   stop cont down backoff time and frozen...
//    Continue cont down when ch is free again...
//
