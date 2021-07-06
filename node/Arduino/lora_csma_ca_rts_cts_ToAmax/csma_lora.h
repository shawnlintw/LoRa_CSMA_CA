
#include<Arduino.h>
#include<LoRa.h>
#include <util/atomic.h>

#define LORA_FREQUENCY      434.5E6
#define LORA_PREAMBLE_LEN   8
#define LORA_SF             12
#define LORA_BW             125E3
#define LORA_CR             8
#define LORA_LOW_DR_OPT     true

#define CH_FREE 0
#define CH_BUSY 1

#define CLEAR_TO_SEND 1


#define HIDDEN_TERMINAL

// 1 CAD about 61 ms.

#define DIFS    6000//4000//2500    // unit : ms
#define SIFS    3000//2000//1000     // unit : ms


#ifdef HIDDEN_TERMINAL
  #define ToAmax  7600                // unit : ms    // 94bytes : 5.613s(), 128bytes : 7.443s(7600), 256bytes, 14.030s()  
#else
  #define ToAmax  0
#endif

extern long CSMA_TIMEOUT;   // unit : s
extern bool TIMEOUT;

typedef void (*irs_func) ();

void LoRa_setup() ;
void CAD_irq_setup( irs_func irs_f) ;
bool Carrier_Sense(long timeout);
extern long backoff_time_seq[4];
