#!/usr/bin/env python3

""" A simple continuous receiver class. """

# Modify by Shawn Shih Hsiung Lin in 2020.

# Copyright 2015 Mayer Analytics Ltd.
#
# This file is part of pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pySX127x is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# details.
#
# You can be released from the requirements of the license by obtaining a commercial license. Such a license is
# mandatory as soon as you develop commercial activities involving pySX127x without disclosing the source code of your
# own applications, or shipping pySX127x with a closed source product.
#
# You should have received a copy of the GNU General Public License along with pySX127.  If not, see
# <http://www.gnu.org/licenses/>.

# usage:
# python p2p_recv.py -f 433 -b BW125 -s 12

import sys 
from time import sleep
import re
sys.path.insert(0, '../')
import json
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD

BOARD.setup()

parser = LoRaArgumentParser("Continous LoRa receiver.")


#import paho.mqtt.client as mqtt
import numpy as np
import json
import time
import ast

host = "iot.cht.com.tw"
#Change "DEVICE_NUMBER" to real number
topic = "/v1/device/<DEVICE_NUMBER>/rawdata"

#Change "API_KEY" to real api key
user, password = "<API_KEY>", "<API_KEY>"  
#client = mqtt.Client() 
#client.username_pw_set(user, password) 
#client.connect(host, 1883, 60) 


class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

    def on_rx_done(self):
        BOARD.led_on()
        print("\nRxDone")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        pkt_rssi = self.get_pkt_rssi_value()
        sys.stdout.write("\r%s %d\n"%("packet", pkt_rssi))
        sys.stdout.flush()
        #print("packet_rssi: ",end="")
        #print(pkt_rssi)
        data = ''.join([chr(c) for c in payload])
        #head = data.split("{",1)[0]
        #tail = data.split("}",1)[1]
        #data = '{'+re.split('{|}',data)[1]+'}'
        #print(data)
        #print(bytes(payload).decode())
        #dict_data = ast.literal_eval(data)

        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        BOARD.led_off()
        self.set_mode(MODE.RXCONT)
        #if (head == "01050" and tail == "43524C46"):
        #    print ("correct")
        #    temp =str( dict_data['Temperature'])
        #    humidity = str(dict_data['Humidity'])
        #    t = str(time.strftime("%Y-%m-%dT%H:%M:%S"))
        #    send_temp=[{"id":"DHT_temp","value":[temp],"time":t}]
        #    print(send_temp)
        #    send_humidity=[{"id":"DHT_humidity","value":[humidity],"time":t}]
        #    print(send_humidity)
            #client.publish(topic, "%s" % ( json.dumps(send_temp) ))
            #client.publish(topic, "%s" % ( json.dumps(send_humidity) )) 

    def on_tx_done(self):
        print("\nTxDone")
        print(self.get_irq_flags())

    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())

    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())

    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())

    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())

    def on_fhss_change_channel(self):
        print("\non_FhssChangeChannel")
        print(self.get_irq_flags())

    def start(self):
        i=0
        pervious_rssi=0
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            i=i+1
            self.set_mode(MODE.RXCONT)
            sleep(.5)
            rssi_value = self.get_rssi_value()
            status = self.get_modem_status()
            rssi_value= rssi_value+pervious_rssi
            if(i==1):
                rssi_value=rssi_value
            else:
                rssi_value=int(rssi_value/2)
            pervious_rssi = rssi_value
            string = "background: "
            sys.stdout.write("\r%s %d\n"%(string, rssi_value))
            sys.stdout.flush()
           # print("background: ",end="")
           # print(rssi_value)
            if(i==5):
                rssi_value=0
                pervious_rssi=0
                i=0
            #sys.stdout.flush()
            #sys.stdout.write("\r%d %d %d" % (rssi_value, status['rx_ongoing'], status['modem_clear']))


lora = LoRaRcvCont(verbose=False)
args = parser.parse_args(lora)

lora.set_mode(MODE.STDBY)
lora.set_pa_config(pa_select=1)
#lora.set_rx_crc(True)
#lora.set_coding_rate(CODING_RATE.CR4_6)
#lora.set_pa_config(max_power=0, output_power=0)
#lora.set_lna_gain(GAIN.G1)
#lora.set_implicit_header_mode(False)
lora.set_low_data_rate_optim(True)
#lora.set_low_data_rate_optim(False)
#lora.set_pa_ramp(PA_RAMP.RAMP_50_us)
#lora.set_agc_auto_on(True)

print(lora)
assert(lora.get_agc_auto_on() == 1)

#try: input("Press enter to start...")
#except: pass

try:
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("")
    lora.set_mode(MODE.SLEEP)
    print(lora)
    BOARD.teardown()
