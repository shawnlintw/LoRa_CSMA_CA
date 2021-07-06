#!/usr/bin/env python3

"""  """

# Copyright 2018 Rui Silva.
#
# This file is part of rpsreal/pySX127x, fork of mayeranalytics/pySX127x.
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

# coding=UTF-8 

import time
import re
import ast
from datetime import datetime
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
import RPi.GPIO as GPIO
import json

BOARD.setup()
BOARD.reset()
parser = LoRaArgumentParser("Lora tester")


class mylora(LoRa):
    def __init__(self, verbose=False):
        super(mylora, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.var=0

    def on_rx_done(self):
        BOARD.led_on()
        #print("\nRxDone")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        data = ''.join([chr(c) for c in payload])
        head = data.split("{",1)[0]
        tail = data.split("}",1)[1]
        data = '{'+re.split('{|}',data)[1]+'}'
        if data.find("RTS")==2:
            devid = ord(data[1])
            print(chr(devid),"RTS")
            now=datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print ("[",current_time,"] ","Receive: ")
            self.write_payload([devid,67,84,83]) # Send CTS
        else:
            print("Data")
            dict_data = ast.literal_eval(data)
            devid= ord(dict_data['Dev id'])
            now=datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print ("[",current_time,"] ","Receive: ")
            #print(bytes(payload).decode("utf-8",'ignore')) # Receive DATA
            payload_str = ''.join(map(chr, payload))
            print(payload_str)
            #json_obj=json.load(payload)
            #print(json_obj)
            BOARD.led_off()
            time.sleep(1) # Wait for the client be ready
            print ("Send: ACK")
            #self.write_payload([255, 255, 0, 0, 65, 67, 75, 0]) # Send ACK
            self.write_payload([devid,65,67,75]) #Send ACK
        #time.sleep(0.1)
        self.set_mode(MODE.TX)
        self.var=1

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
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            while(self.var ==0):
                pass;
            #rssi_value = self.get_rssi_value()
            time.sleep(2)   # Add this delay for make sure the transmit already completely.
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT)
            #print(rssi_value)
            #time.sleep(10)
            self.var=0

'''
    def start(self):          
        while True:
            while (self.var==0):
                print ("Send: INF")
                self.write_payload([255, 255, 0, 0, 73, 78, 70, 0]) # Send INF
                self.set_mode(MODE.TX)
                time.sleep(3) # there must be a better solution but sleep() works
                self.reset_ptr_rx()
                self.set_mode(MODE.RXCONT) # Receiver mode
            
                start_time = time.time()
                while (time.time() - start_time < 10): # wait until receive data or 10s
                    pass;
            
            #self.var=0
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT) # Receiver mode
            time.sleep(10)
            self.var=0;
'''
lora = mylora(verbose=True)
args = parser.parse_args(lora) # configs in LoRaArgumentParser.py

#     Slow+long range  Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. 13 dBm
lora.set_pa_config(pa_select=1, max_power=20, output_power=15)
#lora.set_bw(BW.BW125)
lora.set_coding_rate(CODING_RATE.CR4_8)
lora.set_spreading_factor(12)
lora.set_rx_crc(True)
#lora.set_lna_gain(GAIN.G1)
#lora.set_implicit_header_mode(False)
lora.set_low_data_rate_optim(True)
#  Medium Range  Defaults after init are 434.0MHz, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on 13 dBm
#lora.set_pa_config(pa_select=1)


assert(lora.get_agc_auto_on() == 1)
print(lora)
try:
    #GPIO.cleanup()
    print("START")
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("Exit")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("Exit")
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
    GPIO.cleanup()
