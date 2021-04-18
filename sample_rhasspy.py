# pyuhand - control your uhand
# Copyright (C) 2021  Pete <github@kthxbye.us>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import pyuhand
import glob
import time
uhand = pyuhand.UHand("/dev/ttyUSB0")

import paho.mqtt.client as mqtt
import threading
import time
import json



class UHandSkill(object):
    def __init__(self):
        self._msgThread = threading.Thread(target = self._run)
        self._mqtt_client = mqtt.Client()
        self._mqtt_client.on_connect = self._onConnect
        self._mqtt_client.on_message = self._onMessage
        
        
    def _onConnect(self, client, userdata, flags, rc):
        # subscribe to all messages
        client.subscribe('hermes/intent/uhand:hello')
        client.subscribe('hermes/intent/uhand:blabla')	

    def start(self):
        self._mqtt_client.connect('rhasspy.local', 1883)
        self._msgThread.start()
    
    def _run(self):
        self._mqtt_client.loop_forever()
        print("Ended Skill")
    
    def _wave(self):
        for i in range(3):
            uhand.setTargetPercent(1, 50)
            uhand.setTargetPercent(2, 100)
            uhand.setTargetPercent(3, 100)
            uhand.setTargetPercent(4, 100)
            uhand.setTargetPercent(5, 100)
            uhand.setTargetPercent(6, 50)
            uhand.execute(500)
            uhand.setTargetPercent(1, 50)
            uhand.setTargetPercent(2, 0)
            uhand.setTargetPercent(3, 0)
            uhand.setTargetPercent(4, 0)
            uhand.setTargetPercent(5, 0)
            uhand.setTargetPercent(6, 50)
            uhand.execute(500)
        uhand.setTargetPercentAll(0)
        uhand.execute(500)
    
    def _point(self):
        for i in range(5):
            uhand.setTargetPercent(1, 50)
            uhand.setTargetPercent(2, 100)
            uhand.setTargetPercent(3, 0)
            uhand.setTargetPercent(4, 0)
            uhand.setTargetPercent(5, 0)
            uhand.setTargetPercent(6, 100)
            uhand.execute(500)
            uhand.setTargetPercent(1, 50)
            uhand.setTargetPercent(2, 60)
            uhand.setTargetPercent(3, 0)
            uhand.setTargetPercent(4, 0)
            uhand.setTargetPercent(5, 0)
            uhand.setTargetPercent(6, 100)
            uhand.execute(500)
        
        uhand.setTargetPercent(1, 0)
        uhand.setTargetPercent(2, 0)
        uhand.setTargetPercent(3, 100)
        uhand.setTargetPercent(4, 0)
        uhand.setTargetPercent(5, 0)
        uhand.setTargetPercent(6, 0)

        uhand.execute(500)
        time.sleep(2)
        uhand.setTargetPercentAll(0)
        uhand.execute(500)
        
    def stop(self):
        print("Skill should end")
        self._mqtt_client.disconnect()
        print("mqtt disconnected")

    def _onMessage(self, client, userdata, msg):
        data = json.loads(msg.payload.decode("utf-8"))
        sessionId = data['sessionId']
        print("TOPIC:"+ msg.topic)

        if("uhand:hello" in msg.topic):
            text = "Hello Peter. I wish you a beatiful day!"
            function = self._wave

        if("uhand:insult" in msg.topic):
            text = "Hello Peter. I wish you a beatiful day!"
            function = self._wave
        if("uhand:blabla" in msg.topic):
            text = "Bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla"
            function = self._point

        waveThread = threading.Thread(target = function)
        waveThread.start()
    

        print("Hello for %s" % (sessionId))
        print(sessionId)
        
        returnMsg = {
            "sessionId" : sessionId, 
            "text": text
            }
        
        client.publish("hermes/dialogueManager/endSession", json.dumps(returnMsg))




skill = UHandSkill()
skill.start()
while(True):
    time.sleep(5)
skill.stop()
