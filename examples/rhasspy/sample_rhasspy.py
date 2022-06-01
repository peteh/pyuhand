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

import glob
import time


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
        client.subscribe('hermes/intent/uhand:viktor')	

    def start(self):
        self._mqtt_client.connect('rhasspy.local', 1883)
        self._msgThread.start()
    
    def _run(self):
        self._mqtt_client.loop_forever()
        print("Ended Skill")
    
    def _circle(self):
        elapsed = 0
        delay = 0.05
        while elapsed < 2.:
            returnMsg = {
                    "left" : 1., 
                    "left" : -1., 
                    "honk": 0
                    }
            client.publish("remote", json.dumps(returnMsg))
            time.sleep(delay)
            elapsed += delay
        
    def _beep(self):
        pass
        
    def stop(self):
        print("Skill should end")
        self._mqtt_client.disconnect()
        print("mqtt disconnected")

    def _onMessage(self, client, userdata, msg):
        data = json.loads(msg.payload.decode("utf-8"))
        sessionId = data['sessionId']
        print("TOPIC:"+ msg.topic)

        if("uhand:viktor" in msg.topic):
            text = "Hello Viktor! Have a great day and a happy birthday. Yeaaah Yeaaah Party hard! "
            function = self._circle

        if("uhand:hello" in msg.topic):
            text = "Hello Peter. I wish you a beatiful day!"
            function = self._circle

        if("uhand:insult" in msg.topic):
            text = "Hello Peter. I wish you a beatiful day!"
            function = self._wave
        if("uhand:blabla" in msg.topic):
            text = "Bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla bla"
            function = self._circle

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
