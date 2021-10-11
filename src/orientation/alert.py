#!/usr/bin/env python3

import requests
import yaml
import rospy
from sensor_msgs.msg import NavSatFix


class Alert:
    def __init__(self):
        with open("telegram_secrets.yaml", "r") as stream:
            try:
                config = yaml.safe_load(stream)
                self.chatID = config["chat_id"]
                self.token = config["api_token"]
            except yaml.YAMLError as exc:
                print(exc)
                return

    def send_alert(self, message, location=None):  
        payload = "https://api.telegram.org/bot{0}/sendMessage?chat_id={1}&parse_mode=Markdown&text={2}".format(self.token, self.chatID, message)
        response = requests.get(payload)

        if location is not None:
            self.send_location(location)

        print(response.json())

    def send_location(self, location): 
        latitude = location.latitude
        longitude = location.longitude
        payload = "https://api.telegram.org/bot{0}/sendlocation?chat_id={1}&latitude={2}&longitude={3}".format(self.token, self.chatID, latitude, longitude)
        response = requests.get(payload)
        print(response.json())