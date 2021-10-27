#!/usr/bin/env python
import requests
import yaml
import time
from twilio.rest import Client
from sensor_msgs.msg import NavSatFix

class Alert:
    #define a timeout variable to prevent spam
    TIMEOUT = 60
    def __init__(self):
        self.locked_time = 0

    def locked(self):
        if (time.time() > self.locked_time):
            print("LOCKED")
            return False
        else:
            return True
    
    def lock(self):
        self.locked_time = time.time() + Alert.TIMEOUT
    

class TelegramAlert(Alert):
    def __init__(self):
        with open("telegram_secrets.yaml", "r") as stream:
            try:
                config = yaml.safe_load(stream)
                telegram_config = config['telegram']
                self.chatID = telegram_config["chat_id"]
                self.token = telegram_config["api_token"]
            except yaml.YAMLError as exc:
                print(exc)
                return

    def send_alert(self, message, location=None): 
        if not self.locked(): 
            payload = "https://api.telegram.org/bot{0}/sendMessage?chat_id={1}&parse_mode=Markdown&text={2}".format(self.token, self.chatID, message)
            response = requests.get(payload)

            if location is not None:
                self.send_location(location)

            print(response.json())
            self.lock()
            time.sleep(10)

    def send_location(self, location): 
        latitude = location.latitude
        longitude = location.longitude
        payload = "https://api.telegram.org/bot{0}/sendlocation?chat_id={1}&latitude={2}&longitude={3}".format(self.token, self.chatID, latitude, longitude)
        response = requests.get(payload)
        print(response.json())

class SMSAlert(Alert):
    def __init__(self):
        with open("telegram_secrets.yaml", "r") as stream:
            try:
                config = yaml.safe_load(stream)
                sms_config = config['twilio']
                account_sid = sms_config["account_sid"]
                auth_token = sms_config["auth_token"]
                self.recipient = sms_config["recipient"]
                self.messaging_service_sid = sms_config["messaging_service_sid"]
                self.client = Client(account_sid, auth_token) 
            except yaml.YAMLError as exc:
                print(exc)
                return

    def send_alert(self, message, location=None):  
        if not self.locked():
            latitude = location[0]
            longitude = location[0]
            location_link = 'http://maps.google.com/?q={0},{1}'.format(latitude,longitude)
            message_body = "\nPromenAid Message:\n\n{0} \n\nLocation:\n{1}".format(message,location_link)

            message = self.client.messages.create(  
                                messaging_service_sid=self.messaging_service_sid, 
                                body=message_body,      
                                to=self.recipient 
                            )
            print(message.sid)
            self.lock()
            time.sleep(10)

    def callback(self, data):
        self.send_alert("EMERGERNCY MESSAGE: USER REQUESTING ASSISTANCE", data)