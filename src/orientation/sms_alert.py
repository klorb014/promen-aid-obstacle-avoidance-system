#!/usr/bin/env python3

import yaml
from twilio.rest import Client
from sensor_msgs.msg import NavSatFix
import rospy
import time


class Alert:
    #define a timeout variable to prevent spam
    TIMEOUT = 60

    def __init__(self):
        self.locked_time = 0
        with open("telegram_secrets.yaml", "r") as stream:
            try:
                config = yaml.safe_load(stream)
                account_sid = config["account_sid"]
                auth_token = config["auth_token"]
                self.recipient = config["recipient"]
                self.messaging_service_sid = config["messaging_service_sid"]
                self.client = Client(account_sid, auth_token) 
            except yaml.YAMLError as exc:
                print(exc)
                return

    def locked(self):
        if (time.time() > self.locked_time):
            print("LOCKED")
            return False
        else:
            return True
    
    def lock(self):
        self.locked_time = time.time() + Alert.TIMEOUT

    def send_alert(self, alert_message, location):  

        if not self.locked():
            latitude = round(location.latitude,1)
            longitude = round(location.longitude,1) 
            location_link = 'http://maps.google.com/?q={0},{1}'.format(latitude,longitude)
            message_body = "\nPromenAid Message:\n\n{0} \n\nLocation:\n{1}".format(alert_message,location_link)
            print(message_body)

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


#Main method
if __name__ == '__main__':
    a = Alert()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("gps", NavSatFix, a.callback)
    rospy.spin()
