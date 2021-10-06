#!/usr/bin/env python3

import time
import requests
import yaml
 
  
with open("telegram_secrets.yaml", "r") as stream:
    try:
        config = yaml.safe_load(stream)
        bot_chatID = config["chat_id"]
        bot_token = config["api_token"]
        

    except yaml.YAMLError as exc:
        print(exc)


bot_message = "Hello, this is PromenAidBot."   
bot_chatID
send_text = 'https://api.telegram.org/bot' + bot_token + '/sendMessage?chat_id=' + bot_chatID + '&parse_mode=Markdown&text=' + bot_message

response = requests.get(send_text)

print(response.json())