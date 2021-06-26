import re
from flask import Flask, request, redirect
from twilio.rest import Client
from send_message import message
from To_cal_near import neariest_hosp

account_sid = ###enter your sid
auth_token = ###enter your token

client = Client(account_sid, auth_token)
from twilio.twiml.messaging_response import MessagingResponse
app = Flask(__name__)
content = None





# data="NAME: RAVIKUMAR; LAT:19.42542; LOG:20.65422; IMPACT: 40"

# new=data.split(";")
# dict={}

# for x in new:
#     raw=x.split(":")
#     dict[raw[0].strip()]=raw[1].strip()

# print(dict)

@app.route("/sms", methods=['GET','POST'])
def sms_reply():
    global content
    # print(request.form)
    content = request.form['content']
    number = request.form['sender']
    data = content.strip("B3KXP")
    print('\033[92mMessage Received ')
    print('\033[96mAccident System Initiated')
    print('\033[91mVictims Phone No: {}'.format(number))

    new=data.split(";")
    dict={}

    for x in new:
        raw=x.split(":")
        dict[raw[0].strip()]=raw[1].strip()

    # print(dict)
#     # Start our TwiML response
#     body = request.values.get('Body', None)# not useful
#     no = request.values.get('From', None)# not useful
#     print(body, no)# not useful
#     resp = MessagingResponse()# not useful
    
    send_message(dict)
#     # Add a message
#     resp.message("The Robots are coming! Head for the hills!") # not useful
    # time.sleep(6)
    return str("Done")

def send_message(dict):
    # print(content)
    driver = dict['NAME']
    lat1 = dict['LAT']
    long1 = dict['LOG']
    impact = dict['IMPACT']
    print("\033[91mAccident Took Place at http://maps.google.com/?q={},{}".format(lat1, long1))
    print('\033[93mFetching Nearest Hospital')
    data = neariest_hosp(lat1, long1)
    hospital, contact, distance, lat, log = data
    print('\033[93mFetching Victims Data From The Database')
    message(driver, hospital, contact, distance, lat, log, lat1, long1)

    print('\033[92mThe Messages have been Sent Help will arrive soon\033[0m')
    # client.messages.create(
    #     from_= "+19086450239",
    #     body = content,
    #     to= "+919987882211",
    # )

if __name__ == "__main__":
    app.run(debug=True, port=8080)