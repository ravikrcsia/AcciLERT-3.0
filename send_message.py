import json
from twilio.rest import Client

account_sid = # Twilio Account Sid
auth_token = # Twilio Auth Token 

def message(driver, hospital, contact, distance, lat, log, lat1, long1):
    
    contact_list = []

    with open("driver_info.json", "r") as read_file:
        data = json.load(read_file)
        name = data[driver][0]["name"]
        age = data[driver][0]["age"]
        health = data[driver][0]["health"]
        relative_no1 = data[driver][0]["relatives"][0]["1"]
        relative_no2 = data[driver][0]["relatives"][0]["2"]
        friend_no1 = data[driver][0]["friend"][0]["1"]
        friend_no2 = data[driver][0]["friend"][0]["2"]

        contact_list = [relative_no1, relative_no2, friend_no2]
        # print(contact_list)
        print('\033[92mVictims Data is been Fetched')
        print('\033[96mInitiated Emergency Message System\033[0m')

        for number in contact_list:
            
            # print(type(number))
            client = Client(account_sid, auth_token)
            message = client.messages.create(
                from_= "+19086450239",
                body = "There is been an accident at http://maps.google.com/?q={},{}  \ndriver: {}, \nneariest hospital: http://maps.google.com/?q={},{}".format(lat1, long1, driver, lat, log),
                to= number,
            )
            print(message.sid)

    ####### Hospital message #########

    client = Client(account_sid, auth_token)
    message = client.messages.create(
        from_= "+19086450239",
        body = "There is been and accident at https://www.google.com/maps/dir/{},{}/{},{} \n patient details : name: {}\
            \nage: {} \nmedical details: {}, \ncontact_details: {}".format(lat, log, lat1, long1, name, age, health, contact_list),
        to= "+919768437412",
    )
    print(message.sid)