import json
import geopy.distance

def neariest_hosp(la1,lo1):
    with open("hub_coords.json", mode='r') as read_file:
        data = json.load(read_file)

    data=list(data)
    # for x in data:
    #     print(x)

    # la1=float(input("\nLatitude :"))
    # lo1=float(input("Longitude :"))

    dist_list=[]

    for c,x in enumerate(data,0):
        coords_1 = (la1, lo1)
        coords_2 = (x[1][0], x[1][1])
        d=geopy.distance.geodesic(coords_1, coords_2)
        p="{dis}".format(dis=d)
        q=p.replace(" km", "")
        x.append(float(q))

    low_dis=data[0][3]
    index=0

    for c,x in enumerate(data,0):
        if x[3]<low_dis:
            low_dis=x[3]
            index=c

    hospital = data[index][0]
    lat = data[index][1][0]
    log = data[index][1][1]
    contact = data[index][2]
    distance = low_dis
    print("\n\033[92mHospital Located\nNearest Location: {} \nContact : {} \nDistance : {} km".format(data[index][0], data[index][2], low_dis))
    return (hospital, contact, distance, lat, log)

