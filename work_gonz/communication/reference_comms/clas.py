from Test import *
import time
variable = "c"
mes9 = []
mes10 = []
def messageHandler(client, userdata, message):
    print(f"Received mesage on topic {message.topic}: {message.payload.decode('utf-8')}")
    try:
        f = open("objects.txt", "a")  # append, NOT write
        f.write(message.payload.decode("utf-8") + "\n")
        f.close()

        if message.topic == "/pynqbridge/9/send":
            mes9.append(message.payload[:6].decode("utf-8"))
        elif message.topic == "/pynqbridge/10/send":
            mes10.append(message.payload[:6].decode("utf-8"))
    except UnicodeDecodeError:
        print("Payload could not be decoded!")

broker9 = Broker("mqtt.ics.ele.tue.nl", "robot_9_1", "yawoitCi", ["/pynqbridge/9/send"], messageHandler)
broker10 = Broker("mqtt.ics.ele.tue.nl", "robot_10_1", "CoDryrit9", ["/pynqbridge/10/send"], messageHandler)

while variable != "q":
    variable = input("variable: ")
    if len(mes9) != 0:
        msg_info9 = broker10.client.publish("/pynqbridge/10/recv", mes9.pop(0))
        print(msg_info9.mid)
        msg_info9.wait_for_publish()
        print(msg_info9.is_published())
    if len(mes10) != 0:
        msg_info10 = broker9.client.publish("/pynqbridge/9/recv", mes10.pop(0))
        print(msg_info10.mid)
        msg_info10.wait_for_publish()

try:
    if msg_info9.is_published() == False:
        msg_info9.wait_for_publish()
except:
    print("no message on channel 10 to be published")
try:
    if msg_info10.is_published() == False:
        msg_info10.wait_for_publish()
except:
    print("no message on channel 9 to be published")

open("objects.txt", "w").close()

time.sleep(1)
print(mes9)
print(mes10)

print("Disconnecting")
broker9.client.loop_stop()
broker10.client.loop_stop()
broker9.client.disconnect()
broker10.client.disconnect()