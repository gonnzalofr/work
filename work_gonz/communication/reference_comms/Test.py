import paho.mqtt.client as paho

class Broker:
    def __init__(self, host, username:str, password: str, topicSubList: list[str], messageHandler: callable) -> None:
        self.client = paho.Client(2)
        self.client.username_pw_set(username,password)
        self.client.on_message = messageHandler
        if self.client.connect(host=host) != 0:
            raise RuntimeError("Couldn't connect to the MQTT host")
        try:
            for topic in topicSubList:
                if self.client.subscribe(topic)[0] != paho.MQTT_ERR_SUCCESS:
                    print(f"Couldn't subscribe to {topic}")
                else:
                    print(f"Succes in {topic}")
            self.client.loop_start()
        except Exception as error:
            print(error)
        #finally:
        #    print("Disconnecting")
        #    self.client.disconnect()