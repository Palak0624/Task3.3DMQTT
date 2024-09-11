#include <WiFiNINA.h>
#include <PubSubClient.h>

// Constants
const char* wifiSSID = "Palak";  
const char* wifiPassword = "11223344"; 
const char* mqttBroker = "broker.emqx.io";
const int mqttBrokerPort = 1883;
const int triggerPin = 11;
const int echoPin = 12;
const int ledPin = 2;

class WiFiManager {
public:
    void connect(const char* ssid, const char* password) {
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
    }
};

class MQTTManager {
private:
    PubSubClient client;
public:
    MQTTManager(WiFiClient& wifiClient) : client(wifiClient) {}

    void setup(const char* broker, int port) {
        client.setServer(broker, port);
    }

    void reconnect() {
        while (!client.connected()) {
            Serial.print("Connecting to MQTT...");
            if (client.connect("ArduinoClient")) {
                Serial.println("connected");
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
                delay(5000);
            }
        }
    }

    void setCallback(MQTT_CALLBACK_SIGNATURE) {
        client.setCallback(callback);
    }

    void loop() {
        client.loop();
    }

    void publish(const char* topic, const char* message) {
        client.publish(topic, message);
    }

    bool connected() {
        return client.connected();
    }
};

class DistanceSensor 
{
  private:
    int triggerPin;
    int echoPin;
    int ledPin;
  public:
  DistanceSensor(int trigPin, int echoPin, int ledPin)
        : triggerPin(trigPin), echoPin(echoPin), ledPin(ledPin) {
        pinMode(triggerPin, OUTPUT);
        pinMode(echoPin, INPUT);
        pinMode(ledPin, OUTPUT);
    }

    long measureDistance() {
        digitalWrite(triggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);

        long duration = pulseIn(echoPin, HIGH);
        return duration * 0.034 / 2;
    }

    void flashLED(int times) 
    {
        for (int i = 0; i < times; i++) 
        {
            digitalWrite(ledPin, HIGH);
            delay(500);
            digitalWrite(ledPin, LOW);
            delay(500);
        }
    }
};

// Global instances
WiFiClient wifiClient;
MQTTManager mqttManager(wifiClient);
DistanceSensor distanceSensor(triggerPin, echoPin, ledPin);

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message received on [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  distanceSensor.flashLED(3); 
}

void setup() 
{
  Serial.begin(115200);
  WiFiManager wifiManager;
  wifiManager.connect(wifiSSID, wifiPassword);
  mqttManager.setup(mqttBroker, mqttBrokerPort);
  mqttManager.setCallback(callback);
  mqttManager.reconnect();
  mqttManager.publish("SIT210/wave", "Setup complete");
}

void loop() 
{
  if (!mqttManager.connected()) 
  {
    mqttManager.reconnect();
  }
  mqttManager.loop();

  long distance = distanceSensor.measureDistance();
  if (distance < 10) 
  {
    mqttManager.publish("SIT210/wave", "Palak");
    delay(2000);
  }
}
