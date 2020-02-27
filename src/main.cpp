/*
 * Window servo MQTT FW for the ESP-12E / F module
 * 
 * 
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include "secrets.h"

/* 
 * - Content of secrets.h:
 * -------------------------
 * const char* ssid = "_";             //wifi ssid.
 * const char* password = "_";         //
 * const char* mqtt_server = "_";      //Local Mqtt server
 * const char* userName = "_";
 * const char* passWord = "_";
 * 
 */

const int analogInPin = A0;                   // ESP8266 Analog Pin ADC0 = A0 used for reading servo position line

Servo myservo;  // create servo object to control a servo
const int ServoPin = D2;
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
   delay(100);
    // We start by connecting to a WiFi network
    Serial.print("Connecting to Wifi ssid: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

int getServoFeedback(const int pin)
{
  double sum = 0;
  int i=0;  
  while (i++ < 10) {
    sum += analogRead(pin);
  }
  --i; //- Getting the corect number of iterations
  
  int result = (int) (((sum / i) - 123) * 178/783 + 1 + 0.5) ; //Calcuate average. calibrate offset and scaling to degrees
  result = result < 0 ? 0 : result;
  result = result > 180 ? 180 : result;

  return result;
}


void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Command from MQTT broker is : ");
  Serial.println(topic);

  payload[length] = '\0';                           // Make payload a string by NULL terminating it since it is not there
  int targetAngle = atoi((char *)payload);          // convert the string to an int value
  int startAngle = getServoFeedback(analogInPin);   // Read the actual physical position from the servo feedback signal rather than: myservo.read();
  
  // tell what we are doing
  Serial.print("Mowing from current angle ");
  Serial.print(startAngle);
  Serial.print(" to ");
  Serial.println(targetAngle);
  Serial.print("Please wait as we are going slow... ");

  myservo.attach(ServoPin);
  int direction = (targetAngle > startAngle) ? 1 : -1;    // Seting moving directiion
  for (int i=startAngle; i!=targetAngle; i = i + direction){
    myservo.write(i); // tell servo to go to step tovards target angle.
    delay(40);
    int pos = getServoFeedback(analogInPin);

    // Test if we are lagging to much, if so goto sleep as the servo is likely overloaded 
    if(abs(pos - i) > 20){
      myservo.detach();
      Serial.print("\n..TargetAngle: "); Serial.print(i);
      Serial.print(" servo angle: "); Serial.print(pos);
      Serial.println(" The lag is to high, Going to sleep until reset.. \n");
      // ESP.restart();
      ESP.deepSleep(0); // sleep until reset
      return;
    }    
  }
  myservo.detach();
  Serial.println("..Rotation Complete.\n");

}//end callback

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect

    if (client.connect(clientId.c_str(), userName, passWord)) //if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
     //once connected to MQTT broker, subscribe command if any
      client.subscribe("WindowServo");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 6 seconds before retrying
      delay(6000);
    }
  }
} //end reconnect()

void setup() {
  myservo.write(getServoFeedback(analogInPin));
  myservo.attach(ServoPin);  // attaches the servo on pin D1 to the servo object
  Serial.begin(115200);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  myservo.detach();               // Detach to keep us silent while not changing position

  pinMode(LED_BUILTIN, OUTPUT);             // Turn off anoyng ESP12F blue LED
  digitalWrite(LED_BUILTIN, HIGH);          // BUILTIN_LED same as D4 gpio2
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

}