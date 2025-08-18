#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "TUO_SSID";        // nome WiFi
const char* password = "TUA_PASSWORD";

const char* serverIP = "192.168.1.50"; // IP di Pico2
const int serverPort = 4210;

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Connessione al WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnesso al WiFi!");
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.length() > 0) {
      Serial.print("Invio via WiFi: ");
      Serial.println(msg);
      udp.beginPacket(serverIP, serverPort);
      udp.print(msg);
      udp.endPacket();
    }
  }
}
