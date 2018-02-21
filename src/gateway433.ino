#include <Homie.h>
#include <RCSwitch.h>
#include <ArduinoJson.h>
#include <MD_CirQueue.h>

// BEGIN Configuration
int receivePIN = 5;
int transmitPIN = 4;
// int ledPIN = 2;
// int ledMODE = LOW;
const char* nodeName = "433";
// END Configuration

MD_CirQueue Q(200, sizeof(long) * 5);

// homie nodes & settings
HomieNode rfSwitchNode(nodeName, "switch");

// RF Switch
RCSwitch mySwitch = RCSwitch();

void setupHandler() {
  Q.begin();
}

void loopHandler() {
  if (mySwitch.available()) {
    // long start = millis();
    // rfSwitchNode.setProperty("debug").send("START RECV " + String(start));
    String data = String(mySwitch.getReceivedValue());
    String bitLength = String(mySwitch.getReceivedBitlength());
    String pulseLength = String(mySwitch.getReceivedDelay());
    String protocol = String(mySwitch.getReceivedProtocol());

    mySwitch.resetAvailable();
    Serial << "Receiving 433Mhz > MQTT signal:";
    Serial << " Code: " << data;
    Serial << " Bits: " << bitLength;
    Serial << " PulseLength: " << pulseLength;
    Serial << " Protocol: " << protocol << endl;
    rfSwitchNode.setProperty("recv").send(data);
    ESP.wdtFeed();

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["code"] = data;
    root["bits"] = bitLength;
    root["pulselength"] = pulseLength;
    root["protocol"] = protocol;

    String jsonData;
    root.printTo(jsonData);

    rfSwitchNode.setProperty("recvjson").send(jsonData);
    // long duration = millis() - start;
    // rfSwitchNode.setProperty("debug").send("END RECV " + String(start) + " TIME " + String(duration));
    ESP.wdtFeed();
  }
  if (!Q.isEmpty()) {
    runOneQueue();
    ESP.wdtFeed();
  }
}

bool rfSwitchSendJsonHandler(const HomieRange &range, const String &value) {
  // long start = millis();
  // rfSwitchNode.setProperty("debug").send("START SEND " + String(start));
  StaticJsonBuffer<200> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(value);
  long data = root["code"];
  int bitLength = root["bits"];
  int pulseLength = root["pulselength"];
  int protocol = root["protocol"];

  data = data ?: 1;
  root["code"] = data;

  bitLength = bitLength ?: 24;
  root["bits"] = bitLength;

  pulseLength = pulseLength ?: 350;
  root["pulselength"] = pulseLength;

  protocol = protocol ?: 1;
  root["protocol"] = protocol;

  String jsonData;
  root.printTo(jsonData);

  Serial << "Receiving MQTT JSON > 433Mhz signal:";
  Serial << " Code: " << data;
  Serial << " Bits: " << bitLength;
  Serial << " PulseLength: " << pulseLength;
  Serial << " Protocol: " << protocol << endl;

  queueRFCode(data,bitLength,pulseLength,protocol);

  rfSwitchNode.setProperty("sendjson").send(jsonData);
  // long duration = millis() - start;
  // rfSwitchNode.setProperty("debug").send("END SEND " + String(start) + " TIME " + String(duration));
  return true;
}

bool rfSwitchSendHandler(const HomieRange &range, const String &value) {
  long int data = 0;
  int pulseLength = 350;
  if (value.indexOf(',') > 0) {
      pulseLength = atoi(value.substring(0, value.indexOf(',')).c_str());
      data = atoi(value.substring(value.indexOf(',') + 1).c_str());
  } else {
      data = atoi(value.c_str());
  }
  Serial << "Receiving MQTT > 433Mhz signal: " << pulseLength << ":" << data << endl;

  queueRFCode(data, 24, pulseLength, 1);
  rfSwitchNode.setProperty("send").send(String(data));
  return true;
}

void queueRFCode(long code, int bits, int pulselength, int protocol) {
  int package[5] = {code, bits, pulselength, protocol, 20};
  Q.push((uint8_t *)&package);
}

void runOneQueue() {
  int package[5];
  Q.pop((uint8_t *)&package);
  sendRFCode(package[0],package[1],package[2],package[3]);
  if(package[4] > 0) {
    package[4] = package[4] - 1;
    Q.push((uint8_t *)&package);
  }
}

void sendRFCode(long code, int bits, int pulselength, int protocol) {
  mySwitch.setProtocol(protocol);
  mySwitch.setPulseLength(pulselength);
  mySwitch.send(code, bits);
  rfSwitchNode.setProperty("debug").send("SENT " + String(code));
  ESP.wdtFeed();
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;

  // init RF library
  mySwitch.enableTransmit(transmitPIN);
  mySwitch.setRepeatTransmit(1); // increase transmit repeat to avoid lost of rf sendings
  mySwitch.enableReceive(receivePIN);

  // init Homie
  Homie_setFirmware("gateway433", "1.0.0");
  Homie.disableResetTrigger();
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  // Homie.setLedPin(ledPIN, ledMODE);
  Homie.disableLedFeedback();
  // Homie.setBrand('DJL');

  rfSwitchNode.advertise("send").settable(rfSwitchSendHandler);
  rfSwitchNode.advertise("recv");
  rfSwitchNode.advertise("sendjson").settable(rfSwitchSendJsonHandler);
  rfSwitchNode.advertise("recvjson");
  rfSwitchNode.advertise("debug");

  Homie.setup();
}

void loop() { Homie.loop(); }
