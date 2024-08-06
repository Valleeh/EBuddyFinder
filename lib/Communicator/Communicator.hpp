#ifndef _COMMUNICATORHPP
#define _COMMUNICATORHPP
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#define WIFI

#define GAME_CHANNEL 13

SmartSerial sCom (Serial, "Com/"," -> ");
#define FAR_LIMIT     -100
#define NEAR_LIMIT     -20
class ICommunicator {
  public:
    virtual ~ICommunicator() {}
    virtual bool init(bool be_an_acesspoint) = 0;
    virtual bool Callback() = 0;
    virtual const uint8_t GetDistance() = 0;
    virtual int scanWifi(const char *ssid) = 0;
};
class Communicator : public ICommunicator , private WiFiUDP
{
  private:
    int rssis[3];
    const char *MY_SSID = "EBuddyFinder";
    const char *password = "whatwhat"; // not important, we only scan
    //TODO: delete unneeded code
    // buffer for incoming packets
    bool acesspoint_mode_on=false;
    char packetBuffer[9];
    unsigned int localPort = 2000; // local port to listen for UDP packets
    bool CONNECTED = false;
    bool STARTED = false;
    bool RESTARTED = false;

    struct dataStruct {
        int threshold;
        bool started;
        bool restart;
        int brightness;
    };

    int size = sizeof(dataStruct); //get
    char *buffer = new char[size];


    uint8_t distance{255};
    uint8_t CalculateDistance(uint8_t dBm, uint8_t min_dBm, uint8_t max_dBm);
    void send(dataStruct data);
    bool recieve();
    bool started();
    bool restarted();
  public:
    bool Callback() override ;
    bool init(bool be_an_acesspoint) override ;
    char incomingPacket[255];
    struct dataStruct dataOut;
    struct dataStruct dataIn;
    const uint8_t GetDistance() override ;

    // A UDP instance to let us send and receive packets over UDP
    Communicator();
    const char *ssid = "iPhone SE (2nd generation)";
    const char *pass = "woistmeingeld";

    int scanWifi(const char *ssid) override ;
};
#endif


#ifndef _COMMUNICATORCPP
#define _COMMUNICATORCPP
/* #include "Communicator.hpp" */

// A UDP instance to let us send and receive packets over UDP--------------------------500*TASK_MILLISECOND
Communicator::Communicator() :
    WiFiUDP() {
}

bool Communicator::Callback()  {
    distance = CalculateDistance(scanWifi(MY_SSID), FAR_LIMIT, NEAR_LIMIT);
    return false;
}
//////////////WIFI Init
bool Communicator::init(bool be_an_acesspoint)  {
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect();
    WiFi.softAP(MY_SSID, password, GAME_CHANNEL, 0, 0);
    WiFi.disconnect();
    if (be_an_acesspoint)
    {
        acesspoint_mode_on = true;
        WiFi.softAP(ssid, pass);
        IPAddress local_IP(192, 168, 4, 22);
        IPAddress gateway(192, 168, 4, 9);
        IPAddress subnet(255, 255, 255, 0);
        //Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
        begin(localPort);
        return true;
    }
    else
    {
        WiFi.begin(ssid, pass); //Connect to access point
        delay(1000);
        int i = 0; // Wait for connection
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(1000);
            Serial.print(++i);
            Serial.print(' ');
            if (i > 10)
            {
                Serial.println("Connecting TimeOut");
                return false;
            }
        }
        //WiFi.mode(WIFI_STA);
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        //Start UDP
        Serial.println("Starting UDP");
        begin(localPort);
        Serial.print("Local port: ");
        Serial.println(localPort);
        CONNECTED = true;

        return CONNECTED;
    }
}
void Communicator::send(dataStruct data)
{
    if (acesspoint_mode_on)
    {
        //Get your own IP
        IPAddress ip = WiFi.softAPIP();
        //Last byte for broadcast
        ip[3] = 255;
        beginPacket(ip, localPort);

        // Serial.print("Broadcast IP: ");
        // Serial.println(ip);
    }
    else
    {
        beginPacket(remoteIP(), remotePort());
    }
    memset(buffer, 0x00, size);

    memcpy(buffer, &data, size);

    // Send it
    write(buffer, size);
    endPacket();
}
bool Communicator::recieve()
{
    int packetSize = parsePacket();
    if (packetSize)
    {
        // receive incoming UDP packets
        int len = read(incomingPacket, 255);
        if (len > 0)
        {
            incomingPacket[len] = 0;
        }
        memset(&dataIn, 0x00, size);
        memcpy(&dataIn, &incomingPacket, size);
        return true;
    }
    else
    {
        return false;
    }
}

const uint8_t Communicator::GetDistance()  {
    return distance;
}

uint8_t Communicator::CalculateDistance(uint8_t dBm, uint8_t min_dBm, uint8_t max_dBm)
{
    return abs(map(dBm, min_dBm, max_dBm, 0, 100));
}

int Communicator::scanWifi(const char *ssid)  {
  // scan WiFi for ssid, select strongest signal, return average over the last three scans
  int rssi = FAR_LIMIT;
  int n = WiFi.scanNetworks(false, false, GAME_CHANNEL);
  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      /* Serial.println(WiFi.SSID(i)); */
      if (WiFi.SSID(i) == ssid && WiFi.RSSI(i) > rssi)
        rssi = WiFi.RSSI(i);
      Serial.print("found rssi: ");
      Serial.println(rssi);
    }
  }
  rssis[2] = rssis[1];  rssis[1] = rssis[0];  rssis[0] = rssi;  // store last three scans
  /* return (rssis[0] + rssis[1] + rssis[2] ) / 3; // average */
  return rssi;
}
#endif
