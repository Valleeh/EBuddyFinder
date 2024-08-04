
#ifndef _COMMUNICATORCPP
#define _COMMUNICATORCPP
#include "Communicator.hpp"

// A UDP instance to let us send and receive packets over UDP--------------------------500*TASK_MILLISECOND
Communicator::Communicator(unsigned long period, Scheduler* aS, Scheduler* aSensors) :
    WiFiUDP(),
    Task(period,TASK_FOREVER,aS,false)
{
    // enable();
}

bool Communicator::Callback()
{
    distance = GetDistance(-100,-20);
    return false;
}
//////////////WIFI Init
bool Communicator::init(bool be_an_acesspoint)
{
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect();
    WiFi.softAP(ZOMBIE_SSID, password, GAME_CHANNEL, 0, 0);
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

        if(dataIn.restart)
        {
            RESTARTED=true;
            Serial.println("Restart recieved");
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool Communicator::started()
{
    if (recieve())
    {
        STARTED = dataIn.started;
    }
    else
    {
        Serial.println("Nothing recieved");
    }
    return STARTED;
}

bool Communicator::restarted()
{
    recieve();
    if (RESTARTED)
    {
        RESTARTED = false;
        return true;
    }
    return false;
}
int Communicator::dBm()
{
    return scanWifi(ZOMBIE_SSID);
    /* return WiFi.RSSI(); */
}



int Communicator::GetDistance(int min_dBm, int max_dBm)
{
    return abs(map(dBm(), min_dBm, max_dBm, 0, 100));
}
int Communicator::scanWifi(const char *ssid) {
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
