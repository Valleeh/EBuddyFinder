#include "Arduino.h"
#include "Tasks.hpp"
#include <Communicator.hpp>
#include "light.hpp"
void CommunicatorCallback();
void LightCallback();
Communicator com;
Light light;
Task LightTask(TASK_MILLISECOND * 10, TASK_FOREVER, &LightCallback, &hts ,true);
Task CommunicatorTask(TASK_MILLISECOND*100, TASK_FOREVER, &CommunicatorCallback, &ts, true);
void CommunicatorCallback() {
   com.Callback();
   light.SetSpeed(com.GetDistance());
}
void LightCallback() {
   light.Callback(); 
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("setup");

    ts.setHighPriorityScheduler(&hts);
    com.init(false);
}

void loop()
{
    ts.execute();
}


