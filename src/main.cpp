#include "Arduino.h"
#include "Tasks.hpp"
using namespace utilities; 
#define ACCELTIME 10
#include <Communicator.hpp>
#include "light.hpp"
void CommunicatorCallback();
void LightCallback();
Communicator com;
Light light;
Task  lightTask(TASK_MILLISECOND * 10, TASK_FOREVER, &LightCallback, &hts,false);
/* Task CommunicatorTask(TASK_MILLISECOND*100, TASK_FOREVER, &CommunicatorCallback, &ts, false); */
void CommunicatorCallback() {
   com.Callback(); 
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
    /* com= new Communicator(); */
    com.init(false);
    /* light= new Light(); */
}

void loop()
{
    ts.execute();
}


