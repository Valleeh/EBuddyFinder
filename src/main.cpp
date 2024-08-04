#include "Arduino.h"
#include "Tasks.hpp"
using namespace utilities; 
#define ACCELTIME 10
#include <Communicator.hpp>
Communicator* com;
/* Communicator(unsigned long period, Scheduler* aS, Scheduler* aSensors) : */
#include "light.hpp"

Light* light;


void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("setup");

    ts.setHighPriorityScheduler(&hts);

    com= new Communicator(TASK_MILLISECOND*100,&hts, &ts);
    com->init(false);
    com->enable();
    light= new Light(&hts, com);
}

void loop()
{
    ts.execute();
}


