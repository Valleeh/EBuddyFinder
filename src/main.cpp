#include "Arduino.h"
#include "Tasks.hpp"
#include <Communicator.hpp>
#include "light.hpp"

class App {

  private:
    std::unique_ptr<ILight> lighter;
    std::unique_ptr<ICommunicator> com;

    Task LightTask;
    Task CommunicatorTask;
  public:
    App() {

      lighter = std::make_unique<Light>();
      com = std::make_unique<Communicator>();

      LightTask.set(TASK_MILLISECOND * 10, TASK_FOREVER, 
        [this](){
          if(this->lighter) {
            this->lighter->Callback();
          }
        });
      hts.addTask(LightTask); // give task to high priority
      LightTask.enable();

      CommunicatorTask.set(TASK_MILLISECOND*100, TASK_FOREVER,
        [this](){
          if(this->lighter && this->com) {
            this->com->Callback();
            this->lighter->SetSpeed(this->com->GetDistance());
          }
        });
      ts.addTask(CommunicatorTask);
      CommunicatorTask.enable();
    }
  void setup()
  {
      Serial.begin(115200);
      while (!Serial) {}
      Serial.println("setup");

      ts.setHighPriorityScheduler(&hts);
      com->init(false);
  }
};
App app;
void setup()
{
  app.setup();
}

void loop()
{
    ts.execute();
}


