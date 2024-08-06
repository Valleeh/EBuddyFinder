#ifndef _LIGHTHPP
#define _LIGHTHPP
#include <FastLED.h>
#define FASTLED_INTERRUPT_RETRY_COUNT 0
#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_ESP8266_RAW_PIN_ORDER
#define LED_PIN D6
#define DATA_PIN D6
#define NUM_LEDS 8
#define NUMPIXELS NUM_LEDS
#define BRIGHTNESS 100
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define IMPLEMENTATION LIFO
FASTLED_USING_NAMESPACE

SmartSerial sLight(Serial, "Light/", " # ");
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 1000
CRGBPalette16 currentPalette;
TBlendType currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
class ILight {
 public:
  
  virtual ~ILight() {}
  virtual bool Callback() = 0;
  virtual void SetSpeed(unsigned int new_speed) = 0;
};

class Light : public ILight {
private:
  uint8_t startIndex = 0;
  unsigned int speed{0};
  uint8_t bright = 255;
  uint8_t brightness = BRIGHTNESS;


  void FillLEDsFromPaletteColors(uint8_t colorIndex)
  {
      currentPalette = RainbowColors_p;
      currentBlending = LINEARBLEND;
      for (int i = 0; i < (sizeof(leds) / sizeof(leds[0])); i++) {
        leds[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
      }
      colorIndex += 4;
      FastLED.show();
  }

public:
  ~Light() override {}
  Light()  {

      // FASTLED init
      FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
      FastLED.setBrightness(BRIGHTNESS);

      // Start AccelerationSensor and enable Task Scheduler for this Sensor

      currentBlending = LINEARBLEND;
  }


  void SetSpeed(unsigned int new_speed) override {
    speed = new_speed;
  }

  bool Callback() override {
      Serial.println("light-callback");
      startIndex = startIndex + speed;
      FillLEDsFromPaletteColors(startIndex);
      FastLED.delay(1000 / UPDATES_PER_SECOND);
      return false;
  }
};
#endif // _LIGHTSTATEHPP
