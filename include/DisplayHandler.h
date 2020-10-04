#ifndef DisplayHandler_h
#define DisplayHandler_h

#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
#include <memory> 

class DisplayHandler
{
    public:
      DisplayHandler(uint8_t lcdAddr, uint8_t horizontal_digits, uint8_t vertical_rows );
      void showbootloop();
      // void view(uint8_t menuSelectedSensor);
    private:
      std::unique_ptr<LiquidCrystal_I2C> lcd;
      uint8_t oldSelectedMenu;
};

#endif