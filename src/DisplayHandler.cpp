#include "Arduino.h"
#include "DisplayHandler.h"

DisplayHandler::DisplayHandler(uint8_t lcdAddr, uint8_t horizontal_digits = 16, uint8_t vertical_rows = 2) 
{
    lcd = std::make_unique<LiquidCrystal_I2C>(lcdAddr, horizontal_digits, vertical_rows); 
}
