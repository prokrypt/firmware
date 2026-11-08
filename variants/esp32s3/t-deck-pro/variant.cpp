#include "variant.h"
#include "Arduino.h"

void earlyInitVariant()
{
#if defined(MESHTASTIC_INCLUDE_INKHUD) && defined(BOARD_1V8_EN)
    pinMode(BOARD_1V8_EN, OUTPUT);
    digitalWrite(BOARD_1V8_EN, HIGH);
#endif
    pinMode(LORA_EN, OUTPUT);
    digitalWrite(LORA_EN, HIGH);
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    pinMode(SDCARD_CS, OUTPUT);
    digitalWrite(SDCARD_CS, HIGH);
    pinMode(PIN_EINK_CS, OUTPUT);
    digitalWrite(PIN_EINK_CS, HIGH);
}
