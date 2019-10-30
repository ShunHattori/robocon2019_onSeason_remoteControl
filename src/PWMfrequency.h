#pragma once
#include "Arduino.h"

#if defined(__AVR_ATmega2560__)
void setPwmFrequencyMEGA2560(int pin, int divisor)
{
    byte mode;
    switch (divisor)
    {
    case 1:
        mode = 0x01;
        break;
    case 2:
        mode = 0x02;
        break;
    case 3:
        mode = 0x03;
        break;
    case 4:
        mode = 0x04;
        break;
    case 5:
        mode = 0x05;
        break;
    case 6:
        mode = 0x06;
        break;
    case 7:
        mode = 0x07;
        break;
    default:
        return;
    }
    switch (pin)
    {
    case 2:
        TCCR3B = TCCR3B & 0b11111000;
        TCCR3B = TCCR3B | mode;
        break;
    case 3:
        TCCR3B = TCCR3B & 0b11111000;
        TCCR3B = TCCR3B | mode;
        break;
    case 4:
        TCCR0B = TCCR0B & 0b11111000;
        TCCR0B = TCCR0B | mode;
        break;
    case 5:
        TCCR3B = TCCR3B & 0b11111000;
        TCCR3B = TCCR3B | mode;
        break;
    case 6:
        TCCR4B = TCCR4B & 0b11111000;
        TCCR4B = TCCR4B | mode;
        break;
    case 7:
        TCCR4B = TCCR4B & 0b11111000;
        TCCR4B = TCCR4B | mode;
        break;
    case 8:
        TCCR4B = TCCR4B & 0b11111000;
        TCCR4B = TCCR4B | mode;
        break;
    case 9:
        TCCR2B = TCCR2B & 0b11111000;
        TCCR2B = TCCR2B | mode;
        break;
    case 10:
        TCCR2B = TCCR2B & 0b11111000;
        TCCR2B = TCCR2B | mode;
        break;
    case 11:
        TCCR1B = TCCR1B & 0b11111000;
        TCCR1B = TCCR1B | mode;
        break;
    case 12:
        TCCR1B = TCCR1B & 0b11111000;
        TCCR1B = TCCR1B | mode;
        break;
    case 13:
        TCCR0B = TCCR0B & 0b11111000;
        TCCR0B = TCCR0B | mode;
        break;
    case 44:
        TCCR5C = TCCR5C & 0b11111000;
        TCCR5C = TCCR5C | mode;
        break;
    case 45:
        TCCR5B = TCCR5B & 0b11111000;
        TCCR5B = TCCR5B | mode;
        break;
    case 46:
        TCCR5A = TCCR5A & 0b11111000;
        TCCR5A = TCCR5A | mode;
        break;
    default:
        return;
    }
#ifdef DEBUGPRINT
    Serial.print("PWM frequency changed Pin:");
    Serial.println(pin);
    Serial.print("PWM frequency divisor:");
    Serial.println(divisor);
#endif
}

#endif
