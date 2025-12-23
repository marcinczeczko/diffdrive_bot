#ifndef ENCODER_DRIVER_H_
#define ENCODER_DRIVER_H_

#define ENCODER_USE_INTERRUPTS
#include "Encoder.h"

#include <Arduino.h>

class SafeEncoder
{
  private:
    Encoder* rawEncoder{nullptr};
    int pinA, pinB;
    // We keep this internal so the PID task doesn't have to deal with hardware pins
  public:
    SafeEncoder(int pinA, int pinB) : pinA(pinA), pinB(pinB)
    {
    }

    void begin()
    {
        // Now it's safe to create the encoder and attach interrupts
        rawEncoder = new Encoder(pinA, pinB);
    }

    // This handles the "Atomic Read" for the Uno R4
    auto getTicks() -> int32_t
    {
        int32_t val = 0;
        // The Encoder library is generally interrupt-safe,
        // but on 32-bit registers, a quick noInterrupts ensures
        // we don't read a value that's currently being updated.
        noInterrupts();
        val = rawEncoder->read();
        interrupts();
        return val;
    }

    void reset()
    {
        if (rawEncoder != nullptr)
        {
            noInterrupts();
            rawEncoder->write(0);
            interrupts();
        }
    }
};

#endif