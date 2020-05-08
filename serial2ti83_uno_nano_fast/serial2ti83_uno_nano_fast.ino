/* ArduGrayLink, a modern yet simple TI Graph-Link cable
 *  https://github.com/MTres19/serial2ti83
 *  https://github.com/DSchndr/serial2ti83
 *  https://github.com/jw0k/serial2ti83
 * 
 * Copyright (C) 2017 jw0k
 * Copyright (C) 2018 DSchndr
 * Copyright (C) 2018 Matthew Trescott
 * 
 * MIT-licensed.
 * 
 * SETUP FOR LEONARDO AND PRO MICRO
 */

const int TIP = A0; //arduino port where the tip of the stereo jack plug is connected
const int RING = A1; //arduino port where the middle part of the stereo jack plug is connected
const unsigned long TXTIMEOUT = 1000; //ms
const unsigned long RXTIMEOUT = 1000; //ms

#define TI_TIP_IS_HIGH   (PINC & (uint8_t)0b00000001) // No need for bitshift because it's the first pin.
#define TI_RING_IS_HIGH ((PINC & (uint8_t)0b00000010) >> 1)

#define TI_TIP_IS_LOW   ((~PINC) & (uint8_t)0b00000001) // No need for bitshift because it's the first pin.
#define TI_RING_IS_LOW (((~PINC) & (uint8_t)0b00000010) >> 1)

inline void TiTipLow()
{
    PORTC &= (uint8_t)0b11111110; // Save an instruction https://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_intpromote
}
inline void TiRingLow()
{
    PORTC &= (uint8_t)0b11111101;
}
inline void TiTipHigh()
{
    PORTC |= (uint8_t)0b00000001;
}
inline void TiRingHigh()
{
    PORTC |= (uint8_t)0b00000010;
}
inline void TiTipOutput()
{
    DDRC |= (uint8_t)0b00000001;
}
inline void TiRingOutput()
{
    DDRC |= (uint8_t)0b00000010;
}
inline void TiTipInput()
{
    DDRC &= (uint8_t)0b11111110;
}
inline void TiRingInput()
{
    DDRC &= (uint8_t)0b11111101;
}

void sendByte(uint8_t byte)
{
    unsigned long currentTime;

    for (int i = 0; i < 8; ++i)
    {
        bool bit = byte & 0x01;
        byte >>= 1;

        //poll both lines until they are both high, which means we're ready to send a bit
        currentTime = millis();
        while (TI_TIP_IS_LOW || TI_RING_IS_LOW)
        {
            if (millis() - currentTime > TXTIMEOUT)
            {
                return;
            }
        }

        if (bit)
        {
            // send a bit by pulling appropriate line low
            TiRingOutput();
            TiRingLow();

            // wait for opposite line to become low
            currentTime = millis();
            while (TI_TIP_IS_HIGH)
            {
                if (millis() - currentTime > TXTIMEOUT)
                {
                    TiRingInput();
                    TiRingHigh();
                    return;
                }
            }

            // release our line
            TiRingInput();
            TiRingHigh();

            // wait for opposite line to become high
            currentTime = millis();
            while (TI_TIP_IS_LOW)
            {
                if (millis() - currentTime > TXTIMEOUT)
                {
                    return;
                }
            }
        }
        else
        {
            // send a bit by pulling appropriate line low
            TiTipOutput();
            TiTipLow();

            // wait for opposite line to become low
            currentTime = millis();
            while (TI_RING_IS_HIGH)
            {
                if (millis() - currentTime > TXTIMEOUT)
                {
                    TiTipInput();
                    TiTipHigh();
                    return;
                }
            }

            // release our line
            TiTipInput();
            TiTipHigh();

            // wait for opposite line to become high
            currentTime = millis();
            while (TI_RING_IS_LOW)
            {
                if (millis() - currentTime > TXTIMEOUT)
                {
                    return;
                }
            }
        }
    }
}

bool getByte(uint8_t& byte)
{
    // only enter if a pin is low, otherwise you always get the timeout 
    if (TI_RING_IS_HIGH && TI_TIP_IS_HIGH) return false;

    unsigned long currentTime;
    uint8_t result = 0;
    for (int i = 0; i < 8; ++i)
    {
        //poll both lines until one of them becomes low
        currentTime = millis();
        while (TI_RING_IS_HIGH && TI_TIP_IS_HIGH)
        {
            if (millis() - currentTime > RXTIMEOUT)
            {
                return false;
            }
        }
        
        bool bit = TI_RING_IS_LOW;
        result >>= 1;
        
        if (bit)
        {
            result |= 0x80; //bits are always transmitted LSb first (least significant bit)
            
            // acknowledge a bit by pulling appropriate line low
            TiTipOutput();
            TiTipLow();
            
            //wait for opposite line to become high
            currentTime = millis();
            while (TI_RING_IS_LOW)
            {
                if (millis() - currentTime > RXTIMEOUT)
                {
                    TiTipInput();
                    TiTipHigh();
                    return false;
                }
            }

            // release our line
            TiTipInput();
            TiTipHigh();
        }
        else
        {            
            // acknowledge a bit by pulling appropriate line low
            TiRingOutput();
            TiRingLow();
            
            //wait for opposite line to become high
            while (TI_TIP_IS_LOW)
            {
                if (millis() - currentTime > RXTIMEOUT)
                {
                    TiRingInput();
                    TiRingHigh();
                    return false;
                }
            }

            // release our line
            TiRingInput();
            TiRingHigh();
        }

        // wait for both lines stable high
        currentTime = millis();
        while (TI_TIP_IS_LOW || TI_RING_IS_LOW)
        {
            if (millis() - currentTime > RXTIMEOUT)
            {
                return false;
            }
        }

    }

    byte = result;
    return true;
}

void setup()
{
    //turn off the built-in LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(9600);

    //configure both lines to be in a high-impedance state and enable pull-up resistors
    TiTipInput();
    TiTipHigh();
    TiRingInput();
    TiRingHigh();
}

void loop()
{
    //forward incoming data from PC to the calculator
    while (Serial.available() > 0)
    {
        // Warn if the calculator isn't keeping up with the serial port
        if (Serial.available() > 240) {
          digitalWrite(LED_BUILTIN, HIGH);
        }
        sendByte(Serial.read());
    }

    //forward incoming data from calculator to the PC
    uint8_t byteFromCalc;
    if (getByte(byteFromCalc))
    {
        Serial.write(byteFromCalc);
    }
}
