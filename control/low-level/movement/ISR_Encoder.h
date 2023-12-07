#ifndef ISR_ENCODER_H
#define ISR_ENCODER_H

#include "Encoder.h"

/**
 * @class ISR_Encoder
 * @brief class for encoder interfacing using interrupts
*/
class ISR_Encoder : public Encoder
{
public:
    uint8_t PinA;   ///<the first pin of the encoder
    uint8_t PinB;   ///<the second pin if the encoder

    /**
     * @brief the constructor of the ISR encoder class
     * @param resolution the resolution of the encoder
     * @param pinA the first pin of the encoder
     * @param pinB the second pin of the encoder
    */
    ISR_Encoder(float resolution, uint8_t pinA, uint8_t pinB);

    /**
     * @brief method for interrupts on encoder pin A
    */
    void update_ISR_A(void);

    /**
     * @brief method for interrupts on encoder pin B
    */
    void update_ISR_B(void);

    /**
     * @brief method for encoder setup
    */
    void setup();
    
};

#endif