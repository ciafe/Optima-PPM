#include <SoftwareSerial8e2.h>

#if OPTIMA_PINS==7
  SoftwareSerial8e2 sBus(8, 17, !SBUS_INVERTED); // RX on PB8 (not used), TX on PC3, inverse logic
#elif OPTIMA_PINS==6
  SoftwareSerial8e2 sBus(8, 16, !SBUS_INVERTED); // RX on PB8 (not used), TX on PC2, inverse logic
#else
  #error "Currently only Optima 6 and 7 are supported"
#endif



#define SBUS_MIN_OFFSET       173
#define SBUS_MID_OFFSET       992
#define SBUS_MAX_OFFSET       1811
#define SBUS_CHANNEL_NUMBER   16
#define SBUS_PACKET_LENGTH    25
#define SBUS_FRAME_HEADER     0x0F
#define SBUS_FRAME_FOOTER     0x00
#define SBUS_FRAME_FOOTER_V2  0x04
#define SBUS_STATE_FAILSAFE   0x08
#define SBUS_STATE_SIGNALLOSS 0x04

uint8_t sbusPacket[SBUS_PACKET_LENGTH];

void sbus_init() {
    sBus.begin(100000);
}

uint16_t map16b(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  if (x < in_min){
    x = in_min;
  }
  else if (x > in_max){
    x = in_max;
  }
  return (uint16_t)((uint32_t)(x - in_min) * (uint32_t)(out_max - out_min) / (uint32_t)(in_max - in_min) + out_min);
}

void sbusTransmitPacket(uint16_t channels[], bool isSignalLoss, bool isFailsafe)
{
    uint16_t output[OPTIMA_NUM_CHANNELS];

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < OPTIMA_NUM_CHANNELS; i++) {
        output[i] = map16b(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }
    /* Unused channels, set to minim value */
     /* Removed for code optmization... Fix value SBUS_MIN_OFFSET for unused channels are set below in the sbusPacket[x]
    for (uint8_t i = OPTIMA_NUM_CHANNELS; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = SBUS_MIN_OFFSET;
    }*/
    

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }

    sbusPacket[0] = SBUS_FRAME_HEADER;   //Header
    sbusPacket[1] = (uint8_t) (output[0] & 0x07FF);
    sbusPacket[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    sbusPacket[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    sbusPacket[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    sbusPacket[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    sbusPacket[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    sbusPacket[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    sbusPacket[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    sbusPacket[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    sbusPacket[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    sbusPacket[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    sbusPacket[12] = (uint8_t) ((output[8] & 0x07FF));
    sbusPacket[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | 0x68);

    /* Removed for code optmization... code fix SBUS_MIN_OFFSET values for all unused channels 
    sbusPacket[14] = (uint8_t) ((output[9]  & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    sbusPacket[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    sbusPacket[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    sbusPacket[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    sbusPacket[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    sbusPacket[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    sbusPacket[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    sbusPacket[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    sbusPacket[22] = (uint8_t) ((output[15] & 0x07FF)>>3);
    */
    sbusPacket[14] = (uint8_t)0x45;
    sbusPacket[15] = (uint8_t)0x2B;
    sbusPacket[16] = (uint8_t)0x5A;
    sbusPacket[17] = (uint8_t)0xD1;
    sbusPacket[18] = (uint8_t)0x8A;
    sbusPacket[19] = (uint8_t)0x56;
    sbusPacket[20] = (uint8_t)0xB4;
    sbusPacket[21] = (uint8_t)0xA2;
    sbusPacket[22] = (uint8_t)0x15;
    sbusPacket[23] = stateByte;         //Flags byte
    sbusPacket[24] = SBUS_FRAME_FOOTER; //Footer

    /* transmit SBUS packet */
    for(uint8_t iloop=0; iloop <SBUS_PACKET_LENGTH; iloop++){
      sBus.write_8E2(sbusPacket[iloop]);
    }
}

