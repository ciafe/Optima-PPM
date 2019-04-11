/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>


//#define DEBUG_PRINT               /* Enable only for Arduino compilation!! */

/* Select one of the next compiler options:  */
//#define  PPM_MIXED_FS_ENABLED      /* PWM and PPM, with Fail Safe */
//#define  SBUS_MIXED_FS_ENABLED       /* PWM and SBUS, with Fail Safe */
#define  SBUS_PPM_SEL_MIXED_FS_ENABLED   /* SBUS PPM selectable, with PWM and Fail Safe  */

#define  SBUS_INVERTED      false     /* SBUS pin is using inverted logic, set to false if using nominal SBUS logic */

#if defined(PPM_MIXED_FS_ENABLED) && defined(SBUS_MIXED_FS_ENABLED)
  #error "Please, select only one compiler option."
#elif defined(PPM_MIXED_FS_ENABLED) && defined(SBUS_PPM_SEL_MIXED_FS_ENABLED)
  #error "Please, select only one compiler option."
#elif defined(SBUS_MIXED_FS_ENABLED) && defined(SBUS_PPM_SEL_MIXED_FS_ENABLED)
  #error "Please, select only one compiler option."
#endif

#include "optima.h"
#include "usart.h"

/* 400us  */
#define PPM_PRE_PULSE     400
/* 4600 us */
#define PPM_SYNC_PERIOD  (4600 - PPM_PRE_PULSE)
/* Conversion fctor to get us from the 7.3728MHz ticks */
#define OSC_7_3728MHZ_TO_US_X1000 1356


#define MILLIS_MAX_TIME          0xFFFFFFFF
#define CYCLE_TIME               20                /* value in ms */
#define FS_TIMEOUT               1000/CYCLE_TIME   /* value in ms */
#define SIGNAL_LOST              4                 /* value in cycles */
#define FS_DIS_TRESHOLD_COUNT    10
#define FS_EN_TRESHOLD_COUNT     200

/* State machine states */
enum states   {START, PREAMBLE, PACKET, VERIFY};

/* EEPROM Data info */
#define FAILSAFESTATE_ADDRESS    0x0
#define FAILSAFEDATA_ADDRESS     0x2

static uint16_t inputs[OPTIMA_NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint16_t inputsFS[OPTIMA_NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 
static uint8_t  buffer[2 * OPTIMA_NUM_CHANNELS];
static Type_FailSafeStates failSafeState = FS_DISABLED;
static uint8_t  failSafeDetected = false;
static uint8_t  failSafeOnHold   = true;
static uint8_t  newDataReceived  = false;
static uint16_t failSafetimeout  = 0;
static uint8_t  serialOuputCfg   = 0;
static uint8_t  failSafeDisableRxCount = 0;
static uint8_t  failSafeEnableRxCount = 0;
static unsigned long lastMillis = 0;

inline void failsafe_Init(void);
inline void mode_setup(void);

void Rx_Handler(void);
void ppm_handler(void);
void sbus_handler(void);
void failsafe_handler(void);
void FailSafeConfig_Handler(uint8_t* counter, uint8_t state, uint8_t limit);


void setup()
{
  #if defined(DEBUG_PRINT)
  Serial.begin(115200);
  Serial.println("Init..");
  #else
  usart_init();
  #endif
    
  sei();
  failsafe_Init();
  mode_setup();
  
  lastMillis = millis();
}

inline void mode_setup(void)
{
  uint8_t i;

  #if defined(SBUS_PPM_SEL_MIXED_FS_ENABLED)
    /* Enable pin pullup to be able to read the pin */
    *outputs[MODE_SEL_PIN].port |= outputs[MODE_SEL_PIN].mask;
    /* Add some delay for input stabilization */
    #if F_CPU == 7372800L
    delayMicros(5000);
    #else
    delayMicroseconds(5000);
    #endif
    if(!(*outputs[MODE_SEL_PIN].pin & outputs[MODE_SEL_PIN].mask))
    {
      /* If jumper is set to ground during startup, SBUS is enabled */
      serialOuputCfg = SBUS_ENABLED;
      sbus_init();
      #if defined(DEBUG_PRINT)
      Serial.println("SBUS enabled");
      #endif
    }
    else{
      serialOuputCfg = PPM_ENABLED; /* By default PPM is enabled */
      #if defined(DEBUG_PRINT)
      Serial.println("PPM enabled");
      #endif
    }
    /* Disable pin pullup */
    *outputs[MODE_SEL_PIN].port &= ~outputs[MODE_SEL_PIN].mask;
    /* initialise output - excluding the MODE_SELECTION, keep it as input */
    for(i = 0; i < PWM_PINS; ++i) {
      *outputs[i].ddr |= outputs[i].mask;
    }
    *outputs[PPM_OUT_PIN].ddr |= outputs[PPM_OUT_PIN].mask;
  
  #elif defined(SBUS_MIXED_FS_ENABLED)
    serialOuputCfg = SBUS_ENABLED;
    for(i = 0; i < PWM_PINS; ++i) {
      *outputs[i].ddr |= outputs[i].mask;
    }
    *outputs[PPM_OUT_PIN].ddr |= outputs[PPM_OUT_PIN].mask;
    sbus_init();
  
  #else /* PPM_MIXED_FS_ENABLED */
    serialOuputCfg = PPM_ENABLED;
    for(i = 0; i < PWM_PINS; ++i) {
      *outputs[i].ddr |= outputs[i].mask;
    }
    *outputs[PPM_OUT_PIN].ddr |= outputs[PPM_OUT_PIN].mask;
  #endif
  

}

void loop(void)
{	
  unsigned long cycle_time, current_time;

  /* Check for RX commands from main processor (1 frames each 20ms) */
  Rx_Handler();

  /* Generate system ticks every CYCLE_TIME */
  current_time = millis();
  if (current_time >= lastMillis){
      cycle_time = current_time - lastMillis;
  }
  else{ /* counter overflow detected (each 50 days), just in case... */
      cycle_time = current_time + (MILLIS_MAX_TIME - lastMillis);
  }
  
  if ((cycle_time >= CYCLE_TIME) || (newDataReceived == true)) {
    /* Execute every new frame received or every CYCLE_TIME */
    lastMillis = current_time;

    /* Process Fails safe, if needed */
    failsafe_handler(); 

    /* Process data outputs */
    outputs_handler();
  }
   
}

void Rx_Handler(void)
{
/* State variable */
  static enum states state = START;
  static uint8_t buffer_pos;
  uint8_t i;
  uint8_t byte;
  uint32_t data;

    #if defined(DEBUG_PRINT)
    if(Serial.available())
    {
      byte = Serial.read();
      if ((byte >= '0') && (byte <= '9'))
      {
        newDataReceived = true;
        data = byte - '0';
        data = map(data, 0, 9, 7000, 15000);
        Serial.print("New TX input: ");
        Serial.print(data);
        Serial.print(", converted to: ");
        data = (uint32_t)(data * OSC_7_3728MHZ_TO_US_X1000)/(uint32_t)10000; /* Convert to us */
        Serial.println((uint16_t)data);
        for(i = 0; i < OPTIMA_NUM_CHANNELS; ++i) {
          inputs[i] = data;
        }
      }
      else if (byte == 'E')
      {
        failSafeDisableRxCount = 0;
        FailSafeConfig_Handler(&failSafeEnableRxCount, FS_ENABLED, 10);
      }
      else if (byte == 'D')
      {
        failSafeEnableRxCount = 0;
        FailSafeConfig_Handler(&failSafeDisableRxCount, FS_DISABLED, 5);
      }
      else if (byte == 'S')
      {
        failSafeState = (Type_FailSafeStates)EEPROM.read(FAILSAFESTATE_ADDRESS);
        Serial.print("failSafeState: ");
        Serial.println(failSafeState, HEX);
        EEPROM.get( FAILSAFEDATA_ADDRESS, inputsFS );
        Serial.print("inputsFS: ");
        for(i = 0; i < OPTIMA_NUM_CHANNELS; ++i) {
          Serial.print(inputsFS[i]);Serial.print(", ");
        }
        Serial.println();
        Serial.print("inputs: ");
        for(i = 0; i < OPTIMA_NUM_CHANNELS; ++i) {
          Serial.print(inputs[i]);Serial.print(", ");
        }
        Serial.println();
      }
    }
    #else
    /* We have a character, do something */
    if(!q_empty(usart_rx))
    {
      byte = q_take(usart_rx);
      switch(state)
      {
        case START: // A packet is arriving
          if(byte == OPTIMA_PROTOCOL_START) {
            state = PREAMBLE;
          }   
          break;
        case PREAMBLE: // The packet begins on the next byte
          if(byte == OPTIMA_PROTOCOL_DATA) {
            state = PACKET;
            buffer_pos = 0;
            /* If normal data is received, reset FailSafe counters */
            failSafeDisableRxCount = 0;
            failSafeEnableRxCount = 0;
          } else if(byte == OPTIMA_PROTOCOL_FS_EN) {
            state = PACKET;
            buffer_pos = 0;
            FailSafeConfig_Handler(&failSafeEnableRxCount, FS_ENABLED, FS_EN_TRESHOLD_COUNT);
          } else if(byte == OPTIMA_PROTOCOL_FS_DIS) {
            state = PACKET;
            buffer_pos = 0;
            FailSafeConfig_Handler(&failSafeDisableRxCount, FS_DISABLED, FS_DIS_TRESHOLD_COUNT);  
          } else {
            /* Invalif, unknown PREAMBLE byte received... Restat frame */
            state = START;
          }
          break;
        case PACKET: // Fill the buffer with the packet values
          buffer[buffer_pos++] = byte;
          if(buffer_pos >= (2 * OPTIMA_NUM_CHANNELS)) {
            // We have received the channel data
            state = VERIFY;
          }
          
          break;
        case VERIFY: // Verify the packet has finished
          if(byte == OPTIMA_PROTOCOL_END) {
            // Fill the input values with the adjusted channel timing
            for(i = 0; i < OPTIMA_NUM_CHANNELS; ++i) {
              data = ((uint32_t)(buffer[2*i] << 8) | (uint32_t)buffer[2*i+1]); /* Value in ticks of 7,372800 MHz */
              data = (uint32_t)(data * OSC_7_3728MHZ_TO_US_X1000)/(uint32_t)10000; /* Convert to us */
              inputs[i] = (uint16_t)data;
            }
            /* Set flag for Fail Safe detection */
            newDataReceived = true;
            
          }
          else {
            /* Invalif, unknown VERIFY byte received... Restat frame */
            state = START;
          }
        default:
          state = START;
      }
   }
   else{

   }
   #endif
}

void FailSafeConfig_Handler(uint8_t* counter, uint8_t state, uint8_t limit)
{
  uint8_t i;
  
  *counter = *counter + 1;
  #if defined(DEBUG_PRINT)
  Serial.print("FailSafeConfig_Handler... ");
  Serial.println(*counter);
  #endif
  if (*counter == limit){
    failSafeState = (Type_FailSafeStates)state;
    EEPROM.write(FAILSAFESTATE_ADDRESS, failSafeState);
    /* Get the last received values to be used for FS */
    for(i = 0; i < OPTIMA_NUM_CHANNELS; ++i) {
      inputsFS[i] = inputs[i];
    }
    EEPROM.put( FAILSAFEDATA_ADDRESS, inputsFS );
  }
  else if (*counter > limit){
    *counter = limit + 1; /* stop the counter */
  }
  else{}
}

inline void outputs_handler(void)
{
	 uint8_t i;

   if ((failsafe_SignalLost() && (failSafeDetected == false)) || (failSafeOnHold == true))
   {
     /* - If RX started without TX enabled (failSafeOnHold == true)
      * - If TX signal is lost (after being received before) but Fail Safe, is still not activated (failsafe_SignalLost() && (failSafeDetected == false))
          Then no PWM/PPM generation
     */
   }
   else
   { 
      cli(); /* Disable interrupts to allow precise pulse generation */
    	for(i = 0; i < PPM_CHANNELS; ++i) {
    		/* ppm pre-pulse */
        #if defined(SBUS_PPM_SEL_MIXED_FS_ENABLED) || defined(PPM_MIXED_FS_ENABLED)
        if (serialOuputCfg == PPM_ENABLED){
    		  *outputs[PPM_OUT_PIN].port |= outputs[PPM_OUT_PIN].mask;
        }
        #endif
    		if(i < PWM_PINS)
    			*outputs[i].port |= outputs[i].mask;
        #if F_CPU == 7372800L
    		delayMicros(PPM_PRE_PULSE);
        #else
        delayMicroseconds(PPM_PRE_PULSE);
        #endif
    
    		/* end of pre-pulse */   
        #if defined(SBUS_PPM_SEL_MIXED_FS_ENABLED) || defined(PPM_MIXED_FS_ENABLED) 
        if (serialOuputCfg == PPM_ENABLED){
    		  *outputs[PPM_OUT_PIN].port &= ~outputs[PPM_OUT_PIN].mask;
        }
        #endif
        #if F_CPU == 7372800L
        delayMicros(inputs[i] - PPM_PRE_PULSE);
        #else
        delayMicroseconds(inputs[i] - PPM_PRE_PULSE);
        #endif
    	
    		/* end of channel timing */
    		if(i < PWM_PINS)
    			*outputs[i].port &= ~outputs[i].mask;
    	}
    	/* ppm sync pulse */
      #if defined(SBUS_PPM_SEL_MIXED_FS_ENABLED) || defined(PPM_MIXED_FS_ENABLED) 
      if (serialOuputCfg == PPM_ENABLED){
      	*outputs[PPM_OUT_PIN].port |= outputs[PPM_OUT_PIN].mask;
        #if F_CPU == 7372800L
      	delayMicros(PPM_PRE_PULSE);
        #else
        delayMicroseconds(PPM_PRE_PULSE);
        #endif
      	/* ppm sync period */
      	*outputs[PPM_OUT_PIN].port &= ~outputs[PPM_OUT_PIN].mask;
        sei(); /* Reenable interrupts before the sync Period */
        #if F_CPU == 7372800L
      	delayMicros(PPM_SYNC_PERIOD);
        #else
        delayMicroseconds(PPM_SYNC_PERIOD);
        #endif
      }
      #endif
  
      sei(); /* Reenable interrupts */
    }
  
    /* SBUS is always transmitted, if configured */
    #if defined(SBUS_PPM_SEL_MIXED_FS_ENABLED) || defined(SBUS_MIXED_FS_ENABLED)
    if (serialOuputCfg == SBUS_ENABLED){
      /* SBUS enabled, send data packets */
      sbusTransmitPacket(inputs, failsafe_SignalLost(), failSafeDetected);
    }
    #endif
}




void failsafe_Init(void)
{
  failSafeDetected = false;
  newDataReceived  = false;
  failSafeOnHold   = true;
  
  
  /* Read current fail Safe configuration */
  failSafeState = (Type_FailSafeStates)EEPROM.read(FAILSAFESTATE_ADDRESS);

  #if defined(DEBUG_PRINT)
  Serial.print("FS read: "); Serial.println(failSafeState);
  #endif

  /* Check for initial configuration */
  if ((failSafeState != FS_DISABLED) && (failSafeState != FS_ENABLED))
  {
    failSafeState = FS_DISABLED;
    #if defined(DEBUG_PRINT)
    Serial.println("Init EEPROM state");
    #endif
    EEPROM.write(FAILSAFESTATE_ADDRESS, failSafeState);
  }
  else{
    /* valid state read, kept on global variable */
    EEPROM.get( FAILSAFEDATA_ADDRESS, inputsFS );
  }
}

void failsafe_handler(void)
{
  uint8_t i;
  
  /* Fail Safe detection */
  if (newDataReceived == true)
  {
    failSafeOnHold   = false; /* First data from TX received, let's enable FailSafe */
    failSafeDetected = false;
    newDataReceived  = false;
    failSafetimeout  = 0;
    #if defined(DEBUG_PRINT)
    Serial.println("FS reset");
    #endif
  }
  else
  {
      if (failSafeOnHold == false) /* only check FailSafe if the TX has been enabled and first package received */
      {
        failSafetimeout++;
        if (failSafetimeout >= FS_TIMEOUT){
          #if defined(DEBUG_PRINT)
          if (failSafetimeout == FS_TIMEOUT){
            Serial.println("FS detected...");
          }
          #endif
          failSafetimeout = FS_TIMEOUT;
          if(failSafeState == FS_ENABLED)
          {
            failSafeDetected = true;
          }
        }
      }
  }

  /* Fail Safe reaction */
  if(failSafeDetected == true)
  {
    /* Load fail safe stored values */
    for(i = 0; i < OPTIMA_NUM_CHANNELS; ++i) {
      inputs[i] = inputsFS[i];
    }
  }

  #if defined(DEBUG_PRINT)
  if (failsafe_SignalLost())
  {
    //Serial.println("Signal lost...");
  }
  #endif
}

uint8_t failsafe_SignalLost(void)
{
  return(failSafetimeout > SIGNAL_LOST);
}


#if F_CPU == 7372800L
/* Function created to allow microseconds delays with a 7.3728MHz crystal, 
   base on delayMicroseconds() arduino function */
void delayMicros(unsigned int us)
{
  unsigned long data;

  if (us <= 5) return;
  /* Escalate us value to simulate 8MHz cock: (us*236)/128  */
  data = (unsigned long)us * 236;
  data >>= 7;
  us = (unsigned int)data - 7;
  
  // busy wait
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t" // 2 cycles
    "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
  );
}
#endif
