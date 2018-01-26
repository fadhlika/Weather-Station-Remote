#ifndef LORA_H
#define LORA_H

#include "spi.h"

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

typedef enum _LoRa_Status {
    READY
} LoRa_Status;

typedef struct _LoRa_IO_TypeDef {
    uint16_t Pin;
    GPIO_TypeDef * Port;
} LoRa_IO_TypeDef;

typedef struct _LoRa_HandleTypeDef {
    SPI_HandleTypeDef * hspi;
    LoRa_IO_TypeDef ss;
    LoRa_IO_TypeDef dio0;
    LoRa_IO_TypeDef reset;
    int _frequency;
    int _packetIndex;
    int _implicitHeaderMode;
    void(*_onReceive)(int);
} LoRa_HandleTypeDef;

extern LoRa_HandleTypeDef lora;

HAL_StatusTypeDef LoRa_Begin(long frequency);
HAL_StatusTypeDef LoRa_Transmit(uint8_t* buffer, int length);
HAL_StatusTypeDef LoRa_Receive(uint8_t* buffer);
HAL_StatusTypeDef LoRa_Idle();
HAL_StatusTypeDef LoRa_Sleep();
HAL_StatusTypeDef LoRa_SetTxPower(int level, int outputPin);
HAL_StatusTypeDef LoRa_SetFrequency(long frequency);
HAL_StatusTypeDef LoRa_SetSpreadingFactore(int sf);
HAL_StatusTypeDef LoRa_SetSignalBandwidth(long sbw);
HAL_StatusTypeDef LoRa_SetCodingRate4(int denominator);
HAL_StatusTypeDef LoRa_SetPreambleLength(long length);
HAL_StatusTypeDef LoRa_SetSyncWord(int sw);
HAL_StatusTypeDef LoRa_EnableCrc();
HAL_StatusTypeDef LoRa_DisableCrc();
HAL_StatusTypeDef LoRa_OnDioRise();

static void end();

static int beginPacket(int implicitHeader);
static int endPacket();

static int parsePacket(int size);
static int packetRssi();
static float packetSnr();

// from Print
static void write(uint8_t *buffer, int size);
// from Stream
static int available();
static int read();
static int peek();
static void flush();

static void onReceive(void(*callback)(int));

static void receive(int size);
static void idle();
static void sleep();

static void setTxPower(int level, int outputPin);
static void setFrequency(long frequency);
static void setSpreadingFactor(int sf);
static void setSignalBandwidth(long sbw);
static void setCodingRate4(int denominator);
static void setPreambleLength(long length);
static void setSyncWord(int sw);
static void enableCrc();
static void disableCrc();

static uint8_t random();


static void explicitHeaderMode();
static void implicitHeaderMode();

static void handleDio0Rise();

static uint8_t readRegister(uint8_t address);
static void writeRegister(uint8_t address, uint8_t value);
static uint8_t singleTransfer(uint8_t address, uint8_t value);

static void onDio0Rise();
#endif