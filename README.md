#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <HardwareTimer.h>
#include "codec2.h"
#include "audio.h"
#include "radio_init.h"
#include <malloc.h>

// --- ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ---
HardwareTimer *pwmTimer = NULL;
struct CODEC2 *c2 = NULL;
int samplesPerFrame = 0;
int bytesPerFrame = 0;
int packetSize = 0;

int16_t*  audioBuffer = NULL;    
uint8_t*  txPacketBuffer = NULL; 
uint8_t*  rxPacketBuffer = NULL; 

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

// ============================================
// MULTI-BUFFER BLOCK QUEUE
// ============================================
#define NUM_ADC_BLOCKS 4
#define ADC_BLOCK_SAMPLES 320

struct AdcBlock {
    int16_t data[ADC_BLOCK_SAMPLES];
    uint32_t timestamp;
    uint16_t sequence;
};

AdcBlock adcBlocks[NUM_ADC_BLOCKS];
volatile uint8_t blockWriteIdx = 0;
volatile uint8_t blockReadIdx = 0;
volatile uint8_t blocksAvailable = 0;
volatile uint32_t blockSequence = 0;
volatile uint32_t blocksLost = 0;
volatile bool adcBufferReady = false;

// ============================================
// RX БУФЕР
// ============================================
#define RX_RING_SIZE 1280
int16_t rxRingBuffer[RX_RING_SIZE];
volatile int rxWriteIdx = 0;
volatile int rxReadIdx = 0;
volatile int rxBufferedSamples = 0;
volatile uint32_t rxUnderrunCnt = 0;
volatile uint32_t rxOverrunCnt = 0;

// ============================================
// TX PACKET FIFO
// ============================================
#define TX_QUEUE_SIZE 4
struct TxPacket {
    uint8_t data[32];
    uint8_t len;
};
TxPacket txQueue[TX_QUEUE_SIZE];
volatile uint8_t txHead = 0;
volatile uint8_t txTail = 0;
volatile uint8_t txCount = 0;
volatile uint32_t txLost = 0;

// ============================================
// RX PACKET QUEUE
// ============================================
#define RX_QUEUE_SIZE 4
struct RxPacket {
    uint8_t data[32];
    uint8_t len;
    uint16_t sequence;
    uint32_t timestamp;
};
RxPacket rxQueue[RX_QUEUE_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile uint8_t rxCount = 0;
volatile uint32_t rxQueueLost = 0;

// ============================================
// SEQUENCE + REORDER
// ============================================
#define REORDER_SIZE 8
#define REORDER_MASK (REORDER_SIZE - 1)

struct ReorderSlot {
    uint8_t data[32];
    uint8_t len;
    bool ready;
    uint32_t timestamp;
};

ReorderSlot reorderBuffer[REORDER_SIZE];
volatile uint16_t reorderNextSeq = 0;
volatile uint16_t txSequence = 0;
volatile uint16_t rxSequence = 0;  // ← НОВЫЙ счетчик для RX
volatile uint32_t reorderLost = 0;

// ============================================
// JITTER BUFFER
// ============================================
#define PCM_JITTER_SIZE 1280
#define PCM_START_THRESHOLD 640

int16_t pcmJitter[PCM_JITTER_SIZE];
volatile uint16_t pcmWrite = 0;
volatile uint16_t pcmRead = 0;
volatile bool pcmStarted = false;
volatile uint32_t pcmUnderrun = 0;
volatile uint32_t pcmOverrun = 0;

// ============================================
// ПРОТОТИПЫ
// ============================================
void onDio1Interrupt(void); 
void rxPwmISR(void);
void handleVoiceTransmit(void);
void processIncomingPacket(void); 
void initHardwareAdcTimerDriven(void);
void processAdcBlock(AdcBlock* block);
void processTransmit(void);
void printDiag(void);

// ============================================
// JITTER FUNCTIONS
// ============================================
inline bool pcmPush(int16_t sample) {
    uint16_t next = pcmWrite + 1;
    if (next >= PCM_JITTER_SIZE) next = 0;
    if (next == pcmRead) {
        pcmOverrun++;
        return false;
    }
    pcmJitter[pcmWrite] = sample;
    pcmWrite = next;
    return true;
}

inline bool pcmPop(int16_t* sample) {
    if (pcmRead == pcmWrite) {
        pcmUnderrun++;
        return false;
    }
    *sample = pcmJitter[pcmRead];
    pcmRead++;
    if (pcmRead >= PCM_JITTER_SIZE) pcmRead = 0;
    return true;
}

inline uint16_t pcmAvailable() {
    if (pcmWrite >= pcmRead) return pcmWrite - pcmRead;
    return PCM_JITTER_SIZE - pcmRead + pcmWrite;
}

// ============================================
// RX QUEUE FUNCTIONS
// ============================================
bool rxQueuePush(uint8_t* data, uint8_t len, uint16_t seq) {
    if (rxCount >= RX_QUEUE_SIZE) {
        rxQueueLost++;
        return false;
    }
    memcpy(rxQueue[rxTail].data, data, len);
    rxQueue[rxTail].len = len;
    rxQueue[rxTail].sequence = seq;
    rxQueue[rxTail].timestamp = micros();
    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    rxCount++;
    return true;
}

bool rxQueuePop(uint8_t* data, uint8_t* len, uint16_t* seq) {
    if (rxCount == 0) return false;
    memcpy(data, rxQueue[rxHead].data, rxQueue[rxHead].len);
    *len = rxQueue[rxHead].len;
    *seq = rxQueue[rxHead].sequence;
    rxHead = (rxHead + 1) % RX_QUEUE_SIZE;
    rxCount--;
    return true;
}

// ============================================
// REORDER FUNCTIONS
// ============================================
bool reorderPush(uint8_t* data, uint8_t len, uint16_t seq) {
    uint16_t slot = seq & REORDER_MASK;
    if (reorderBuffer[slot].ready) {
        reorderLost++;
        return false;
    }
    memcpy(reorderBuffer[slot].data, data, len);
    reorderBuffer[slot].len = len;
    reorderBuffer[slot].ready = true;
    reorderBuffer[slot].timestamp = micros();
    return true;
}

bool reorderPop(uint8_t* data, uint8_t* len) {
    uint16_t slot = reorderNextSeq & REORDER_MASK;
    if (!reorderBuffer[slot].ready) return false;
    memcpy(data, reorderBuffer[slot].data, reorderBuffer[slot].len);
    *len = reorderBuffer[slot].len;
    reorderBuffer[slot].ready = false;
    reorderNextSeq++;
    return true;
}

// ============================================
// TX QUEUE FUNCTIONS
// ============================================
bool txQueuePush(uint8_t* data, uint8_t len) {
    if (txCount >= TX_QUEUE_SIZE) {
        txLost++;
        return false;
    }
    memcpy(txQueue[txTail].data, data, len);
    txQueue[txTail].len = len;
    txTail = (txTail + 1) % TX_QUEUE_SIZE;
    txCount++;
    return true;
}

bool txQueuePop(uint8_t* data, uint8_t* len) {
    if (txCount == 0) return false;
    memcpy(data, txQueue[txHead].data, txQueue[txHead].len);
    *len = txQueue[txHead].len;
    txHead = (txHead + 1) % TX_QUEUE_SIZE;
    txCount--;
    return true;
}

volatile bool radioActionDone = false; 
volatile bool radioActionInProgress = false; 

Module* radioModule = new Module(PIN_NSS, PIN_DIO1, PIN_NRST, PIN_BUSY);
LLCC68 radio = radioModule; 

bool radioReady = false;
bool isTransmitting = false;

float currentRssi = -120.0;
float lastPacketRssi = -120.0;
uint32_t rssiReadCount = 0;

volatile uint32_t framesEncoded = 0;
volatile uint32_t packetsSent = 0;
volatile uint32_t adcReadCount = 0;
uint32_t lastDebugPrint = 0;

volatile uint32_t rxPacketsReceived = 0;
volatile uint32_t rxPacketsFailed = 0;
volatile uint32_t lastPacketTime = 0;
volatile uint32_t rxIntervalMax = 0;
volatile uint32_t rxIntervalMin = 99999;
volatile uint32_t bufferEmptyEvents = 0;

// ============================================
// ДИАГНОСТИЧЕСКИЕ СЧЁТЧИКИ
// ============================================
volatile uint32_t txPacketCnt = 0;
volatile uint32_t txPacketLost = 0;
volatile uint32_t dmaBlockCnt = 0;
volatile uint32_t dmaIrqCount = 0;

HardwareTimer *rxPwmTimer8k = NULL;

// ============================================
// ВЕКТОР ПРЕРЫВАНИЯ DMA
// ============================================
extern "C" void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
}

// ============================================
// ISR — ТОЛЬКО ФЛАГ
// ============================================
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        if (isTransmitting) {
            adcBufferReady = true;
            dmaBlockCnt++;
        }
        dmaIrqCount++;
    }
}

// ============================================
// ПРЕРЫВАНИЕ DIO1
// ============================================
void onDio1Interrupt(void) {
    radioActionDone = true; 
}

// ============================================
// ШИМ ВЫВОД — JITTER BUFFER
// ============================================
void rxPwmISR(void) {
    if (!isTransmitting) {
        if (!pcmStarted && pcmAvailable() >= PCM_START_THRESHOLD) {
            pcmStarted = true;
            Serial.println("[PWM] JITTER STARTED!");
        }
        
        int16_t sample = 0;
        if (pcmStarted && pcmPop(&sample)) {
            // Нормальное воспроизведение
        } else {
            sample = 0;  // Тишина
        }
        
        int32_t v = (int32_t)sample + 32768;
        if (v < 0) v = 0;
        if (v > 65535) v = 65535;
        TIM1->CCR1 = ((uint32_t)v * (TIM1->ARR + 1)) >> 16;
    }
}

// ============================================
// ИНИЦИАЛИЗАЦИЯ АЦП
// ============================================
void initHardwareAdcTimerDriven() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_RCC_ADC1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE; 
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; 
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;       
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_9; 
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBlocks[0].data, ADC_BLOCK_SAMPLES);
    blockWriteIdx = 1;
    adcBufferReady = false;

    HardwareTimer *adcTimer = new HardwareTimer(TIM3);
    adcTimer->setOverflow(8000, HERTZ_FORMAT);
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(adcTimer->getHandle(), &sMasterConfig);
    adcTimer->resume();
}

// ============================================
// ОБРАБОТКА БЛОКА АЦП
// ============================================
void processAdcBlock(AdcBlock* block) {
    if (block == NULL) return;
    
    static int32_t dcOffset = 1800; 
    static int txFrameCounter = 0;
    
    for (int i = 0; i < ADC_BLOCK_SAMPLES; i++) {
        uint16_t rawAdc = block->data[i];
        
        dcOffset = (dcOffset * 15 + rawAdc) / 16; 

        int32_t sample32 = ((int32_t)rawAdc - dcOffset) * 12; 
        
        if (sample32 > 32767)  sample32 = 32767;
        if (sample32 < -32768) sample32 = -32768;
        
        audioBuffer[i] = (int16_t)sample32;
        adcReadCount++;
    }

    framesEncoded++;

    uint8_t* dest = txPacketBuffer + (txFrameCounter * bytesPerFrame);
    codec2_encode(c2, dest, audioBuffer);
    txFrameCounter++;

    if (txFrameCounter >= FRAMES_PER_PACKET) {
        txFrameCounter = 0;
        txQueuePush(txPacketBuffer, packetSize);
    }
}

// ============================================
// ПЕРЕДАЧА
// ============================================
void processTransmit() {
    if (txCount == 0) return;
    if (radioActionInProgress) return;
    if (!radioReady) return;
    
    uint8_t data[32];
    uint8_t len;
    if (txQueuePop(data, &len)) {
        radioActionInProgress = true;
        int state = radio.startTransmit(data, len);
        if (state == RADIOLIB_ERR_NONE) {
            packetsSent++;
            txPacketCnt++;
        } else {
            radioActionInProgress = false;
            txPacketLost++;
            if (txPacketLost % 5 == 0) {
                Serial.print("[TX_ERR] ");
                Serial.println(state);
            }
        }
    }
}

// ============================================
// ОБРАБОТКА ОЧЕРЕДИ БЛОКОВ
// ============================================
void handleVoiceTransmit() {
    if (adcBufferReady) {
        adcBufferReady = false;
        
        if (blocksAvailable >= NUM_ADC_BLOCKS) {
            blocksLost++;
            blocksAvailable = 0;
            blockReadIdx = blockWriteIdx;
        }
        
        uint8_t readyBlock = blockWriteIdx;
        blockWriteIdx = (blockWriteIdx + 1) % NUM_ADC_BLOCKS;
        
        adcBlocks[readyBlock].timestamp = micros();
        adcBlocks[readyBlock].sequence = blockSequence++;
        blocksAvailable++;
    }
    
    while (blocksAvailable > 0) {
        blocksAvailable--;
        AdcBlock* block = &adcBlocks[blockReadIdx];
        blockReadIdx = (blockReadIdx + 1) % NUM_ADC_BLOCKS;
        processAdcBlock(block);
    }
}

// ============================================
// ОБРАБОТКА RX ПАКЕТОВ (из очереди в reorder)
// ============================================
void processRxQueue() {
    uint8_t data[32];
    uint8_t len;
    uint16_t seq;
    
    while (rxCount > 0) {
        if (rxQueuePop(data, &len, &seq)) {
            bool success = reorderPush(data, len, seq);
            if (!success) {
                // Слот занят — потеря пакета
                static uint32_t reorderFull = 0;
                reorderFull++;
                if (reorderFull % 10 == 0) {
                    Serial.print("[REORDER] Slot busy, lost: ");
                    Serial.println(reorderFull);
                }
            }
        }
    }
}

// ============================================
// ОБРАБОТКА REORDER (в правильном порядке)
// ============================================
void processReorder() {
    uint8_t data[32];
    uint8_t len;
    static uint32_t decodedPackets = 0;
    static uint32_t lastLogTime = 0;
    uint32_t decodedThisCall = 0;
    
    while (reorderPop(data, &len)) {
        decodedThisCall++;
        decodedPackets++;
        
        // Декодируем все кадры в пакете
        for (int f = 0; f < FRAMES_PER_PACKET; f++) {
            uint8_t* framePtr = data + (f * bytesPerFrame);
            codec2_decode(c2, audioBuffer, framePtr);
            
            for (int i = 0; i < samplesPerFrame; i++) {
                pcmPush(audioBuffer[i]);
            }
        }
    }
    
    // Логирование каждую секунду
    if (millis() - lastLogTime > 1000) {
        lastLogTime = millis();
        if (decodedPackets > 0) {
            Serial.print("[REORDER] Decoded: ");
            Serial.print(decodedPackets);
            Serial.print(" total, JITTER: ");
            Serial.print(pcmAvailable());
            Serial.print("/");
            Serial.println(PCM_JITTER_SIZE);
        }
    }
}

// ============================================
// ПРИЕМ ПАКЕТОВ
// ============================================
void processIncomingPacket() {
    if (!radioActionDone) return;
    radioActionDone = false;
    
    int state = radio.readData(rxPacketBuffer, packetSize);
    
    if (state == RADIOLIB_ERR_NONE) {
        rxPacketsReceived++;
        lastPacketRssi = radio.getRSSI();
        
        // ✅ ИСПРАВЛЕНО: используем rxSequence для нумерации принятых пакетов
        rxQueuePush(rxPacketBuffer, packetSize, rxSequence);
        rxSequence++;  // ← Увеличиваем счетчик RX
        
        if (rxPacketsReceived % 10 == 0) {
            Serial.print("[RX] ");
            Serial.print(rxPacketsReceived);
            Serial.print(" | RSSI:");
            Serial.print(lastPacketRssi, 1);
            Serial.print(" | JITTER:");
            Serial.print(pcmAvailable());
            Serial.print("/");
            Serial.println(PCM_JITTER_SIZE);
        }

    } else {
        rxPacketsFailed++;
        if (rxPacketsFailed % 5 == 0) {
            Serial.print("[RX_ERR] ");
            Serial.println(state);
        }
    }
    
    setTxenRxen(false, true);
    radio.startReceive();
}

// ============================================
// ДИАГНОСТИКА
// ============================================
void printDiag() {
    Serial.println("\n=== DIAG ===");
    Serial.print("DMA blocks:"); Serial.print(dmaBlockCnt);
    Serial.print(" | Q blocks:"); Serial.print(blocksAvailable);
    Serial.print(" lost:"); Serial.print(blocksLost);
    Serial.print(" | TX:"); Serial.print(txPacketCnt);
    Serial.print(" lost:"); Serial.print(txPacketLost);
    Serial.print(" | Q:"); Serial.print(txCount);
    Serial.print("/"); Serial.println(TX_QUEUE_SIZE);
    
    Serial.print("RX:"); Serial.print(rxPacketsReceived);
    Serial.print(" err:"); Serial.print(rxPacketsFailed);
    Serial.print(" | RXseq:"); Serial.print(rxSequence);
    Serial.print(" | ReorderNext:"); Serial.print(reorderNextSeq);
    Serial.print(" | Lost:"); Serial.print(reorderLost);
    Serial.println();
    
    Serial.print("JITTER:"); Serial.print(pcmAvailable());
    Serial.print("/"); Serial.print(PCM_JITTER_SIZE);
    Serial.print(" | underrun:"); Serial.print(pcmUnderrun);
    Serial.print(" overrun:"); Serial.println(pcmOverrun);
    
    Serial.print("RX Queue:"); Serial.print(rxCount);
    Serial.print("/"); Serial.print(RX_QUEUE_SIZE);
    Serial.print(" lost:"); Serial.println(rxQueueLost);
    Serial.println("============");
}

// ============================================
// SETUP
// ============================================
void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);
    
    pinMode(PIN_PTT, INPUT_PULLUP);
    digitalWrite(PIN_LED, HIGH);

    initAudioCodec();
    initHardwareAdcTimerDriven();
    initRadioHardware();

    if (radioReady) {
        radio.setDio1Action(onDio1Interrupt);
        HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    }

    rxPwmTimer8k = new HardwareTimer(TIM2);
    rxPwmTimer8k->setOverflow(8000, HERTZ_FORMAT);
    rxPwmTimer8k->attachInterrupt(rxPwmISR);
    rxPwmTimer8k->resume();

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    // === ИНИЦИАЛИЗАЦИЯ ===
    blockWriteIdx = 1;
    blockReadIdx = 0;
    blocksAvailable = 0;
    blockSequence = 0;
    blocksLost = 0;
    adcBufferReady = false;
    
    pcmWrite = 0;
    pcmRead = 0;
    pcmStarted = false;
    pcmUnderrun = 0;
    pcmOverrun = 0;
    
    txHead = 0;
    txTail = 0;
    txCount = 0;
    txLost = 0;
    
    rxHead = 0;
    rxTail = 0;
    rxCount = 0;
    rxQueueLost = 0;
    txSequence = 0;
    rxSequence = 0;  // ← Инициализация нового счетчика
    reorderNextSeq = 0;
    
    for (int i = 0; i < REORDER_SIZE; i++) {
        reorderBuffer[i].ready = false;
    }
    
    Serial.println("\n=== SYSTEM READY ===");
    Serial.print("PCM_JITTER_SIZE: "); Serial.println(PCM_JITTER_SIZE);
    Serial.print("PCM_START_THRESHOLD: "); Serial.println(PCM_START_THRESHOLD);
    Serial.print("FRAMES_PER_PACKET: "); Serial.println(FRAMES_PER_PACKET);
    Serial.print("Packet size: "); Serial.println(packetSize);
}

// ============================================
// LOOP
// ============================================
void loop() {
    bool pttPressed = (digitalRead(PIN_PTT) == LOW);

    if (pttPressed && !isTransmitting) {
        isTransmitting = true;
        digitalWrite(PIN_LED, LOW); 
        
        framesEncoded = 0;
        packetsSent = 0;
        adcReadCount = 0;
        dmaIrqCount = 0;
        dmaBlockCnt = 0;
        blocksLost = 0;
        blocksAvailable = 0;
        blockSequence = 0;
        adcBufferReady = false;
        
        txHead = 0;
        txTail = 0;
        txCount = 0;
        txLost = 0;

        pcmWrite = 0;
        pcmRead = 0;
        pcmStarted = false;
        pcmUnderrun = 0;
        pcmOverrun = 0;

        rxHead = 0;
        rxTail = 0;
        rxCount = 0;
        rxQueueLost = 0;
        txSequence = 0;
        rxSequence = 0;      // ← Сброс счетчика RX
        reorderNextSeq = 0;
        reorderLost = 0;
        
        for (int i = 0; i < REORDER_SIZE; i++) {
            reorderBuffer[i].ready = false;
        }
        
        rxPacketsReceived = 0;
        rxPacketsFailed = 0;
        rxIntervalMax = 0;
        rxIntervalMin = 99999;
        bufferEmptyEvents = 0;

        radioActionDone = false; 
        radioActionInProgress = false; 

        setTxenRxen(true, false);
        delayMicroseconds(50);
        
        Serial.println("[MODE] TX START");
    } 
    else if (!pttPressed && isTransmitting) {
        isTransmitting = false;
        digitalWrite(PIN_LED, HIGH); 

        uint32_t waitStart = millis();
        while (txCount > 0 && (millis() - waitStart < 500)) {
            processTransmit();
            delayMicroseconds(100);
        }

        radio.standby();
        radio.finishTransmit();
        radioActionDone = false;
        radioActionInProgress = false;
        setTxenRxen(false, true);
        radio.startReceive();

        printDiag();
        Serial.println("[MODE] RX START");
    }

    // === ОБРАБОТКА ПРИЕМА ===
    if (radioActionDone) {
        processIncomingPacket();  // 1. Принимаем → в RX_QUEUE
    }
    
    processRxQueue();            // 2. RX_QUEUE → REORDER
    processReorder();            // 3. REORDER → JITTER BUFFER

    // === ОБРАБОТКА ПЕРЕДАЧИ ===
    if (isTransmitting) {
        handleVoiceTransmit();   // 4. ADC → CODEC2 → TX_QUEUE
        processTransmit();       // 5. TX_QUEUE → RADIO
    }
}








#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>
#include <HardwareTimer.h>
#include "codec2.h"
#include "stm32f4xx_hal.h"

// ==================== ПИНЫ АУДИО ====================
#define MIC_PIN         PB1
#define AUDIO_OUT_PIN   PA8

// 🔧 ВЫБЕРИТЕ РЕЖИМ: 700, 1300, 2400 или 3200
#define CODEC2_MODE CODEC2_MODE_1300

// ЖЕСТКАЯ ФИКСАЦИЯ: 2 кадра = 40 мс звука
#define FRAMES_PER_PACKET 2
// ==================== БУФЕРЫ ПЕРИФЕРИИ ====================
#define ADC_BUFFER_SIZE 640

// ==================== РАЗМЕРЫ КАДРОВ ДЛЯ CODEC2 ====================
#define CODEC2_BYTES_PER_FRAME_700   4   
#define CODEC2_BYTES_PER_FRAME_1300  7
#define CODEC2_BYTES_PER_FRAME_2400  12
#define CODEC2_BYTES_PER_FRAME_3200  16

inline int getCodec2BytesPerFrame(int mode) {
    switch(mode) {
        case CODEC2_MODE_700:  return CODEC2_BYTES_PER_FRAME_700; 
        case CODEC2_MODE_1300: return CODEC2_BYTES_PER_FRAME_1300;
        case CODEC2_MODE_2400: return CODEC2_BYTES_PER_FRAME_2400;
        case CODEC2_MODE_3200: return CODEC2_BYTES_PER_FRAME_3200;
        default: return 0;
    }
}

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (extern) ====================
extern HardwareTimer *pwmTimer;
extern struct CODEC2 *c2;
extern int samplesPerFrame;
extern int bytesPerFrame;
extern int packetSize;           

extern int16_t*  audioBuffer;    
extern uint8_t*  txPacketBuffer; 
extern uint8_t*  rxPacketBuffer; 

extern volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];
extern volatile int adcIndex;
extern volatile bool dmaReady;
extern volatile uint32_t dmaIrqCount;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// ==================== PWM ВЫВОД ЗВУКА ====================
inline void initAudioOut() {
    Serial.println(F("    [PWM] Start..."));
    pinMode(AUDIO_OUT_PIN, OUTPUT);
    pwmTimer = new HardwareTimer(TIM1);
    pwmTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, AUDIO_OUT_PIN);
    
    // ✅ ПРАВИЛЬНАЯ НАСТРОЙКА PWM
    // 200 кГц несущая, 16-битная точность
    pwmTimer->setOverflow(200000, HERTZ_FORMAT);
    pwmTimer->setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
    pwmTimer->resume();
    Serial.println(F("    [PWM] OK"));
}

inline void audioOutWrite(uint8_t pwmValue) {
    if (pwmTimer == NULL) return;
    if (pwmValue > 100) pwmValue = 100;
    pwmTimer->setCaptureCompare(1, pwmValue, PERCENT_COMPARE_FORMAT);
}

// ==================== ИНИЦИАЛИЗАЦИЯ КОДЕКА ====================
inline void initAudioCodec() {
    Serial.println(F("\n[initAudioCodec] START"));
    
    Serial.print(F("  Codec2 create..."));
    c2 = codec2_create(CODEC2_MODE);
    if (!c2) {
        Serial.println(F(" FAIL"));
        return;
    }
    Serial.println(F(" OK"));
    
    samplesPerFrame = codec2_samples_per_frame(c2);
    bytesPerFrame = getCodec2BytesPerFrame(CODEC2_MODE);
    packetSize = bytesPerFrame * FRAMES_PER_PACKET;

    Serial.print(F("    samplesPerFrame: ")); Serial.println(samplesPerFrame);
    Serial.print(F("    bytesPerFrame: ")); Serial.println(bytesPerFrame);
    Serial.print(F("    packetSize: ")); Serial.println(packetSize);
    Serial.print(F("    FRAMES_PER_PACKET: ")); Serial.println(FRAMES_PER_PACKET);

    Serial.print(F("  malloc buffers..."));
    audioBuffer = (int16_t*)malloc(samplesPerFrame * sizeof(int16_t));
    txPacketBuffer = (uint8_t*)malloc(packetSize);
    rxPacketBuffer = (uint8_t*)malloc(packetSize);
    if (audioBuffer && txPacketBuffer && rxPacketBuffer) {
        Serial.println(F(" OK"));
    } else {
        Serial.println(F(" FAIL"));
        return;
    }
    
    Serial.print(F("  Init PWM..."));
    initAudioOut();
    
    Serial.println(F("[initAudioCodec] END"));
}

#endif




#ifndef RADIO_INIT_H
#define RADIO_INIT_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <math.h>

#define PIN_NSS     PA4
#define PIN_SCK     PA5
#define PIN_MOSI    PA7
#define PIN_MISO    PA6
#define PIN_BUSY    PA1
#define PIN_DIO1    PB0
#define PIN_NRST    PA2
#define PIN_TXEN    PB10
#define PIN_RXEN    PA0
#define PIN_PTT     PA3
#define PIN_LED     PC13

// ==================== ВЫБОР СКОРОСТИ (x10) ====================
// Раскомментируй нужную строку:
// #define RADIO_BITRATE_KBPSx10  24   // 2.4 кбит/с
//#define RADIO_BITRATE_KBPSx10  48   // 4.8 кбит/с
// #define RADIO_BITRATE_KBPSx10  96   // 9.6 кбит/с
// #define RADIO_BITRATE_KBPSx10  192  // 19.2 кбит/с
 #define RADIO_BITRATE_KBPSx10  384  // 38.4 кбит/с
// #define RADIO_BITRATE_KBPSx10  500  // 50.0 кбит/с

// ==================== ПАРАМЕТРЫ ДЛЯ КАЖДОЙ СКОРОСТИ ====================
#if RADIO_BITRATE_KBPSx10 == 24
    #define RADIO_BITRATE_KBPS  2.4
    #define RADIO_DEVIATION_KHZ  2.4
    #define RADIO_RX_BANDWIDTH   9.7
    #define RADIO_PREAMBLE_LENGTH 16
    #define RADIO_SHAPING        RADIOLIB_SHAPING_NONE

#elif RADIO_BITRATE_KBPSx10 == 48
    #define RADIO_BITRATE_KBPS  4.8
    #define RADIO_DEVIATION_KHZ  4.8
    #define RADIO_RX_BANDWIDTH   14.6
    #define RADIO_PREAMBLE_LENGTH 16
    #define RADIO_SHAPING        RADIOLIB_SHAPING_0_5

#elif RADIO_BITRATE_KBPSx10 == 96
    #define RADIO_BITRATE_KBPS  9.6
    #define RADIO_DEVIATION_KHZ  4.8
    #define RADIO_RX_BANDWIDTH   19.5
    #define RADIO_PREAMBLE_LENGTH 16
    #define RADIO_SHAPING        RADIOLIB_SHAPING_0_5

#elif RADIO_BITRATE_KBPSx10 == 192
    #define RADIO_BITRATE_KBPS  19.2
    #define RADIO_DEVIATION_KHZ  9.6
    #define RADIO_RX_BANDWIDTH   39.0
    #define RADIO_PREAMBLE_LENGTH 16
    #define RADIO_SHAPING        RADIOLIB_SHAPING_0_5

#elif RADIO_BITRATE_KBPSx10 == 384
    #define RADIO_BITRATE_KBPS  38.4
    #define RADIO_DEVIATION_KHZ  19.2
    #define RADIO_RX_BANDWIDTH   78.2
    #define RADIO_PREAMBLE_LENGTH 16
    #define RADIO_SHAPING        RADIOLIB_SHAPING_0_5

#elif RADIO_BITRATE_KBPSx10 == 500
    #define RADIO_BITRATE_KBPS  50.0
    #define RADIO_DEVIATION_KHZ  25.0
    #define RADIO_RX_BANDWIDTH   156.2
    #define RADIO_PREAMBLE_LENGTH 16
    #define RADIO_SHAPING        RADIOLIB_SHAPING_0_5

#else
    #error "Unsupported RADIO_BITRATE_KBPSx10! Use 24, 48, 96, 192, 384 or 500"
#endif

// ==================== ОБЩИЕ ПАРАМЕТРЫ ====================
#define RADIO_FREQ_MHZ      433.900
#define RADIO_POWER_DBM     22

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
extern LLCC68 radio;
extern bool radioReady;
extern float currentRssi;
extern float lastPacketRssi;
extern uint32_t rssiReadCount;
extern int packetSize;

// ==================== АВТОМАТИЧЕСКИЙ РАСЧЕТ TX_WAIT ====================
#define RADIO_TX_WAIT_US(packetSize) ((((packetSize * 8) / RADIO_BITRATE_KBPS) * 1000) * 2)

// ==================== ФУНКЦИИ ====================
inline bool waitBusy(uint32_t timeout) {
    uint32_t start = millis();
    while (digitalRead(PIN_BUSY)) {
        if (millis() - start > timeout) return false;
        delayMicroseconds(1);
    }
    return true;
}

inline float readRSSI_SPI() {
    uint8_t buf;
    if (!waitBusy(100)) return currentRssi;
    
    digitalWrite(PIN_NSS, LOW);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(0x1D);
    buf = SPI.transfer(0x00);
    buf = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(PIN_NSS, HIGH);
    
    float rssi = -((float)buf) / 2.0;
    if (rssi < -130 || rssi > -10) return currentRssi;
    
    rssiReadCount++;
    if (rssiReadCount < 5) {
        currentRssi = rssi;
    } else {
        currentRssi = currentRssi * 0.7 + rssi * 0.3;
    }
    return currentRssi;
}

inline void setTxenRxen(bool txen, bool rxen) {
    digitalWrite(PIN_TXEN, txen ? HIGH : LOW);
    digitalWrite(PIN_RXEN, rxen ? HIGH : LOW);
    delayMicroseconds(50);
}

inline void initRadioHardware() {
    Serial.println(F("\n=== GFSK RADIO INIT (MANUAL CONFIG) ==="));
    Serial.print(F("  Bitrate: ")); Serial.print(RADIO_BITRATE_KBPS); Serial.println(F(" kbps"));
    Serial.print(F("  Deviation: ")); Serial.print(RADIO_DEVIATION_KHZ); Serial.println(F(" kHz"));
    Serial.print(F("  Bandwidth: ")); Serial.print(RADIO_RX_BANDWIDTH); Serial.println(F(" kHz"));
    Serial.print(F("  Preamble: ")); Serial.println(RADIO_PREAMBLE_LENGTH);
    Serial.print(F("  Shaping: ")); 
    #if RADIO_SHAPING == RADIOLIB_SHAPING_NONE
        Serial.println(F("NONE"));
    #else
        Serial.println(F("0.5"));
    #endif
    
    pinMode(PIN_TXEN, OUTPUT);
    pinMode(PIN_RXEN, OUTPUT);
    setTxenRxen(false, true);
    
    pinMode(PIN_NSS, OUTPUT);
    digitalWrite(PIN_NSS, HIGH);
    pinMode(PIN_BUSY, INPUT);
    pinMode(PIN_DIO1, INPUT);
    pinMode(PIN_NRST, OUTPUT);
    digitalWrite(PIN_NRST, HIGH);
    
    SPI.begin();
    Serial.println(F("[RADIO] SPI started"));
    
    int state = radio.beginFSK();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("[RADIO] ERROR: beginFSK failed: "));
        Serial.println(state);
        radioReady = false;
        return;
    }
    Serial.println(F("[RADIO] beginFSK OK"));
    
    radio.calibrateImage(RADIO_FREQ_MHZ);
    radio.setDio2AsRfSwitch(true);
    radio.setRegulatorDCDC();
    radio.setCurrentLimit(140.0);
    
    radio.setFrequency(RADIO_FREQ_MHZ);
    
    int s;
    s = radio.setBitRate(RADIO_BITRATE_KBPS);
    Serial.print(F("  setBitRate: ")); Serial.println(s);
    
    s = radio.setFrequencyDeviation(RADIO_DEVIATION_KHZ);
    Serial.print(F("  setDeviation: ")); Serial.println(s);
    
    s = radio.setRxBandwidth(RADIO_RX_BANDWIDTH);
    Serial.print(F("  setRxBandwidth: ")); Serial.println(s);
    
    s = radio.setDataShaping(RADIO_SHAPING);
    Serial.print(F("  setShaping: ")); Serial.println(s);
    
    s = radio.setPreambleLength(RADIO_PREAMBLE_LENGTH);
    Serial.print(F("  setPreamble: ")); Serial.println(s);
    
    uint8_t syncWord[] = {0x12, 0xAD, 0x2B};
    radio.setSyncWord(syncWord, sizeof(syncWord));
    radio.setOutputPower(RADIO_POWER_DBM);
    radio.setCRC(2);
    
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        radioReady = true;
        Serial.println(F("=== GFSK INIT SUCCESS ===\n"));
    } else {
        Serial.print(F("startReceive FAILED: "));
        Serial.println(state);
        radioReady = false;
    }
    
    delay(100);
    currentRssi = readRSSI_SPI();
}

#endif
