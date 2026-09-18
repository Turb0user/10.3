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
int radioPacketSize = 0;   // ✅ НОВОЕ: размер радиопакета

int16_t*  audioBuffer = NULL;    
uint8_t*  txPacketBuffer = NULL; 
uint8_t*  rxPacketBuffer = NULL; 

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

// ============================================
// ADC DMA DOUBLE BUFFER
// ============================================
#define ADC_BLOCK_SAMPLES 160

int16_t adcDmaBufferA[ADC_BLOCK_SAMPLES];
int16_t adcDmaBufferB[ADC_BLOCK_SAMPLES];

volatile uint8_t currentDmaBuffer = 0;
volatile uint8_t readyBuffer = 0xFF;

// ============================================
// MULTI-BUFFER BLOCK QUEUE
// ============================================
#define NUM_ADC_BLOCKS 3

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
// JITTER BUFFER
// ============================================
#define PCM_JITTER_SIZE 4096
#define PCM_START_THRESHOLD 512

int16_t pcmJitter[PCM_JITTER_SIZE];
volatile uint16_t pcmWrite = 0;
volatile uint16_t pcmRead = 0;
volatile bool pcmStarted = false;
volatile uint32_t pcmUnderrun = 0;
volatile uint32_t pcmOverrun = 0;

// ============================================
// PLC (Packet Loss Concealment)
// ============================================
#define PLC_FRAME_SIZE 160
static int16_t lastFrame[PLC_FRAME_SIZE];
static float plcFade = 1.0;
static bool lastFrameValid = false;

// ============================================
// ПРОТОТИПЫ
// ============================================
void onDio1Interrupt(void); 
void rxPwmISR(void);
void handleVoiceTransmit(void);
void processIncomingPacket(void); 
void initHardwareAdcTimerDriven(void);
void processAdcBlock(int16_t* data, uint32_t size);
void processTransmit(void);
void printDiag(void);
void initPwmAudio(void);
void plcGenerate(void);
void handleDmaReadyBlocks(void);

// ============================================
// JITTER FUNCTIONS
// ============================================
inline bool pcmPush(int16_t sample) {
    uint16_t next = pcmWrite + 1;
    if (next >= PCM_JITTER_SIZE) next = 0;
    if (next == pcmRead) {
        pcmOverrun++;
        pcmRead = (pcmRead + 1) % PCM_JITTER_SIZE;
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
// PLC FUNCTIONS
// ============================================
void plcGenerate() {
    if (!lastFrameValid) {
        for (int i = 0; i < PLC_FRAME_SIZE; i++) {
            pcmPush(0);
        }
        return;
    }
    plcFade *= 0.85;
    if (plcFade < 0.1) plcFade = 0.1;
    for (int i = 0; i < PLC_FRAME_SIZE; i++) {
        int32_t sample = (int32_t)lastFrame[i] * plcFade;
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        pcmPush((int16_t)sample);
    }
}

void plcSaveFrame(int16_t* data, uint32_t size) {
    if (data == NULL || size > PLC_FRAME_SIZE) return;
    memcpy(lastFrame, data, size * sizeof(int16_t));
    lastFrameValid = true;
    plcFade = 1.0;
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

// ============================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================
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

volatile uint32_t rxPacketsReceived = 0;
volatile uint32_t rxPacketsFailed = 0;

volatile uint32_t txPacketCnt = 0;
volatile uint32_t txPacketLost = 0;
volatile uint32_t dmaBlockCnt = 0;
volatile uint32_t dmaIrqCount = 0;

HardwareTimer *rxPwmTimer8k = NULL;

volatile uint32_t txQueueFullCount = 0;
volatile uint32_t txStartTransmitCalls = 0;
volatile uint32_t txStartTransmitSuccess = 0;
volatile uint32_t txStartTransmitFail = 0;
volatile uint32_t txDio1Interrupts = 0;
volatile uint32_t txTimeoutResets = 0;

volatile uint16_t txSequence = 0;   // ✅ НОВОЕ: счетчик sequence для TX

// ============================================
// DMA ISR
// ============================================
extern "C" void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        if (currentDmaBuffer == 0) {
            readyBuffer = 0;
            HAL_ADC_Stop_DMA(hadc);
            HAL_ADC_Start_DMA(hadc, (uint32_t*)adcDmaBufferB, ADC_BLOCK_SAMPLES);
            currentDmaBuffer = 1;
        } else {
            readyBuffer = 1;
            HAL_ADC_Stop_DMA(hadc);
            HAL_ADC_Start_DMA(hadc, (uint32_t*)adcDmaBufferA, ADC_BLOCK_SAMPLES);
            currentDmaBuffer = 0;
        }
        dmaBlockCnt++;
        dmaIrqCount++;
    }
}

// ============================================
// DIO1 ISR
// ============================================
void onDio1Interrupt(void) {
    radioActionDone = true;
    radioActionInProgress = false;
    txDio1Interrupts++;
}

// ============================================
// PWM ISR
// ============================================
void rxPwmISR(void) {
    TIM2->SR &= ~TIM_SR_UIF;
    
    uint16_t avail = pcmAvailable();
    
    if (!pcmStarted && avail >= PCM_START_THRESHOLD) {
        pcmStarted = true;
    }
    
    // ✅ СБРОС ПРИ ОПУСТОШЕНИИ
    if (pcmStarted && avail == 0) {
        pcmStarted = false;
    }
    
    int16_t sample = 0;
    if (pcmStarted && pcmPop(&sample)) {
        // OK
    } else {
        sample = 0;
    }
    
    uint32_t pwmValue = ((uint32_t)(sample + 32768) * 4095) >> 16;
    TIM1->CCR1 = pwmValue;
}

// ============================================
// ADC INIT
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

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDmaBufferA, ADC_BLOCK_SAMPLES);
    currentDmaBuffer = 0;
    readyBuffer = 0xFF;

    HardwareTimer *adcTimer = new HardwareTimer(TIM3);
    adcTimer->setOverflow(8000, HERTZ_FORMAT);
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(adcTimer->getHandle(), &sMasterConfig);
    adcTimer->resume();
}

// ============================================
// ОБРАБОТКА БЛОКА АЦП — С ДОБАВЛЕНИЕМ SEQUENCE
// ============================================
void processAdcBlock(int16_t* data, uint32_t size) {
    if (data == NULL) return;
    
    static int32_t dcOffset = 1800; 
    static int txFrameCounter = 0;
    
    for (uint32_t i = 0; i < size; i++) {
        uint16_t rawAdc = data[i];
        dcOffset = (dcOffset * 15 + rawAdc) / 16; 
        int32_t sample32 = ((int32_t)rawAdc - dcOffset) * 12; 
        if (sample32 > 32767)  sample32 = 32767;
        if (sample32 < -32768) sample32 = -32768;
        audioBuffer[i] = (int16_t)sample32;
        adcReadCount++;
    }

    framesEncoded++;
    
    // ✅ КОДИРУЕМ ПОСЛЕ 2-БАЙТОВОГО ЗАГОЛОВКА
    uint8_t* dest = txPacketBuffer + PACKET_HEADER_SIZE +
                    (txFrameCounter * bytesPerFrame);
    codec2_encode(c2, dest, audioBuffer);
    txFrameCounter++;

    if (txFrameCounter >= FRAMES_PER_PACKET) {
        txFrameCounter = 0;
        
        // ✅ ДОБАВЛЯЕМ SEQUENCE В НАЧАЛО ПАКЕТА
        txPacketBuffer[0] = txSequence & 0xFF;
        txPacketBuffer[1] = (txSequence >> 8) & 0xFF;
        txSequence++;
        
        if (!txQueuePush(txPacketBuffer, radioPacketSize)) {
            txQueueFullCount++;
        }
    }
}

// ============================================
// DMA READY BLOCKS
// ============================================
void handleDmaReadyBlocks() {
    if (readyBuffer != 0xFF) {
        int16_t* bufferData;
        if (readyBuffer == 0) {
            bufferData = adcDmaBufferA;
        } else {
            bufferData = adcDmaBufferB;
        }
        
        AdcBlock* block = &adcBlocks[blockWriteIdx];
        memcpy(block->data, bufferData, ADC_BLOCK_SAMPLES * sizeof(int16_t));
        block->timestamp = micros();
        block->sequence = blockSequence++;
        
        blockWriteIdx = (blockWriteIdx + 1) % NUM_ADC_BLOCKS;
        blocksAvailable++;
        
        if (blocksAvailable >= NUM_ADC_BLOCKS) {
            blocksLost++;
            blockReadIdx = (blockReadIdx + 1) % NUM_ADC_BLOCKS;
            blocksAvailable--;
        }
        readyBuffer = 0xFF;
    }
}

// ============================================
// ПЕРЕДАЧА
// ============================================
void processTransmit() {
    static uint32_t timeoutStart = 0;
    
    if (txCount == 0) {
        timeoutStart = 0;
        return;
    }
    
    if (radioActionInProgress) {
        if (timeoutStart == 0) timeoutStart = millis();
        if (millis() - timeoutStart > 300) {
            Serial.println("[TX_ERR] Timeout! Resetting...");
            radioActionInProgress = false;
            timeoutStart = 0;
            txTimeoutResets++;
            radio.standby();
        }
        return;
    }
    
    timeoutStart = 0;
    if (!radioReady) return;
    
    uint8_t data[32];
    uint8_t len;
    if (txQueuePop(data, &len)) {
        radioActionInProgress = true;
        txStartTransmitCalls++;
        
        int state = radio.startTransmit(data, len);
        
        if (state == RADIOLIB_ERR_NONE) {
            packetsSent++;
            txPacketCnt++;
            txStartTransmitSuccess++;
            timeoutStart = 0;
        } else {
            radioActionInProgress = false;
            txPacketLost++;
            txStartTransmitFail++;
            timeoutStart = 0;
        }
    }
}

// ============================================
// ГОЛОС ПРИ ПЕРЕДАЧЕ
// ============================================
void handleVoiceTransmit() {
    while (blocksAvailable > 0) {
        blocksAvailable--;
        AdcBlock* block = &adcBlocks[blockReadIdx];
        blockReadIdx = (blockReadIdx + 1) % NUM_ADC_BLOCKS;
        processAdcBlock(block->data, ADC_BLOCK_SAMPLES);
    }
}

// ============================================
// ПРИЕМ ПАКЕТОВ — ЧИТАЕМ SEQUENCE, ДЕКОДИРУЕМ АУДИО
// ============================================
void processIncomingPacket() {
    if (!radioActionDone) return;
    
    if (radioActionInProgress) {
        radioActionInProgress = false;
    }
    
    if (isTransmitting) {
        radioActionDone = false;
        return;
    }
    
    radioActionDone = false;
    
    // ✅ ЧИТАЕМ РАДИОПАКЕТ (16 БАЙТ)
    int state = radio.readData(rxPacketBuffer, radioPacketSize);
    
    if (state == RADIOLIB_ERR_NONE) {
        rxPacketsReceived++;
        lastPacketRssi = radio.getRSSI();
        
        // ✅ ЧИТАЕМ SEQUENCE
        uint16_t seq = rxPacketBuffer[0] | (rxPacketBuffer[1] << 8);
        
        static uint32_t lastDebug = 0;
        if (millis() - lastDebug > 2000) {
            lastDebug = millis();
            Serial.print("[RX_DBG] Seq=");
            Serial.println(seq);
        }
        
        // ✅ ДЕКОДИРУЕМ ТОЛЬКО АУДИОДАННЫЕ (ПОСЛЕ 2-БАЙТОВОГО ЗАГОЛОВКА)
        uint8_t* audioData = rxPacketBuffer + PACKET_HEADER_SIZE;
        
        for (int f = 0; f < FRAMES_PER_PACKET; f++) {
            uint8_t* framePtr = audioData + (f * bytesPerFrame);
            codec2_decode(c2, audioBuffer, framePtr);
            
            if (f == 0) {
                plcSaveFrame(audioBuffer, samplesPerFrame);
            }
            
            for (int i = 0; i < samplesPerFrame; i++) {
                pcmPush(audioBuffer[i]);
            }
        }
        
        if (rxPacketsReceived % 10 == 0) {
            Serial.print("[RX] ");
            Serial.print(rxPacketsReceived);
            Serial.print(" RSSI:");
            Serial.print(lastPacketRssi, 1);
            Serial.print(" JITTER:");
            Serial.println(pcmAvailable());
        }
    } else {
        rxPacketsFailed++;
        if (rxPacketsFailed % 10 == 0) {
            Serial.print("[RX_ERR] ");
            Serial.println(state);
        }
    }
    
    setTxenRxen(false, true);
    radio.startReceive();
}

// ============================================
// PWM INIT — ИСПРАВЛЕН ARR = 4095
// ============================================
void initPwmAudio() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = GPIO_PIN_8;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    gpioInit.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    
    TIM_HandleTypeDef htim1 = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4095;   // ✅ БЫЛО 419 — ИСПРАВЛЕНО!
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    TIM_OC_InitTypeDef ocConfig = {0};
    ocConfig.OCMode = TIM_OCMODE_PWM1;
    ocConfig.Pulse = 2048;    // ✅ ЦЕНТР (тишина)
    ocConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    ocConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    ocConfig.OCFastMode = TIM_OCFAST_DISABLE;
    ocConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
    ocConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &ocConfig, TIM_CHANNEL_1);
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

// ============================================
// ДИАГНОСТИКА — ПРОСТОЙ ФОРМАТ
// ============================================
void printDiag() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < 1000) return;
    lastPrint = millis();
    
    Serial.print("DMA:");
    Serial.print(dmaBlockCnt);
    Serial.print(" Blk:");
    Serial.print(blocksAvailable);
    Serial.print("/");
    Serial.print(NUM_ADC_BLOCKS);
    Serial.print(" lost:");
    Serial.print(blocksLost);
    
    Serial.print(" | TX:");
    Serial.print(txPacketCnt);
    Serial.print(" Q:");
    Serial.print(txCount);
    Serial.print("/");
    Serial.print(TX_QUEUE_SIZE);
    
    Serial.print(" | TXcalls:");
    Serial.print(txStartTransmitCalls);
    Serial.print(" OK:");
    Serial.print(txStartTransmitSuccess);
    Serial.print(" Fail:");
    Serial.print(txStartTransmitFail);
    Serial.print(" TO:");
    Serial.print(txTimeoutResets);
    
    Serial.print(" | RX:");
    Serial.print(rxPacketsReceived);
    Serial.print(" err:");
    Serial.print(rxPacketsFailed);
    Serial.print(" DIO1:");
    Serial.print(txDio1Interrupts);
    
    Serial.print(" | JIT:");
    Serial.print(pcmAvailable());
    Serial.print("/");
    Serial.print(PCM_JITTER_SIZE);
    Serial.print(" ur:");
    Serial.print(pcmUnderrun);
    Serial.print(" or:");
    Serial.print(pcmOverrun);
    
    Serial.print(" | ");
    Serial.print(isTransmitting ? "TX" : "RX");
    Serial.print(" radio:");
    Serial.print(radioReady ? "Y" : "N");
    Serial.print(" pcm:");
    Serial.println(pcmStarted ? "Y" : "N");
}

// ============================================
// SETUP
// ============================================
void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);
    
    pinMode(PIN_PTT, INPUT_PULLUP);
    digitalWrite(PIN_LED, HIGH);

    Serial.println("\n=== SYSTEM INIT ===");

    initAudioCodec();
    initPwmAudio();           // ✅ ТОЛЬКО ЗДЕСЬ
    initHardwareAdcTimerDriven();
    initRadioHardware();

    if (radioReady) {
        radio.setDio1Action(onDio1Interrupt);
        HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        Serial.println("[INIT] Radio OK");
    } else {
        Serial.println("[INIT] Radio FAIL");
    }

    rxPwmTimer8k = new HardwareTimer(TIM2);
    rxPwmTimer8k->setOverflow(8000, HERTZ_FORMAT);
    rxPwmTimer8k->attachInterrupt(rxPwmISR);
    rxPwmTimer8k->resume();

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    pcmWrite = 0;
    pcmRead = 0;
    pcmStarted = false;
    pcmUnderrun = 0;
    pcmOverrun = 0;
    
    txHead = 0;
    txTail = 0;
    txCount = 0;
    txLost = 0;
    
    txSequence = 0;   // ✅ ИНИЦИАЛИЗАЦИЯ SEQUENCE
    
    currentDmaBuffer = 0;
    readyBuffer = 0xFF;
    dmaBlockCnt = 0;
    blocksAvailable = 0;
    blockWriteIdx = 0;
    blockReadIdx = 0;
    
    lastFrameValid = false;
    plcFade = 1.0;
    
    txQueueFullCount = 0;
    txStartTransmitCalls = 0;
    txStartTransmitSuccess = 0;
    txStartTransmitFail = 0;
    txDio1Interrupts = 0;
    txTimeoutResets = 0;
    
    Serial.println("=== READY ===");
    Serial.print("ADC:");
    Serial.print(ADC_BLOCK_SAMPLES);
    Serial.print(" JIT:");
    Serial.print(PCM_JITTER_SIZE);
    Serial.print(" START:");
    Serial.print(PCM_START_THRESHOLD);
    Serial.print(" TXQ:");
    Serial.print(TX_QUEUE_SIZE);
    Serial.print(" radioPkt:");
    Serial.println(radioPacketSize);
    Serial.println("PTT -> TX, release -> RX");
    Serial.println();
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
        txPacketCnt = 0;
        txPacketLost = 0;
        
        txHead = 0;
        txTail = 0;
        txCount = 0;
        txLost = 0;
        txQueueFullCount = 0;
        txStartTransmitCalls = 0;
        txStartTransmitSuccess = 0;
        txStartTransmitFail = 0;
        txDio1Interrupts = 0;
        txTimeoutResets = 0;
        
        txSequence = 0;   // ✅ СБРОС SEQUENCE
        
        rxPacketsReceived = 0;
        rxPacketsFailed = 0;

        radioActionDone = false; 
        radioActionInProgress = false; 

        setTxenRxen(true, false);
        delayMicroseconds(50);
        
        Serial.println("\n[MODE] TX START");
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

        Serial.println("[MODE] RX START");
    }

    handleDmaReadyBlocks();

    if (radioActionDone) {
        processIncomingPacket();
    }

    if (isTransmitting) {
        handleVoiceTransmit();
        processTransmit();
    }
    
    printDiag();
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

// ✅ РАЗМЕР ЗАГОЛОВКА ПАКЕТА (2 байта sequence)
#define PACKET_HEADER_SIZE 2

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
extern int packetSize;           // Размер аудиоданных (14 байт)
extern int radioPacketSize;      // Размер радиопакета (16 байт = 2 + 14)

extern int16_t*  audioBuffer;    
extern uint8_t*  txPacketBuffer; 
extern uint8_t*  rxPacketBuffer; 

extern volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];
extern volatile int adcIndex;
extern volatile bool dmaReady;
extern volatile uint32_t dmaIrqCount;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// ==================== ИНИЦИАЛИЗАЦИЯ КОДЕКА ====================
// ✅ УБРАНА инициализация PWM! Только Codec2 и буферы.
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
    radioPacketSize = packetSize + PACKET_HEADER_SIZE;

    Serial.print(F("    samplesPerFrame: ")); Serial.println(samplesPerFrame);
    Serial.print(F("    bytesPerFrame: ")); Serial.println(bytesPerFrame);
    Serial.print(F("    packetSize: ")); Serial.println(packetSize);
    Serial.print(F("    radioPacketSize: ")); Serial.println(radioPacketSize);
    Serial.print(F("    FRAMES_PER_PACKET: ")); Serial.println(FRAMES_PER_PACKET);

    Serial.print(F("  malloc buffers..."));
    audioBuffer = (int16_t*)malloc(samplesPerFrame * sizeof(int16_t));
    txPacketBuffer = (uint8_t*)malloc(radioPacketSize);
    rxPacketBuffer = (uint8_t*)malloc(radioPacketSize);
    if (audioBuffer && txPacketBuffer && rxPacketBuffer) {
        Serial.println(F(" OK"));
    } else {
        Serial.println(F(" FAIL"));
        return;
    }
    
    // ✅ НЕ ВЫЗЫВАЕМ initAudioOut() ЗДЕСЬ!
    // PWM инициализируется только в initPwmAudio() (в main.cpp)
    
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
#define RADIO_BITRATE_KBPSx10  96   // 9.6 кбит/с
// #define RADIO_BITRATE_KBPSx10  192  // 19.2 кбит/с
// #define RADIO_BITRATE_KBPSx10  384  // 38.4 кбит/с
//  #define RADIO_BITRATE_KBPSx10  500  // 50.0 кбит/с

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
