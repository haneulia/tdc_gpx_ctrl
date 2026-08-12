#ifndef TEST_XAXIVDMA_H
#define TEST_XAXIVDMA_H

#include <stdint.h>

#define XAXIVDMA_MAX_FRAMESTORE 32U
#define XAXIVDMA_WRITE 1U
#define XAXIVDMA_IXR_FRMCNT_MASK 0x00001000U
#define XAXIVDMA_IXR_ERROR_MASK 0x00004000U
#define XAXIVDMA_IXR_COMPLETION_MASK 0x00003000U
#define XAXIVDMA_IXR_ALL_MASK 0x00007000U
#define XAXIVDMA_HANDLER_GENERAL 1U
#define XAXIVDMA_HANDLER_ERROR 2U
#define XAXIVDMA_ENABLE_DBG_FRM_CNTR 0x04U
#define XAXIVDMA_ENABLE_DBG_DLY_CNTR 0x08U

typedef uintptr_t UINTPTR;

typedef struct {
    uintptr_t ChanBase;
    int IsValid;
    unsigned int NumFrames;
    uint32_t DbgFeatureFlags;
} XAxiVdma_Channel;

typedef struct {
    int HasMm2S;
    int HasS2Mm;
    XAxiVdma_Channel WriteChannel;
} XAxiVdma;

typedef struct {
    uint8_t ReadFrameCount;
    uint8_t ReadDelayTimerCount;
    uint8_t WriteFrameCount;
    uint8_t WriteDelayTimerCount;
} XAxiVdma_FrameCounter;

int XAxiVdma_SetFrameCounter(
    XAxiVdma *instance,
    XAxiVdma_FrameCounter *frame_counter);

int XAxiVdma_SetCallBack(
    XAxiVdma *instance,
    uint32_t handler_type,
    void *callback,
    void *callback_reference,
    uint16_t direction);

void XAxiVdma_IntrDisable(
    XAxiVdma *instance,
    uint32_t interrupt_types,
    uint16_t direction);
void XAxiVdma_IntrEnable(
    XAxiVdma *instance,
    uint32_t interrupt_types,
    uint16_t direction);
void XAxiVdma_DmaStop(XAxiVdma *instance, uint16_t direction);
int XAxiVdma_DmaStart(XAxiVdma *instance, uint16_t direction);
int XAxiVdma_IsBusy(XAxiVdma *instance, uint16_t direction);
uint32_t XAxiVdma_CurrFrameStore(XAxiVdma *instance, uint16_t direction);
uint32_t XAxiVdma_IntrGetPending(XAxiVdma *instance, uint16_t direction);
void XAxiVdma_IntrClear(
    XAxiVdma *instance,
    uint32_t interrupt_types,
    uint16_t direction);

#endif
