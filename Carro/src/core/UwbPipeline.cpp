#include "UwbPipeline.h"
#include "Log.h"

namespace {
  QueueHandle_t q = nullptr; // size 1 queue
  volatile uint32_t produced = 0;
  volatile uint32_t consumed = 0;
  volatile uint32_t lastSeq = 0;
  volatile uint32_t skippedSeq = 0;
  volatile uint32_t worstLatencyUs = 0;
  volatile uint32_t lastLatencyUs = 0;
  volatile uint32_t latencySamples = 0;
  volatile uint64_t latencyAccumUs = 0;
}

namespace UwbPipeline {
  bool init(){
    if(q) return true;
    q = xQueueCreate(1, sizeof(UwbMeasurement));
    if(!q){ LOGE("UWBQ","No se pudo crear cola"); return false; }
    produced=consumed=lastSeq=skippedSeq=worstLatencyUs=lastLatencyUs=latencySamples=latencyAccumUs=0;
    return true;
  }
  bool publish(const UwbMeasurement& m){ if(!q) return false; xQueueOverwrite(q,&m); produced++; lastSeq=m.seq; return true; }
  bool wait(UwbMeasurement& m, uint32_t timeoutMs){
    if(!q) return false;
    if(xQueueReceive(q,&m, timeoutMs==UINT32_MAX?portMAX_DELAY:pdMS_TO_TICKS(timeoutMs))==pdTRUE){
      consumed++;
      static uint32_t prev=0; if(prev && m.seq!=prev+1) skippedSeq++; prev=m.seq; return true; }
    return false;
  }
  void recordControlLatency(uint32_t us){ lastLatencyUs=us; if(us>worstLatencyUs) worstLatencyUs=us; latencySamples++; latencyAccumUs+=us; }
  void dumpStats(){
    float avg = latencySamples? (float)latencyAccumUs/latencySamples:0.0f;
    LOGI("UWBQ","PIPE stats prod=%u cons=%u last=%u skip=%u lastLat=%u worst=%u avg=%.1f", produced, consumed, lastSeq, skippedSeq, lastLatencyUs, worstLatencyUs, avg);
  }
  void resetStats(){ produced=consumed=skippedSeq=worstLatencyUs=lastLatencyUs=latencySamples=latencyAccumUs=0; }
}
