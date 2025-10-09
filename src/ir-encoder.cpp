#include "Arduino.h"
#include "ir-encoder.h"
// ================= IR SENSOR (N marks per revolution) =================
#define INNER_IR_PIN 4                     // D4 on Nano ESP32 (GPIO4)
#define OUTER_IR_PIN 16

#define OUTER_MARKS_PER_REV 8               // <<<--- set number of lines on drum here [TUNE]
#define INNER_MARKS_PER_REV 1               // <<<--- set number of lines on drum here [TUNE]
#define RPM_TIMEOUT_MS 3000           // if no edges for this long -> feedback invalid

// Adaptive debounce clamps
#define DEBOUNCE_MIN_US 10000         // lower bound [TUNE]
#define DEBOUNCE_MAX_US 120000        // upper bound [TUNE]
#define DEBOUNCE_FRACTION 10

IrSensor InnerSensor(DEBOUNCE_MAX_US, INNER_IR_PIN, INNER_MARKS_PER_REV);
IrSensor OuterSensor(DEBOUNCE_MAX_US, OUTER_IR_PIN, OUTER_MARKS_PER_REV);
Position FusedPosition;

void IRAM_ATTR onIrEdge(void* pvParameter) {
  uint32_t now = micros();

  IrSensor* sensor = static_cast<IrSensor*>(pvParameter);

  portENTER_CRITICAL_ISR(&sensor->lock);
  //not valid detection/missdetection
  if (!((uint32_t)(now - sensor->lastEdgeUs) > sensor->debounceUs)){
      portEXIT_CRITICAL_ISR(&sensor->lock);
      return;
  }
  //we need two edges to calculate period.
  if (sensor->prevEdgeUs != 0) {
      sensor->lastPeriodUs = now - sensor->prevEdgeUs;
  } else {
      sensor->lastPeriodUs = 0;
  }
  sensor->prevEdgeUs = now;
  sensor->lastEdgeUs = now;
  sensor->passCount++;
  sensor->newestEdgeProcessed = false;
  portEXIT_CRITICAL_ISR(&sensor->lock);

  portENTER_CRITICAL_ISR(&FusedPosition.lock);
  if (sensor == &InnerSensor){
      FusedPosition.haveSeenInner = true;
      FusedPosition.resetOnNext = true;
      portEXIT_CRITICAL_ISR(&FusedPosition.lock);
      //FusedPosition.outerEdgesSinceInner = 0;
      return;
  } 
  //OuterSensor
  if (sensor == &OuterSensor && FusedPosition.haveSeenInner == true){
      if (FusedPosition.resetOnNext){
        FusedPosition.outerEdgesSinceInner = 0;     
        FusedPosition.resetOnNext = false;
      } else{
      uint32_t outerEdgesSinceInner = FusedPosition.outerEdgesSinceInner + 1;
      //safety agaist double detections or missdetections, less cycles than using modulo.
      if (outerEdgesSinceInner >= (uint32_t)OuterSensor.marksPerRev) outerEdgesSinceInner = 0; //unsure whether to reset or keep at maximum
      FusedPosition.outerEdgesSinceInner = outerEdgesSinceInner;
      }
  }   
  portEXIT_CRITICAL_ISR(&FusedPosition.lock);
}

// Helper: print RPM without floats
static void printRpmLine(uint32_t pass, uint32_t periodUs, int marksPerRev) {
  if (periodUs == 0) {
    Serial.printf("[IR] pass=%lu  rpm=0.00  (period=%lu us)\n",
                  (unsigned long)pass, (unsigned long)periodUs);
    return;
  }

  // Adjust for multiple marks per revolution
  uint64_t rpm_x100_local = (6000000000ULL / (uint64_t)periodUs) / marksPerRev;

  uint32_t rpm_int  = (uint32_t)(rpm_x100_local / 100ULL);
  uint32_t rpm_frac = (uint32_t)(rpm_x100_local % 100ULL);
  Serial.printf("[IR] pass=%lu  rpm=%lu.%02lu  (period=%lu us)\n",
                (unsigned long)pass,
                (unsigned long)rpm_int,
                (unsigned long)rpm_frac,
                (unsigned long)periodUs);
}

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void printAngularPosition(){
    portENTER_CRITICAL(&FusedPosition.lock);
    int positionIndex = FusedPosition.outerEdgesSinceInner;
    bool validPosition = FusedPosition.haveSeenInner;
    portEXIT_CRITICAL(&FusedPosition.lock);

    if (validPosition){
        float angle = (positionIndex * 360)/OUTER_MARKS_PER_REV;
        Serial.printf("Angle = %.2f°\n", angle);
    } else {
        Serial.printf("Angle not defined,");
    }
}

void updateDebounceTime(IrSensor* sensor, int32_t lastPeriodUs) {
    uint32_t debounce_time = lastPeriodUs / DEBOUNCE_FRACTION; // 10% of period
    debounce_time = clamp_u32(debounce_time, DEBOUNCE_MIN_US, DEBOUNCE_MAX_US);
    portENTER_CRITICAL(&sensor->lock);
    sensor->debounceUs = debounce_time;
    portEXIT_CRITICAL(&sensor->lock);
};


enum class State{
    Idle,
    Active
};

void IrSensorTask(void* pvParameter){

    IrSensor* sensor = static_cast<IrSensor*>(pvParameter);
    int marksPerRev = sensor->marksPerRev;
    State IrSensorState = State::Idle;
    uint32_t last_seen = micros();

    uint32_t passCount, lastPeriodUs, lastEdgeUs;

    for (;;){

        portENTER_CRITICAL(&sensor->lock);
        bool newdata = !sensor->newestEdgeProcessed;
        if (newdata) {
            sensor->newestEdgeProcessed = true;
        }
        passCount = sensor->passCount;
        lastPeriodUs = sensor->lastPeriodUs;
        lastEdgeUs = sensor->lastEdgeUs;

        last_seen = lastEdgeUs;
        portEXIT_CRITICAL(&sensor->lock);
        bool timeout = false;

        switch(IrSensorState){
            case State::Idle:

                if (!newdata){
                    break;
                }

                //update debounce
                //we need two marks in order to get a period
                if (lastPeriodUs >0){
                    updateDebounceTime(sensor, lastPeriodUs);
                    //printRpmLine(passCount,lastPeriodUs, marksPerRev);
                    printAngularPosition();
            
                }

                IrSensorState = State::Active;
                break;

            case State::Active:

                if (newdata){
                    updateDebounceTime(sensor, lastPeriodUs);
//                    printRpmLine(passCount,lastPeriodUs, marksPerRev);
                    printAngularPosition();
                }

                timeout =  
                        ((uint32_t)(micros() - last_seen) > (uint32_t)RPM_TIMEOUT_MS * 1000UL);

                if (timeout) {
                    Serial.printf("Timeout!");

                    //reset stuff, not sure how much
                    portENTER_CRITICAL(&sensor->lock);
                    sensor->lastPeriodUs = 0;
                    sensor->lastEdgeUs = 0;
                    sensor->prevEdgeUs = 0;
                    sensor->debounceUs = DEBOUNCE_MAX_US;
                    portEXIT_CRITICAL(&sensor->lock);

                    IrSensorState = State::Idle;
                    break;
                }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

