#include <Arduino.h>
#include <SPI.h>

// -------------------- Start of AD9833 Class Code --------------------

//#define FNC_PIN 4 // Define if you want to use digitalWriteFast (not required)

// If using digitalWriteFast, include it here
#ifdef FNC_PIN
  #include "digitalWriteFast.h"
  #define WRITE_FNCPIN(Val) digitalWriteFast2(FNC_PIN,(Val))
#else
  #define WRITE_FNCPIN(Val) digitalWrite(FNCpin,(Val))
#endif

#define pow2_28        268435456L   // 2^28 used in frequency word calculation
#define BITS_PER_DEG   11.3777777777778  // 4096 / 360

// Control register bit masks
#define RESET_CMD       0x0100
#define SLEEP_MODE      0x00C0
#define DISABLE_DAC     0x0040
#define DISABLE_INT_CLK 0x0080

// Phase and freq write commands
#define PHASE_WRITE_CMD   0xC000
#define PHASE1_WRITE_REG  0x2000
#define FREQ0_WRITE_REG   0x4000
#define FREQ1_WRITE_REG   0x8000
#define PHASE1_OUTPUT_REG 0x0400
#define FREQ1_OUTPUT_REG  0x0800

typedef enum { SINE_WAVE = 0x2000, TRIANGLE_WAVE = 0x2002,
               SQUARE_WAVE = 0x2028, HALF_SQUARE_WAVE = 0x2020 } WaveformType;

typedef enum { REG0, REG1, SAME_AS_REG0 } Registers;

class AD9833 {
public:
  AD9833 (uint8_t FNCpin, uint32_t referenceFrequency = 25000000UL);

  void Begin(void);
  void ApplySignal (WaveformType waveType, Registers freqReg,
                    float frequencyInHz,
                    Registers phaseReg = SAME_AS_REG0, float phaseInDeg = 0.0);

  void Reset(void);
  void SetFrequency (Registers freqReg, float frequency);
  void IncrementFrequency (Registers freqReg, float freqIncHz);
  void SetPhase (Registers phaseReg, float phaseInDeg);
  void IncrementPhase (Registers phaseReg, float phaseIncDeg);
  void SetWaveform (Registers waveFormReg, WaveformType waveType);
  void SetOutputSource (Registers freqReg, Registers phaseReg = SAME_AS_REG0);
  void EnableOutput (bool enable);
  void SleepMode (bool enable);
  void DisableDAC (bool enable);
  void DisableInternalClock (bool enable);
  float GetActualProgrammedFrequency (Registers reg);
  float GetActualProgrammedPhase (Registers reg);
  float GetResolution (void);

private:
  void WriteRegister (int16_t dat);
  void WriteControlRegister (void);

  uint16_t waveForm0, waveForm1;
#ifndef FNC_PIN
  uint8_t FNCpin;
#endif
  uint8_t outputEnabled, DacDisabled, IntClkDisabled;
  uint32_t refFrequency;
  float frequency0, frequency1, phase0, phase1;
  Registers activeFreq, activePhase;
};

// -------------------- AD9833 Method Implementations --------------------

AD9833::AD9833 (uint8_t FNCpin, uint32_t referenceFrequency) {
#ifdef FNC_PIN
  pinMode(FNC_PIN,OUTPUT);
#else
  this->FNCpin = FNCpin;
  pinMode(FNCpin,OUTPUT);
#endif
  WRITE_FNCPIN(HIGH);

  refFrequency = referenceFrequency;

  DacDisabled = false;
  IntClkDisabled = false;
  outputEnabled = false;
  waveForm0 = waveForm1 = SINE_WAVE;
  frequency0 = frequency1 = 1000.0;
  phase0 = phase1 = 0.0;
  activeFreq = REG0; activePhase = REG0;
}

void AD9833::Begin(void) {
  SPI.begin();
  delay(100);
  Reset();
}

void AD9833::ApplySignal (WaveformType waveType, Registers freqReg,
                          float frequencyInHz,
                          Registers phaseReg, float phaseInDeg) {
  SetFrequency(freqReg, frequencyInHz);
  SetPhase(phaseReg, phaseInDeg);
  SetWaveform(freqReg, waveType);
  SetOutputSource(freqReg, phaseReg);
}

void AD9833::Reset(void) {
  WriteRegister(RESET_CMD);
  delay(15);
}

void AD9833::SetFrequency (Registers freqReg, float frequency) {
  if (frequency > 12.5e6)
    frequency = 12.5e6;
  if (frequency < 0.0) frequency = 0.0;

  if (freqReg == REG0) frequency0 = frequency;
  else frequency1 = frequency;

  int32_t freqWord = (int32_t)((frequency * pow2_28) / (float)refFrequency);
  int16_t upper14 = (int16_t)((freqWord & 0xFFFC000) >> 14),
          lower14 = (int16_t)(freqWord & 0x3FFF);

  uint16_t reg = freqReg == REG0 ? FREQ0_WRITE_REG : FREQ1_WRITE_REG;
  lower14 |= reg;
  upper14 |= reg;

  WriteControlRegister();
  WriteRegister(lower14);
  WriteRegister(upper14);
}

void AD9833::IncrementFrequency (Registers freqReg, float freqIncHz) {
  float frequency = (freqReg == REG0) ? frequency0 : frequency1;
  SetFrequency(freqReg, frequency + freqIncHz);
}

void AD9833::SetPhase (Registers phaseReg, float phaseInDeg) {
  phaseInDeg = fmod(phaseInDeg,360);
  if (phaseInDeg < 0) phaseInDeg += 360;

  uint16_t phaseVal = (uint16_t)(BITS_PER_DEG * phaseInDeg) & 0x0FFF;
  phaseVal |= PHASE_WRITE_CMD;

  if (phaseReg == REG0) {
    phase0 = phaseInDeg;
  } else {
    phase1 = phaseInDeg;
    phaseVal |= PHASE1_WRITE_REG;
  }
  WriteRegister(phaseVal);
}

void AD9833::IncrementPhase (Registers phaseReg, float phaseIncDeg) {
  float phase = (phaseReg == REG0) ? phase0 : phase1;
  SetPhase(phaseReg, phase + phaseIncDeg);
}

void AD9833::SetWaveform (Registers waveFormReg, WaveformType waveType) {
  if (waveFormReg == REG0)
    waveForm0 = waveType;
  else
    waveForm1 = waveType;
  WriteControlRegister();
}

void AD9833::SetOutputSource (Registers freqReg, Registers phaseReg) {
  activeFreq = freqReg;
  if (phaseReg == SAME_AS_REG0) activePhase = activeFreq;
  else activePhase = phaseReg;
  WriteControlRegister();
}

void AD9833::EnableOutput (bool enable) {
  outputEnabled = enable;
  WriteControlRegister();
}

void AD9833::SleepMode (bool enable) {
  DacDisabled = enable;
  IntClkDisabled = enable;
  WriteControlRegister();
}

void AD9833::DisableDAC (bool enable) {
  DacDisabled = enable;
  WriteControlRegister();
}

void AD9833::DisableInternalClock (bool enable) {
  IntClkDisabled = enable;
  WriteControlRegister();
}

float AD9833::GetActualProgrammedFrequency (Registers reg) {
  float frequency = reg == REG0 ? frequency0 : frequency1;
  int32_t freqWord = (uint32_t)((frequency * pow2_28) / (float)refFrequency) & 0x0FFFFFFFUL;
  return (float)freqWord * (float)refFrequency / (float)pow2_28;
}

float AD9833::GetActualProgrammedPhase (Registers reg) {
  float phase = reg == REG0 ? phase0 : phase1;
  uint16_t phaseVal = (uint16_t)(BITS_PER_DEG * phase) & 0x0FFF;
  return (float)phaseVal / BITS_PER_DEG;
}

float AD9833::GetResolution (void) {
  return (float)refFrequency / (float)pow2_28;
}
 
void AD9833::WriteControlRegister (void) {
  uint16_t waveForm;
  if (activeFreq == REG0) {
    waveForm = waveForm0;
    waveForm &= ~FREQ1_OUTPUT_REG;
  }
  else {
    waveForm = waveForm1;
    waveForm |= FREQ1_OUTPUT_REG;
  }

  if (activePhase == REG0)
    waveForm &= ~PHASE1_OUTPUT_REG;
  else
    waveForm |= PHASE1_OUTPUT_REG;

  if (outputEnabled)
    waveForm &= ~RESET_CMD;
  else
    waveForm |= RESET_CMD;

  if (DacDisabled)
    waveForm |= DISABLE_DAC;
  else
    waveForm &= ~DISABLE_DAC;

  if (IntClkDisabled)
    waveForm |= DISABLE_INT_CLK;
  else
    waveForm &= ~DISABLE_INT_CLK;

  WriteRegister(waveForm);
}

void AD9833::WriteRegister (int16_t dat) {
  SPI.setDataMode(SPI_MODE2);
  WRITE_FNCPIN(LOW);
  SPI.transfer(highByte(dat));
  SPI.transfer(lowByte(dat));
  WRITE_FNCPIN(HIGH);
}


const uint8_t FSYNC_PIN = 10;
const uint32_t AD9833_REF_FREQ = 25000000UL;

AD9833 dds(FSYNC_PIN, AD9833_REF_FREQ);

// Frequency range for jamming (adjust as needed)
const float MIN_FREQ = 20000.0; // 20 kHz
const float MAX_FREQ = 30000.0; // 30 kHz

void setup() {
  SPI.begin();
  dds.Begin();
  dds.SetWaveform(REG0, SQUARE_WAVE);  // Square wave for richer harmonics
  dds.EnableOutput(true);
}

void loop() {
  // Generate a random frequency within the band
  float randomFreq = randomFrequency(MIN_FREQ, MAX_FREQ);
  dds.SetFrequency(REG0, randomFreq);

  // Very short delay to allow the frequency to appear momentarily
  // before switching again
  delayMicroseconds(500); // half a millisecond, adjust as needed
}

float randomFrequency(float minF, float maxF) {
  // Pick a random frequency within [minF, maxF]
  float f = minF + (maxF - minF) * (float)random(0,10000)/10000.0;
  return f;
}
