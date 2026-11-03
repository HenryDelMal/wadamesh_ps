#pragma once

#include "CustomSX1262.h"
#include "RadioLibWrappers.h"
#include "SX126xReset.h"

#ifndef USE_SX1262
#define USE_SX1262
#endif

class CustomSX1262Wrapper : public RadioLibWrapper {
public:
  CustomSX1262Wrapper(CustomSX1262& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    prepareForRadioConfig();
    ((CustomSX1262 *)_radio)->setFrequency(freq);
    ((CustomSX1262 *)_radio)->setSpreadingFactor(sf);
    ((CustomSX1262 *)_radio)->setBandwidth(bw);
    ((CustomSX1262 *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
    PacketMillis pm = calcMaxPacketMillis(sf, bw, cr, preambleLengthForSF(sf));
    ((CustomSX1262 *)_radio)->setPreambleMillis(pm.preambleMillis);
    ((CustomSX1262 *)_radio)->setMaxPayloadMillis(pm.payloadMillis);
  }

  bool isReceivingPacket() override { 
    if (_rx_ps_armed && isChipBusy()) return false;
    return ((CustomSX1262 *)_radio)->isReceiving();
  }
  float getCurrentRSSI() override {
    radioAcquire();
    const float rssi = _rx_ps_armed && isChipBusy() ? -120.0f : ((CustomSX1262 *)_radio)->getRSSI(false);
    radioRelease();
    return rssi;
  }

  float packetScore(float snr, int packet_len) override {
    int sf = ((CustomSX1262 *)_radio)->spreadingFactor;
    return packetScoreInt(snr, sf, packet_len);
  }
  uint32_t getEstAirtimeFor(int len_bytes) override {
    radioAcquire();
    const auto result = _rx_ps_armed
        ? ((CustomSX1262 *)_radio)->loraTimeOnAir(len_bytes) / 1000
        : RadioLibWrapper::getEstAirtimeFor(len_bytes);
    radioRelease();
    return result;
  }
  uint8_t getSpreadingFactor() const override { return ((CustomSX1262 *)_radio)->spreadingFactor; }
  virtual void powerOff() override {
    radioAcquire();
    if (_rx_ps_armed) stopReceiveDutyCycle();
    ((CustomSX1262 *)_radio)->sleep(false);
    radioRelease();
  }

  // See RadioLibWrapper::pollRxIfNoIrq — RX-done straight from the IRQ register.
  bool pollRxDone() override {
    return (((CustomSX1262 *)_radio)->getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE) != 0;
  }
  bool setRxBoostedGainMode(bool en) override {
    radioAcquire();
    prepareForRadioConfig();
    const bool ok = ((CustomSX1262 *)_radio)->setRxBoostedGainMode(en) == RADIOLIB_ERR_NONE;
    radioRelease();
    return ok;
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomSX1262 *)_radio)->getRxBoostedGainMode();
  }

  void doResetAGC() override { sx126xResetAGC((SX126x *)_radio, getRxBoostedGainMode()); }

  uint32_t rxPowerSavingTransitionUs() const override {
    return ((CustomSX1262 *)_radio)->dutyCycleTransitionUs();
  }

protected:
  bool supportsRxPowerSaving() const override {
#ifdef MESH_RADIO_DIO1_POLLED
    return false;
#else
    return true;
#endif
  }

  bool isChipBusy() override { return ((CustomSX1262 *)_radio)->isChipBusy(); }

  int16_t armDutyCycle(RadioLibIrqFlags_t flags, RadioLibIrqFlags_t mask,
                       uint32_t rx_us, uint32_t sleep_us, uint32_t& effective_rx_us) override {
    effective_rx_us = rx_us;
    return ((CustomSX1262 *)_radio)->startReceiveDutyCycle(rx_us, sleep_us, flags, mask);
  }

  int16_t stopDutyCycleHardware() override {
    const int16_t err = _radio->standby();
    return err == RADIOLIB_ERR_NONE ? ((CustomSX1262 *)_radio)->stopRTC() : err;
  }
};
