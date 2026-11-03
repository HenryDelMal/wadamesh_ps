#pragma once

#include "CustomLR1110.h"
#include "RadioLibWrappers.h"
#include "LR11x0Reset.h"

class CustomLR1110Wrapper : public RadioLibWrapper {
public:
  CustomLR1110Wrapper(CustomLR1110& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    prepareForRadioConfig();
    ((CustomLR1110 *)_radio)->setFrequency(freq);
    ((CustomLR1110 *)_radio)->setSpreadingFactor(sf);
    ((CustomLR1110 *)_radio)->setBandwidth(bw);
    ((CustomLR1110 *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
    PacketMillis pm = calcMaxPacketMillis(sf, bw, cr, preambleLengthForSF(sf));
    ((CustomLR1110 *)_radio)->setPreambleMillis(pm.preambleMillis);
    ((CustomLR1110 *)_radio)->setMaxPayloadMillis(pm.payloadMillis);
  }

  bool isReceivingPacket() override {
    if (_rx_ps_armed && isChipBusy()) return false;
    return ((CustomLR1110 *)_radio)->isReceiving();
  }
  float getCurrentRSSI() override {
    float rssi = -110;
    radioAcquire();
    if (!_rx_ps_armed || !isChipBusy()) ((CustomLR1110 *)_radio)->getRssiInst(&rssi);
    radioRelease();
    return rssi;
  }

  uint32_t getEstAirtimeFor(int len_bytes) override {
    radioAcquire();
    auto airtime = _rx_ps_armed
        ? ((CustomLR1110 *)_radio)->loraTimeOnAir(len_bytes) / 1000
        : RadioLibWrapper::getEstAirtimeFor(len_bytes);
    radioRelease();
    return airtime < 200 ? 200 : airtime;   // at least 200 millis
  }

  void onSendFinished() override {
    RadioLibWrapper::onSendFinished();
    _radio->setPreambleLength(preambleLengthForSF(getSpreadingFactor())); // overcomes weird issues with small and big pkts
  }


  uint8_t getSpreadingFactor() const override { return ((CustomLR1110 *)_radio)->getSpreadingFactor(); }
  
  bool setRxBoostedGainMode(bool en) override {
    radioAcquire();
    prepareForRadioConfig();
    const bool ok = ((CustomLR1110 *)_radio)->setRxBoostedGainMode(en) == RADIOLIB_ERR_NONE;
    radioRelease();
    return ok;
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomLR1110 *)_radio)->getRxBoostedGainMode();
  }

  void doResetAGC() override { lr11x0ResetAGC((LR11x0 *)_radio, ((CustomLR1110 *)_radio)->getFreqMHz(), getRxBoostedGainMode()); }

  float rxPowerSavingCaptureCostSymbols() const override {
    return RX_POWERSAVING_CAPTURE_COST_SYMBOLS_LR11X0;
  }

  uint32_t rxPowerSavingTransitionUs() const override {
    return ((CustomLR1110 *)_radio)->getTcxoDelay() + 1000;
  }

protected:
  bool supportsRxPowerSaving() const override {
#ifdef MESH_RADIO_DIO1_POLLED
    return false;
#else
    return true;
#endif
  }

  bool isChipBusy() override { return ((CustomLR1110 *)_radio)->isChipBusy(); }

  int16_t startContinuousReceive() override {
    const int16_t err = ((CustomLR1110 *)_radio)->restoreLfClock();
    return err == RADIOLIB_ERR_NONE ? _radio->startReceive() : err;
  }

  int16_t armDutyCycle(RadioLibIrqFlags_t flags, RadioLibIrqFlags_t mask,
                       uint32_t rx_us, uint32_t sleep_us, uint32_t& effective_rx_us) override {
    return ((CustomLR1110 *)_radio)->startReceiveDutyCycle(rx_us, sleep_us, flags, mask, &effective_rx_us);
  }

  int16_t stopDutyCycleHardware() override {
    const int16_t err = _radio->standby();
    return err == RADIOLIB_ERR_NONE ? ((CustomLR1110 *)_radio)->restoreLfClock() : err;
  }
};
