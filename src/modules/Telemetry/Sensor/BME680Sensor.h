#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_BME680.h>)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "BME680IaqEstimator.h"
#include "Observer.h"
#include "TelemetrySensor.h"

#include <Adafruit_BME680.h>
#include <memory>

class BME680Sensor : public TelemetrySensor
{
  private:
    using BME680Ptr = std::unique_ptr<Adafruit_BME680>;

    static BME680Ptr makeBME680(TwoWire *bus) { return BME680Ptr(new Adafruit_BME680(bus)); }

    BME680Ptr bme680;
    BME680IaqEstimator iaqEstimator;

    static constexpr uint32_t SAMPLE_INTERVAL_MS = 60 * 1000;
    // getMetrics() publishes the cached async sample only while it is this
    // fresh; a failed refresh past this age drops the BME680 fields from the
    // packet rather than freezing the last reading on the wire
    static constexpr uint32_t SAMPLE_FRESH_MS = 2 * 60 * 1000;
    // A heater-unstable cycle reports gas_resistance 0; carry the previous IAQ
    // through such blips, but not forever
    static constexpr uint32_t IAQ_CARRY_MS = 10 * 60 * 1000;
    static constexpr const char *stateFileName = "/prefs/bme680.dat";

    // Async sampling state (driven from runOnce)
    bool readingInFlight = false;
    uint32_t readingDoneAtMs = 0;

    // Cached last sample
    bool haveSample = false;
    uint32_t lastSampleMs = 0;
    float lastTemperature = 0;
    float lastHumidity = 0;
    float lastPressureHPa = 0;
    float lastGasOhms = 0;
    uint16_t lastIaq = 0;
    bool lastIaqValid = false;
    uint32_t lastIaqMs = 0;

    void captureSample();
    // The baseline is written to flash only on the user's shutdown or reboot. Every sample also copies it to
    // RTC memory, so an ESP32 deep-sleeping between samples keeps its burn-in without writing flash.
    void loadState();
    void carryState();
    int saveState(void *unused = nullptr);
    CallbackObserver<BME680Sensor, void *> userPowerOffObserver =
        CallbackObserver<BME680Sensor, void *>(this, &BME680Sensor::saveState);

  public:
    BME680Sensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;
};

#endif
