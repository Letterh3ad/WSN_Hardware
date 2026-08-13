#include <HardwareSerial.h>
#include <MHZ19.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_INA219.h>
#include <SPI.h>
#include <driver/rtc_io.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

// Arduino auto-generates forward declarations for every function and hoists
// them near the top of the file, above where they were originally written.
// SensorReadings has to be defined before that point or those generated
// declarations reference an unknown type and the whole file fails to parse.
struct SensorReadings {
    int co2;
    float temp;
    float hum;
    uint16_t light[10];
    bool lightOk;
    uint16_t lightGain;
    bool lightSaturated;
    float soilMoisture;
    float soilTemp;
    uint16_t soilEC;
    float soilPh;
    uint16_t soilN, soilP, soilK;
    bool soilOk;
    float batteryV, batteryI, batteryP;
    bool batteryOk;
    bool charging;
    uint32_t awakeMs;
};

// sensor pins, unchanged from sensor_test_all.ino
#define MHZ19_RX     13
#define MHZ19_TX     14
#define DHT_PIN       4
#define DHT_TYPE     DHT22
#define RS485_RX     25
#define RS485_TX     26
#define RS485_DE_RE  27
#define SWITCH_PIN   23

// lora pins, unchanged from lora_console_esp32.ino
#define LORA_CS     15
#define LORA_CLK    18
#define LORA_MOSI   5
#define LORA_MISO   19
#define LORA_RESET  17
#define LORA_BUSY   34
#define LORA_DIO1   35
#define LORA_RXEN   33
#define LORA_TXEN   32

// single ina219, wired directly in series with the battery so its current
// already is the net battery current, no second monitor to subtract against
// with only one device on the bus there is no address collision to strap
// around, so it just sits at the chip's default address
#define INA_BATTERY_ADDR 0x40

// current threshold in mA above which the battery counts as charging
// positive current is assumed to mean charging, flip the sign in readPower()
// if your shunt is oriented the other way
#define CHARGE_THRESHOLD_MA 5.0

// status led, blinked for the whole of the awake phase so the duty cycle is
// visible on the bench without a serial monitor attached
// gpio 2 is the onboard led on most esp32 devkits, change if yours differs
#define LED_PIN        2
#define LED_ON         HIGH
#define LED_OFF        LOW

// half-period of the blink, so 250 gives a 2Hz flash
#define BLINK_HALF_MS  250

// high-side switch levels, confirmed from sensor_test_all.ino
#define SWITCH_ON  HIGH
#define SWITCH_OFF LOW

// as7341 integration registers, set explicitly because the chip powers up at
// atime 0 / astep 999, which gives a full-scale count of only 1000 and makes
// every channel clip in daylight regardless of gain
// full scale = (atime + 1) * (astep + 1), capped at 65535
// 29 and 599 give an 18000 count ceiling over a 50ms integration
#define AS7341_ATIME  29
#define AS7341_ASTEP  599
#define AS7341_FULL_SCALE ((uint32_t)(AS7341_ATIME + 1) * (AS7341_ASTEP + 1))

// step the gain down once any channel passes this, leaving headroom below the
// ceiling so a reading near the limit is not recorded as if it were valid
#define AS7341_SATURATION_COUNTS ((uint16_t)(AS7341_FULL_SCALE * 9 / 10))

// as7341 auto-gain ladder, stepped down until no channel clips
// the gain in force is transmitted with every reading so the clear channel can
// be normalised to gain-independent counts during analysis, without which the
// insolation index is not comparable between a dull morning and full sun
const as7341_gain_t GAIN_LADDER[] = {
    AS7341_GAIN_512X, AS7341_GAIN_256X, AS7341_GAIN_128X, AS7341_GAIN_64X,
    AS7341_GAIN_32X, AS7341_GAIN_16X, AS7341_GAIN_8X, AS7341_GAIN_4X,
    AS7341_GAIN_2X, AS7341_GAIN_1X
};
// integer multipliers only, the 0.5X step is left off the ladder so the gain
// can travel in the payload as a plain integer
const uint16_t GAIN_MULTIPLIER[] = {512, 256, 128, 64, 32, 16, 8, 4, 2, 1};
const int GAIN_LADDER_LEN = sizeof(GAIN_LADDER) / sizeof(GAIN_LADDER[0]);

// fixed duty cycle for bench testing: 15s awake, 15s asleep
// AWAKE_TARGET_MS pads the wake phase out to a full 15s after the sensing,
// tx and ack window have finished, so each cycle is the same length
// regardless of how long the sensors took. set it to 0 to go straight back
// to sleep as soon as the work is done, which is the field behaviour
#define DUTY_SLEEP_S     15
#define AWAKE_TARGET_MS  15000

// with a fixed duty cycle the base station's sleep: ack must not move the
// interval, set this to 0 to hand control back to applyAck()
#define FIXED_DUTY_CYCLE 1

// fallback sleep interval used on cold boot and whenever no ack is received
#define DEFAULT_SLEEP_S DUTY_SLEEP_S

// task watchdog timeout for the wake phase. a full wake is a few seconds of
// sensing plus the tx and ack window, so 30s leaves generous headroom while
// still catching a genuine hang, most likely a sensor holding the i2c bus.
// on timeout the chip resets and the next wake retries, so one bad cycle
// costs a single reading rather than the whole unattended run
#define WDT_TIMEOUT_S 30

// how long the node keeps its rx open after transmitting to catch the ack
// exits early the moment the ack lands, so this only costs energy on a miss
#define ACK_WINDOW_MS 3000

// current sleep interval, persisted across deep sleep in rtc memory
// the initialiser only runs on a cold boot, deep-sleep wakes keep the value
RTC_DATA_ATTR uint32_t sleepIntervalS = DEFAULT_SLEEP_S;

// 868.000000 MHz register value, assumes 32MHz crystal, must match the Jetson side
const byte FREQ_REG[4] = {0x36, 0x40, 0x00, 0x00};

// private network sync word, must match on both ends or packets are silently dropped
const uint16_t SYNC_WORD = 0x1424;

const uint16_t IRQ_TX_DONE = 0x0001;
const uint16_t IRQ_RX_DONE = 0x0002;
const uint16_t IRQ_TIMEOUT = 0x0200;

HardwareSerial  mhzSerial(2);
MHZ19           mhz19;
DHT             dht(DHT_PIN, DHT_TYPE);
Adafruit_AS7341 as7341;
Adafruit_INA219 inaBattery(INA_BATTERY_ADDR);
HardwareSerial  rs485Serial(1);
SPIClass        loraSPI(HSPI);

bool batteryPresent = false;

esp_timer_handle_t blinkTimer = NULL;
bool ledState = false;

// the blink runs off a periodic esp_timer rather than delay() calls inside the
// sensing code, so it keeps flashing through the blocking reads and the rx
// window without any of those paths having to know about it
void blinkCallback(void *arg) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
}

void startBlink() {
    ledState = false;
    digitalWrite(LED_PIN, LED_OFF);
    const esp_timer_create_args_t args = {
        .callback = &blinkCallback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "blink",
    };
    if (esp_timer_create(&args, &blinkTimer) == ESP_OK) {
        esp_timer_start_periodic(blinkTimer, (uint64_t)BLINK_HALF_MS * 1000ULL);
    } else {
        Serial.println("blink timer: create failed");
        blinkTimer = NULL;
    }
}

void stopBlink() {
    if (blinkTimer) {
        esp_timer_stop(blinkTimer);
        esp_timer_delete(blinkTimer);
        blinkTimer = NULL;
    }
    digitalWrite(LED_PIN, LED_OFF);
}

// holds the wake phase open until AWAKE_TARGET_MS has elapsed since boot so
// every cycle is the same length, kicking the watchdog as it waits since this
// is a deliberate idle rather than a hang
void holdAwakeUntilTarget() {
    if (AWAKE_TARGET_MS == 0) return;
    while (millis() < AWAKE_TARGET_MS) {
        esp_task_wdt_reset();
        delay(50);
    }
}

// prints every address that acks on the i2c bus, useful if the battery
// monitor or as7341 stop showing up
void scanI2C() {
    Serial.println("i2c scan:");
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  found device at 0x%02X\n", addr);
        }
    }
}

byte soilRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};

// fixed switched-rail timings, named rather than inline so the per-cycle
// overhead used in the energy model is read from one place
#define SWITCH_SETTLE_MS    2000
#define SWITCH_DISCHARGE_MS 2000

void switchOn()  { digitalWrite(SWITCH_PIN, SWITCH_ON);  delay(SWITCH_SETTLE_MS); }
// the discharge delay matters for the energy accounting as much as for the
// sensors, it is a fixed per-cycle cost that shorter intervals pay more often
void switchOff() { digitalWrite(SWITCH_PIN, SWITCH_OFF); delay(SWITCH_DISCHARGE_MS); }

void readCO2(SensorReadings &r) {
    r.co2 = mhz19.getCO2();
}

void readDHT(SensorReadings &r) {
    r.temp = dht.readTemperature();
    r.hum = dht.readHumidity();
}

// steps gain down the ladder until no channel clips, so the clear channel
// stays usable in full sun instead of pinning at the top of its range
// r.lightGain records the multiplier actually used, which analysis divides by
void readLight(SensorReadings &r) {
    r.lightOk = false;
    r.lightSaturated = false;
    r.lightGain = 0;

    for (int g = 0; g < GAIN_LADDER_LEN; g++) {
        esp_task_wdt_reset();
        as7341.setGain(GAIN_LADDER[g]);
        delay(50);
        if (!as7341.readAllChannels()) return;

        uint16_t peak = 0;
        for (int c = 0; c < 10; c++) {
            uint16_t v = as7341.getChannel((as7341_color_channel_t)c);
            if (v > peak) peak = v;
        }

        r.lightGain = GAIN_MULTIPLIER[g];
        if (peak < AS7341_SATURATION_COUNTS) break;
        // still clipping at the bottom of the ladder, flag it rather than
        // silently reporting a saturated count as if it were a real reading
        if (g == GAIN_LADDER_LEN - 1) r.lightSaturated = true;
    }

    r.lightOk = true;
    r.light[0] = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
    r.light[1] = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
    r.light[2] = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
    r.light[3] = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
    r.light[4] = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
    r.light[5] = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
    r.light[6] = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
    r.light[7] = as7341.getChannel(AS7341_CHANNEL_680nm_F8);
    r.light[8] = as7341.getChannel(AS7341_CHANNEL_NIR);
    r.light[9] = as7341.getChannel(AS7341_CHANNEL_CLEAR);
}

void readSoil(SensorReadings &r) {
    r.soilOk = false;
    while (rs485Serial.available()) rs485Serial.read();
    digitalWrite(RS485_DE_RE, HIGH);
    delayMicroseconds(200);
    rs485Serial.write(soilRequest, sizeof(soilRequest));
    rs485Serial.flush();
    delay(5);
    digitalWrite(RS485_DE_RE, LOW);
    unsigned long t = millis();
    while (millis() - t < 1500 && !rs485Serial.available()) delay(10);
    if (!rs485Serial.available()) return;
    delay(50);
    byte buf[19]; int i = 0;
    while (rs485Serial.available() && i < 19) buf[i++] = rs485Serial.read();
    if (i == 19 && buf[0] == 0x01 && buf[1] == 0x03 && buf[2] == 0x0E) {
        r.soilMoisture = ((buf[3]<<8)|buf[4]) / 10.0f;
        r.soilTemp     = ((buf[5]<<8)|buf[6]) / 10.0f;
        r.soilEC       = (buf[7]<<8)|buf[8];
        r.soilPh       = ((buf[9]<<8)|buf[10]) / 10.0f;
        r.soilN        = (buf[11]<<8)|buf[12];
        r.soilP        = (buf[13]<<8)|buf[14];
        r.soilK        = (buf[15]<<8)|buf[16];
        r.soilOk = true;
    }
}

// the ina219 sits on the always-on i2c rail, so it does not need the
// high-side switch, but it is read here alongside the other sensors for
// locality
//
// wired directly in series with the battery: positive current means the
// battery is net charging, negative means it is net discharging, no second
// monitor or subtraction needed since this single reading already is net
//
// power is computed as V * I from the same voltage/current snapshot rather
// than read from the chip's own power register, which drifted 30-40% from
// V*I on this hardware
void readPower(SensorReadings &r) {
    r.batteryOk = batteryPresent;
    if (batteryPresent) {
        r.batteryV = inaBattery.getBusVoltage_V();
        r.batteryI = inaBattery.getCurrent_mA();
        r.batteryP = r.batteryV * r.batteryI;
        r.charging = r.batteryI > CHARGE_THRESHOLD_MA;
    }
}

String buildPayload(const SensorReadings &r) {
    String p = "";
    p += "co2:" + String(r.co2);
    p += ",t:" + String(r.temp, 1);
    p += ",h:" + String(r.hum, 1);
    if (r.lightOk) {
        p += ",clear:" + String(r.light[9]);
        p += ",nir:" + String(r.light[8]);
        p += ",g:" + String(r.lightGain);
        if (r.lightSaturated) p += ",gsat:1";
    }
    if (r.soilOk) {
        p += ",sm:" + String(r.soilMoisture, 1);
        p += ",st:" + String(r.soilTemp, 1);
        p += ",ec:" + String(r.soilEC);
        p += ",ph:" + String(r.soilPh, 1);
        p += ",n:" + String(r.soilN);
        p += ",p:" + String(r.soilP);
        p += ",k:" + String(r.soilK);
    }
    if (r.batteryOk) {
        p += ",bv:" + String(r.batteryV, 3);
        p += ",bi:" + String(r.batteryI, 1);
        p += ",bp:" + String(r.batteryP, 1);
    }
    // the interval in force for the cycle just completed, and how long the
    // node was awake for it, so every row carries the duty-cycle configuration
    // it was recorded under rather than relying on the base station's current
    // setting still matching what the node was doing at the time
    p += ",si:" + String((unsigned long)sleepIntervalS);
    p += ",aw:" + String((unsigned long)r.awakeMs);
    return p;
}

bool waitBusy(unsigned long timeoutMs = 2000) {
    unsigned long start = millis();
    while (digitalRead(LORA_BUSY) == HIGH) {
        if (millis() - start > timeoutMs) return false;
    }
    return true;
}

void resetChip() {
    digitalWrite(LORA_RESET, LOW);
    delay(1);
    digitalWrite(LORA_RESET, HIGH);
    delay(5);
    waitBusy();
}

void writeCommand(byte opcode, const byte* params, int len) {
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(opcode);
    for (int i = 0; i < len; i++) loraSPI.transfer(params[i]);
    digitalWrite(LORA_CS, HIGH);
    waitBusy();
}

void writeRegister(uint16_t addr, byte value) {
    byte params[3] = {(byte)(addr >> 8), (byte)(addr & 0xFF), value};
    writeCommand(0x0D, params, 3);
}

void writeBuffer(const byte* data, int len) {
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(0x0E);
    loraSPI.transfer(0x00);
    for (int i = 0; i < len; i++) loraSPI.transfer(data[i]);
    digitalWrite(LORA_CS, HIGH);
    waitBusy();
}

// reads back a received packet from the radio fifo starting at startPtr
String readBuffer(byte startPtr, byte length) {
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(0x1E);
    loraSPI.transfer(startPtr);
    loraSPI.transfer(0x00);
    char buf[256];
    for (int i = 0; i < length; i++) buf[i] = (char)loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    waitBusy();
    buf[length] = '\0';
    return String(buf);
}

// framing mirrors the Jetson read_command(0x13, 3): status, length, start pointer
void getRxBufferStatus(byte &length, byte &startPtr) {
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(0x13);
    loraSPI.transfer(0x00);
    length = loraSPI.transfer(0x00);
    startPtr = loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    waitBusy();
}

uint16_t getIrqStatus() {
    byte data[3];
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(0x12);
    for (int i = 0; i < 3; i++) data[i] = loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    waitBusy();
    return ((uint16_t)data[1] << 8) | data[2];
}

void setStandbyRC() {
    byte p[1] = {0x00};
    writeCommand(0x80, p, 1);
}

void setRegulatorMode() {
    // 0x01 selects DC-DC + LDO, needed for the PA to draw enough current during TX
    byte p[1] = {0x01};
    writeCommand(0x96, p, 1);
}

void clearDeviceErrors() {
    byte p[2] = {0x00, 0x00};
    writeCommand(0x07, p, 2);
}

void setDio3AsTcxoCtrl(byte voltageCode = 0x01, uint32_t delaySteps = 320) {
    byte p[4] = {voltageCode, (byte)(delaySteps >> 16), (byte)(delaySteps >> 8), (byte)delaySteps};
    writeCommand(0x97, p, 4);
}

void setPacketTypeLoRa() {
    byte p[1] = {0x01};
    writeCommand(0x8A, p, 1);
}

void setRfFrequency() {
    writeCommand(0x86, FREQ_REG, 4);
}

void setPaConfig() {
    byte p[4] = {0x02, 0x02, 0x00, 0x01};
    writeCommand(0x95, p, 4);
}

void setTxParams(int8_t powerDbm = 14, byte ramp = 0x04) {
    byte p[2] = {(byte)powerDbm, ramp};
    writeCommand(0x8E, p, 2);
}

void setBufferBaseAddress() {
    byte p[2] = {0x00, 0x00};
    writeCommand(0x8F, p, 2);
}

void setModulationParams(byte sf = 7, byte bw = 0x04, byte cr = 0x01, byte ldro = 0x00) {
    byte p[4] = {sf, bw, cr, ldro};
    writeCommand(0x8B, p, 4);
}

void setPacketParams(byte payloadLen, uint16_t preamble = 8, byte headerType = 0x00, byte crcOn = 0x01, byte invertIq = 0x00) {
    byte p[6] = {(byte)(preamble >> 8), (byte)(preamble & 0xFF), headerType, payloadLen, crcOn, invertIq};
    writeCommand(0x8C, p, 6);
}

void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask) {
    byte p[8] = {(byte)(irqMask >> 8), (byte)(irqMask & 0xFF),
                 (byte)(dio1Mask >> 8), (byte)(dio1Mask & 0xFF),
                 0x00, 0x00, 0x00, 0x00};
    writeCommand(0x08, p, 8);
}

void clearIrqStatus() {
    byte p[2] = {0xFF, 0xFF};
    writeCommand(0x02, p, 2);
}

void setTx(uint32_t timeoutMs = 2000) {
    uint32_t steps = (uint32_t)(timeoutMs / 0.015625);
    byte p[3] = {(byte)(steps >> 16), (byte)(steps >> 8), (byte)steps};
    writeCommand(0x83, p, 3);
}

void setRx(uint32_t timeoutMs) {
    uint32_t steps = (uint32_t)(timeoutMs / 0.015625);
    byte p[3] = {(byte)(steps >> 16), (byte)(steps >> 8), (byte)steps};
    writeCommand(0x82, p, 3);
}

// unlike every other command, sleep is bit-banged directly rather than going
// through writeCommand(), because BUSY behaves differently once the chip is
// told to sleep. the delay below is a fixed settling time in its place, long
// enough for the command to latch before the spi bus gets shut down right
// after this returns - without it, the radio can be left mid-transition in
// an active standby state for the entire deep sleep period instead of its
// true low-power sleep, which draws several mA continuously instead of ~1uA
void setLoraSleep() {
    digitalWrite(LORA_RXEN, LOW);
    digitalWrite(LORA_TXEN, LOW);
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(0x84);
    loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    delay(10);
}

// recalibrate all blocks against the tcxo clock, stale image/pll cal left over
// from the xtal clock is the usual cause of a receiver that completes on noise
void calibrateAll() {
    byte p[1] = {0x7F};
    writeCommand(0x89, p, 1);
    delay(5);
}

uint16_t getDeviceErrors() {
    byte data[3];
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(0x17);
    for (int i = 0; i < 3; i++) data[i] = loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    waitBusy();
    return ((uint16_t)data[1] << 8) | data[2];
}

void printDeviceErrors(const char* tag) {
    uint16_t mask = getDeviceErrors();
    Serial.printf("device errors%s: 0x%04X", tag, mask);
    if (mask & 0x0020) Serial.print(" XOSC_START");
    if (mask & 0x0040) Serial.print(" PLL_LOCK");
    if (mask & 0x0004) Serial.print(" PLL_CALIB");
    if (mask & 0x0010) Serial.print(" IMG_CALIB");
    Serial.println();
}

void initRadio() {
    resetChip();
    setStandbyRC();
    setDio3AsTcxoCtrl();
    calibrateAll();
    clearDeviceErrors();
    setRegulatorMode();
    setPacketTypeLoRa();
    setRfFrequency();
    setPaConfig();
    setTxParams();
    setBufferBaseAddress();
    setModulationParams();
    writeRegister(0x0740, (SYNC_WORD >> 8) & 0xFF);
    writeRegister(0x0741, SYNC_WORD & 0xFF);
    clearIrqStatus();
    printDeviceErrors(" post-init");
}

void sendMessage(const String &message) {
    int len = message.length();
    byte data[256];
    message.getBytes(data, len + 1);

    setPacketParams(len);
    writeBuffer(data, len);
    setDioIrqParams(IRQ_TX_DONE | IRQ_TIMEOUT, IRQ_TX_DONE | IRQ_TIMEOUT);
    clearIrqStatus();
    digitalWrite(LORA_TXEN, HIGH);
    digitalWrite(LORA_RXEN, LOW);
    setTx(2000);

    unsigned long start = millis();
    while (millis() - start < 3000) {
        if (digitalRead(LORA_DIO1) == HIGH) break;
        delay(10);
    }

    uint16_t irq = getIrqStatus();
    clearIrqStatus();
    digitalWrite(LORA_TXEN, LOW);

    if (irq & IRQ_TX_DONE) {
        Serial.printf("sent: \"%s\"\n", message.c_str());
    } else {
        Serial.printf("send failed, irq status: 0x%04X\n", irq);
    }
}

// opens a single rx window right after tx and returns the received payload,
// or an empty string on timeout, exits early on rx done to save energy
String listenForAck(uint32_t timeoutMs) {
    setPacketParams(255);
    setDioIrqParams(IRQ_RX_DONE | IRQ_TIMEOUT, IRQ_RX_DONE | IRQ_TIMEOUT);
    clearIrqStatus();
    digitalWrite(LORA_RXEN, HIGH);
    digitalWrite(LORA_TXEN, LOW);
    setRx(timeoutMs);

    unsigned long start = millis();
    while (millis() - start < timeoutMs + 200) {
        if (digitalRead(LORA_DIO1) == HIGH) break;
        delay(5);
    }

    uint16_t irq = getIrqStatus();
    clearIrqStatus();
    digitalWrite(LORA_RXEN, LOW);

    if (irq & IRQ_RX_DONE) {
        byte length, startPtr;
        getRxBufferStatus(length, startPtr);
        return readBuffer(startPtr, length);
    }
    return "";
}

// parses "sleep:<seconds>" and updates the persisted interval if in range,
// out-of-range or malformed acks are ignored so the last good value stands
void applyAck(const String &ack) {
#if FIXED_DUTY_CYCLE
    // interval is pinned by DUTY_SLEEP_S, the ack is logged but not acted on
    Serial.println("fixed duty cycle, ack interval ignored");
    return;
#endif
    int idx = ack.indexOf("sleep:");
    if (idx < 0) return;
    long v = ack.substring(idx + 6).toInt();
    if (v >= 5 && v <= 86400) {
        sleepIntervalS = (uint32_t)v;
        Serial.printf("ack: next sleep %lus\n", (unsigned long)sleepIntervalS);
    } else {
        Serial.printf("ack ignored, out of range: %ld\n", v);
    }
}

// puts the switch pin and lora control pins into deep sleep hold so none of
// them float while the esp itself is asleep, then starts deep sleep
void goToSleep() {
    digitalWrite(SWITCH_PIN, SWITCH_OFF);
    digitalWrite(LORA_CS, HIGH);
    digitalWrite(LORA_RXEN, LOW);
    digitalWrite(LORA_TXEN, LOW);
    digitalWrite(LED_PIN, LED_OFF);

    // led is held too so it cannot float back on while the chip is asleep,
    // this only works on an rtc-capable pin, which gpio 2 is
    gpio_hold_en((gpio_num_t)LED_PIN);
    gpio_hold_en((gpio_num_t)SWITCH_PIN);
    gpio_hold_en((gpio_num_t)LORA_CS);
    gpio_hold_en((gpio_num_t)LORA_RXEN);
    gpio_hold_en((gpio_num_t)LORA_TXEN);
    gpio_deep_sleep_hold_en();

#if FIXED_DUTY_CYCLE
    // set explicitly rather than trusting the rtc-persisted value, which can
    // survive a reflash carrying an interval from a previous run
    sleepIntervalS = DUTY_SLEEP_S;
#endif
    uint64_t sleepUs = (uint64_t)sleepIntervalS * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleepUs);

    Serial.printf("entering deep sleep for %lus, switch held off\n", (unsigned long)sleepIntervalS);
    Serial.flush();
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    delay(200);

    // arm the task watchdog first thing, before any sensor or radio work that
    // could hang. esp_task_wdt_reset() is called at each phase boundary below
    // so a stall anywhere in the wake sequence trips it
    // the init signature changed in esp-idf 5.x from (timeout, panic) to a
    // config struct, so branch on the idf version to build on both
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_cfg);
#else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
    esp_task_wdt_add(NULL);

    // release any hold left over from the previous deep sleep before driving the pins
    gpio_hold_dis((gpio_num_t)LED_PIN);
    gpio_hold_dis((gpio_num_t)SWITCH_PIN);
    gpio_hold_dis((gpio_num_t)LORA_CS);
    gpio_hold_dis((gpio_num_t)LORA_RXEN);
    gpio_hold_dis((gpio_num_t)LORA_TXEN);
    pinMode(SWITCH_PIN, OUTPUT);
    digitalWrite(SWITCH_PIN, SWITCH_OFF);

    // starts as early as possible so the led is flashing for the whole of the
    // awake window, not just the sensing part of it
    pinMode(LED_PIN, OUTPUT);
    startBlink();

    pinMode(RS485_DE_RE, OUTPUT);
    digitalWrite(RS485_DE_RE, LOW);
    mhzSerial.begin(9600, SERIAL_8N1, MHZ19_RX, MHZ19_TX);
    mhz19.begin(mhzSerial);
    mhz19.autoCalibration(false);
    dht.begin();
    Wire.begin();
    // without this a device holding the bus low blocks the arduino i2c calls
    // forever, which is the failure the watchdog exists to catch, but a bus
    // timeout turns most cases into a recoverable error before the wdt fires
    Wire.setTimeOut(250);
    esp_task_wdt_reset();
    if (!as7341.begin()) {
        Serial.println("AS7341: init failed");
    } else {
        // must be set after begin(), which leaves the power-on defaults in place
        as7341.setATIME(AS7341_ATIME);
        as7341.setASTEP(AS7341_ASTEP);
        Serial.printf("AS7341: full scale %lu counts\n", (unsigned long)AS7341_FULL_SCALE);
    }

    scanI2C();

    // ina219 shares the as7341 i2c bus, default 32V/2A calibration is applied
    // by begin(), swap to setCalibration_16V_400mA() for finer resolution if
    // your battery current stays comfortably below 400mA
    batteryPresent = inaBattery.begin();
    if (!batteryPresent) Serial.println("INA219 battery: init failed");

    rs485Serial.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);

    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    pinMode(LORA_BUSY, INPUT);
    pinMode(LORA_DIO1, INPUT);
    pinMode(LORA_RXEN, OUTPUT);
    digitalWrite(LORA_RXEN, LOW);
    pinMode(LORA_TXEN, OUTPUT);
    digitalWrite(LORA_TXEN, LOW);

    loraSPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI, LORA_CS);
    loraSPI.setFrequency(2000000);
    loraSPI.setDataMode(SPI_MODE0);
    initRadio();

    SensorReadings r = {};
    switchOn();
    readCO2(r);
    esp_task_wdt_reset();
    readDHT(r);
    readLight(r);
    esp_task_wdt_reset();
    readSoil(r);
    readPower(r);
    switchOff();
    esp_task_wdt_reset();

    // captured before the payload is built so it covers everything from boot
    // through the end of the sensing phase, the tx and ack window that follow
    // are a fixed cost measured separately in the dark-run characterisation
    r.awakeMs = millis();

    String payload = buildPayload(r);
    sendMessage(payload);

    String ack = listenForAck(ACK_WINDOW_MS);
    if (ack.length() > 0) {
        Serial.printf("ack rx: \"%s\"\n", ack.c_str());
        applyAck(ack);
    } else {
        Serial.printf("no ack, keeping last interval %lus\n", (unsigned long)sleepIntervalS);
    }

    setLoraSleep();
    loraSPI.end();

    holdAwakeUntilTarget();
    Serial.printf("awake %lums, sensing took %lums\n",
                  (unsigned long)millis(), (unsigned long)r.awakeMs);

    stopBlink();
    goToSleep();
}

void loop() {
    // never reached, esp resets and re-runs setup() after each deep sleep wake
}
