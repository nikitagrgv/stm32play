#include "BaseMQ.h"

#include "Sleep.h"

#include <cmath>

BaseMQ::BaseMQ() {}

// BaseMQ::BaseMQ(uint8_t pin, uint8_t pinHeater)
// {
// _pin = pin;
// _pinHeater = pinHeater;
// pinMode(_pinHeater, OUTPUT);
// }

BaseMQ::~BaseMQ() {}

// фиксированая калибровка датчика
// при знании сопративления датчика на чистом воздухе
void BaseMQ::calibrate(float ro)
{
    _ro = ro;
    _stateCalibrate = true;
}

// калибровка датчика
// считывания показаний сопративление датчика на чистом воздухе
// далее фиксированая калибровка датчика
void BaseMQ::calibrate()
{
    float rs = readRs();
    float ro = rs / getRoInCleanAir();
    calibrate(ro);
}

// включение нагревателя на 100%
void BaseMQ::heaterPwrHigh()
{
    // digitalWrite(_pinHeater, HIGH);
    // _heater = true;
    // _prMillis = millis();
}

// включение нагревателья на 20%
void BaseMQ::heaterPwrLow()
{
    // analogWrite(_pinHeater, 75);
    // _heater = true;
    // _cooler = true;
    // _prMillis = millis();
}

// выключение нагревателя
void BaseMQ::heaterPwrOff()
{
    // digitalWrite(_pinHeater, LOW);
    // _heater = false;
}

// циклическое считывание сопративления датчика
float BaseMQ::readRs() const
{
    float rs = 0;
    for (int i = 0; i < MQ_SAMPLE_TIMES; i++)
    {
        rs += calculateResistance();
        utils::sleepMsec(MQ_SAMPLE_INTERVAL);
    }
    rs = rs / MQ_SAMPLE_TIMES;
    return rs;
}


// сопротивление датчика
float BaseMQ::calculateResistance() const
{
    float sensorVoltage = getVoltage();
    float sensorResistance = (OPERATING_VOLTAGE - sensorVoltage) / sensorVoltage * getRL();
    return sensorResistance;
}

float BaseMQ::readScaled(float a, float b) const
{
    return std::exp((std::log(cached_ratio) - b) / a);
}

float BaseMQ::readRatio() const
{
    return readRs() / getRo();
}

void BaseMQ::updateRatio()
{
    cached_ratio = readRatio();
}

bool BaseMQ::heatingCompleted() const
{
    // if ((_heater) && (!_cooler) && (millis() - _prMillis > MQ_HEATING_TIME)) {
    // return true;
    // } else {
    // return false;
    // }
    return true;
}

bool BaseMQ::coolanceCompleted() const
{
    // if ((_heater) && (_cooler) && (millis() - _prMillis > MQ_COOLANCE_TIME)) {
    // return true;
    // } else {
    // return false;
    // }
    return true;
}

void BaseMQ::cycleHeat()
{
    // _heater = false;
    // _cooler = false;
    // heaterPwrHigh();
    //  Serial.println("Heated sensor");
}

bool BaseMQ::atHeatCycleEnd()
{
    // if (heatingCompleted()) {
    // heaterPwrLow();
    //    Serial.println("Cool sensor");
    //   return false;
    // } else if (coolanceCompleted()) {
    //   heaterPwrOff();
    //   return true;
    // } else {
    //   return false;
    // }
    return true;
}
