#pragma once

struct AggregatedBmsData {
    bool    valid;
    uint8_t validCount;
    float   avgVoltage;
    float   totalCurrent;
    float   avgTemperature;
    float   minCellVoltage;
    float   maxCellVoltage;
    bool    chargeAllowed;
    bool    dischargeAllowed;
};
