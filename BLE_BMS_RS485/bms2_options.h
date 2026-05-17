#ifndef BMS2_OPTIONS_H
#define BMS2_OPTIONS_H

// Memory-saving options: Comment these out if you do not need them.
// It will save a slight bit of SRAM, which is very precious on 8-bit AVRs.
// #define BMS_OPTION_PRODUCTION_DATE
// #define BMS_OPTION_SW_VERSION
#define BMS_OPTION_NAME
#define BMS_OPTION_FAULT_COUNTS
// #define BMS_OPTION_DEBUG
// #define BMS_OPTION_DEBUG_STATE_MACHINE
// #define BMS_OPTION_DEBUG_WRITE
// #define BMS_OPTION_DEBUG_PARAM

#define BMS_TIMEOUT         500
#define BMS_MAX_CELLS       16
#define BMS_MAX_NTCs        4
#define BMS_MAX_RX_DATA_LEN 64

#endif  // BMS2_OPTIONS_H
