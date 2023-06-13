#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

// Default name
#if defined(PROJECT_CHAMELEON_ULTRA)
#define DEVICE_NAME_STR         "C_Tester"
#define DEVICE_NAME_STR_SHORT   "CT"
#elif defined(PROJECT_CHAMELEON_LITE)
#define DEVICE_NAME_STR         "CL_TEST"
#define DEVICE_NAME_STR_SHORT   "CLT"
#else
#error "Unknown device name?"
#endif


/**
 * From 2byte version code merge to a U16 value,
 * like: 1(byte).0(byte) -> 1.0
 */
#define VER_CODE_TO_NUM(major, minor)   (((major << 8) & 0xFFFF) | (minor & 0xFF))


#endif
