#ifndef CRC8_H
#define CRC8_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**           Notes about CRC API
 * The ESP32 ROM include some CRC tables and CRC APIs to speed up CRC calculation.
 * The CRC APIs include CRC8, CRC16, CRC32 algorithms for both little endian and big endian modes.
 * Here are the polynomials for the algorithms:
 * CRC-8        x8+x2+x1+1                                              0x07
*/



/**
 * @brief CRC8 value in big endian.
 *
 * @param crc: Initial CRC value (result of last calculation or 0 for the first time)
 * @param buf: Data buffer that used to calculate the CRC value
 * @param len: Length of the data buffer
 * @return CRC8 value
 */
uint8_t crc8(uint8_t crc, uint8_t const *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif