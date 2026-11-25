/* barrc_types.h - fixed-width type names required by BARR-C:2018 rule 5.4
 *
 * float32_t and float64_t name the IEEE 754 single and double precision
 * types used by the RP2040 toolchain. Integer types come from <stdint.h>.
 */
#ifndef BARRC_TYPES_H
#define BARRC_TYPES_H

#include <stdint.h>

typedef float float32_t;
typedef double float64_t;

#endif /* BARRC_TYPES_H */

/*** end of file ***/
