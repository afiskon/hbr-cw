#ifndef _SPURS_MASKING_H_
#define _SPURS_MASKING_H_

#include <stdint.h>
#include <stddef.h>

typedef struct {
	int32_t freq;
	int32_t clar;
	int32_t shift;
} SpurMaskInfo;

const SpurMaskInfo* getSpurMaskingInfo(int32_t freq);

#endif // _SPURS_MASKING_H_