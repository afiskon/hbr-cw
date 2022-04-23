#include "spur_masking.h"

// Several spurs seem to be inevitable in a 2-IF homebrew receiver.
// At least I didn't manage to get rid of them using filters or shields.
// They can be easily masked though, by changing CLAR of SHIFT jist a bit.
// It is worth noticing that I live in quite a noisy area and my power
// supply is not particularly clean, so not all listed frequencies are
// necessarily real spurs.
// Also, consider tweaking IF2Frequency variable.

static const SpurMaskInfo FreqsWithSpurs[] = {
	// 80 meters
	{ 3500200, 0, 10 },
	{ 3501600, 50, 0 },
	{ 3501700, 50, 0 },
	{ 3503700, 0, 10 },
	{ 3503800, 0, 10 },
	{ 3505900, 50, 0 },
	{ 3521800, 50, 0 },
	{ 3527300, 50, 0 },
	{ 3527400, 50, 0 },
	{ 3528300, 50, 0 },
	{ 3530900, 50, 0 },
	{ 3538500, 50, 0 },
	{ 3541400, 50, 0 },
	{ 3552100, 50, 0 },
	{ 3565400, 50, 0 },

	// 40 meters
	{ 7000800, 50, 0 },
	{ 7002500, 50, 0 },
	{ 7006700, 50, 0 },
	{ 7031000, 50, 0 },
	{ 7037500, 50, 0 },

	// 30 meters
	{ 10110100, 50, 0 },
	{ 10117400, 50, 0 },
	{ 10119300, 50, 0 },
	{ 10123100, 50, 0 },
	{ 10128800, 50, 0 },
	{ 10130000, 50, 0 },

	// 20 meters
	{ 14001600, 50, 0 },
	{ 14002200, 50, 0 },
	{ 14003900, 50, 0 },
	{ 14004500, 50, 0 },
	{ 14004700, -50, 0 },
	{ 14005200, 0, -10 },
	{ 14005300, 0, -10 },
	{ 14005800, 0, 10 },
	{ 14006200, 50, 0 },
	{ 14006400, 0, -10 },
	{ 14002800, 50, 0 },
	{ 14008700, 50, 0 },
	{ 14035100, 50, 0 },
	{ 14046000, 50, 0 },
	{ 14056100, 50, 0 },
	{ 14056700, 50, 0 },
	{ 14059900, 0, 10 },
	{ 14060500, 50, 0 },
	{ 14061900, 50, 0 },
	{ 14063200, 50, 0 },
	{ 14063800, 50, 0 },
	{ 14064600, 50, 0 },
	{ 14065100, 50, 0 },
	{ 14065700, 50, 0 },
	{ 14067000, 50, 0 },
	{ 14067600, 50, 0 },
	{ 14068400, 50, 0 },

	// 17 meters
	{ 18086700, 50, 0 },
	{ 18074200, 0, -10 },
	{ 18074300, 0, -10 },
	{ 18074400, 0, -10 },
	{ 18074500, 0, -10 },
	{ 18074600, 0, -10 },
	{ 18074700, 0, -10 },
	{ 18080600, 0, 20 },
	{ 18087300, 0, -10 },
	{ 18087400, 0, -10 },
	{ 18087500, 0, -10 },
	{ 18087600, 0, -10 },
	{ 18087700, 0, -10 },
	{ 18089400, 0, -10 },
	{ 18089500, 0, -10 },
	{ 18089600, 0, -10 },
	{ 18089700, 0, -10 },
	{ 18089800, 0, -10 },

	// 15 meters
	{ 21000000, 0, -20},
	{ 21000200, 0, 40},
	{ 21000300, 0, 40},
	{ 21000400, 0, 10},
	{ 21000600, 0, -20},
	{ 21000800, 0, -10},
	{ 21001200, 0, -10},
	{ 21001300, 0, -10},
	{ 21002000, 0, 60},
	{ 21002100, 0, 60},
	{ 21002200, 0, 60},
	{ 21002300, 0, 10},
	{ 21002500, 0, -10},
	{ 21002600, 0, -10},
	{ 21002700, 0, 50},
	{ 21002800, 0, 40},
	{ 21002900, 0, -60},
	{ 21003800, 0, -30},
	{ 21003900, 0, -30},
	{ 21004000, 0, -50},
	{ 21004100, 0, -50},
	{ 21004200, 0, -150},
	{ 21004300, 0, -170},
	{ 21004400, 0, -200},
	{ 21004500, 0, 50},
	{ 21004600, 0, 40},
	{ 21004700, 0, 30},
	{ 21004800, 0, 20},
	// TODO: 21.005.0 - 21.008.0
	{ 21064300, -50, 0 },

	{ 21009600, 0, -40},
	{ 21009700, 0, -40},
	{ 21009800, 0, -40},
	{ 21010200, 0, -40},
	{ 21010300, 0, -40},
	{ 21010400, 0, -40},
	{ 21012900, 0, -10},
	{ 21014700, 0, -40},
	{ 21014800, 0, -40},
	{ 21014900, 0, -40},
	{ 21015400, 0, -20},
	{ 21015500, 0, -20},
	{ 21019700, 0, -10},
	{ 21019800, 0, -10},
	{ 21020400, 0, -10},
	{ 21020500, 0, -10},

	// 12 meters
	{ 24894800, 50, 0 },
	{ 24904400, 50, 0 },
	{ 24909700, 50, 0 },
	{ 24910300, 50, 0 },
	{ 24910700, 50, 0 },
	{ 24913200, 50, 0 },
	{ 24914200, 50, 0 },

	// 10 meters
	{ 28002600, 50, 0 },
	{ 28002800, 50, 0 },
	{ 28004500, 0, -10 },
	{ 28005100, 50, 0 },
	{ 28006200, 50, 0 },
	{ 28007600, -50, 0 },
	{ 28008700, 50, 0 },
	{ 28009300, 50, 0 },
	{ 28015500, 50, 0 },
	{ 28024300, 50, 0 },
	{ 28033900, 50, 0 },
	{ 28037900, 50, 0 },
	{ 28040700, 50, 0 },
	{ 28050400, 50, 0 },
	{ 28051600, 50, 0 },
	{ 28052500, 50, 0 },
	{ 28053400, 50, 0 },
	{ 28054300, 50, 0 },
	{ 28054400, 50, 0 },
	{ 28058300, 50, 0 },
	{ 28059200, 50, 0 },
	{ 28060200, 50, 0 },
}; 


const SpurMaskInfo* getSpurMaskingInfo(int32_t freq) {
	// binary search
	int left = 0;
	int right = sizeof(FreqsWithSpurs)/sizeof(FreqsWithSpurs[0]);

	while(left <= right) {
		int current = (left + right) / 2;
		if(FreqsWithSpurs[current].freq == freq) {
			return &FreqsWithSpurs[current];
		}

		if(FreqsWithSpurs[current].freq > freq) {
			right = current - 1;
		} else {
			left = current + 1;
		}
	}

	/* not found */
	return NULL;
}