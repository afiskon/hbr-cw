#include "spur_masking.h"

// Spurs seem to be inevitable in a 2-IF homebrew receiver. At least I didn't
// manage to get rid of them using filters or shields. They can be masked
// though, by changing CLAR and/or SHIFT just a bit. It is worth noticing that
// I live in a noisy area and my power supply is not particularly clean, so not
// all listed frequencies are necessarily real spurs.

// Also, consider tweaking IF2Frequency variable.

// Keep the list sorted by frequency, otherwise getSpurMaskingInfo()
// procedure will not work!
static const SpurMaskInfo FreqsWithSpurs[] = {
	// 80 meters
	{ 3500200, 0, 10 },
	{ 3501600, 50, 0 },
	{ 3501700, 50, 0 },
	{ 3503400, 50, 0 },
	{ 3503700, 0, 10 },
	{ 3503800, 0, 10 },
	{ 3504100, 0, 10 },
	{ 3505500, 0, -10 },
	{ 3505800, 0, -20 },
	{ 3505900, 50, -20 },
	{ 3506100, 0, -30 },
	{ 3506200, 0, -30 },
	{ 3506300, 0, -20 },
	{ 3506400, 0, 20 },
	{ 3506500, 0, 10 },
	{ 3506900, -50, 0 },
	{ 3507900, 0, -10 },
	{ 3510000, 0, -10 },
	{ 3512400, 0, -10 },
	{ 3513500, 50, 0 },
	{ 3517700, -50, 0 },
	{ 3520200, 0, -10 },
	{ 3520700, 0, -10 },
	{ 3521700, 0, 20 },
	{ 3521800, 50, 0 },
	{ 3522700, 0, -10 },
	{ 3525100, 0, -10 },
	{ 3526000, 0, -10 },
	{ 3527300, 50, 0 },
	{ 3527400, 50, 0 },
	{ 3528300, 50, 0 },
	{ 3530500, 0, -10 },
	{ 3530900, 50, 0 },
	{ 3533100, 0, -10 },
	{ 3533800, 0, -10 },
	{ 3534700, 0, -30 },
	{ 3534800, 0, -30 },
	{ 3535200, 0, 20 },
	{ 3535500, 0, 40 },
	{ 3536000, 0, 10 },
	// TODO: 3.537.4, 3.537.5: 2 freqs in total
	{ 3538500, 50, 0 },
	{ 3538700, 0, -10 },
	{ 3540600, 0, -10 },
	{ 3541400, 50, 0 },
	{ 3541900, 0, -10 },
	{ 3543100, 0, -10 },
	{ 3543900, 0, -10 },
	{ 3544000, 0, -10 },
	{ 3544100, 0, -10 },
	{ 3544200, 0, -10 },
	{ 3544300, 0, -10 },
	{ 3544400, 0, -10 },
	{ 3545100, 0, 10 },
	{ 3545500, 0, 10 },
	{ 3546700, 0, 10 },
	{ 3547400, 0, 10 },
	{ 3548500, 0, 10 },
	{ 3548700, 0, -30 },
	{ 3548900, 0, -30 },
	{ 3550800, 0, 10 },
	{ 3552100, 50, 10 },
	{ 3559200, 0, 20 },
	{ 3559600, 0, 30 },
	{ 3559700, 0, 30 },
	{ 3560600, 0, -10 },
	{ 3565400, 50, 0 },
	{ 3565700, 0, -10 },
	{ 3566700, 0, -10 },
	{ 3568200, 0, -10 },
	{ 3568600, 0, -10 },
	{ 3568900, 50, 0 },

	// 40 meters
	{ 7000000, 50, 0 },
	{ 7000200, 0, 20 },
	{ 7000600, 0, 10 },
	{ 7000800, 50, 0 },
	{ 7001000, 0, -20 },
	{ 7001300, 0, -30 },
	{ 7001400, 0, -30 },
	{ 7001700, 0, 10 },
	{ 7001900, 0, 10 },
	{ 7002500, 50, 10 },
	{ 7002700, 0, -20},
	{ 7002900, 0, -10},
	{ 7003100, 50, 0},
	{ 7003400, 0, -30},
	{ 7003500, 0, -30},
	{ 7003600, 0, -30},
	{ 7003800, 0, -10},
	{ 7004200, 0, -20},
	{ 7004400, 0, -20},
	{ 7005000, 0, -10},
	// TODO: 7.005.3-7.006.5: 12 freqs in total
	{ 7006700, 50, 10 },
	{ 7007600, 0, 10},
	{ 7007800, 0, 10},
	{ 7008600, 50, 0},
	{ 7009200, 0, -10},
	{ 7009400, -50, 0},
	{ 7010000, 0, -10},
	{ 7010700, 0, 10},
	{ 7011000, 0, -10},
	{ 7011200, 0, -10},
	{ 7012100, 50, 0},
	{ 7012700, 0, -20},
	{ 7012900, 0, -30},
	{ 7013500, 0, -10},
	{ 7015200, 0, -20},
	{ 7015300, 0, -20},
	{ 7017100, 0, -10},
	{ 7031000, 50, 0 },
	{ 7034000, 0, 10},
	{ 7034500, 0, 30},
	{ 7034600, 0, 30},
	{ 7034700, 0, 30},
	{ 7034800, 0, 30},
	{ 7034900, 0, 30},
	{ 7035000, 0, 30},
	{ 7035900, 0, 10},
	{ 7037200, 0, -30},
	{ 7037300, 0, -40},
	{ 7037500, 50, 0 },
	{ 7037800, 0, 20},
	{ 7037900, 0, 20},
	{ 7038200, 0, -10},

	// 30 meters
	{ 10108200, 0, -10},
	{ 10108900, 0, 10},
	{ 10110100, 50, 0 },
	{ 10110800, 0, -10 },
	{ 10112000, 0, -10 },
	{ 10113600, 0, 10 },
	{ 10114800, 0, 10 },
	{ 10115500, 0, 10 },
	{ 10115800, 0, -10 },
	{ 10116700, 0, 10 },
	{ 10117400, 50, 0 },
	{ 10118600, 0, 10 },
	{ 10119300, 50, 0 },
	{ 10119600, 0, -10 },
	{ 10120500, 0, 50 },
	{ 10121200, -50, 0 },
	{ 10122200, 50, 0 },
	// TODO: 10.122.4-10.126.2: 38 freqs in total
	/* { 10123100, 50, 0 },
	{ 10125700, 50, 0 }, */
	{ 10126600, 0, -10 },
	{ 10126900, 0, 20 },
	{ 10127400, 0, 20 },
	{ 10127800, 0, -10 },
	{ 10128800, 50, 0 },
	{ 10130000, 50, 0 },

	// 20 meters
	{ 14000500, 0, 20},
	{ 14000700, -50, 0},
	{ 14001600, 50, 0 },
	{ 14002200, 50, 0 },
	{ 14002800, 50, 0 },
	{ 14003700, 0, 20},
	{ 14003900, 50, 0 },
	{ 14004500, 50, 0 },
	{ 14004700, -50, 0 },
	// TODO: 14.004.9-14.006.1: 12 freqs in total
	/* { 14005000, 0, -20 },
	{ 14005200, 0, -10 },
	{ 14005300, 0, -10 },
	{ 14005800, 0, 10 }, */
	{ 14006200, 50, 0 },
	{ 14006400, 0, -10 },
	{ 14008700, 50, 0 },
	{ 14008900, 0, 10},
	{ 14009800, 0, 10},
	// TODO: 14.011.2-14.011.8: 6 freqs in total
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
	{ 14067800, 0, -20},
	{ 14068200, 0, 20},
	{ 14068400, 50, 0 },

	// 17 meters
	{ 18072100, 0, -20},
	{ 18072200, 0, -20},
	{ 18072300, 0, -20},
	{ 18072400, 0, -20},
	{ 18072500, 0, -20},
	{ 18072600, 0, -20},
	{ 18072700, 0, -20},
	{ 18072800, 0, -20},
	{ 18072900, 0, -20},
	{ 18073000, 0, -20},
	{ 18074200, 0, -10 },
	{ 18074300, 0, -10 },
	{ 18074400, 0, -10 },
	{ 18074500, 0, -10 },
	{ 18074600, 0, -10 },
	{ 18074700, 0, -10 },
	{ 18080600, 0, 20 },
	{ 18081800, 0, -20 },
	{ 18081900, 0, -20 },
	{ 18082000, 0, -20 },
	{ 18082100, 0, -20 },
	{ 18082200, 0, -20 },
	{ 18086700, 50, 0 },
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
	{ 18095000, 0, -10 },

	// 15 meters
	{ 21000000, 0, -20},
	{ 21000200, 0, 40},
	{ 21000300, 0, 40},
	{ 21000400, 0, 10},
	{ 21000600, 0, -20},
	{ 21000800, 0, -10},
	{ 21000900, 0, -10},
	{ 21001200, 0, -10},
	{ 21001300, 0, -10},
	{ 21001500, 0, 20},
	// TODO: 21.002.0 - 21.009.0: 70 freqs in total
	/* { 21002000, 0, 60},
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
	{ 21004800, 0, 20}, */
	{ 21009600, 0, -40},
	{ 21009700, 0, -40},
	{ 21009800, 0, -40},
	{ 21010200, 0, -40},
	{ 21010300, 0, -40},
	{ 21010400, 0, -40},
	{ 21011500, 0, -10},
	{ 21012400, 0, -20},
	{ 21012700, 0, -20},
	{ 21012900, 0, -10},
	{ 21014700, 0, -40},
	{ 21014800, 0, -40},
	{ 21014900, 0, -40},
	{ 21015200, 0, -10},
	{ 21015400, 0, -20},
	{ 21015500, 0, -20},
	// TODO 21.016.4-21.017.9: 15 freqs in total
	/* { 21016900, 0, -40},
	{ 21017000, 0, -40},
	{ 21017100, 0, -40},
	{ 21017200, 0, -40},
	{ 21017300, 0, -30},
	{ 21017900, 0, -20}, */

	{ 21019700, 0, -10},
	{ 21019800, 0, -10},
	{ 21020400, 0, -10},
	{ 21020500, 0, -10},
	{ 21023000, 0, -10},
	{ 21027300, 0, -10},
	{ 21027400, 0, -10},
	{ 21028000, 0, -10},
	{ 21028100, 0, -10},
	{ 21034900, 0, -10},
	{ 21037400, 0, -10},
	{ 21037500, 0, -10},
	{ 21064300, -50, 0 },

	// 12 meters
	{ 24894800, 50, 0 },
	{ 24904400, 50, 0 },
	{ 24909700, 50, 0 },
	{ 24910300, 50, 0 },
	{ 24910700, 50, 0 },
	{ 24913200, 50, 0 },
	{ 24914200, 50, 0 },

	// 10 meters
	{ 28000900, 0, -10 },
	{ 28002600, 50, 0 },
	{ 28002800, 50, -10 },
	{ 28003900, 0, -20},
	{ 28004000, 0, -20},
	{ 28004100, 0, -30},
	{ 28004500, 0, -10 },
	{ 28004600, 0, 20 },
	{ 28005100, 50, 0 },
	{ 28006200, 50, 0 },
	{ 28006800, 0, -10 },
	{ 28007000, 0, -10 },
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

bool isSpurMaskingInfoSorted(int32_t* first_mismatch) {
	int i;
	for(i = 1; i < sizeof(FreqsWithSpurs)/sizeof(FreqsWithSpurs[0]); i++) {
		if(FreqsWithSpurs[i].freq <= FreqsWithSpurs[i-1].freq) {
			*first_mismatch = FreqsWithSpurs[i].freq;
			return false;
		}
	}

	return true;
}

const SpurMaskInfo* getSpurMaskingInfo(int32_t freq) {
	// binary search
	int left = 0;
	int right = sizeof(FreqsWithSpurs)/sizeof(FreqsWithSpurs[0]) - 1;

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