//
// Created by hujianzhe
//

#include "../inc/number_define.h"

#ifdef __cplusplus
extern "C" {
#endif

int CCTNum_chkval(CCTNum_t num) {
	if (isinf(num) || isnan(num)) {
		return 0;
	}
	return 1;
}

int CCTNum_chkvals(const CCTNum_t* num, size_t cnt) {
	const CCTNum_t* end = num + cnt;
	if (num >= end) {
		return 0;
	}
	do {
		if (!CCTNum_chkval(*num)) {
			return 0;
		}
	} while (++num < end);
	return 1;
}

#ifdef __cplusplus
}
#endif