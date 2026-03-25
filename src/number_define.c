//
// Created by hujianzhe
//

#include "../inc/number_define.h"
#include <stdlib.h>

static void* fn_calloc(const CCTAllocator_t* ac, size_t nitems, size_t size) { return calloc(nitems, size); }
static void* fn_malloc(const CCTAllocator_t* ac, size_t size) { return malloc(size); }
static void* fn_realloc(const CCTAllocator_t* ac, void* ptr, size_t size) { return realloc(ptr, size); }
static void fn_free(const CCTAllocator_t* ac, void* ptr) { free(ptr); }

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

const CCTAllocator_t* CCTAllocator_stdc(CCTAllocator_t* ac) {
	if (ac) {
		ac->fn_calloc = fn_calloc;
		ac->fn_malloc = fn_malloc;
		ac->fn_realloc = fn_realloc;
		ac->fn_free = fn_free;
		ac->ctx = NULL;
		return ac;
	}
	else {
		static const CCTAllocator_t s_stdc = {
			fn_calloc, fn_malloc, fn_realloc, fn_free, NULL
		};
		return &s_stdc;
	}
}

#ifdef __cplusplus
}
#endif
