//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_NUMBER_DEFINE_H
#define	UTIL_C_CRT_GEOMETRY_NUMBER_DEFINE_H

#include <math.h>

#ifdef CCT_NUM_FLOAT
	typedef	float					CCTNum_t;
	#define	CCTNum(n)				n##f
	#define	CCTNums_3(x, y, z)		x##f, y##f, z##f
	#define	CCTNums_4(x, y, z, w)	x##f, y##f, z##f, w##f

	#define	CCT_EPSILON				1e-5f
	#define	CCT_EPSILON_NEGATE		-1e-5f
	#define	CCT_GAP_DISTANCE		1e-3f

	#define	CCTNum_abs(n)			fabsf(n)
	#define	CCTNum_sq(n)			(n*n)
	#define	CCTNum_sqrt(n)			sqrtf(n)
	#define	CCTNum_acos(n)			acosf(n)
	#define	CCTNum_cos(n)			cosf(n)
	#define	CCTNum_sin(n)			sinf(n)
	#define	CCTNum_atan2(y, x)		atan2f(y, x)

#elif	CCT_NUM_DOUBLE
	typedef	double					CCTNum_t;
	#define	CCTNum(n)				n
	#define	CCTNums_3(x, y, z)		x, y, z
	#define	CCTNums_4(x, y, z, w)	x, y, z, w

	#define	CCT_EPSILON				1e-5
	#define	CCT_EPSILON_NEGATE		-1e-5
	#define	CCT_GAP_DISTANCE		1e-3

	#define	CCTNum_abs(n)			fabs(n)
	#define	CCTNum_sq(n)			(n*n)
	#define	CCTNum_sqrt(n)			sqrt(n)
	#define	CCTNum_acos(n)			acos(n)
	#define	CCTNum_cos(n)			cos(n)
	#define	CCTNum_sin(n)			sin(n)
	#define	CCTNum_atan2(y, x)		atan2(y, x)

#elif	CCT_NUM_LONG_DOUBLE
	typedef	long double				CCTNum_t;
	#define	CCTNum(n)				n##L
	#define	CCTNums_3(x, y, z)		x##L, y##L, z##L
	#define	CCTNums_4(x, y, z, w)	x##L, y##L, z##L, w##L

	#define	CCT_EPSILON				1e-5L
	#define	CCT_EPSILON_NEGATE		-1e-5L
	#define	CCT_GAP_DISTANCE		1e-3L

	#define	CCTNum_abs(n)			fabsl(n)
	#define	CCTNum_sq(n)			(n*n)
	#define	CCTNum_sqrt(n)			sqrtl(n)
	#define	CCTNum_acos(n)			acosl(n)
	#define	CCTNum_cos(n)			cosl(n)
	#define	CCTNum_sin(n)			sinl(n)
	#define	CCTNum_atan2(y, x)		atan2l(y, x)

#else
	#error	"CCTNum_t type isn't defined, should be float/double/long double ."
#endif

#endif
