//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_NUMBER_DEFINE_H
#define	UTIL_C_CRT_GEOMETRY_NUMBER_DEFINE_H

#include "compiler_define.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

#ifdef CCT_NUM_FLOAT
	typedef	float					CCTNum_t;
	#define	CCTNum(n)				n##f
	#define	CCTNums_3(x, y, z)		x##f, y##f, z##f
	#define	CCTNums_4(x, y, z, w)	x##f, y##f, z##f, w##f

	#define	CCT_EPSILON				1e-5f
	#define	CCT_EPSILON_NEGATE		-1e-5f
	#define	CCT_GAP_DISTANCE		0.0142f

	#define	CCT_PI					3.1415927f
	#define	CCT_2PI					6.2831853f
	#define	CCT_INV_PI				0.31830987f
	#define	CCT_RADIAN_PER_DEGREE	0.017453292f
	#define	CCT_DEGREE_PER_RADIAN	57.295776f

	#define	CCTNum_abs(n)			fabsf(n)
	#define	CCTNum_sq(n)			((n)*(n))
	#define	CCTNum_sqrt(n)			sqrtf(n)
	#define	CCTNum_acos(n)			acosf(n)
	#define	CCTNum_cos(n)			cosf(n)
	#define	CCTNum_sin(n)			sinf(n)
	#define	CCTNum_atan2(y, x)		atan2f(y, x)
	#define	CCTNum_log(n,base)		(logf(n) / logf(base))
	#define	CCTNum_ceil(n)			ceilf(n)
	#define	CCTNum_floor(n)			floorf(n)
	#define	CCTNum_degree_to_radian(deg)	(CCT_RADIAN_PER_DEGREE*(deg))
	#define	CCTNum_radian_to_degree(rad)	(CCT_DEGREE_PER_RADIAN*(rad))

#elif	CCT_NUM_DOUBLE
	typedef	double					CCTNum_t;
	#define	CCTNum(n)				n
	#define	CCTNums_3(x, y, z)		x, y, z
	#define	CCTNums_4(x, y, z, w)	x, y, z, w

	#define	CCT_EPSILON				1e-7
	#define	CCT_EPSILON_NEGATE		-1e-7
	#define	CCT_GAP_DISTANCE		1e-3

	#define	CCT_PI					3.141592653589793
	#define	CCT_2PI					6.283185307179586
	#define	CCT_INV_PI				0.3183098861837907
	#define	CCT_RADIAN_PER_DEGREE	0.017453292519943295
	#define	CCT_DEGREE_PER_RADIAN	57.29577951308232

	#define	CCTNum_abs(n)			fabs(n)
	#define	CCTNum_sq(n)			((n)*(n))
	#define	CCTNum_sqrt(n)			sqrt(n)
	#define	CCTNum_acos(n)			acos(n)
	#define	CCTNum_cos(n)			cos(n)
	#define	CCTNum_sin(n)			sin(n)
	#define	CCTNum_atan2(y, x)		atan2(y, x)
	#define	CCTNum_log(n,base)		(log(n) / log(base))
	#define	CCTNum_ceil(n)			ceil(n)
	#define	CCTNum_floor(n)			floor(n)
	#define	CCTNum_degree_to_radian(deg)	(CCT_RADIAN_PER_DEGREE*(deg))
	#define	CCTNum_radian_to_degree(rad)	(CCT_DEGREE_PER_RADIAN*(rad))

#elif	CCT_NUM_LONG_DOUBLE
	typedef	long double				CCTNum_t;
	#define	CCTNum(n)				n##L
	#define	CCTNums_3(x, y, z)		x##L, y##L, z##L
	#define	CCTNums_4(x, y, z, w)	x##L, y##L, z##L, w##L

	#define	CCT_EPSILON				1e-7L
	#define	CCT_EPSILON_NEGATE		-1e-7L
	#define	CCT_GAP_DISTANCE		1e-3L

	#define	CCT_PI					3.141592653589793238L
	#define	CCT_2PI					6.283185307179586476L
	#define	CCT_INV_PI				0.3183098861837906715L
	#define	CCT_RADIAN_PER_DEGREE	0.017453292519943295767L
	#define	CCT_DEGREE_PER_RADIAN	57.29577951308232088L

	#define	CCTNum_abs(n)			fabsl(n)
	#define	CCTNum_sq(n)			((n)*(n))
	#define	CCTNum_sqrt(n)			sqrtl(n)
	#define	CCTNum_acos(n)			acosl(n)
	#define	CCTNum_cos(n)			cosl(n)
	#define	CCTNum_sin(n)			sinl(n)
	#define	CCTNum_atan2(y, x)		atan2l(y, x)
	#define	CCTNum_log(n,base)		(logl(n) / logl(base))
	#define	CCTNum_ceil(n)			ceill(n)
	#define	CCTNum_floor(n)			floorl(n)
	#define	CCTNum_degree_to_radian(deg)	(CCT_RADIAN_PER_DEGREE*(deg))
	#define	CCTNum_radian_to_degree(rad)	(CCT_DEGREE_PER_RADIAN*(rad))

#else
	#error	"CCTNum_t type isn't defined, should be float/double/long double ."
#endif

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll int CCTNum_chkval(CCTNum_t num);
__declspec_dll int CCTNum_chkvals(const CCTNum_t* num, size_t cnt);

#ifdef __cplusplus
}
#endif

#endif
