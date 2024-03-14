//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_NUMBER_DEFINE_H
#define	UTIL_C_CRT_GEOMETRY_NUMBER_DEFINE_H

#ifdef CCT_NUM_FLOAT
	typedef	float					CCTNum_t;
	#define	CCTNum(n)				n##f
	#define	CCTNums_3(x, y, z)		x##f, y##f, z##f
	#define	CCTNums_4(x, y, z, w)	x##f, y##f, z##f, w##f

	#define	CCTNum_abs(n)			fabsf(n)
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

	#define	CCTNum_abs(n)			fabs(n)
	#define	CCTNum_sqrt(n)			sqrt(n)
	#define	CCTNum_acos(n)			acos(n)
	#define	CCTNum_cos(n)			cos(n)
	#define	CCTNum_sin(n)			sin(n)
	#define	CCTNum_atan2(y, x)		atan2(y, x)

#else
	#error	"CCT_NUM type isn't defined, should be float or double"
#endif

#ifndef CCT_EPSILON
	#define	CCT_EPSILON			CCTNum(1E-5)
	#define	CCT_EPSILON_NEGATE	CCTNum(-1E-5)
#endif

#endif
