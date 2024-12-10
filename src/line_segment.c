//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/line_segment.h"

#ifdef __cplusplus
extern "C" {
#endif

const unsigned int Segment_Indices_Default[2] = { 0, 1 };

CCTNum_t mathPointProjectionLine(const CCTNum_t p[3], const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], CCTNum_t np[3]) {
	CCTNum_t vp[3], dot;
	mathVec3Sub(vp, p, ls_v);
	dot = mathVec3Dot(vp, lsdir);
	mathVec3Copy(np, ls_v);
	mathVec3AddScalar(np, lsdir, dot);
	return dot;
}

void mathPointProjectionLine_v2(const CCTNum_t p[3], const CCTNum_t ls_v0[3], const CCTNum_t ls_v1[3], CCTNum_t np[3]) {
	CCTNum_t ls_v[3], vp[3], lensq, dot;
	mathVec3Sub(vp, p, ls_v0);
	mathVec3Sub(ls_v, ls_v1, ls_v0);
	lensq = mathVec3LenSq(ls_v);
	dot = mathVec3Dot(vp, ls_v);
	mathVec3Copy(np, ls_v0);
	mathVec3AddScalar(np, ls_v, dot / lensq);
}

CCTNum_t mathLineCrossLine(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3]) {
	CCTNum_t v[3], dot, lensq;
	mathPointProjectionLine(lsv1, lsv2, lsdir2, v);
	if (mathVec3Equal(lsv1, v)) {
		return CCTNum(0.0);
	}
	mathVec3Sub(v, v, lsv1);
	dot = mathVec3Dot(lsdir1, v);
	lensq = mathVec3LenSq(v);
	return lensq / dot;
}

int mathPointProjectionSegment(const CCTNum_t p[3], const CCTNum_t ls0[3], const CCTNum_t ls1[3], CCTNum_t np[3], CCTNum_t epsilon) {
	CCTNum_t ls_v[3], vp[3], lensq, dot;
	mathVec3Sub(vp, p, ls0);
	mathVec3Sub(ls_v, ls1, ls0);
	dot = mathVec3Dot(vp, ls_v);
	if (dot < -epsilon) {
		return 0;
	}
	lensq = mathVec3LenSq(ls_v);
	if (dot > lensq + epsilon) {
		return 0;
	}
	if (np) {
		mathVec3Copy(np, ls0);
		mathVec3AddScalar(np, ls_v, dot / lensq);
	}
	return 1;
}

#ifdef __cplusplus
}
#endif
