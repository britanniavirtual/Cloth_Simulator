#pragma once
class TriangleIntersect
{

public:

	int tri_tri_overlap_test_2d(float p1[2], float q1[2], float r1[2], float p2[2], float q2[2], float r2[2]);
	int ccw_tri_tri_intersection_2d(float p1[2], float q1[2], float r1[2], float p2[2], float q2[2], float r2[2]);
	int tri_tri_intersection_test_3d(float p1[3], float q1[3], float r1[3], float p2[3], float q2[3], float r2[3], int * coplanar, float source[3], float target[3]);
	int coplanar_tri_tri3d(float p1[3], float q1[3], float r1[3], float p2[3], float q2[3], float r2[3], float normal_1[3], float normal_2[3]);
	int triangleIntersect(float p1[3], float q1[3], float r1[3], float p2[3], float q2[3], float r2[3]);
};

