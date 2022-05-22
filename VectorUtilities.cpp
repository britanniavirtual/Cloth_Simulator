#include <iostream>
#include <sstream>

#include "JointHeader.h"

using namespace std;

//Euclidean distance
float Vector3DUtils::dist(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return(sqrt(pow((float)(x2 - x1), (float)2) + pow((float)(y2 - y1), (float)2) + pow((float)(z2 - z1), (float)2)));
}

//Cross product of two vector array.
Vector3D Vector3DUtils::cross(const Vector3D& A, const Vector3D& B)
{
	Vector3D crossP(0, 0, 0);
	crossP.x = A.y * B.z - A.z * B.y;

	crossP.y = A.z * B.x - A.x * B.z;
	crossP.z = A.x * B.y - A.y * B.x;

	return crossP;
}

float Vector3DUtils::angularDifference(Vector3D a, Vector3D b)
{
	float angle = acos(dot(a, b) / (length(a)*length(b)));
	return angle;
}



double Vector3DUtils::dot(Vector3D input1, const Vector3D& input2)
{
	return input1.x * input2.x + input1.y * input2.y + input1.z * input2.z;
}

Vector3D Vector3DUtils::intersectPoint(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint)
{
	Vector3D diff = rayPoint - planePoint;
	double prod1 = dot(diff, planeNormal);
	double prod2 = dot(rayVector, planeNormal);
	double prod3 = prod1 / prod2;
	return rayPoint - rayVector * prod3;
}

//Get arbitrary 3d vector that is perpendicular to the parameter vector
//There are infinite such vectors, get one such
Vector3D Vector3DUtils::arbitraryOrthogonal(Vector3D vec)
{
	bool b0 = (vec.x < vec.y) && (vec.x < vec.z);
	bool b1 = (vec.y <= vec.x) && (vec.y < vec.z);
	bool b2 = (vec.z <= vec.x) && (vec.z <= vec.y);

	Vector3D op(0, 0, 0);
	op = cross(vec, Vector3D(int(b0), int(b1), int(b2)));

	return op;
}

//Use spherical coordinates to get a position
Vector3D Vector3DUtils::OrbitalPosition(float angle1, float angle2, Vector3D centroid)
{
	float sx = centroid.x;// -0.013;
	float sy = centroid.y;// 1.06;
	float sz = centroid.z;// 1.06;

	float Theta = angle1;
	float Phi = angle2;
	float radius = 1.0;
	float Y = radius * sin(Theta);
	float X = radius * cos(Theta) * cos(Phi);
	float Z = radius * cos(Theta) * sin(Phi);

	return Vector3D(X + sx, Y + sy, Z + sz);
}

//Set the length (magnitude) of a given vector
Vector3D Vector3DUtils::setVectorMagnitude(Vector3D input, float newMag)
{
	float mag = sqrt(input.x * input.x + input.y * input.y + input.z * input.z);

	float new_x = input.x * newMag / mag;
	float new_y = input.y * newMag / mag;
	float new_z = input.z * newMag / mag;

	Vector3D op(new_x, new_y, new_z);
	return op;
}


Vector3D Vector3DUtils::lerp(Vector3D a, Vector3D b, float scale)
{
	Vector3D op0(0, 0, 0);

	//[End-Start]
	op0.x = b.x - a.x;
	op0.y = b.y - a.y;
	op0.z = b.z - a.z;
	//[Multiply by scale]
	op0 *= scale;

	Vector3D op1(0, 0, 0);
	op1.x = a.x + op0.x;
	op1.y = a.y + op0.y;
	op1.z = a.z + op0.z;

	return(op1);
}


Vector3D Vector3DUtils::displaceVectorTowards(Vector3D a, Vector3D b, float amount)
{
	if (utils.dist(a.x, a.y, a.z, b.x, b.y, b.z) <= 0.0)
	{
		//Vector3D op4(256, 256, 256);
		return(a);
	}

	Vector3D op0(0, 0, 0);

	//[End-Start]
	op0.x = b.x - a.x;
	op0.y = b.y - a.y;
	op0.z = b.z - a.z;
	
	Vector3D op1(op0.x, op0.y, op0.z);
	Vector3D vi(op1.x, op1.y, op1.z);
	float vLen0 = length(vi);
	float vLen1 = 1 / vLen0;//Amount to scale to increase by 1

	Vector3D op3(op1.x, op1.y, op1.z);
	op3 *= vLen1 * amount;

	Vector3D op2(0, 0, 0);
	op2.x = a.x + op3.x;
	op2.y = a.y + op3.y;
	op2.z = a.z + op3.z;

	return(op2);
}

double Vector3DUtils::length(Vector3D vec)
{
	return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

Vector3D Vector3DUtils::normalize(Vector3D vec)
{
	float op1 = pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2);
	op1 = sqrt(op1);

	Vector3D op;
	op.x = vec.x / op1;
	op.y = vec.y / op1;
	op.z = vec.z / op1;

	return op;
}


//Algorithm to test if the line actually reaches rather than extending to infinity
//Get position of intersect, if it lies between start and end of the line then its a hit
//Get Top XYZ and Bottom XYZ

Vector3D vertex0;
Vector3D vertex1;
Vector3D vertex2;
Vector3D edge1, edge2, h, s, q;
const float EPSILON = 0.0000001;
Vector3D rayVector;
bool Vector3DUtils::LineTriangleIntersect(Vector3D lineStart, Vector3D lineEnd, Vector3D v1, Vector3D v2, Vector3D v3, Vector3D* outIntersectionPoint)
{
	rayVector = vector3DUtils.normalize(lineEnd - lineStart);

	//Vector3D rayVector(0, 0, -1);

	vertex0 = v1;
	vertex1 = v2;
	vertex2 = v3;

	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;

	
	//h = rayVector.crossProduct(edge2);
	h = cross(rayVector, edge2);

	//a = edge1.dotProduct(h);
	a = dot(edge1, h);

	if (a > -EPSILON && a < EPSILON)
		return false;//This ray is parallel to this triangle.

	f = 1.0 / a;
	s = lineStart - vertex0;
	u = f * dot(s, h);//s.dotProduct(h);
	if (u < 0.0 || u > 1.0)
		return false;

	q = cross(s, edge1);//s.crossProduct(edge1);
	v = f * dot(rayVector, q);//rayVector.dotProduct(q);
	if (v < 0.0 || u + v > 1.0)
		return false;

	//At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * dot(edge2, q);//edge2.dotProduct(q);
	if (t > EPSILON)//ray intersection
	{
		*outIntersectionPoint = lineStart + rayVector * t;
		//return true;
	}
	else//No ray intersection
	{
		return false;
	}

	float d1 = dist(lineStart.x, lineStart.y, lineStart.z, outIntersectionPoint->x, outIntersectionPoint->y, outIntersectionPoint->z);
	float d2 = dist(lineEnd.x, lineEnd.y, lineEnd.z, outIntersectionPoint->x, outIntersectionPoint->y, outIntersectionPoint->z);
	
	bool furthest = 0;
	if (d1 < d2) { furthest = 1; }

	float lineLength = dist(lineStart.x, lineStart.y, lineStart.z, lineEnd.x, lineEnd.y, lineEnd.z);

	if (furthest == 0)//D2
	{	
		if (lineLength > d1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (lineLength > d2)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

//Möller–Trumbore intersection algorithm. Fastest.
bool Vector3DUtils::RayTriangleIntersect(Vector3D rayOrigin, Vector3D rayVector, Vector3D* v1, Vector3D* v2, Vector3D* v3, Vector3D& outIntersectionPoint)
{
	const float EPSILON = 0.0000001;
	Vector3D vertex0 = *v1;
	Vector3D vertex1 = *v2;
	Vector3D vertex2 = *v3;
	Vector3D edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;

	//h = rayVector.crossProduct(edge2);
	h = cross(rayVector, edge2);

	//a = edge1.dotProduct(h);
	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.

	f = 1.0 / a;
	s = rayOrigin - vertex0;
	u = f * dot(s, h);//s.dotProduct(h);
	if (u < 0.0 || u > 1.0)
		return false;

	q = cross(s, edge1);// s.crossProduct(edge1);
	v = f * dot(rayVector, q);// rayVector.dotProduct(q);
	if (v < 0.0 || u + v > 1.0)
		return false;

	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * dot(edge2, q);// edge2.dotProduct(q);
	if (t > EPSILON) // ray intersection
	{
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}



Vector3D Vector3DUtils::getTriangleNormal(Vector3D a, Vector3D b, Vector3D c)
{
	//This seems to work now!! Needs to be added to centroid!
	Vector3D A = b - a;
	Vector3D B = c - a;

	Vector3D N;
	N.x = A.y * B.z - A.z * B.y;
	N.y = A.z * B.x - A.x * B.z;
	N.z = A.x * B.y - A.y * B.x;

	/*
	Vector3D N(0, 0, 0);

	N.x = a.y * b.z - a.z * b.y;
	N.y = a.z * b.x - a.x * b.z;
	N.z = a.x * b.y - a.y * b.x;
	*/

	return N;
}

Vector3D Vector3DUtils::vectorToPoint(Vector3D startPoint, Vector3D endPoint)
{
	return endPoint - startPoint;
}


Vector3D Vector3DUtils::closestPlanePoint(Vector3D pointPosition, Vector3D planePosition, Vector3D planeNormal)
{
	float sb, sn, sd;

	Vector3D d1 = pointPosition - planePosition;
	sn = -vector3DUtils.dot(planeNormal, d1);
	sd = vector3DUtils.dot(planeNormal, planeNormal);

	sb = sn / sd;

	//sn = -Vector3D.Dot(planeNormal, (pointPosition - planePosition));
	//sd = Vector3D.Dot(planeNormal, planeNormal);
	//sb = sn / sd;

	Vector3D result = pointPosition + (planeNormal * sb);

	return result;
}

//-------------------------------------
//
//-------------------------------------
bool Vector3DUtils::vectorPointsTowards(Vector3D centroid, Vector3D a, Vector3D b)
{
	Vector3D an = normalize(a - centroid);
	Vector3D bn = normalize(b - centroid);

	if (dot(an, bn) < 0)
	{
		return 0;
	}

	return 1;
}


///////////////////////////////////////////
//Point triangle distance
///////////////////////////////////////////
template <typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

//-------------------------------------
//
//-------------------------------------
float dot2(Vector3D v) { return vector3DUtils.dot(v, v); }

//-------------------------------------
//
//-------------------------------------
double clamp(double d, double min, double max)
{
	const double t = d < min ? min : d;
	return t > max ? max : t;
}


bool Vector3DUtils::pointWithinTriangle(Vector3D triV1, Vector3D triV2, Vector3D triV3, Vector3D point)
{
	//Area of triangle
	float distX = triV1.x - triV2.x;
	float distY = triV1.y - triV2.y;
	float distZ = triV1.z - triV2.z;

	float edgeLength1 = sqrt(distX*distX + distY * distY + distZ * distZ);

	distX = triV1.x - triV3.x;
	distY = triV1.y - triV3.y;
	distZ = triV1.z - triV3.z;

	float edgeLength2 = sqrt(distX*distX + distY * distY + distZ * distZ);

	distX = triV2.x - triV3.x;
	distY = triV2.y - triV3.y;
	distZ = triV2.z - triV3.z;

	float edgeLength3 = sqrt(distX*distX + distY * distY + distZ * distZ);

	float s = (edgeLength1 + edgeLength2 + edgeLength3) / 2.0f;

	float mainTriArea = sqrt(s*(s - edgeLength1)*(s - edgeLength2)*(s - edgeLength3));

	float smallTriArea[3] = { 0.0f, 0.0f, 0.0f };

	Vector3D triVert[4];
	triVert[0] = triV1;
	triVert[1] = triV2;
	triVert[2] = triV3;
	triVert[3] = triV1;

	//Find 3 triangle areas using the plane intersecting point
	for (int i = 0; i < 3; i++)
	{
		distX = point.x - triVert[i].x;
		distY = point.y - triVert[i].y;
		distZ = point.z - triVert[i].z;

		edgeLength1 = sqrt(distX*distX + distY * distY + distZ * distZ);

		distX = point.x - triVert[i + 1].x;
		distY = point.y - triVert[i + 1].y;
		distZ = point.z - triVert[i + 1].z;

		edgeLength2 = sqrt(distX*distX + distY * distY + distZ * distZ);

		distX = triVert[i].x - triVert[i + 1].x;
		distY = triVert[i].y - triVert[i + 1].y;
		distZ = triVert[i].z - triVert[i + 1].z;

		edgeLength3 = sqrt(distX*distX + distY * distY + distZ * distZ);

		s = (edgeLength1 + edgeLength2 + edgeLength3) / 2.0f;

		smallTriArea[i] = sqrt(s*(s - edgeLength1)*(s - edgeLength2)*(s - edgeLength3));
	}

	float totalSmallTriArea = smallTriArea[0] + smallTriArea[1] + smallTriArea[2];

	if (mainTriArea >= (totalSmallTriArea - 0.001f))
	{
		return true;
	}

	return false;
}