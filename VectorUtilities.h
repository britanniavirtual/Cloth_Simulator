#pragma once

/////////////////////////////////////////////////////////////////////////
// Self-contained 3D vector class.Utility functions in ::Vector3DUtils
/////////////////////////////////////////////////////////////////////////
class Vector3D
{
public:

	double x, y, z;//The 3 VEC3 floats

	Vector3D(double x, double y, double z)//Constructor
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3D()//Overloaded Constructor
	{
		x = 0;
		y = 0;
		z = 0;
	}

	//[Mathematics operations]

	//Add two vectors
	Vector3D operator+(const Vector3D& inputVector) const
	{
		return Vector3D(x + inputVector.x, y + inputVector.y, z + inputVector.z);
	}

	//Subtract two vectors
	Vector3D operator-(const Vector3D& inputVector) const
	{
		return Vector3D(x - inputVector.x, y - inputVector.y, z - inputVector.z);
	}

	//Multiply two vectors
	Vector3D operator*(const Vector3D& inputVector) const
	{
		return Vector3D(x * inputVector.x, y * inputVector.y, z * inputVector.z);
	}

	//Divide two vectors
	Vector3D operator/(const Vector3D& inputVector) const
	{
		return Vector3D(x / inputVector.x, y / inputVector.y, z / inputVector.z);
	}

	//Add floating point to vector
	Vector3D operator+(double f) const
	{
		return Vector3D(f + x, f + y, f + z);
	}

	//Subtract floating point from vector
	Vector3D operator-(double f) const
	{
		return Vector3D(f - x, f - y, f - z);
	}

	//Multiply vector by floating point
	Vector3D operator*(double f) const
	{
		return Vector3D(f*x, f*y, f*z);
	}

	//Divide vector by floating point
	Vector3D operator/(double f) const
	{
		return Vector3D(f / x, f / y, f / z);
	}

	//[Cumulative operations]
	void operator+=(const Vector3D& inputVector)
	{
		this->x = inputVector.x + this->x;
		this->y = inputVector.y + this->y;
		this->z = inputVector.z + this->z;
	}

	void operator-=(const Vector3D& inputVector)
	{
		this->x -= inputVector.x + this->x;
		this->y -= inputVector.y + this->y;
		this->z -= inputVector.z + this->z;
	}

	void operator*=(const Vector3D& inputVector)
	{
		this->x = inputVector.x * this->x;
		this->y = inputVector.y * this->y;
		this->z = inputVector.z * this->z;
	}

	void operator/=(const Vector3D& inputVector)
	{
		this->x = this->x / inputVector.x;
		this->y = this->y / inputVector.y;
		this->z = this->z / inputVector.z;
	}

	void operator*=(double f)
	{
		this->x = f * this->x;
		this->y = f * this->y;
		this->z = f * this->z;
	}

	void operator/=(double f)
	{
		this->x = this->x / f;
		this->y = this->y / f;
		this->z = this->z / f;
	}
};



/////////////////////////////////////////////////////////////////////////
// Vector Utilties
/////////////////////////////////////////////////////////////////////////
class Vector3DUtils
{
public:

	//Euclidean distance
	float dist(float x1, float y1, float z1, float x2, float y2, float z2);

	//Cross product of two Vector3D's.
	Vector3D cross(const Vector3D& A, const Vector3D& B);

	//Dot product
	double dot(Vector3D input1, const Vector3D& input2);

	//Vector plane intersect
	Vector3D intersectPoint(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint);

	//Get arbitrary 3d vector that is perpendicular to the parameter vector	
	//There are infinite such vectors, return one such.
	Vector3D arbitraryOrthogonal(Vector3D vec);

	//Use spherical coordinates to get a position
	Vector3D OrbitalPosition(float angle1, float angle2, Vector3D centroid);

	//Set the length (magnitude) of a given vector
	Vector3D setVectorMagnitude(Vector3D input, float newMag);

	//Get magnitude of vector
	double length(Vector3D vec);

	//Linear interpolation
	Vector3D lerp(Vector3D a, Vector3D b, float scale);

	//Move vector a towards b by a set amount
	Vector3D displaceVectorTowards(Vector3D a, Vector3D b, float amount);

	//Scalar difference between two 3D vectors
	float angularDifference(Vector3D a, Vector3D b);

	Vector3D normalize(Vector3D vec);

	bool RayTriangleIntersect(Vector3D rayOrigin, Vector3D rayVector, Vector3D* v1, Vector3D* v2, Vector3D* v3, Vector3D& outIntersectionPoint);//Möller–Trumbore intersection algorithm

	bool LineTriangleIntersect(Vector3D lineStart, Vector3D lineEnd, Vector3D v1, Vector3D v2, Vector3D v3, Vector3D* outIntersectionPoint);

	//float triangleDistance(float IntersectPointA[3], float IntersectPointB[3], Vector3D pA1, Vector3D pA2, Vector3D pA3, Vector3D pB1, Vector3D pB2, Vector3D pB3);//Minimum distance between two triangles

	bool vectorPointsTowards(Vector3D centroid, Vector3D a, Vector3D b);//Return if two vectors point towards or away

	Vector3D closestPlanePoint(Vector3D pointPosition, Vector3D planePosition, Vector3D planeNormal);

	Vector3D getTriangleNormal(Vector3D A, Vector3D B, Vector3D C);

	Vector3D vectorToPoint(Vector3D startPoint, Vector3D endPoint);

	bool pointWithinTriangle(Vector3D triV1, Vector3D triV2, Vector3D triV3, Vector3D point);
};