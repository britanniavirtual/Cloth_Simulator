#pragma once

class Spring
{
public:
	int p1, p2;
	float rest_length;
	float Ks, Kd;
	int type;
};

class TriangleIndex
{
public:
	int a;//Triangle point index from the cloth array
	int b;
	int c;
};

//Very simple per-vertex turbulence
class TurbulenceRegion
{
public:

	TurbulenceRegion()
	{
		float f = 0 - 360;
		tgtAngle1 = float(rand()) / float((RAND_MAX)) * f;
	}

	int vertex;

	Vector3D windVec;

	Vector3D newWind;

	float windDirectionSphericalCoordinatesX = 0;
	float windDirectionSphericalCoordinatesY = 0;// [0, -90 : back facing wind]
	float directionVariation = 360;
	float windStrength = 4.2;
	float directionChangeDelay = 10;
	float maxWindStrength = 10.5;
	float minWindStrength = 3;
	float strengthChangeTime = 100;
	float strengthVariation = 0.0;

	float lastTurbulenceTime = 0.0;
	float directionTime = 0.0;
	float strengthTime = 0.0;

	float elapsedTurbulenceTime;

	float tgtAngle1 = 0;
	float tgtAngle2 = 0;
	float curAngle1 = 0;
	float curAngle2 = 0;

	bool setAngle = false;

	Vector3D turb;
};

class Triangle
{
public:
	Vector3D p1;
	Vector3D p2;
	Vector3D p3;

	Triangle()//Constructor
	{
	}

	Triangle(Vector3D t1, Vector3D t2, Vector3D t3)//Constructor
	{
		this->p1 = t1;
		this->p2 = t2;
		this->p3 = t3;
	}
};

class Sphere
{
public:
	float radius;
	Vector3D position;
};

class Bounds
{
public:
	float maxX;
	float maxY;
	float maxZ;
	float minX;
	float minY;
	float minZ;
};

class Cube
{
public:
	Vector3D p1;
	Vector3D p2;
	Vector3D p3;
	Vector3D p4;

	Vector3D p5;
	Vector3D p6;
	Vector3D p7;
	Vector3D p8;

	void translate(int axis, float amount);
};

struct MeshVoxel//A divided-up voxel containing TRIANGLES. This object is considered in collision detection.
{
	Bounds bounds;
	vector <Triangle> triangles;//Array of triangles
	Vector3D centroid;
};

struct VertVoxel//A divided-up voxel containing VERTICES. This object is considered in collision detection.
{
	Bounds bounds;
	vector <Vector3D> vertices;//Array of triangles
	Vector3D centroid;
};

//Bounds for every collision mesh triangle
struct MeshTriangle
{
	Bounds bounds;
	Triangle triangle;
	Vector3D centroid;
};

class ClothQuad
{
public:

	int A;//Vertices in the cloth mesh that make up this quad
	int B;
	int C;
	int D;
};

//Step the simulation / Initialize the cloth
class ClothWorld
{
public:
	
	ClothWorld()
	{
		gravity.x = 0;
		gravity.y = -9.81;
		gravity.z = 0;
	}
	void initPins();

	vector<int> removeDuplicates(vector<int> input, int ignore);

	//Get the points which make up quads of the cloth mesh (Ie: Convert from tri mesh)
	//NB: The topology of the cloth must support this
	ClothQuad clothQuads[2048];
	int clothQuadCount = 0;

	//[Compute patches on init]
	vector<int> clothPatch[655336];
	int clothPatchCount[65536];
	

	//[New cloth structure]
	//Used for both meshes and patches
	//Only the points are moved by the engine
	int clothIndices2[2048];
	Vector3D clothPoints2[2048];
	int pointCount2 = 0;
	int indicesCount2 = 0;
	//

	Vector3D nvec1;
	void sinWave();
	Vector3D sinPos[65536];

	bool isPinned(int vertex);//Return if the paramater vertex is pinned

	//[Compute the bounds of every cmesh triangle]
	vector <MeshTriangle> CMeshTriangleBounds;
	void computeCMeshTriangleBounds();
	//[Store which faces of the cloth intersect with what]
	int clothCollisionMap[1024][1024];

	Triangle triangle2;
	Triangle closestCMTriangle;
	Vector3D closestCMPointToCloth;
	float closestCMDist;

	TurbulenceRegion turbulence[65536];//One turbulence region for each vertex
	void computeTurbulence(float et);

	vector <Triangle> getBoundedCMTriangles(Vector3D p1, Vector3D p2, Vector3D p3);

	vector <Vector3D> getBoundedCMPoints(Vector3D p1, Vector3D p2, Vector3D p3);


	Vector3D closestCMPoint;
	Vector3D closestClothPoint;

	const float constStep = 0.005;//0.005;//MAIN PHYSICS STEP AMOUNT

	vector <Vector3D> getCMVertices(Bounds boundsA, int *voxelRef);

	float lastDists1[65536];//Last distance above the threshold
	float lastDists2[65536];

	void stepClothWorld();

	Triangle getNearestCMTriangle(Vector3D position);//Return nearest triangle on the CM

	const float DEFAULT_DAMPING = -0.0125f;
	float KsStruct = 0.75f, KdStruct = -0.25f;
	float KsShear = 0.75f, KdShear = -0.25f;
	float KsBend = 0.95f, KdBend = -0.25f;

	Vector3D vectorsToDraw[65536];
	Vector3D vectorsToDraw_Type2[65536];
	int vectorsToDrawCount = 0;
	int vectorsToDrawCount_Type2 = 0;

	float stepDist2 = 0.0;

	float lastTime = 0.0;
	float lastWindTime = 0.0;
	float elapsedTime = 0.0;

	float lastTime1 = 0.0;
	float lastTime2 = 0.0;
	float lastTime3 = 0.0;

	bool pointIntersects(int point);//Point is part of any triangle that intersects the cube
	bool pointIntersects_2(int point, vector<int> * others, vector<int> * collisionObjectTriangles);//Point intersects and data

	//WavefrontObj collisionMeshes[1];
	WavefrontObj collisionMesh;
	int collisionMeshCount = 0;

	WavefrontObj clothMesh1;
	bool clothMeshLoaded = false;
	
	WavefrontObj objPointsMesh;//Pre-subdivided collision mesh

	vector<short> indices;//Simply the triangle indices of the generated patch
	vector<Spring> springs;
	Vector3D VertexPos[65536];//Actual cloth points
	Vector3D PriorVertexPos[65536];//For keeping away from the CM

	vector<Vector3D> TmpVertexVel;
	vector<Vector3D> TmpVertexPosition;

	vector<Vector3D> VertexVel;
	vector<Vector3D> SpringForce;
	vector<Vector3D> PrevVel;//Probably DELETE

	Vector3D previousPos[65536];//Points where there was no triangle collision (for reversion)
	Vector3D previousVel[65536];
	float pointDistance[65536];//Distance of cloth points to triangle

	Vector3D centroid;
	Vector3D computeCentroid();//Get collision mesh centroid
	Vector3D testPoint;

	float frameDelay = 0;
	float worldTime = 0;

	int pins[2048];
	int pinCount = 0;

	const int STRUCTURAL_SPRING = 0;
	const int SHEAR_SPRING = 1;
	const int BEND_SPRING = 2;
	int spring_count = 0;

	const float dampingCoefficient = 200;
	const float stiffnessCoefficient = 1;// Affects spring stiffness

	const float maxVel = 1.0;//DELETE

	Vector3D gravity;
	float mass = 0.5f;

	Vector3D environmentWind;
	float bWind = 1;

	int numX;
	int numY;
	int vertexCountCloth;
	float hsize = 4 / 2.0f;
	int patchLinks[65536][128];//Link lookup array. Each link has 8 possible other nodes connecting it.
	int linkCount = 0;
	int patchEntrycount[65536];//How many entries each patch entry has (ie: up to 8 exluding any indcidental duplicates)

	vector<int> getPatch(int node);

	//Triangle indices of the cloth
	vector<TriangleIndex> clothTriangles;
	void computeClothTriangles();

	void AddSpring(int a, int b, float ks, float kd, int type);

	void initClothPatch();
	void initClothWorld();
	void loadCollisionMesh(string filename);

	void updateClothMeshWithPosition();

	//-------------------
	//[Wind computation]
	Vector3D windVec;

	float windDirectionSphericalCoordinatesX = -10;
	float windDirectionSphericalCoordinatesY = -90;// [0, -90 : back facing wind]
	float directionVariation = 180;

	//[Wind strength]
	float windStrength = 4.2;// 0.05;

	float directionChangeDelay = 75;

	float maxWindStrength = 10.5;// 0.05;
	float minWindStrength = 3;
	float strengthChangeTime = 1;

	float strengthVariation = 0.0;
	void computeWind(float et);
	//-------------------

	void springDynamics();
	void computeForces(float et);

	void addPinAtPos(float x, float y, float z);
	void addPinAtPos(Vector3D pos);
	void addPinVert(int ref);

	void computeCollisionMeshBoundingBox();
	bool InsideBox(Vector3D point);

	vector <Cube> cubes;
	vector <MeshVoxel> collisionMeshVoxels;
	vector <VertVoxel> collisionMeshVertexVoxels;

	Bounds getTriangleBounds(Vector3D p1, Vector3D p2, Vector3D p3);

	bool boundsOverlap(Bounds a, Bounds b);

	Bounds computeBounds(Cube cube);
	int voxelCount = 0;
	void voxelizeCollisionMesh();
	int getVoxelForPosition(Vector3D pos);

	void voxelizeCollisionMesh_Vertices();//Voxelize only points of the CM

	vector <Triangle> getCMTriangles(Bounds a, int * voxelRef);
	vector <Vector3D> getNearestVoxelVertices(Vector3D postion);

	//Vector3D vectorAwayFromTriangle(Triangle input);
	//Vector3D getParallelVector(Triangle input, Vector3D incident);
	
private:

	bool closestPositionToCM_4(int vertN, Vector3D *collisionPoint, Triangle * nearestTriangle);

	//Wind computation
	float tgtAngle1 = 0;
	float tgtAngle2 = 0;
	float curAngle1 = 0;
	float curAngle2 = 0;
	float directionTime = 0;
	float strengthTime = 0;
};