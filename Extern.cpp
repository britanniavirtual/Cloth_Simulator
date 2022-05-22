#include "JointHeader.h"

vector<int> nearestObjPoints;
vector<Vector3D> nearestNormals;
vector<Vector3D> pointsToDraw;
WavefrontObj renderWireframeMesh;
WavefrontUtils wavefrontUtils;

string springData;

TriangleIntersect triangleIntersect;
Vector3DUtils vector3DUtils;
Utils utils;
ClothWorld clothWorld;
Console console;
