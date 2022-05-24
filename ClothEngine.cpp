#include "JointHeader.h"

//--------------------------------------
//Cosntants
const bool PINS_ENABLED = 1;
const bool TRAVEL = 0;
const bool WIND_ON = 1;
const bool TURBULENCE_ON = 0;
const bool COLLISION_ON = 1;
const bool GRAVITY_ON = 1;
const float TRAVEL_SPEED = 0.00026;
const float WIND_SCALE = 0.7;
const float RENDER_DELAY_COEFFICIENT = 1.0;
//-------------------------------------

void ClothWorld::loadCollisionMesh(string filename)
{
	//WavefrontObj obj;
	collisionMesh.loadObj(filename);

	//collisionMeshes[collisionMeshCount] = obj;
	collisionMeshCount++;
}

void ClothWorld::updateClothMeshWithPosition()
{
	for (int a = 0; a < vertexCountCloth; a++)
	{
		clothPoints2[a].x = VertexPos[a].x;// Vector3D(x, y, z - ((float)numX / (float)v + 1));//NB: Center the cloth with z- the last bit
		clothPoints2[a].y = VertexPos[a].y;
		clothPoints2[a].z = VertexPos[a].z;
	}
}

void ClothWorld::initPins()
{
	addPinVert(0);
	addPinVert(20);

	/*
	addPinAtPos(-1.20647, 3.65046, 1.361);
	addPinAtPos(-1.20647, 3.65046, -0.266349);
	*/
}

vector<int> ClothWorld::removeDuplicates(vector<int> input, int ignore)
{
	vector<int> ret;

	for (int a = 0; a < input.size(); a++)
	{
		int cur = input[a];

		bool found = false;
		for (int a = 0; a < ret.size(); a++)
		{
			if (ret[a] == cur)
			{
				found = true;
				break;
			}
		}

		if (!found && cur != ignore)
		{
			ret.push_back(cur);
		}
	}

	return ret;
}

void ClothWorld::initClothWorld()
{
	for (int a = 0; a < 65536; a++)
	{
		//surfaceResistance[a] = 0;
	}

	if (clothMeshLoaded)
	{
	}
	else
	{
		initClothPatch();
	}

	//[Compute cloth patches]
	for (int a = 0; a < vertexCountCloth; a++)
	{
		vector<int> nodes = getPatch(a);

		for (int b = 0; b < nodes.size(); b++)
		{
			clothPatch[a].push_back(nodes[b]);
		}
	}
}

void ClothWorld::AddSpring(int VertexRefA, int VertexRefB, float ks, float kd, int type)
{
	Spring spring;
	spring.p1 = VertexRefA;
	spring.p2 = VertexRefB;
	spring.Ks = ks;
	spring.Kd = kd;
	spring.type = type;
	Vector3D deltaP = VertexPos[VertexRefA] - VertexPos[VertexRefB];
	spring.rest_length = sqrt(vector3DUtils.dot(deltaP, deltaP));

	springs.push_back(spring);
}

void ClothWorld::computeClothTriangles()
{
}

//Get every index that is connected to the parameter index
//ToDo: Compute only once
vector<int> ClothWorld::getPatch(int vertexNode)
{
	vector<int> op;

	Vector3D curVec = VertexPos[indices[vertexNode]];

	int ind1;
	int ind2;
	int ind3;

	//cout << " indices.size() : " << indices.size() << endl;

	int i = 0;
	//Indices are the nodes for the cloth
	//Indices are simply the triangle points
	for (i = 0; i < indices.size(); i += 3)//If any verts of current face match the tgt vert than add every vert of the current face
	{
		ind1 = indices[i];
		ind2 = indices[i + 1];
		ind3 = indices[i + 2];

		if (ind1 == vertexNode || ind2 == vertexNode || ind3 == vertexNode)
		{
			op.push_back(ind1);
			op.push_back(ind2);
			op.push_back(ind3);
		}
	}

	return op;
}


Vector3D normalize(Vector3D input)
{
	Vector3D output;

	double length = sqrt(input.x * input.x + input.y * input.y + input.z * input.z);
	output.x = input.x / length;
	output.y = input.y / length;
	output.z = input.z / length;

	return output;
}

Vector3D triangleNormal(Vector3D a, Vector3D b, Vector3D c)
{
	float Vector[3], Vector2[3];

	float v1[] = { a.x,  a.y, a.z };
	float v2[] = { b.x,  b.y, b.z };
	float v3[] = { c.x,  c.y, c.z };

	Vector[0] = v3[0] - v1[0];
	Vector[1] = v3[1] - v1[1];
	Vector[2] = v3[2] - v1[2];

	Vector2[0] = v2[0] - v1[0];
	Vector2[1] = v2[1] - v1[1];
	Vector2[2] = v2[2] - v1[2];

	Vector3D VectorB1(Vector[0], Vector[1], Vector[2]);
	Vector3D VectorB2(Vector2[0], Vector2[1], Vector2[2]);

	Vector3D normal = vector3DUtils.cross(VectorB1, VectorB2);

	return normalize(normal);
}


//-------------------------------------
// Algorithm to generate a grid of X.Y
// NB: The patch will need to have a certain minimum resolution so there are no poking-though artefacts with any collision
// mesh.
//-------------------------------------
void ClothWorld::initClothPatch()
{
	computeCentroid();

	//NxN vertices for the cloth patch
	numX = 32;
	numY = 32;

	vertexCountCloth = (numX + 1) * (numY + 1);

	int i = 0, j = 0;
	int l1 = 0, l2 = 0;

	int count = 0;
	indices.resize(numX * numY * 2 * 3);
	//VertexPosition.resize(total_points);
	VertexVel.resize(vertexCountCloth);
	SpringForce.resize(vertexCountCloth);
	PrevVel.resize(vertexCountCloth);

	//[Generate cloth vertices]
	float ypos = 3.3;////2.4
	float xpos = 0.0;
	float zpos = 1.0;

	TmpVertexVel.resize(vertexCountCloth);
	TmpVertexPosition.resize(vertexCountCloth);

	int v = numY + 1;
	int u = numX + 1;

	int priorN = 0;
	//[Setup cloth VertexPosition]
	for (j = 0; j < v; j++)
	{
		for (i = 0; i < u; i++)
		{
			float x = ((float(i) / (u - 1)) * 2 - 1) * hsize;
			float y = ypos;//Up/Down
			float z = ((float(j) / (v - 1)) * 4) + zpos;

			VertexPos[count++] = Vector3D(x, y, z - ((float)numX / (float)v + 1));//NB: Center the cloth with z- the last bit
			PriorVertexPos[priorN] = Vector3D(x, y, z - ((float)numX / (float)v + 1));
			priorN++;
		}
	}



	//[Setup VertexVel]
	memset(&(VertexVel[0].x), 0, vertexCountCloth * sizeof(Vector3D));
	memset(&(TmpVertexVel[0].x), 0, vertexCountCloth * sizeof(Vector3D));

	//[Setup indices]
	short* cloth_ID = &indices[0];
	for (i = 0; i < numY; i++)
	{
		for (j = 0; j < numX; j++)
		{
			int i0 = i * (numX + 1) + j;
			int i1 = i0 + 1;
			int i2 = i0 + (numX + 1);
			int i3 = i2 + 1;
			if ((j + i) % 2)
			{
				*cloth_ID++ = i0; *cloth_ID++ = i2; *cloth_ID++ = i1;
				*cloth_ID++ = i1; *cloth_ID++ = i2; *cloth_ID++ = i3;
			}
			else
			{
				*cloth_ID++ = i0; *cloth_ID++ = i2; *cloth_ID++ = i3;
				*cloth_ID++ = i0; *cloth_ID++ = i3; *cloth_ID++ = i1;
			}
		}
	}
	
	//[Setup cloth springs]

	//Horizontal
	int aaa = 0;
	for (int l1 = 0; l1 < v; l1++)
	{
		for (int l2 = 0; l2 < (u - 1); l2++)
		{
			AddSpring((l1 * u) + l2, (l1 * u) + l2 + 1, KsStruct, KdStruct, STRUCTURAL_SPRING);
			//cout << ":" << (l1 * u) + l2 << " " << (l1 * u) + l2 + 1 << endl;
		}
	}

	//Vertical
	for (int l1 = 0; l1 < (u); l1++)
	{
		for (int l2 = 0; l2 < (v - 1); l2++)
		{
			AddSpring((l2 * u) + l1, ((l2 + 1) * u) + l1, KsStruct, KdStruct, STRUCTURAL_SPRING);
		}
	}

	//Shear
	for (int l1 = 0; l1 < (v - 1); l1++)
	{
		for (int l2 = 0; l2 < (u - 1); l2++)
		{
			AddSpring((l1 * u) + l2, ((l1 + 1) * u) + l2 + 1, KsShear, KdShear, SHEAR_SPRING);
			AddSpring(((l1 + 1) * u) + l2, (l1 * u) + l2 + 1, KsShear, KdShear, SHEAR_SPRING);
		}
	}

	//vectorToDrawCount = 0;
	int aabb = 0;

	//Bend 1
	for (int l1 = 0; l1 < (v); l1++)
	{
		for (l2 = 0; l2 < (u - 2); l2++)
		{
			AddSpring((l1 * u) + l2, (l1 * u) + l2 + 2, KsBend, KdBend, BEND_SPRING);
		}
		AddSpring((l1 * u) + (u - 3), (l1 * u) + (u - 1), KsBend, KdBend, BEND_SPRING);//<--- This just fills in the last column
	}
	
	//Bend 2
	for (int l1 = 0; l1 < (u); l1++)
	{
		for (int l2 = 0; l2 < (v - 2); l2++)
		{
			AddSpring((l2 * u) + l1, ((l2 + 2) * u) + l1, KsBend, KdBend, BEND_SPRING);
		}
		AddSpring(((v - 3) * u) + l1, ((v - 1) * u) + l1, KsBend, KdBend, BEND_SPRING);//<--- This just fills in the last column[?]
	}

	initPins();
}


float wn = 0;
//-------------------------------
//
//-------------------------------
void ClothWorld::sinWave()
{
	float a[2] = { 0.8, 0.2 };
	float k[2] = { 1.0, 8.0 };
	float w[2] = { 1.0, 8.0 };
	float p[2] = { 0.0, 1.0 };

	wn += 0.002;

	for (int c = 0; c < vertexCountCloth; c++)
	{
		float d = utils.dist(VertexPos[c].z, 0, 0, 0, 0, 0) + wn;
		float sum = 0.0;

		for (int i = 0; i < 2; i++)
		{
			float tm = 1;
			sum += sin(k[i] * d - tm);
		}

		float ypos = 0;
		sinPos[c].y = (sum / 15) + ypos;
	}
}

//-------------------------------
//
//-------------------------------
void ClothWorld::computeCMeshTriangleBounds()
{
	for (int a = 0; a < collisionMesh.indicesCount; a += 3)
	{
		int i1 = collisionMesh.indices[a];
		int i2 = collisionMesh.indices[a + 1];
		int i3 = collisionMesh.indices[a + 2];

		Vector3D vert1(collisionMesh.vertices[i1].x, collisionMesh.vertices[i1].y, collisionMesh.vertices[i1].z);
		Vector3D vert2(collisionMesh.vertices[i2].x, collisionMesh.vertices[i2].y, collisionMesh.vertices[i2].z);
		Vector3D vert3(collisionMesh.vertices[i3].x, collisionMesh.vertices[i3].y, collisionMesh.vertices[i3].z);

		Bounds curBounds;

		curBounds = getTriangleBounds(vert1, vert2, vert3);

		curBounds.maxX += 0.01;
		curBounds.maxY += 0.01;
		curBounds.maxZ += 0.01;

		curBounds.minX -= 0.01;
		curBounds.minY -= 0.01;
		curBounds.minZ -= 0.01;

		MeshTriangle cur;
		Triangle tri(vert1, vert2, vert3);
		cur.bounds = curBounds;
		cur.triangle = tri;
		
		CMeshTriangleBounds.push_back(cur);
	}
}

//-------------------------------
//
//-------------------------------
void ClothWorld::computeTurbulence(float et)
{
	if (!TURBULENCE_ON) { return; }

	for (int a = 0; a < vertexCountCloth; a++)//1 turbulence for each vertex
	{
		TurbulenceRegion curTurb = turbulence[a];

		curTurb.elapsedTurbulenceTime = ((clock() - curTurb.lastTurbulenceTime));
		curTurb.directionTime += curTurb.elapsedTurbulenceTime;
		curTurb.strengthTime += curTurb.elapsedTurbulenceTime;
		curTurb.lastTurbulenceTime = clock();

		//Change dir
		if (curTurb.directionTime > curTurb.directionChangeDelay)
		{
			int randAmount = curTurb.directionVariation;
			curTurb.tgtAngle1 += utils.random_2(1,10);// (curTurb.windDirectionSphericalCoordinatesX - randAmount) + (rand() % (2 * randAmount));
			//curTurb.tgtAngle2 += 5;// (curTurb.windDirectionSphericalCoordinatesY - randAmount) + (rand() % (2 * randAmount));
			curTurb.directionTime = 0;

			//Set manually
			curTurb.curAngle1 = curTurb.tgtAngle1;
			curTurb.curAngle2 = curTurb.tgtAngle2;
		}

		//if (!curTurb.setAngle)
		//{
			curTurb.curAngle1 = curTurb.tgtAngle1;
			curTurb.curAngle2 = curTurb.tgtAngle2;
		//}
		
		curTurb.setAngle = true;
		curTurb.windVec = vector3DUtils.OrbitalPosition(curTurb.curAngle1 * 0.017, curTurb.curAngle2 * 0.017, Vector3D(0, 0, 0));
		Vector3D newWind(curTurb.windVec.x, curTurb.windVec.y, curTurb.windVec.z);
		newWind *= curTurb.windStrength;
		
		curTurb.turb = newWind * 5.5;
		//cout << newWind.x << " " << newWind.y << " " << newWind.z << endl;
		turbulence[a] = curTurb;
	}
}

//Move the wind vector around smoothly / randomly
//Gradually move towards spherical coordinates
//Select new spherical coordinates every N seconds
bool setAngle = false;
void ClothWorld::computeWind(float et)
{
	computeTurbulence(et);

	if (!WIND_ON) { return; }

	float elapsedWindTime = ((clock() - lastWindTime));
	directionTime += elapsedWindTime;
	strengthTime += elapsedWindTime;
	lastWindTime = clock();

	//Change dir
	if (directionTime > directionChangeDelay)
	{
		int randAmount = directionVariation;
		tgtAngle1 = (windDirectionSphericalCoordinatesX - randAmount) + (rand() % (2 * randAmount));
		tgtAngle2 = (windDirectionSphericalCoordinatesY - randAmount) + (rand() % (2 * randAmount));

		directionTime = 0;
	}

	if (!setAngle)
	{
		curAngle1 = tgtAngle1;
		curAngle2 = tgtAngle2;
	}
	setAngle = true;

	float changeAmount = 0.5;
	if (utils.dist(tgtAngle1, 0, 0, curAngle1, 0, 0) > 1)
	{
		//Move toward
		if (curAngle1 > tgtAngle1)
		{
			curAngle1 -= changeAmount;
		}
		else
		{
			curAngle1 += changeAmount;
		}
	}

	if (utils.dist(tgtAngle2, 0, 0, curAngle2, 0, 0) > 1)
	{
		//Move toward
		if (curAngle2 > tgtAngle2)
		{
			curAngle2 -= changeAmount;
		}
		else
		{
			curAngle2 += changeAmount;
		}
	}

	//Change strength
	if (strengthTime > 1000 * strengthChangeTime)
	{
		windStrength = utils.random_2(minWindStrength, maxWindStrength);
		//cout << windStrength << endl;
		strengthTime = 0;
	}
	
	//cout << curAngle1 << " " << curAngle2 << endl;
	//curAngle1 *= 0.017;
	//curAngle2 *= 0.017;

	windVec = vector3DUtils.OrbitalPosition(curAngle1 * 0.017, curAngle2 * 0.017, Vector3D(0, 0, 0));
	Vector3D newWind(windVec.x, windVec.y, windVec.z);
	newWind *= windStrength;


	environmentWind = newWind;
}

bool ClothWorld::pointIntersects(int point)
{
	return 0;
}

bool ClothWorld::isPinned(int vertex)
{
	for (int a = 0; a < pinCount; a++)
	{
		if (pins[a] == vertex) { return true; }
	}

	return false;
}

//-----------------------
//Affects the vertex velocities due to spring forces
//-----------------------
void ClothWorld::springDynamics()
{
	//cout << endl << endl;

	for (int i = 0; i < springs.size(); i++)
	{
		//Check the current lengths of all springs
		Vector3D p1(0, 0, 0);
		p1.x = VertexPos[springs[i].p1].x;
		p1.y = VertexPos[springs[i].p1].y;
		p1.z = VertexPos[springs[i].p1].z;

		Vector3D p2(0, 0, 0);
		p2.x = VertexPos[springs[i].p2].x;
		p2.y = VertexPos[springs[i].p2].y;
		p2.z = VertexPos[springs[i].p2].z;

		Vector3D deltaP = (p1 - p2);
		
		float dist = vector3DUtils.length(deltaP);

	
		//[Frame delay here seems to effect it positively]
		//deltaP *= frameDelay * 1000;

		//
		if (dist > springs[i].rest_length)
		{
			dist -= (springs[i].rest_length);
			dist /= 2.0f;

			deltaP = vector3DUtils.normalize(deltaP);

			//NB: Multiplying deltaP by 25 increases stiffness (Less flex) but it still bounces everywhere. Add damping to correct.

			//[Error: If deltaP coefficent too high then it explodes]
			//************If its too low then its too loose. How to fix?

			deltaP *= dist;/// *0.5 * 250;// / 10 // *frameDelay * 10;

			//cout << deltaP.x << " , " << deltaP.y << " , " << deltaP.z << endl;

			//deltaP.x = utils.clamp(deltaP.x, 1, -1);
			//deltaP.y = utils.clamp(deltaP.y, 1, -1);
			//deltaP.z = utils.clamp(deltaP.z, 1, -1);

			///if (abs(vector3DUtils.length(deltaP) > 1.0))
			//{
			//	continue;
			//}

			deltaP *= 10;// * 10 works			

			//[Additional For hinge - Start]
			//Purpose is to update: VertexVel, for use in the main step
			if (PINS_ENABLED)
			{
				if (isPinned(springs[i].p1))
				{
					VertexVel[springs[i].p2].x += deltaP.x;
					VertexVel[springs[i].p2].y += deltaP.y;
					VertexVel[springs[i].p2].z += deltaP.z;
				}
				else if (isPinned(springs[i].p2))
				{
					VertexVel[springs[i].p1].x -= deltaP.x;
					VertexVel[springs[i].p1].y -= deltaP.y;
					VertexVel[springs[i].p1].z -= deltaP.z;
				}//[Additional For hinge - End]
				else
				{
					VertexVel[springs[i].p1].x -= deltaP.x;
					VertexVel[springs[i].p1].y -= deltaP.y;
					VertexVel[springs[i].p1].z -= deltaP.z;

					VertexVel[springs[i].p2].x += deltaP.x;
					VertexVel[springs[i].p2].y += deltaP.y;
					VertexVel[springs[i].p2].z += deltaP.z;
				}
			}
			else
			{
				VertexVel[springs[i].p1].x -= deltaP.x;
				VertexVel[springs[i].p1].y -= deltaP.y;
				VertexVel[springs[i].p1].z -= deltaP.z;

				VertexVel[springs[i].p2].x += deltaP.x;
				VertexVel[springs[i].p2].y += deltaP.y;
				VertexVel[springs[i].p2].z += deltaP.z;
			}
		}
	}
}



//Acceleration step done here
//Wind and gravity applied here
void ClothWorld::computeForces(float et)
{
	//sinWave();
	
	if (!GRAVITY_ON)
	{
		gravity.x = 0;
		gravity.y = 0;
		gravity.z = 0;
	}

	size_t i = 0;
	//vectorToDrawCount = 0;

	for (i = 0; i < vertexCountCloth; i++)//Simply for each point add a new force (Gravity, wind)
	{
		SpringForce[i] = Vector3D(0,0,0);

		//add gravity force only for non-fixed points
		if (PINS_ENABLED)
		{
			if (!(isPinned(i)))//For hinge (1/3)
			{
				SpringForce[i] += gravity + turbulence[i].turb + (environmentWind * WIND_SCALE);// *10 * frameDelay;// +(environmentWind * bWind);// *frameDelay;
			}
		}
		else
		{
			SpringForce[i] += gravity + turbulence[i].turb + (environmentWind * WIND_SCALE);// *10 * frameDelay;// +(environmentWind * bWind);// *frameDelay;
		}

		//[DAMPING]
		SpringForce[i] += VertexVel[i] * DEFAULT_DAMPING * 20;// *dampingCoefficient;

	}

	//[Add spring forces]
	for (i = 0; i < springs.size(); i++)
	{
		Vector3D p1 = VertexPos[springs[i].p1];
		Vector3D p2 = VertexPos[springs[i].p2];
		Vector3D v1 = VertexVel[springs[i].p1];
		Vector3D v2 = VertexVel[springs[i].p2];

		Vector3D deltaP = p1 - p2;
		Vector3D deltaV = v1 - v2;
		float dist = vector3DUtils.length(deltaP);

		float leftTerm = -springs[i].Ks * (dist - springs[i].rest_length);
		float rightTerm = springs[i].Kd * vector3DUtils.dot(deltaV, deltaP) / dist;

		Vector3D curSpringForce = vector3DUtils.normalize(deltaP) * (leftTerm + rightTerm) * frameDelay * 250;// DELETE FRAME DELAY

		if (!PINS_ENABLED)
		{
			VertexVel[springs[i].p1].x += curSpringForce.x;
			VertexVel[springs[i].p1].y += curSpringForce.y;
			VertexVel[springs[i].p1].z += curSpringForce.z;

			VertexVel[springs[i].p2].x -= curSpringForce.x;
			VertexVel[springs[i].p2].y -= curSpringForce.y;
			VertexVel[springs[i].p2].z -= curSpringForce.z;
		}
		else
		{
			if (!(isPinned(springs[i].p1))) //For hinge (2/3)
			{
				VertexVel[springs[i].p1].x += curSpringForce.x;
				VertexVel[springs[i].p1].y += curSpringForce.y;
				VertexVel[springs[i].p1].z += curSpringForce.z;
			}

			if (!(isPinned(springs[i].p2))) //For hinge (3/3)
			{
				VertexVel[springs[i].p2].x -= curSpringForce.x;
				VertexVel[springs[i].p2].y -= curSpringForce.y;
				VertexVel[springs[i].p2].z -= curSpringForce.z;	
			}
		}
	}
}


Triangle ClothWorld::getNearestCMTriangle(Vector3D position)
{
	Triangle curTriangle;
	float nearestDist = 4096;
	for (int c = 0; c < collisionMesh.indicesCount; c += 3)
	{
		int i1 = collisionMesh.indices[c];
		int i2 = collisionMesh.indices[c + 1];
		int i3 = collisionMesh.indices[c + 2];

		Vector3D centroid((collisionMesh.vertices[i1].x + collisionMesh.vertices[i2].x + collisionMesh.vertices[i3].x) / 3, (collisionMesh.vertices[i1].y + collisionMesh.vertices[i2].y + collisionMesh.vertices[i3].y) / 3, (collisionMesh.vertices[i1].z + collisionMesh.vertices[i2].z + collisionMesh.vertices[i3].z) / 3);
	
		float curDist = utils.dist(centroid.x, centroid.y, centroid.z, position.x, position.y, position.z);

		if (curDist < nearestDist)
		{
			nearestDist = curDist;

			curTriangle.p1.x = collisionMesh.vertices[i1].x;
			curTriangle.p1.y = collisionMesh.vertices[i1].y;
			curTriangle.p1.z = collisionMesh.vertices[i1].z;

			curTriangle.p2.x = collisionMesh.vertices[i2].x;
			curTriangle.p2.y = collisionMesh.vertices[i2].y;
			curTriangle.p2.z = collisionMesh.vertices[i2].z;

			curTriangle.p3.x = collisionMesh.vertices[i3].x;
			curTriangle.p3.y = collisionMesh.vertices[i3].y;
			curTriangle.p3.z = collisionMesh.vertices[i3].z;
		}
	}

	return curTriangle;
}

Triangle nearTri[65536];
//--------------------------------
// 
//--------------------------------
bool ClothWorld::closestPositionToCM_4(int vertN, Vector3D *collisionPoint, Triangle * nearestTriangle)
{
	//vector<int> nodes = getPatch(vertN);//Get array of triangles which compose the patch
	vector<int> clothNodes = clothPatch[vertN];//Use those computed on init = +200fps!

	for (int a = 0; a < clothNodes.size(); a += 3)
	{
		//Get bounded CM triangles
		vector <Triangle> cTris = getBoundedCMTriangles(VertexPos[clothNodes[a]], VertexPos[clothNodes[a + 1]], VertexPos[clothNodes[a + 2]]);
		//cout << "cTris " << cTris.size() << endl;
		
		for (int b = 0; b < cTris.size(); b++)
		{
			//NB: Vertices to test for collision
			float point1[3] = { VertexPos[clothNodes[a]].x,      VertexPos[clothNodes[a]].y,      VertexPos[clothNodes[a]].z };//Collsion mesh verts
			float point2[3] = { VertexPos[clothNodes[a + 1]].x,  VertexPos[clothNodes[a + 1]].y,  VertexPos[clothNodes[a + 1]].z };
			float point3[3] = { VertexPos[clothNodes[a + 2]].x,  VertexPos[clothNodes[a + 2]].y,  VertexPos[clothNodes[a + 2]].z };

			float cmPoint1[3] = { cTris[b].p1.x, cTris[b].p1.y, cTris[b].p1.z };//Patch verts
			float cmPoint2[3] = { cTris[b].p2.x, cTris[b].p2.y, cTris[b].p2.z };
			float cmPoint3[3] = { cTris[b].p3.x, cTris[b].p3.y, cTris[b].p3.z };

			if (triangleIntersect.triangleIntersect(point1, point2, point3, cmPoint1, cmPoint2, cmPoint3) == 1)
			{
				nearTri[vertN] = cTris[b];
				nearestTriangle = &cTris[b];
				return true;
			}
		}
	}

	return false;
}

Vector3D closestVec2[65536];
float greatestSpringDist = 0.0;
int furthestSpring = 0;
float movedDist = 0;
Vector3D prev2[66536];
bool gotPrev = false;
bool start = false;
bool lastCollided[65536];
Vector3D lastCollisionPoint[65536];
bool stuck = false;
Triangle tri0;
bool started = false;
void ClothWorld::stepClothWorld()
{
	float elapsedTime = ((clock() - lastTime) / 1000);
	//cout << elapsedTime << endl;



	worldTime += elapsedTime;
	lastTime = clock();
	elapsedTime = constStep / RENDER_DELAY_COEFFICIENT;//Override - uniform step needed//0.1 fail 0.01 ok
	float deltaTimeMass = elapsedTime;// *0.1;// 0.01 needs to scale with framerate drop was 0.1
	frameDelay = elapsedTime;
	computeWind(elapsedTime);



	//[Two main cloth computation functions]:
	computeForces(elapsedTime);
	springDynamics();

	//[Step every vertex]
	for (int i = 0; i < vertexCountCloth; i++)
	{
		//[Main step vertex]

		if (!start)
		{
			start = true;
		}

		//[Store previous vectors for reversion after collision]
		Vector3D previousPos = VertexPos[i];
		Vector3D previousVel = VertexVel[i];

		Vector3D oldV = VertexVel[i];
		VertexVel[i] += (SpringForce[i] * deltaTimeMass);
		VertexPos[i] += oldV * elapsedTime;

		Vector3D velOperation = (SpringForce[i] * deltaTimeMass);//+= both
		Vector3D posOperation = (VertexVel[i] * elapsedTime);
		//--------------------------------

		//[Pin travel]
		//Drag the cloth backwards at the pins
		if (TRAVEL)
		{
			if (isPinned(i))
			{
				VertexPos[i].z -= TRAVEL_SPEED;
			}
		}
		//--------------------------------

		if (!COLLISION_ON) { continue; }

		//[Store a position 'away' from the last point]
		float priorDist = utils.dist(PriorVertexPos[i].x, PriorVertexPos[i].y, PriorVertexPos[i].z, VertexPos[i].x, VertexPos[i].y, VertexPos[i].z);
		if (priorDist > 0.3)
		{
			PriorVertexPos[i] = VertexPos[i];
		}
		//--------------------------------

		float stepDist = utils.dist(VertexPos[i].x, VertexPos[i].y, VertexPos[i].z, previousPos.x, previousPos.y, previousPos.z);

		//[MAIN COLLIDE]
		
		Vector3D cp00;
		float cdist = INFINITE;// closestPositionToCM_3(i, &cp00, &tri0);// INFINITE;// closestPositionToCM_3(i, &cp00, &tri0);//Main collision detect function with voxel algorithm //300fps		
		bool collide = closestPositionToCM_4(i, &cp00, &tri0);//A lot faster (x2) than: closestPositionToCM_3. Doesnt have distance which is essential. //600fps

		if (!collide)
		{
			closestVec2[i] = VertexPos[i];
		}
		else//Collided (Set to previous position)
		{
			lastCollisionPoint[i] = VertexPos[i];
			lastCollided[i] = true;
			VertexPos[i] = previousPos;

			VertexVel[i].x = 0;
			VertexVel[i].y = 0;
			VertexVel[i].z = 0;
		}
	}

	updateClothMeshWithPosition();
}


void ClothWorld::addPinAtPos(Vector3D pos)
{
	float nearestDistance = 2048;
	int closestInt = 0;
	for (int a = 0; a < vertexCountCloth; a++)
	{
		float curDist = utils.dist(pos.x, pos.y, pos.z, VertexPos[a].x, VertexPos[a].y, VertexPos[a].x);

		if (curDist < nearestDistance)
		{
			nearestDistance = curDist;
			closestInt = a;
		}
	}

	addPinVert(closestInt);
}


void ClothWorld::addPinAtPos(float x, float y, float z)
{
	float nearestDistance = 2048;
	int closestInt = 0;
	for (int a = 0; a < vertexCountCloth; a++)
	{
		float curDist = utils.dist(x, y, z, VertexPos[a].x, VertexPos[a].y, VertexPos[a].z);

		//cout << VertexPosition[a].x << " " << VertexPosition[a].y << " " << VertexPosition[a].x << endl;

		if (curDist < nearestDistance)
		{

			nearestDistance = curDist;
			closestInt = a;
		}
	}

	//system("PAUSE");

	//vectorsToDraw[vectorToDrawCount] = VertexPosition[closestInt];
	//vectorToDrawCount++;

	addPinVert(closestInt);
}

void ClothWorld::addPinVert(int ref)
{
	pins[pinCount] = ref;
	pinCount++;
}

//NB: Needs every triangle the point parameter is part of. If any of these triangle collide, return the points as a vector
//Get if point intersects & all other related points that intersect
bool ClothWorld::pointIntersects_2(int point, vector<int> * collisionPoints, vector<int> * collisionObjectIndices)
{
	bool collides = false;
	//objNormals.clear();

	if (collisionMeshCount > 0)
	{
		//Get triangle for this point by looking through all indices
		int tgtIndex = point;

		//Loop through every triangle. Test the triangle collision with the cube. If the point is part of this triangle then freeze.
		//Get all triangles the point is part of

		vector <TriangleIndex> allTriangles;//Store all triangles this point is part of
		for (int b = 0; b < indices.size(); b += 3)//Get all triangles this point is part of
		{
			if (tgtIndex == indices[b] || tgtIndex == indices[b + 1] || tgtIndex == indices[b + 2])
			{
				TriangleIndex curTriangle;
				curTriangle.a = indices[b];
				curTriangle.b = indices[b + 1];
				curTriangle.c = indices[b + 2];
				allTriangles.push_back(curTriangle);
			}
		}

		//Loop through allTriangles
		for (int b = 0; b < allTriangles.size(); b++)
		{
			Vector3D vertA = clothWorld.VertexPos[allTriangles[b].a];
			Vector3D vertB = clothWorld.VertexPos[allTriangles[b].b];
			Vector3D vertC = clothWorld.VertexPos[allTriangles[b].c];

			for (int c = 0; c < collisionMesh.indicesCount; c += 3)//Loop through entire collision object mesh
			{
				//Get collision mesh triangle
				int i1 = collisionMesh.indices[c];
				int i2 = collisionMesh.indices[c + 1];
				int i3 = collisionMesh.indices[c + 2];

				Vector3D vert1(collisionMesh.vertices[i1].x, collisionMesh.vertices[i1].y, collisionMesh.vertices[i1].z);
				Vector3D vert2(collisionMesh.vertices[i2].x, collisionMesh.vertices[i2].y, collisionMesh.vertices[i2].z);
				Vector3D vert3(collisionMesh.vertices[i3].x, collisionMesh.vertices[i3].y, collisionMesh.vertices[i3].z);

				//NB: Vertices to test for collision
				float point1[3] = { vert1.x, vert1.y, vert1.z };//Collsion mesh verts
				float point2[3] = { vert2.x, vert2.y, vert2.z };
				float point3[3] = { vert3.x, vert3.y, vert3.z };

				float clothPoint1[3] = { vertA.x, vertA.y, vertA.z };//Patch verts
				float clothPoint2[3] = { vertB.x, vertB.y, vertB.z };
				float clothPoint3[3] = { vertC.x, vertC.y, vertC.z };

				if (triangleIntersect.triangleIntersect(point1, point2, point3, clothPoint1, clothPoint2, clothPoint3) == 1)
				{
					//Return triangle indices of the collision object that have collided
					//Each of these is essentially a vertex ref
					collisionObjectIndices->push_back(i1);
					collisionObjectIndices->push_back(i2);
					collisionObjectIndices->push_back(i3);

					//Return all points of all triangles of the cloth that have collided
					collisionPoints->push_back(allTriangles[b].a);
					collisionPoints->push_back(allTriangles[b].b);
					collisionPoints->push_back(allTriangles[b].c);

					//cout << "Collide" << endl;
					//system("PAUSE");
					collides = true;
				}
			}
		}
	}

	return collides;
}



Vector3D ClothWorld::computeCentroid()
{
	float avgX = 0;
	float avgY = 0;
	float avgZ = 0;

	int n = 0;
	//[Get collision mesh triangles]
	for (int c = 0; c < collisionMesh.indicesCount; c += 3)
	{
		int i1 = collisionMesh.indices[c];
		int i2 = collisionMesh.indices[c + 1];
		int i3 = collisionMesh.indices[c + 2];

		//[CM verts]:

		avgX += collisionMesh.vertices[i1].x;
		avgY += collisionMesh.vertices[i1].y;
		avgZ += collisionMesh.vertices[i1].z;

		avgX += collisionMesh.vertices[i2].x;
		avgY += collisionMesh.vertices[i2].y;
		avgZ += collisionMesh.vertices[i2].z;

		avgX += collisionMesh.vertices[i3].x;
		avgY += collisionMesh.vertices[i3].y;
		avgZ += collisionMesh.vertices[i3].z;

		n += 3;
	}

	/*
	//coordinate of the vertices
	float x1 = 1, x2 = 3, x3 = 6;
	float y1 = 2, y2 = -4, y3 = -7;

	//Formula to calculate centroid
	float x = (x1 + x2 + x3) / 3;
	float y = (y1 + y2 + y3) / 3;
	*/

	centroid.x = avgX / n;
	centroid.y = avgY / n;
	centroid.z = avgZ / n;

	//cout << centroid.x << " " << centroid.y << " " << centroid.z << endl;
	//system("PAUSE");

	return centroid;
}

float maxX = 0;
float minX = 2048;
float maxY = 0;
float minY = 2048;
float maxZ = 0;
float minZ = 2048;
void ClothWorld::computeCollisionMeshBoundingBox()
{
	for (int a = 0; a < collisionMesh.originalVertexCount; a++)
	{
		if (collisionMesh.originalVertices[a].x > maxX)
		{
			maxX = collisionMesh.originalVertices[a].x;
		}

		if (collisionMesh.originalVertices[a].x < minX)
		{
			minX = collisionMesh.originalVertices[a].x;
		}

		if (collisionMesh.originalVertices[a].y > maxY)
		{
			maxY = collisionMesh.originalVertices[a].y;
		}

		if (collisionMesh.originalVertices[a].y < minY)
		{
			minY = collisionMesh.originalVertices[a].y;
		}

		if (collisionMesh.originalVertices[a].z > maxZ)
		{
			maxZ = collisionMesh.originalVertices[a].z;
		}

		if (collisionMesh.originalVertices[a].z < minZ)
		{
			minZ = collisionMesh.originalVertices[a].z;
		}
	}
}

bool ClothWorld::InsideBox(Vector3D point)
{
	if (point.x < maxX && point.x > minX)
	{
		if (point.y < maxY && point.y > minY)
		{
			if (point.z < maxZ && point.z > minZ)
			{
				return true;
			}
		}
	}

	return false;
}


void Cube::translate(int axis, float amount)
{
	if (axis == 0)
	{
		p1.x += amount;
		p2.x += amount;
		p3.x += amount;
		p4.x += amount;

		p5.x += amount;
		p6.x += amount;
		p7.x += amount;
		p8.x += amount;
	}

	if (axis == 1)
	{
		p1.y += amount;
		p2.y += amount;
		p3.y += amount;
		p4.y += amount;

		p5.y += amount;
		p6.y += amount;
		p7.y += amount;
		p8.y += amount;
	}

	if (axis == 2)
	{
		p1.z += amount;
		p2.z += amount;
		p3.z += amount;
		p4.z += amount;

		p5.z += amount;
		p6.z += amount;
		p7.z += amount;
		p8.z += amount;
	}
}

Bounds ClothWorld::computeBounds(Cube cube)
{
	vector <Vector3D> curVertices;
	curVertices.push_back(cube.p1);
	curVertices.push_back(cube.p2);
	curVertices.push_back(cube.p3);
	curVertices.push_back(cube.p4);
	curVertices.push_back(cube.p5);
	curVertices.push_back(cube.p6);
	curVertices.push_back(cube.p7);
	curVertices.push_back(cube.p8);

	float maxX = -8192;
	float maxY = -8192;
	float maxZ = -8192;
 
	float minX = 8192;
	float minY = 8192;
	float minZ = 8192;

	Bounds bounds;

	//Compute mesh bounding box
	for (int a = 0; a < curVertices.size(); a++)
	{
		if (curVertices[a].x > maxX)
		{
			maxX = curVertices[a].x;
			bounds.maxX = curVertices[a].x;
		}

		if (curVertices[a].x < minX)
		{
			minX = curVertices[a].x;
			bounds.minX = curVertices[a].x;
		}

		if (curVertices[a].y > maxY)
		{
			maxY = curVertices[a].y;
			bounds.maxY = curVertices[a].y;
		}

		if (curVertices[a].y < minY)
		{
			minY = curVertices[a].y;
			bounds.minY = curVertices[a].y;
		}

		if (curVertices[a].z > maxZ)
		{
			maxZ = curVertices[a].z;
			bounds.maxZ = curVertices[a].z;
		}

		if (curVertices[a].z < minZ)
		{
			minZ = curVertices[a].z;
			bounds.minZ = curVertices[a].z;
		}
	}

	//bounds.maxY += 0.1;
	//bounds.minY -= 0.1;

	return bounds;
}



vector <Vector3D> ClothWorld::getBoundedCMPoints(Vector3D p1, Vector3D p2, Vector3D p3)
{
	Bounds bounds1 = getTriangleBounds(p1, p2, p3);
	bounds1.maxX += 0.155;
	bounds1.maxY += 0.155;
	bounds1.maxZ += 0.155;
	bounds1.minX -= 0.155;
	bounds1.minY -= 0.155;
	bounds1.minZ -= 0.155;

	vector <Vector3D> boundedPoints;

	//Now get all triangles from the CM which intersect
	for (int a = 0; a < collisionMeshVertexVoxels.size(); a++)//Loop CM triangles
	{
		Bounds bounds2 = collisionMeshVertexVoxels[a].bounds;

		bool bounded = boundsOverlap(bounds1, bounds2);//Check if its overlapping

		if (bounded)
		{
			for (int b = 0; b < collisionMeshVertexVoxels[a].vertices.size(); b++)//Loop CM triangles
			{
				boundedPoints.push_back(collisionMeshVertexVoxels[a].vertices[b]);
			}
		}
	}

	return boundedPoints;
}

void ClothWorld::voxelizeCollisionMesh_Vertices()
{
	int chopX = 2;
	int chopY = 2;
	int chopZ = 2;

	//[Bounding box]
	float maxX = -8192;
	float maxY = -8192;
	float maxZ = -8192;

	float minX = 8192;
	float minY = 8192;
	float minZ = 8192;


	//[Compute mesh bounding box]
	for (int a = 0; a < objPointsMesh.originalVertexCount; a++)
	{

		if (objPointsMesh.originalVertices[a].x > maxX)
		{
			maxX = objPointsMesh.originalVertices[a].x;
		}

		if (objPointsMesh.originalVertices[a].x < minX)
		{
			minX = objPointsMesh.originalVertices[a].x;
		}

		if (objPointsMesh.originalVertices[a].y > maxY)
		{
			maxY = objPointsMesh.originalVertices[a].y;
		}

		if (objPointsMesh.originalVertices[a].y < minY)
		{
			minY = objPointsMesh.originalVertices[a].y;
		}

		if (objPointsMesh.originalVertices[a].z > maxZ)
		{
			maxZ = objPointsMesh.originalVertices[a].z;
		}

		if (objPointsMesh.originalVertices[a].z < minZ)
		{
			minZ = objPointsMesh.originalVertices[a].z;
		}
	}

	float widthX = ((maxX + 0.05) - (minX - 0.05)) / chopX;
	float widthY = ((maxY + 0.05) - (minY - 0.05)) / chopY;
	float widthZ = ((maxZ + 0.05) - (minZ - 0.05)) / chopZ;

	Cube initialCube;

	initialCube.p1.x = minX;
	initialCube.p1.y = minY;
	initialCube.p1.z = minZ;

	initialCube.p2.x = minX + widthX;
	initialCube.p2.y = minY;
	initialCube.p2.z = minZ;

	initialCube.p3.x = minX;
	initialCube.p3.y = minY + widthY;
	initialCube.p3.z = minZ;

	initialCube.p4.x = minX + widthX;
	initialCube.p4.y = minY + widthY;
	initialCube.p4.z = minZ;

	//

	initialCube.p5.x = minX;
	initialCube.p5.y = minY;
	initialCube.p5.z = minZ + widthZ;

	initialCube.p6.x = minX + widthX;
	initialCube.p6.y = minY;
	initialCube.p6.z = minZ + widthZ;

	initialCube.p7.x = minX;
	initialCube.p7.y = minY + widthY;
	initialCube.p7.z = minZ + widthZ;

	initialCube.p8.x = minX + widthX;
	initialCube.p8.y = minY + widthY;
	initialCube.p8.z = minZ + widthZ;

	cubes.push_back(initialCube);

	int xN = 0;
	int yN = 0;
	int zN = 0;


	int cn = 0;
	while (1)
	{
		xN++;

		if (xN >= chopX && yN >= chopY - 1 && zN >= chopZ - 1)
		{
			break;
		}

		if (xN >= chopX)
		{
			xN = 0;
			yN++;
		}

		if (yN >= chopY)
		{
			yN = 0;
			zN++;
		}

		{
			Cube curCube = initialCube;
			curCube.translate(0, widthX * xN);
			curCube.translate(1, widthY * yN);
			curCube.translate(2, widthZ * zN);

			cubes.push_back(curCube);
			/*
			if (cn == 0)
			{
				vectorsToDraw[vectorToDrawCount] = curCube.p1;
				vectorToDrawCount++;
				vectorsToDraw[vectorToDrawCount] = curCube.p2;
				vectorToDrawCount++;
				vectorsToDraw[vectorToDrawCount] = curCube.p3;
				vectorToDrawCount++;
				vectorsToDraw[vectorToDrawCount] = curCube.p4;
				vectorToDrawCount++;
				vectorsToDraw[vectorToDrawCount] = curCube.p5;
				vectorToDrawCount++;
				vectorsToDraw[vectorToDrawCount] = curCube.p6;
				vectorToDrawCount++;
				vectorsToDraw[vectorToDrawCount] = curCube.p7;
				vectorToDrawCount++;
				vectorsToDraw[vectorToDrawCount] = curCube.p8;
				vectorToDrawCount++;
			}
			*/
			cn++;

		}
	}

	//cout << "-->" << cubes.size() << endl;
	//system("PAUSE");

	//[Compute the voxels]
	for (int a = 0; a < cubes.size(); a++)//Iterate through segmented CM cubes
	{
		Cube curCube = cubes[a];

		Bounds curBound = computeBounds(curCube);//Get bounds of cur cube

		//[Compute voxel centroid]
		float xAvg = (curCube.p1.x + curCube.p2.x + curCube.p3.x + curCube.p4.x + curCube.p5.x + curCube.p6.x + curCube.p7.x + curCube.p8.x) / 8;
		float yAvg = (curCube.p1.y + curCube.p2.y + curCube.p3.y + curCube.p4.y + curCube.p5.y + curCube.p6.y + curCube.p7.y + curCube.p8.y) / 8;
		float zAvg = (curCube.p1.z + curCube.p2.z + curCube.p3.z + curCube.p4.z + curCube.p5.z + curCube.p6.z + curCube.p7.z + curCube.p8.z) / 8;

		VertVoxel curVertVoxel;

		curVertVoxel.bounds = curBound;
		curVertVoxel.centroid.x = xAvg;
		curVertVoxel.centroid.y = yAvg;
		curVertVoxel.centroid.z = zAvg;

		//[Get all CM vertices inside the current bounding box and store to meshVoxels]
		bool got = false;
		//for (int c = 0; c < colMesh1.indicesCount; c += 3)

		for (int c = 0; c < objPointsMesh.originalVertexCount; c++)
		{
			Vector3D point(objPointsMesh.originalVertices[c].x, objPointsMesh.originalVertices[c].y, objPointsMesh.originalVertices[c].z);

			//[Get if point inside bounds]
			bool in = false;
			if (point.x < curVertVoxel.bounds.maxX && point.x > curVertVoxel.bounds.minX)
			{
				if (point.y < curVertVoxel.bounds.maxY && point.y > curVertVoxel.bounds.minY)
				{
					if (point.z < curVertVoxel.bounds.maxZ && point.z > curVertVoxel.bounds.minZ)
					{
						got = true;
						curVertVoxel.vertices.push_back(point);
						//Vector3D point(curVertVoxel.vertices[d].x, curVertVoxel.vertices[d].y, curVertVoxel.vertices[d].z);
					}
				}
			}
		}

		if (got) { collisionMeshVertexVoxels.push_back(curVertVoxel); }
	}

	//cout << "--->" << collisionMeshVertexVoxels[0].vertices.size() << endl;

	return;
}


void ClothWorld::voxelizeCollisionMesh()
{

	int chopX = 6;
	int chopY = 6;
	int chopZ = 6;

	//Bounding box

	float maxX = -8192;
	float maxY = -8192;
	float maxZ = -8192;

	float minX = 8192;
	float minY = 8192;
	float minZ = 8192;

	//Compute mesh bounding box
	for (int a = 0; a < collisionMesh.originalVertexCount; a++)
	{
		if (collisionMesh.originalVertices[a].x > maxX)
		{
			maxX = collisionMesh.originalVertices[a].x;
		}

		if (collisionMesh.originalVertices[a].x < minX)
		{
			minX = collisionMesh.originalVertices[a].x;
		}

		if (collisionMesh.originalVertices[a].y > maxY)
		{
			maxY = collisionMesh.originalVertices[a].y;
		}

		if (collisionMesh.originalVertices[a].y < minY)
		{
			minY = collisionMesh.originalVertices[a].y;
		}

		if (collisionMesh.originalVertices[a].z > maxZ)
		{
			maxZ = collisionMesh.originalVertices[a].z;
		}

		if (collisionMesh.originalVertices[a].z < minZ)
		{
			minZ = collisionMesh.originalVertices[a].z;
		}
	}

	float widthX = ((maxX + 0.01) - (minX - 0.01)) / chopX;
	float widthY = ((maxY + 0.01) - (minY - 0.01)) / chopY;
	float widthZ = ((maxZ + 0.01) - (minZ - 0.01)) / chopZ;

	//cout << widthX << " " << widthY << " " << widthZ << endl;
	//cout << minX << " " << minY << " " << minZ << endl;
	//system("PAUSE");

	//Get cube for min x,y,z with given a width.

	Cube initialCube;

	initialCube.p1.x = minX;
	initialCube.p1.y = minY;
	initialCube.p1.z = minZ;

	initialCube.p2.x = minX + widthX;
	initialCube.p2.y = minY;
	initialCube.p2.z = minZ;

	initialCube.p3.x = minX;
	initialCube.p3.y = minY + widthY;
	initialCube.p3.z = minZ;

	initialCube.p4.x = minX + widthX;
	initialCube.p4.y = minY + widthY;
	initialCube.p4.z = minZ;

	//

	initialCube.p5.x = minX;
	initialCube.p5.y = minY;
	initialCube.p5.z = minZ + widthZ;

	initialCube.p6.x = minX + widthX;
	initialCube.p6.y = minY;
	initialCube.p6.z = minZ + widthZ;

	initialCube.p7.x = minX;
	initialCube.p7.y = minY + widthY;
	initialCube.p7.z = minZ + widthZ;

	initialCube.p8.x = minX + widthX;
	initialCube.p8.y = minY + widthY;
	initialCube.p8.z = minZ + widthZ;

	cubes.push_back(initialCube);

	//Cube curCube = initialCube;

	int xN = 0;
	int yN = 0;
	int zN = 0;
	while (1)
	{
		xN++;

		if (xN >= chopX && yN >= chopY - 1 && zN >= chopZ - 1)
		{
			break;
		}

		if (xN >= chopX)
		{
			xN = 0;
			yN++;
		}

		if (yN >= chopY)
		{
			yN = 0;
			zN++;
		}

		{
			Cube curCube = initialCube;
			curCube.translate(0, widthX * xN);
			curCube.translate(1, widthY * yN);
			curCube.translate(2, widthZ * zN);

			cubes.push_back(curCube);
		}
	}

	//[Compute the voxels]
	for (int a = 0; a < cubes.size(); a++)//Iterate through segmented CM cubes
	{
		Cube curCube = cubes[a];

		Bounds curBound = computeBounds(curCube);//Get bounds of cur cube

		//[Compute voxel centroid]
		float xAvg = (curCube.p1.x + curCube.p2.x + curCube.p3.x + curCube.p4.x + curCube.p5.x + curCube.p6.x + curCube.p7.x + curCube.p8.x) / 8;
		float yAvg = (curCube.p1.y + curCube.p2.y + curCube.p3.y + curCube.p4.y + curCube.p5.y + curCube.p6.y + curCube.p7.y + curCube.p8.y) / 8;
		float zAvg = (curCube.p1.z + curCube.p2.z + curCube.p3.z + curCube.p4.z + curCube.p5.z + curCube.p6.z + curCube.p7.z + curCube.p8.z) / 8;

		//curBound.maxY += 0.2;
		//curBound.maxX += 0.2;
		//curBound.maxY += 0.1;
		//curBound.maxZ += 0.2;

		//curBound.minY -= 0.2;
		//curBound.minX -= 0.2;
		//curBound.minY -= 0.1;
		//curBound.minZ -= 0.2;

		MeshVoxel curVoxel;

		curVoxel.bounds = curBound;
		curVoxel.centroid.x = xAvg;
		curVoxel.centroid.y = yAvg;
		curVoxel.centroid.z = zAvg;

		//[Get all CM triangles inside the current bounding box and store to meshVoxels]

		bool got = false;
		for (int c = 0; c < collisionMesh.indicesCount; c += 3)
		{
			int i1 = collisionMesh.indices[c];
			int i2 = collisionMesh.indices[c + 1];
			int i3 = collisionMesh.indices[c + 2];

			//[CM triangle]:
			Vector3D vert1(collisionMesh.vertices[i1].x, collisionMesh.vertices[i1].y, collisionMesh.vertices[i1].z);
			Vector3D vert2(collisionMesh.vertices[i2].x, collisionMesh.vertices[i2].y, collisionMesh.vertices[i2].z);
			Vector3D vert3(collisionMesh.vertices[i3].x, collisionMesh.vertices[i3].y, collisionMesh.vertices[i3].z);

			Triangle curTri(vert1, vert2, vert3);

			//[Get if triangle intersects the bounding box]
			//[Get triangle bounding box]

			Bounds tb = getTriangleBounds(vert1, vert2, vert3);

			bool overlaps = boundsOverlap(tb, curBound);

			if (overlaps) { curVoxel.triangles.push_back(curTri); got = true; }
		}

		if (got) { collisionMeshVoxels.push_back(curVoxel); }
	}

	return;
}

bool ClothWorld::boundsOverlap(Bounds a, Bounds b)
{
	bool Xi = false;
	bool Yi = false;
	bool Zi = false;


	if (a.maxY >= b.maxY)
	{
		if (a.minY <= b.maxY) { Yi = true; }
	}

	if (a.maxY <= b.maxY)
	{
		if (a.maxY >= b.minY) { Yi = true; }
	}



	if (a.maxX >= b.maxX)
	{
		if (a.minX <= b.maxX) { Xi = true; }
	}

	if (a.maxX <= b.maxX)
	{
		if (a.maxX >= b.minX) {	Xi = true; }
	}

	

	if (a.maxZ >= b.maxZ)
	{
		if (a.minZ <= b.maxZ) { Zi = true; }
	}

	if (a.maxZ <= b.maxZ)
	{
		if (a.maxZ >= b.minZ) { Zi = true; }
	}

	if (Zi == true && Xi == true && Yi == true)
	{
		return true;
	}

	return false;
}

/*
//Get vector which is aligned parallel to the face from the input vector
//Magnitude can simulate friction
Vector3D ClothWorld::getParallelVector(Triangle tri, Vector3D incident)
{

	//NEW
	Vector3D centroid2((tri.p1.x + tri.p2.x + tri.p3.x) / 3, (tri.p1.y + tri.p2.y + tri.p3.y) / 3, (tri.p1.z + tri.p2.z + tri.p3.z) / 3);
	Vector3D triangleNormal3 = (vector3DUtils.getTriangleNormal(tri.p1, tri.p2, tri.p3));
	Vector3D toClosest1c = vector3DUtils.closestPlanePoint(Vector3D(0, 0, 0), centroid2, triangleNormal3);//Origin of parallel vector (zero)
	Vector3D toClosest2c = vector3DUtils.closestPlanePoint(incident, centroid2, triangleNormal3);//Tip of parallel vector
	Vector3D finalVector = vector3DUtils.vectorToPoint(toClosest1c, toClosest2c);
	//finalVector = vector3DUtils.setVectorMagnitude(finalVector, 1.0);//Dont set magnitude
	return finalVector;
}
*/

/*
Vector3D ClothWorld::vectorAwayFromTriangle(Triangle input)
{
	float amount = 0.4;

	Vector3D U(0, 0, 0);
	Vector3D V(0, 0, 0);
	U = input.p2 - input.p1;
	V = input.p3 - input.p1;

	Vector3D Normal(0, 0, 0);
	Normal.x = (U.y * V.z) - (U.z * V.y);
	Normal.y = (U.z * V.x) - (U.x * V.z);
	Normal.z = (U.x * V.y) - (U.y * V.x);

	return Normal;
}
*/

//Get voxel vertices in range of point
vector <Vector3D> ClothWorld::getNearestVoxelVertices(Vector3D postion)
{
	float closestDistance = 2048;
	int closestIndex = 0;
	for (int a = 0; a < collisionMeshVoxels.size(); a++)
	{
		float curDist = utils.dist(collisionMeshVoxels[a].centroid.x, collisionMeshVoxels[a].centroid.y, collisionMeshVoxels[a].centroid.z, postion.x, postion.y, postion.z);

		if (curDist < closestDistance)
		{
			closestDistance = curDist;
			closestIndex = a;
		}
	}

	//Return vertices of the closest voxel

	int a;
	vector <Triangle> tris = getCMTriangles(collisionMeshVoxels[closestIndex].bounds, &a);

	vector <Vector3D> verts;

	for (int a = 0; a < tris.size(); a++)
	{
		verts.push_back(tris[a].p1);
		verts.push_back(tris[a].p2);
		verts.push_back(tris[a].p3);
	}

	return verts;
}



vector <Triangle> ClothWorld::getBoundedCMTriangles(Vector3D p1, Vector3D p2, Vector3D p3)
{
	Bounds bounds1 = getTriangleBounds(p1, p2, p3);
	bounds1.maxX += 0.155;
	bounds1.maxY += 0.155;
	bounds1.maxZ += 0.155;
	bounds1.minX -= 0.155;
	bounds1.minY -= 0.155;
	bounds1.minZ -= 0.155;

	vector <Triangle> boundedTriangles;

	//Now get all triangles from the CM which intersect
	for (int a = 0; a < CMeshTriangleBounds.size(); a++)//Loop CM triangles
	{
		Bounds bounds2 = CMeshTriangleBounds[a].bounds;

		bool bounded = boundsOverlap(bounds1, bounds2);//Check if its overlapping

		if (bounded)
		{
			boundedTriangles.push_back(CMeshTriangleBounds[a].triangle);
		}
	}

	return boundedTriangles;
}

vector <Triangle> ClothWorld::getCMTriangles(Bounds boundsA, int *voxelRef)
{
	vector <Triangle> tris;

	int voxelsCount = 0;

	for (int a = 0; a < collisionMeshVoxels.size(); a++)
	{
		if (boundsOverlap(boundsA, collisionMeshVoxels[a].bounds) == true)
		{
			voxelsCount++;
			for (int b = 0; b < collisionMeshVoxels[a].triangles.size(); b++)
			{
				tris.push_back(collisionMeshVoxels[a].triangles[b]);
			}
		}
	}


	//[Remove duplicates from vector]

	vector <Triangle> tris2;

	for (int a = 0; a < tris.size(); a++)
	{
		bool found = false;

		Triangle curTri = tris[a];

		for (int b = 0; b < tris2.size(); b++)
		{
			if (tris2[b].p1.x == curTri.p1.x && tris2[b].p1.y == curTri.p1.y && tris2[b].p1.z == curTri.p1.z)
			{
				if (tris2[b].p2.x == curTri.p2.x && tris2[b].p2.y == curTri.p2.y && tris2[b].p2.z == curTri.p2.z)
				{
					if (tris2[b].p3.x == curTri.p3.x && tris2[b].p3.y == curTri.p3.y && tris2[b].p3.z == curTri.p3.z)
					{
						found = true;
					}
				}
			}
		}

		if (found)
		{
		}
		else
		{
			tris2.push_back(curTri);
		}
	}

	return tris2;
}


vector <Vector3D> ClothWorld::getCMVertices(Bounds boundsA, int *voxelRef)
{
	vector <Vector3D> verts;

	int voxelsCount = 0;

	for (int a = 0; a < collisionMeshVertexVoxels.size(); a++)
	{
		if (boundsOverlap(boundsA, collisionMeshVertexVoxels[a].bounds) == true)
		{
			voxelsCount++;
			for (int b = 0; b < collisionMeshVertexVoxels[a].vertices.size(); b++)
			{
				verts.push_back(collisionMeshVertexVoxels[a].vertices[b]);
			}
		}
	}

	return verts;
}




Bounds ClothWorld::getTriangleBounds(Vector3D p1, Vector3D p2, Vector3D p3)
{
	vector <Vector3D> curVertices;
	curVertices.push_back(p1);
	curVertices.push_back(p2);
	curVertices.push_back(p3);

	float maxX = -8192;
	float maxY = -8192;
	float maxZ = -8192;

	float minX = 8192;
	float minY = 8192;
	float minZ = 8192;

	Bounds bounds;

	//Compute mesh bounding box
	for (int a = 0; a < curVertices.size(); a++)
	{
		if (curVertices[a].x > maxX)
		{
			maxX = curVertices[a].x;
			bounds.maxX = curVertices[a].x;
		}

		if (curVertices[a].x < minX)
		{
			minX = curVertices[a].x;
			bounds.minX = curVertices[a].x;
		}

		if (curVertices[a].y > maxY)
		{
			maxY = curVertices[a].y;
			bounds.maxY = curVertices[a].y;
		}

		if (curVertices[a].y < minY)
		{
			minY = curVertices[a].y;
			bounds.minY = curVertices[a].y;
		}

		if (curVertices[a].z > maxZ)
		{
			maxZ = curVertices[a].z;
			bounds.maxZ = curVertices[a].z;
		}

		if (curVertices[a].z < minZ)
		{
			minZ = curVertices[a].z;
			bounds.minZ = curVertices[a].z;
		}
	}

	return bounds;
}

int ClothWorld::getVoxelForPosition(Vector3D pos)
{
	return 0;
}

