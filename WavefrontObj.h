#pragma once

#define MAXINDEX 100000

class VEC2
{
public:
	float x;
	float y;
};

class VEC3
{
public:
	float x;
	float y;
	float z;
};

class VEC4
{
public:
	float x;
	float y;
	float z;
	float w;
};

class WavefrontObj
{
public:

	WavefrontObj()
	{
		vertices = new VEC3[MAXINDEX];
		verticesQuads = new VEC3[MAXINDEX];
		normals = new VEC3[MAXINDEX];
		indices = new int[MAXINDEX];
		uvs = new VEC2[MAXINDEX];
		vertStrings = new string[MAXINDEX];
		faceStrings = new string[MAXINDEX];
		vtStrings = new string[MAXINDEX];
		faceTextures = new string[MAXINDEX];
		mtlNames = new string[64];
		curVerts = new VEC3[MAXINDEX];
		curVns = new VEC3[MAXINDEX];

		//Orig indices
		originalIndices = new int*[MAXINDEX];
		for (int i = 0; i < MAXINDEX; i++)
		{
			originalIndices[i] = new int[9];
		}

		//Orig indices
		originalIndicesQuads = new int*[MAXINDEX];
		for (int i = 0; i < MAXINDEX; i++)
		{
			originalIndicesQuads[i] = new int[12];
		}


		originalVertices = new VEC3[MAXINDEX];
		originalNormals = new VEC3[MAXINDEX];
		originalUvs = new VEC2[MAXINDEX];
	}

	~WavefrontObj()
	{
		delete []vertices;
		delete []normals;
		delete []uvs;
		delete []indices;

		delete []verticesQuads;

		delete []vertStrings;
		delete []faceStrings;
		delete []vtStrings;
		delete []faceTextures;
		delete []mtlNames;
		delete []curVerts;
		delete []curVns;

		for (int i = 0; i < MAXINDEX; i++)
		{
			delete[]originalIndicesQuads[i];
		}

		delete[]originalIndicesQuads;

		for (int i = 0; i < MAXINDEX; i++)
		{
			delete []originalIndices[i];
		}

		delete []originalIndices;

		delete []originalVertices;
		delete []originalNormals;
		delete []originalUvs;
	}

	VEC3 *vertices;
	VEC3 *normals;
	VEC2 *uvs;
	int *indices;
	string *vertStrings;
	string *faceStrings;
	string *vtStrings;
	string *faceTextures;
	string *mtlNames;
	VEC3 *curVerts;
	VEC3 *curVns;

	//Original as parsed from the file (but n -= 1)
	int **originalIndices;
	int originalIndicesCount = 0;
	VEC3 *originalVertices;
	VEC3 *originalNormals;
	VEC2 *originalUvs;

	//Quads
	int **originalIndicesQuads;
	int originalIndicesQuadCount = 0;
	VEC3 *verticesQuads;
	VEC3 *normalsQuads;
	int vertexQuadCount = 0;

	int originalVertexCount = 0;
	int originalNormalCount = 0;
	int originalUVCount = 0;

	int vertexCount = 0;
	int normalCount = 0;
	int uvCount = 0;
	int indicesCount = 0;
	int curVtCount = 0;
	int curVertCount = 0;
	int curVnCount = 0;
	int curUVCount = 0;
	int vertCount2 = 0;
	int faceCount = -1;
	int UVCount = 0;
	
	VEC3 extractFloat3(string data);
	void extractFaceElements(string data, int *i1, int *i2, int *i3, int *i4, int *i5, int *i6, int *i7, int *i8, int *i9);
	void extractFaceElementsQuad(string data, int *i1, int *i2, int *i3, int *i4, int *i5, int *i6, int *i7, int *i8, int *i9, int *i810, int *i11, int *i12);
	VEC2 extractUVs(string data);
	
	void writeObj(string ouputFileName);
	void writeObj_UsingOriginalStructure(string ouputFileName);//Use the original Obj structure
	void writeObjVerticesOnly(string fileName);

	void loadObj(string FileName);
	void loadQuadObj(string FileName);
};


class WavefrontUtils
{
public:
	void expandMesh(WavefrontObj *obj, float amount);

private:
	void computeVertexGroups(WavefrontObj *obj);
	void computeAverageNormals(WavefrontObj *obj);

	int matching[16384][16];
	int matchingCount[16384];
	int foundCount = 0;
	Vector3D averageNormals[16384];//Computed Average for unique vertex
};
