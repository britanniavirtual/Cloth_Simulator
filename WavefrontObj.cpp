#include "JointHeader.h"

void WavefrontUtils::expandMesh(WavefrontObj *obj, float amount)
{
	computeAverageNormals(obj);

	for (int a = 0; a < obj->vertexCount; a++)
	{
		Vector3D expand(averageNormals[a].x, averageNormals[a].y, averageNormals[a].z);
		expand = vector3DUtils.setVectorMagnitude(expand, amount);

		obj->vertices[a].x += expand.x;
		obj->vertices[a].y += expand.y;
		obj->vertices[a].z += expand.z;
	}
}

//Triangles are defined by three unique points for simplicity of rendering (I.e. in DirectX) (No points shared between polys)
//Get the matching point arrays.
void WavefrontUtils::computeVertexGroups(WavefrontObj * obj)
{
	//vector <int> iterated;//Dont add duplicates

	for (int a = 0; a < obj->vertexCount; a++)//For every vertex
	{
		Vector3D curVertex1(obj->vertices[a].x, obj->vertices[a].y, obj->vertices[a].z);

		bool foundA = false;
		for (int b = 0; b < obj->vertexCount; b++)//Find matching
		{
			bool found = false;
			//if (b == a) { continue; }
			Vector3D curVertex2(obj->vertices[b].x, obj->vertices[b].y, obj->vertices[b].z);

			if (curVertex1.x == curVertex2.x && curVertex1.y == curVertex2.y && curVertex1.z == curVertex2.z)
			{
				found = true;
			}

			/*
			bool found2 = false;

			for (int c = 0; c < iterated.size(); c++)
			{
				if (iterated[c] == b)
				{
					found2 = true;
					break;
				}
			}
			*/

			if (found)// && !found2)
			{
				//cout << a << " - " << b << endl;
				matching[a][matchingCount[a]] = b;
				matchingCount[a]++;
				foundA = true;

				//iterated.push_back(b);
				//iterated.push_back(a);
			}
		}

		if (foundA) { foundCount++; }
	}
}

void WavefrontUtils::computeAverageNormals(WavefrontObj* obj)
{
	computeVertexGroups(obj);

	for (int a = 0; a < foundCount; a++)
	{
		Vector3D avgNormal(0, 0, 0);

		vector <Vector3D> curNormals;

		//Tgt vertex: Find all matching. NB: Some vertex indexes will have no matching!
		int curMatching = matching[a][0];//Only look at one (Expand to all in the group later)
		Vector3D tgtVertex(obj->vertices[curMatching].x, obj->vertices[curMatching].y, obj->vertices[curMatching].z);

		//Compute the average for every face that contains the tgt vertex
		for (int c = 0; c < obj->indicesCount; c += 3)//For every face
		{
			int i1 = obj->indices[c];
			int i2 = obj->indices[c + 1];
			int i3 = obj->indices[c + 2];

			bool found = false;

			if (obj->vertices[i1].x == tgtVertex.x && obj->vertices[i1].y == tgtVertex.y && obj->vertices[i1].z == tgtVertex.z)
			{
				found = true;
			}

			if (obj->vertices[i2].x == tgtVertex.x && obj->vertices[i2].y == tgtVertex.y && obj->vertices[i2].z == tgtVertex.z)
			{
				found = true;
			}

			if (obj->vertices[i3].x == tgtVertex.x && obj->vertices[i3].y == tgtVertex.y && obj->vertices[i3].z == tgtVertex.z)
			{
				found = true;
			}

			if (found == true)//All points have entries
			{
				Vector3D curNorm(obj->normals[i1].x, obj->normals[i1].y, obj->normals[i1].z);//Correct normal for this face
				//cout << "found. Normal: [" << curNorm.x << " " << curNorm.y << " " << curNorm.z << "]" << endl;
				curNormals.push_back(curNorm);
			}
		}

		//[Remove duplicate normals for correct computation. Should be maximum 3 matches for each point.]
		vector <Vector3D> uniqueNormals;

		for (int a = 0; a < curNormals.size(); a++)
		{
			Vector3D curNormal(curNormals[a].x, curNormals[a].y, curNormals[a].z);
			bool found = false;
			for (int a = 0; a < uniqueNormals.size(); a++)
			{
				if (curNormal.x == uniqueNormals[a].x && curNormal.y == uniqueNormals[a].y && curNormal.z == uniqueNormals[a].z)
				{
					found = true;
				}
			}

			if (!found) { uniqueNormals.push_back(curNormal); }
		}

		curNormals = uniqueNormals;

		//[Average normal. Compute average for every face that contained the tgt vertex (prior block)]
		for (int a = 0; a < curNormals.size(); a++)
		{
			avgNormal.x += curNormals[a].x;
			avgNormal.y += curNormals[a].y;
			avgNormal.z += curNormals[a].z;
		}

		avgNormal.x /= curNormals.size() - 1;
		avgNormal.y /= curNormals.size() - 1;
		avgNormal.z /= curNormals.size() - 1;

		for (int b = 0; b < matchingCount[a]; b++)
		{
			int curInt = matching[a][b];
			for (int b = 0; b < matchingCount[a]; b++)
			{
				averageNormals[curInt] = avgNormal;
			}
		}
	}
}


void WavefrontObj::extractFaceElements(string data, int *i1, int *i2, int *i3, int *i4, int *i5, int *i6, int *i7, int *i8, int *i9)//<--- Extract face data to the struct in a single pass
{
	unsigned short elementsInt[9];
	int curElement = 0;
	string strCurrentInt = "";
	for (unsigned short a = 0; a < data.length(); a++)
	{
		strCurrentInt += data[a];

		if (data[a + 1] == '/' || data[a + 1] == '\n' || data[a + 1] == ' ' || data[a + 1] == '\r' || a == data.length() - 1)
		{
			a++;
			//elements[curElement] = strCurrentInt;
			elementsInt[curElement] = atoi(strCurrentInt.c_str());
			//console.popup(strCurrentInt);
			strCurrentInt = "";
			curElement++;
		}
	}

	*i1 = elementsInt[0];
	*i2 = elementsInt[1];
	*i3 = elementsInt[2];
	*i4 = elementsInt[3];
	*i5 = elementsInt[4];
	*i6 = elementsInt[5];
	*i7 = elementsInt[6];
	*i8 = elementsInt[7];
	*i9 = elementsInt[8];
}

void WavefrontObj::extractFaceElementsQuad(string data, int *i1, int *i2, int *i3, int *i4, int *i5, int *i6, int *i7, int *i8, int *i9, int *i10, int *i11, int *i12)//<--- Extract face data to the struct in a single pass
{
	unsigned short elementsInt[12];
	int curElement = 0;
	string strCurrentInt = "";
	for (unsigned short a = 0; a < data.length(); a++)
	{
		strCurrentInt += data[a];

		if (data[a + 1] == '/' || data[a + 1] == '\n' || data[a + 1] == ' ' || data[a + 1] == '\r' || a == data.length() - 1)
		{
			a++;
			//elements[curElement] = strCurrentInt;
			elementsInt[curElement] = atoi(strCurrentInt.c_str());
			//console.popup(strCurrentInt);
			strCurrentInt = "";
			curElement++;
		}
	}

	*i1 = elementsInt[0];
	*i2 = elementsInt[1];
	*i3 = elementsInt[2];
	*i4 = elementsInt[3];
	*i5 = elementsInt[4];
	*i6 = elementsInt[5];
	*i7 = elementsInt[6];
	*i8 = elementsInt[7];
	*i9 = elementsInt[8];
	*i10 = elementsInt[9];
	*i11 = elementsInt[10];
	*i12 = elementsInt[11];
}


//-------------------------------
// Extract actual UV values
//-------------------------------
VEC2 WavefrontObj::extractUVs(string data)
{
	string vertexStr[2];

	bool inNumber = false;
	string curNumStr = "";
	int curNum = -1;

	for (int a = 0; a <= data.length(); a++)
	{
		if (inNumber == false && data[a] != ' ')
		{
			inNumber = true;
		}

		if (inNumber == true && data[a] == ' ')
		{
			curNum++;

			vertexStr[curNum] = curNumStr;
			curNumStr = "";
			inNumber = false;
		}

		if (inNumber == true && a == data.length())
		{
			curNum++;
			vertexStr[curNum] = curNumStr;//Stack overflow error [>2]
			curNumStr = "";
			inNumber = false;
		}

		if (inNumber == true && data[a] != ' ')
		{
			curNumStr += data[a];
		}
	}

	VEC2 uvOut;
	uvOut.x = atof(vertexStr[0].c_str());
	uvOut.y = atof(vertexStr[1].c_str());

	//VEC2 uvOut;
	//uvOut.x = 1;
	//uvOut.y = 1;

	return uvOut;
}


//-------------------------------
//
//-------------------------------
VEC3 WavefrontObj::extractFloat3(string data)
{
	string vertexStr[3];

	bool inNumber = false;
	string curNumStr = "";
	int curNum = -1;

	for (int a = 0; a <= data.length(); a++)
	{
		if (inNumber == false && data[a] != ' ')
		{
			inNumber = true;
		}

		if (inNumber == true && data[a] == ' ')
		{
			curNum++;

			vertexStr[curNum] = curNumStr;
			curNumStr = "";
			inNumber = false;
		}

		if (inNumber == true && a == data.length())
		{
			curNum++;
			vertexStr[curNum] = curNumStr;
			curNumStr = "";
			inNumber = false;
		}

		if (inNumber == true && data[a] != ' ')
		{
			curNumStr += data[a];
		}
	}

	VEC3 vertOut;
	vertOut.x = atof(vertexStr[0].c_str());
	vertOut.y = atof(vertexStr[1].c_str());
	vertOut.z = atof(vertexStr[2].c_str());

	return vertOut;
}

	

void WavefrontObj::writeObj_UsingOriginalStructure(string fileName)
{
	cout << "Writing (orig struct): '" << fileName << "'" << endl;
	cout << "originalVertexCount: " << originalVertexCount << endl;

	stringstream data;
	for (int a = 0; a < originalVertexCount; a++)
	{
		data << "v " << originalVertices[a].x << " " << originalVertices[a].y << " " << originalVertices[a].z << endl;
	}

	data << endl;

	for (int a = 1; a < originalUVCount + 1; a++)
	{
		data << "vt " << originalUvs[a].x << " " << originalUvs[a].y << endl;
	}

	data << endl;
	for (int a = 0; a < originalNormalCount; a++)
	{
		data << "vn " << originalNormals[a].x << " " << originalNormals[a].y << " " << originalNormals[a].z << endl;
	}

	data << endl;

	for (int a = 0; a < faceCount + 1; a ++)
	{
		data << "f " << (originalIndices[a][0]) + 1 << "/" << originalIndices[a][1] + 1 << "/" << originalIndices[a][2] + 1 << " ";
		data << (originalIndices[a][3]) + 1 << "/" << originalIndices[a][4] + 1 << "/" << originalIndices[a][5] + 1 << " ";
		data << (originalIndices[a][6]) + 1 << "/" << originalIndices[a][7] + 1 << "/" << originalIndices[a][8] + 1 << endl;
	}

	ofstream myfile;
	myfile.open(fileName);
	myfile << data.str();
	myfile.close();
}

void WavefrontObj::writeObjVerticesOnly(string fileName)
{
	cout << "Writing: '" << fileName << "'" << endl;

	stringstream data;
	for (int a = 0; a < originalVertexCount; a++)
	{
		data << " v " << originalVertices[a].x << " " << originalVertices[a].y << " " << originalVertices[a].z << endl;
	}

	ofstream myfile;
	myfile.open(fileName);
	myfile << data.str();
	myfile.close();
}

//Write a triangle obj mesh from the raw vertex / uv / normal data loaded
void WavefrontObj::writeObj(string fileName)
{
	cout << "Writing " << fileName << endl;

	stringstream data;
	for (int a = 0; a < vertexCount; a++)
	{
		data << " v " << vertices[a].x << " " << vertices[a].y << " " << vertices[a].z << endl;
	}

	data << endl;

	for (int a = 0; a < uvCount; a++)
	{
		data << " vt " << uvs[a].y << " " << uvs[a].x << endl;
	}

	data << endl;

	if (vertexCount != uvCount)
	{
		for (int a = 0; a < vertexCount; a += 3)
		{
			data << " f " << a + 1 << " " << a + 2 << " " << a + 3 << endl;
		}
	}
	else
	{
		for (int a = 0; a < vertexCount; a += 3)
		{
			data << " f " << a + 1 << "/" << a + 1 << " " << a + 2 << "/" << a + 2 << " " << a + 3 << "/" << a + 3 << endl;
		}
	}

	ofstream myfile;
	myfile.open(fileName);
	myfile << data.str();
	myfile.close();
}

//Load an obj file which consists of quads
void WavefrontObj::loadQuadObj(string fileName)
{
	cout << "Loading: " << fileName << "..." << endl;

	//[Load file]

	string line;
	ifstream myfile(fileName);
	string data = "";
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			data += line + '\n';
		}
		myfile.close();
	}

	//[Parse file]

	//Reset(Essential)
	int curVertex = -1;
	bool inVert = false;
	curVertCount = 0;
	curUVCount = 0;
	curVtCount = 0;
	curVnCount = 0;

	//NB: Parse file data in single pass through the data
	int curVt = 0;
	bool inVt = false;
	stringstream str2;
	int curVn = 0;
	bool inVn = false;
	faceCount = -1;
	bool inFace = false;
	int curMtlInteger = 0;

	string vnStr = "";
	string vertStr = "";

	for (int i = 0; i < data.length(); i++)
	{
		//[(1) Read initial vertex list]
		if (data[i] == 'v' && data[i + 1] == ' ')
		{
			inVert = true;
			curVertex++;
			originalVertexCount++;
			//vertStrings[curVertex] = ' ';
			i += 2;
		}

		if (data[i] == '\n' && inVert == true)
		{
			VEC3 tmpVec = extractFloat3(vertStr);
			curVerts[curVtCount].x = tmpVec.x;
			curVerts[curVtCount].y = tmpVec.y;
			curVerts[curVtCount].z = tmpVec.z;

			originalVertices[curVtCount].x = tmpVec.x;
			originalVertices[curVtCount].y = tmpVec.y;
			originalVertices[curVtCount].z = tmpVec.z;

			curVtCount++;
			vertCount2++;

			vertStr = "";
			inVert = false;
		}

		if (inVert == true)
		{
			vertStr += data[i];
		}


		//[(3) Read vn list]

		if (data[i] == 'v' && data[i + 1] == 'n' && data[i + 2] == ' ' && !inVt)
		{
			inVn = true;
			curVn++;
			//vnStrings[curVn] = ' ';
			i += 3;

		}
		if (data[i] == '\n' && inVn == true)
		{
			VEC3 curVn = extractFloat3(vnStr);

			curVns[curVnCount].x = curVn.x;
			curVns[curVnCount].y = curVn.y;
			curVns[curVnCount].z = curVn.z;

			originalNormals[originalNormalCount] = curVn;
			originalNormalCount++;

			curVnCount++;
			vnStr = "";
			inVn = false;
		}

		if (inVn == true)
		{
			vnStr += data[i];
		}

		if (data[i] == 'f' && data[i + 1] == ' ')
		{
			inFace = true;
			faceCount++;
			faceStrings[faceCount] = ' ';
			i += 2;
		}

		if (data[i] == '\n' && inFace == true)
		{
			inFace = false;
		}

		if (inFace == true)
		{
			faceStrings[faceCount] += data[i];
		}
	}


	//[Parse quad faces]

	vertexCount = 0;
	uvCount = 0;
	normalCount = 0;
	indicesCount = 0;
	curVertCount = 0;

	int i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12;
	for (int a = 0; a <= faceCount; a++)//For every face line from the obj file
	{
		extractFaceElementsQuad(faceStrings[a], &i1, &i2, &i3, &i4, &i5, &i6, &i7, &i8, &i9, &i10, &i11, &i12);

		originalIndicesQuads[a][0] = i1 - 1;
		originalIndicesQuads[a][1] = i2 - 1;
		originalIndicesQuads[a][2] = i3 - 1;

		originalIndicesQuads[a][3] = i4 - 1;
		originalIndicesQuads[a][4] = i5 - 1;
		originalIndicesQuads[a][5] = i6 - 1;

		originalIndicesQuads[a][6] = i7 - 1;
		originalIndicesQuads[a][7] = i8 - 1;
		originalIndicesQuads[a][8] = i9 - 1;

		originalIndicesQuads[a][9] = i10 - 1;
		originalIndicesQuads[a][10] = i11 - 1;
		originalIndicesQuads[a][11] = i12 - 1;

		originalIndicesQuadCount++;

		//Vertices
		//This is affected by the preceding statements
		VEC3 curVert1 = curVerts[i1 - 1];
		VEC3 curVert2 = curVerts[i4 - 1];
		VEC3 curVert3 = curVerts[i7 - 1];
		VEC3 curVert4 = curVerts[i10 - 1];

		curVertCount += 3;

		verticesQuads[vertexQuadCount].x = curVert1.x;
		verticesQuads[vertexQuadCount].y = curVert1.y;
		verticesQuads[vertexQuadCount].z = curVert1.z;
		vertexQuadCount++;

		verticesQuads[vertexQuadCount].x = curVert2.x;
		verticesQuads[vertexQuadCount].y = curVert2.y;
		verticesQuads[vertexQuadCount].z = curVert2.z;
		vertexQuadCount++;

		verticesQuads[vertexQuadCount].x = curVert3.x;
		verticesQuads[vertexQuadCount].y = curVert3.y;
		verticesQuads[vertexQuadCount].z = curVert3.z;
		vertexQuadCount++;

		verticesQuads[vertexQuadCount].x = curVert4.x;
		verticesQuads[vertexQuadCount].y = curVert4.y;
		verticesQuads[vertexQuadCount].z = curVert4.z;
		vertexQuadCount++;

		//ToDo:
		/*
		//[UVs]
		VEC2 uv1 = extractUVs(vtStrings[i2]);
		VEC2 uv2 = extractUVs(vtStrings[i5]);
		VEC2 uv3 = extractUVs(vtStrings[i8]);

		uvs[uvCount].x = uv1.y;
		uvs[uvCount].y = uv1.x;
		uvCount++;

		uvs[uvCount].x = uv2.y;
		uvs[uvCount].y = uv2.x;
		uvCount++;

		uvs[uvCount].x = uv3.y;
		uvs[uvCount].y = uv3.x;
		uvCount++;

		//[Normals]
		VEC3 curNormal1 = curVns[i3 - 1];
		VEC3 curNormal2 = curVns[i6 - 1];
		VEC3 curNormal3 = curVns[i9 - 1];

		normals[normalCount].x = curNormal1.x;
		normals[normalCount].y = curNormal1.y;
		normals[normalCount].z = curNormal1.z;

		normalCount++;

		normals[normalCount].x = curNormal2.x;
		normals[normalCount].y = curNormal2.y;
		normals[normalCount].z = curNormal2.z;

		normalCount++;

		normals[normalCount].x = curNormal3.x;
		normals[normalCount].y = curNormal3.y;
		normals[normalCount].z = curNormal3.z;

		normalCount++;
		*/
	}
}

//Load an obj file which consists of triangles
void WavefrontObj::loadObj(string fileName)
{
	cout << "Loading: " << fileName << "..." << endl;

	//[Load file]

	string line;
	ifstream myfile(fileName);
	string data = "";
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			data += line + '\n';
		}
		myfile.close();
	}

	//[Parse file]

	//Reset(Essential)
	int curVertex = -1;
	bool inVert = false;
	curVertCount = 0;
	curUVCount = 0;
	curVtCount = 0;
	curVnCount = 0;

	//NB: Parse file data in single pass through the data
	int curVt = 0;
	bool inVt = false;
	stringstream str2;
	int curVn = 0;
	bool inVn = false;
	faceCount = -1;
	bool inFace = false;
	int curMtlInteger = 0;

	string vnStr = "";
	string vertStr = "";

	for (int i = 0; i < data.length(); i++)
	{
		//[(1) Read initial vertex list]
		if (data[i] == 'v' && data[i + 1] == ' ')
		{
			inVert = true;
			curVertex++;
			originalVertexCount++;
			//vertStrings[curVertex] = ' ';
			i += 2;
		}

		if (data[i] == '\n' && inVert == true)
		{
			VEC3 tmpVec = extractFloat3(vertStr);
			curVerts[curVtCount].x = tmpVec.x;
			curVerts[curVtCount].y = tmpVec.y;
			curVerts[curVtCount].z = tmpVec.z;

			originalVertices[curVtCount].x = tmpVec.x;
			originalVertices[curVtCount].y = tmpVec.y;
			originalVertices[curVtCount].z = tmpVec.z;

			curVtCount++;
			vertCount2++;

			vertStr = "";
			inVert = false;
		}

		if (inVert == true)
		{
			//vertStrings[curVertex] += data[i];
			vertStr += data[i];
		}

		
		//[(2) Read vt list]
		//NB: Compute the vt strings first, update the floats in the model object when computing the faces

		if (data[i] == 'v' && data[i + 1] == 't' && data[i + 2] == ' ' && !inVt)
		{
			//string curVtStr = vtStrings[curVt];

			inVt = true;
			curVt++;
			vtStrings[curVt] = "";
			i += 3;
		}

		if (data[i] == '\n' && inVt == true)
		{
			inVt = false;
			originalUVCount++;
			originalUvs[originalUVCount].x = extractUVs(vtStrings[originalUVCount]).x;
			originalUvs[originalUVCount].y = extractUVs(vtStrings[originalUVCount]).y;
		}

		if (inVt == true)
		{
			vtStrings[curVt] += data[i];
		}
		

		//[(3) Read vn list]

		if (data[i] == 'v' && data[i + 1] == 'n' && data[i + 2] == ' ' && !inVt)
		{
			inVn = true;
			curVn++;
			//vnStrings[curVn] = ' ';
			i += 3;

		}

		if (data[i] == '\n' && inVn == true)
		{
			VEC3 curVn = extractFloat3(vnStr);

			//strFileDebug << curVn.x << " , " << curVn.y << " , " << curVn.z << endl;

			curVns[curVnCount].x = curVn.x;
			curVns[curVnCount].y = curVn.y;
			curVns[curVnCount].z = curVn.z;
			
			originalNormals[originalNormalCount] = curVn;
			originalNormalCount++;

			curVnCount++;
			vnStr = "";
			inVn = false;
		}

		if (inVn == true)
		{
			//vnStrings[curVn] += data[i];
			vnStr += data[i];
		}

		if (data[i] == 'f' && data[i + 1] == ' ')
		{
			inFace = true;
			faceCount++;
			faceStrings[faceCount] = ' ';
			i += 2;
			//faceTextures[curFace] = curMtlInteger;
		}

		if (data[i] == '\n' && inFace == true)
		{
			//cout << faceStrings[faceCount] << endl;
			inFace = false;
		}

		if (inFace == true)
		{
			faceStrings[faceCount] += data[i];
		}
	}

	vertexCount = 0;
	uvCount = 0;
	normalCount = 0;
	indicesCount = 0;
	curVertCount = 0;

	int i1, i2, i3, i4, i5, i6, i7, i8, i9;
	for (int a = 0; a <= faceCount; a++)//For every face line from the obj file
	{
		extractFaceElements(faceStrings[a], &i1, &i2, &i3, &i4, &i5, &i6, &i7, &i8, &i9);

		originalIndices[a][0] = i1 - 1;
		originalIndices[a][1] = i2 - 1;
		originalIndices[a][2] = i3 - 1;

		originalIndices[a][3] = i4 - 1;
		originalIndices[a][4] = i5 - 1;
		originalIndices[a][5] = i6 - 1;

		originalIndices[a][6] = i7 - 1;
		originalIndices[a][7] = i8 - 1;
		originalIndices[a][8] = i9 - 1;

		originalIndicesCount++;

		//[Vertices]
		VEC3 curVert1 = curVerts[i1 - 1];
		VEC3 curVert2 = curVerts[i4 - 1];
		VEC3 curVert3 = curVerts[i7 - 1];

		curVertCount += 3;

		vertices[vertexCount].x = curVert1.x;
		vertices[vertexCount].y = curVert1.y;
		vertices[vertexCount].z = curVert1.z;
		vertexCount++;

		vertices[vertexCount].x = curVert2.x;
		vertices[vertexCount].y = curVert2.y;
		vertices[vertexCount].z = curVert2.z;
		vertexCount++;

		vertices[vertexCount].x = curVert3.x;
		vertices[vertexCount].y = curVert3.y;
		vertices[vertexCount].z = curVert3.z;
		vertexCount++;

		//[UVs]
		VEC2 uv1 = extractUVs(vtStrings[i2]);
		VEC2 uv2 = extractUVs(vtStrings[i5]);
		VEC2 uv3 = extractUVs(vtStrings[i8]);

		uvs[uvCount].x = uv1.y;
		uvs[uvCount].y = uv1.x;
		uvCount++;
		
		uvs[uvCount].x = uv2.y;
		uvs[uvCount].y = uv2.x;
		uvCount++;

		uvs[uvCount].x = uv3.y;
		uvs[uvCount].y = uv3.x;
		uvCount++;
		

		//[Normals]
		VEC3 curNormal1 = curVns[i3 - 1];
		VEC3 curNormal2 = curVns[i6 - 1];
		VEC3 curNormal3 = curVns[i9 - 1];

		normals[normalCount].x = curNormal1.x;
		normals[normalCount].y = curNormal1.y;
		normals[normalCount].z = curNormal1.z;

		normalCount++;

		normals[normalCount].x = curNormal2.x;
		normals[normalCount].y = curNormal2.y;
		normals[normalCount].z = curNormal2.z;

		normalCount++;

		normals[normalCount].x = curNormal3.x;
		normals[normalCount].y = curNormal3.y;
		normals[normalCount].z = curNormal3.z;

		normalCount++;
	}

	//[Compute indices]
	for (int a = 0; a < curVertCount; a += 3)
	{
		indices[a] = a + 2;
		indices[a + 1] = a + 1;
		indices[a + 2] = a;
		indicesCount += 3;
	}

	cout << vertexCount << " vertices loaded." << endl;
	cout << vertexCount / 3 << " triangles loaded." << endl;

	UVCount = curVt;
}