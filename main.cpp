#include "JointHeader.h"

#pragma comment(lib, "glew32.lib")

const int width = 1024;
const int height = 1024;
const int GRID_SIZE = 5;
const float timeStep = 1/60.0f;

float currentTime = 0;
double accumulator = timeStep;
int selected_index = -1;

int oldX=0, oldY=0;
float rX=15, rY=0;
int state = 1;
float dist= -23;

GLint viewport[4];
GLdouble MV[16];
GLdouble P[16];

glm::vec3 Up=glm::vec3(0,1,0), Right, viewDir;

LARGE_INTEGER frequency;//Ticks per second
LARGE_INTEGER t1, t2;
double frameTimeQP=0;
float frameTime = 0;
float startTime = 0;
float fps = 0;
int totalFrames = 0;
char info[MAX_PATH] = {0};

void drawCube(float x, float y, float z);

void OnMouseDown(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN)
	{
		oldX = x;
		oldY = y;
		int window_y = (height - y);
		float norm_y = float(window_y)/float(height/2.0);
		int window_x = x ;
		float norm_x = float(window_x)/float(width/2.0);

		float winZ=0;
		glReadPixels( x, height-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
		if(winZ==1)
			winZ=0;
		double objX=0, objY=0, objZ=0;
		gluUnProject(window_x,window_y, winZ,  MV,  P, viewport, &objX, &objY, &objZ);
		glm::vec3 pt(objX,objY, objZ);
		size_t i=0;
	}

	if(button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;

	if(s==GLUT_UP) {
		selected_index= -1;
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

void OnMouseMove(int x, int y)
{
	if(selected_index == -1)
	{
		if (state == 0)
			dist *= (1 + (y - oldY)/60.0f);
		else
		{
			rY += (x - oldX)/5.0f;
			rX += (y - oldY)/5.0f;
		}
	}
	else
	{
		float delta = 1500/abs(dist);
		float valX = (x - oldX)/delta;
		float valY = (oldY - y)/delta;
		if(abs(valX)>abs(valY))
			glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
		else
			glutSetCursor(GLUT_CURSOR_UP_DOWN);
	}

	oldX = x;
	oldY = y;

	glutPostRedisplay();
}


void DrawMeshVertexRef()
{
	stringstream str3;
	str3 << "x";

	for (int a = 0; a < clothWorld.pointCount2; a++)
	{
		Vector3D vec(clothWorld.clothPoints2[a].x, clothWorld.clothPoints2[a].y, clothWorld.clothPoints2[a].z);

		glRasterPos3f(vec.x, vec.y, vec.z);
		glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)str3.str().c_str());
	}
}

//Draw loaded obj shape file
void DrawWavefrontGeoPoints()
{
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 1.0f);

	int triCount = 0;
	for (int a = 0; a < renderWireframeMesh.vertexCount; a++)
	{
		Vector3D vert(renderWireframeMesh.vertices[a].x, renderWireframeMesh.vertices[a].y, renderWireframeMesh.vertices[a].z);
		glVertex3f(vert.x, vert.y, vert.z);
	}

	glEnd();
}

//Draw loaded obj shape file
void DrawWavefrontGeo()
{
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 0.0f);


	int triCount = 0;
	for (int a = 0; a < renderWireframeMesh.indicesCount; a += 3)
	{
		int i1 = renderWireframeMesh.indices[a];
		int i2 = renderWireframeMesh.indices[a + 1];
		int i3 = renderWireframeMesh.indices[a + 2];

		Vector3D vert1(renderWireframeMesh.vertices[i1].x, renderWireframeMesh.vertices[i1].y, renderWireframeMesh.vertices[i1].z);
		Vector3D vert2(renderWireframeMesh.vertices[i2].x, renderWireframeMesh.vertices[i2].y, renderWireframeMesh.vertices[i2].z);
		Vector3D vert3(renderWireframeMesh.vertices[i3].x, renderWireframeMesh.vertices[i3].y, renderWireframeMesh.vertices[i3].z);

		glVertex3f(vert1.x, vert1.y, vert1.z);
		glVertex3f(vert2.x, vert2.y, vert2.z);
		glVertex3f(vert3.x, vert3.y, vert3.z);

		triCount++;
	}

	glEnd();
}


//////////////
//For drawing a test cube
//////////////

GLfloat color[8][3] =
{
	{0.0,0.0,0.0},
	{1.0,0.0,0.0},
	{1.0,1.0,0.0},
	{0.0,1.0,0.0},
	{0.0,0.0,1.0},
	{1.0,0.0,1.0},
	{1.0,1.0,1.0},
	{0.0,1.0,1.0},
};

void quad(int a, int b, int c, int d, float x, float y, float z)
{
	float scale = 0.05;
	float ver[8][3] =
	{
		{-1.0 * scale,-1.0 * scale,1.0 * scale},
		{-1.0 * scale,1.0 * scale,1.0 * scale},
		{1.0 * scale,1.0 * scale,1.0 * scale},
		{1.0 * scale,-1.0 * scale,1.0 * scale},
		{-1.0 * scale,-1.0 * scale,-1.0 * scale},
		{-1.0 * scale,1.0 * scale,-1.0 * scale},
		{1.0 * scale,1.0 * scale,-1.0 * scale},
		{1.0 * scale,-1.0 * scale,-1.0 * scale},
	};

	//-------------------
	ver[0][0] += x;
	ver[0][1] += y;
	ver[0][2] += z;

	ver[1][0] += x;
	ver[1][1] += y;
	ver[1][2] += z;

	ver[2][0] += x;
	ver[2][1] += y;
	ver[2][2] += z;

	ver[3][0] += x;
	ver[3][1] += y;
	ver[3][2] += z;

	ver[4][0] += x;
	ver[4][1] += y;
	ver[4][2] += z;

	ver[5][0] += x;
	ver[5][1] += y;
	ver[5][2] += z;

	ver[6][0] += x;
	ver[6][1] += y;
	ver[6][2] += z;

	ver[7][0] += x;
	ver[7][1] += y;
	ver[7][2] += z;

	//-------------------

	glBegin(GL_QUADS);
	//glColor3fv(color[a]);
	glVertex3fv(ver[a]);

	//glColor3fv(color[b]);
	glVertex3fv(ver[b]);

	//glColor3fv(color[c]);
	glVertex3fv(ver[c]);

	//glColor3fv(color[d]);
	glVertex3fv(ver[d]);
	glEnd();
}

void drawCube(float x, float y, float z)
{
	quad(0, 3, 2, 1, x, y, z);
	quad(2, 3, 7, 6, x, y, z);
	quad(0, 4, 7, 3, x, y, z);
	quad(1, 2, 6, 5, x, y, z);
	quad(4, 5, 6, 7, x, y, z);
	quad(0, 1, 5, 4, x, y, z);
}

void DrawTestVectors()
{
	return;

	//for (int a = 0; a < 3; a++)
	for (int a = 0; a < clothWorld.vectorsToDrawCount; a++)
	{
		glColor3f(1.0f, 0.0f, 0.0f);

		//Vector3D normal1(clothWorld.vectorsToDraw[a].x, clothWorld.vectorsToDraw[a].y, clothWorld.vectorsToDraw[a].z);
		//Vector3D normal1(1,1,1);

		glBegin(GL_LINES);
		//glVertex3f(0,0,0);
		//glVertex3f(clothWorld.vectorsToDraw[0].x, clothWorld.vectorsToDraw[0].y, clothWorld.vectorsToDraw[0].z);

		drawCube(clothWorld.vectorsToDraw[a].x, clothWorld.vectorsToDraw[a].y, clothWorld.vectorsToDraw[a].z);
		glEnd();
	}
}

void DrawClothInfo()
{
	return;

	glColor3f(1, 0, 0);
	for (int i = 0; i < clothWorld.vertexCountCloth; i++)
	{
		stringstream str3;
		str3 << i;
		glRasterPos3f(clothWorld.VertexPos[i].x, clothWorld.VertexPos[i].y, clothWorld.VertexPos[i].z);
		glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)str3.str().c_str());
	}
}


void DrawVecInfo()
{
}

void DrawCloth()
{
	//Draw points
	glBegin(GL_POINTS);

	for (int i = 0; i < clothWorld.vertexCountCloth; i++)
	{
		glm::vec3 p;
		p.x = clothWorld.VertexPos[i].x;
		p.y = clothWorld.VertexPos[i].y;
		p.z = clothWorld.VertexPos[i].z;

		//Color the cloth vertices:
		glColor3f(0, 0, 1);
		glVertex3f(p.x, p.y, p.z);
	}
	glEnd();

	//[Draw cloth polygons from cloth world object]
	glColor3f(1,1,1);
	
	glBegin(GL_TRIANGLES);
	glColor3f(0.67, 0.68, 0.70);
	
	int triToColor = 100;
	int curTri = 0;
	for (int i = 0; i < clothWorld.indices.size(); i += 3)
	{
		glm::vec3 p1;
		p1.x = clothWorld.VertexPos[clothWorld.indices[i]].x;
		p1.y = clothWorld.VertexPos[clothWorld.indices[i]].y;
		p1.z = clothWorld.VertexPos[clothWorld.indices[i]].z;

		glm::vec3 p2;
		p2.x = clothWorld.VertexPos[clothWorld.indices[i + 1]].x;
		p2.y = clothWorld.VertexPos[clothWorld.indices[i + 1]].y;
		p2.z = clothWorld.VertexPos[clothWorld.indices[i + 1]].z;

		glm::vec3 p3;
		p3.x = clothWorld.VertexPos[clothWorld.indices[i + 2]].x;
		p3.y = clothWorld.VertexPos[clothWorld.indices[i + 2]].y;
		p3.z = clothWorld.VertexPos[clothWorld.indices[i + 2]].z;

		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p3.x, p3.y, p3.z);
	}

	glEnd();

	DrawClothInfo();
	DrawVecInfo();
}

void DrawGrid()
{
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.5f);
	for(int i = -GRID_SIZE; i <= GRID_SIZE; i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);
		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}

void InitGL()
{
	startTime = (float)glutGet(GLUT_ELAPSED_TIME);
	currentTime = startTime;

	//get ticks per second
    QueryPerformanceFrequency(&frequency);

    //start timer
    QueryPerformanceCounter(&t1);

	glEnable(GL_DEPTH_TEST);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//GL_FILL or GL_LINE
	//glPolygonMode(GL_BACK, GL_LINE);
	glPointSize(5);

	wglSwapIntervalEXT(0);
}

void OnReshape(int nw, int nh)
{
	glViewport(0,0,nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 1.f, 100.0f);
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, P);
	glMatrixMode(GL_MODELVIEW);
}

void OnRender()
{
	size_t i = 0;
	float newTime = (float)glutGet(GLUT_ELAPSED_TIME);

	clothWorld.elapsedTime = (newTime - currentTime);

	frameTime = newTime - currentTime;
	currentTime = newTime;

	QueryPerformanceCounter(&t2);
	//compute and print the elapsed time in millisec
	frameTimeQP = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	t1 = t2;
	accumulator += 0.01;

	
	++totalFrames;
	if ((newTime - startTime) > 1000)
	{
		float elapsedTime = (newTime - startTime);
		fps = (totalFrames / elapsedTime) * 1000;
		startTime = newTime;
		totalFrames = 0;
	}

	//sprintf_s(info, "FPS: %3.2f, Frame time (GLUT): %3.4f msecs, Frame time (QP): %3.3f", fps, frameTime, frameTimeQP);
	sprintf_s(info, "Cloth Simulator 1.0.0 | Frame rate: %3.2f fps", fps);
	glutSetWindowTitle(info);

	glClearColor(0,
		0,
		0.5,
		1);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0, 0, dist);
	glRotatef(rX, 1, 0, 0);
	glRotatef(rY, 0, 1, 0);

	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	viewDir.x = (float)-MV[2];
	viewDir.y = (float)-MV[6];
	viewDir.z = (float)-MV[10];
	Right = glm::cross(viewDir, Up);

	//[Draw world grid]
	DrawGrid();

	//[Draw wavefront geometry]
	//DrawWavefrontGeoPoints();
	DrawWavefrontGeo();
	DrawTestVectors();

	DrawCloth();

	glutSwapBuffers();
}


vector <Sphere> spheres;
void CreateSpheres()
{
	return;
}

bool spheresCreated = false;
void DrawSpheres()
{
	return;
}

void StepPhysics()
{
	clothWorld.stepClothWorld();
}


void OnIdle()
{
	StepPhysics();

	glutPostRedisplay();
}

static void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBegin(GL_QUADS);
	glColor3d(1, 0, 0);
	glVertex3f(-1, -1, -10);
	glColor3d(1, 1, 0);
	glVertex3f(1, -1, -10);
	glColor3d(1, 1, 1);
	glVertex3f(1, 1, -10);
	glColor3d(0, 1, 1);
	glVertex3f(-1, 1, -10);
	glEnd();
	glutSwapBuffers();
}

/////////////////////////////////////
const float COLLISION_DISTANCE = 0.03;
//const float MAX_FPS = 60;//ToDo
/////////////////////////////////////

void main(int argc, char** argv)
{
	renderWireframeMesh.loadObj("cube.obj");//(1/2)

	clothWorld.initClothWorld();
	clothWorld.loadCollisionMesh("cube.obj");//(2/2)

	wavefrontUtils.expandMesh(&clothWorld.collisionMesh, COLLISION_DISTANCE);//Expand the collision mesh outwards by desired collision distance

	clothWorld.voxelizeCollisionMesh();
	clothWorld.computeCentroid();

	clothWorld.computeCMeshTriangleBounds();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("Cloth Simulator");

	glutDisplayFunc(OnRender);
	glutReshapeFunc(OnReshape);

	glutIdleFunc(OnIdle);

	glutMouseFunc(OnMouseDown);
	glutMotionFunc(OnMouseMove);
	//glutCloseFunc(OnShutdown);

	glewInit();
	InitGL();//<-- Wireframe mode set et al

	glutMainLoop();
}