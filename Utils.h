#pragma once
class Utils
{
public:
	float dist(float x1, float y1, float z1, float x2, float y2, float z2);
	float randFloat();
	float random_2(float min, float max);//Random float between two values
	float normalize2(float max, float value);
	float clamp(float val, float max, float min);
	string toLower(string in);
	string toUpper(string in);
};

