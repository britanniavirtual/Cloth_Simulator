#include "JointHeader.h"

float Utils::dist(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return(sqrt(pow((float)(x2 - x1), (float)2) + pow((float)(y2 - y1), (float)2) + pow((float)(z2 - z1), (float)2)));
}

//Random float between two values
float Utils::random_2(float min, float max)
{
	float f = max - min;
	return min + (float(rand()) / float((RAND_MAX)) * f);
}

float Utils::randFloat()
{
	float X = 1.0;
	float r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
	return r2;
}

float Utils::normalize2(float max, float value)
{
	if (value > max)
	{
		return 1.0;
	}

	if (value > 0)
	{
		return 1.0 / (max / value);
	}

	return 0;
}


float Utils::clamp(float val, float max, float min)
{
	if (val < min) { return min; }
	if (val > max) { return max; }
	return val;
}


string Utils::toLower(string in)
{
	string ret = "";

	for (int a = 0; a < in.length(); a++)
	{
		char c = in[a];
		ret += tolower(c);
	}

	return ret;
}

string Utils::toUpper(string in)
{
	string ret = "";

	for (int a = 0; a < in.length(); a++)
	{
		char c = in[a];
		ret += toupper(c);
	}

	return ret;
}