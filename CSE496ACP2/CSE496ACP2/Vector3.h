#include <math.h>

class Vector3
{
public:
	Vector3(void);
	Vector3(float X, float Y, float Z);
	~Vector3(void);
	void setX(float newX);
	void setY(float newY);
	void setZ(float newZ);
	void setXYZ(float newX, float newY, float newZ);
	float Length();
	Vector3 Normalize();
	float DistanceTo(Vector3* vector);
	float X, Y, Z;
};