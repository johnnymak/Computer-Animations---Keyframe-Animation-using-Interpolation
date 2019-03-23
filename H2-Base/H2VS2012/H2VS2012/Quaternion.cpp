
#include "Quaternion.h"
#include <cmath>


// CONSTRUCTORS 
Quaternion::Quaternion(): angle(1), x(0), y(0), z(0) {
};


Quaternion::Quaternion(float angle, float x, float y, float z) : angle(angle), x(x), y(y), z(z) {
};


// CONVERT MATRIX TO QUATERNION
Quaternion Quaternion::convertFromMat4(float* mat) {

	Quaternion temp;

	temp.angle = std::sqrtf(mat[0] + mat[5] + mat[10] + mat[15]) / 2.0f;

	float a4 = temp.angle * 4.0f;

	temp.x = (mat[9] - mat[6]) / a4;
	temp.y = (mat[2] - mat[8]) / a4;
	temp.z = (mat[4] - mat[1]) / a4;

	return temp;
}

// CONVERT QUATERNION TO MATRIX
void Quaternion::convertToMat4(Quaternion quat, float* output) {

	output[0] = 1 - 2 * ((quat.y * quat.y) + (quat.z * quat.z));
	output[1] = 2 * ((quat.x * quat.y) - (quat.z * quat.angle));
	output[2] = 2 * ((quat.x * quat.z) + (quat.y * quat.angle));
	output[3] = 0;

	output[4] = 2 * ((quat.x * quat.y) + (quat.z * quat.angle));
	output[5] = 1 - 2 * ((quat.x * quat.x) + (quat.z * quat.z));
	output[6] = 2 * ((quat.y * quat.z) - (quat.x * quat.angle));
	output[7] = 0;

	output[8]  = 2 * ((quat.x * quat.z) - (quat.y * quat.angle));
	output[9]  = 2 * ((quat.y * quat.z) + (quat.x * quat.angle));
	output[10] = 1 - 2 * ((quat.x * quat.x) + (quat.y * quat.y));
	output[11] = 0;

	output[12] = 0;
	output[13] = 0;
	output[14] = 0;
	output[15] = 1;

}

// QUATERNION LINEAR INTERPOLATION
Quaternion Quaternion::quaternionLERP(Quaternion start, Quaternion end, float timeStep) {

	float timeOffset = 1 - timeStep;

	return Quaternion{((start.angle * timeOffset) + (end.angle * timeStep)),
					  ((start.x * timeOffset) + (end.x * timeStep)), 
					  ((start.y * timeOffset) + (end.y * timeStep)), 
					  ((start.z * timeOffset) + (end.z * timeStep)) };
}

// QUATERNION MULTIPLICATION
Quaternion Quaternion::multiply(Quaternion first, Quaternion second)  {

	return Quaternion{ (first.angle * second.angle - first.x * second.x - first.z * second.z - first.y * second.y),
					   (first.angle * second.x + first.x * second.angle - first.z * second.y + first.y * second.z),
					   (first.angle * second.y - first.x * second.z + first.y * second.angle + first.z * second.x),
					   (first.angle * second.z + first.x * second.y + first.z * second.angle - first.y * second.x) };
}

