#pragma once

#include "../../simpleMath.h"

class Quaternion {

	public: 

		float angle;
		float x, y, z;

		Quaternion();
		Quaternion(float angle, float x, float y, float z);

		static Quaternion convertFromMat4(float* mat);

		static void convertToMat4(Quaternion quat, float* output);

		static Quaternion quaternionLERP(Quaternion start, Quaternion end, float timeStep);

		static Quaternion multiply(Quaternion first, Quaternion second);
};