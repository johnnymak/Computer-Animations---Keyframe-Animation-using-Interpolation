#pragma once

#include <vector>
#include "Quaternion.h"

class Frame {

	public:
		std::vector<Quaternion> frame;

		Frame();

		void addFrame(Quaternion quat);

};

