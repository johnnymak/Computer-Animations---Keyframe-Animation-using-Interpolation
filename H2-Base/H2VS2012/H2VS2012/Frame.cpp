
#include "Frame.h"

// DEFAULT CONSTRUCTOR
Frame::Frame() {
}

void Frame::addFrame(Quaternion quat) {
	frame.push_back(quat);
}