#pragma once

#include <time.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

class FramerateTracker
{
public:
	FramerateTracker();
	virtual ~FramerateTracker();

	// This is to be called once per tick of the main program loop.
	void Track(FILE* fp);

	double statPrintsPerSecond;
	int maxTrackedFrames;

private:

	struct Frame
	{
		double timeTakenSeconds;
		int frameNumber;
	};

	clock_t lastTime;
	std::vector<Frame> frameFifo;
	int frameCount;
	double timeToNextStatPrintSeconds;
};