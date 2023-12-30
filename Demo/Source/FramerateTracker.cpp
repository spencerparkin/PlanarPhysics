#include "FramerateTracker.h"

FramerateTracker::FramerateTracker()
{
	this->lastTime = 0;
	this->maxTrackedFrames = 100;
	this->statPrintsPerSecond = 1.0;
	this->timeToNextStatPrintSeconds = this->statPrintsPerSecond;
	this->frameCount = 0;
}

/*virtual*/ FramerateTracker::~FramerateTracker()
{
}

void FramerateTracker::Track(FILE* fp)
{
	clock_t currentTime = ::clock();

	if (this->lastTime != 0)
	{
		clock_t elapsedTime = currentTime - lastTime;
		double elapsedTimeSeconds = double(elapsedTime) / double(CLOCKS_PER_SEC);
		this->frameFifo.push_back(Frame{ elapsedTimeSeconds, this->frameCount });
		while ((signed)this->frameFifo.size() > this->maxTrackedFrames)
			this->frameFifo.erase(this->frameFifo.begin());

		this->timeToNextStatPrintSeconds -= elapsedTimeSeconds;

		if (this->timeToNextStatPrintSeconds <= 0.0)
		{
			this->timeToNextStatPrintSeconds += this->statPrintsPerSecond;

			double slowestFrametimeSeconds = std::numeric_limits<double>::min();
			double fastestFrametimeSeconds = std::numeric_limits<double>::max();
			double totalFrametimeSeconds = 0.0;

			for (const Frame& frame : this->frameFifo)
			{
				if (frame.timeTakenSeconds > slowestFrametimeSeconds)
					slowestFrametimeSeconds = frame.timeTakenSeconds;

				if (frame.timeTakenSeconds < fastestFrametimeSeconds)
					fastestFrametimeSeconds = frame.timeTakenSeconds;

				totalFrametimeSeconds += frame.timeTakenSeconds;
			}

			double averageFrametimeSeconds = totalFrametimeSeconds / double(this->frameFifo.size());
			double averageFramerate = 1.0 / averageFrametimeSeconds;
			double worstFramerate = 1.0 / slowestFrametimeSeconds;
			double bestFramerate = 1.0 / fastestFrametimeSeconds;

			fprintf(fp, "================================================\n");
			fprintf(fp, "For frames %d through %d...\n", this->frameFifo[0].frameNumber, this->frameFifo[this->frameFifo.size() - 1].frameNumber);
			fprintf(fp, "Average FPS: %07.3f\n", averageFramerate);
			fprintf(fp, "  Worst FPS: %07.3f\n", worstFramerate);
			fprintf(fp, "   Best FPS: %07.3f\n", bestFramerate);
		}
	}

	this->lastTime = currentTime;
	this->frameCount++;
}