/**
* This file is part of DSO, written by Jakob Engel.
* It has been modified by Lukas von Stumberg for the inclusion in DM-VIO (http://vision.in.tum.de/dm-vio).
*
* Copyright 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/



#pragma once
#include "util/settings.h"
#include <thread>
#include <stdio.h>
#include <iostream>
#include <condition_variable>
#include <functional>


namespace dso
{
using namespace std::placeholders;
template<typename Running>
class IndexThreadReduce
{

public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	inline IndexThreadReduce()
	{
		nextIndex = 0;
		maxIndex = 0;
		stepSize = 1;
		callPerIndex = std::bind(&IndexThreadReduce::callPerIndexDefault, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);

		running = true;
		for(int i=0;i<NUM_THREADS;i++)
		{
			isDone[i] = false;
			gotOne[i] = true;
			workerThreads[i] = std::thread(&IndexThreadReduce::workerLoop, this, i);
		}

	}
	inline ~IndexThreadReduce()
	{
		running = false;

		exMutex.lock();
		todo_signal.notify_all();
		exMutex.unlock();

		for(int i=0;i<NUM_THREADS;i++)
			workerThreads[i].join();


		printf("destroyed ThreadReduce\n");

	}

	inline void reduce(std::function<void(int,int,Running*,int)> callPerIndex, int first, int end, int stepSize = 0)
	{

		memset(&stats, 0, sizeof(Running));

//		if(!multiThreading)
//		{
//			callPerIndex(first, end, &stats, 0);
//			return;
//		}



		if(stepSize == 0)
			stepSize = ((end-first)+NUM_THREADS-1)/NUM_THREADS;


		//printf("reduce called\n");

		std::unique_lock<std::mutex> lock(exMutex);

		// save
		this->callPerIndex = callPerIndex;
		nextIndex = first;
		maxIndex = end;
		this->stepSize = stepSize;

		// go worker threads!
		for(int i=0;i<NUM_THREADS;i++)
		{
			isDone[i] = false;
			gotOne[i] = false;
		}

		// let them start!
		todo_signal.notify_all();


		//printf("reduce waiting for threads to finish\n");
		// wait for all worker threads to signal they are done.
		while(true)
		{
			// wait for at least one to finish
			done_signal.wait(lock);
			//printf("thread finished!\n");

			// check if actually all are finished.
			bool allDone = true;
			for(int i=0;i<NUM_THREADS;i++)
				allDone = allDone && isDone[i];

			// all are finished! exit.
			if(allDone)
				break;
		}

		nextIndex = 0;
		maxIndex = 0;
		this->callPerIndex = std::bind(&IndexThreadReduce::callPerIndexDefault, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);

		//printf("reduce done (all threads finished)\n");
	}

	Running stats;

private:
	std::thread workerThreads[NUM_THREADS];
	bool isDone[NUM_THREADS];
	bool gotOne[NUM_THREADS];

	std::mutex exMutex;
	std::condition_variable todo_signal;
	std::condition_variable done_signal;

	int nextIndex;
	int maxIndex;
	int stepSize;

	bool running;

	std::function<void(int,int,Running*,int)> callPerIndex;

	void callPerIndexDefault(int i, int j,Running* k, int tid)
	{
		printf("ERROR: should never be called....\n");
		assert(false);
	}

	void workerLoop(int idx)
	{
		std::unique_lock<std::mutex> lock(exMutex);

		while(running)
		{
			// try to get something to do.
			int todo = 0;
			bool gotSomething = false;
			if(nextIndex < maxIndex)
			{
				// got something!
				todo = nextIndex;
				nextIndex+=stepSize;
				gotSomething = true;
			}

			// if got something: do it (unlock in the meantime)
			if(gotSomething)
			{
				lock.unlock();

				assert(callPerIndex != 0);

				Running s; memset(&s, 0, sizeof(Running));
				callPerIndex(todo, std::min(todo+stepSize, maxIndex), &s, idx);
				gotOne[idx] = true;
				lock.lock();
				stats += s;
			}

			// otherwise wait on signal, releasing lock in the meantime.
			else
			{
				if(!gotOne[idx])
				{
					lock.unlock();
					assert(callPerIndex != 0);
					Running s; memset(&s, 0, sizeof(Running));
					callPerIndex(0, 0, &s, idx);
					gotOne[idx] = true;
					lock.lock();
					stats += s;
				}
				isDone[idx] = true;
				//printf("worker %d waiting..\n", idx);
				done_signal.notify_all();
				todo_signal.wait(lock);
			}
		}
	}
};
}
