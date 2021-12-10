#include <stdlib.h>
#include <algorithm>
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#include <CL/cl.h>
#include <fstream>
#include "environment_utilities.h"
#include <thread>
#include <mutex>
#include <atomic>

#define ROW 720
#define COL 1280
#define N 16

using namespace std;
int main() {
	Environment env(ROW, COL);
	env.add_start(20, 20);
	env.add_goal(800, 450);
	env.add_circle_obstacle(700, 300, 50);
	env.add_circle_obstacle(900, 600, 50);
	env.add_circle_obstacle(600, 400, 30);
	env.add_rectangle_obstacle(450, 10, 500, 300);
	env.add_rectangle_obstacle(200, 400, 250, 650);
	env.add_rectangle_obstacle(450, 10, 750, 70);
	double pi = acos(-1);
	for (int i = 0; i < 30; i++){
		for (int j = 0; j < 80; j++) {
			env.add_disturbance(100 + j * cosf(2 * pi * i / 30), 150 + j * sinf(2 * pi * i / 30), make_pair(-j/2 * sinf(2 * pi * i / 30), j / 2 * cosf(2 * pi * i / 30)));
		}
	}

	vector<Pointrobot> robots;
	for (int i = 0; i < N; i++) {
		robots.push_back(Pointrobot(env, i, DC, DS));
	}

	
	vector<thread> threadPool;
	for (int i = 0; i < N; i++) {
		threadPool.push_back(thread([&env, &robots, i]() {
			// cout << i << endl;
			while (true) {
				robots[i].move(env, robots);
			}
		}));
	}
	threadPool.push_back(thread([&env, &robots]() {
		// cout << i << endl;
		while (true) {
			showMap(env, robots);
		}
	}));
	for (auto& th : threadPool) th.join();
	/*
	for (int i = 0; i < N; i++) {
		robots[i].move(env, robots);
	}
	*/

	return 0;
}