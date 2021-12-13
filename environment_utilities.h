#include <unordered_map>
#include <vector>
#include <utility>
#include <random>
#include <chrono>
#include <cassert>
#include <math.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#define DC 100
#define DS 30
#define MAX_STEP DS
#define OBSTACLE_VAL -0.01
#define NUM_STEPS 50
#define DISTURB_FACTOR 0.5

/* The map */
class Environment {
public:
	Environment (int rows, int cols);
	~Environment ();
	// add a goal/starting point on the map
	void add_goal(int x, int y);
	void add_start(int x, int y);

	// add a rectangular obstacle on the map
	void add_rectangle_obstacle(int ulx, int uly, int lrx, int lry);
	void add_rectangle_obstacle(std::pair<int, int> ul, std::pair<int, int> lr);

	// add a circular obstacle on the map
	void add_circle_obstacle(int cx, int cy, int radius);
	void add_circle_obstacle(std::pair<int, int> center, int radius);

	// add an arbitrary obstacle (pixel) on the map
	void add_obstacle(int x, int y);
	void add_obstacle(std::pair<int, int> pos);

	// add an arbitrary turbulance (pixel) on the map
	void add_disturbance(int x, int y, std::pair<int, int> direction);
	void add_disturbance(std::pair<int, int> pos, std::pair<int, int> direction);

	int get_cols() const;
	int get_rows() const;
	std::vector<std::pair<int, int> > get_goals() const;
	std::vector<std::pair<int, int> > get_starts() const;
	float* get_map();
	std::unordered_map<int, std::pair<int, int> >* get_disturbance();

private:
	/* let's try multiple s-g pairs */
	std::vector<std::pair<int, int> > goals;
	std::vector<std::pair<int, int> > starts;
	std::vector<float> map;
	std::unordered_map<int, std::pair<int, int> > disturbance;
	int rows;
	int cols;
	/* the robots need to know the environment */
	friend class Pointrobot;
};

/* One point robot */
class Pointrobot {
	std::vector<float> own_map;
	std::unordered_map<int, std::pair<int, int> > own_disturbance;
	int d_sensor;
	int d_communication;
	/* position */
	float x, y;
	/* velocity */
	float vx, vy;
	int ID;
	/* IDs of its neighbors */
	std::vector<int> neighbors;

public:
	Pointrobot(Environment& environment, int _ID, int dc, int ds);
	~Pointrobot();
	// get the position of the point robot
	std::pair<float, float> get_position() const;
	std::pair<float, float> get_velocity() const;
	// move the point robot based on the environment
	void move(Environment& environment, std::vector<Pointrobot>& robots);
	// update the map with the information from another robot
	void update_map(int cols, int rows, std::vector<float> &map, std::unordered_map<int, std::pair<int, int> > &disturbance);

private:
	// try to find its neighbors
	void communicate(int cols, int rows, std::vector<Pointrobot>& robots);
	// sense the environment and record in local memory
	void sense(Environment& environment);
	// try to match the velocities of its neighbors
	std::pair<float, float> velocity_match(std::vector<Pointrobot>& robots);
	// voronoi partitioning within its sensor range
	std::pair<float, float> voronoi_partition(int cols, int rows, std::vector<Pointrobot>& robots) const;
};


void showMap(Environment& environment, std::vector<Pointrobot>& robots);