#include "environment_utilities.h"
using namespace std;
using namespace cv;

// a number between 0 and 1, describing how much the motion of a robot 
// is affected by its neighbors against being affected by the environment
#define LAMBDA 0.6

void Environment::add_goal(int x, int y) {
	goals.push_back(make_pair(x, y));
	// construct potential field
	for (auto i = 0; i < rows; i++) {
		for (auto j = 0; j < cols; j++) {
			float dist_sqr = pow(y - i, 2) + pow(x - j, 2);
			float gauss = 10*exp(-dist_sqr*0.02/(rows+cols));
			this->map[i * this->cols + j] += gauss;
		}
	}
}

void Environment::add_start(int x, int y) {
	starts.push_back(make_pair(x, y));
}

void Environment::add_rectangle_obstacle(int ulx, int uly, int lrx, int lry) {
	if (ulx < 0) {
		ulx = 0;
	}
	if (uly < 0) {
		uly = 0;
	}
	if (lrx >= this->cols) {
		lrx = this->cols - 1;
	}
	if (lry >= this->rows) {
		lry = this->rows - 1;
	}
	// assert(ulx >= 0 && uly >= 0 && lrx < this->cols && lry < this->rows);
	for (int i = uly; i < lry + 1; i++) {
		for (int j = ulx; j < lrx + 1; j++) {
			this->map[i * this->cols + j] = OBSTACLE_VAL;
		}
	}
}

void Environment::add_rectangle_obstacle(pair<int, int> ul, pair<int, int> lr) {
	int ulx = ul.first;
	int uly = ul.second;
	int lrx = lr.first;
	int lry = lr.second;
	add_rectangle_obstacle(ulx, uly, lrx, lry);
}

void Environment::add_circle_obstacle(int cx, int cy, int radius) {
	int ulx = cx - radius;
	int uly = cy - radius;
	int lrx = cx + radius;
	int lry = cy + radius;
	int radius_sqr = pow(radius, 2);
	for (int i = uly; i < lry + 1; i++) {
		for (int j = ulx; j < lrx + 1; j++) {
			if (j >= 0 && i >= 0 && j < this->cols && i < this->rows) {
				if (pow(i - cy, 2) + pow(j - cx, 2) <= radius_sqr) {
					this->map[i * this->cols + j] = OBSTACLE_VAL;
				}
			}
		}
	}
}

void Environment::add_circle_obstacle(pair<int, int> center, int radius) {
	int cx = center.first;
	int cy = center.second;
	add_circle_obstacle(cx, cy, radius);
}

void Environment::add_obstacle(int x, int y) {
	if (x < 0) {
		x = 0;
	}
	if (y < 0) {
		y = 0;
	}
	if (x >= this->cols) {
		x = this->cols - 1;
	}
	if (y >= this->rows) {
		y = this->rows - 1;
	}
	this->map[y * this->cols + x] = OBSTACLE_VAL;
}
void Environment::add_obstacle(pair<int, int> pos) {
	int x = pos.first;
	int y = pos.second;
	add_obstacle(x, y);
}

void Environment::add_disturbance(int x, int y, pair<int, int> direction) {
	if (x < 0) {
		x = 0;
	}
	if (y < 0) {
		y = 0;
	}
	if (x >= this->cols) {
		x = this->cols - 1;
	}
	if (y >= this->rows) {
		y = this->rows - 1;
	}
	this->disturbance[y * this->cols + x] = make_pair(direction.first, direction.second);
}
void Environment::add_disturbance(pair<int, int> pos, pair<int, int> direction) {
	int x = pos.first;
	int y = pos.second;
	add_disturbance(x, y, direction);
}

Environment::~Environment() {}
Pointrobot::~Pointrobot() {}

Environment::Environment(int _rows, int _cols) {
	this->rows = _rows;
	this->cols = _cols;
	this->map.resize(_rows * _cols, 0.1);
}

int Environment::get_cols() const {
	return this->cols;
}

int Environment::get_rows() const {
	return this->rows;
}

float* Environment::get_map() {
	return this->map.data();
}

vector<pair<int, int> > Environment::get_goals() const {
	return this->goals;
}

vector<pair<int, int> > Environment::get_starts() const {
	return this->starts;
}

unordered_map<int, std::pair<int, int> >* Environment::get_disturbance() {
	return &disturbance;
}

Pointrobot::Pointrobot(Environment& environment, int _ID, int dc, int ds) {
	this->d_communication = dc;
	this->d_sensor = ds;
	this->ID = _ID;
	this->own_map.resize(environment.rows * environment.cols, -1);
	int num_starts = environment.starts.size();
	std::uniform_int_distribution<int> start_uni(0, num_starts-1);
	std::uniform_int_distribution<int> dev_uni(-environment.cols * environment.rows / 100000, environment.cols * environment.rows / 100000);
	auto seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine random_eng(seed);
	std::pair<int, int> selected_start = environment.starts[start_uni(random_eng)];
	this->x = selected_start.first + dev_uni(random_eng);
	this->y = selected_start.second + dev_uni(random_eng);
	this->vx = 0.0;
	this->vy = 0.0;
}

pair<float, float> Pointrobot::get_position() const {
	return make_pair(this->x, this->y);
}

pair<float, float> Pointrobot::get_velocity() const {
	return make_pair(this->vx, this->vy);
}

void Pointrobot::move(Environment& environment, vector<Pointrobot>& robots) {
	communicate(environment.cols, environment.rows, robots);
	sense(environment);
	pair<float, float> desired_pos = voronoi_partition(environment.cols, environment.rows, robots);
	// cout << "Robot " << this->ID << " at " << this->x << ", " << this->y << " going to " << desired_pos.first << ", " << desired_pos.second << endl;
	float xd = desired_pos.first;
	float yd = desired_pos.second;

	double ratio = sqrt(pow(xd - this->x, 2) + pow(yd - this->y, 2)) / MAX_STEP;
	// cout << ratio << endl;
	if (ratio > 1) {
		xd = this->x + (xd - this->x)*1.0 / ratio;
		yd = this->y + (yd - this->y)*1.0 / ratio;
	}

	double x_step_size = (xd - this->x)*1.0 / NUM_STEPS;
	double y_step_size = (yd - this->y)*1.0 / NUM_STEPS;

	if (!(this->neighbors.empty())) {
		pair<float, float> neighbor_mean_vel = velocity_match(robots);
		this->vx = (1 - LAMBDA) * x_step_size + LAMBDA * neighbor_mean_vel.first;
		this->vy = (1 - LAMBDA) * y_step_size + LAMBDA * neighbor_mean_vel.second;
	} else {
		this->vx = x_step_size;
		this->vy = y_step_size;
	}

	// move in NUM_STEPS steps
	for (int i = 0; i < NUM_STEPS; i++) {
		this->x += this->vx;
		this->y += this->vy;

		int index = floor(this->y) * environment.cols + floor(this->x);
		unordered_map<int, pair<int, int> >::iterator own_iter = this->own_disturbance.find(index);
		unordered_map<int, pair<int, int> >::iterator iter = environment.disturbance.find(index);
		if (own_iter == this->own_disturbance.end() && iter != environment.disturbance.end()) {
			pair<int, int> direction = iter->second;
			this->x += DISTURB_FACTOR * direction.first;
			this->y += DISTURB_FACTOR * direction.second;
			own_disturbance[index] = direction;
		}
	}

	this_thread::sleep_for(chrono::nanoseconds(1));
}

void Pointrobot::communicate(int cols, int rows, vector<Pointrobot>& robots){
	this->neighbors.clear();
	for (int i = 0; i < robots.size(); i++) {
		pair<int, int> pos = robots[i].get_position();
		int dist = sqrt(pow(pos.first - this->x, 2) + pow(pos.second - this->y, 2));
		// check within the communication range
		if (robots[i].ID != this->ID && dist <= this->d_communication) {
			this->neighbors.push_back(i);
			// update_map(cols, rows, robots[i].own_map, robots[i].own_disturbance);
		}
	}
}

void Pointrobot::sense(Environment& environment) {
	int ulx = this->x - d_sensor;
	int uly = this->y - d_sensor;
	int lrx = this->x + d_sensor;
	int lry = this->y + d_sensor;
	int radius_sqr = pow(d_sensor, 2);
	for (int i = uly; i < lry + 1; i++) {
		for (int j = ulx; j < lrx + 1; j++) {
			// shrinking down the map range in a square
			if (j >= 0 && i >= 0 && j < environment.cols && i < environment.rows) {
				int index = i * environment.cols + j;
				// only need to update the map where the robot does not recall.
				if (this->own_map[index] == -1 && pow(i - this->y, 2) + pow(j - this->x, 2) <= radius_sqr) {
					// update own_map if a pixel is in range
					this->own_map[index] = environment.map[index];
				}
			}
		}
	}
}

std::pair<float, float> Pointrobot::voronoi_partition(int cols, int rows, vector<Pointrobot>& robots) const {
	double x_val = 0.0;
	double y_val = 0.0;
	// count the number of pixels to check
	// int count = 0;
	double M = 0.0;
	int ulx = this->x - d_sensor;
	int uly = this->y - d_sensor;
	int lrx = this->x + d_sensor;
	int lry = this->y + d_sensor;
	float radius_sqr = pow(d_sensor, 2);

	for (int i = uly; i < lry + 1; i++) {
		for (int j = ulx; j < lrx + 1; j++) {
			if (j >= 0 && i >= 0 && j < cols && i < rows) {
				// the distance^2 of the selected pixel to the robot
				float dist_sqr = pow(i - this->y, 2) + pow(j - this->x, 2);
				// cout << dist_sqr << '\t' << radius_sqr << endl;
				if (dist_sqr <= radius_sqr) {
					// if within sensing range, check whether the pixel is closer to the current robot
					bool is_in_cell = true;
					for (int k = 0; k < neighbors.size(); k++) {
						pair<int, int> neighbor_pos = robots[this->neighbors[k]].get_position();
						if (pow(i - neighbor_pos.second, 2) + pow(j - neighbor_pos.first, 2) <= dist_sqr) {
							is_in_cell = false;
							break;
						}
					}

					if (is_in_cell) {
						int index = i * cols + j;
						M += this->own_map[index];
						// integrate along each axis
						x_val += this->own_map[index]*j;
						y_val += this->own_map[index]*i;
						// cout << x_val << '\t' << y_val << endl;
					}
				}
			}
		}
	}
	// cout << x_val << '\t' << y_val << '\t' << M << endl;
	// normalize
	x_val /= M;
	y_val /= M;
	return make_pair(x_val, y_val);
}

/* Due to communication overhead, we can only update a random portion of the input map */
void Pointrobot::update_map(int cols, int rows, vector<float> &map, unordered_map<int, pair<int, int> > &disturbance) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int index = i * cols + j;
			if (this->own_map[index] == -1 && this->own_map[index] != map[index]) {
				this->own_map[index] = map[index];
			}
			if (disturbance.find(index) != disturbance.end() && this->own_disturbance.find(index) == this->own_disturbance.end()) {
				this->own_disturbance[index] = disturbance.find(index)->second;
			}
		}
	}
}

pair<float, float> Pointrobot::velocity_match(vector<Pointrobot>& robots) {
	double vel_x = vx*0.5;
	double vel_y = vy*0.5;
	for (int k = 0; k < this->neighbors.size(); k++) {
		pair<float, float> neighbor_vel = robots[this->neighbors[k]].get_velocity();
		vel_x += neighbor_vel.first;
		vel_y += neighbor_vel.second;
	}
	return make_pair(vel_x / this->neighbors.size(), vel_y / this->neighbors.size());
}

void showMap(Environment& environment, std::vector<Pointrobot>& robots) {
	Mat image_raw = Mat(environment.get_rows(), environment.get_cols(), CV_32FC1, environment.get_map());
	Mat image;
	image_raw.convertTo(image, -1, 0.1); //
	for (unordered_map<int, pair<int, int> >::iterator iter = environment.get_disturbance()->begin(); iter != environment.get_disturbance()->end(); iter++) {
		int index = iter->first;
		int x = index % environment.get_cols();
		int y = index / environment.get_cols();
		pair<int, int> direction = iter->second;
		arrowedLine(image, Point(x, y), Point(x + (iter->second).first, y + (iter->second).second), 0.35, 1, 8, 0, 0.2);
	}

	for (int i = 0; i < robots.size(); i++) {
		pair<int, int> position = robots[i].get_position();
		circle(image, Point(position.first, position.second), 3, 1, -1);
		circle(image, Point(position.first, position.second), 4, 0, 1);
	}

	auto goals = environment.get_goals();
	for (int i = 0; i < goals.size(); i++) {
		circle(image, Point(goals[i].first, goals[i].second), 8, 0, 2);
		// circle(image, Point(goals[i].first, goals[i].second), 7, 1, 0);
	}

	imshow("environment", image);
	waitKey(1);
}