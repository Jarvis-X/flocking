# flocking
Flocking simulation that combines decentralized control, local voronoi partitioning, and velocity match that (seems to) obey Reynolds Flocking Rules
## To compile and run
Obviously, this project was developped in VS 2019. Thus, if intending to work with the .sln solution, one needs to configure OpenCV and OpenCL. 
More specifically, tell the MSVC comipler where to find the header files and the libraries. 
* For OpenCV, I followed https://towardsdatascience.com/install-and-configure-opencv-4-2-0-in-windows-10-vc-d132c52063a1
* For OpenCL, I followed https://medium.com/@pratikone/opencl-on-visual-studio-configuration-tutorial-for-the-confused-3ec1c2b5f0ca
  * OpenCL is not used yet since pthread and coarse graining already gives me satisfying performance. 
  Therefore, clearing out configurations about OpenCL does not affect the compilation for now.

Moreover, since the current version depends only on external libraries OpenCV and pthread, one can compile the project 
with any compiler that supports C++11 or higher using the header file and the cpp files on a machine where OpenCV and pthread are installed.

## Technical details
* The environment composed of goals, obstacles and disturbances. The robot swarm tries to approach the goal, avoid the obstacles, and offset the disturbances.
  * A goal is integrated into the environment as a positive gaussian function of distance to the goal point. The robots will approach regions of higher values.
  * An obstacle is integrated into the environment as a solid block with a negative value.
  * A disturbance region is integrated into the environment as a region such that changes the motion of a robot motion upon entry until exiting.
    * A robot will record the difference between its desired position and actual position and will try to compensate if disturbed again 
    (For now, the disturbance is completely compensated if recorded).
* The algorithm is considered decentralized because each robot instance senses the environment, does the local Voronoi partition, and moves in an independent thread.
  * About local Voronoi partition: each robot has a range of sensing, which we assume to be smaller than its communication range. One robot does Voronoi partition 
  inside its sensing range against its neighbors.
    * A robot considers all other robots that enter its communication range as its neighbors.
    * Even a neighbor is outside the sensing range of one robot, the robot will still consider the influence of the neighbor.
* Aside from Voronoi partition, each robot also tries to match the average velocity of its neighbors.
* All robots have the same maximum moving distance during one iteration.
* For future extension, the sensing will include a Kalman filter, while the compesation of the disturbance will include a gaussian process.

## Results
* Here are some videos of the simulation under different settings of the environment
 * one goal, one starting point: https://drive.google.com/file/d/12FpDzVW8JGW73ktIKMlRONQSl5SsHrQU/view?usp=sharing
 * two goals, two starting points: https://drive.google.com/file/d/1MK17Y-IviVe09gM3KORz7HBoGwPIZ6Dk/view?usp=sharing
 * three goals, two starting point: https://drive.google.com/file/d/1ooN8JlKTCuZxVNkiWjXN25zzWqEWjc30/view?usp=sharing
