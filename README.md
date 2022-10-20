# Overview

This package contains a ROS action server and C++ code for the paper "[Uncertainty-Aware Manipulation Planning using Gravity and Environment Geometry](https://ieeexplore.ieee.org/document/9894673/)".

It can be used stand-alone or with the robot setup shown above (released [here](https://github.com/o2ac/o2ac-ur/)) to:

- Represent the pose uncertainty of an object
- Estimate the pose uncertainty after performing an action
- Plan an action sequence to reduce the uncertainty

# Package

This package implements an action server for the `updateDistribution` action.

When an object is grasped by a robot, but the object pose is uncertain, the robot can perform actions to reduce the uncertainty: "touch", "look", "place", "grasp" and "push".

- "touch" : Move the gripper until the grasped object touches the environment.
- "look" : Take an image of the object with a camera
- "place" : Place the object on a support surface
- "grasp" : Grasp the object with two flat fingers
- "push" : Push the object with the gripper

When the action server receives a pose estimate (a pose with an uncertainty) and an action, it calculates the updated pose estimate with uncertainty. 

We use two methods to represent the uncertainty around a pose.

- RPY representation: The object pose is represented as a 6-element vector (x, y, z, roll, pitch, yaw).
  The first three values (x, y, z) represent the translation.  
  The latter three values (roll, pitch, yaw) represent the rotation.  
  A pose with uncertainty is represented by two elements: a 6-element vector representing the mean of the pose distribution and a 6x6 covariance matrix of the pose distribution.

- Lie representation: We use the approach presented in http://ncfrn.cim.mcgill.ca/members/pubs/barfoot_tro14.pdf .  
  The object pose is regarded as an element of the 3-dimensional special Euclidean group $SE(3)$.  
  A pose with ambiguity is represented by a pair of the mean of the pose and a 6x6 covariance matrix, which represents a distribution in the 6-dimensional vector space identified with $se(3)$, the Lie algebra of corresponding to $SE(3)$.  


For the description of actions, see the "Message types" section.

Additionally, this package provides a service server for visualize pose beliefs.
When the server is called with a pose belief, it publishes a marker array visualizing the pose belief.

# Test Sample

Launch the action server and a test client:
```
roslaunch o2ac_pose_distribution_updater visualize_test.launch
```
The results of the place, grasp and push actions will be visualized when they are received.

# Usage

To run the action server for updateDistribution action and the visualization server, run the following command:

```
roslaunch o2ac_pose_distribution_updater distribution_updater.launch
```

Then a node named `pose_distribution_updater` is launched and provides the action server to update distributions named `update_distribution` and
the service server to visualize pose beliefs named `visualize_pose_belief`.

The service server publishes marker arrays to a topic (default: `o2ac_pose_belief_visualization_marker_array`).

The action to update the pose distribution is defined in `o2ac_msgs/updateDistributionAction.h`.

The service to visualize pose beliefs is defined in `o2ac_msgs/visualizePoseBelief.h`.

Client Sample Code:
```
#include "o2ac_msgs/updateDistributionAction.h"
#include "o2ac_msgs/visualizePoseBelief.h"
...
int main(...)
{
	...
	ros::NodeHandle nd;

	actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> update_client("update_distribution", true);
	update_client.waitForServer();
	ros::ServiceClient visualizer_client = nd.serviceClient<o2ac_msgs::visualizePoseBelief>("visualize_pose_belief");
	...
	moveit_msgs::CollisionObject object;
	geometry_msgs::PoseWithCovariance distribution;
	...
	whlle(...){
		...
		o2ac_msgs::updateDistributionGoal goal;
		goal.gripped_object = object;
		goal.distribution = distribution;
		...
		// take some observation
		...
		// update the distribution
		update_client.sendGoal(goal);
		update_client.waitForResult();		
		auto result = update_client.getResult();
		if(result->seccess){
			distribution = result->distribution;
		}
		...
		// visualize the pose belief
		o2ac_msgs::visualizePoseBelief pose_belief;
		pose_belief.object = object;
		pose_belief.distribution = distribution;
		visualizer_client.call(pose_belief);
		...
	}
	...
}
```

## Action server parameters
This section desribes the parameters for action server.

The parameters are defined in `launch/distribution_updater.launch`.

### For Gaussian particle filter

The updated pose distribution for the "touch" and "look" actions is calculated using a Gaussian particle filter.

- `number_of_particles` : The number of particles
- `noise_variance`: A 6-dimensional vector representing the variance of noise in each step

### Touch action

Two different objects may be used as the environment: "ground" or "box". They are parametrized by:

- `ground_size`: 3-dimensional vector representing the size of the ground object
- `ground_position`: 3-dimensional vector representing the position of the ground object
- `box_size`: 3-dimensional vector representing the size of the box object
- `box_position`: 3-dimensional vector representing the position of the box object
- `distance_threshold`: If the distance between two objects is less than this value, they are regarded as touching each other.

### Look action

- `look_threshold`: A thresholding value for a grayscale image. If the value of the image at a pixel is less than or equal to this value, the pixel is considered part of the object.
- `calibration_object_points`: A vector representing the 3 dimensional coordinates of the points which are used for camera calibration. 
This vector is the concatenation of the coordinates of the points, e.g., if the points are point (x0, y0, z0), point (x1, y1, z1) and point (x2, y2, z2), this vector is (x0, y0, z0, x1, y1, z1, x2, y2, z2).
- `calibration_image_points`: A vector representing the 2 dimensional coordinates of calibration points projected by the camera.
This vector is the concatenation of the coordinates of the points, e.g., if the points are point (x0, y0), point (x1, y1) and point (x2, y2), this vector is (x0, y0, x1, y1, x2, y2).

The four parameters below are the intrinsic parameters of the camera.

- `camera_fx` and  `camera_fy`: Focal length in terms of pixel
- `camera_cx` and `camera_cy`: The principal point

### Visualization

- `marker_array_topic_name`: The name of topic to which the marker arrays to visualize pose beliefs are published
- `visualization_scale`: The scale of the object
- `mean_color`: The color of the visualized mean pose of the object, represented by RGBA.
- `variance_color`: The color of the visualized pose distribution of the object representing uncertainty, represented by RGBA. An alpha of < 0.5 is recommended.
- `number_of_particles_to_visualize`: The number of particles to visualize the pose uncertainty. A reasonable default is 20-50.

# Message types

This section describes the message, action and service. The types are defined in the package `o2ac_msgs` ([here](https://github.com/o2ac/o2ac-ur/)).

## Observation messages
This section describes messages sending information about the actions.

### `TouchObservation.msg`
- `uint8 touched_object_id`: An enum of the touched object. If this value is 0, the touched object is the ground object. If it is 1, the touched object is the box object. 

### `LookObservation.msg`
- `sensor_msgs/Image looked_image`: The image obtained by the camera, represented as bgr8 image
- `std_msgs/uint32[4] ROI`: An array of length 4 representing the range of interests of the image. The range of interests is a rectangle and this array is [top boundary, bottom boundary, left boundary, right boundary].

### `PlaceObservation.msg`
- `float64 support_surface`: The z-coordinate of the support surface where the object is placed.

## updateDistribution action
This section describes the updateDistribution action. It receives the current pose belief along with information about actions and returns the new pose distribution.

### Goal
- `uint8 observation_type`: The type of the action. If this value is `TOUCH_OBSERVATION` (constant, equal to 0), the action is "touch". If it is `LOOK_OBSERVATION` (constant, equal to 1), the action is "look". If it is `PLACE_OBSERVATION` (constant, equal to 2), the action is "place". If it is `GRASP_OBSERVATION` (constant, equal to 3), the action is "grasp". If it is `PUSH_OBSERVATION` (constant, equal to 4), the action is "push".
- `TouchObservation touch_observation`: When the type of action is "touch", this represents information about the action.
- `LookObservation look_observation`: When the type of action is "look", this represents information about the action.
- `PlaceObservation place_observation`: When the type of action is "place", this represents information about the action.
- `geometry_msgs/PoseStamped gripper_pose`: The pose of the gripper when the action is executed.
- `moveit_msgs/CollisionObject gripped_object`: A CollisionObject representing the grasped object.
- `uint8 distribution_type`: The method that is used to represent uncertainty of the pose. If this value is `RPY_COVARIANCE` (constant, equal to 0), the `covariance` attribute of the following `distribution` is interpreted as the covariance matrix in the space of x, y, z, roll, pitch, yaw. If it is `LIE_COVARIANCE` (constant, equal to 1),  the `covariance` attribute is interpreted as the covariance matrix in the vector space identified with the Lie algebra $se(3)$.
- `geometry_msgs/PoseWithCovarianceStamped distribution`: The current pose distribution of the object

### Result
- `bool success`: If the update was calculated successfully, this value is `true`. Otherwise `false`.
- `geometry_msgs/PoseWithCovarianceStamped distribution`: If the update is calculated successfully, this stores the updated distribution of the pose of the object.
- `std_msgs/String error_message`: If update is not calculated successfully, this string contains the error message.

### Feedback
Nothing

## visualizePoseBelief service
This section describes the visualizePoseBelief service, a service to receive a pose belief and publish a markey array to visualize it.

## Request
- `moveit_msgs/CollisionObject object`: A CollisionObject representing the object.
- `uint8 distribution_type`: Which method is used to represent uncertainty of the pose. Same as in `updateDistributionGoal`.
- `geometry_msgs/PoseWithCovarianceStamped distribution`: The current distribution of the pose of the object
- `duration lifetime`: The lifetime of the markers.

## Response
Nothing

# Details of messages
This section describes the messages sending information about the actions.


# File Structure
```
o2ac_pose_distribution_updater           # package direcotory
├── CMakeLists.txt
├── include                              # directory containing header files
│   └── o2ac_pose_distribution_updater   # header files for this package
│       ├── base			 # directory containing header files without ros
│	│   ├── conversions.hpp              # conversion functions, header only
│  	│   ├── convex_hull.hpp              # fuctions about convex hulls
│       │   ├── estimator.hpp                # class calculating distributions
│       │   ├── grasp_action_helpers.hpp     # functions for calculations associated to grasp action
│       │   ├── operators_for_Lie_distribution.hpp     # functions for Lie distribution, header only
│       │   ├── place_action_helpers.hpp     # functions for calculations associated to place action
│       │   ├── planners.hpp                 # class of planners
│       │   ├── planner_helpers.hpp          # functions for calculations associated to planning
│       │   ├── push_action_helpers.hpp      # functions for calculations associated to push action
│       │   ├── random_particle.hpp          # function to generate random particles
│       │   └── read_stl.hpp                 # function to read stl files
│       ├── ros				 # directory containing header files with ros
│       │   ├── distribution_conversions.hpp # fuctions to convert between PRY and Lie
│       │   ├── pose_belief_visualizer.hpp   # class to visualize pose beliefs
│       │   ├── ros_converted_estimator.hpp  # class calculating distributions, wrapped for ros message input
│       │   └── ros_converters.hpp           # conversion functions associated with ros message
│  	└── test			 # directory containing header files for test
│           ├── test.hpp                     # procedures for unit test
│           └── test_tools.hpp               # functions associated to testing
├── launch                               # direcotory containing launch files
│   ├── distribution_updater.launch      # for launching action server for updateDistribution action
│   └── estimator configure              # configuration file for estimator
├── package.xml                          # package discription
├── README.md                            # this README
├── src                                  # direcotory containing source files
│   ├── base				 # direcotory containing source files without ros
│   │	├── conversions.cpp                  # implementation of conversions.hpp
│   │	├── estimator.cpp                    # implementation of estimator.hpp
│   │	├── convex_hull.cpp                  # implementation of convex_hull.hpp
│   │	├── grasp_action_helpers.cpp         # implementation of grasp_action_helpers.hpp
│   │	├── place_action_helpers.cpp         # implementation of place_action_helpers.hpp
│   │	├── planners.cpp                     # implementation of planner.hpp
│   │	├── planner_helpers.cpp              # implementation of planner_helpers.hpp
│   │	├── push_action_helpers.cpp          # implementation of push_action_helpers.hpp
│   │	├── random_particle.cpp              # implementation of random_particle.hpp
│   │	└── read_stl.cpp                     # implementation of read_stl.hpp
│   ├── ros				 # direcotory containing source files with ros
│   │	├── action_server.cpp                # implementation of the action server
│   │	├── distribution_conversions.cpp     # implementation of distribution_conversios.hpp
│   │	├── pose_belief_visualizer.cpp       # implementation of pose_belief_visualizer.hpp
│   │	├── ros_converted_estimator.cpp      # implementation of ros_converted_estimator.hpp
│   │	└── ros_converters.cpp               # implementation of ros_converters.hpp
│   └── test                             # sources for unit test
│       ├── look_test.cpp                # implementation of look_test in test.hpp
│       ├── place_test.cpp               # implementation of place_test in test.hpp
│       ├── test_client.cpp              # test client which executes touch, look and place tests
│       ├── touch_test.cpp               # implementation of touch_test in test.hpp
│       └── ...                          # other tests
├── test                                 # files used in unit test
│   ├── CAD                              # stl files
│   │   └── ...
│   ├── grasp_test_cones_Lie_1.txt       # test cases for grasp test
│   ├── ...
│   ├── look_action_images               # jpg files used in look test
│   │   └── ...
│   ├── look_gripper_tip.csv             # csv files used in look test
│   ├── place_test_cones_1.txt           # test cases for place test
│   ├── ...
│   ├── test.rviz                        # rviz config file for unit test
│   ├── test_client.launch               # launch file for test client
│   ├── touch_input_gearmotor_1.txt      # test case for touch test
│   ├── unit_test.test                   # .test case file for unit test
│   ├── visualize_test.rviz              # rviz config file for test sample
│   ├── visualize_test.launch            # launch file for test sample
│   └── visualize_test_config.txt        # config text file for test sample
└── test_results
```
