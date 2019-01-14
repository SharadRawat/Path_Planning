
## SDCND - Term 3 - Path Planning Project

##### Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

The project must meet all points in the rubric. 

#### Project Solution

The project has been completed in main.cpp file. Apart from main.cpp, spline.h was used as described in chapter lessons. Cubic splines were used to generate smooth trajectories. The helper functions in the main.cpp are not modified and used as is. Only the main is changed.

The max speed is within the max 50 mph. This was enforced by keeping all the velocities below a `max_vel` value set as `49.5 mph`. The acceleration and jerk is below `10 m/s^2` and `10 m/s^3` at all times becuase an increment of `0.224` which is roughly equal to `5m/s` velocity increment is enforced between lines 303 and 331.

In lines between 255 to 299, the positions of all other vehicles are analyzed relative to the 'ego' vehicle. The boolean too_close is changed to true if the two vehicles are within 30m of each other. Similarly, vehicles from neighbouring lanes are also scanned, if the vehicles are close (within the margin), `car_left` and `car_right` are changed to true.

`// Lane identifiers for other cars
			bool too_close = false;
			bool car_left = false;
			bool car_right = false;
			// Find ref_v to use, see if car is in lane
			for (int i = 0; i < sensor_fusion.size(); i++) {
				// Car is in my lane
				float d = sensor_fusion[i][6];
				// Identify the lane of the car 
				int car_lane;
				if (d >= 0 && d < 4) {
					car_lane = 0;
				} else if (d >= 4 && d < 8) {
					car_lane = 1;
				} else if (d >= 8 && d <= 12) {
					car_lane = 2;
				} else {
					continue;
				}
				// Check width of lane, in case cars are merging into our lane
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx + vy*vy);
				double check_car_s = sensor_fusion[i][5];
				// If using previous points can project an s value outwards in time
				// (What position we will be in in the future)
				// check s values greater than ours and s gap
				check_car_s += ((double)prev_size*0.02*check_speed);
				int gap = 30; // m
				// Identify whether the car is ahead, to the left, or to the right
				if (car_lane == lane) {
					// Another car is ahead
					too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);
				} else if (car_lane - lane == 1) {
					// Another car is to the right
					car_right |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
				} else if (lane - car_lane == 1) {
					// Another car is to the left
					car_left |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
				}
			}`


In line 302 to 331, logic for action has been encoded on how to change speed and how to change lanes.

A) If a car is ahead within 30 meters, left and right lanes are inspected. If one of them is empty, the car will change lanes. Else it slows down.

B) If the area in front of the car is clear, no matter the lane, the car speeds up.

C) The car will move back to the center lane when it becomes clear. 



`double acc = 0.224;
			double max_speed = 49.5;
			if (too_close) {
				// A car is ahead
				// Decide to shift lanes or slow down
				if (!car_right && lane < 2) {
					// No car to the right AND there is a right lane -> shift right
					lane++;
				} else if (!car_left && lane > 0) {
					// No car to the left AND there is a left lane -> shift left
					lane--;
				} else {
					// Nowhere to shift -> slow down
					ref_vel -= acc;
				}
			} else {
				if (lane != 1) {
					// Not in the center lane. Check if it is safe to move back
					if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
						// Move back to the center lane
						lane = 1;
					}
				}
				if (ref_vel < max_speed) {
					// No car ahead AND we are below the speed limit -> speed limit
					ref_vel += acc;
				}
			}
`

The trajectory is generated in the lines from 332 to 436. Here, previous 2 points are also appended in the next set of points to reduce the jerk due to sudden change in the direction. `3` evenly spaced points are generated at 30m, 60m, and 90m. A rotation and translation transformation is also applied to change the coordinate system from global to local (car's). 

From lines 395 to 435, the spline is used to compute the points for the trajectory.

`
  vector<double> ptsx;
			vector<double> ptsy;
			// Reference x, y, yaw states
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			// If previous size is almost empty, use the car as starting reference
			if (prev_size < 2) {
				// Use two points that make the path tangent to the car
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);
				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);
				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			} else {
				// Use the previous path's endpoint as starting ref
				// Redefine reference state as previous path end point
        		// Last point
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];
				// 2nd-to-last point
				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
				// Use two points that make the path tangent to the path's previous endpoint
				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);
				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}
			// Using Frenet, add 30 m evenly spaced points ahead of the starting reference
			vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);
			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);
			for (int i = 0; i < ptsx.size(); i++) {
				// Shift car reference angle to 0 degrees
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;
				ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
				ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
			}
			// Create a spline called s
			tk::spline s;
			// Set (x,y) points to the spline
			s.set_points(ptsx, ptsy);
			// Define the actual (x,y) points we will use for the planner
			vector<double> next_x_vals;
			vector<double> next_y_vals;
			// Start with all the previous path points from last time
			for (int i = 0; i < previous_path_x.size(); i++) {
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}
			// Compute how to break up spline points so we travel at our desired reference velocity
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
			double x_add_on = 0;
			// Fill up the rest of the path planner to always output 50 points
			for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
				double N = (target_dist/(.02*ref_vel/2.24));
				double x_point = x_add_on + (target_x) / N;
				double y_point = s(x_point);
				x_add_on = x_point;
				double x_ref = x_point;
				double y_ref = y_point;
				// Rotate back to normal after rotating it earlier
				x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
				x_point += ref_x;
				y_point += ref_y;
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}
`
