# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## [Rubric](https://review.udacity.com/#!/rubrics/896/view) points discussion

### The Model
The kinematic model includes:
- vehicle's x and y coordinates
- orientation angle (psi)
- velocity
- cross-track error (cte)
- psi error (epsi). 
Outputs from the actuators are acceleration and delta (steering angle).
POPSI model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the following equations:

    // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
		  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
		  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
		  // v_[t+1] = v[t] + a[t] * dt
		  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
		  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
### Timestep Length and Elapsed Duration (N & dt)
The values chosen for N and dt are 8 and 0.1. I started with the values suggesed ( N = 10 , dt = 0.1) and then did some experiments varying the values ( 10>N>7) (0.2 >dt>0.08) till I got the best behaviour.

Also Weights for how much attention the cost function will pay for  each of these cost terms(Attributes), are tuned to the following values: 
	  - cte_weight = 2000;
	  - epsi_weight = 3000;
	  - v_weight = 1;
	  - delta_weight = 5;
	  - a_weight = 5;
	  - delta_change_weight = 200;
	  - a_change_weight = 10;
### Polynomial Fitting and MPC Preprocessing
// Transform from Simulator into car coordinate
		  // this means we can consider px = 0, py = 0, and psi = 0
		  // 
		  for (int i = 0; i < ptsx.size(); i++) {
			  double shift_x = ptsx[i] - px;
			  double shift_y = ptsy[i] - py;
			  // Rotating around the origin
			  ptsx[i] = (shift_x * cos(-psi) - shift_y * sin(-psi));
			  ptsy[i] = (shift_x * sin(-psi) + shift_y * cos(-psi));
		  }


		  double* ptrx = &ptsx[0];
		  double* ptry = &ptsy[0];
		  Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
		  Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

		// Fitting the transformed points to  a 3rd - order polynomial
		  auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
### Model Predictive Control with Latency
 In the model equations the actuations from the previous timestep are generated with a delay of 100ms. 
 Model update equations are used as follows:
    fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		  fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		  fg[2 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
		  fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
		  fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		  fg[2 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
# P10
