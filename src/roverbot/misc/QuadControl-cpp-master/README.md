# Quadcopter Controller
C++ library for embedded control of quadrotor

Please see the video of the controller code in action:

[Test video of bi-level controller](https://www.youtube.com/watch?v=SFXgEcob1RA)

- Please see my posts for setting up the beaglebone as the main onboard controller and an arduino to control the motors and the IMU sensor
  1. http://www.nagavenkat.adurthi.com/?p=304
  2. http://www.nagavenkat.adurthi.com/?p=614
  3. http://www.nagavenkat.adurthi.com/?p=601
  4. http://www.nagavenkat.adurthi.com/?p=563

## A brief summary of the controller codes

- The controller code consists of 3 main different controllers
  1. The ground station that sends position and attitude measurements over UDP to the main onboard controller
  2. The onboard controller that receives measurements and computes the control action. It also runs a Extended Kalman Filter to estimate velocity, position and attitude from noisy measurements
  3. The onboard -sub motor controller that controls the speeds of the motors. 
- The onboard controller does estimation (EKF using quaternions) and a geometric nonlinear controller.
- All the code is implemented in c++
- The matrix computations are done using the Eigen library
- There is a Joystick control incuded in it too. You can instantaneously switch to automatic or manual control
- the main sensor measurements and communications with onboard moter controller and ground station are all multi-threaded for parallel execution of estimation, filtering and control.
- Onboard sensor measurements come from IMU at a higher rate for attitude stabilization and control
- The motors are controlled using PWM using an ardruino(the onboard -sub motor controller).  The main onboard controller and this arduino sub controller communicate on SPI. There is also a self stabilization code within the ardunio code to automatically stabilize the quadcopter when it recieves no control actions from the main onboard controller.
- The IMU communicates over I2C to the main onboard controller and the sub-motor controller(ardunio)
