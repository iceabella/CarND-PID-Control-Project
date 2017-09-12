# CarND-Controls-PID
This is my implementation of the PID project for Udacity's Self-Driving Car Engineer Nanodegree Program.

## Project introduction
In this project a PID controller is used to calculate the steering angle and throttle actuations for driving around a test track in [this Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases). The inputs to the PID controller from the simulator are vehicle velocity and the cross track error to a reference trajectory. 

## Parameter tuning and effect of different P, I, D gains
For the throttle I only used a simple P-controller since I didn't feel that a more advanced controller was needed. The error I tried to controll towards was the current vehicle speed compared to the wanted/nominal speed.

For the steering I used manual tuning. I also used Twiddle but in the end I realised it was easier to find good values by tuning manually, therefore I didn't use it in the end.

Parts of how I reached my final set of parameters for the steering control and how different parameters affect the behaviour can be seen in [this video](https://youtu.be/bQxfa3n6nS8). I will here the discuss the parameters with the use of the video. 

I started by only changing the P gain to see how this would affect the steering. The first value I got which took the vehicle around the test track was Kp=0.05, which can be seen in the video. What can also be seen is that this steering is not enough to keep the vehicle in the middle of the road. If I increased Kp I could see that I got more steering power and that the steering was reaching the middle faster, but also started to oscillate since it was overshooting all the time.

By introducing a small gain on Kd the oscillations did get smaller, but if I added a too high Kd the oscillations increased again.

I tried to find smooth driving by increasing Kp and Kd in iterations but felt that I didn't reach the result I wanted. Therefore I changed approach.

Notice that what I wanted to achive was that the vehicle steers smoothly, i.e. intuitively I would like that the difference between the current and last error has a bigger influence than the current error. What I wanted was to make the vehicle able to turn enough by increasing the Kp value but still let the Kd value have more influence. This can be seen in the next part of the video with Kp = 0.05 and Kd = 0.1. This resulted in that we can go around the track smoothly but the steering seems to be a little bit slow, i.e. Kp needs to be increased. When we increase Kp we also want to increase Kd so that it still has bigger influence. 

By increasing Kp = 0.1 and Kd = 0.25 we can see in the next part of the video that the vehicle can drive smoothly around the test track and with a better placement on the road.

By adding a Ki gain we can see in the next part of the video that we get a faster response of the system (Ki = 0.01). We can see that the system drives more in the middle of the road, since it starts to react earlier.

My next step was that I wanted to increase the speed. When I did that with the PID controller Kp = 0.05, Kd = 0.1 and Ki = 0.01, the vehicle was not able to drive around the track anymore (it was oscillating too much). 

I was setting Ki = 0 again to tune without its influence. I then increased the Kd value to be able to get the oscillations away but realised it just made them worse, therefore I tried to decrease it instead. This made the steering smoother again, the Kd I used before was creating overshoots. This can seem to be quite logic since in a higher speed the vehicle will travel further between each measurement which means that I will needed to steer the vehicle with smaller steering values.

I needed to increase Kp again to be able to steer enough in the curves, thereafter I also added a Ki gain again. My final values (Kp = 0.07, Kd = 0.06 and Ki = 0.0015) where possible to use for both lower and higher speeds (18-55mph), which can be seen in the last part of the video.


--- 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to keep it as simple and environment agnostic as possible. However, we recommend using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.
