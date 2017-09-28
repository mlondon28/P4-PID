# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Describe the effect each of the P, I, D components had in your implementation.

For this project I decided to go with a PD controller. Trying to deal with the integral windup was not worth the effort.

Description of Effect of each type of gain:

**Proportional** - The further away the current state is from the desired state, the harder the control effor. *Control effort is proportional to how far away it is to the set point.* In terms of the car on the track, the control input to turn toward the center of the track would increase the further away from the center of the track the car got.

**Integral** - If the car had some kind of steering drift causing the car to settle slightly off the set point, integral gain would help increase the control effort in the right direction until the desired position matched the actual position. Unfortunately this was difficult to tune with the track changing so suddenly. Once the lane turned and the car was off center for a period of time it would have a lot of trouble recovering from the integral windup. Integral windup is when the integral term gets so large that it actually passes the desired set point and then keeps going in the wrong direction. This could be solved by implementing a maximum amount of integral gain storeable. Also can implement something that zeros the integral gain once the crosstrack error is zero.

**Derivative** - This is effectively a reaction damper on quick changes in the control effort. This could cause problems with settling on the exact desired setpoint (the center of the track) but as the track is a continuously changing path, it works well enough.



## Describe how the final hyperparameters were chosen.

I tuned these paramters using a combination of the twiddle algorithm, Zeigler-Nichols principles and manual tweaking. 

I started using a few ZN principles and disabled the D and I terms. Once I got the P term in the ballpark range where it could work properly I enabled the twiddle algorithm. 

The twiddle algorithm was only tuning the PD terms as I had disabled the integral term. I had it set up to go about halfway around the first term before restarting with modified gain with respect to the Mean square error results. 

Finally I fine tuned the gains to get the rest of the way around the track with twiddle disabled.














