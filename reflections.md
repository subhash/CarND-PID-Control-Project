The cross-track error is the deviation of the vehicle from it's destined path. By formulating the steering-angle as a function of the cross-track error, the PID controller aims to continually "nudge" back the vehicle to its path.

```
  double prop = this->Kp * this->p_error;
  double diff = this->Kd * this->d_error;
  double integ = this->Ki * this->i_error;
  double err =  - prop - diff - integ;
```

The P-gain is the most obvious control parameter. It forces the steering angle to be in proportion to the current cross-track error and in the opposite direction. This can quickly bring the vehicle to its path but also causes it to overshoot.

The D-gain aims to prevent the overshooting by providing an opposing force to that of P-gain. It works on the differential cross-track error and pushes the vehicle away from the destined path, proportional to the differential cte. This avoids linear trajectories and forces a smoother convergence to the zero-cte path.

The above gains would suffice if the vehicle does not suffer from "bias". Sometimes, due to vehicle configuration, there is a constant error between the control steering-angle and the actual steering-angle. This error accumulates as the vehicle moves. Therefore, we need a third I-gain, which works on the integral/sum of the cte and compensates for the bias in the system.

I took the option of manually tuning the hyperparameters because I thought that would teach me how the different gains actually worked. Before starting on setting values for the parameters, I studied the system to see if there was any bias by always sending a zero steering-angle and checking the response. I found a bias of `0.0174`, a constant error in the straight path. A rough calculation ensued, based on the initial cte - `0.73` -  `0.0174/0.73 = 0.023`. This would be the value of `ki` required to correct the bias right in the first step, but it would weigh too heavy in subsequent steps which presumedly have lesser cte. Therefore, I decided to compensate for the initial bias in 200 steps, by which time more bias error would have crept in and the integral gain would again be justified. I settled for `ki=0.23/200`.

For the proportional gain, I started with an initial value of `kp=0.1`. With both these values, the vehicle seemed to be able to track the road, but was not keeping to the middle path. I increased `kp=0.15` and it gave better results.

As the vehicle progressed, it started oscillating, and I tried out again with `kd=0.6`. The oscillations stopped in the lesser turns but continued in sharp turns. I doubled the value of `kd=1.2` and when that proved insufficient, turned it further up to `kd=3.0` and the vehicle successfully completed the lap.
