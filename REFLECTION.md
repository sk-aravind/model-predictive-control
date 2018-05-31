# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Reflections

### The Model
The Model comprises of 6 variables that describe the state of the car. Namely, x-coordinate of car, y-coordinate of car, orientation of car, velocity of the car, cross-track error and orientation error. The model also has two actuators, the steering angle and the acceleration. Using this variable as a model we are able to calculate the next state.

In order to update the these state variables we utilize the kinematic vehicle model that is given below
![](/imgs/equations.png "Equations for Model")

Where:
* 'x' and 'y' - the coordinates of our car
* 'psi' - heading direction of the car
* 'v' - velocity of the car
* 'cte' - cross track error
* 'epsi' - orientation error
* 'delta' - the steering angle
* 'a' - acceleration  
* 'Lf' - the distance between the front wheels and the center of gravity of the car.



### Timestep Length and Elapsed Duration (N & dt)

The number of points (N) and the time interval (dt)  help in computing the prediction horizon (= N * dt). With higher number of points the computational cost of the algorithm increases and results in slower predictions, with a lower number of points there is at time insufficient data to accurately predict the future in sharp turns. After experimenting with different values I found that the combination of N=10, dt=0.1 gave the best results.




### Polynomial Fitting and MPC Preprocessing
We need to preprocess the way points in order to convert them from global map coordinates to car coordinates. To fit the way points we utilize a polynomial of order 3.

```
for (int i = 0; i < ptsx.size(); i++) {
            double d_x = ptsx[i] - px;
            double d_y = ptsy[i] - py;
            ptsx_t[i] = d_x * cos(-psi) - d_y * sin(-psi);
            ptsy_t[i] = d_x * sin(-psi) + d_y * cos(-psi);
          }

          // Fit a polynomial to points x, y
          auto coeffs = polyfit(ptsx_t, ptsy_t, 3);
```

### Model Predicted Control with Latency

By computing the state values using the model and delay interval of 100ms we were able to deal with the actuator latency and prevent the vehicle from oscilating at high speed. Therefore, when the MPC calculates, it uses the values with the updated position instead of the initial position.
