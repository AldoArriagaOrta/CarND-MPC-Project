#  Model Predictive Control

---

**MPC Controller Project**

The goals of this project are the following:

* The code should compile.
* A reflection/discussion on the implementation  should be included.
* The vehicle must successfully drive a lap around the track in the simulation tool.

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points

---

### Reflection

#### The Model. Detailed description including the state, actuators and update equations.

It is based on the CRTV model, in which the state vector is defined by x and y coordinates, speed and yaw angle (psi); the actuators are represented by the variables delta and alpha (the steering angle and throttle input, respectively):

```c++
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
```

Where dt represents the timestep duration. The cross track error and orientation error are computed as:

```c++
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where LF represents the the length from front to CoG.

However, these equations were manipulated in order to be used in the solver and to set the constraints to 0.


#### Timestep Length and Elapsed Duration (N & dt)

As starting point, the values of the lesson were used (N=25, dt=0.5). However, these values resulted too large i.e.  the computations took too long for N=25. This would need to be further considered for latency and thus smaller values were tested.
When dt>latency, the effects of these latter would be masked. Nonetheless, smaller values for the dt are needed for a good approximation of the continuous state (to avoid large discretization errors).

The following values were finally used:

```c++
size_t N = 10;
double dt = 0.1;
```

#### Latency Handling

These model equations are used to predict where the car will be after the latency period

```c++
 double latency_s = 0.1;

 double x_lat = px + v * cos(psi) * latency_s;
 double y_lat = py + v * sin(psi) * latency_s;
 double psi_lat = psi +  v / Lf * delta * latency_s;
 double v_lat = v + acc * latency_s; //this is not really accurate, the throttle value is not the acceleration... 
```      
      
This approach decouples the latency of the timestep duration. These values are now used as initial state when calling MPC solve.

#### Polynomial Fitting and MPC Preprocessing

Transformation of waypoints from global reference frame to vehicle coordinates and mapping std::vector to Eigen::VectorXd is done as shown in the next snippet:

```c++
  Eigen::VectorXd ptsx_(ptsx.size());
  Eigen::VectorXd ptsy_(ptsy.size());
  for (unsigned int i = 0; i < ptsx.size(); i++) {
    double x_offset = ptsx[i] - x_lat;
    double y_offset = ptsy[i] - y_lat;
    ptsx_[i] = x_offset * cos(0-psi_lat) - y_offset * sin(0-psi_lat);
    ptsy_[i] = x_offset * sin(0-psi_lat) + y_offset * cos(0-psi_lat);
  }
```  

The waypoints are fitted to to a 3rd order polynomial:

```c++
	auto coeffs = polyfit(ptsx_, ptsy_, 3);
```

The cross track error is calculated by evaluating at polynomial at x, f(x) and subtracting y.
```c++
	double cte = polyeval(coeffs, px) - py;
```

After the homogeneous transformation we are seeing the values at vehicle's coordinates and the previous definition becomes:

```c++
  double cte = polyeval(coeffs, 0);
```

As for the yaw angle error (epsi) the derivative of the polynomial is considered, and after the transformation to vehicle's coordinates:

```c++
  double epsi = -atan(coeffs[1]);
```

Finally, the state vector is built and sent, along with the polynomial coefficients, to MPC solve, in order to obtain the steering angle and throttle input:

```c++
  Eigen::VectorXd state(6);
  //set the current state variables
  state << 0, 0, 0, v_lat, cte, epsi;

  double steer_value;
  double throttle_value;

  //Call MPC
  auto controls = mpc.Solve(state, coeffs);
  steer_value = controls[0]/(deg2rad(25)*Lf);
  throttle_value = controls[1];
```

