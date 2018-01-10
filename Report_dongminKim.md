#CarND - MPC - Project#
---


### The Model ###
I used [x, y, psi, v, cte, epsi] as the state and [delta, a] as the actuators. Below is update equations in my code in MPC.cpp line 90:


```

    fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[1 + psi_start + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * dt);
    fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
    fg[1 + cte_start + t] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
    fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0) + ((v0 / Lf) * delta0 * dt);

```
fg is vector which save the error about each states. Every right parts in equations which multiply dt are update equations. Below equations were based by this code.
```
x​t+1​​=x​t​​+v​t​​∗cos(ψ​t​​)∗dt

y​t+1​​=y​t​​+v​t​​∗sin(ψ​t​​)∗dt

ψ​t+1​​=ψ​t​​+​L​f​​​​v​t​​​​∗δ​t​​∗dt

v​t+1​​=v​t​​+a​t​​∗dt

cte​t+1​​=f(x​t​​)−y​t​​+(v​t​​∗sin(eψ​t​​)∗dt)

eψ​t+1​​=eψ​t​​+​L​f​​​​v​t​​​​∗δ​t​​∗dt
```

### Timestep Length and Elapsed Duration ###
In Motion Predictive Controller, general guidelines is that T should be as large as possible, while dt should be as small as possible. So, I I used N: 25 and dt: 0.05(T = N * dt) which values are used in MPC Quiz.


### Polynomial Fitting and MPC Preprocessing ###
I used Cubic equations for polynomial fitting in main.cpp line 118:
```

auto coeffs = polyfit(trans_ptsx, trans_ptsy, 3);

```
In MPC preprocessing, I transformed ptsx, ptsy by vihicle's coordinate system using px, py in main.cpp line 106:
```

Eigen::VectorXd trans_ptsx(ptsx.size());
Eigen::VectorXd trans_ptsy(ptsy.size());
    for(unsigned short i = 0; i < ptsx.size(); i++){
        double x = ptsx[i] - px;
        double y = ptsy[i] - py;
        trans_ptsx[i] = x * cos(-psi) - y * sin(-psi);
        trans_ptsy[i] = x * sin(-psi) + y * cos(-psi);
    }
    
```

### Model Predictive Control with Latency ###
```
    psi = -(v / Lf) * delta * Latency;
    v = v + a * Latency;
    px = v * cos(psi) * Latency;
    py = v * sin(psi) * Latency;
    
```
Above codes (in main.cpp line 113) show how I handle latency. Latency is 100ms so I set this 0.1. And calculate after 100ms states. I used psi and v after 100ms value and then update x and y coordinate using  psi, v.



