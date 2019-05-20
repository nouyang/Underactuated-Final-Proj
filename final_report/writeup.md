https://en.wikipedia.org/wiki/Nonlinear_system

This a really neat image



![upload successful](/hexblog/images/pendulum_dynamics_linearization.png)

https://en.wikipedia.org/wiki/Phase_portrait
phase portrait! of trajectories using state variables

http://underactuated.mit.edu/Spring2019/final_project.html
> Perform region of attraction estimation on a real robot (e.g., for an LQR stabilization task). There are natural ways to incorporate uncertainty into the verification (just ask the staff). What types of uncertainty do you need to include to get a reasonable verification of the true system?


## Abstract


## Introduction


## Simulation




## Hardware


### Design decisions
make a diagram of the three iterations
the tdifferent max torques possible

### Theory to Reality

Our analysis is performed with torque output. In reality, we control motor voltage. We also do not directly., and are performing a naive "state estimation" by integrating discrete quadrature encoder counts.

### Bang-bang control
 
State machine - make sure directions are correct

### PD Control

blah blah blah

### Hardcoded assumptions

for testing... 

### Results

table of times
You can prod it and it rejects the disturbance

### motor velocity state estimation

* Current control -- different for different stages of hbridge
* But if we did, we could do... (calculations)
 * Methods: measuring motor constant
 
 We saw systems on the internet which did not need this, so we proceeded without it.
 
#### Mechanical Lessons Learned

Pressfits are great! They're used for... everything.
Decisiveness is good -- we weren't certain about going for the better motor, which instinctually thought it'd be needed but went for it and glad we did.
Glad we gave some consideration to clearance -- bigger motor and flwheel barely fit (scrapes the staples).
Maybe would go back to 3d printed version
Debugging...
 
### Future work

probably upright stabilization
----------------------------------------------------------
## Analyses 
----------------------------------------------------------

### Napkin calculations

Torque output (control authority) of three designs -- how far can we deviate and still be able to return?


mgl sin(theta)

original m:
2nd m
3rd m


### LQR 

We briefly review the equations of motion and linearization techniques used for
linear quadratic regulator (LQR) control. This is considered a "full state"
feedback control method. In fact, our control will be simply defined as 



Optimal in some cost function, with respect to this quadratic cost function 
optimal linear full state feedback


Explain what is LQR


Confirmed with sympy. (have code one-liners)

To investigate: the devis-nichols system specification (for sympy)


### Detour: System on wheels

What is we stuck the whole thing on wheels instead? 
Sort of combining this system with the cartpole. 
Re-derive equations of motion -- develop LQR, maybe region of attraction analysis.

#### Controllability

We can either control the , or the, not both.

## Results

In section 1, we presented a graph of our hardwares.
In section 2, we presented the modified equations of motion for a system.



## Conclusion / Reflection / Future work

Implement it in hardware on wheels, clearly.
