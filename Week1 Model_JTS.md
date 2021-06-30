```python

```

<center> <h1><u> Week 1 </u></h1></center>

## Introduction

Have you tried balancing a stick on one end with your palm? It is difficult the first time one tries, but with some practice, one could stabilize the stick around the vertical position more or less for a certain duration of time. One who tries this would certainly agree that the act of balancing a stick requires a great deal of fine control.

The above example is just one of the many instances around us where things need to be controlled. Control refers to the action of trying to bring about desired behaviour/ properties in the state of a system, which is obvious. Many devices which seem to work like a charm require a good amount of control in the background for them to function properly. This is required for various reasons, such as ensuring that unwanted factors like noise or disturbances do not jeopardize their working, ensuring optimal resource usage, and maintaining stability.

If you don’t want your systems to go out of control, this is the place to know a few basic ideas behind controlling.

Anybody who wishes to know more about control theory will certainly refer to this at some point, so here is the link to get started… 

[Control Theory](https://en.wikipedia.org/wiki/Control_theory)

These are motivational videos to get psyched for what’s about to come 

[Why Learn Control Theory](https://www.youtube.com/watch?v=oBc_BHxw78s&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=1
)

[Control Theory Overview](https://www.youtube.com/watch?v=Pi7l8mMjYVE&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m&index=2)

With all the enthu you have now, let’s begin…

## Overview

The following broad topics will be covered this week-
*	Control Systems, their types and features
*	Model representation using differential equations and transfer functions
*	Stability and Controllability
*	Introduction to PID Control



## Control systems
 


**Control system** - A collection of components that are collectively responsible for bringing the output of the system to the one desired by the input.

![image-2.png](attachment:image-2.png)

The **input** consists of parameters that dictate what the desired output should be. 

The **output** consists of a set of variables that describe the features of the system, collectively referred to as the state of the system. The state of a system is generally represented by $x$ .

Example — position and velocity of a car.

A control system generally consists of the following components - 

**Plant** — The part of the control system that is being controlled. It consists of the actuator, which executes the control command, and the process which responds to the actuation and undergoes a time evolution. In many cases, the plant could be a dynamic system like a car or a pendulum.

**Controller** — The part of the control system that provides control commands to the plant so that the state gets driven to the desired one. The control command is generally represented by $u$ .

**Sensors** — The part of the control system that takes an observation/measurement of the state completely or partially. The sensor observation is generally represented by $y$ .

## Types of control systems

**Open Loop control systems** - Applying control commands without taking any measurements of the output
![image-3.png](attachment:image-3.png)
 

**Closed Loop control systems/ Feedback control systems**- Applying control commands based on the measurements of the output with the help of a controller

![image-4.png](attachment:image-4.png)
 

**Advantages of Closed Loop control** - 

*	Greater resistance to noise/bias/disturbances to the system
*	Efficient energy consumption
*	Alter the dynamic properties of the overall system

To know more about control systems, check out the following-

[Control Systems Intro](https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm)

[Closed-Loop Control](https://www.youtube.com/watch?v=O-OqgFE9SD4&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=6)



## Types of control-

**Passive control** — It involves minimal energy expenditure to impose desired behaviour

Example - streamlined shapes of vehicles

**Active control** — It involves greater energy expenditure for desired behaviour.

Example — oscillating the base of an inverted pendulum at a frequency to keep it stable.

## Linear and Nonlinear systems

**Linear systems** — Systems where the evolution of the state of the system is related linearly with the state. An example is radioactive decay.

$$\dot x = Ax$$

In a more general context, systems in which the output depends linearly on the input (satisfies properties of scaling and linear combination) are also described as linear systems.

**Nonlinear systems** — Systems where the evolution of state is not related linearly with the state.
An example is a simple pendulum in a gravitational field.

$$ \dot x = f(x) $$

Many nonlinear systems can be treated as linear systems approximately around certain states
(called fixed points, where $\dot x=0$), through **local linearization**, which will be explored in the upcoming weeks.

Check out the following to know more about Linear systems in different contexts-

[Linear Systems](https://www.youtube.com/watch?v=nyqJJdhReiA&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m&index=3)

[LTI](https://www.youtube.com/watch?v=3eDDTFcSC_Y&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=4)



## Continuous and discrete-time systems

**Continuous-time systems** — Systems where the evolution of the state is considered for any time. 

Example - $$\dot x = Ax$$

**Discrete-time systems** — Systems where the evolution of the state is considered only at discrete time steps.

Example - (Discrete-time, linear system)  $$ x_{k + 1} = \tilde Ax_{k}$$ 

Each time step $k$ is separated  $\Delta t >=\epsilon > 0$



## Model representation

**Impulse response** — Output of a system when subjected to a control command which is an impulse (Dirac delta function) in time. Impulse responses in the time domain are linear and can be scaled and superpositioned (using convolution) to obtain responses to more general inputs.

**Step response** - Output of a system when subjected to a control command which is a step function in time.

The model of a control system/parts of a control system can be represented using
Differential equations or Transfer functions.

* **Differential equations** — They model the relationship between the input and output of the system in the time domain.
* **Transfer function** — Laplace Transform of the impulse response of a system when all initial conditions are set to zero. It models the relation between the input and the output in the frequency domain.
The values of s for which the denominator of the transfer function goes to zero are called **poles** and the values of s for which the numerator of the transfer function goes to zero are called **zeros**.

Consider the example of a spring attached to a wall, subjected to a force (control) $F=u(t)$ .

$ m\ddot x(t) + k x(t) = u(t) $ is the Differential equation representation.

If $ u(t) =\delta(t) $ (impulse function) and setting  $x(0)=0, \dot x(0) =0$  gives
  $ X(s) = \frac{1}{ms^2+k} $ which is the Transfer function representation.

To know more about Differential equations and Transfer functions, check out the following

[Time and Frequency Domain](https://www.youtube.com/watch?v=noycLIZbK_k&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=4)

[Transfer functions](https://www.youtube.com/watch?v=RJleGwXorUk)

[Laplace Transform and the Transfer function](https://www.youtube.com/watch?v=0mnTByVKqLM)

[Laplace Transform- A Graphical Approach](https://www.youtube.com/watch?v=ZGPtPkTft8g&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=8&t=3s)

[Zeros and Poles of a Transfer function](https://www.youtube.com/watch?v=AZ7_MvANy_Q)




















## Stability of Linear systems


**Evolution of a continuous-time linear system**

 $$ \dot x = Ax $$
 $$ \Rightarrow x(t) = e^{At} x(0) $$
 $$ e^{At} = \sum_{n=1}^\infty \frac{A^nt^n}{n!} $$

If $A$  can be represented as $V\Lambda V^{-1}$  where V is the eigenvector matrix and $\Lambda$ is the matrix of eigenvalues (these are also the poles of the system transfer function, hence their importance!), then
$$ e^{At} = V e^{\Lambda t} V^{-1} $$
$ \Rightarrow x_i(t) = e^{\lambda_i t} x_i(0)$ where $x_i$   is an individual component of the state vector and $\lambda_i$  is the corresponding eigenvalue. Thus, the different components get decoupled from each other.

**Condition for stability**

In continuous-time,

 $\forall i, \quad Re(\lambda_i) <=0 \Rightarrow $ Stable
 
  $\exists i \quad s.t \quad  Re(\lambda_i) > 0 \Rightarrow$ Unstable

Using a suitable controller in certain systems can help alter the eigenvalues/poles of the system, thus altering the stability of various states. This is the main idea behind **pole placement**, which will be seen later.


## Controllability of systems

**Controllability** — It is the property of a system where it is possible to access any state in the state space with a suitable controller (reachability / reachable set  $ R_t =\mathbb{R}^n $ from any given initial state in a finite time. 
In many cases, this implies that the eigenvalues of the system can be set to any arbitrary value (**arbitrary pole placement** in the context of transfer functions)

**Stabilizability** — It is the property of a system in which the unstable eigenvector directions are controllable (lie in the controllable subspace).

**Dynamics of a linear system in the presence of a controller**

$$ \dot x = Ax + Bu $$ where $Bu$ is the controller term

**Test for controllability of a system**

Controllability matrix, $C = \begin{bmatrix} B & AB & A^2B & A^3B & ... & A^{n-1}B \end{bmatrix} $

$n$ is the state space dimension
 
System is controllable  $\iff$ C is full rank.

**Example** - 

  $ A = \begin{bmatrix} 1 & 1 \\ 0 & 2 \end{bmatrix} $
  $ B = \begin{bmatrix} 0 \\ 1 \\ \end{bmatrix}$
  
  $ C = \begin{bmatrix} B & AB \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 1 & 2 \end{bmatrix}$

rank(C) = 2, so the system is controllable.

To know more about stability, controllability, reachability, and other ideas, refer to the following
[Difference between controllability and reachability](https://math.stackexchange.com/questions/3030305/what-is-the-difference-between-controllability-and-reachability)

[Reachability and Controllability](http://www.dii.unimo.it/~zanasi/didattica/Teoria_dei_Sistemi/Luc_TDS_ING_2016_Reachability_and_Controllability.pdf)

[NPTEL — Controllability and Reachability](https://www.youtube.com/watch?v=kkbq3pDf8UE)

[Control Theory Boot camp (Lectures 3-11)](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)












## Control law

**Control law** refers to the relation between the control command and other parameters, such as the state and input.

**Example** -

The control law for **Linear Quadratic Regulator (LQR)**
$$u(t)=-K(x(t)-x_0)$$  

When $u=-Kx$ is applied to a continuous-time linear system with a controller, the entire system becomes a closed-loop linear system where the dynamics are heavily influenced by the chosen $K$.

$ \dot x = Ax + Bu $
$ \Rightarrow \dot x = (A-BK)x = \bar A x $
 
The use of this shall be seen in the upcoming weeks.


## PID Controller

![image-2.png](attachment:image-2.png)


For a great introduction to PID Control, refer to the following

[Classical Control Theory — PID Controller](https://www.youtube.com/watch?v=UR0hOmjaHp0&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=28)

[PID Controller - Wikipedia](https://en.wikipedia.org/wiki/PID_controller)

[Controlling Self Driving Cars](https://www.youtube.com/watch?v=4Y7zG48uHRo)


**Control law for PID controller**  

$$ u(t) = K_p e(t) + K_i \int e(t) dt + K_d e'(t) = K_p(e(t) + \frac{1}{\tau_i}\int e(t) dt + \tau_d e'(t))$$
 
* $K_p$ is the proportional weight
* $K_i$ is the integral weight
* $K_d$ is the derivative weight
* $\tau_i$ is the integral time constant
* $\tau_d$ is the derivative time constant



**Transfer function representation of the PID control law**

$$ \frac{U(s)}{E(s)} = ( K_p + \frac{K_i}{s} + K_d s ) $$

where $U(s)$ is the Laplace transform of $u(t)$ and $E(s)$ is the Laplace transform of $e(t)$.

**Intuition behind the role of different components**

* **P (Proportional)** - In many simple situations, proportional control ensures that the desired state (set point) is asymptotically reached from the initial state. It deals with the knowledge of the present error.

* **I (Integral)** - Integral control takes into account the past errors and their duration of persistence, thus it plays an important role in reducing steady-state error in many situations. It deals with the knowledge of past errors.

* **D (Derivative)** - Derivative control takes into account the rate at which the error is decreasing, thus it plays an important role in preventing possible overshoot due to the integrator. It deals with the knowledge of future error change.

**Drawbacks of P**

* In the discrete-time version, the state can oscillate around a certain mean state which isn’t the set point.
* Not suitable in cases where a certain state needs to be maintained and external forces like gravity are present irrespective of the error.

**Drawbacks of I**
* The possibility of overshooting the desired state is high.
* In the case of actuator saturation, integral wind-up can occur.

**Drawbacks of D**
* High-frequency noise can make the derivative contribution higher than required.

To know more about the features of PID Control and get more intuition, check out 

[Understanding PID Control — MATLAB Tech Talk](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)

If you wish to know more about the application of PID Controllers in simple scenarios such as car speed regulation, refer to the following links - 

[PID Math Demystified](https://www.youtube.com/watch?v=JEpWlTl95Tw)

[Simple Examples of PID Control](https://www.youtube.com/watch?v=XfAt6hNV8XM)

Watch this video to see a physical demonstration of a PID Controller

[Hardware Demo of a PID Controller](https://www.youtube.com/watch?v=fusr9eTceEo)



## Discrete-time PID Control

 
$$ u(t) = K_p( e(k) + \frac{1}{\tau_i}\sum_{i=0}^k e(k)\Delta t + \tau_d \frac{(e(k) - e(k-1))}{\Delta t} $$

The discrete form of the PID controller is used when sampling frequency ( $\frac{1}{\Delta t}$) is much lower compared to the speed of dynamics of the system.
This is also one of the forms of PID Control used in most practical situations, especially in programs (spoiler alert!).



## Step response of a PID Controlled System

Depending on the values of $K_p,K_i,K_d$ , different output responses can be obtained in a PID Controlled system. 

Consider an example in which the angular velocity of an object is controlled such that it reaches a certain desired angle (3 radians in this case)
The step input is the desired angle, the control command is the angular velocity and the output is the angle. 
The below figures show some of the different possible step responses for different weight sets.
The different responses show the importance of selecting appropriate weights for the different components of the controller. Looking at the step response is one of the ways to begin tuning the PID Controller so that it behaves in the desired manner.

When $ K_p = 0.5, K_i = 2, K_d = 2 $ 
![image-2.png](attachment:image-2.png)



When $K_p = 5, K_i = 1, K_d = 2$
![image-3.png](attachment:image-3.png)
 

However, the question remains. How do we choose appropriate weights so that the controller behaves as desired? This brings us to the idea of **PID Tuning**, which will be discussed next week.

In the meantime, here is another great example where PID Control is utilized in a spring-mass-damper system and the step responses in various cases are analyzed. You shall also get a sneak peek into PID Tuning as well.

[Introduction: PID Controller Design](https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID)

To know more about step responses in general and the various features used for control design, check out

[The Step Response](https://www.youtube.com/watch?v=USH75nuHV6w)



## References and Additional Materials

YouTube links-

[Control Theory Boot camp — Steve Brunton](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)

[Underactuated Robotics 2020 - Russ Tedrake](https://www.youtube.com/playlist?list=PLkx8KyIQkMfX1WpWYqtep7TOmboZeDtev)

[Classical Control Theory — Brian Douglas](https://www.youtube.com/playlist?list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk)

