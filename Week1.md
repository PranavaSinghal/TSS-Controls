# Week 1


## Introduction
Have you tried balancing a stick on one end with your palm? It is difficult the first time one tries, but with some practice, one could stabilize the stick around the vertical position more or less for a certain duration of time. One who tries this would certainly agree that the act of balancing a stick requires a great deal of fine control.

The above example is just one of the many instances around us where things need to be controlled. Control refers to the action of trying to bring about desired behaviour/ properties in the state of a system, which is obvious. Many devices which seem to work like a charm require a good amount of control in the background for them to function properly. This is required for various reasons, such as ensuring that unwanted factors like noise or disturbances do not jeopardize their working, ensuring optimal resource usage, and maintaining stability.
If you don’t want your systems to go out of control, this is the place to know a few basic ideas behind controlling.

Anybody who wishes to know more about control theory will certainly refer to this at some point, so here is the link to get started… 

[Control Theory](https://en.wikipedia.org/wiki/Control_theory)

These are motivational videos to get psyched for what’s about to come 

[Why Learn Control Theory](https://www.youtube.com/watch?v=oBc_BHxw78s&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=1)

[Control Theory Overview](https://www.youtube.com/watch?v=Pi7l8mMjYVE&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m&index=2)

With all the enthu you have now, let’s begin…

## Overview
The following broad topics will be covered this week-

*	Control Systems, their types and features
* Model representation using differential equations and transfer functions
*	Stability and Controllability
*	Introduction to PID Control

## Control systems

**Control system**- A collection of components that are collectively responsible for bringing the output of the system to the one desired by the input.
![image](https://user-images.githubusercontent.com/85403032/123793796-cd1b5700-d8ea-11eb-900c-9ad89b1b3a64.png)
The **input** consists of parameters that dictate what the desired output should be. 

The **output** consists of a set of variables that describe the features of the system, collectively referred to as the state of the system. The state of a system is generally represented by 𝑥.

Example— position and velocity of a car.

A control system generally consists of the following components - 

**Plant** — The part of the control system that is being controlled. It consists of the actuator, which executes the control command, and the process which responds to the actuation and undergoes a time evolution. In many cases, the plant could be a dynamic system like a car or a pendulum.

**Controller** — The part of the control system that provides control commands to the plant so that the state gets driven to the desired one. The control command is generally represented by 𝑢.

**Sensors** — The part of the control system that takes an observation/measurement of the state completely or partially. The sensor observation is generally represented by 𝑦.

## Types of control systems

**Open Loop control systems**
Applying control commands without taking any measurements of the output
![image](https://user-images.githubusercontent.com/85403032/123794143-33a07500-d8eb-11eb-87ca-1ce0ae3f3802.png)

**Closed Loop control systems/ Feedback control systems**
Applying control commands based on the measurements of the output with the help of a controller
![image](https://user-images.githubusercontent.com/85403032/123794161-38652900-d8eb-11eb-8051-6053a51a29f6.png)

**Advantages of Closed Loop control** - 
* Greater resistance to noise/bias/disturbances to the system
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

<img src="https://render.githubusercontent.com/render/math?math=\large \dot x = Ax">

In a more general context, systems in which the output depends linearly on the input (satisfies properties of scaling and linear combination) are also described as linear systems.

**Nonlinear systems** — Systems where the evolution of state is not related linearly with the state.
An example is a simple pendulum in a gravitational field.

<img src="https://render.githubusercontent.com/render/math?math=\large \dot x = f(x)">

Many nonlinear systems can be treated as linear systems approximately around certain states
(called fixed points, where 
<img src="https://render.githubusercontent.com/render/math?math=\large \dot x = 0">), through **local linearization**, which will be explored in the upcoming weeks.

Check out the following to know more about Linear systems in different contexts-

[Linear Systems](https://www.youtube.com/watch?v=nyqJJdhReiA&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m&index=3)

[LTI](https://www.youtube.com/watch?v=3eDDTFcSC_Y&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=4)

## Continuous and discrete-time systems

**Continuous-time systems** — Systems where the evolution of the state is considered for any time. 

Example - <img src="https://render.githubusercontent.com/render/math?math=\large \dot x = Ax"> 

**Discrete-time systems** — Systems where the evolution of the state is considered only at discrete time steps.

Example -  <img src="https://render.githubusercontent.com/render/math?math=\large x_{k %2B 1} = \tilde Ax_{k}">   (Discrete-time, linear system)

Each time step <img src="https://render.githubusercontent.com/render/math?math=\large k">  is separated by <img src="https://render.githubusercontent.com/render/math?math=\large \Delta t >=\epsilon > 0">

## Model representation

**Impulse response** — Output of a system when subjected to a control command which is an impulse (Dirac delta function) in time. Impulse responses in the time domain are linear and can be scaled and superpositioned (using convolution) to obtain responses to more general inputs.

**Step response** - Output of a system when subjected to a control command which is a step function in time.

The model of a control system/parts of a control system can be represented using
Differential equations or Transfer functions.

* **Differential equations** — They model the relationship between the input and output of the system in the time domain.
* **Transfer function** — Laplace Transform of the impulse response of a system when all initial conditions are set to zero. It models the relation between the input and the output in the frequency domain.
The values of s for which the denominator of the transfer function goes to zero are called **poles** and the values of s for which the numerator of the transfer function goes to zero are called **zeros**.

Consider the example of a spring attached to a wall, subjected to a force (control) <img src="https://render.githubusercontent.com/render/math?math=\large F = u(t)">  .

 <img src="https://render.githubusercontent.com/render/math?math=\large \dot m\ddot x(t) + kx(t) = u(t)"> is the Differential equation representation.

If   <img src="https://render.githubusercontent.com/render/math?math=\large \dot x = f(x)">  (impulse function) and setting  <img src="https://render.githubusercontent.com/render/math?math=\large \dot x = f(x)">  gives
 <img src="https://render.githubusercontent.com/render/math?math=\large \dot x = f(x)"> which is the Transfer function representation.

To know more about Differential equations and Transfer functions, check out the following

[Time and Frequency Domain](https://www.youtube.com/watch?v=noycLIZbK_k&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=4)

[Transfer functions](https://www.youtube.com/watch?v=RJleGwXorUk)

[Laplace Transform and the Transfer function](https://www.youtube.com/watch?v=0mnTByVKqLM)

[Laplace Transform- A Graphical Approach](https://www.youtube.com/watch?v=ZGPtPkTft8g&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk&index=8&t=3s)

[Zeros and Poles of a Transfer function](https://www.youtube.com/watch?v=AZ7_MvANy_Q)













