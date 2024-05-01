# Simulations 

In this worksheet, we will get you started on simulating a quadrotor. There are two levels of simlations we will be considering: (A) a raw implementation where we write all the equations and dynamics, and (B) an implementation that uses a physics engine like Gazebo.


As you are working on the simulator, think about what is the goal of writing the simulator? In this case, the goal is simply to get started, and try out a few controllers. But often the specific things you want to test in a simulator will guide various design decisions - which language to use? what features to implement? accuracy vs computation speed? discrete time vs continuous time? disturbances? unknown parameters? fidelity or accuracy? all of these design  choices make it challenging to design a single simulation interface that is suitable for all needs. As such, it is likely that you will need to re-implement or modify the simulation interfaces as you go. 

## Setup 

Use any language you want, `julia`, `python`, `matlab`, `c++`, `rust`, whatever you want. The goal is to simulate a quadrotor flying through an arena. Many of the other assignments will assume you have some simulator running. 

## Stage 1: Double integrator

First, we start by ensuring that we can run a simple approximation of the quadrotor - the double integrator. In particular, the goal is to simulate the system and see if it can get to (A) get to a target waypoint, and (B) track a desired trajectory, and (C) optimize its path around a set of obstables. We will be implementing a few different controllers, including a PID controller, a LQR controller, and a MPC controller. 



The goal is to simulate the response of the following 1D double integrator:

$$
\\ddot x = u
$$

where $u$ is the control input, and $x$ is the position system. Normally, we cannot simulate 'higher order systems' like this, i.e. systems where the second derivative appear in the dynamics. We must reduce it to a first order system, as follows:

$$
\\begin{aligned}
\\dot x_1 &= x_2 \\\\ 
\\dot x_2 &= u
\\end{aligned}
$$


Lets start with the following controller:

$$
u = -k_1 (x_1 - x_g) - k_2 x_2
$$

which for $ k_1 = 1, k_2 = 1 $ will drive the system towards the equilibrium state $x_1 = x_g, x_2 = 0$, i.e., to a desired goal location $x_g$.  



Look up the documentation for the diffeq library you are using: how would you setup the simulation? 

Here is some starter code for running this in `julia`. The code should be fairly self explanatory:

```julia

using DifferentialEquations

# here is the controller
function controller(state, time, params)

  x1 = state[1]
  x2 = state[2]

  k1 = params[1]
  k2 = params[2]
  xg = params[3]

  u = ...

  return u

end

# this is the closed-loop dynamics
function dynamics(state, params, time)

  x1, x2 = state

  u = controller(state, time, params)

  x1dot = ...
  x2dot = ...

  return [x1dot, x2dot]

end

# setup the simulation
tspan = (0.0, 2.0)
k1 = ...
k2 = ...
xg = ...
params = (k1, k2, xg) # define the parameters

x0 = randn(2) # randomized initial state

prob = ODEProblem(dynamics, x0, tspan, params)

# solve the problem
# look up the documentation to see what parameters you can change
sol = solve(prob, Tsit5()) 

# plot the solution
using Plots

# try other plots to see what the solution looks like
plot(sol)

```

**Questions:**
1. What happens as you tweak the controller gains?
2. What are the best controller gains to minimize overshoot? To minimize rise time? 
3. What happens if you add a disturbance to the dynamics, for example if you add a `randn()` disturbance in `dynamics`?
4. Try making `xg = sin(3 * time)` instead of a constant - how well does your controller track the solution? Can you change the controller such that it can achieve perfect tracking?
5. Try changing the solver from `Tsit5()` to other choices - do you see a difference?


**Extensions:**
1. This is a PD controller, since we have a proportional gain $k_1$, and a derivative gain $k_2$. Introduce a third gain, $k_3$ for the integral term, and update your code to run a PID controller. 
2. Try replacing the controller with an LQR controller, either derived by hand or perhaps using the [`ControlSystemsBase.lqr` function](https://juliacontrol.github.io/ControlSystems.jl/stable/lib/synthesis/)? What are the similarities and differences?
3. Take a look at the documentation for the [Stochastic Differential Equations](https://docs.sciml.ai/DiffEqDocs/stable/tutorials/sde_example/) - try to see how robust your implementations are to noise.


## State 2: Quadrotors

The previous example was relatively simple, because it was a linear system. Now consider implementing the full 3D nonlinear quadrotor. If you want a simpler 'quadrotor' consider using the [planar quadrotor](https://underactuated.mit.edu/acrobot.html#section3).

Take a look at [this paper](https://arxiv.org/pdf/1003.2005): 
```bibtex
@inproceedings{lee2010geometric,
  title={Geometric tracking control of a quadrotor UAV on SE (3)},
  author={Lee, Taeyoung and Leok, Melvin and McClamroch, N Harris},
  booktitle={49th IEEE conference on decision and control (CDC)},
  pages={5420--5425},
  year={2010},
  organization={IEEE}
}
```

This paper has become a go-to on the design of controllers for quadrotors, since it proposes a controller and proves that it is almost globally stablizing. We will try to implement this paper. The maths is interesting, but it comes down to implementing the dynamics in (Eq. 2-5):

$$
\\begin{aligned}
\\dot x &= v\\\\
\\dot v &= \\frac{1}{m} \\left( m g e_3 - f R e_3 \\right)\\\\
\\dot R &= R \\hat{\\Omega} \\\\
\\dot \Omega &= J^{-1} \left ( M - \\Omega \\times J \\Omega \right )
\\end{aligned}
$$

where the control inputs are force $f$ (a scalar) and the torque $M$ (a three-vector). Naturally, in a quadrotor one must control the angular speed of the four motors, and in (Eq. 1) Lee shows the conversion that is commonly used in practice. 

Question: What approximations did Lee make in deriving these dynamics equations for a quadrotor? 


Now try implementing a controller. The basic form is the same:

```julia


using DifferentialEquations, Plots

function controller(state, params, time)
  
  x = state[1:3]
  v = state[4:6]
  ...

  # Lee's Geometric controller?
  # cascaded PID?
  # RL based controller?

  f = ...
  M = ...

  return [f; M]

end

function dynamics(state, params, time)

  x = state[1:3]
  v = state[4:6]
  ...

  u = controller(state, params, time)
  f = u[1]
  M = u[3]


  xdot = v
  vdot = ...
  Rdot = ...
  Ωdot = ...

  return [xdot; vdot; Rdot; Ωdot]

end

tspan = ...
state0 = ...
params = ...
prob = ODEProblem(...)

sol = solve(prob, ...)

plot(sol)

```

**Hints:**
- Look at [ComponentArrays](https://github.com/jonniedie/ComponentArrays.jl) if you are lazy to write the destructuring and restructuring code
- The main equations you need to implement the controller are all in Section V of the paper. Proposition 3 proves that it is a stabilizing controller. 
- Dev's implementation of this controller is [here](https://github.com/dev10110/RoboticSystems.jl/blob/cbfafe4263306d6eae55b673eac37d186084a84e/src/quadrotors/controllers.jl#L180-L205)

**Tasks**:
- Use the quadrotor parameters listed in Lee's paper, and see what the simulation results look like. What happens if you change the tuning of the controller?
- Modify your code to accept a trajectory to be tracked instead of a single setpoint. What changes? Can you get perfect tracking (hint: you should be able to)
- Try some really aggressive trajectories - can it still track them?
- What happens if there is a mismatch between the quadrotor mass used in the `dynamics` function and the quadrotor mass used in the `controller` function?
- Modify the code to include an integral error term to account for the mass mismatch. 
- Benchmark your code. Can you make it faster if you choose a [different implementation](https://docs.sciml.ai/DiffEqDocs/stable/tutorials/faster_ode_example/)? 
- Find another paper on quadrotor control design. Implement it and compare the performance of the controllers. 
- Add in a wind source, and see what effect this has on the behaviour of the controllers.
- Try to implement a CBF based barrier to prevent the quadrotor (or double integrator) from passing through a wall that you have defined. 
- Try different controllers for the quadrotor - perhaps linearize about an equilibrium point and use the linearized controller. Very often people use Euler angles instead of quaternions or rotation matrices. What does your controller look like then?


## A tangent: Writing your own simulators

Now it is time to understand a bit more about the numerical implemetation of the diffeq libraries. Understanding this is critical to running good sims in the future. Try implemeting a diffeq solver that uses a similar syntax to whatever libraryyou used for the above examples. Implement a `ForwardEuler()` scheme and a `RungeKutta4()` scheme. Here are some resources to get you started.
- https://web.stanford.edu/class/cs205b/lectures/lecture4.pdf
- https://www.cfm.brown.edu/people/sg/AM35odes.pdf


I personally dont think you should be implementing these yourself. Chris Rackaukas has implemented a remarkably performance library in julia, using a huge number of theoretical and implementation tricks to get the performance good. See for example https://www.youtube.com/watch?v=75SCMIRlNXM. But it is good to know whats happening under the hood for a tool that is so critical to our research. 


## Stage 4: Gazebo Simulations

The next step is to try running these sims using a commercial/opensourced physics engine. The main benefit is that you can very easily start to integrate sensors, comms, disturbances, and all sorts of other challenging features. Continue here... [coming soon]

