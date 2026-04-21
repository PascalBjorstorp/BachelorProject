# MPC Thesis Section Outline

This document provides a full section structure for a bachelor thesis chapter or major section about Model Predictive Control (MPC). The outline is written for a technical engineering thesis and is tailored to an autonomous vehicle or racing-controller project.

You do not need to use all text exactly as written, but this structure covers the main elements that an examiner would usually expect.

---

## 1. Introduction to MPC

### 1.1 Purpose of the Section
- State that this section introduces the Model Predictive Control approach used in the project.
- Explain that the goal is to describe both the theory behind MPC and the specific formulation implemented in the system.

### 1.2 Basic Idea of MPC
- Explain that MPC predicts future system behavior over a finite horizon.
- Explain that it solves an optimization problem at each control step.
- Explain that only the first input is applied.
- Explain that the optimization is repeated at the next sampling instant.

### 1.3 General Benefits of MPC
- Ability to handle constraints explicitly.
- Ability to optimize several objectives at the same time.
- Predictive behavior compared to reactive controllers.
- Suitability for multi-variable dynamic systems.

---

## 2. Why MPC Was Chosen for This Problem

### 2.1 Control Requirements of the Application
- Describe the control task in your project.
- Explain why the controller must consider future path behavior.
- Explain why simple feedback control may be insufficient.

### 2.2 Comparison With Simpler Controllers
- Briefly compare MPC with PID, Pure Pursuit, Stanley, or LQR if relevant.
- Explain what those methods do well.
- Explain their limitations for constrained high-performance driving.

### 2.3 Motivation for MPC in Autonomous Racing
- Track following requires anticipation of upcoming curvature.
- Steering and acceleration are constrained.
- Vehicle states are coupled.
- High-speed behavior benefits from predictive optimization.

---

## 3. System Model Used for Prediction

### 3.1 Role of the Prediction Model
- Explain that MPC needs a model to predict future states.
- State that controller quality depends strongly on model quality.

### 3.2 Choice of Vehicle Model
- State which model is used.
- Examples: kinematic bicycle model, dynamic bicycle model, Frenet-frame vehicle model.
- Explain why this model was selected.

### 3.3 States
- List all states used by the controller.
- Explain the physical meaning of each state.
- If relevant, explain whether the states are in global coordinates, body coordinates, or Frenet coordinates.

Example items to describe:
- lateral error
- heading error
- longitudinal velocity
- lateral velocity
- yaw rate
- steering angle
- progress along path

### 3.4 Control Inputs
- List all control inputs.
- Explain their physical meaning.

Examples:
- steering angle
- steering rate
- longitudinal acceleration
- progress speed or virtual velocity

### 3.5 Assumptions and Simplifications
- Explain what physical effects are neglected or simplified.
- Mention assumptions such as flat road, constant tire parameters, or small-angle approximations if relevant.
- Explain why these assumptions are acceptable for the controller design.

---

## 4. Discretization of the Model

### 4.1 Continuous-Time to Discrete-Time Conversion
- Explain whether the original model is continuous-time.
- Explain why a discrete-time model is needed for digital control.

### 4.2 Discretization Method
- State the method used.
- Examples: Forward Euler, zero-order hold, exact discretization, linearization plus discretization.

### 4.3 Sampling Time
- State the selected sampling time $\Delta t$.
- Explain the tradeoff between accuracy and computation time.

### 4.4 Final Discrete Model
- Present the final discrete-time state update equation.
- For a linear model, this is typically:

$$
x_{k+1} = A x_k + B u_k
$$

- For a nonlinear model, explain the nonlinear state transition function.

---

## 5. Prediction Horizon

### 5.1 Definition of the Horizon
- Define the prediction horizon length $N$.
- Explain that the optimizer predicts from the current state over $N$ future steps.

### 5.2 Horizon Parameters Used in the Project
- State the chosen values for $N$ and $\Delta t$.
- State the total look-ahead time:

$$
T_{\text{horizon}} = N \cdot \Delta t
$$

### 5.3 Tradeoff in Horizon Selection
- Longer horizon gives better foresight.
- Longer horizon increases computational complexity.
- Short horizon may react too locally.

---

## 6. Cost Function / Objective Function

### 6.1 Purpose of the Cost Function
- Explain that the cost function defines the control objective.
- Explain that MPC chooses the control sequence that minimizes this cost.

### 6.2 Tracking Terms
- Describe which tracking errors are penalized.
- Examples:
- contouring or lateral error
- lag or longitudinal path error
- heading error
- velocity tracking error

### 6.3 Control Effort Terms
- Explain that large control actions are penalized.
- State why this is useful for realistic and smooth behavior.

### 6.4 Control Rate Terms
- Explain penalties on input changes.
- State why this improves smoothness and actuator realism.

### 6.5 Terminal Cost
- Explain whether a terminal cost is used.
- Explain what role it plays in the optimization.

### 6.6 Full Mathematical Objective
- Present the full objective function.
- If you use quadratic cost, write it clearly.
- Explain each weight and what behavior it promotes.

Example topics to describe:
- large tracking weights improve path following
- large steering penalties reduce aggressive turning
- large rate penalties reduce oscillations
- terminal weights influence end-of-horizon behavior

---

## 7. Constraints

### 7.1 Importance of Constraints in MPC
- Explain that one major advantage of MPC is explicit constraint handling.

### 7.2 Input Constraints
- Steering angle bounds.
- Steering rate bounds.
- Acceleration or braking limits.

### 7.3 State Constraints
- Velocity bounds.
- Lateral deviation limits.
- Progress-rate bounds if applicable.

### 7.4 Track or Path Constraints
- Explain how the vehicle is kept inside the track corridor.
- If using left and right bounds, explain how they are represented.

### 7.5 Obstacle Constraints
- If obstacles are included, explain how obstacle avoidance is represented.
- If not used, state that clearly.

### 7.6 Physical Interpretation of Constraints
- Explain why each constraint is needed from a vehicle or safety perspective.

---

## 8. Optimization Problem Formulation

### 8.1 Full MPC Problem Statement
- Write the optimization problem in standard mathematical form.
- Include objective, dynamics, constraints, and initial condition.

Typical structure:

$$
\min_{u_0, \dots, u_{N-1}} J
$$

subject to:

$$
x_{k+1} = f(x_k, u_k)
$$

$$
x_k \in \mathcal{X}, \quad u_k \in \mathcal{U}
$$

$$
x_0 = x_{\text{measured}}
$$

### 8.2 Linear or Nonlinear Formulation
- State whether the problem is linear MPC, nonlinear MPC, or a linearized MPC.
- Explain how that choice affects computation and performance.

### 8.3 Convexity and Solvability
- Briefly state whether the optimization is convex or not.
- Explain what that means for solver reliability and runtime.

---

## 9. Receding Horizon Principle

### 9.1 Online Operation
- Explain that the optimization is solved repeatedly at every control step.
- Explain that only the first input is used.

### 9.2 Feedback Nature of MPC
- Explain that MPC is not open-loop because the optimization is repeated with updated state measurements.

### 9.3 Practical Importance
- Explain why this makes MPC robust to disturbances and model mismatch compared to a one-time open-loop plan.

---

## 10. Solver and Numerical Implementation

### 10.1 Solver Type
- State what solver is used in practice.
- Examples: quadratic programming solver, ADMM-based method, Riccati-based solver, nonlinear optimization routine.

### 10.2 Why This Solver Was Chosen
- Explain why it fits your formulation.
- Explain runtime advantages or implementation reasons.

### 10.3 Real-Time Feasibility
- State controller frequency.
- State available computation time per control step.
- Explain whether the optimization fits in that budget.

### 10.4 Embedded or Hardware Considerations
- If relevant, explain real-time deployment constraints.
- If FPGA, GPU, or embedded CPU acceleration is relevant, mention it here.

### 10.5 Numerical Stability and Convergence
- Briefly discuss convergence behavior.
- Mention tolerances, iterations, warm starts, or failure handling if relevant.

---

## 11. Tuning of MPC Parameters and Weights

### 11.1 Why Tuning Is Necessary
- Explain that MPC performance depends strongly on weight selection and horizon settings.

### 11.2 Parameters That Were Tuned
- Tracking weights.
- control-effort weights.
- control-rate weights.
- terminal weights.
- horizon and sampling time if relevant.

### 11.3 Tuning Procedure
- Explain how tuning was performed.
- Examples:
- manual iteration
- parameter sweeps
- simulation-based optimization
- comparison of different scenarios

### 11.4 Observed Tradeoffs
- Higher progress reward may increase speed but reduce safety margin.
- Higher contouring weight improves tracking but can reduce aggressiveness.
- Higher steering penalty smooths behavior but may limit path-following authority.

### 11.5 Final Selected Parameters
- Present the final tuning values used in your final configuration.
- Explain briefly why those values were accepted.

---

## 12. Integration Into the Overall System

### 12.1 Inputs to the Controller
- State what measurements or estimates the MPC receives.
- Examples:
- vehicle pose
- velocity estimate
- reference trajectory
- track boundaries

### 12.2 Outputs of the Controller
- State what commands the MPC produces.
- Examples:
- steering command
- acceleration command
- velocity command

### 12.3 Control Loop Timing
- State how often the controller runs.
- Explain how this relates to the model timestep and prediction horizon.

### 12.4 Interface With Other Modules
- Explain how MPC interacts with localization, planning, and actuation.
- This is especially useful in a full autonomous racing stack.

---

## 13. Advantages and Limitations

### 13.1 Advantages
- Explicit constraint handling.
- Predictive behavior.
- Good performance for coupled multivariable systems.
- Natural way to balance competing objectives.

### 13.2 Limitations
- High computational cost compared with simpler controllers.
- Sensitivity to model mismatch.
- Sensitivity to parameter tuning.
- Dependence on accurate state estimation.

### 13.3 Limitations in This Specific Project
- Mention the most relevant practical limitations from your implementation.
- Examples:
- approximation errors in the vehicle model
- limited real-time compute budget
- imperfect track-bound or reference data
- solver convergence issues in aggressive scenarios

---

## 14. Summary of the MPC Section

### 14.1 What Was Presented
- Briefly summarize the controller model, cost, constraints, and solver.

### 14.2 Main Design Rationale
- Explain in one short paragraph why this MPC structure is appropriate for the application.

### 14.3 Link to the Next Section
- End by connecting the MPC theory section to implementation, experiments, or results.

Examples:
- simulation evaluation
- real-car testing
- comparison with alternative controllers

---

## Short Recommendation for Writing Style

- Start each major section with the engineering purpose before giving equations.
- Use equations for the formal controller definition, not for everything.
- After each equation, explain the physical meaning in words.
- Keep the section focused on your implemented MPC, not only textbook MPC.
- If you include symbols, add a notation table if the chapter becomes dense.

---

## Minimum Version If You Need to Shorten It Later

If you later need a shorter version, keep these as the essential parts:

1. Introduction to MPC
2. Why MPC was chosen
3. Vehicle model
4. Discrete-time formulation
5. Objective function
6. Constraints
7. Optimization problem
8. Solver and implementation
9. Tuning
10. Limitations

---

## Recommended Thesis Headings You Can Reuse Directly

1. Introduction to Model Predictive Control
2. Motivation for Using MPC in Autonomous Racing
3. Vehicle Model for Prediction
4. Discrete-Time MPC Formulation
5. Prediction Horizon
6. Objective Function
7. Constraints
8. Optimization Problem
9. Receding Horizon Strategy
10. Numerical Solver and Real-Time Implementation
11. Parameter Tuning
12. System Integration
13. Advantages and Limitations
14. Summary