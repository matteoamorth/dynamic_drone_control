# Drone Control System Project 🚁

## Introduction 📚

This project demonstrates the outcomes of the Automatic Control course, where the focus was on creating a control system to manage the motion of a drone.

In this project, we assume a simplified version of the drone's dynamics:

\[
\begin{cases}
\dot{e}_z = v_z \\
\dot{v}_z = -g + \frac{f}{m_d} \\
\dot{e}_\psi = \omega_z \\
\dot{\omega}_z = \frac{\tau_C}{I_z}
\end{cases}
\tag{1}
\]

### Variables:
- \( v \): drone velocity
- \( \omega \): angular velocity
- \( e \): error
- \( f \): thrusting force
- \( \tau_C \): control torque
- \( m_d \): drone mass
- \( I_z \): moment of inertia with respect to the \( z \)-axis

### Drone Variables 📊

![Drone Variables](1.jpg)

The state space, output, and input vectors are defined as:

\[
\begin{aligned}
x &= \begin{bmatrix} e_z \\ v_z \\ e_\psi \\ \omega_z \end{bmatrix} &
z &= \begin{bmatrix} e_z \\ e_\psi \end{bmatrix} &
u &= \begin{bmatrix} \hat{f} \\ \tau_C \end{bmatrix}
\end{aligned}
\]

### State Space Variables:
- \( e_z \): altitude error
- \( v_z \): vertical velocity
- \( e_\psi \): heading error
- \( \omega_z \): angular velocity around the vertical axis

### System Control Inputs:
- \( \hat{f} \): modified thrust force
- \( \tau_C \): control torque

## State Space Structure ⚙️

The system representation is:

\[
\left\{
\begin{array}{l}
\dot{x} = A \cdot x + B \cdot u \\
z = C \cdot x + D \cdot u
\end{array}
\right.
\]

### System Matrices:
\[
\begin{array}{@{}cc@{}}
A = \begin{bmatrix}
    0 & 1 & 0 & 0 \\
    0 & 0 & 0 & 0 \\
    0 & 0 & 0 & 1 \\
    0 & 0 & 0 & 0
\end{bmatrix} &
B = \begin{bmatrix}
    0 & 0 \\
    \frac{1}{m_d} & 0 \\
    0 & 0 \\
    0 & \frac{1}{I_z} \\
\end{bmatrix}
C = \begin{bmatrix}
    1 & 0 & 0 & 0 \\
    0 & 0 & 1 & 0
\end{bmatrix}
D = \begin{bmatrix}
    0 & 0 \\
    0 & 0
\end{bmatrix}
\end{array}
\]

To check the controllability of the system, an evaluation was performed with MatLab. Since the rank of \( A \) matches its size, the system is deemed controllable.

## Full-state Static Feedback Control 🎮

A feedback control was implemented with:

\[
u = K \cdot x
\]

Where \( K \) is the gain matrix. The control has a convergence rate of \( \alpha = 1 \), ensuring stability. The actuator effort magnitude is limited to \( \|K\| \leq \overline{k} = 200 \). The obtained gain matrix is:

\[
K = \begin{bmatrix}
   -2.6734 & -1.7870 & 0 & 0 \\
   0 & 0 & -0.2833 & -0.1670
\end{bmatrix}
\]

### Eigenvalues:
\[
\begin{bmatrix}
-1.787 & -1.787 & -1.669 & -1.669
\end{bmatrix}
\]

## Wind Perturbations Environment 🌬️

In a more realistic environment, wind disturbances interact with the drone. These are represented as an additional force and torque acting on the drone's dynamics. The perturbation vector is defined as:

\[
w = [w_z \quad w_\psi]^T
\]

Two integral terms are incorporated into the controller to counteract perturbations:

\[
\left\{
\begin{array}{l}
\sigma˙z = e_z \\
\sigma˙\psi = e_\psi
\end{array}
\right.
\]

### Dynamics with Perturbations 🌀

With new terms added, the system evolves as:

\[
\left\{
\begin{array}{l}
\dot{x} = A \cdot x + B \cdot u + E \cdot w \\
z = C \cdot x + D \cdot u + F \cdot w
\end{array}
\right.
\]

The state space now includes: \( x = [\sigma_z \quad e_z \quad v_z \quad \sigma_\psi \quad e_\psi \quad \omega_z]^T \), but the output vector remains the same. The new matrices are:

\[
\begin{array}{@{}cc@{}}
A = \begin{bmatrix}
    0 & 1 & 0 & 0 & 0 & 0 \\
    0 & 0 & 1 & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & 1 & 0 \\
    0 & 0 & 0 & 0 & 0 & 1 \\
    0 & 0 & 0 & 0 & 0 & 0 \\
\end{bmatrix}
\end{array}
\]

### New Matrices:

\[
\begin{array}{@{}cc@{}}
B = \begin{bmatrix}
    0 & 0 \\
    0 & 0 \\
    0 & 0 \\
    \frac{1}{m_d} & 0 \\
    0 & 0 \\
    0 & \frac{1}{I_z} \\
\end{bmatrix}
\end{array}
\]

### Optimality Curve 📉

The control problem is first implemented with the \( \mathcal{L}_2 \) gain. By imposing a constraint on \( \overline{k} \) within the interval \( [10, 1400] \), we obtain the optimality curve shown below:

![Optimality Curve](2.jpg)

In the second step, more constraints are applied, but the results show that the problem is infeasible.

## Colleague Solution 🤝

To satisfy the constraint \( \overline{k} = 1200 \), we use Linear Matrix Inequality (LMI) to conclude that the problem is feasible with the following parameters:

\[
K = \begin{bmatrix}
   -122.8601 & -119.1662 & -3.9170 & 0 & 0 & 0 \\
   0 & 0 & 0 & -31.7029 & -29.5815 & -0.9167
\end{bmatrix}
\]

The norm of \( K \):

\[
||K|| = 171.2031
\]

### Eigenvalues:

\[
\begin{bmatrix}
-1.0631 & -3.3854 & -3.3854 & -8.6136 & -8.6136 & -1.1074
\end{bmatrix}
\]

### Stability Check:

\[
\gamma = 0.0960
\]

## System Simulation 🎥

A 15-second simulation was run to validate the design. Initial conditions:

\[
x(0) = \begin{bmatrix} 0 & 1 & 0.1 & 0 & 0.5 & 0 \end{bmatrix}
\]

Perturbations:

\[
w_z = A_z = 2, \quad w_\psi = A_\psi = 0.2
\]

The simulation, run with `ode45()`, shows that the controller design successfully stabilizes the errors \( e_z \) and \( e_\psi \). The errors exponentially converge to zero, demonstrating the controller's effectiveness in guiding the drone to the desired state.

![Simulation Results](3.jpg)
