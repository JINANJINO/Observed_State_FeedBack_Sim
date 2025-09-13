# 📘 Observer-based State Feedback Control

## 1. Introduction
This project demonstrates the design and simulation of a **state-feedback controller with an observer (Luenberger observer)** for a simple **double integrator system**.
The purpose is to illustrate:
- State-space modeling of dynamic systems,
- Pole-placement based state-feedback design,
- State estimation via observer design,
- Verification of the **Separation Principle** through simulation.

---

## 2. Theoretical Background

### 2.1 State-Space Model
A continuous-time linear system can be expressed as:

$$
\dot{x}(t) = A x(t) + B u(t), \quad y(t) = C x(t) + D u(t)
$$

where
- $$x(t) \in \mathbb{R}^n$$ : state vector,
- $$u(t) \in \mathbb{R}^m$$ : input vector,
- $$y(t) \in \mathbb{R}^p$$ : output vector.

For the **double integrator system** used here:

$$
\begin{array}{cc}
    A = \begin{bmatrix} 0 & 1 \\\\ 0 & 0 \end{bmatrix} & B = \begin{bmatrix} 0 \\\\ 1 \end{bmatrix} \\\\
    C = \begin{bmatrix} 1 & 0 \end{bmatrix} & D = 0
\end{array}
$$

---

### 2.2 State Feedback Control
The control law is defined as:
$$
u(t) = -Kx(t)
$$

The feedback gain $$K$$ is determined using **pole placement** so that the closed-loop poles achieve the desired dynamic performance.

---

### 2.3 State Observer (Luenberger Observer)
Since not all states may be measurable, a state observer estimates the state as:
$$\frac{d\hat{x}}{dt} = A \hat{x} + B u + L (y - C \hat{x})$$

where $$L$$ is the observer gain.
The estimation error dynamics, where $e(t) = x(t) - \hat{x}(t)$, are:
$$\dot{e}(t) = (A - LC)e(t)$$

If the eigenvalues of $$(A - LC)$$ are chosen in the left-half plane, the estimation error converges to zero.

---

### 2.4 Separation Principle
The **controller gain $$K$$** and **observer gain $L$** can be designed independently.
The combined system, using the control law $u = -K\hat{x}$, is given by:

$$
\begin{aligned}
\dot{x} &= Ax - BK\hat{x} \\\\
\dot{\hat{x}} &= LCx + (A - BK - LC)\hat{x}
\end{aligned}
$$



The eigenvalues of this augmented system are simply the union of the closed-loop poles (eigenvalues of $A-BK$) and observer poles (eigenvalues of $A-LC$).

---

## 3. System Design

- Desired controller poles: $$[-1, -1]$$
- Desired observer poles: $$[-2, -2]$$

### MATLAB implementation:
```matlab
% System Matrices
A = [0 1; 0 0];
B = [0; 1];
C = [1 0]; 
D = 0;

% Desired pole locations
controller_poles = [-1 -1];
observer_poles = [-2 -2];

% Gain Calculation
K = acker(A, B, controller_poles);      % State feedback gain
L = acker(A', C', observer_poles)';    % Observer gain

% Augmented System (Controller + Observer)
% State vector: z = [x; x_hat]
A_aug = [A, -B*K; L*C, A-B*K-L*C];
B_aug = [B; B]; % Input matrix for reference tracking
C_aug = [C, zeros(size(C))]; % Output is based on true state 'x'
D_aug = 0;

h = ss(A_aug, B_aug, C_aug, D_aug);

% Simulation
t = 0:0.01:10;
ref = zeros(size(t)); % Zero reference input (regulator problem)
initial_conditions = [1; 0.5; 0; 0]; % x(0)=[1; 0.5], x_hat(0)=[0; 0]
[y, ts, x] = lsim(h, ref, t, initial_conditions);

```

## 4. Simulation Results

### Simulation Setup
- **Initial Conditions**: $x(0) = [1, \quad 0.5]^T$ and $\hat{x}(0) = [0, \quad 0]^T$
- **Input**: $u(t) = 0$ (Free response for regulation)

### Analysis
The simulation confirms that the observer successfully estimates the true states of the system. The estimation error converges to zero, and the system states are driven to the origin, which validates the stability of both the controller and the observer design.

📊 **Comparison of True vs. Estimated States**


<img width="597" height="473" alt="스크린샷 2025-09-13 144103" src="https://github.com/user-attachments/assets/85297804-9d67-4edc-9121-0275fbafb04c" />


This graph presents the key simulation results that validate the performance and efficacy of the designed **observer-based state-feedback control system**. The analysis clearly demonstrates the following three points:

* **Verification of Observer Performance**
    A clear **initial estimation error** exists at t=0, as the true state ($x(0)$) and the estimated state ($\hat{x}(0)$) are different. As the simulation progresses, the estimated state vector ($\hat{x}(t)$), represented by the dashed line, rapidly **converges** to the true state vector ($x(t)$), the solid line. This indicates that the error dynamics ($\dot{e}=(A-LC)e$), governed by the observer poles at s = -2, are **asymptotically stable**, ensuring the estimation error successfully converges to zero.

* **Verification of Controller Performance**
    As the observer provides an accurate state estimate, the state-feedback controller ($u=-K\hat{x}$) effectively stabilizes the system. Ultimately, all state variables (both true and estimated) converge to zero, demonstrating successful **regulation** to the equilibrium point. This proves that the **closed-loop system** is stable, as dictated by the controller poles at s = -1.

* **Empirical Demonstration of the Separation Principle**
    In conclusion, the simulation shows that the two primary objectives—the observer accurately estimating the true state and the controller using that estimate to stabilize the system—are achieved independently. This result provides an empirical validation of the **Separation Principle**, which states that the controller gain K and the observer gain L can be designed separately.
---

## 5. Conclusion
- This project successfully demonstrates the design of a state-feedback controller with a Luenberger observer for a double integrator system.
- The simulation results confirm that the observer can accurately estimate unmeasured states from the system's output.
- The successful independent design of the controller and observer provides a practical verification of the **Separation Principle**.

---

## 6. Future Work
✍️ Possible extensions for this project include:
- **Reference Tracking**: Implementing integral action to eliminate steady-state error for step inputs.
- **Digital Control**: Designing and implementing the controller in a discrete-time domain for digital systems.
- **System Complexity**: Applying the design methodology to higher-order or Multi-Input Multi-Output (MIMO) systems.
