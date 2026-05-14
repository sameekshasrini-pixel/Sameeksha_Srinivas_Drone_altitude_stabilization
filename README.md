# drone-altitude-stabilization
PID Control System for Dron Altitude Stabilization using MATLAB and Simulink

# Drone Altitude Stabilization Using PID Control

## About the Project

Drones operating in real environments face constant challenges 
from external disturbances like wind, turbulence, and sensor noise. 
Without a proper control system, even a small gust of wind can 
cause the drone to lose altitude and crash.

This project designs a complete altitude control system for a drone 
using a PID (Proportional Integral Derivative) controller. The system 
is mathematically modelled, tuned, simulated, and analysed using 
MATLAB and Simulink.

The goal is to make the drone:
- Reach a target altitude of 1 metre quickly
- Stay there without drifting
- Recover automatically when wind hits it

---

##  Problem Statement

The vertical dynamics of the drone are approximated by the 
following transfer function:

$$G(s) = \frac{1}{s^2 + 2s + 5}$$

Where:
- **Input** = Thrust command from controller
- **Output** = Drone altitude in metres
- **Disturbance** = External wind force at t = 5 seconds

### Design Requirements:
| Requirement | Target Value |
|---|---|
| Overshoot | Less than 10% |
| Settling Time | Less than 3 seconds |
| Steady State Error | Zero |
| Stability | Stable under wind disturbance |

---

##  Dependencies

### Software Required:
| Software | Version | Purpose |
|---|---|---|
| MATLAB | R2023a or later | Main coding environment |
| Simulink | Included with MATLAB | Visual block diagram simulation |

### MATLAB Toolboxes Required:
| Toolbox | Purpose |
|---|---|
| Control System Toolbox | tf(), feedback(), pidtune(), margin() |
| Simulink Control Design | PID block, saturation block |
| Signal Processing Toolbox | Noise simulation |

### How to Check if Toolboxes are Installed:
Type this in MATLAB command window:
```matlab
ver
```
This lists all installed toolboxes.

---

##  Our Approach

### Step 1 — Mathematical Modelling
We started by modelling the drone as a second order transfer 
function G(s). This captures the drone's natural behaviour — 
how it responds to thrust commands based on its physical 
properties like mass and air resistance.

Without any controller, the drone only reaches 0.2 metres 
due to the DC gain of G(s) = 1/5. This confirmed the need 
for a controller.

### Step 2 — Stability Analysis
Before designing the controller we analysed the open loop 
system to understand its natural behaviour:
- Calculated poles at s = -1 ± 2j
- Both poles have negative real parts → system is stable
- Plotted Bode plot to understand frequency response

### Step 3 — PID Controller Design
We chose a PID controller because:
- P term reacts to current altitude error instantly
- I term eliminates long term steady state error completely
- D term prevents overshooting the target altitude
- PID is the industry standard for altitude control systems

MATLAB's pidtune() function was used to automatically calculate 
optimal gains using loop shaping and frequency domain analysis.

### Step 4 — Performance Verification
The closed loop step response was analysed using stepinfo() 
to verify all design specifications were met.

### Step 5 — Robustness Testing
A wind disturbance of magnitude 10 was introduced at t = 5 
seconds using lsim() simulation to verify the controller 
can recover from real world disturbances.

### Step 6 — Stability Margins
Gain margin and phase margin were calculated to confirm the 
system has sufficient robustness against parameter variations 
and uncertainties.

### Step 7 — Simulink Digital Twin
A complete Simulink model was built including:
- Motor saturation to simulate real thrust limits
- Sensor noise to simulate real world measurement errors
This makes the simulation much more realistic than a 
simple transfer function model.

---

## ⚙️ How It Works
### How PID Works:

**Proportional (P):**
Looks at current error and reacts immediately.
Large error → large correction.
Small error → small correction.

**Integral (I):**
Adds up all past errors over time.
If drone has been slightly below target for a long time
the I term builds up force to push it to exactly 1 metre.
This eliminates steady state error completely.

**Derivative (D):**
Looks at how fast the error is changing.
If drone is approaching target too fast
D term applies braking to prevent overshoot.

### Combined PID Formula:
$$u(t) = K_p \cdot e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$$

Where:
- u(t) = control output (thrust command)
- e(t) = error (target altitude - actual altitude)
- Kp = 10.25, Ki = 13.14, Kd = 1.934

---
