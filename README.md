# Autonomous Vehicle Trajectory Tracking using Stanley Controller

## 1.1 Background
[The rapid advancement of Advanced Driver Assistance Systems (ADAS) and autonomous driving technologies has fundamentally transformed modern vehicle engineering. At the core of these autonomous capabilities lies the lateral control and trajectory tracking problem - the ability of a vehicle to accurately and safely follow a predefined reference path under various kinematic conditions. Effective trajectory tracking is essential not only for fully autonomous navigation but also for critical active safety features such as Lane Keeping Assist (LKA) and automated parking systems.
To solve the lateral control problem, various algorithms have been developed, ranging from complex Model Predictive Control (MPC) and AI-based solutions to more computationally efficient geometric controllers. Among these, the Stanley Controller - originally developed by Stanford University for the DARPA Grand Challenge - stands out due to its elegance, robustness, and reliance on fundamental vehicle kinematics. Instead of demanding excessive computational power, it utilizes a Kinematic Bicycle Model to calculate steering commands based on the vehicle's geometric relationship with the target path, making it highly applicable for real-time automotive systems.
]

## 1.2 Objective
[The primary objective of this semester project is to implement, simulate, and quantitatively evaluate the performance of the Stanley Controller for autonomous vehicle trajectory tracking. Rather than relying on complex 3D simulators, the core control logic and vehicle dynamics are built from scratch using a Python-based 2D simulation environment (utilizing NumPy and Matplotlib).
Specific goals of this documentation include:
•	Implementation: Constructing a mathematical vehicle environment based on the Kinematic Bicycle Model to accurately simulate steering and longitudinal motion constraints.
•	Control Execution: Developing the Stanley algorithm to minimize both cross-track error (lateral deviation) and heading error (angular deviation) across a complex trajectory, such as an S-curve.
•	Quantitative Analysis: Tuning the controller's gain parameters k and analyzing its behavioral limits (e.g., overshoot and phase lag) under varying longitudinal velocities. The performance will be evaluated through descriptive statistical metrics, specifically the maximum, mean, and variance of the tracking errors.
By addressing these objectives, this report will demonstrate a comprehensive understanding of the intersection between vehicle dynamics and control theory, proving the viability of geometric controllers in fundamental autonomous driving applications.
]
