# Engineering Adaptation: Solving the Problem of Inherent Imperfection in Autonomous Robotics
*Shyla Agrawal, NCHS*

## Abstract
This technical paper details the journey of designing and implementing an Autonomous Mobile Robot (AMR) for the Science Olympiad "Robot Tour" event, focusing on overcoming fundamental hardware imperfections through adaptive software. Initially, the project was plagued by persistent, non-linear translational drift due to minuscule variances between identical drive motors. After several unsuccessful attempts at mechanical and simple closed-loop odometry correction (including compass and gyrometer feedback), the solution shifted to a predictive, model-based control system. The final implemented methodology, based on the principles of the Path Pursuit algorithm, successfully mitigated accumulated errors by dynamically calculating trajectory adjustments, achieving reliable straight-line navigation in a tight time window. The work concludes that in mechatronic systems, perfection is impossible, and the true challenge lies in designing robust, adaptive software to manage and compensate for inherent hardware imperfections.

## 1. Introduction and The Robot Tour Challenge
The Science Olympiad event, "Robot Tour," presents a constrained optimization problem: design an autonomous robot to navigate a defined grid, score the maximum possible points by hitting targets, and complete the sequence within a tight time limit. Success is dictated by the ability to execute a pre-determined, high-score path with exceptional precision and speed. The core requirements demand highly consistent movement, as every millimeter of positional drift and every fraction of a second directly impacts the final score.
My initial hypothesis centered on optimizing the hardware. The robot utilized a basic differential drive configuration, where two motorized wheels are used for movement and steering. The expectation was that by meticulously designing the chassis, power system, and sensor suite, **perfect open-loop motion** could be achieved. This premise proved flawed, leading to a profound realization about the nature of mechanical systems.

## 2. The Problem of Inherent Imperfection

<img width="641" height="487" alt="image" src="https://github.com/user-attachments/assets/e79f56ee-a5dd-4030-b2ff-18515674fb5d" />

### 2.1. Diagnosis of Non-Linear Drift
Despite weeks spent on mechanical tuning and calibration, the robot consistently exhibited a slight, yet critical, **translational drift**. It would begin perfectly aligned, traverse a short distance, and then veer subtly to one side. Over the distance of the competition grid, this error compounded, making the trajectory unreliable.
Initial efforts to isolate a **mechanical cause** included testing various **caster wheel types (ball and vertical)**, meticulous **weight distribution adjustments**, and even cleaning internal motor components. These efforts confirmed that the issue was not structural or environmental.

### 2.2. The Motor Imbalance Realization
The root cause was traced to an **inherent imperfection** between the two drive motors: one drew a fraction more voltage or possessed slightly less internal resistance, causing it to spin at a microscopically different rate. This differential rotation, amplified over distance, caused the chronic drift. The realization was that **absolute mechanical perfection was an engineering impossibility**, demanding an adaptive, closed-loop control system.

## 3. Iterative Control Systems: The Cost of Simple Odometry
The failure to achieve mechanical perfection necessitated a shift to software-based control, using odometry and sensor feedback to manage the course.

### 3.1. Failed Attempt 1: Compass-Based Correction (Magnetic Field Interference)
This system attempted constant angular corrections using a digital compass. However, the electric current in the motor circuits created a local magnetic field, **corrupting the compass readings** and leading to inaccurate, self-sabotaging corrections.

<img width="466.5" height="318" alt="image" src="https://github.com/user-attachments/assets/9b5517b2-72a5-4067-8d60-e0e1cd2db6eb" />

### 3.2. Failed Attempt 2: Gyrometer Feedback (High Latency)
A second attempt used a gyrometer to measure the rate of angular change and implement a simple proportional control system to adjust wheel speeds during movement. The system failed due to **high latency**, causing the robot to constantly **over-compensate** and enter a state of never-ending oscillations.

### 3.3. Failed Attempt 3: Encoder-Based Correction at Stops (Positional Drift)
This approach used accurate motor encoders to correct the robot's angle at the end of each movement segment, ensuring the robot always faced the correct direction. However, this failed to compensate for the **accumulated translational drift** that occurred during the segment, resulting in significant positional error.
// C++ snippet for Angular Correction (Executed at stop)
float desired_angle = 90.0; 
// Calculate current angle based on encoder difference
float current_angle_from_encoders = (left_encoder - right_encoder) * ENCODER_FACTOR;
float drift_angle = current_angle_from_encoders - desired_angle;

if (fabs(drift_angle) > 1.0) {
    // Corrects the angle, but does not compensate for horizontal (XY) position error
    robot.rotate(-drift_angle); 
    delay(500); // 0.5 second halt for calibration - a major time inefficiency
}

<img width="631.5" height="246" alt="image" src="https://github.com/user-attachments/assets/62b9c2c8-1c25-40c9-87b6-34c178d03c85" />

## 4. Path Pursuit: The Adaptive Control Solution
The fundamental flaw in all previous attempts was the focus on discrete **error correction** rather than continuous **trajectory tracking**. The solution required a control algorithm that could dynamically plan a path to a moving target while adjusting for instantaneous velocity errors.

### 4.1. The Path Pursuit Logic
The Path Pursuit (or Pure Pursuit) algorithm transformed the path-following problem into a continuous geometric steering problem. This methodology **continuously predicts deviations** and adjusts motor power on the move, eliminating the need for constant halts.

<img width="822" height="321" alt="image" src="https://github.com/user-attachments/assets/2db4230a-32e0-4da2-9334-e102a8d74226" />

### 4.2. Continuous Dynamic Steering
The core of the Path Pursuit algorithm is the continuous calculation of the **Lookahead Point ($\boldsymbol{P_L}$)**—a target point along the desired path a fixed distance ahead. The system then determines the **curvature ($\boldsymbol{\kappa}$)** required to steer the robot from its current position to $\boldsymbol{P_L}$. This curvature is mapped directly to the differential velocity of the two drive motors, resulting in smooth, continuous path correction.
// C++ snippet for Path Pursuit Control Loop (Run every 50ms)
// 1. Find the Lookahead Point (P_L) on the global path
Point lookahead_point = path.getLookaheadPoint(robot.position, LOOKAHEAD_DISTANCE);

// 2. Calculate required curvature (kappa) to hit P_L
float curvature_required = calculateCurvature(robot.position, robot.heading, lookahead_point);

// 3. Map curvature to differential wheel velocities
float v_center = TARGET_SPEED;
float delta_v = curvature_required * TRACK_WIDTH / 2.0;

float v_left = v_center + delta_v;
float v_right = v_center - delta_v;

// 4. Execute movement (continuous and smooth)
robot.set_velocity(v_left, v_right);

## 5. Results and Performance
The Path Pursuit implementation successfully traded the search for hardware perfection for the resilience of adaptive software. The data below compares the performance of the most successful prior attempt (Encoder Correction at Stops) against the final Path Pursuit solution over a standard 2-meter drive segment.

### 5.1. Performance Comparison Over 2-Meter Segment
<img width="666" height="362" alt="image" src="https://github.com/user-attachments/assets/f4b14727-31e6-4011-91f8-47b7637c1c6e" />

The Path Pursuit algorithm reduced the cumulative positional error by 80% while simultaneously reducing the time required to complete the segment by 35% by eliminating the half-second calibration halt.

<img width="661.5" height="364" alt="image" src="https://github.com/user-attachments/assets/8eaa2bdd-cc1e-4f9c-a91d-bb5a759e426b" />

## 6. Conclusion
The project demonstrated that in the domain of autonomous robotics, a system's resilience depends not on the perfection of its components, but on the **adaptability of its control architecture**. The failure of the initial mechanical approach and the limitations of simple error-correction methods (Compass, Gyrometer, and discrete Encoder feedback) highlighted a critical principle of mechatronics: hardware will always be imperfect. The implemented Path Pursuit solution successfully managed this inherent imperfection, providing a robust, fast, and accurate system that placed competitively at the State level.
This work confirms that persistence in adapting to failure will always yield superior results compared to the rigid pursuit of an impossible ideal.



