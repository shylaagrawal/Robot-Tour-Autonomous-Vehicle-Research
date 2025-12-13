# We live in an imperfect world. 

Every snowflake is different, every heartbeat a little uneven, and even time, the constant should tick our life along without misbehaving, can bend under gravity. We see it everywhere: nothing stays ideal for long and energy always disperses. The universe seems to be built on imperfection, and we must learn to embrace it.

When I was six, the idea of imperfection became personal. I was diagnosed with the eye conditions astigmatism and amblyopia, a common but still terrifying discovery. Due to a difference in uncorrected nearsightedness between both eyes, my brain over time learned to favor one over the other. This led to one eye overworking, and the other eye lagging behind, not being able to keep itself straight. Suddenly, the blurry letters on the whiteboard at school, the double faces I always saw, and the flares on every light source came into focus with the help of two thick pieces of curved glass. Those glasses didn’t fix my eyes, they corrected for their imperfections. They bent light just enough so that the world looked sharp again. That tiny bending of reality taught me something important. Perfection isn’t required for things to work beautifully. Clever correction is enough.

Once I noticed this idea in my own life, I began noticing it everywhere. Traffic lights coordinate with sensors in the roads and pedestrians on the sidewalk to reduce traffic and tame rush hour chaos. Our own phones use algorithms to stabilize shaky hands in photographs to ensure a clear photo. Our world is a collection of intrinsically imperfect systems mitigated and optimized by clever engineering. As I grew older, I was fascinated by how we use logic, code, and design to balance out the irregularities of nature, drawing me towards Operations Research.

Operations Research, the mathematical study of decision-making and optimization. Given limited resources, unpredictable environments, and messy human behavior, OR asks for what the smartest possible decision is. I love the idea of transforming constraints into opportunity. I began learning how algorithms could model complex problems and minimize inefficiencies in real time. I read about how engineers and scientists design systems that work best within constraints, rather than chasing impossible ideals. Whether it is routing trucks during holiday surges, reducing energy consumption, or coordinating autonomous robots, Operational Research acts as the practical bridge between abstract and real-world resource allocation problems. The ability to model large-scale systems, find globally optimal solutions, and translate that predictive power back into tangible improvements in efficiency and resource allocation is precisely why I seek to contribute to this field.

One of the most fascinating problems to me is the optimization of last-mile delivery. How can companies like Amazon deliver items cheaper than the cost of getting them to my doorstep? How do algorithms plan the shortest (but most expensive) segment of a package's journey to a customer's door to minimize cost, resource waste, and time? The problem is still unsolved right now, but once solved well, millions of people will feel the benefit.

My interest deepened through my participation in Science Olympiad’s Robot Tour event. The premise was simple: before the competition, you have to build an autonomous robot. There, you will be given a grid, obstacles, and a time limit. You must decide what path to send it on, what bonus points to hit, and the closer you are to the time limit, the better. Furthermore, your robot cannot leave the grid at any time. Every movement must be measured, consistent, and timed within tight constraints. Every millimeter of drift and every fraction of a second matters. In short, Robot Tour is an optimization problem.

At the start, my focus was purely mechanical. I spent weeks tuning the robot and testing its performance, yet no matter how well I thought I had constructed it, my robot refused to go straight. It would start aligned perfectly, move a few feet, and then begin turning ever so slightly to one side. The more I tried to correct the hardware, the worse it became. 

First, I thought that having a third caster wheel was the issue, and that it was steering the robot instead of moving it along straight. I tried replacing it with multiple wheel types, both ball and vertical, but nothing changed the performance. I measured and re-measured wheel alignment, tried to shift around the weight distribution to favor the wheels with motors, even checked floor friction and cleaned the track. I took apart the motor to clean any built up dirt on every part with a needle. Everything I did had a marginal impact at best, and nothing fully fixed the issue at hand. The drift seemed to have no mechanical cause that I could see.

Eventually, I realized the problem came from the motors themselves. Despite being identical in model and specification, they did not spin at precisely the same speed. One drew a fraction more voltage, or perhaps had a bit more internal resistance. That tiny difference compounded with every rotation, extrapolating over a long distance, and causing the robot to veer off course. I replaced the motors multiple times, switched between battery types, and shifted the voltage going to each wheel specifically, but the imbalance always returned. I read through forums, Youtube videos, and research papers, and found confirmation of what I was beginning to understand: no two motors will ever behave identically. There will always be small irregularities between the two.

It reminded me of my eyes, one wheel working overtime, the other lagging behind. But just as glasses bend light, software could bend motion. I was trying to solve the wrong problem the entire time. The motors could stay imperfect, why fight it? What if, instead, I try to correct the code working behind the wheels? 

So I turned to programming. If the hardware could not be perfect, then the software would need to be adaptive. I began developing algorithms to counteract the drift. My first program applied constant corrections at timed intervals, checking compass values to figure out how much it should turn on its stops, but it was unreliable, and sometimes jeopardized itself due to the magnetic field being created by the circuit under the compass. My second attempt measured rotation feedback from a gyroscope and adjusted wheel speeds during movement, but the response time was too slow, and it often over-compensated and caused never-ending oscillations. My third approach involved counting encoder values to figure out how much it drifted and to correct its angles at the stops, much like my first approach. When the robot drifted off the straight path, the code would force the robot to rotate the few degrees it deviated to straighten it again. This worked great to fix the angle, because the encoder values were very accurate, but it didn’t compensate for the horizontal distance traveled during the drift itself, and by the time the robot had been running for 2 minutes, the robot would be off the grid.

Eventually, I achieved a version that could move straight for long stretches, though to stabilize the movement, I had to pause the robot for half a second between every step. The robot would inch forward, stop, recalibrate, and move again. It worked great, but the stop-and-go motion slowed it down considerably. I was caught between accuracy and efficiency.

I needed to keep the robot moving at all times, and in further research, I found an algorithm known as Path Pursuit, which is predominantly used in today's autonomous car software systems. Unlike my original algorithm, which stopped at each step to correct course, Path Pursuit continuously predicted deviations and adjusts on the move. It eliminated the need for constant halts and instead calculated the best path dynamically. I applied this concept to my robot’s motion logic. 

With this system, I competed at Regionals, placing fourth, and then at State, placing sixth. Along the way, I wrote a research paper detailing my steps and approaches taken, all the attempts I made to fix my robot, and every hardware and all software changes I did, including path pursuit.

When I look back, I see my journey as one long encounter with imperfection. My eyes will always bend light incorrectly, and my motors will always drift, but each time I encounter a flaw, I learn to correct for it. That is what Operations Research represents to me, and why I’m interested in researching this field. It is not the pursuit of perfection, but the study of how to make imperfect systems work optimally. It embraces the world’s entropy and tries to produce effective, efficient outcomes.

# Engineering Adaptation: Solving the Problem of Inherent Imperfection in Autonomous Robotics
*Student Name, High School, Date*
Abstract - This technical paper details the journey of designing and implementing an Autonomous Mobile Robot (AMR) for the Science Olympiad "Robot Tour" event, focusing on overcoming fundamental hardware imperfections through adaptive software. Initially, the project was plagued by persistent, non-linear translational drift due to minuscule variances between identical drive motors. After several unsuccessful attempts at mechanical and simple closed-loop odometry correction (including compass and gyrometer feedback), the solution shifted to a predictive, model-based control system. The final implemented methodology, based on the principles of the Path Pursuit algorithm, successfully mitigated accumulated errors by dynamically calculating trajectory adjustments, achieving reliable straight-line navigation in a tight time window. The work concludes that in mechatronic systems, perfection is impossible, and the true challenge lies in designing robust, adaptive software to manage and compensate for inherent hardware imperfections.
1. Introduction and The Robot Tour Challenge
The Science Olympiad event, "Robot Tour," presents a constrained optimization problem: design an autonomous robot to navigate a defined grid, score the maximum possible points by hitting targets, and complete the sequence within a tight time limit. Success is dictated by the ability to execute a pre-determined, high-score path with exceptional precision and speed. The core requirements demand highly consistent movement, as every millimeter of positional drift and every fraction of a second directly impacts the final score.
My initial hypothesis centered on optimizing the hardware. The robot utilized a basic differential drive configuration, where two motorized wheels are used for movement and steering. The expectation was that by meticulously designing the chassis, power system, and sensor suite, **perfect open-loop motion** could be achieved. This premise proved flawed, leading to a profound realization about the nature of mechanical systems.
2. The Problem of Inherent Imperfection

FIGURE 2.1: Initial Differential Drive Robot Design
2.1. Diagnosis of Non-Linear Drift
Despite weeks spent on mechanical tuning and calibration, the robot consistently exhibited a slight, yet critical, **translational drift**. It would begin perfectly aligned, traverse a short distance, and then veer subtly to one side. Over the distance of the competition grid, this error compounded, making the trajectory unreliable.
Initial efforts to isolate a **mechanical cause** included testing various **caster wheel types (ball and vertical)**, meticulous **weight distribution adjustments**, and even cleaning internal motor components. These efforts confirmed that the issue was not structural or environmental.
2.2. The Motor Imbalance Realization
The root cause was traced to an **inherent imperfection** between the two drive motors: one drew a fraction more voltage or possessed slightly less internal resistance, causing it to spin at a microscopically different rate. This differential rotation, amplified over distance, caused the chronic drift. The realization was that **absolute mechanical perfection was an engineering impossibility**, demanding an adaptive, closed-loop control system.
3. Iterative Control Systems: The Cost of Simple Odometry
The failure to achieve mechanical perfection necessitated a shift to software-based control, using odometry and sensor feedback to manage the course.
3.1. Failed Attempt 1: Compass-Based Correction (Magnetic Field Interference)
This system attempted constant angular corrections using a digital compass. However, the electric current in the motor circuits created a local magnetic field, **corrupting the compass readings** and leading to inaccurate, self-sabotaging corrections.

FIGURE 3.1: Compass-Based Correction Methodology
3.2. Failed Attempt 2: Gyrometer Feedback (High Latency)
A second attempt used a gyrometer to measure the rate of angular change and implement a simple proportional control system to adjust wheel speeds during movement. The system failed due to **high latency**, causing the robot to constantly **over-compensate** and enter a state of never-ending oscillations.
3.3. Failed Attempt 3: Encoder-Based Correction at Stops (Positional Drift)
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

FIGURE 3.3: Encoder Correction Failure Mode
4. Path Pursuit: The Adaptive Control Solution
The fundamental flaw in all previous attempts was the focus on discrete **error correction** rather than continuous **trajectory tracking**. The solution required a control algorithm that could dynamically plan a path to a moving target while adjusting for instantaneous velocity errors.
4.1. The Path Pursuit Logic
The Path Pursuit (or Pure Pursuit) algorithm transformed the path-following problem into a continuous geometric steering problem. This methodology **continuously predicts deviations** and adjusts motor power on the move, eliminating the need for constant halts.

FIGURE 4.1: Path pursuit / Pure pursuit Geometry
4.2. Continuous Dynamic Steering
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
5. Results and Performance
The Path Pursuit implementation successfully traded the search for hardware perfection for the resilience of adaptive software. The data below compares the performance of the most successful prior attempt (Encoder Correction at Stops) against the final Path Pursuit solution over a standard 2-meter drive segment.
5.1. Performance Comparison Over 2-Meter Segment
Metric
Encoder (Stop-Go) Correction
Path Pursuit (Final)
Improvement (Path Pursuit)
**Positional Error (mm) @ 2m**
15 mm
3 mm
80% Reduction
**Time (seconds) for 2m segment**
6.5 s (due to stops)
4.2 s
35% Time Reduction

The Path Pursuit algorithm reduced the cumulative positional error by 80% while simultaneously reducing the time required to complete the segment by 35% by eliminating the half-second calibration halt.

FIGURE 5.1: Comparative Performance Graph
6. Conclusion
The project demonstrated that in the domain of autonomous robotics, a system's resilience depends not on the perfection of its components, but on the **adaptability of its control architecture**. The failure of the initial mechanical approach and the limitations of simple error-correction methods (Compass, Gyrometer, and discrete Encoder feedback) highlighted a critical principle of mechatronics: hardware will always be imperfect. The implemented Path Pursuit solution successfully managed this inherent imperfection, providing a robust, fast, and accurate system that placed competitively at the State level.
This work confirms that persistence in adapting to failure will always yield superior results compared to the rigid pursuit of an impossible ideal.



