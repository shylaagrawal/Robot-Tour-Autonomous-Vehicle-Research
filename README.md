# Engineering Adaptation: Solving the Problem of Inherent Imperfection in Autonomous Robotics
*Shyla Agrawal 2025*

**Abstract** - This technical paper details the journey of designing and implementing an Autonomous Mobile Robot (AMR) for Science Olympiad’s "Robot Tour" event, focusing on overcoming fundamental hardware imperfections through adaptive software. Initially, the project was plagued by persistent, non-linear drift due to minuscule variances between identical drive motors. After several unsuccessful attempts at mechanical and simple odometry correction (including compass and gyrometer feedback), the solution shifted to a predictive, model-based control system. The final implemented methodology, based on the principles of the Path Pursuit algorithm, successfully mitigated accumulated errors by dynamically calculating trajectory adjustments, achieving reliable straight-line navigation in a tight time window. The work concludes that in mechatronic systems, perfection is impossible, and the true challenge lies in designing robust, adaptive software to manage and compensate for inherent hardware imperfections.

## 1. Introduction and The Robot Tour Challenge
The Science Olympiad event, "Robot Tour," presents a constrained optimization problem: design an autonomous robot to navigate a defined grid, score the maximum possible points by hitting targets, and complete the sequence within a tight time limit. Success is dictated by the ability to execute a pre-determined, high-score path with exceptional precision and speed. The core requirements demand highly consistent movement, as every millimeter of positional drift and every fraction of a second directly impacts the final score.
My initial hypothesis centered on optimizing the hardware. The robot utilized a basic differential drive configuration, where two motorized wheels are used for movement and steering. The expectation was that by meticulously designing the chassis, power system, and sensor suite, perfect open-loop motion could be achieved. This premise proved flawed, leading to a profound realization about the nature of mechanical systems.

## 2. The Problem of Inherent Imperfection

<img width="400.4" height="321.2" alt="image" src="https://github.com/user-attachments/assets/9dd7e4aa-a97e-4a68-a4ba-7e696836be38" />
**FIGURE 2.1: Initial Differential Drive Robot Design**

### 2.1. Diagnosis of Non-Linear Drift
Despite weeks spent on mechanical tuning and calibration, the robot consistently exhibited a slight, yet critical, translational drift. It would begin perfectly aligned, traverse a short distance, and then veer subtly to one side. Over the distance of the competition grid, this error compounded, making the trajectory unreliable.
Initial efforts to isolate a mechanical cause included testing various caster wheel types (ball and vertical), meticulous weight distribution adjustments, and even cleaning internal motor components. These efforts confirmed that the issue was not structural or environmental.

### 2.2. The Motor Imbalance Realization
The root cause was traced to an inherent imperfection between the two drive motors: one drew a fraction more voltage or possessed slightly less internal resistance, causing it to spin at a microscopically different rate. This differential rotation, amplified over distance, caused the chronic drift. The realization was that absolute mechanical perfection was an engineering impossibility, demanding an adaptive, closed-loop control system.

## 3. Iterative Control Systems: The Cost of Simple Odometry
The failure to achieve mechanical perfection necessitated a shift to software-based control, using odometry and sensor feedback to manage the course.

### 3.1. Failed Attempt 1: Compass-Based Correction (Magnetic Field Interference)

<img width="254.4" height="288.4" alt="image" src="https://github.com/user-attachments/assets/d8f74a90-0253-4d87-8b86-28a54fb69299" />

**FIGURE 3.1: Compass-Based Correction Methodology**

This system attempted constant angular corrections using a digital compass. However, the electric current in the motor circuits created a local magnetic field, corrupting the compass readings and leading to inaccurate, self-sabotaging corrections.

Below is the code which measured and implemented the compass values:
```ts
// Corrects heading drift using the median of compass readings
function correctHeading(): void {
    const targetHeading = 0
    const currentHeading = medianCompass(5)
    const correction = normalizeAngle(targetHeading - currentHeading)

    if (correction !== 0) {
        if (correction <= 180) {
            finch.setTurn(RLDir.Right, correction, turnSpeed)
        } else {
            finch.setTurn(RLDir.Left, 360 - correction, turnSpeed)
        }
    }
}

// Returns the median of multiple compass samples to reduce noise
function medianCompass(samples: number): number {
    let readings: number[] = []

    for (let i = 0; i < samples; i++) {
        readings.push(finch.getFinchCompass())
    }

    readings.sort((a, b) => a - b)
    const mid = Math.floor(readings.length / 2)

    return readings.length % 2 === 0
        ? (readings[mid - 1] + readings[mid]) / 2
        : readings[mid]
}

// Normalizes angle to [0, 360)
function normalizeAngle(angle: number): number {
    return (angle % 360 + 360) % 360
}

let turnSpeed = 75
```
### 3.2. Failed Attempt 2: Gyrometer Feedback (High Latency)
A second attempt used a gyrometer to measure the rate of angular change and implement a simple proportional control system to adjust wheel speeds during movement. The system failed due to a high delay, causing the robot to constantly over-compensate and enter a state of never-ending oscillations.

### 3.3. Failed Attempt 3: Encoder-Based Correction at Stops (Positional Drift)

<img width="660.8" height="111.2" alt="image" src="https://github.com/user-attachments/assets/a8763622-aff0-43c9-84dc-269a02729f12" />

**FIGURE 3.3: Encoder Correction - Angle Fixed, Position Lost**

This approach used accurate motor encoders to correct the robot's angle at the end of each movement segment, ensuring the robot always faced the correct direction. However, this failed to compensate for the accumulated translational drift that occurred during the segment, resulting in significant positional error.

## 4. Path Pursuit: The Adaptive Control Solution
The fundamental flaw in all previous attempts was the focus on discrete error correction rather than continuous trajectory tracking. The solution required a control algorithm that could dynamically plan a path to a moving target while adjusting for instantaneous velocity errors.

### 4.1. The Path Pursuit Logic
The Path Pursuit (or Pure Pursuit) algorithm transformed the path-following problem into a continuous geometric steering problem. This methodology continuously predicts deviations and adjusts motor power on the move, eliminating the need for constant halts and thereby decreasing the time as well.

<img width="549.6" height="303.6" alt="image" src="https://github.com/user-attachments/assets/3f12d708-b67e-430c-9ffe-be26c09ada20" />

**FIGURE 4.1: Path Pursuit Geometry**

### 4.2. Odometry as a Position Reference
For the Path Pursuit algorithm to function, the robot must continuously estimate its current position and heading. This was accomplished using wheel encoder–based odometry, which infers motion from the rotation of the drive wheels.

Each wheel’s encoder reports how far it has rotated. Using the known wheel diameter and the distance between the wheels, these rotations are converted into linear distance and angular change. The difference in distance traveled by the left and right wheels determines how much the robot has turned, while the average distance represents forward motion. From this information, the robot estimates its change in position over short time intervals.

While encoder-based odometry is known to accumulate error over long distances, it performs reliably over short segments. This makes it well suited for Path Pursuit, which does not rely on a fixed global position. Instead, the algorithm continuously updates its steering based on a near-term target point, allowing small odometry errors to be corrected dynamically rather than accumulated.

Ahmed (2023) shows that for a differential-drive robot, wheel encoder measurements can be converted into reliable estimates of position and orientation by modeling motion as short circular arcs and updating pose using the average wheel displacement and relative heading change. (https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299)

The following implementation computes planar odometry for a differential drive robot and logs position and orientation data used by the Path Pursuit controller:
```ts
// Computes planar odometry using differential drive kinematics
function odometry(): void {
    const ticksPerRevolution = 792
    const wheelDiameter = 5
    const wheelBase = 10

    const leftTicks = finch.getEncoder(RLDir.Left) * ticksPerRevolution
    const rightTicks = finch.getEncoder(RLDir.Right) * ticksPerRevolution

    const leftDistance = Math.PI * wheelDiameter * leftTicks / ticksPerRevolution
    const rightDistance = Math.PI * wheelDiameter * rightTicks / ticksPerRevolution

    const deltaTheta = (rightDistance - leftDistance) / wheelBase
    const distanceTraveled = (leftDistance + rightDistance) / 2

    const x = distanceTraveled * Math.cos(deltaTheta / 2)
    const y = distanceTraveled * Math.sin(deltaTheta / 2)

    datalogger.log(datalogger.createCV("leftEncoder", leftTicks))
    datalogger.log(datalogger.createCV("rightEncoder", rightTicks))
    datalogger.log(datalogger.createCV("leftDistance", leftDistance))
    datalogger.log(datalogger.createCV("rightDistance", rightDistance))
    datalogger.log(datalogger.createCV("angleOfDeviation", deltaTheta * 180 / Math.PI))
    datalogger.log(datalogger.createCV("distanceRobotTraveled", distanceTraveled))
    datalogger.log(datalogger.createCV("xDeviation", x))
    datalogger.log(datalogger.createCV("yDeviation", y))
}
```
### 4.3. Continuous Dynamic Steering
The core of the Path Pursuit algorithm is the continuous calculation of the Lookahead Point (LP), a target point along the desired path a fixed distance ahead. The system then determines the curvature required to steer the robot from its current position to the next point on the Global Path. This curvature is mapped directly to the differential velocity of the two drive motors, resulting in smooth, continuous path correction.

A simple simulation of the Path Pursuit algorithm in context can be found here: https://path-pursuit-sim.w3spaces.com/saved-from-Tryit-2025-12-14.html 

The robot follows a predefined path using a Pure Pursuit controller, which continuously steers toward a nearby “lookahead” point rather than aiming for the final destination directly. At each step, the robot estimates its position and heading using wheel encoders, then computes the curvature needed to smoothly approach the lookahead point. This curvature is translated into left and right wheel speeds, allowing the robot to follow both straight and curved segments.

Standard Pure Pursuit performs poorly when the required turn is very large. To address this, the system detects sharp heading changes and temporarily switches to a turn-in-place mode. In this state, the robot rotates on the spot until its heading aligns with the next path segment, then resumes normal path following. This hybrid approach improves reliability on paths containing reversals or abrupt direction changes while preserving smooth motion elsewhere.

Below is the final code and most updated version of the Path Pursuit algorithm:
```ts
namespace finch {

    /* -------------------- Robot Constants -------------------- */
    const WHEEL_DIAMETER = 5.0        // cm
    const WHEEL_BASE = 10.0           // cm
    const TICKS_PER_ROT = 792
    const CM_PER_TICK = Math.PI * WHEEL_DIAMETER / TICKS_PER_ROT

    const SHARP_TURN_THRESHOLD = 150  // deg
    const TURN_IN_PLACE_SPEED = 30    // motor %
    const TURN_TOLERANCE = 5           // deg

    /* -------------------- Robot Pose -------------------- */
    let x = 0, y = 0, theta = 0        // position (cm) and heading (deg)
    let prevLeftEnc = 0, prevRightEnc = 0

    let turningInPlace = false
    let turnTarget: number | null = null

    /* -------------------- Utility -------------------- */
    const toRad = (d: number) => d * Math.PI / 180
    const toDeg = (r: number) => r * 180 / Math.PI

    function log(msg: string) {
        datalogger.log(datalogger.createCV("Message", msg))
    }

    /* -------------------- Odometry -------------------- */
    function initOdometry(): void {
        x = 0; y = 0; theta = 0
        resetEncoders()
        basic.pause(100)
        prevLeftEnc = getEncoder(RLDir.Left) * TICKS_PER_ROT
        prevRightEnc = getEncoder(RLDir.Right) * TICKS_PER_ROT
    }

    function updateOdometry(): void {
        const left = getEncoder(RLDir.Left) * TICKS_PER_ROT
        const right = getEncoder(RLDir.Right) * TICKS_PER_ROT

        const dL = (left - prevLeftEnc) * CM_PER_TICK
        const dR = (right - prevRightEnc) * CM_PER_TICK

        prevLeftEnc = left
        prevRightEnc = right

        const d = (dL + dR) / 2
        const dTheta = toDeg((dR - dL) / WHEEL_BASE)

        const thetaRad = toRad(theta + dTheta / 2)
        x += d * Math.cos(thetaRad)
        y += d * Math.sin(thetaRad)

        theta += dTheta
        while (theta > 180) theta -= 360
        while (theta <= -180) theta += 360
    }

    /* -------------------- Lookahead & Geometry -------------------- */
    function distance(x1: number, y1: number, x2: number, y2: number): number {
        return Math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    }

    function findLookahead(path: number[][], r: number, start: number): number[] {
        for (let i = start; i < path.length - 1; i++) {
            const [x1, y1] = path[i]
            const [x2, y2] = path[i + 1]

            const dx = x2 - x1, dy = y2 - y1
            const fx = x1 - x, fy = y1 - y

            const a = dx * dx + dy * dy
            const b = 2 * (fx * dx + fy * dy)
            const c = fx * fx + fy * fy - r * r

            const disc = b * b - 4 * a * c
            if (disc < 0) continue

            const t = (-b + Math.sqrt(disc)) / (2 * a)
            if (t >= 0 && t <= 1)
                return [x1 + t * dx, y1 + t * dy]
        }
        return path[path.length - 1]
    }

    function curvatureTo(point: number[], dir: number): number {
        const dx = point[0] - x
        const dy = point[1] - y
        const dist = Math.sqrt(dx * dx + dy * dy)
        if (dist < 0.1) return 0

        let targetAngle = Math.atan2(dy, dx)
        if (dir < 0) targetAngle += Math.PI

        let diff = targetAngle - toRad(theta)
        while (diff > Math.PI) diff -= 2 * Math.PI
        while (diff <= -Math.PI) diff += 2 * Math.PI

        return 2 * Math.sin(diff) / dist
    }

    function wheelSpeeds(curv: number, base: number): number[] {
        let left = base * (1 - curv * WHEEL_BASE / 2)
        let right = base * (1 + curv * WHEEL_BASE / 2)
        return [
            Math.max(-100, Math.min(100, Math.round(left))),
            Math.max(-100, Math.min(100, Math.round(right)))
        ]
    }

    /* -------------------- Pure Pursuit Controller -------------------- */
    export function purePursuit(path: number[][], lookahead: number, speed: number): void {
        initOdometry()
        let index = 0

        while (index < path.length - 1) {
            updateOdometry()

            if (turningInPlace && turnTarget !== null) {
                let err = turnTarget - theta
                while (err > 180) err -= 360
                while (err <= -180) err += 360

                if (Math.abs(err) < TURN_TOLERANCE) {
                    turningInPlace = false
                    stopMotors()
                } else {
                    startMotors(
                        err > 0 ? -TURN_IN_PLACE_SPEED : TURN_IN_PLACE_SPEED,
                        err > 0 ? TURN_IN_PLACE_SPEED : -TURN_IN_PLACE_SPEED
                    )
                }
                basic.pause(50)
                continue
            }

            const look = findLookahead(path, lookahead, index)
            const dx = look[0] - x, dy = look[1] - y
            let headingErr = Math.atan2(dy, dx) - toRad(theta)

            while (headingErr > Math.PI) headingErr -= 2 * Math.PI
            while (headingErr <= -Math.PI) headingErr += 2 * Math.PI

            const dir = Math.abs(headingErr) <= Math.PI / 2 ? 1 : -1

            if (Math.abs(toDeg(headingErr)) > SHARP_TURN_THRESHOLD) {
                turningInPlace = true
                const next = path[index + 1]
                turnTarget = toDeg(Math.atan2(next[1] - y, next[0] - x))
                stopMotors()
                continue
            }

            const curv = curvatureTo(look, dir)
            const speeds = wheelSpeeds(curv, speed * dir)
            startMotors(speeds[0], speeds[1])

            if (distance(x, y, path[index + 1][0], path[index + 1][1]) < lookahead)
                index++

            basic.pause(50)
        }

        stopMotors()
    }
}
```

# 5. Results and Performance
The Path Pursuit implementation successfully traded the search for hardware perfection for the resilience of adaptive software. The data below compares the performance of the most successful prior attempt (Encoder Correction at Stops) against the final Path Pursuit solution over a standard 2-meter drive segment.

### 5.1. Performance Comparison Over 2-Meter Segment

<img width="570.8" height="213.6" alt="image" src="https://github.com/user-attachments/assets/5842033e-84ff-4472-855f-412b182ac856" />
The Path Pursuit algorithm reduced the cumulative positional error by 80% while simultaneously reducing the time required to complete the segment by 35% by eliminating the half-second calibration halt.

## 6. Conclusion
The project demonstrated that in the domain of autonomous robotics, a system's resilience depends not on the perfection of its components, but on the adaptability of its control architecture. The failure of the initial mechanical approach and the limitations of simple error-correction methods (Compass, Gyrometer, and discrete Encoder feedback) highlighted a critical principle of mechatronics: hardware will always be imperfect. The implemented Path Pursuit solution successfully managed this inherent imperfection, providing a robust, fast, and accurate system that placed competitively at the State level.

