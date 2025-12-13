namespace finch {
    // ... (existing constants and state variables) ...

    // --- State for Turn-In-Place ---
    let performingSharpTurn = false;
    let sharpTurnTargetHeading: number | null = null;

    // --- Tuning Constants for Turn-In-Place ---
    const SHARP_TURN_ANGLE_THRESHOLD_DEG = 150; // If angleDiff > this, trigger turn-in-place
    const TURN_IN_PLACE_SPEED = 30;           // Motor speed (%) for turning
    const TURN_ANGLE_TOLERANCE_DEG = 5.0;     // How close to target angle to stop turning


    // --- Constants ---
    // IMPORTANT: Verify these values for your specific Finch model and setup!
    const wheelDiameter = 5.0; // cm (example - measure your wheels)
    const wheelTrack = 10.0;  // cm (example - measure distance between wheel centers)
    const ticksPerRotation = 792; // Ticks per full wheel rotation (confirm this!)
    const cmPerTick = (Math.PI * wheelDiameter) / ticksPerRotation; // Calculated conversion factor

    // --- Robot State ---
    let robotX = 0;         // World X coordinate (cm)
    let robotY = 0;         // World Y coordinate (cm)
    let robotTheta = 0;     // Heading in degrees (-180 to 180 or 0 to 360)

    // --- Odometry State Variables ---
    let previousLeftEncoder = 0;
    let previousRightEncoder = 0;

    // --- Helper function for logging ---
    function logMessage(message: string) { // Use string type for clarity
        datalogger.log(datalogger.createCV("Message", message))
    }

    // --- Helper Math Functions ---
    function toRadian(degrees: number): number { return degrees * Math.PI / 180; }
    function toDegrees(radian: number): number { return radian * 180 / Math.PI; }

    // --- Initialization (Example Trigger) ---
    input.onButtonPressed(Button.A, function () {
        let pathWaypoints: number[][] = [];
        // Example path with backward segments
        pathWaypoints = [[0, 0], [50, 0], [0,1]]; // Ends moving backward
        let detailedPath = createPath(pathWaypoints, 5); // Use path interpolation

        datalogger.setColumnTitles("Message"); // Setup datalogger
        startFinch(); // Assumed function to initialize Finch hardware
        purePursuit(detailedPath, 10, 50); // Path, Lookahead(cm), Target Speed(%)
        basic.pause(500); // Pause after completion
        stopMotors();
    });

    /**
     * Initializes Odometry: Resets pose and captures starting encoder values.
     */
    function initializeOdometry(): void {
        robotX = 0;
        robotY = 0;
        robotTheta = 0; // Start facing along the positive X-axis

        resetEncoders(); // Reset hardware encoders
        basic.pause(100); // Short pause to ensure reset completes
        // Read initial values AFTER reset
        previousLeftEncoder = getEncoder(RLDir.Left) * ticksPerRotation;
        previousRightEncoder = getEncoder(RLDir.Right) * ticksPerRotation;
        logMessage(`Odom Init: X=0, Y=0, T=0 | Enc L=${previousLeftEncoder} R=${previousRightEncoder}`);
    }


    /**
     * Updates the robot's estimated pose (X, Y, Theta) using wheel encoder data.
     * ASSUMES getEncoder() returns raw cumulative ticks.
     */
    function updateOdometry(): void {
        // 1. Read current cumulative encoder ticks directly
        const currentLeftEncoder = getEncoder(RLDir.Left) * ticksPerRotation;
        const currentRightEncoder = getEncoder(RLDir.Right) * ticksPerRotation;

        // 2. Calculate the change in ticks since the last update
        const deltaLeftTicks = currentLeftEncoder - previousLeftEncoder;
        const deltaRightTicks = currentRightEncoder - previousRightEncoder;

        // 3. Store current values for the next iteration
        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder;

        // 4. Convert tick deltas to distance in cm for each wheel
        const deltaLeft = deltaLeftTicks * cmPerTick;
        const deltaRight = deltaRightTicks * cmPerTick;

        // 5. Calculate the average distance travelled and change in angle
        const deltaDistance = (deltaLeft + deltaRight) / 2.0;
        let deltaThetaRad = 0;
        if (Math.abs(wheelTrack) > 1e-6) {
            deltaThetaRad = (deltaRight - deltaLeft) / wheelTrack; // In radians
        }
        const deltaThetaDeg = toDegrees(deltaThetaRad);

        // Store initial angle for average calculation
        const initialThetaRad = toRadian(robotTheta);

        // 7. Update heading
        robotTheta += deltaThetaDeg;
        // Normalize angle (e.g., to -180 to 180)
        while (robotTheta > 180) robotTheta -= 360;
        while (robotTheta <= -180) robotTheta += 360;

        // 8. Update position using average angle during the step
        const avgThetaRad = initialThetaRad + (deltaThetaRad / 2.0);
        robotX += deltaDistance * Math.cos(avgThetaRad);
        robotY += deltaDistance * Math.sin(avgThetaRad);

        // logMessage(`Odom: X=${robotX.toFixed(1)}, Y=${robotY.toFixed(1)}, T=${robotTheta.toFixed(1)}`);
    }

    function getDistance(x1: number, y1: number, x2: number, y2: number): number {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    /**
     * Finds the lookahead point on the path segments.
     * Searches forward along the path array from startIndex.
     */
    function getLookaheadPoint(path: number[][], lookaheadDistance: number, startIndex: number): number[] | null {
        let lookaheadTarget: number[] | null = null;
        let bestIntersectionInfo = { point: null as number[] | null, distAlongPath: -1 };
        let p1, p2
        for (let i = startIndex; i < path.length - 1; i++) {
            p1 = path[i];
            p2 = path[i + 1];


            const segmentDx = p2[0] - p1[0];
            const segmentDy = p2[1] - p1[1];
            const segmentLenSq = segmentDx * segmentDx + segmentDy * segmentDy;

            if (segmentLenSq < 1e-6) continue; // Skip zero-length segments

            const robotToP1X = p1[0] - robotX;
            const robotToP1Y = p1[1] - robotY;

            const a = segmentLenSq;
            const b = 2 * (robotToP1X * segmentDx + robotToP1Y * segmentDy);
            const c = robotToP1X * robotToP1X + robotToP1Y * robotToP1Y - lookaheadDistance * lookaheadDistance;
            const discriminant = b * b - 4 * a * c;

            if (discriminant >= 0) {
                const sqrtDiscriminant = Math.sqrt(discriminant);
                const t1 = (-b + sqrtDiscriminant) / (2 * a);
                const t2 = (-b - sqrtDiscriminant) / (2 * a);

                // Check both intersections
                const candidates = [t1, t2];
                for (const t of candidates) {
                    if (t >= 0 && t <= 1) {
                        // Valid intersection on this segment
                        const intersectionPoint = [p1[0] + t * segmentDx, p1[1] + t * segmentDy];
                        // For simplicity, return the first valid intersection found >= startIndex.
                        // A more advanced PP might find all intersections and pick the "best" one.
                        //logMessage(` GLP: Found intersection on segment ${i} at t=${t.toFixed(2)}`);
                        logMessage("robot:(" + Math.roundWithPrecision(robotX, 2) + ":" + Math.roundWithPrecision(robotY, 2) + "); " + "Radius: " + lookaheadDistance + "; P1:(" + Math.roundWithPrecision(p1[0], 2) + ":" + Math.roundWithPrecision(p1[1], 2) + "); P2:(" + Math.roundWithPrecision(p2[0], 2) + ":" + Math.roundWithPrecision(p2[1], 2) + ")");
                        return intersectionPoint;
                    }
                }
            }
        }


        // If no intersection found on subsequent segments, return the last point as fallback
        if (path.length > 0) {
            logMessage(" GLP: No intersection found, targeting last point");
            return path[path.length - 1];
        }
        return null; // Should only happen if path is empty
    }


    /**
     * Calculates curvature needed to reach the lookahead point, adapting for direction.
     * @param lookaheadPoint Target point [x, y]
     * @param direction +1 for forward movement, -1 for backward movement
     * @returns curvature value (positive=left turn forward, negative=right turn forward)
     */
    function calculateCurvature(lookaheadPoint: number[], direction: number): number {
        const dx = lookaheadPoint[0] - robotX;
        const dy = lookaheadPoint[1] - robotY;
        const distanceToTarget = Math.sqrt(dx * dx + dy * dy);

        if (distanceToTarget < 0.1) return 0; // Already at target

        const angleToTargetRad = Math.atan2(dy, dx);
        const robotAngleRad = toRadian(robotTheta);

        let angleDiffRad = 0;
        if (direction > 0) {
            // Forward: Target angle relative to robot's front
            angleDiffRad = angleToTargetRad - robotAngleRad;
        } else {
            // Backward: Target angle relative to robot's REAR
            const robotRearAngleRad = robotAngleRad + Math.PI;
            angleDiffRad = angleToTargetRad - robotRearAngleRad;
        }

        // Normalize angle difference to [-PI, PI]
        while (angleDiffRad > Math.PI) angleDiffRad -= 2 * Math.PI;
        while (angleDiffRad <= -Math.PI) angleDiffRad += 2 * Math.PI;

        // Curvature formula: 2 * sin(angleDiff) / distance
        const curvature = (2 * Math.sin(angleDiffRad)) / distanceToTarget;

        // logMessage(` Curv: dir=${direction}, angleDiff=${toDegrees(angleDiffRad).toFixed(1)}, dist=${distanceToTarget.toFixed(1)}, curv=${curvature.toFixed(3)}`);
        return curvature;
    }

    /**
     * Calculates target wheel speeds based on curvature and a SIGNED base target speed.
     * @param curvature Positive = turn left (forward), Negative = turn right (forward)
     * @param baseSignedTargetSpeed Overall desired base speed (+ve for fwd, -ve for bwd), often related to motor %
     * @returns [leftSpeed, rightSpeed] scaled motor speeds (-100 to 100)
     */
    function calculateWheelSpeeds(curvature: number, baseSignedTargetSpeed: number): number[] {
        const v_base = baseSignedTargetSpeed; // Base speed (+/- indicating direction)
        const L = wheelTrack;      // Wheel track distance

        // Calculate ideal speeds relative to base speed
        let vLeft_ideal = v_base * (1 - curvature * L / 2.0);
        let vRight_ideal = v_base * (1 + curvature * L / 2.0);

        // --- Simple Scaling & Clamping ---
        // Assume baseSignedTargetSpeed is directly proportional to desired motor %
        // (e.g., if targetVelocity=50 passed to purePursuit, baseSignedTargetSpeed is +50 or -50)
        // No additional scaling factor needed here if targetVelocity represents desired motor power %.
        let leftSpeed = Math.round(vLeft_ideal);
        let rightSpeed = Math.round(vRight_ideal);

        // Clamp speeds to the valid motor range [-100, 100]
        leftSpeed = Math.max(-100, Math.min(100, leftSpeed));
        rightSpeed = Math.max(-100, Math.min(100, rightSpeed));

        // logMessage(` Speeds: Vbase=${v_base}, L=${leftSpeed}, R=${rightSpeed}`);
        return [leftSpeed, rightSpeed];
    }

    /**
     * Main Pure Pursuit control loop. Follows a given path.
     * Handles both forward and backward motion.
     * @param path Array of [x, y] waypoints in cm.
     * @param lookahead Lookahead distance in cm.
     * @param targetVelocity Desired base speed (e.g., as motor %, 0-100). Sign indicates direction.
     */
    export function purePursuit(path: number[][], lookahead: number, targetVelocity: number): void {
        if (!path || path.length < 2) {
            logMessage("[Error] Path has insufficient points.");
            return;
        }

        initializeOdometry();
        logMessage("Pure Pursuit started (Handles Sharp Turns via Turn-In-Place).");

        let currentPathIndex = 0;
        performingSharpTurn = false; // Ensure state is reset at start
        sharpTurnTargetHeading = null;

        while (currentPathIndex < path.length - 1) {
            updateOdometry(); // Update X, Y, Theta

            // --- Check if currently executing a sharp turn ---
            if (performingSharpTurn && sharpTurnTargetHeading !== null) {
                logMessage("State: PERFORMING_SHARP_TURN");
                let headingError = sharpTurnTargetHeading - robotTheta;
                // Normalize error -180 to 180
                while (headingError > 180) headingError -= 360;
                while (headingError <= -180) headingError += 360;

                //logMessage(` Turn Progress: Target=${sharpTurnTargetHeading.toFixed(1)}, Current=${robotTheta.toFixed(1)}, Err=${headingError.toFixed(1)}`);

                // Check if turn is complete
                if (Math.abs(headingError) < TURN_ANGLE_TOLERANCE_DEG) {
                    logMessage(" -> Sharp Turn Complete.");
                    performingSharpTurn = false; // Exit turn state
                    sharpTurnTargetHeading = null;
                    stopMotors();
                    basic.pause(100); // Brief pause after turn before resuming
                } else {
                    // Continue turning
                    let left = 0, right = 0;
                    if (headingError > 0) { // Need to increase theta (turn CCW)
                        left = -TURN_IN_PLACE_SPEED; right = TURN_IN_PLACE_SPEED;
                    } else { // Need to decrease theta (turn CW)
                        left = TURN_IN_PLACE_SPEED; right = -TURN_IN_PLACE_SPEED;
                    }
                    startMotors(left, right);
                }
                // Skip normal pursuit logic while turning
                basic.pause(50);
                continue; // Go to next loop iteration
            }

            // --- Normal Operation / Check for Need to Turn ---
            const lookaheadPoint = getLookaheadPoint(path, lookahead, currentPathIndex);

            if (!lookaheadPoint) {
                logMessage("[Error] Could not find lookahead point. Stopping.");
                stopMotors();
                break;
            }

            // Calculate vector and angle to lookahead point
            const vectorToLookaheadX = lookaheadPoint[0] - robotX;
            const vectorToLookaheadY = lookaheadPoint[1] - robotY;
            const angleToLookaheadRad = Math.atan2(vectorToLookaheadY, vectorToLookaheadX);
            const robotAngleRad = toRadian(robotTheta);

            // Calculate simple heading difference to lookahead point
            let headingDifferenceRad = angleToLookaheadRad - robotAngleRad;
            while (headingDifferenceRad > Math.PI) headingDifferenceRad -= 2 * Math.PI;
            while (headingDifferenceRad <= -Math.PI) headingDifferenceRad += 2 * Math.PI;

            // Determine nominal direction (+1 forward, -1 backward) based on lookahead point location
            let direction = (Math.abs(headingDifferenceRad) <= Math.PI / 2) ? 1 : -1;

            // --- Check for Sharp Turn Condition ---
            // Trigger if the lookahead requires a turn greater than the threshold
            if (Math.abs(headingDifferenceRad) > toRadian(SHARP_TURN_ANGLE_THRESHOLD_DEG) && currentPathIndex < path.length - 1) {
                logMessage(`Sharp Turn (> ${SHARP_TURN_ANGLE_THRESHOLD_DEG} deg) Detected! AngleDiff=${toDegrees(headingDifferenceRad)}`);
                performingSharpTurn = true; // Enter turning state

                // Target heading should be the direction of the *next* path segment
                const nextWaypoint = path[currentPathIndex + 1];
                // Use current robot position for angle calculation for robustness
                const angleToNextWaypointRad = Math.atan2(nextWaypoint[1] - robotY, nextWaypoint[0] - robotX);
                sharpTurnTargetHeading = toDegrees(angleToNextWaypointRad);


                // Normalize target heading
                while (sharpTurnTargetHeading > 180) sharpTurnTargetHeading -= 360;
                while (sharpTurnTargetHeading <= -180) sharpTurnTargetHeading += 360;

                logMessage(` Initiating Turn-In-Place to: ${sharpTurnTargetHeading} deg`);
                stopMotors(); // Stop before turning
                basic.pause(200); // Allow motors to stop

                // Skip the rest of the loop and start turning on the next iteration
                basic.pause(50);
                continue;
            }

            // --- If not performing sharp turn, proceed with standard (adapted) Pure Pursuit ---
            logMessage(`Normal Pursuit: Dir=${direction}`);
            // Calculate curvature based on lookahead point and REQUIRED direction
            // Note: The 'direction' here might still be -1 if the turn isn't sharp enough to trigger the override
            const curvature = calculateCurvature(lookaheadPoint, direction);

            // Determine the target speed with direction sign
            const signedTargetSpeed = Math.abs(targetVelocity) * direction;

            // Calculate and set wheel speeds
            const speeds = calculateWheelSpeeds(curvature, signedTargetSpeed);
            startMotors(speeds[0], speeds[1]);

            // --- Path Advancement Logic ---
            const nextWaypointIndex = currentPathIndex + 1;
            const nextWaypoint = path[nextWaypointIndex];
            const distanceToNextWaypoint = getDistance(robotX, robotY, nextWaypoint[0], nextWaypoint[1]);
            //logMessage(` Path Advance Check: Dist to WP ${nextWaypointIndex} = ${distanceToNextWaypoint.toFixed(1)} cm`);

            const advancementThreshold = lookahead; // Adjust threshold if needed
            if (distanceToNextWaypoint < advancementThreshold) {
                currentPathIndex++;
                logMessage(` --> Advanced path index to ${currentPathIndex}`);
            }
            // ---------------------------

            basic.pause(50); // Control loop delay
        } // End while loop

        // --- Loop finished ---
        stopMotors();
        logMessage("Pure Pursuit path segments finished.");
        updateOdometry(); // Final update
        //logMessage(`Final Pose: X=${robotX.toFixed(1)}, Y=${robotY.toFixed(1)}, Theta=${robotTheta.toFixed(1)}`);
    } // End purePursuit function


    // --- IMPORTANT: Ensure these dependent functions are correct ---
    // calculateCurvature(lookaheadPoint: number[], direction: number): number
    // calculateWheelSpeeds(curvature: number, baseSignedTargetSpeed: number): number[]
    // initializeOdometry(): void
    // updateOdometry(): void
    // getLookaheadPoint(path: number[][], lookaheadDistance: number, startIndex: number): number[] | null
    // getDistance(x1: number, y1: number, x2: number, y2: number): number
    // Other helpers (toRadian, toDegrees, logMessage, motor controls etc.)

 // End namespace finch





    /**
     * Interpolates points along a path defined by waypoints.
     * Density roughly corresponds to points per 20cm (adjust as needed).
     */
    export function createPath(waypoints: number[][], density: number = 5): number[][] {
        const interpolatedPath: number[][] = [];
        if (waypoints.length < 2) {
            return waypoints; // Return original if not enough points
        }

        // Always add the very first waypoint
        interpolatedPath.push(waypoints[0]);

        for (let i = 0; i < waypoints.length - 1; i++) {
            const p1 = waypoints[i];
            const p2 = waypoints[i + 1];

            const distance = getDistance(p1[0], p1[1], p2[0], p2[1]);
            // Calculate number of intermediate points based on distance and density
            // Ensure at least one segment even for short distances if density > 0
            const numPoints = Math.max(0, Math.round(density * distance / 20) - 1); // -1 because we add p2 later

            for (let j = 1; j <= numPoints; j++) {
                const t = j / (numPoints + 1); // Interpolation factor
                const interpolatedX = p1[0] + t * (p2[0] - p1[0]);
                const interpolatedY = p1[1] + t * (p2[1] - p1[1]);
                interpolatedPath.push([interpolatedX, interpolatedY]);
            }
            // Add the endpoint of this segment (which is the start of the next, or the final point)
            interpolatedPath.push(p2);
        }
        // Basic duplicate removal in case waypoints were identical
        // Can be made more robust if needed

        const uniquePath: number[][] = []; // Initialize an empty array for unique points
        const seenPoints: { [key: string]: boolean } = {}; // Use an object as a hash set

        for (const point of interpolatedPath) {
            // Create a unique string key for each point (e.g., "x,y")
            const key = `${point[0]},${point[1]}`;

            // Check if we've already added a point with these coordinates
            if (!seenPoints[key]) {
                // If not seen before, add it to the unique path and mark it as seen
                uniquePath.push(point);
                seenPoints[key] = true;
            }
        }
        // 'uniquePath' now holds the path with consecutive duplicates removed
        // logMessage(`Original waypoints: ${waypoints.length}, Interpolated points: ${uniquePath.length}`);
        return uniquePath;
    }


 // End namespace finch

}
