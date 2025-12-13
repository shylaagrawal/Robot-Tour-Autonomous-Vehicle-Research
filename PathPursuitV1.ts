

namespace finch {

    

    // --- Constants for Pure Pursuit ---
    const wheelDiameter = 5; // cm (example - adjust if needed)
    const wheelTrack = 10.0;  // cm (example - adjust if needed)
    const ticksPerRotation = 792; // From the provided code
    const cmPerTick = (Math.PI * wheelDiameter) / ticksPerRotation;

    let robotX = 0;
    let robotY = 0;
    let robotTheta = 0; // in degrees

    let previousLeftEncoder = 0
    let previousRightEncoder = 0

    function logMessage(message: String){
        datalogger.log(datalogger.createCV("Message", message))
    }

    input.onButtonPressed(Button.A, function () {
        let grid1: number[][] = [];
        //grid1 = createPath([[0, 0], [10, 10], [10, 20], [25, 30], [30, 50]], 1)
        grid1 = createPath([[0, 0], [50, 0], [50, 50], [100, 50], [100, 150], [50, 150], [50, 100]]);
        datalogger.setColumnTitles("Message")
        startFinch();
        purePursuit(grid1, 10, 100);
        basic.pause(5000);
    })

    export function purePursuit(path: number[][], lookahead: number, targetVelocity: number): void {
        robotX = 0;
        robotY = 0;
        robotTheta = 0;
        previousLeftEncoder = getEncoder(RLDir.Left) * ticksPerRotation;
        previousRightEncoder = getEncoder(RLDir.Right) * ticksPerRotation;
        resetEncoders();
        logMessage("Pure Pursuit started.");
        logMessage(`Path: ${JSON.stringify(path)}`);

        let currentPathIndex = 0;

        while (currentPathIndex < path.length - 1) {
            //logMessage("[PP] START ITERATION - X: " + Math.roundWithPrecision(robotX, 2) + " Y: " + Math.roundWithPrecision(robotY, 2) + " THETA: " + Math.roundWithPrecision(robotTheta, 2));
            updateOdometry();
            const lookaheadPoint = getLookaheadPoint(path, lookahead, currentPathIndex);

            if (!lookaheadPoint) {
                const lastPoint = path[path.length - 1];
                const distanceToLast = lastPoint ? getDistance(robotX, robotY, lastPoint[0], lastPoint[1]) : -1;
                logMessage("[Error] Lookahead Point: NULL - No point found! Distance to last: " + Math.roundWithPrecision(distanceToLast, 2));
                stopMotors();
                break;
            } else {
                //logMessage("Lookahead Point: X = " + Math.roundWithPrecision(lookaheadPoint[0],2) + " Y= " + Math.roundWithPrecision(lookaheadPoint[1], 2));
            }

            const curvature = calculateCurvature(lookaheadPoint);
            //logMessage("[PP] Calculated Curvature: " + Math.roundWithPrecision(curvature, 2));

            const speeds = calculateWheelSpeeds(curvature, targetVelocity * 0.8);
            //logMessage(`[PP] Wheel Speeds - Left : ${speeds[0]}, Right : ${speeds[1]}`);
            startMotors(speeds[0], speeds[1]);

            //logMessage(`[PP] Current Path Index: ${currentPathIndex}`);


            const nextWaypointIndex = (currentPathIndex + 1) % path.length
            const distanceToNextWaypoint = getDistance(robotX, robotY, path[nextWaypointIndex][0], path[nextWaypointIndex][1])
            logMessage("[PP] Distance to next waypoint: " + Math.roundWithPrecision(distanceToNextWaypoint, 2) + " PathIndex: " + currentPathIndex );
           
            if (distanceToNextWaypoint < lookahead) {
                currentPathIndex++;
                logMessage("Advancing path index to " + currentPathIndex)

                
                //datalogger.log(datalogger.createCV("Message", "Reached waypoint " + path[currentPathIndex][0] + ":" + path[currentPathIndex][1] + " moving to next."))
            }

            basic.pause(50);


        }

        if (currentPathIndex === path.length - 1) {
            // Approaching the final waypoint
            const distanceToFinalWaypoint = getDistance(robotX, robotY, path[path.length - 1][0], path[path.length - 1][1]);
            if (distanceToFinalWaypoint < 5) { // Small tolerance in pixels
                finch.stopMotors()
                currentPathIndex++; // Ensure we exit the main loop condition
                logMessage("Stopped at final waypoint."); // Instrumentation
            } else {
                // Still moving towards the final waypoint, reduce speed?
                const lookaheadPoint = getLookaheadPoint(path, lookahead, currentPathIndex);
                if (lookaheadPoint) {
                    const curvature = calculateCurvature(lookaheadPoint);
                    const speeds = calculateWheelSpeeds(curvature, targetVelocity * 0.2); // Reduce speed
                    startMotors(speeds[0], speeds[1]);
                }
            }
        } else if (currentPathIndex >= path.length) {
            finch.stopMotors() // Ensure it stays stopped
        }

        stopMotors();
        logMessage("Pure Pursuit complete.");
    }

    function updateOdometry(): void {
        const currentLeftEncoder = getEncoder(RLDir.Left) * ticksPerRotation;
        const currentRightEncoder = getEncoder(RLDir.Right) * ticksPerRotation;
        const deltaLeftTicks = currentLeftEncoder - previousLeftEncoder; 
        const deltaRightTicks = currentRightEncoder - previousRightEncoder; 
        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder; 
        const deltaLeft = deltaLeftTicks * cmPerTick; 
        const deltaRight = deltaRightTicks * cmPerTick; 
        
        
        //datalogger.log(datalogger.createCV("Message", "deltaLeft: " + Math.roundWithPrecision(deltaLeft,2) + " deltaRight: " + Math.roundWithPrecision(deltaRight, 2)))
        //resetEncoders();

        const deltaDistance = (deltaLeft + deltaRight) / 2; 
        const robotThetaRad = (deltaRight - deltaLeft) / wheelTrack;
        let deltaThetaDeg = toDegrees(robotThetaRad)
        const initialThetaDeg = robotTheta;
        const initialThetaRad = toRadian(initialThetaDeg);
    
        robotTheta += deltaThetaDeg


        while(robotTheta > 180) robotTheta -=360;
        while(robotTheta <= -180) robotTheta +=360;

        const avgThetaRad = (initialThetaRad + robotThetaRad)
        const deltaX = deltaDistance * Math.cos(avgThetaRad);
        const deltaY = deltaDistance * Math.sin(avgThetaRad);

        robotX += deltaX; 
        robotY += deltaY;
        //datalogger.log(datalogger.createCV("Message", "ODOM: Robot theta: " + Math.roundWithPrecision(robotTheta,2) + "AND robotX: " + Math.roundWithPrecision(robotX, 2) + "AND robotY: " + Math.roundWithPrecision(robotY,2)))
    }

    function getDistance(x1: number, y1: number, x2: number, y2: number): number {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    function getLookaheadPoint(path: number[][], lookaheadDistance: number, startIndex: number): number[] | null {
        let lookaheadTarget: number[] | null = null;
        //datalogger.log(datalogger.createCV("Message", "We are now in getLookahead"));
        for (let i = startIndex; i < path.length - 1; i++) {
            const p1 = path[i];
            const p2 = path[i + 1];
            logMessage("robot:(" + Math.roundWithPrecision(robotX, 2) + ":" + Math.roundWithPrecision(robotY, 2) + "); " + "Radius: " + lookaheadDistance + "; P1:(" + Math.roundWithPrecision(p1[0], 2) + ":" + Math.roundWithPrecision(p1[1], 2) + "); P2:(" + Math.roundWithPrecision(p2[0], 2) + ":" + Math.roundWithPrecision(p2[1], 2) + ")");

            // Calculate the vector from robot to segment start
            const dx1 = p1[0] - robotX;
            const dy1 = p1[1] - robotY;

            // Calculate the vector of the segment
            const segmentDx = p2[0] - p1[0];
            const segmentDy = p2[1] - p1[1];

            const a = segmentDx * segmentDx + segmentDy * segmentDy;
            const b = 2 * (dx1 * segmentDx + dy1 * segmentDy);
            const c = dx1 * dx1 + dy1 * dy1 - lookaheadDistance * lookaheadDistance;
            const discriminant = b * b - 4 * a * c;
            //datalogger.log(datalogger.createCV("Message", "discriminant: " + discriminant ));
            if (discriminant >= 0) {
                const t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
                const t2 = (-b - Math.sqrt(discriminant)) / (2 * a);
                //datalogger.log(datalogger.createCV("Message", "t1: " + t1 + " and t2: " + t2))
                //datalogger.log(datalogger.createCV("Message", "robotX: " + robotX + " and robotY: " + robotY))

                if (t1 >= 0 && t1 <= 1) {
                    lookaheadTarget = [p1[0] + t1 * segmentDx, p1[1] + t1 * segmentDy];
                    return lookaheadTarget;
                }
                if (t2 >= 0 && t2 <= 1) {
                    lookaheadTarget = [p1[0] + t2 * segmentDx, p1[1] + t2 * segmentDy];
                    return lookaheadTarget;
                }
            }

        // If no intersection found, aim for the last point if close enough
        }
        if (path.length > 0)
            return path[path.length - 1];
    return null;
    }


    function toRadian(degrees: number) {return degrees * Math.PI / 180}
    function toDegrees(radian: number) { return radian * 180 / Math.PI }

    function calculateCurvature(lookaheadPoint: number[]): number {
        const dx = lookaheadPoint[0] - robotX;
        const dy = lookaheadPoint[1] - robotY;
        const angleToTarget = Math.atan2(dy, dx);
        const robotAngleRad = toRadian(robotTheta)
        let angleDiff = angleToTarget - robotAngleRad;
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI
        while (angleDiff <= -Math.PI) angleDiff += 2 * Math.PI

        const distanceToTarget = getDistance(robotX, robotY, lookaheadPoint[0], lookaheadPoint[1]);
        //logMessage(`CURV: Angle to Target: ..., Curvature: ${angleToTarget}`);

        if (distanceToTarget < 1) return 0; // Prevent division by zero

        return (2 * Math.sin(angleDiff)) / distanceToTarget;
    }

    function calculateWheelSpeeds(curvature: number, targetSpeed: number): number[] {
        const v = targetSpeed; // Target linear velocity (e.g., cm/s)
        const L = wheelTrack;  // Wheel track (distance between wheels)

        const vRight = v * (2 + curvature * L) / 2;
        const vLeft = v * (2 - curvature * L) / 2;

        // Convert linear velocities (cm/s) to motor speeds (-100 to 100)
        // Need to determine a scaling factor based on your robot's characteristics
        const speedFactor = 0.8; // Example - adjust as needed

        const rightSpeed = Math.round(vRight * speedFactor);
        const leftSpeed = Math.round(vLeft * speedFactor);
        return [leftSpeed, rightSpeed];
    }
   
    export function createPath(waypoints: number[][], density: number = 5): number[][] {
        const interpolatedPath: number[][] = [];
        if (waypoints.length < 2) {
            return waypoints;
        }

        for (let i = 0; i < waypoints.length - 1; i++) {
            const p1 = waypoints[i];
            const p2 = waypoints[i + 1];
            interpolatedPath.push(p1); // Add the starting point

            const distance = getDistance(p1[0], p1[1], p2[0], p2[1]);
            const numPoints = Math.max(1, Math.round(density * distance / 20)); // Adjust divisor for desired density based on distance

            for (let j = 1; j <= numPoints; j++) {
                const t = j / (numPoints + 1);
                const interpolatedX = p1[0] + t * (p2[0] - p1[0]);
                const interpolatedY = p1[1] + t * (p2[1] - p1[1]);
                interpolatedPath.push([interpolatedX, interpolatedY]);
            }
        }
        interpolatedPath.push(waypoints[waypoints.length - 1]); // Add the final point
        return interpolatedPath;
    }

}
