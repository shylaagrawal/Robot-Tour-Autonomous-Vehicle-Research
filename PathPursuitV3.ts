namespace finch {
    // --- Constants ---
    const wheelDiameter = 5.0; // cm
    const wheelTrack = 10.0;  // cm
    const ticksPerRotation = 792;
    const cmPerTick = (Math.PI * wheelDiameter) / ticksPerRotation;

    // --- Robot State ---
    let robotX = 0;
    let robotY = 0;
    let robotTheta = 0; // degrees

    // --- Odometry State ---
    let previousLeftEncoder = 0;
    let previousRightEncoder = 0;

    // --- Helper Functions ---
    function toRadian(degrees: number): number { return degrees * Math.PI / 180; }
    function toDegrees(radian: number): number { return radian * 180 / Math.PI; }
    function getDistance(x1: number, y1: number, x2: number, y2: number): number {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    function logMessage(message: string) {
        datalogger.log(datalogger.createCV("Message", message));
    }

    // --- Odometry Functions ---
    function initializeOdometry(): void {
        robotX = 0;
        robotY = 0;
        robotTheta = 0;
        resetEncoders();
        basic.pause(100);
        previousLeftEncoder = getEncoder(RLDir.Left) * ticksPerRotation;
        previousRightEncoder = getEncoder(RLDir.Right) * ticksPerRotation;
    }

    function updateOdometry(): void {
        const currentLeftEncoder = getEncoder(RLDir.Left) * ticksPerRotation;
        const currentRightEncoder = getEncoder(RLDir.Right) * ticksPerRotation;

        const deltaLeft = (currentLeftEncoder - previousLeftEncoder) * cmPerTick;
        const deltaRight = (currentRightEncoder - previousRightEncoder) * cmPerTick;

        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder;

        const deltaDistance = (deltaLeft + deltaRight) / 2.0;
        const deltaThetaRad = (deltaRight - deltaLeft) / wheelTrack;
        const deltaThetaDeg = toDegrees(deltaThetaRad);

        const avgThetaRad = toRadian(robotTheta) + deltaThetaRad / 2.0;

        robotTheta += deltaThetaDeg;
        while (robotTheta > 180) robotTheta -= 360;
        while (robotTheta <= -180) robotTheta += 360;

        robotX += deltaDistance * Math.cos(avgThetaRad);
        robotY += deltaDistance * Math.sin(avgThetaRad);
    }

    // --- Path Lookahead ---
    function getLookaheadPoint(path: number[][], lookaheadDistance: number, startIndex: number): number[] | null {
        for (let i = startIndex; i < path.length - 1; i++) {
            const p1 = path[i];
            const p2 = path[i + 1];

            const dx = p2[0] - p1[0];
            const dy = p2[1] - p1[1];
            const segmentLenSq = dx * dx + dy * dy;
            if (segmentLenSq < 1e-6) continue;

            const fx = p1[0] - robotX;
            const fy = p1[1] - robotY;
            const a = segmentLenSq;
            const b = 2 * (fx * dx + fy * dy);
            const c = fx * fx + fy * fy - lookaheadDistance * lookaheadDistance;
            const disc = b * b - 4 * a * c;
            if (disc >= 0) {
                const sqrtDisc = Math.sqrt(disc);
                const t1 = (-b + sqrtDisc) / (2 * a);
                const t2 = (-b - sqrtDisc) / (2 * a);
                for (const t of [t1, t2]) {
                    if (t >= 0 && t <= 1) return [p1[0] + t * dx, p1[1] + t * dy];
                }
            }
        }
        return path.length ? path[path.length - 1] : null;
    }

    function calculateCurvature(lookaheadPoint: number[], direction: number): number {
        const dx = lookaheadPoint[0] - robotX;
        const dy = lookaheadPoint[1] - robotY;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 0.1) return 0;

        const angleToTarget = Math.atan2(dy, dx);
        const robotAngle = toRadian(robotTheta + (direction < 0 ? 180 : 0));
        let angleDiff = angleToTarget - robotAngle;
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff <= -Math.PI) angleDiff += 2 * Math.PI;

        return (2 * Math.sin(angleDiff)) / dist;
    }

    function calculateWheelSpeeds(curvature: number, baseSpeed: number): number[] {
        let left = baseSpeed * (1 - curvature * wheelTrack / 2);
        let right = baseSpeed * (1 + curvature * wheelTrack / 2);
        left = Math.max(-100, Math.min(100, left));
        right = Math.max(-100, Math.min(100, right));
        return [Math.round(left), Math.round(right)];
    }

    // --- Pure Pursuit Main Loop ---
    export function purePursuit(path: number[][], lookahead: number, targetVelocity: number): void {
        if (!path || path.length < 2) return;

        initializeOdometry();
        let currentPathIndex = 0;

        while (currentPathIndex < path.length - 1) {
            updateOdometry();

            const lookaheadPoint = getLookaheadPoint(path, lookahead, currentPathIndex);
            if (!lookaheadPoint) break;

            let direction = 1; // Always move forward
            const curvature = calculateCurvature(lookaheadPoint, direction);
            const speeds = calculateWheelSpeeds(curvature, targetVelocity);
            startMotors(speeds[0], speeds[1]);

            // Advance path index
            const distToNext = getDistance(robotX, robotY, path[currentPathIndex + 1][0], path[currentPathIndex + 1][1]);
            if (distToNext < lookahead) currentPathIndex++;

            basic.pause(50);
        }

        stopMotors();
    }

    // --- Path Interpolation ---
    export function createPath(waypoints: number[][], density: number = 5): number[][] {
        const path: number[][] = [];
        if (waypoints.length < 2) return waypoints;

        path.push(waypoints[0]);
        for (let i = 0; i < waypoints.length - 1; i++) {
            const p1 = waypoints[i];
            const p2 = waypoints[i + 1];
            const distance = getDistance(p1[0], p1[1], p2[0], p2[1]);
            const numPoints = Math.max(0, Math.round(density * distance / 20) - 1);
            for (let j = 1; j <= numPoints; j++) {
                const t = j / (numPoints + 1);
                path.push([p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1])]);
            }
            path.push(p2);
        }
        return path;
    }
}
