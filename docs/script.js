const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

const ROWS = 4;
const COLS = 5;
const CELL_SIZE = 100;

let userPath = [];
let dragging = false;

let robotX = 0;
let robotY = 0;
let robotTheta = 0; // degrees
let runningPath = false;

let lookahead = 30;   // pixels
let targetSpeed = 2;  // pixels per frame
let pathIndex = 0;

// --- Helpers ---
function getCellCenter(row, col) {
    return [col*CELL_SIZE + CELL_SIZE/2, row*CELL_SIZE + CELL_SIZE/2];
}

function getMouseCell(e) {
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const col = Math.floor(x / CELL_SIZE);
    const row = Math.floor(y / CELL_SIZE);
    return [row, col];
}

function distance(x1,y1,x2,y2){ return Math.hypot(x2-x1, y2-y1); }

// --- Event Listeners ---
canvas.addEventListener("mousedown", e => { dragging = true; addPath(e); });
canvas.addEventListener("mousemove", e => { if(dragging) addPath(e); });
canvas.addEventListener("mouseup", e => { dragging = false; });

function addPath(e) {
    const [row,col] = getMouseCell(e);
    if(!userPath.some(p=>p[0]==row && p[1]==col)){
        userPath.push([row,col]);
    }
}

document.getElementById("runBtn").addEventListener("click", ()=>{
    if(userPath.length>0){
        const [row,col] = userPath[0];
        [robotX, robotY] = getCellCenter(row,col);
        robotTheta = 0;
        pathIndex = 0;
        runningPath = true;
    }
});

// --- Drawing ---
function drawGrid() {
    ctx.clearRect(0,0,canvas.width,canvas.height);
    for(let r=0;r<ROWS;r++){
        for(let c=0;c<COLS;c++){
            ctx.strokeStyle="black";
            ctx.lineWidth=1;
            ctx.strokeRect(c*CELL_SIZE,r*CELL_SIZE,CELL_SIZE,CELL_SIZE);
        }
    }
}

function drawUserPath(){
    if(userPath.length>1){
        ctx.strokeStyle="blue";
        ctx.lineWidth=3;
        ctx.beginPath();
        let [x,y] = getCellCenter(userPath[0][0], userPath[0][1]);
        ctx.moveTo(x,y);
        for(let i=1;i<userPath.length;i++){
            [x,y] = getCellCenter(userPath[i][0], userPath[i][1]);
            ctx.lineTo(x,y);
        }
        ctx.stroke();
    }
}

function drawRobot(){
    ctx.fillStyle="red";
    ctx.beginPath();
    ctx.arc(robotX, robotY, 8, 0, Math.PI*2);
    ctx.fill();
}

// --- Pure Pursuit Simulation ---
function getLookaheadPoint(path, startIndex, lookaheadDist) {
    for(let i=startIndex; i<path.length-1; i++){
        const [x1,y1] = getCellCenter(path[i][0], path[i][1]);
        const [x2,y2] = getCellCenter(path[i+1][0], path[i+1][1]);

        const dx = x2-x1;
        const dy = y2-y1;
        const segLenSq = dx*dx + dy*dy;
        if(segLenSq < 1e-6) continue;

        const fx = x1-robotX;
        const fy = y1-robotY;
        const a = segLenSq;
        const b = 2*(fx*dx + fy*dy);
        const c = fx*fx + fy*fy - lookaheadDist*lookaheadDist;
        const disc = b*b - 4*a*c;

        if(disc>=0){
            const sqrtDisc = Math.sqrt(disc);
            const t1 = (-b + sqrtDisc)/(2*a);
            const t2 = (-b - sqrtDisc)/(2*a);
            for(const t of [t1,t2]){
                if(t>=0 && t<=1){
                    return [x1 + t*dx, y1 + t*dy];
                }
            }
        }
    }
    return path.length ? getCellCenter(path[path.length-1][0], path[path.length-1][1]) : null;
}

function simulateRobot() {
    if(!runningPath || pathIndex>=userPath.length) return;

    const lookaheadPoint = getLookaheadPoint(userPath, pathIndex, lookahead);
    if(!lookaheadPoint) return;

    const [lx,ly] = lookaheadPoint;
    const dx = lx-robotX;
    const dy = ly-robotY;
    const angleToTarget = Math.atan2(dy,dx)*180/Math.PI;
    let angleDiff = angleToTarget - robotTheta;
    while(angleDiff>180) angleDiff-=360;
    while(angleDiff<-180) angleDiff+=360;

    // Smooth rotation
    robotTheta += Math.sign(angleDiff) * Math.min(5, Math.abs(angleDiff));

    // Move forward along heading
    const dist = distance(robotX,robotY,lx,ly);
    if(dist>1){
        const rad = robotTheta*Math.PI/180;
        robotX += Math.cos(rad)*Math.min(targetSpeed, dist);
        robotY += Math.sin(rad)*Math.min(targetSpeed, dist);
    }

    // Advance path index if close to next waypoint
    const [nextX,nextY] = getCellCenter(userPath[pathIndex][0], userPath[pathIndex][1]);
    if(distance(robotX,robotY,nextX,nextY)<lookahead/2){
        pathIndex++;
    }
}

// --- Main Loop ---
function loop(){
    drawGrid();
    drawUserPath();
    simulateRobot();
    drawRobot();
    requestAnimationFrame(loop);
}

loop();
