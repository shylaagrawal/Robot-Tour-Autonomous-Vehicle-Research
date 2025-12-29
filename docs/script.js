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

function simulateRobot(){
    if(!runningPath || userPath.length < 2) return;

    const LOOKAHEAD = 40;
    const SPEED = 2;
    const TURN_GAIN = 0.05;

    // Find lookahead target
    let targetX = null;
    let targetY = null;

    for(let i = robotIndex; i < userPath.length; i++){
        const [r,c] = userPath[i];
        const [cx,cy] = getCellCenter(r,c);
        if(distance(robotX, robotY, cx, cy) > LOOKAHEAD){
            targetX = cx;
            targetY = cy;
            robotIndex = i;
            break;
        }
    }

    // If no lookahead found, target final point
    if(targetX === null){
        const [r,c] = userPath[userPath.length-1];
        [targetX,targetY] = getCellCenter(r,c);
    }

    // Heading to lookahead
    const dx = targetX - robotX;
    const dy = targetY - robotY;
    const targetTheta = Math.atan2(dy,dx);
    const currentTheta = robotTheta * Math.PI/180;

    let angleError = targetTheta - currentTheta;
    while(angleError > Math.PI) angleError -= 2*Math.PI;
    while(angleError < -Math.PI) angleError += 2*Math.PI;

    // Continuous curvature control
    robotTheta += angleError * TURN_GAIN * 180/Math.PI;

    // Always move forward
    robotX += Math.cos(robotTheta*Math.PI/180) * SPEED;
    robotY += Math.sin(robotTheta*Math.PI/180) * SPEED;
}


loop();
