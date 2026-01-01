<canvas id="gridCanvas" width="500" height="400"></canvas>
<button id="runBtn">Run</button>

<script>
const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

const ROWS = 4;
const COLS = 5;
const CELL_SIZE = 200;

// --- Robot parameters (from v1) ---
const robotRadius = 8;
const wheelTrack = 30;
const lookaheadDistance = 40;
const targetVelocity = 4;

let userPath = [];
let path = [];
let dragging = false;
let runningPath = false;
let currentPathIndex = 0;

let robotX = 0;
let robotY = 0;
let robotTheta = 0; // degrees

// ---------- Helpers ----------
function getCellCenter(r,c){
    return [c*CELL_SIZE + CELL_SIZE/2,
            r*CELL_SIZE + CELL_SIZE/2];
}

function distance(x1,y1,x2,y2){
    return Math.hypot(x2-x1, y2-y1);
}

function toRad(d){ return d*Math.PI/180; }
function toDeg(r){ return r*180/Math.PI; }

// ---------- Mouse ----------
canvas.addEventListener("mousedown",e=>{
    dragging = true;
    addPath(e);
});
canvas.addEventListener("mousemove",e=>{
    if(dragging) addPath(e);
});
canvas.addEventListener("mouseup",()=>dragging=false);

function addPath(e){
    const rect = canvas.getBoundingClientRect();
    const r = Math.floor((e.clientY-rect.top)/CELL_SIZE);
    const c = Math.floor((e.clientX-rect.left)/CELL_SIZE);
    if(r<0||c<0||r>=ROWS||c>=COLS) return;
    if(!userPath.some(p=>p[0]==r && p[1]==c)){
        userPath.push([r,c]);
    }
}

// ---------- Run ----------
document.getElementById("runBtn").addEventListener("click",()=>{
    if(userPath.length<2) return;

    path = userPath.map(p => getCellCenter(p[0],p[1]));
    currentPathIndex = 0;

    robotX = path[0][0];
    robotY = path[0][1];
    robotTheta = 0;
    runningPath = true;
});

// ---------- Pure Pursuit (EXACT v1 LOGIC) ----------
function getLookaheadPoint(){
    for(let i=currentPathIndex;i<path.length-1;i++){
        const [x1,y1] = path[i];
        const [x2,y2] = path[i+1];

        const dx = x2-x1;
        const dy = y2-y1;

        const fx = x1-robotX;
        const fy = y1-robotY;

        const a = dx*dx + dy*dy;
        const b = 2*(fx*dx + fy*dy);
        const c = fx*fx + fy*fy - lookaheadDistance*lookaheadDistance;

        const disc = b*b - 4*a*c;
        if(disc < 0) continue;

        const t = (-b + Math.sqrt(disc)) / (2*a);
        if(t>=0 && t<=1){
            return [x1 + t*dx, y1 + t*dy];
        }
    }
    return path[path.length-1];
}

function calculateCurvature(lx,ly){
    const dx = lx-robotX;
    const dy = ly-robotY;

    const angleToTarget = Math.atan2(dy,dx);
    let angleDiff = angleToTarget - toRad(robotTheta);

    while(angleDiff>Math.PI) angleDiff-=2*Math.PI;
    while(angleDiff<-Math.PI) angleDiff+=2*Math.PI;

    const dist = Math.hypot(dx,dy);
    if(dist<1) return 0;

    return (2*Math.sin(angleDiff))/dist;
}

function wheelSpeeds(curv){
    const v = targetVelocity;
    return [
        v*(2-curv*wheelTrack)/2,
        v*(2+curv*wheelTrack)/2
    ];
}

function updateOdometry(vL,vR){
    const dl = vL;
    const dr = vR;

    const dc = (dl+dr)/2;
    const dtheta = (dr-dl)/wheelTrack;

    robotTheta += toDeg(dtheta);
    robotX += dc*Math.cos(toRad(robotTheta));
    robotY += dc*Math.sin(toRad(robotTheta));
}

function simulateRobot(){
    if(!runningPath || currentPathIndex>=path.length-1) return;

    const lookahead = getLookaheadPoint();
    const curvature = calculateCurvature(lookahead[0],lookahead[1]);
    const [vL,vR] = wheelSpeeds(curvature);
    updateOdometry(vL,vR);

    if(distance(robotX,robotY,
                path[currentPathIndex+1][0],
                path[currentPathIndex+1][1]) < lookaheadDistance/2){
        currentPathIndex++;
    }
}

// ---------- Draw ----------
function drawGrid(){
    ctx.clearRect(0,0,canvas.width,canvas.height);
    for(let r=0;r<ROWS;r++){
        for(let c=0;c<COLS;c++){
            ctx.strokeRect(c*CELL_SIZE,r*CELL_SIZE,
                           CELL_SIZE,CELL_SIZE);
        }
    }
}

function drawPath(){
    if(path.length<2) return;
    ctx.strokeStyle="blue";
    ctx.lineWidth=3;
    ctx.beginPath();
    ctx.moveTo(path[0][0],path[0][1]);
    for(let i=1;i<path.length;i++){
        ctx.lineTo(path[i][0],path[i][1]);
    }
    ctx.stroke();
}

function drawRobot(){
    ctx.fillStyle="red";
    ctx.beginPath();
    ctx.arc(robotX,robotY,robotRadius,0,Math.PI*2);
    ctx.fill();

    const a = toRad(robotTheta);
    ctx.beginPath();
    ctx.moveTo(robotX,robotY);
    ctx.lineTo(robotX+15*Math.cos(a), robotY+15*Math.sin(a));
    ctx.stroke();
}

// ---------- Loop ----------
function loop(){
    drawGrid();
    drawPath();
    simulateRobot();
    drawRobot();
    requestAnimationFrame(loop);
}
loop();
</script>
