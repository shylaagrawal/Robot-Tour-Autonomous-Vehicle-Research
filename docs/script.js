<canvas id="gridCanvas" width="500" height="400"></canvas>
<button id="runBtn">Run</button>

<script>
const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

const ROWS = 4;
const COLS = 5;
const CELL_SIZE = 100;

// ---- Robot parameters ----
const WHEEL_TRACK = 30;
const LOOKAHEAD = 40;
const TARGET_SPEED = 4;

let userPath = [];
let smoothPath = [];
let dragging = false;
let runningPath = false;
let robotIndex = 0;

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

function getMouseCell(e){
    const rect = canvas.getBoundingClientRect();
    return [
        Math.floor((e.clientY-rect.top)/CELL_SIZE),
        Math.floor((e.clientX-rect.left)/CELL_SIZE)
    ];
}

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
    const [r,c] = getMouseCell(e);
    if(r<0||c<0||r>=ROWS||c>=COLS) return;
    if(!userPath.some(p=>p[0]==r && p[1]==c)){
        userPath.push([r,c]);
    }
}

// ---------- Run ----------
document.getElementById("runBtn").addEventListener("click",()=>{
    if(userPath.length<2) return;

    smoothPath = [];
    for(let i=0;i<userPath.length-1;i++){
        const [r1,c1]=userPath[i];
        const [r2,c2]=userPath[i+1];
        const [x1,y1]=getCellCenter(r1,c1);
        const [x2,y2]=getCellCenter(r2,c2);
        for(let t=0;t<=25;t++){
            smoothPath.push([
                x1+(x2-x1)*t/25,
                y1+(y2-y1)*t/25
            ]);
        }
    }

    robotIndex = 0;
    [robotX,robotY] = smoothPath[0];
    robotTheta = 0;
    runningPath = true;
});

// ---------- Pure Pursuit ----------
function calculateCurvature(lx,ly){
    const dx = lx-robotX;
    const dy = ly-robotY;

    const targetAngle = Math.atan2(dy,dx);
    const robotAngle = robotTheta*Math.PI/180;

    let diff = targetAngle-robotAngle;
    while(diff>Math.PI) diff-=2*Math.PI;
    while(diff<-Math.PI) diff+=2*Math.PI;

    const dist = Math.hypot(dx,dy);
    if(dist<1) return 0;

    return (2*Math.sin(diff))/dist;
}

function wheelSpeeds(curv){
    const v = TARGET_SPEED;
    return [
        v*(2-curv*WHEEL_TRACK)/2,
        v*(2+curv*WHEEL_TRACK)/2
    ];
}

function updateOdometry(vL,vR){
    const dl=vL, dr=vR;
    const dc=(dl+dr)/2;
    const dtheta=(dr-dl)/WHEEL_TRACK;

    robotTheta += dtheta*180/Math.PI;
    robotX += dc*Math.cos(robotTheta*Math.PI/180);
    robotY += dc*Math.sin(robotTheta*Math.PI/180);
}

function simulateRobot(){
    if(!runningPath||robotIndex>=smoothPath.length) return;

    while(robotIndex<smoothPath.length-1 &&
          distance(robotX,robotY,
                   smoothPath[robotIndex][0],
                   smoothPath[robotIndex][1])<6){
        robotIndex++;
    }

    let lx=smoothPath[robotIndex][0];
    let ly=smoothPath[robotIndex][1];
    for(let i=robotIndex;i<smoothPath.length;i++){
        if(distance(robotX,robotY,
                    smoothPath[i][0],
                    smoothPath[i][1])>=LOOKAHEAD){
            lx=smoothPath[i][0];
            ly=smoothPath[i][1];
            break;
        }
    }

    const curv = calculateCurvature(lx,ly);
    const [vL,vR] = wheelSpeeds(curv);
    updateOdometry(vL,vR);
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
    if(userPath.length<2) return;
    ctx.strokeStyle="blue";
    ctx.lineWidth=3;
    ctx.beginPath();
    let [x,y]=getCellCenter(userPath[0][0],userPath[0][1]);
    ctx.moveTo(x,y);
    for(let i=1;i<userPath.length;i++){
        [x,y]=getCellCenter(userPath[i][0],userPath[i][1]);
        ctx.lineTo(x,y);
    }
    ctx.stroke();
}

function drawRobot(){
    ctx.fillStyle="red";
    ctx.beginPath();
    ctx.arc(robotX,robotY,8,0,Math.PI*2);
    ctx.fill();
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
