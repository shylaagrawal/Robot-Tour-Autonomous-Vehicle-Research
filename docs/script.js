const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

const ROWS = 4;
const COLS = 5;
const CELL_SIZE = 100;

let userPath = [];
let smoothPath = [];
let dragging = false;
let runningPath = false;
let robotIndex = 0;

let robotX = 0;
let robotY = 0;
let robotTheta = 0; // degrees

// ---------- Helpers ----------
function getCellCenter(row, col){
    return [col * CELL_SIZE + CELL_SIZE / 2,
            row * CELL_SIZE + CELL_SIZE / 2];
}

function distance(x1,y1,x2,y2){
    return Math.hypot(x2-x1, y2-y1);
}

function getMouseCell(e){
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    return [
        Math.floor(y / CELL_SIZE),
        Math.floor(x / CELL_SIZE)
    ];
}

// ---------- Mouse Input ----------
canvas.addEventListener("mousedown", e=>{
    dragging = true;
    addPath(e);
});

canvas.addEventListener("mousemove", e=>{
    if(dragging) addPath(e);
});

canvas.addEventListener("mouseup", ()=>{
    dragging = false;
});

function addPath(e){
    const [r,c] = getMouseCell(e);
    if(r<0 || c<0 || r>=ROWS || c>=COLS) return;
    if(!userPath.some(p => p[0] === r && p[1] === c)){
        userPath.push([r,c]);
    }
}

// ---------- Run Button ----------
document.getElementById("runBtn").addEventListener("click", ()=>{
    if(userPath.length < 2) return;

    smoothPath = [];

    for(let i=0;i<userPath.length-1;i++){
        const [r1,c1] = userPath[i];
        const [r2,c2] = userPath[i+1];
        const [x1,y1] = getCellCenter(r1,c1);
        const [x2,y2] = getCellCenter(r2,c2);

        const steps = 25;
        for(let t=0;t<=steps;t++){
            smoothPath.push([
                x1 + (x2-x1)*t/steps,
                y1 + (y2-y1)*t/steps
            ]);
        }
    }

    robotIndex = 0;
    [robotX, robotY] = smoothPath[0];
    robotTheta = 0;
    runningPath = true;
});

// ---------- Drawing ----------
function drawGrid(){
    ctx.clearRect(0,0,canvas.width,canvas.height);
    ctx.strokeStyle = "black";
    for(let r=0;r<ROWS;r++){
        for(let c=0;c<COLS;c++){
            ctx.strokeRect(
                c*CELL_SIZE,
                r*CELL_SIZE,
                CELL_SIZE,
                CELL_SIZE
            );
        }
    }
}

function drawUserPath(){
    if(userPath.length < 2) return;
    ctx.strokeStyle = "blue";
    ctx.lineWidth = 3;
    ctx.beginPath();
    let [x,y] = getCellCenter(userPath[0][0], userPath[0][1]);
    ctx.moveTo(x,y);
    for(let i=1;i<userPath.length;i++){
        [x,y] = getCellCenter(userPath[i][0], userPath[i][1]);
        ctx.lineTo(x,y);
    }
    ctx.stroke();
}

function drawRobot(){
    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(robotX, robotY, 8, 0, Math.PI*2);
    ctx.fill();
}

// ---------- Pure Pursuit ----------
function simulateRobot(){
    if(!runningPath || robotIndex >= smoothPath.length) return;

    const LOOKAHEAD = 40;
    const SPEED = 2.2;
    const TURN_GAIN = 0.08;

    // Advance path index
    while(
        robotIndex < smoothPath.length-1 &&
        distance(robotX, robotY,
                 smoothPath[robotIndex][0],
                 smoothPath[robotIndex][1]) < 6
    ){
        robotIndex++;
    }

    // Lookahead target
    let lx = smoothPath[robotIndex][0];
    let ly = smoothPath[robotIndex][1];

    for(let i=robotIndex;i<smoothPath.length;i++){
        if(distance(robotX, robotY,
                    smoothPath[i][0],
                    smoothPath[i][1]) >= LOOKAHEAD){
            lx = smoothPath[i][0];
            ly = smoothPath[i][1];
            break;
        }
    }

    const targetTheta = Math.atan2(ly-robotY, lx-robotX);
    const currentTheta = robotTheta * Math.PI/180;

    let error = targetTheta - currentTheta;
    while(error > Math.PI) error -= 2*Math.PI;
    while(error < -Math.PI) error += 2*Math.PI;

    robotTheta += error * TURN_GAIN * 180/Math.PI;

    robotX += Math.cos(robotTheta*Math.PI/180) * SPEED;
    robotY += Math.sin(robotTheta*Math.PI/180) * SPEED;
}

// ---------- Loop ----------
function loop(){
    drawGrid();
    drawUserPath();
    simulateRobot();
    drawRobot();
    requestAnimationFrame(loop);
}

loop();
