const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

const ROWS = 4;
const COLS = 5;
const CELL_SIZE = 100;

let userPath = [];
let dragging = false;
let robotIndex = 0;
let runningPath = false;

let robotX = 0;
let robotY = 0;
let robotTheta = 0; // In degrees

// --- Helpers ---
function getCellCenter(row, col){
    return [col*CELL_SIZE + CELL_SIZE/2, row*CELL_SIZE + CELL_SIZE/2];
}

function getMouseCell(e){
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const col = Math.floor(x / CELL_SIZE);
    const row = Math.floor(y / CELL_SIZE);
    return [row, col];
}

function distance(x1,y1,x2,y2){ return Math.hypot(x2-x1, y2-y1); }

// --- Event Listeners ---
canvas.addEventListener("mousedown", e=>{dragging=true; addPath(e);});
canvas.addEventListener("mousemove", e=>{if(dragging) addPath(e);});
canvas.addEventListener("mouseup", e=>{dragging=false;});

function addPath(e){
    const [row,col] = getMouseCell(e);
    if(!userPath.some(p=>p[0]==row && p[1]==col)){
        userPath.push([row,col]);
    }
}

document.getElementById("runBtn").addEventListener("click", ()=>{
    if(userPath.length>0){
        robotIndex = 0;
        robotX = userPath[0][1]*CELL_SIZE + CELL_SIZE/2;
        robotY = userPath[0][0]*CELL_SIZE + CELL_SIZE/2;
        robotTheta = 0;
        runningPath = true;
    }
});

// --- Drawing ---
function drawGrid(){
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

// --- Simulation Logic (simplified Pure Pursuit) ---
function simulateRobot(){
    if(!runningPath || robotIndex>=userPath.length) return;
    const [targetRow,targetCol] = userPath[robotIndex];
    const [targetX,targetY] = getCellCenter(targetRow,targetCol);

    const dx = targetX - robotX;
    const dy = targetY - robotY;
    const targetTheta = Math.atan2(dy,dx)*180/Math.PI;
    let angleDiff = targetTheta - robotTheta;
    while(angleDiff>180) angleDiff-=360;
    while(angleDiff<-180) angleDiff+=360;

    // Turn in place if angle difference is big
    if(Math.abs(angleDiff)>10){
        robotTheta += Math.sign(angleDiff)*5; // turn speed
    } else {
        const dist = distance(robotX,robotY,targetX,targetY);
        if(dist>2){
            robotX += Math.cos(robotTheta*Math.PI/180)*2;
            robotY += Math.sin(robotTheta*Math.PI/180)*2;
        } else {
            robotIndex++;
        }
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
