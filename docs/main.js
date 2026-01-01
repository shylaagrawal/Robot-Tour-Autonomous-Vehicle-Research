const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

// Make canvas fill window
function resizeCanvas() {
  canvas.width = window.innerWidth * 0.9;
  canvas.height = window.innerHeight * 0.9;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Grid settings
const GRID_SIZE = 100;
const GRID_COLS = 5;
const GRID_ROWS = 4;

// Path drawing
let userPath = [];
let smoothPath = [];
let dragging = false;
let runningPath = false;
let robotIndex = 0;

// Robot
let robotX = 0;
let robotY = 0;
let robotTheta = 0;
const lookaheadDistance = 40;
const baseSpeed = 4;
const wheelBase = 20;
let debug = {};
let reachedEnd = false;

/* ---------- HELPERS ---------- */
function getCellCenter(row, col){
  return [col*GRID_SIZE + GRID_SIZE/2, row*GRID_SIZE + GRID_SIZE/2];
}

function distance(x1,y1,x2,y2){ return Math.hypot(x2-x1, y2-y1); }

function getMouseCell(e){
  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left;
  const y = e.clientY - rect.top;
  return [Math.floor(y/GRID_SIZE), Math.floor(x/GRID_SIZE)];
}

/* ---------- MOUSE INPUT ---------- */
canvas.addEventListener("mousedown", e=>{
  dragging = true;
  addPath(e);
});

canvas.addEventListener("mousemove", e=>{
  if(dragging) addPath(e);
});

canvas.addEventListener("mouseup", e=>{
  dragging = false;
  createSmoothPath();
});

function addPath(e){
  const [r,c] = getMouseCell(e);
  if(r<0||c<0||r>=GRID_ROWS||c>=GRID_COLS) return;
  if(!userPath.some(p=>p[0]===r && p[1]===c)){
    userPath.push([r,c]);
  }
}

/* ---------- CREATE SMOOTH PATH ---------- */
function createSmoothPath(){
  smoothPath = [];
  for(let i=0;i<userPath.length-1;i++){
    const [r1,c1] = userPath[i];
    const [r2,c2] = userPath[i+1];
    const [x1,y1] = getCellCenter(r1,c1);
    const [x2,y2] = getCellCenter(r2,c2);
    const steps = 25;
    for(let t=0;t<=steps;t++){
      smoothPath.push([x1 + (x2-x1)*t/steps, y1 + (y2-y1)*t/steps]);
    }
  }
  if(smoothPath.length>0){
    robotIndex = 0;
    [robotX, robotY] = smoothPath[0];
    robotTheta = 0;
    runningPath = true;
    reachedEnd = false;
  }
}

/* ---------- DRAW GRID & PATH ---------- */
function drawGrid(){
  ctx.fillStyle = "#f9f9f9";
  ctx.fillRect(0,0,canvas.width, canvas.height);

  ctx.strokeStyle = "#ccc";
  for(let x=0;x<=GRID_COLS*GRID_SIZE;x+=GRID_SIZE){
    ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,GRID_ROWS*GRID_SIZE); ctx.stroke();
  }
  for(let y=0;y<=GRID_ROWS*GRID_SIZE;y+=GRID_SIZE){
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(GRID_COLS*GRID_SIZE,y); ctx.stroke();
  }

  ctx.fillStyle = "#888";
  ctx.font = "12px monospace";
  for(let r=0;r<GRID_ROWS;r++){
    for(let c=0;c<GRID_COLS;c++){
      ctx.fillText(`(${c},${r})`, c*GRID_SIZE+5, r*GRID_SIZE+15);
    }
  }
}

function drawUserPath(){
  if(userPath.length<2) return;
  ctx.strokeStyle = "blue";
  ctx.lineWidth = 4;
  ctx.beginPath();
  let [x,y] = getCellCenter(userPath[0][0], userPath[0][1]);
  ctx.moveTo(x,y);
  for(let i=1;i<userPath.length;i++){
    [x,y] = getCellCenter(userPath[i][0], userPath[i][1]);
    ctx.lineTo(x,y);
  }
  ctx.stroke();
}

/* ---------- ROBOT & PURE PURSUIT ---------- */
function drawRobot(){
  ctx.fillStyle = "red";
  ctx.beginPath(); ctx.arc(robotX, robotY, 8, 0, Math.PI*2); ctx.fill();

  ctx.strokeStyle = "black";
  ctx.beginPath();
  ctx.moveTo(robotX, robotY);
  ctx.lineTo(robotX+15*Math.cos(robotTheta), robotY+15*Math.sin(robotTheta));
  ctx.stroke();

  if(!reachedEnd && smoothPath.length>0){
    ctx.fillStyle = "green";
    ctx.beginPath();
    ctx.arc(debug.lookahead ? debug.lookahead[0] : robotX,
            debug.lookahead ? debug.lookahead[1] : robotY,
            5,0,Math.PI*2);
    ctx.fill();
  }
}

function simulateRobot(){
  if(!runningPath || reachedEnd || robotIndex>=smoothPath.length) return;

  const LOOKAHEAD = lookaheadDistance;
  const SPEED = baseSpeed;
  const TURN_GAIN = 0.08;

  while(robotIndex<smoothPath.length-1 &&
        distance(robotX, robotY, smoothPath[robotIndex][0], smoothPath[robotIndex][1])<6){
    robotIndex++;
  }

  let lx = smoothPath[robotIndex][0];
  let ly = smoothPath[robotIndex][1];

  for(let i=robotIndex;i<smoothPath.length;i++){
    if(distance(robotX, robotY, smoothPath[i][0], smoothPath[i][1])>=LOOKAHEAD){
      lx = smoothPath[i][0]; ly = smoothPath[i][1]; break;
    }
  }

  let targetTheta = Math.atan2(ly-robotY, lx-robotX);
  let currentTheta = robotTheta;

  let error = targetTheta - currentTheta;
  while(error>Math.PI) error-=2*Math.PI;
  while(error<-Math.PI) error+=2*Math.PI;

  robotTheta += error*TURN_GAIN;

  robotX += Math.cos(robotTheta)*SPEED;
  robotY += Math.sin(robotTheta)*SPEED;

  if(robotIndex===smoothPath.length-1 &&
     distance(robotX, robotY, smoothPath[smoothPath.length-1][0], smoothPath[smoothPath.length-1][1])<2){
    reachedEnd = true;
  }

  debug = { lookahead: [lx,ly] };
}

/* ---------- DEBUG PANEL ---------- */
function drawDebugPanel(){
  ctx.fillStyle = "#f4f4f4";
  ctx.fillRect(canvas.width-400,0,400,canvas.height);
  ctx.strokeStyle = "#000";
  ctx.beginPath();
  ctx.moveTo(canvas.width-400,0); ctx.lineTo(canvas.width-400,canvas.height); ctx.stroke();
}

function drawDebug(){
  ctx.fillStyle = "black";
  ctx.font = "13px monospace";
  let x = canvas.width-380, y=20;
  function line(t){ ctx.fillText(t,x,y); y+=16; }

  line("PURE PURSUIT DEBUG"); y+=8;
  line(`Robot X: ${robotX.toFixed(2)}`);
  line(`Robot Y: ${robotY.toFixed(2)}`);
  line(`Heading θ: ${(robotTheta*180/Math.PI).toFixed(2)} deg`);
  line(`Path points: ${userPath.length}`);
  line(`Smooth path length: ${smoothPath.length}`);
  if(!reachedEnd && smoothPath.length>0){
    line(`Lookahead X: ${debug.lookahead[0].toFixed(2)}`);
    line(`Lookahead Y: ${debug.lookahead[1].toFixed(2)}`);
  } else line("Robot has reached the last point.");
}

/* ---------- LOOP ---------- */
function loop(){
  ctx.clearRect(0,0,canvas.width,canvas.height);
  drawGrid();
  drawUserPath();
  simulateRobot();
  drawRobot();
  drawDebugPanel();
  drawDebug();
  requestAnimationFrame(loop);
}
loop();
