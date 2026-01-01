const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

// Make canvas fill window
function resizeCanvas() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Grid settings
const GRID_SIZE = 100;
const GRID_COLS = 5;
const GRID_ROWS = 4;

// Path inside visible area
let path = [
  [50, 50],
  [150, 50],
  [250, 100],
  [350, 150],
  [450, 200]
];

let robotX = path[0][0];
let robotY = path[0][1];
let robotTheta = 0;
let currentPathIndex = 0;

const lookaheadDistance = 40;
const baseSpeed = 4;
const wheelBase = 20;

let debug = {};
let reachedEnd = false;

/* ---------- DRAW ---------- */
function drawGrid() {
  ctx.fillStyle = "#f9f9f9";
  ctx.fillRect(0,0,GRID_COLS*GRID_SIZE, GRID_ROWS*GRID_SIZE);

  ctx.strokeStyle = "#ccc";
  for (let x = 0; x <= GRID_COLS*GRID_SIZE; x += GRID_SIZE) {
    ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,GRID_ROWS*GRID_SIZE); ctx.stroke();
  }
  for (let y = 0; y <= GRID_ROWS*GRID_SIZE; y += GRID_SIZE) {
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(GRID_COLS*GRID_SIZE,y); ctx.stroke();
  }

  // Draw coordinates
  ctx.fillStyle = "#888";
  ctx.font = "12px monospace";
  for (let r = 0; r < GRID_ROWS; r++) {
    for (let c = 0; c < GRID_COLS; c++) {
      ctx.fillText(`(${c},${r})`, c*GRID_SIZE+5, r*GRID_SIZE+15);
    }
  }
}

function drawPath() {
  ctx.strokeStyle = "blue";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(path[0][0], path[0][1]);
  for (let p of path) ctx.lineTo(p[0], p[1]);
  ctx.stroke();
}

function drawRobot() {
  // Robot circle
  ctx.fillStyle = "red";
  ctx.beginPath();
  ctx.arc(robotX, robotY, 8, 0, Math.PI*2);
  ctx.fill();

  // Heading line
  ctx.strokeStyle = "black";
  ctx.beginPath();
  ctx.moveTo(robotX, robotY);
  ctx.lineTo(robotX + 15*Math.cos(robotTheta), robotY + 15*Math.sin(robotTheta));
  ctx.stroke();

  // Lookahead point
  if(!reachedEnd){
    ctx.fillStyle = "green";
    ctx.beginPath();
    ctx.arc(debug.lookahead ? debug.lookahead[0] : robotX,
            debug.lookahead ? debug.lookahead[1] : robotY,
            5,0,Math.PI*2);
    ctx.fill();
  }
}

/* ---------- PURE PURSUIT ---------- */
function simulateRobot() {
  if(reachedEnd) return;

  let target = path[currentPathIndex];
  let dx = target[0] - robotX;
  let dy = target[1] - robotY;
  let dist = Math.hypot(dx, dy);

  if(dist < lookaheadDistance && currentPathIndex < path.length-1) {
    currentPathIndex++;
    target = path[currentPathIndex];
    dx = target[0] - robotX;
    dy = target[1] - robotY;
    dist = Math.hypot(dx, dy);
  }

  if(currentPathIndex === path.length-1 && dist < 2){
    reachedEnd = true;
    return;
  }

  let angleToTarget = Math.atan2(dy, dx);
  let angleError = angleToTarget - robotTheta;
  angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));

  let curvature = (2*Math.sin(angleError))/dist;
  let vL = baseSpeed*(2 - curvature*wheelBase)/2;
  let vR = baseSpeed*(2 + curvature*wheelBase)/2;

  let v = (vL+vR)/2;
  let omega = (vR-vL)/wheelBase;

  robotX += v*Math.cos(robotTheta);
  robotY += v*Math.sin(robotTheta);
  robotTheta += omega;

  debug = { lookahead: target, distToLookahead: dist,
            angleToTarget: angleToTarget*180/Math.PI,
            angleError: angleError*180/Math.PI,
            curvature, vL, vR };
}

/* ---------- DEBUG PANEL ---------- */
function drawDebugPanel() {
  ctx.fillStyle = "#f4f4f4";
  ctx.fillRect(500,0,400,canvas.height);
  ctx.strokeStyle = "#000";
  ctx.beginPath();
  ctx.moveTo(500,0); ctx.lineTo(500,canvas.height); ctx.stroke();
}

function drawDebug() {
  ctx.fillStyle = "black";
  ctx.font = "13px monospace";
  let x=520, y=20;
  function line(t){ ctx.fillText(t,x,y); y+=16; }

  line("PURE PURSUIT DEBUG"); y+=8;
  line(`Robot X: ${robotX.toFixed(2)}`);
  line(`Robot Y: ${robotY.toFixed(2)}`);
  line(`Heading θ: ${(robotTheta*180/Math.PI).toFixed(2)} deg`);
  line(`Path index: ${currentPathIndex}`); y+=8;
  line(`Lookahead dist: ${lookaheadDistance}`);
  if(!reachedEnd) {
    line(`Target X: ${debug.lookahead[0].toFixed(2)}`);
    line(`Target Y: ${debug.lookahead[1].toFixed(2)}`);
    line(`Dist to target: ${debug.distToLookahead.toFixed(2)}`);
    line(`Angle to target: ${debug.angleToTarget.toFixed(2)} deg`);
    line(`Angle error: ${debug.angleError.toFixed(2)} deg`);
    line(`Curvature κ: ${debug.curvature.toFixed(5)}`);
    line(`Left wheel v: ${debug.vL.toFixed(2)}`);
    line(`Right wheel v: ${debug.vR.toFixed(2)}`);
  } else {
    line("Robot has reached the last point.");
  }
}

/* ---------- LOOP ---------- */
function loop() {
  ctx.clearRect(0,0,canvas.width,canvas.height);
  drawGrid();
  drawPath();
  simulateRobot();
  drawRobot();
  drawDebugPanel();
  drawDebug();
  requestAnimationFrame(loop);
}
loop();
