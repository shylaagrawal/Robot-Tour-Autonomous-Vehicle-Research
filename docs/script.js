<canvas id="gridCanvas" width="500" height="400"></canvas>
<button id="runBtn">Run</button>

<script>
const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");

const ROWS = 4;
const COLS = 5;
const CELL_SIZE = 100;

// --- Robot parameters ---
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

// --- Instrumentation state ---
let debug = {
    lookahead: [0,0],
    curvature: 0,
    vL: 0,
    vR: 0,
    angleToTarget: 0,
    angleError: 0,
    distToLookahead: 0
};

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

// ---------- Pure Pursuit ----------
function getLookaheadPoint(){
    for(let i=currentPathIndex;i<path.length-1;i++){
        const [x1,y1] = path[i];
        const [x2,y2] = path[i+1];

        const dx = x2
