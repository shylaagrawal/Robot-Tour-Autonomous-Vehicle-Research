function correctDistanceFromObstacle (boxThickness: number) {
    distance = finch.getDistance()
    if(distance > 50){
        return
    }
    if (distance < boxThickness) {
        finch.setMove(MoveDir.Backward, boxThickness - distance, 100)
    }
    if (distance > boxThickness) {
        finch.setMove(MoveDir.Forward, distance - boxThickness, 100)
    }
}

input.onButtonPressed(Button.A, function () {
    finch.startFinch()
    finch.setBeak(255, 0, 30)
    start = input.runningTime()
    finch.setMove(MoveDir.Forward, 28, 75)
    finch.setMove(MoveDir.Forward, 50, speed)     ///1
    finch.setTurn(RLDir.Left, 90, speed)     ///2
    finch.setMove(MoveDir.Forward, 50, speed)     ///3
    finch.setTurn(RLDir.Left, 90, speed)     ///4
    finch.setMove(MoveDir.Forward, 50, speed)     ///5
    finch.setTurn(RLDir.Right, 90, speed)     ///6
    finch.setMove(MoveDir.Forward, 25, speed)     ///7
    finch.setMove(MoveDir.Backward, 25, speed)     ///8
    finch.setTurn(RLDir.Right, 90, speed)     ///9
    finch.setMove(MoveDir.Forward, 50, speed)     ///10
    finch.setTurn(RLDir.Right, 86, speed)     ///11
    finch.setMove(MoveDir.Forward, 100, speed)     ///12
    //finch.setMove(MoveDir.Forward, 50, speed)     ///13
    finch.setTurn(RLDir.Right, 90, speed)     ///14
    finch.setMove(MoveDir.Forward, 50, speed)     ///15
    finch.setTurn(RLDir.Left, 90, speed)     ///16
    finch.setMove(MoveDir.Forward, 50, speed)     ///17
    finch.setTurn(RLDir.Left, 90, speed)     ///18
    finch.setMove(MoveDir.Forward, 30, speed)     ///19
    finch.setMove(MoveDir.Backward, 30, speed)     ///20
    finch.setTurn(RLDir.Left, 90, speed)     ///21
    finch.setMove(MoveDir.Forward, 50, speed)     ///22
    finch.setTurn(RLDir.Right, 90, speed)     ///23
    finch.setMove(MoveDir.Forward, 100, speed)     ///24
    //finch.setMove(MoveDir.Forward, 50, speed)     ///25
    finch.setTurn(RLDir.Right, 90, speed)     ///26
    finch.setMove(MoveDir.Forward, 50, speed)     ///27
    finch.setTurn(RLDir.Left, 90, speed)     ///28
    finch.setMove(MoveDir.Forward, 50, speed)     ///29
    finch.setTurn(RLDir.Left, 90, speed)     ///30
    finch.setMove(MoveDir.Forward, 100, speed)     ///31
    //finch.setMove(MoveDir.Forward, 50, speed)     ///32
    elapsed = input.runningTime() - start
    let targetTime = 19
    if (elapsed / 1000 + 2.717 < targetTime) {
        let remainingTime = targetTime - (elapsed / 1000)
        let forwardTime = 2.717
        let forwardSpeed = 50 / forwardTime
        let forwardSpeedConversionFactor = 90 / forwardSpeed
        let newSlowSpeed = (50 / remainingTime) * forwardSpeedConversionFactor
        newSlowSpeed /= 2;
        if (newSlowSpeed < 5) {
            newSlowSpeed = 5
        }
        finch.setMove(MoveDir.Forward, 50, newSlowSpeed - 1)
    } else {
        finch.setMove(MoveDir.Forward, 50, 90)
    }
    let totalTime = input.runningTime() - start
    basic.showNumber(totalTime)
    basic.showString("!")
    basic.showNumber(totalTime)
    basic.showString("!")
    basic.showNumber(totalTime)
    basic.showString("!")
    finch.stop()
})


function setMove(direction: MoveDir, distance: number = 10, speed: number = 50): void {
    let velocity = 0
    let tick_speed = 0
    let positionControlFlag = 0

    distance = Math.round(finch.capToBounds(distance, 0, 10000) * 49.7)
    if (distance == 0) { return; } //ticks=0 is the motor command for continuous motion. Must exit early so that command is not sent.

    if (direction == MoveDir.Forward) {
        velocity = finch.convertSpeed(speed);
    }
    else if (direction == MoveDir.Backward) {
        velocity = finch.convertSpeed(-speed);
    }
    finch.sendMotor(velocity, distance, velocity, distance)
    basic.pause(50)
    positionControlFlag = finch.getPositionControlFlag()
    while (positionControlFlag == 1) {
        positionControlFlag = finch.getPositionControlFlag()
        basic.pause(15)
    }
    finch.stopMotors()
    basic.pause(0)
}

function setTurn(direction: RLDir, angle: number = 90, speed: number = 50): void {
    let r_speed = 0
    let l_speed = 0
    let positionControlFlag = 0;
    const dist = Math.round(4.335 * finch.capToBounds(angle, 0, 360000))
    if (dist == 0) { return; } //ticks=0 is the motor command for continuous motion. Must exit early so that command is not sent.

    if (direction == RLDir.Left) {
        l_speed = finch.convertSpeed(-speed);
        r_speed = finch.convertSpeed(speed);
    } else {
        l_speed = finch.convertSpeed(speed);
        r_speed = finch.convertSpeed(-speed);
    }

    finch.sendMotor(l_speed, dist, r_speed, dist)
    basic.pause(50)
    positionControlFlag = finch.getPositionControlFlag()
    while (positionControlFlag == 1) {
        positionControlFlag = finch.getPositionControlFlag()
        basic.pause(15)
    }
    finch.stopMotors()
    basic.pause(0)
}

let elapsed = 0
let start = 0
let distance = 0
let speed = 0
// *******MODIFY*********
speed = 93
let speed1 = 90
