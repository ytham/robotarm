/*
 * Leap Motion robot arm manipulator
 * Written by Yu Jiang Tham, 2013
 */

var Leap = require('leapjs').Leap;
var five = require('johnny-five');
var board, servoBase, servoShoulder, servoElbow, servoClaw;
var handPosition;
var fingerDistance;
var angles;
var moveBase, moveShoulder, moveElbow, moveClaw;
var frames = [];
var normalize = 3;
var minimumClawDistance = 10;
var l1 = 40*normalize;
var l2 = 40*normalize;

/*
 * Need to set up 4 servos: shoulder, elbow, claw, and base
 * Use inverse kinematics equation on servoShoulder & servoElbow
 * Use fingerDistance to rotate servoClaw to desired position
 */

// Leap motion controller
var controller = new Leap.Controller();

// Main Leap frame loop
controller.on('frame', function(frame) {
  if(frame.hands.length > 0) {
    handPosition = frame.hands[0].palmPosition;
    angles = calculateInverseKinematics(0,-10+handPosition[1]/normalize,handPosition[2]/normalize);
    moveBase = 180-calculateBaseAngle(handPosition[0]/2);
    moveShoulder = toDegrees(angles.theta1);
    moveElbow = 45+toDegrees(angles.theta2);
  }
  if(frame.pointables.length > 1) {
    f1 = frame.pointables[0];
    f2 = frame.pointables[1];
    fingerDistance = distance(f1.tipPosition[0],f1.tipPosition[1],f1.tipPosition[2],f2.tipPosition[0],f2.tipPosition[1],f2.tipPosition[2]);
    moveClaw = (fingerDistance/1.5) - minimumClawDistance;
  }
  frames.push(frame);
});

// Leap Motion connected
controller.on('connect', function(frame) {
  console.log("Leap Connected.");
  setTimeout(function() { 
    var time = frames.length/2;
  }, 200);
});

controller.connect();


// Johnny-Five controller
board = new five.Board();
board.on('ready', function() {
  servoBase = new five.Servo(3);
  servoShoulder = new five.Servo(9);
  servoElbow = new five.Servo(10); 
  servoClaw = new five.Servo(6);

  // Initial positions of the robot arm
  servoBase.center();
  servoShoulder.center();
  servoElbow.move(45);
  servoClaw.move(100);

  // Move each component
  this.loop(40, function() {
    if(!isNaN(moveShoulder) && !isNaN(moveElbow)) {
      servoBase.move(moveBase);
      servoShoulder.move(moveShoulder);
      servoElbow.move(moveElbow);
    }
    if(moveClaw >= 0 && moveClaw <= 100) {
      servoClaw.move(moveClaw);
    }
    console.log("Base: " + Math.floor(moveBase) + "\tShoulder: " + Math.floor(moveShoulder) + "\tElbow: " + Math.floor(moveElbow) + "\tClaw: " + Math.floor(moveClaw));
  });
});

function calculateBaseAngle(x) {
  var n = 100*normalize;
  x = 1.5+2*x/n;
  var angle = 90+Math.cos(x)*90;
  return angle;
}

function calculateInverseKinematics(x,y,z) {
  z = -z;
  var t1 = Math.acos((square(z)+square(y)-square(l1)-square(l2))/(2*l1*l2));
  var t2 = Math.asin(((l1+l2*Math.cos(t1))*y-l2*Math.sin(t1)*z)/(square(l1)+square(l2)+2*l1*l2*Math.cos(t1)));
  return {
    theta1: t1,
    theta2: t2
  }
}

function distance(x1,y1,z1,x2,y2,z2) {
  return Math.sqrt(square(x2-x1)+square(y2-y1)+square(z2-z1));
}

function square(x) {
  return x*x;
}

function toDegrees(r) {
  return r*57.2957795;
}
