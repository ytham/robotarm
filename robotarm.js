/*
 * Leap Motion robot arm manipulator
 * Written by Yu Jiang Tham, 2013
 */

// Variables
var Leap = require('leapjs').Leap;
var five = require('johnny-five');
var board, servoBase, servoShoulder, servoElbow, servoClaw;
var handPosition;
var fingerDistance;
var angles;
var moveBase, moveShoulder, moveElbow, moveClaw;
var frames = [];
var normalize = 3;
var minimumClawDistance = 15;

// Restricted physical movement values.  
// You can change these depending on how you build your robot arm.
var MAX_Y = 415;
var MIN_Y = 120;
var MAX_Z = 200;


/*
 * Need to set up 4 servos: shoulder, elbow, claw, and base
 * Use inverse kinematics equation on servoShoulder & servoElbow
 * Use fingerDistance to rotate servoClaw to desired position
 * Servos must be placed on the Arduino's PWM pins
 */

// Leap motion controller
var controller = new Leap.Controller();

// Main Leap frame loop
controller.on('frame', function(frame) {
  // Hand position controls the robot arm position
  if(frame.hands.length > 0) {
    handPosition = frame.hands[0].palmPosition;

    // Restrict certain inputs to prevent physical damage
    // These values can be changed depending on 
    if(handPosition[1] < 120) handPosition[1] = 120;
    if(handPosition[1] > 415) handPosition[1] = 415;
    if(handPosition[2] > 200) handPosition[2] = 200;

    // Calculate all of the movement angles
    angles = calculateInverseKinematics(0,-10+handPosition[1]/normalize,handPosition[2]/normalize);
    moveBase = 180-calculateBaseAngle(handPosition[0]/1.5);
    moveShoulder = toDegrees(angles.theta1);
    moveElbow = 45+toDegrees(angles.theta2);
  }

  // Finger distance (of two fingers only) controls the end effector
  if(frame.pointables.length > 1) {
    f1 = frame.pointables[0];
    f2 = frame.pointables[1];
    fingerDistance = distance(f1.tipPosition[0],f1.tipPosition[1],f1.tipPosition[2],f2.tipPosition[0],f2.tipPosition[1],f2.tipPosition[2]);
    moveClaw = (fingerDistance/1.2) - minimumClawDistance;
    //frame.pointables[0].tipPosition[0] - frame.pointables[1].tipPosition[0];
    //console.log("fingerDistance: " + fingerDistance);
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
  this.loop(20, function() {
    if(!isNaN(moveShoulder) && !isNaN(moveElbow)) {
      servoShoulder.move(moveShoulder);
      servoElbow.move(moveElbow);
    } else {
      console.log("Shoulder/Elbow NaN value detected.");
    }
    if(moveBase >= 0 && moveBase <= 180) {
      servoBase.move(moveBase);
    }
    if(moveClaw >= 0 && moveClaw <= 100) {
      servoClaw.move(moveClaw);
    }
    console.log("Base: " + Math.floor(moveBase) + "\tShoulder: " + Math.floor(moveShoulder) + "\tElbow: " + Math.floor(moveElbow) + "\tClaw: " + Math.floor(moveClaw));
  });
});


/*
 * Angle Calculation Functions
 */

function calculateBaseAngle(x) {
  // Massage the input values a bit; salt to taste.
  var n = 100*normalize;
  x = 1.5+2*x/n;

  // 90 Degrees is the center of the base servo, and we want the movement to be
  // nonlinear due to the circular nature of the base.
  var angle = 90+Math.cos(x)*90;
  return angle;
}

function calculateInverseKinematics(x,y,z) {
  // Adjust the input values
  y += 60;
  z = -z;
  
  // Normalize the values to mesh with your desired input range
  var l1 = 40*normalize;
  var l2 = 40*normalize;

  // Inverse kinematics equations
  var t1 = Math.acos((square(z)+square(y)-square(l1)-square(l2))/(2*l1*l2));
  var t2 = Math.asin(((l1+l2*Math.cos(t1))*y-l2*Math.sin(t1)*z)/(square(l1)+square(l2)+2*l1*l2*Math.cos(t1)));
  //console.log("[DEBUG] THETA1: " + toDegrees(t1) + "\tTHETA2: " + toDegrees(t2));
  return {
    theta1: t1,
    theta2: t2
  }
}


/*
 * Utility Functions
 */

function distance(x1,y1,z1,x2,y2,z2) {
  return Math.sqrt(square(x2-x1)+square(y2-y1)+square(z2-z1));
}

function square(x) {
  return x*x;
}

function toDegrees(r) {
  return r*57.2957795;
}


// This function has been deprecated.
/*
function calculateInverseKinematics_DEP(x,y,z) {
  var length = 100;
  var c2 = (square(z)+square(y)-square(length)-square(length))/(2*length*length);
  var s2 = Math.sqrt(1-square(c2));
  console.log("\t%c" + "y: " + y + " | z: " + z + " | c2: " + c2);
  var shoulderAngle = Math.acos(c2); // psi
  var elbowAngle = Math.asin((y*(length+length*c2)-z*length*s2)/(square(z)+square(y))); //theta
  //console.log("PSI: " + shoulderAngle*57.295 + "\tTHETA: " + elbowAngle*57.295);
  return {
    shoulderAngle: shoulderAngle,
    elbowAngle: elbowAngle
  }
}*/