/*
 * Leap Motion robot arm manipulator
 * Written by Yu Jiang Tham, 2013
 * And re-written in 2015!
 */

/*
 * Variables
 */
var Leap = require('leapjs');
var five = require('johnny-five');

// Board and servos for Johnny-Five
var board, servoBase, servoShoulder, servoElbow, servoClaw;

// Position variables for the Leap
var handPosition;
var handHistory = [];
var fingerDistance;
var armAngles;

// Movement variables
var baseAngle, shoulderAngle, elbowAngle, clawAngle;
var frames = [];

/*
 * Settings
 */
var normalize = 3;
var minimumClawDistance = 15;
var boardOptions = { port: '/dev/cu.usbmodemfa131' };

// PWM Pins
var PIN_BASE = 3;
var PIN_SHOULDER = 9;
var PIN_ELBOW = 10;
var PIN_CLAW = 6;

// Arm length in millimeters
var LENGTH1 = 160;
var LENGTH2 = 160;

// Leap input zero point for calculating inverse kinematics

// Restricted input values (in Leap space).
// You can change these depending on how you build your robot arm.
var MAX_Y = 415;
var MIN_Y = 0;
var MIN_Z = 0;
var MAX_Z = 800;

// How many past frames to cache for smoothing; slows down response time with a higher number
var SMOOTHING_FRAMES = 10;

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

    // Offset z to always keep the value positive
    frame.hands[0].palmPosition[1] -= 150;
    frame.hands[0].palmPosition[2] = 200 + (-1*frame.hands[0].palmPosition[2]);

    var smoothedInput = smoothInput(handPosition);
    smoothingQueue(handPosition);

    // Restrict certain inputs to prevent physical damage
    // These values should be changed depending on your arm design
    if(smoothedInput.y < MIN_Y) smoothedInput.y = MIN_Y;
    if(smoothedInput.y > MAX_Y) smoothedInput.y = MAX_Y;
    if(smoothedInput.z < MIN_Z) smoothedInput.z = MIN_Z;
    if(smoothedInput.z > MAX_Z) smoothedInput.z = MAX_Z;
    // console.log(smoothedInput);

    // Calculate all of the movement angles
    //angles = calculateInverseKinematics(0,-10+handPosition[1]/normalize,handPosition[2]/normalize);
    armAngles = calculateInverseKinematics(smoothedInput.y, smoothedInput.z);
    baseAngle = calculateBaseAngle(smoothedInput.x, smoothedInput.z);
    shoulderAngle = armAngles.theta1;
    elbowAngle = armAngles.theta2;
  }

  // Finger distance (of two fingers only) controls the end effector
  if(frame.pointables.length > 1) {
    f1 = frame.pointables[0];
    f2 = frame.pointables[1];
    fingerDistance = distance(f1.tipPosition[0],f1.tipPosition[1],f1.tipPosition[2],f2.tipPosition[0],f2.tipPosition[1],f2.tipPosition[2]);
    clawAngle = 120-fingerDistance;
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
  servoBase = new five.Servo(PIN_BASE);
  servoShoulder = new five.Servo(PIN_SHOULDER);
  servoElbow = new five.Servo(PIN_ELBOW);
  servoClaw = new five.Servo(PIN_CLAW);

  // Initial positions of the robot arm
  servoBase.to(90);
  servoShoulder.to(90);
  servoElbow.to(45);
  servoClaw.to(40);

  // Move each component
  this.loop(30, function() {
    if(!isNaN(shoulderAngle) && !isNaN(elbowAngle)) {
      servoShoulder.to(shoulderAngle);
      servoElbow.to(elbowAngle);
    } else {
      //console.log("Shoulder/Elbow NaN value detected.");
    }
    if(baseAngle >= 0 && baseAngle <= 180) {
      servoBase.to(baseAngle);
    }
    if(clawAngle >= 20 && clawAngle <= 70) {
      servoClaw.to(clawAngle);
    }
    console.log("Base: " + Math.floor(baseAngle) + "\tShoulder: " + Math.floor(shoulderAngle) + "\tElbow: " + Math.floor(elbowAngle) + "\tClaw: " + Math.floor(clawAngle));
  });
});


/*
 * Smoothing the input - takes the average value of SMOOTHING_FRAMES number of frames
 */
function smoothInput(current) {
  if (handHistory.length === 0) {
    return current;
  }

  var x = 0, y = 0, z = 0;
  var periods = handHistory.length;

  for (var i = 0; i < periods; i++) {
    x += current[0] + handHistory[i][0];
    y += current[1] + handHistory[i][1];
    z += current[2] + handHistory[i][2];
  }

  periods += 1; // To incldue the current frame
  return {x: x/periods, y: y/periods, z: z/periods};
}

function smoothingQueue(current) {
  handHistory.unshift(current);
  if (handHistory.length > SMOOTHING_FRAMES) {
    handHistory.pop();
  }
}


/*
 * Angle Calculation Functions
 */

function calculateBaseAngle(x,z) {
  var angle = Math.tan(x/z);
  return 90 - toDegrees(angle);
}

function calculateInverseKinematics(y,z) {
  var hypotenuse = Math.sqrt(square(y)+square(z));
  var a = Math.atan(y/z);
  var b = Math.acos((square(LENGTH1)+square(hypotenuse)-square(LENGTH2))/(2*LENGTH1*hypotenuse));
  var theta1 = toDegrees(a+b);

  // Get second angle
  var c = Math.acos((square(LENGTH2)+square(LENGTH1)-square(hypotenuse))/(2*LENGTH1*LENGTH2));
  var theta2 = 180 - toDegrees(c);
  // console.log("t1: %s\tt2: %s", theta1, theta2);
  return {
    theta1: theta1,
    theta2: theta2
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
