/*
 * Leap Motion robot arm manipulator
 * Written by Yu Jiang Tham, 2013
 */

/*
 * Variables
 */
var Leap = require('leapjs').Leap;
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

// Arm length in millimeters
var LENGTH1 = 400;
var LENGTH2 = 400;

// Restricted input values (in Leap space).
// You can change these depending on how you build your robot arm.
var MAX_Y = 415;
var MIN_Y = 120;
var MAX_Z = 200;

// How many past frames to cache for smoothing; slows down response time with a higher number
var SMOOTHING_FRAMES = 1;

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

    var smoothedInput = smoothInput(handPosition);
    smoothingQueue(handPosition);

    // Restrict certain inputs to prevent physical damage
    // These values can be changed depending on
    if(smoothedInput.y < MIN_Y) smoothedInput.y = MIN_Y;
    if(smoothedInput.y > MAX_Y) smoothedInput.y = MAX_Y;
    if(smoothedInput.z > MAX_Z) smoothedInput.z = MAX_Z;
    console.log(smoothedInput);

    // Calculate all of the movement angles
    //angles = calculateInverseKinematics(0,-10+handPosition[1]/normalize,handPosition[2]/normalize);
    armAngles = calculateInverseKinematics(smoothInput.y, smoothInput.z);
    baseAngle = calculateBaseAngle(smoothInput.x, smoothInput.z);
    shoulderAngle = armAngles.theta1;
    elbowAngle = armAngles.theta2;
  }

  // Finger distance (of two fingers only) controls the end effector
  if(frame.pointables.length > 1) {
    f1 = frame.pointables[0];
    f2 = frame.pointables[1];
    fingerDistance = distance(f1.tipPosition[0],f1.tipPosition[1],f1.tipPosition[2],f2.tipPosition[0],f2.tipPosition[1],f2.tipPosition[2]);
    clawAngle = (fingerDistance/1.2) - minimumClawDistance;
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
    if(!isNaN(shoulderAngle) && !isNaN(elbowAngle)) {
      servoShoulder.move(shoulderAngle);
      servoElbow.move(elbowAngle);
    } else {
      console.log("Shoulder/Elbow NaN value detected.");
    }
    if(baseAngle >= 0 && baseAngle <= 180) {
      servoBase.move(baseAngle);
    }
    if(clawAngle >= 0 && clawAngle <= 100) {
      servoClaw.move(clawAngle);
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

  for (var i = 0; i < periods, i++) {
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
  /*
  // Massage the input values a bit; salt to taste.
  var n = 100*normalize;
  x = 1.5+2*x/n;

  // 90 Degrees is the center of the base servo, and we want the movement to be
  // nonlinear due to the circular nature of the base.
  var angle = 90+Math.cos(x)*90;
  return angle;
  */

  var angle = Math.tan(x/z);
  return toDegrees(angle);
}

function calculateInverseKinematics(y,z) {
  /*
  // Adjust the input values
  y = y*1.5 + 40;
  z = -z*1.5;

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
  */
  // Get first angle
  var hypotenuse = Math.sqrt(square(y)+square(z))
  var a = Math.atan(z/y);
  var b = Math.acos((square(LENGTH1)+square(hypotenuse)-square(LENGTH2))/(2*LENGTH1*hypotenuse));
  var theta1 = toDegrees(a+b);

  // Get second angle
  var c = Math.acos((square(LENGTH2)+square(LENGTH1)-square(hypotenuse))/(2*LENGTH1*LENGTH2));
  var theta2 = 180 - toDegrees(c);

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
