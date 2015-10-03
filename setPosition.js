var five = require('johnny-five');

// Board and servos for Johnny-Five
var board, servoBase, servoShoulder, servoElbow, servoClaw;

// PWM Pins
var PIN_BASE = 3;
var PIN_SHOULDER = 9;
var PIN_ELBOW = 10;
var PIN_CLAW = 6;

// Position to move to (passed in via command line); otherwise move to zero.
var POSITION = process.argv[2] || 0;

board = new five.Board();
board.on('ready', function () {
  servoBase = new five.Servo(PIN_BASE);
  servoShoulder = new five.Servo(PIN_SHOULDER);
  servoElbow = new five.Servo(PIN_ELBOW);
  servoClaw = new five.Servo(PIN_CLAW);

  console.log("Moving all servos to %s", POSITION);

  servoBase.move(POSITION);
  servoShoulder.move(POSITION);
  servoElbow.move(POSITION);
  servoClaw.move(POSITION);
});
