// /////////////////////////////////////////////////////////////////////////////
//
// blast-bot Abrasive Blasting Robot | The Living | 2016
//
// /////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
// DESCRIPTION
//------------------------------------------------------------------------------
// The blast-bot is a 2-axis cartesian robot for CNC abrasive blasting.
// Movement occurs horizontally along the X-axis and vertically along
// the Y-axis. Linear motion is achieved through pulley-driven timing
// belts. Each axis is actuated through paired stepper motors driven by
// slaved stepper drivers -- a single step and direction signal is shared
// by both drivers/motors.


//------------------------------------------------------------------------------
// ACKNOWLEDGEMENTS
//------------------------------------------------------------------------------
// Serial communication and GCODE parsing
// inspired by DrawBot robot by dan@marginallyclever.com
// http://www.github.com/MarginallyClever/Makelangelo


//------------------------------------------------------------------------------
// EXTERNAL DEPENDENCIES
//------------------------------------------------------------------------------

//AccelStepper and MultiStepper libraries
//For stepper motor control
//http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
#include <AccelStepper.h>
#include <MultiStepper.h>

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//VERBOSE mode for debugging
#define VERBOSE 		(0) // 0: False | 1: True

//define serial communication protocol
#define BAUD          	(57600) //serial comm bitrate
#define MAX_BUF       	(64) //serial input buffer size
#define TIMEOUT_OK    	(1000) //timeout length

//Define motor pins
//MX - X-axis motors
//stepper drivers are paired and recieve
//identical step & dir signals
//and move in the same direction

#define MX_STEP       	(2) //MX Stepping Signal
#define MX_DIR        	(3) //MX Direction Signal

//MZ - Z-axis motors
//stepper drivers are paired and recieve
//identical step & dir signals
//and move in opposite directions
#define MY_STEP       	(4) //MZ Stepping Signal
#define MY_DIR        	(5) //MZ Direction Signal

// BLAST CONTROL
// Pin for controlling blasting on/off toggle
#define BLAST		(8)

//define motor specs
#define STEPS_PER_TURN  (200) // Steps per full revolution
#define MIN_FEEDRATE	(1) // 1 Step / Second
#define MAX_FEEDRATE	(10000) //10,000 Steps / Second (Upper Limit of Arduino)

//define linear motion specs
#define BELT_PITCH 		(3.0) //GT3 Timing Bel
#define PULLEY_TEETH_X 	(20)
#define PULLEY_TEETH_Y 	(20)
#define BELT_REDUCTION_X	(1)
#define BELT_REDUCTION_Y	(2)

//define forward motor direction
// 0 == clockwise
// 1 == counterclockwise
#define MX_FORWARD    	(1) //MX Forward Direction
#define MY_FORWARD   	(0) //MZ Forward Direction

//define geometry motions and resolution
//for arc directions
#define ARC_CW      	(1)
#define ARC_CCW     	(-1)

//arcs and lines are constructed by subdivision
//into line segments define length of segment
//for subdivision (in mm)
#define MM_PER_SEGMENT  (1.0)

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

// initialize stepper motors as AccelStepper objects
// http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
static AccelStepper mx( 1, MX_STEP, MX_DIR );
static AccelStepper my( 1, MY_STEP, MY_DIR );

static int microstep_x = 8; // Microsteps per full step on X-axis
static int microstep_y = 8; // Microsteps per full step on Z-axis 

static bool accel = 0; // acceleration toggle 0: Off  1: On

// Movement Bias
// If microstepping and/or belt reduction doesnt match between
// axes, then speed will be biased along a particular direction
// Disabling bias will clamp maximum feedrate to slowest axis
static bool bias = 1; // 0: unbiased, equal movement 1: biased movement

static int max_speed_x; // max speed along X-axis (mm/s)
static int max_speed_y; // max speed along Y-axis (mm/s)
static int feed_rate_x; // max speed along X-axis (step/s)
static int feed_rate_y; // max speed along Y-axis (step/s)

static int motor_speed_x = 800; //steps per s
static int motor_accel_x = 400;
static int motor_speed_y = 800; //steps per s
static int motor_accel_y = 400;
static int jog_dist = 10;
static int jogxl_dist = 100;

//initialize Multistepper object
// http://www.airspayce.com/mikem/arduino/AccelStepper/classMultiStepper.html
static MultiStepper steppers;

// Blasting Dimensions
// all distances are measured relative to the calibration point of the plotter
// (normally, this is located in the center of the blasting space)
// measurement is in millimeters
// -----------------------------
// |             X             | <-- X = ORIGIN
// -----------------------------

static float limit_top = 305.0; //distance to top blast limit
static float limit_bottom = -305.0; //distance to bottom blast limit
static float limit_right = 915.0; //distance to right blast limit
static float limit_left = -915.0; //distance to left blast limit

static boolean hardLimits = false;

// set pulley feed rates (mm/step)
static float dist_per_step_X;
static float dist_per_step_Y;

//plotter position
static float posx, posy;

//serial communication reception
static char buffer[ MAX_BUF + 1 ]; // Serial buffer
static int sofar; //Serial buffer progress counter
static long last_cmd_time; //timeout counter
long line_number; //current line in multi-line GCODE

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

// MACHINE SETTING FUNCTIONS
//------------------------------------------------------------------------------
static void getStepDist(){
	
	//dist = (pulley teeth) * (belt_pitch) / (steps per turn * reduction * microstep)
	dist_per_step_X = (PULLEY_TEETH_X * BELT_PITCH) / (STEPS_PER_TURN * BELT_REDUCTION_X * microstep_x);
	dist_per_step_Y = (PULLEY_TEETH_Y * BELT_PITCH) / (STEPS_PER_TURN * BELT_REDUCTION_Y * microstep_y);

}

static void calcMaxSpeed(){
	// calculate the maximum speed (in mm/s) achievable with hardware
	// max speed is limited by clock-rate of Arduino which can output
	// approx. 10,000 steps / second

	max_speed_x = MAX_FEEDRATE * dist_per_step_X;
	max_speed_y = MAX_FEEDRATE * dist_per_step_Y;
}

static void calcFeedrate( float x, float y, float f, float &vx, float &vy){
	//convert linear velocity to component axial velocities

	float dx = abs( posx - x );
	float dy = abs( posy - y );

	float vec = sqrt( dx * dx + dy * dy);

	vx = f * ( dx / vec ); // cosine of movement vector angle
	vy = f * ( dy / vec ); // sine of movement vector angle
}

static void setFeedrate( float x_speed, float y_speed, float x_accel, float y_accel ){
	// set max speed for each motor
	// limit to max calculated speed for microstepping state
	x_speed = min( x_speed, max_speed_x );
	y_speed = min(y_speed, max_speed_y);

	// set new max speed (used by Multistepper object in move planning)
	mx.setMaxSpeed( x_speed );
    my.setMaxSpeed( y_speed );
    
    // limit acceleration to current speed
    mx.setAcceleration( min(x_accel, x_speed) );
    my.setAcceleration( min(y_accel, y_speed) );
}

// TRIGONOMETRIC FUNCTIONS
//------------------------------------------------------------------------------
static float atan3( float dy, float dx ) {
  
  //return angle of dy/dx as a value from 0 to 2PI
  float a = atan2( dy, dx );
  if ( a < 0 ) a = ( PI * 2.0 ) + a;
  return a;

}

// KINEMATICS
//------------------------------------------------------------------------------
static void IK(float x, float y, long &m1, long &m2){
	//inverse kinematics - stepper position from x,y position	
	//divide x-position
	m1 = floor(x / dist_per_step_X);
	m2 = floor(y / dist_per_step_Y);

}

static void FK(float l1, float l2, float &x, float &y){
	//forward kinematics - x,y position from stepper position
	l1 *= dist_per_step_X;
	l2 *= dist_per_step_Y;

	x = l1;
	y = l2;
}

// MOVEMENT
//------------------------------------------------------------------------------

void pause( long ms ) {

  delay( ms / 1000 ); //whole second delay
  delayMicroseconds( ms % 1000 ); //microsecond delay for remainder

}

static void line( float x, float y) {
  //Line drawing method

  if (hardLimits){
  	//enforce hard limits to prevent machine from moving beyond workspace
  	if (x > limit_right or x < limit_left or y > limit_top or y < limit_bottom){
  		Serial.println("OUTOFBOUNDS");
  		return; // escape out of command
  	}
  }

  long positions[2]; //array of desired string lengths
  IK( x, y, positions[0], positions[1]);
  
  steppers.moveTo( positions ); //set positions for each motor

  steppers.runSpeedToPosition(); //Blocks until all steppers are in position

  //update current position
  posx = x;
  posy = y;
  
}

static void line_safe( float x, float y ) {
  //Subdivided line drawing method

  //measure length of line to be drawn
  float dx = x - posx;
  float dy = y - posy;

  float len = sqrt( dx * dx + dy * dy );

  // check if line is within allowable resolution
  if ( len <= MM_PER_SEGMENT ) {
    //if so, draw line
    line( x, y );
    return;
  }

  // if too long, subdivide into smaller segments
  long pieces = floor( len / MM_PER_SEGMENT );
  float x0 = posx;
  float y0 = posy;
  float a;

  //draw sequential line segments
  for ( long j = 1; j < pieces; j++ ) {
    a = (float)j / (float)pieces;

    line( (x - x0)*a + x0, (y - y0)*a + y0 );
  }

  line( x, y ); //draw final line segment

}

static void arc( float cx, float cy, float x, float y, float dir ) {
  //Arc drawing method
  //Assumes fixed radius and max angle of 180 degrees (PI radians)

  //determine radius
  float dx = posx - cx;
  float dy = posy - cy;
  float radius = sqrt( dx * dx + dy * dy );

  //determine angle of arc (sweep)
  float angle1 = atan3( dy, dx );
  float angle2 = atan3( y - cy, x - cx );
  float theta = angle2 - angle1;

  if ( dir > 0 && theta < 0 ) angle2 += 2 * PI;
  else if ( dir < 0 && theta > 0) angle1 += 2 * PI;

  theta = angle2 - angle1;

  //determine length of arc
  float len = abs(theta) * radius;

  //subdivide arc into safe line segments
  int i, segments = ceil( len / MM_PER_SEGMENT );

  float nx, ny, angle3, scale;

  for ( i = 0; i < segments; ++i ) {
    //interpolate line segments around the arc
    scale = ((float)i) / ((float)segments);

    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;

    //send to line command
    line( nx, ny );
  }

  //draw final line segment
  line( x, y );
}

static void teleport( float x, float y ) {
  //Position reset method (no movement)

  posx = 0;
  posy = 0;

  //calculate stepper positions from coordinates
  long L1, L2;
  IK( posx, posy, L1, L2 );

  mx.setCurrentPosition(L1);
  my.setCurrentPosition(L2);

}

void where(){
  // Report x,y position
  // and motor step positions
  // and speed settings

  Serial.print( "POS_X: ");
  Serial.print( posx );
  Serial.print( " POS_Y: ");
  Serial.println( posy );

  Serial.print( "MX: " );
  Serial.print( mx.currentPosition() );
  Serial.print( " MY: " );
  Serial.println( my.currentPosition() );

  Serial.print( "FEED_X: ");
  Serial.print( motor_speed_x );
  Serial.print( " FEED_Y: " );
  Serial.println( motor_speed_y );
  Serial.print( "ACCEL_X: " );
  Serial.print( motor_accel_x );
  Serial.print( " ACCEL_Y: ");
  Serial.println( motor_accel_y );


}

// COMMAND METHODS
//------------------------------------------------------------------------------

float parsenumber( char code, float val ) {
  //method for parsing GCODE command type

  char *ptr = buffer;
  while ( ptr && *ptr && ptr < buffer + sofar ) {
    if ( *ptr == code ) {
      return atof( ptr + 1 );
    }
    ptr = strchr( ptr, ' ') + 1;
  }
  return val;
}

static void processCommand() {
  //method for parsing GCODE commands

  //skip blank lines
  if ( buffer[0] == ';' ) return;

  long cmd;

  // SEQUENCE CHECKS
  //---------------------------
  cmd = parsenumber( 'N', -1);
  //check for line number
  //line number must appear first on the line
  if ( cmd != -1 && buffer[0] == 'N' ) {
    if ( cmd != line_number ) {
      //indicate wrong line number and return
      Serial.print( "BADLINENUM " );
      Serial.println( line_number );
      return;
    }

    //check for checksum
    if ( strchr( buffer, '*' ) != 0 ) {
      //check validity
      unsigned char checksum = 0;
      int c = 0;
      while ( buffer[c] != '*' && c < MAX_BUF ) checksum ^= buffer[ c++ ];
      c++; //skip checksum indicator (*)
      unsigned char against = (unsigned char)strtod( buffer + c, NULL );
      if ( checksum != against ) {
        //indicate wrong checksum and return
        Serial.print( "BADCHECKSUM " );
        Serial.println( line_number );
        return;
      }
    } else {
      Serial.print( "NOCHECKSUM " );
      Serial.println( line_number );
      return;
    }

    //indicate command recieved and OK
    if( VERBOSE ){
    	Serial.print( "OK " );
    	Serial.println( line_number );
    	line_number++;
    }
  }

  // MACHINE SETTINGS COMMANDS
  //---------------------------
  cmd = parsenumber( 'D', -1 );
  switch ( cmd ) {
    case 1: { //update machine dimensions
        limit_top = parsenumber( 'T', limit_top );
        limit_bottom = parsenumber( 'B', limit_bottom );
        limit_right = parsenumber( 'R', limit_right );
        limit_left = parsenumber( 'L', limit_left );

        if( VERBOSE ){
        	Serial.println( "LIMITS" );
        	Serial.print( "T:" );
        	Serial.print( limit_top );
        
        	Serial.print( "B:" );
        	Serial.print( limit_bottom );
        
        	Serial.print( "R:" );
        	Serial.print( limit_right );
        
        	Serial.print( "L:" );
        	Serial.println( limit_left );
        }
        

        teleport( posx, posy ); //update motor positions

        break;
      }

    case 5: { //update microstep settings
        microstep_x = parsenumber( 'X', microstep_x );
        microstep_y = parsenumber( 'Y', microstep_y );

        if( VERBOSE ){
        	Serial.println( "MICROSTEPPING"  );
        	Serial.print( "X: " );
        	Serial.print( microstep_x );
        	Serial.print( " Y: ");
        	Serial.println( microstep_y );
        }

        break;
      }

    case 10: { //Update motor speeds & acceleration
        motor_speed_x = min( parsenumber( 'X', motor_speed_x ), MAX_SPEED);
        motor_speed_y = min( parsenumber( 'Y', motor_speed_y ), MAX_SPEED);
        motor_accel_x = min( parsenumber( 'A', motor_accel_x ), motor_speed_x);
        motor_accel_y = min( parsenumber( 'B', motor_accel_y ), motor_speed_y);
        
        setFeedrate(motor_speed_x, motor_speed_y, motor_accel_x, motor_accel_y);
        
        if( VERBOSE ){
        	Serial.println( "SPEED/ACCEL" );
        	Serial.print( "X: ");
        	Serial.print( motor_speed_x );
        	Serial.print( "/" );
        	Serial.print( motor_accel_x);
        	Serial.print( " Y: " );
        	Serial.print( motor_speed_y );
        	Serial.print( "/" );
        	Serial.println( motor_accel_y );
        }
        
        break;
      }
      
    case 20: //UNASSIGNED

    case 30: { //report position
        where();
        break;
      }
  }


  // MOTOR OVERRIDE COMMANDS
  //---------------------------
  cmd = parsenumber( 'M', -1 );
  switch ( cmd ) {
    case 0: // UNUSED
    case 1: // UNUSED

    case 10: {
        mx.move(jog_dist * parsenumber( 'S', 1 )); mx.runToPosition(); break; //jog M1 stepper forward
      }
    case 11: {
        mx.move(-jog_dist * parsenumber( 'S', 1 )); mx.runToPosition(); break; //jog M1 stepper forward
      }

    case 20: {
        my.move(jog_dist * parsenumber( 'S', 1 )); my.runToPosition(); break; //jog M2 stepper forward
      }
    case 21: {
        my.move(-jog_dist * parsenumber( 'S', 1 )); my.runToPosition(); break; //jog M2 stepper forward
      }

    case 100: { //M100 - set stepper positions manually
        teleport( parsenumber( 'X', 0 ), parsenumber( 'Y', 0 ) );
        break;
      }
  }

  // MOVE/DRAW COMMANDS
  //---------------------------
  cmd = parsenumber( 'G', -1 );
  switch (cmd) {
    case 0: //move command (no spray)
    case 1: { //G01 - Line command

        //enable sprayer if G01 command
        if ( cmd == 1) {
          digitalWrite(BLAST, 1);
        } else {
          digitalWrite(BLAST, 0);
        }

        line_safe( parsenumber( 'X', posx ), parsenumber( 'Y', posy ) );

        //disengage sprayer
        digitalWrite(BLAST, 0);
        break;
      }
    case 2: //G02 - CW Arc command
    case 3: { //G03 - CCW Arc command
        //enable sprayer
        digitalWrite(BLAST, 1);

        arc( parsenumber( 'I', posx ),
             parsenumber( 'J', posy ),
             parsenumber( 'X', posx ),
             parsenumber( 'Y', posy ),
             ( cmd == 2 ) ? ARC_CCW : ARC_CW );

        //disable sprayer
        digitalWrite(BLAST, 0);

        break;
      }
    case 4: { //G04 - Pause command
        pause( parsenumber( 'S', 0 ) + parsenumber( 'P', 0 ) * 1000.0f );
        break;
      }
  }
}

void ready() {
  //Prepares input buffer for new messages and
  //signals serial device it is ready for a new command

  sofar = 0; //reset input buffer position
  Serial.print( "\n> " );
  last_cmd_time = millis();
}

void Serial_listen() {
  //Method for listening for commands over serial
  while ( Serial.available() > 0 ) {
    char c = Serial.read();
    if ( sofar < MAX_BUF ) buffer[ sofar++ ] = c;
    if ( c == '\n' || c == '\r') {
      buffer[sofar] = 0;

      //echo command
      //Serial.println(buffer);

      processCommand();
      ready();
      break;
    }
  }
}

//------------------------------------------------------------------------------
//SETUP
//------------------------------------------------------------------------------
void setup() {

  //pin settings
  pinMode(MX_STEP, OUTPUT);
  pinMode(MX_DIR, OUTPUT);

  pinMode(MY_STEP, OUTPUT);
  pinMode(MY_DIR, OUTPUT);

  pinMode(BLAST, OUTPUT);

  //initialize serial read buffer
  sofar = 0;
  Serial.begin( BAUD );
  Serial.println( F("\n\nHELLO WORLD! I AM BLAST-BOT"));

  // set stepper directions
  mx.setPinsInverted(MX_FORWARD, false, false); //direction inversion for X
  my.setPinsInverted(MY_FORWARD, false, false); //direction inversion for Y

  // set stepper speed & acceleration
  setFeedrate( motor_speed_x, motor_speed_y, motor_accel_x, motor_accel_y);

  // add steppers to MultiStepper object
  steppers.addStepper( mx );
  steppers.addStepper( my );

  //initialize plotter positions
  getStepDist();
  
  if( VERBOSE ){
  	where();
  }
 
  teleport( 0, 0 );
   
  //LET'S GO!
  Serial.println( "...AND I AM READY TO BLAST");
  ready();
}

//------------------------------------------------------------------------------
//LOOP
//------------------------------------------------------------------------------
void loop() {
  Serial_listen();

  //TIMEOUT CHECK
  //If Arduino hasn't recieved new instructions in a while,
  //send a ready() signal again
  if ( millis() - last_cmd_time > TIMEOUT_OK ) {
    ready();
  }
}
