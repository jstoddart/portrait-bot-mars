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
// by both drivers/motors


//------------------------------------------------------------------------------
// ACKNOWLEDGEMENTS
//------------------------------------------------------------------------------
// Serial communication and GCODE parsing
// inspired by DrawBot robot by dan@marginallyclever.com
// http://www.github.com/MarginallyClever/Makelangelo


//------------------------------------------------------------------------------
// EXTERNAL DEPENDENCIES
//------------------------------------------------------------------------------

// AccelStepper and MultiStepper libraries
// For stepper motor control
// http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
#include <AccelStepper.h>
#include <MultiStepper.h>

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
// VERBOSE mode for debugging
#define VERBOSE 		(0) // 0: False | 1: True

// Define serial communication protocol
// These control communication with the software issuing
// GCODE commands
#define BAUD          	(115200) // Serial comm bitrate
#define MAX_BUF       	(64) // Serial input buffer size
#define TIMEOUT_OK    	(200) // Timeout length

// STEPPER MOTOR CONTROLS
// MX - X-axis motors
// Stepper drivers are paired and recieve
// identical step & dir signals
// and move in the same direction
#define MX_STEP       	(2) // MX Stepping Signal
#define MX_DIR        	(3) // MX Direction Signal

// MY - Y-axis motors
// Stepper drivers are paired and recieve
// identical step & dir signals
// and move in opposite directions
#define MY_STEP       	(5) //MY Stepping Signal
#define MY_DIR        	(6) //MY Direction Signal

// BLAST CONTROL
// Pin for controlling blasting on/off toggle
#define BLAST		(12)
#define AIR     (13)

// Define stepper motor specs
#define STEPS_PER_TURN  (200) // Steps per full revolution
#define MIN_FEEDRATE	(1) // 1 Step / Second
#define MAX_FEEDRATE	(10000) //10,000 Steps / Second (Upper Limit of Arduino)
#define TOPSPEED_X (300.0) //Abs. Max speed along X-AXIS
#define TOPSPEED_Y (80.0) //Abs. Max speed along Y-AXIS


#define BELT_PITCH 		(3.0) // GT3 Timing Belt
#define PULLEY_TEETH_X 	(20)
#define PULLEY_TEETH_Y 	(20)
#define BELT_REDUCTION_X	(1)
#define BELT_REDUCTION_Y	(1)

// Define forward motor direction
// 0 == clockwise
// 1 == counterclockwise
#define MX_FORWARD    	(1) // MX Forward Direction
#define MY_FORWARD   	(1) // MZ Forward Direction

// Define geometry motions and resolution
// for arc directions
#define ARC_CW      	(1)
#define ARC_CCW     	(-1)

// Arcs and lines are constructed by subdivision
// into line segments define length of segment
// for subdivision (in mm)
#define MM_PER_SEGMENT  (1.0)

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

// Initialize stepper motors as AccelStepper objects
// http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
static AccelStepper mx( 1, MX_STEP, MX_DIR ); // AccelStepper object for MX
static AccelStepper my( 1, MY_STEP, MY_DIR ); // AccelStepper object for MY

static int microstep_x = 2; // Microsteps per full step on X-axis
static int microstep_y = 8; // Microsteps per full step on Z-axis 

static float feed_x; // Feedrate along x-axis (step/s)
static float feed_y; // Feedrate along y-axi (step/s)

static bool accel = 0; // Enable (1) / Disable (0) acceleration
static bool bias = 1; // Enable (1) / Disable (0) movement bias

static float feedrate = 20; // Desired feedrate (mm/s)

static float x_limit = 200.0;
static float y_limit = 80.0;
static int max_speed_x; // Max speed along X-axis (mm/s)
static int max_speed_y; // Max speed along Y-axis (mm/s)

static int jog_dist = 10; // Steps to rotate for a jog command

// Pulley feed rates (mm/step)
static float dist_per_step_X;
static float dist_per_step_Y;

// Initialize Multistepper object
// http://www.airspayce.com/mikem/arduino/AccelStepper/classMultiStepper.html
static MultiStepper steppers;

// Blasting Dimensions
// All distances are measured relative to the calibration point of the plotter
// (normally, this is located in the center of the blasting space)
// measurement is in millimeters
// -----------------------------
// |             X             | <-- X = ORIGIN
// -----------------------------

static float limit_top = 550.0; // Distance to top blast limit
static float limit_bottom = -100.0; // Distance to bottom blast limit
static float limit_right = 1985.0; // Distance to right blast limit
static float limit_left = 0.0; // Distance to left blast limit

static bool hardLimits = 1; // Disable (0) / Enable (1) Movement Constraints

// Plotter position
static float posx, posy;

// Serial communication reception
static char buffer[ MAX_BUF + 1 ]; // Serial buffer
static int sofar; // Serial buffer progress counter
static long last_cmd_time; // Timeout counter
long line_number; // Current line in multi-line GCODE

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

// MACHINE SETTING FUNCTIONS
//------------------------------------------------------------------------------
static void getStepDist(){
	
	// DIST = (PULLEY-TEETH) * (BELT-PITCH) / (STEPS-PER-TURN * BELT-REDUCTION * MICROSTEP)
	dist_per_step_X = (PULLEY_TEETH_X * BELT_PITCH) / (STEPS_PER_TURN * BELT_REDUCTION_X * microstep_x);
	dist_per_step_Y = (PULLEY_TEETH_Y * BELT_PITCH) / (STEPS_PER_TURN * BELT_REDUCTION_Y * microstep_y);

  if( VERBOSE ){
    Serial.print( "DIST PER STEP X: ");
    Serial.println( dist_per_step_X );
    Serial.print( "DIST PER STEP Y: ");
    Serial.println( dist_per_step_Y );
  }

}

static void calcMaxSpeed(){
	// Calculate the maximum speed (in mm/s) achievable with hardware
	// Max speed is limited by clock-rate of Arduino which can output
	// approx. 10,000 steps / second

	max_speed_x = min( MAX_FEEDRATE * dist_per_step_X, TOPSPEED_X);
	max_speed_y = min( MAX_FEEDRATE * dist_per_step_Y, TOPSPEED_Y);

  // MOVEMENT BIAS
  // If disabled, limits speeds to slowest axis
  // Will guarantee constant speed regardless of direction of movement
  if (!bias){
    max_speed_x = min(max_speed_x, max_speed_y);
    max_speed_y = min(max_speed_y, max_speed_x);
  }

  if( VERBOSE ){
    Serial.print("MAX SPEED X: ");
    Serial.println( max_speed_x );
    Serial.print("MAX SPEED Y: ");
    Serial.println( max_speed_y );
  }
}

static void setFeedrate( float x, float y, float f){
	// Convert linear velocity to component axial velocities

  // Determine axial velocity
  float dx = abs(posx - x);
  float dy = abs(posy - y);

	float vec = sqrt( dx * dx + dy * dy);

	float vx = min(f * dx / vec , max_speed_x); // Cosine of movement vector angle
	float vy = min(f * dy / vec , max_speed_y); // Sine of movement vector angle

  feed_x = max(vx / dist_per_step_X, MIN_FEEDRATE); // Convert X-axis mm/sec to steps/sec
  feed_y = max(vy / dist_per_step_Y, MIN_FEEDRATE); // Convert Y-axis mm/sec to steps/sec

  // Set motor speeds to limit to desired feedrate
  // MultiStepper .runSpeedToPosition() refers to maxSpeed for coordinated motion planning
  mx.setMaxSpeed( feed_x );
  my.setMaxSpeed( feed_y );

  if( VERBOSE ){
    Serial.print("F: ");
    Serial.println( f );
    Serial.print("X_F: ");
    Serial.println( feed_x );
    Serial.print("Y_F: ");
    Serial.println( feed_y );
  }



  // Update acceleration settings
  // Acceleration not used by MultiStepper
  if (accel){
    mx.setAcceleration( feed_x/2.0 );
    my.setAcceleration( feed_y/2.0 );
  } else {
    mx.setAcceleration( feed_x );
    my.setAcceleration( feed_y );
  }

}

// TRIGONOMETRIC FUNCTIONS
//------------------------------------------------------------------------------
static float atan3( float dy, float dx ) {
  // Return angle of dy/dx as a value from 0 to 2PI
  
  float a = atan2( dy, dx );
  if ( a < 0 ) a = ( PI * 2.0 ) + a;
  return a;

}

// KINEMATICS
//------------------------------------------------------------------------------
static void IK(float x, float y, long &m1, long &m2){
	// Inverse kinematics - stepper position from x,y position	

	m1 = floor(x / dist_per_step_X);
	m2 = floor(y / dist_per_step_Y);

}

static void FK(float l1, float l2, float &x, float &y){
	// Forward kinematics - x,y position from stepper position
	
  l1 *= dist_per_step_X;
	l2 *= dist_per_step_Y;

	x = l1;
	y = l2;
}

// MOVEMENT
//------------------------------------------------------------------------------

void pause( long ms ) {
  delay( ms / 1000 ); // Whole second delay
  delayMicroseconds( ms % 1000 ); // Microsecond delay for remainder
}

static void line( float x, float y) {
  // Line drawing method

  setFeedrate( x, y, feedrate );

  if (hardLimits){
  	// Enforce hard limits to prevent machine from moving beyond workspace
  	if (x > limit_right or x < limit_left or y > limit_top or y < limit_bottom){
  		Serial.print( "OUTOFBOUNDS (" );
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.println(")");
  		return; // Escape out of command
  	}
  }

  long positions[2]; // Array of desired string lengths
  IK( x, y, positions[0], positions[1]);
  
  steppers.moveTo( positions ); // Set positions for each motor

  steppers.runSpeedToPosition(); // Blocks until all steppers are in position

  // Update current position
  posx = x;
  posy = y;
  
}

static void line_safe( float x, float y ) {
  // Subdivided line drawing method

  // Measure length of line to be drawn
  float dx = x - posx;
  float dy = y - posy;

  float len = sqrt( dx * dx + dy * dy );

  // Check if line is within allowable resolution
  if ( len <= MM_PER_SEGMENT ) {
    line( x, y );
    return;
  }

  // Too long - subdivide into smaller segments
  long pieces = floor( len / MM_PER_SEGMENT );
  float x0 = posx;
  float y0 = posy;
  float a;

  // Draw sequential line segments
  for ( long j = 1; j < pieces; j++ ) {
    a = (float)j / (float)pieces;

    line( ( x - x0 )*a + x0, ( y - y0 )*a + y0 );
  }

  line( x, y ); // Draw final line segment

}

static void arc( float cx, float cy, float x, float y, float dir ) {
  // Arc drawing method
  // Assumes fixed radius and max angle of 180 degrees (PI radians)

  // Determine radius
  float dx = posx - cx;
  float dy = posy - cy;
  float radius = sqrt( dx * dx + dy * dy );

  // Determine angle of arc (sweep)
  float angle1 = atan3( dy, dx );
  float angle2 = atan3( y - cy, x - cx );
  float theta = angle2 - angle1;

  if ( dir > 0 && theta < 0 ) angle2 += 2 * PI;
  else if ( dir < 0 && theta > 0) angle1 += 2 * PI;

  theta = angle2 - angle1;

  // Determine length of arc
  float len = abs( theta ) * radius;

  // Subdivide arc into safe line segments
  int i, segments = ceil( len / MM_PER_SEGMENT );

  float nx, ny, angle3, scale;

  for ( i = 0; i < segments; ++i ) {
    // Interpolate line segments around the arc
    scale = ( (float)i ) / ( (float)segments );

    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos( angle3 ) * radius;
    ny = cy + sin( angle3 ) * radius;

    // Send to line command
    line( nx, ny );
  }

  // Draw final line segment
  line( x, y );
}

static void teleport( float x, float y ) {
  // Position reset method (no movement)

  posx = x;
  posy = y;

  // Calculate stepper positions from coordinates
  long L1, L2;
  IK( posx, posy, L1, L2 );

  mx.setCurrentPosition( L1 );
  my.setCurrentPosition( L2 );

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

  Serial.print( "FEED: ");
  Serial.println( feedrate );
  Serial.print( "ACCEL: ");
  Serial.println( accel );
  Serial.print( "BIAS: ");
  Serial.println( bias );

  Serial.print( "MICROSTEP X: ");
  Serial.println( microstep_x );
  Serial.print( "MICROSTEP Y: ");
  Serial.println( microstep_y );


}

// AIR/BLAST CONTROL
//------------------------------------------------------------------------------

static void blastToggle( int toggle ){
  digitalWrite( BLAST, toggle );
  digitalWrite( AIR, toggle );

  if( VERBOSE ){
    if( toggle ){
      Serial.println( "BLAST / AIR ON" );
    } else{
      Serial.println( "BLAST/AIR OFF" );
    }
  }
}

static void airToggle( int toggle ){
  digitalWrite( AIR, toggle );

  if( VERBOSE ){
    if( toggle ){
      Serial.println( "AIR ON" );
    } else{
      Serial.println( "AIR OFF" );
    }
  }
}

// COMMAND METHODS
//------------------------------------------------------------------------------

float parsenumber( char code, float val ) {
  // Method for parsing GCODE command type

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
  // Method for parsing GCODE commands

  // Skip blank lines
  if ( buffer[0] == ';' ) return;

  long cmd;


  // SEQUENCE CHECKS
  //---------------------------

  cmd = parsenumber( 'N', -1);
  // Check for line number
  // Line number must appear first on the line
  if ( cmd != -1 && buffer[0] == 'N' ) {
    if ( cmd != line_number ) {
      // Indicate wrong line number and return
      Serial.print( "BADLINENUM " );
      Serial.println( line_number );
      return;
    }

    // Check for checksum
    if ( strchr( buffer, '*' ) != 0 ) {
      // Check validity
      unsigned char checksum = 0;
      int c = 0;
      while ( buffer[c] != '*' && c < MAX_BUF ) checksum ^= buffer[ c++ ];
      c++; // Skip checksum indicator (*)
      unsigned char against = (unsigned char)strtod( buffer + c, NULL );
      if ( checksum != against ) {
        // Indicate wrong checksum and return
        Serial.print( "BADCHECKSUM " );
        Serial.println( line_number );
        return;
      }
    } else {
      Serial.print( "NOCHECKSUM " );
      Serial.println( line_number );
      return;
    }

    // Indicate command recieved and OK
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
    case 1: { // Update machine dimensions
        limit_top = parsenumber( 'T', limit_top );
        limit_bottom = parsenumber( 'B', limit_bottom );
        limit_right = parsenumber( 'R', limit_right );
        limit_left = parsenumber( 'L', limit_left );

        if( 1 ){
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

        if( 1 ){
        	Serial.println( "MICROSTEPPING"  );
        	Serial.print( "X: " );
        	Serial.print( microstep_x );
        	Serial.print( " Y: ");
        	Serial.println( microstep_y );
        }

        break;
      }

    case 10: { // Update feedrate, acceleration, and bias settings
        feedrate = parsenumber( 'F', feedrate );
        accel = ( parsenumber('A', accel) == 1 );
        bias = ( parsenumber( 'B', bias) == 1 );
        
        if( 1 ){
        	Serial.print( "F: ");
        	Serial.println( feedrate );
          Serial.print( "ACCEL: " );
          Serial.println( accel );
          Serial.print( "BIAS: ");
          Serial.println( bias );

        }
        
        break;
      }
      
    case 20: // UNASSIGNED

    case 30: { // Report position
        where();
        break;
      }
  }


  // MOTOR OVERRIDE COMMANDS
  //---------------------------
  cmd = parsenumber( 'M', -1 );
  switch ( cmd ) {
    case 0: { //RELATIVE MOVE COMMAND
      int x_steps = parsenumber( 'X', 0 )/dist_per_step_X;
      int y_steps = parsenumber( 'Y', 0 )/dist_per_step_Y;

      if ( VERBOSE ){
        if ( x_steps ){
          Serial.print( "X JOG: " );
          Serial.println( x_steps );
        }
        if ( y_steps ){
          Serial.print( "Y JOG: " );
          Serial.println( y_steps );
        }
      }
      mx.setSpeed( min(x_steps, TOPSPEED_X/dist_per_step_X) );
      mx.setAcceleration( min(x_steps, TOPSPEED_X/dist_per_step_X) );
      mx.move( x_steps );
      mx.runToPosition();

      my.setSpeed( min(y_steps, TOPSPEED_Y/dist_per_step_Y) );
      my.setAcceleration( min(y_steps, TOPSPEED_Y/dist_per_step_Y) );
      my.move( y_steps );
      my.runToPosition();

      FK( mx.currentPosition(), my.currentPosition(), posx, posy );

      break;

    }
    case 1: // UNUSED

    case 10: {
        mx.setSpeed( TOPSPEED_X/2.0 );
        mx.setAcceleration( TOPSPEED_X/2.0 );
        mx.move(jog_dist * parsenumber( 'S', 1 )/dist_per_step_X);
        mx.runToPosition();
        break; // Jog MX stepper forward
      }
    case 11: {
        mx.setSpeed( TOPSPEED_X/2.0 );
        mx.setAcceleration( TOPSPEED_X/2.0 );
        mx.move(-jog_dist * parsenumber( 'S', 1 )/dist_per_step_X);
        mx.runToPosition();
        break; // Jog MX stepper forward
      }

    case 20: {
        my.setSpeed( TOPSPEED_Y/2.0 );
        my.setAcceleration( TOPSPEED_Y/2.0 );
        my.move(jog_dist * parsenumber( 'S', 1 )/dist_per_step_Y);
        my.runToPosition();
        break; // Jog MY stepper forward
      }
    case 21: {
        my.setSpeed( TOPSPEED_Y/2.0 );
        my.setAcceleration( TOPSPEED_Y/2.0 );
        my.move(-jog_dist * parsenumber( 'S', 1 )/dist_per_step_Y);
        my.runToPosition();
        break; // Jog MY stepper forward
      }
    
    case 50: blastToggle( 1 ); break;
    case 51: blastToggle( 0 ); break;

    case 60: airToggle( 1 ); break;
    case 61: airToggle( 0 ); break;

    case 100: { // Set stepper positions manually to O
        teleport( parsenumber( 'X', 0 ), parsenumber( 'Y', 0 ) );
        break;
      }
  }

  // MOVE/DRAW COMMANDS
  //---------------------------
  cmd = parsenumber( 'G', -1 );
  switch (cmd) {
    case 0: // Move command (no spray)
    case 1: { // Line command

        feedrate = parsenumber( 'F', feedrate );

        // Confirm that feedrate has been set
        // If not, give error command and break
        if ( !feedrate ){
          Serial.println( "ERROR - FEEDRATE NOT SET" );
          break;
        }

        // Enable sprayer if G01 command
        blastToggle( cmd );
        if( cmd != 1){
          airToggle( 1 );
        }
        
        /*
        if ( cmd == 1) {
          digitalWrite( BLAST, 1 );
        } else {
          digitalWrite( BLAST, 0 );
        }
        */

        line_safe( parsenumber( 'X', posx ), parsenumber( 'Y', posy ) );

        // Disengage sprayer
        blastToggle( 0 );
        //digitalWrite( BLAST, 0 );
        break;
      }
    case 2: //G02 - CW Arc command
    case 3: { //G03 - CCW Arc command

        feedrate = parsenumber( 'F', feedrate );

        // Confirm that feedrate has been set
        // If not, give error command and break
        if ( !feedrate ){
          Serial.println( "ERROR - FEEDRATE NOT SET" );
          break;
        }

        // Enable sprayer
        blastToggle( 1 );
        //digitalWrite( BLAST, 1 );

        feedrate = parsenumber( 'F', feedrate );

        arc( parsenumber( 'I', posx ),
             parsenumber( 'J', posy ),
             parsenumber( 'X', posx ),
             parsenumber( 'Y', posy ),
             ( cmd == 2 ) ? ARC_CCW : ARC_CW );

        // Disable sprayer
        blastToggle( 0 );
        //digitalWrite( BLAST, 0 );

        break;
      }
    case 4: //Pause command
    case 5: { // Dwell command
        blastToggle( cmd == 5);

        pause( parsenumber( 'S', 0 ) + parsenumber( 'P', 0 ) * 1000.0f );
        
        blastToggle( 0 );
        break;
      }
  }
}

void ready() {
  // Prepares input buffer for new messages and
  // signals serial device it is ready for a new command

  sofar = 0; // Reset input buffer position
  Serial.print( "\n> " );
  last_cmd_time = millis();
}

void Serial_listen() {
  // Method for listening for commands over serial
  while ( Serial.available() > 0 ) {
    char c = Serial.read();
    if ( sofar < MAX_BUF ) buffer[ sofar++ ] = c;
    if ( c == '\n' || c == '\r') {
      buffer[sofar] = 0;

      // Echo command
      if( VERBOSE ){
        Serial.println(buffer);
      }

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
  pinMode( MX_STEP, OUTPUT );
  pinMode( MX_DIR, OUTPUT );

  pinMode( MY_STEP, OUTPUT );
  pinMode( MY_DIR, OUTPUT );

  pinMode( BLAST, OUTPUT );
  pinMode( AIR, OUTPUT );
  digitalWrite( BLAST, 0 );
  digitalWrite( AIR, 0 );

  //initialize serial read buffer
  sofar = 0;
  Serial.begin( BAUD );
  Serial.println( "\n\nHELLO WORLD! I AM BLAST-BOT" );

  // set stepper directions
  mx.setPinsInverted( MX_FORWARD, false, false ); // direction inversion for X
  my.setPinsInverted( MY_FORWARD, false, false ); //direction inversion for Y

  // set stepper speed & acceleration
  getStepDist();
  calcMaxSpeed();

  // add steppers to MultiStepper object
  steppers.addStepper( mx );
  steppers.addStepper( my );

  // toggle air on and off
  airToggle( 1 );
  delay( 2000 );
  airToggle( 0 );
  delay( 500 );

  //initialize plotter positions  
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
