////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// brush-bot Airbrushing Robot | The Living | 2016                            //
// Portrait Mode                                                              //
// v4.0 2017.06.12                                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// EXTERNAL DEPENDENCIES
//------------------------------------------------------------------------------
import controlP5.*; //ControlP5 - UI Interface
import processing.serial.*; //Serial - Com protocol with Arduino
import javax.swing.JOptionPane; //Interface for COM port selection

// GLOBAL VARIABLES
//------------------------------------------------------------------------------
// DEBUG
Boolean VERBOSE = false; //default: false -- if enabled, print all responses from GRBL
Boolean SIMPLE_MODE = false; //default: false (buffer-fill mode) / true (line-response mode)
int reportFreq = 5; //
// IO
Boolean type_gcode = true;
Boolean load_dir = true;
String fp = "";
// UX
ControlP5 cP5;
PFont font24, font18, font16i, font14, font12;
color black, white, grey, charcoal, green, red, blue;
PVector origin;
float scalar;
PShape preview;
// GCODE
StringList gcode;
int line;
int issued, completed;
// MACHINE
float posx, posy, lastx, lasty, spray_speed;
float canvas_width, canvas_height, canvas_margin;
int sprayoff = 450;
int sprayon = 10;
// STATUS
String status;
Boolean streaming, spraying, paused, loaded, idle;
String versionPattern = "Grbl*";
String startupPattern = ">*:ok";
String statusPattern = "<*>";
String okPattern = "*ok*";
String errorPattern = "*error*";
String CMD_VERSION, CMD_STARTUP, CMD_STATUS, CMD_OK, CMD_ERROR;
Boolean match;
// STREAM MODE
IntList c_line;
// SERIAL
Serial port;
String portname;
String val, sent;
String lastSent;
Boolean connected;
int r = 0;
int timeout = 0;

// SETUP
//------------------------------------------------------------------------------
void setup() {
  settings(); //INITIALIZE WINDOW SIZE

  initVariables(); //INITIALIZE SYSTEM VARIABLES
  initPatterns(); //INITIALIZE MESSAGE PATTERNS

  initFonts(); //INITIALIZE UX FONTS
  initColors(); //INITIALIZE UX COLORS

  initPreview( ); //INITIALIZE GCODE PREVIEW

  setupControls(); //GENERATE UX
  selectSerial(); //ATTEMPT TO CONNECT TO SERIAL
}

// DRAW
//------------------------------------------------------------------------------
void draw(){
  displayUI(); // DRAW UI
  renderPreview( ); // DRAW GCODE PREVIEW
  displayStats(); // DISPLAY DRAWING STATUS
  checkStatus(); // UPDATE BUTTONS BY STATE

  // REALTIME STATUS REPORTING
  if(connected && r>reportFreq){
    statusReport();
    r = 0;
  }
  r++;

  if (connected) serialRun(); // CHECK SERIAL FOR UPDATES
  renderNozzle(); // DRAW NOZZLE ON PREVIEW

  // TIMEOUT IF SYSTEM HANGS
  // FIRST SHUTS OFF SPRAY
  // THEN CANCELS STREAM AND GOES HOME
  if( idle && spraying && timeout > 120 ){
    send( gSpray(false) );
  }
  if( idle && streaming ){
    timeout++;
    if ( timeout > 1200 ){
      print("TIMED OUT, GOING HOME\n");
      streaming = false;
      line = 0;
      timeout = 0;
      send( home() );
    }
  }
}

// SETTINGS
//------------------------------------------------------------------------------
void settings(){
  size(1300, 750);
  smooth();
}

// INIT VARIABLES
void initVariables(){
  // UX
  origin = new PVector(950,350);
  scalar = 5.0;
  // GCODE
  gcode = new StringList();
  line = 0;
  issued = 0;
  completed = 0;
  // MACHINE
  posx = 0.0;
  posy = 0.0;
  status = "[...]";
  spray_speed = 5000.0;
  canvas_width = 65.0;
  canvas_height = 90.0;
  canvas_margin = 1.0;
  //STATUS
  streaming = false;
  spraying = false;
  paused = false;
  loaded = false;
  match = false;
  idle = false;
  //STREAM MODE
  c_line = new IntList();
  // SERIAL
  port = null;
  portname = null;
  val = "...";
  sent = "...";
  connected = false;
  lastSent = "";
}

void initPatterns(){
  CMD_VERSION = versionPattern.replaceAll(".","[$0]").replace("[*]",".*");
  CMD_STARTUP = startupPattern.replaceAll(".","[$0]").replace("[*]",".*");
  CMD_STATUS = statusPattern.replaceAll(".","[$0]").replace("[*]",".*");
  CMD_OK = okPattern.replaceAll(".","[$0]").replace("[*]",".*");
  CMD_ERROR = errorPattern.replaceAll(".","[$0]").replace("[*]",".*");

}

// PARSE NUMBER FROM GCODE STRING
// Used to extract numerical values from GCode
float parseNumber(String s, String c, float f){
  String num = "-.0123456789";
  c = c.toUpperCase();
  s = s.toUpperCase();
  int start = s.indexOf(c);
  if( start < 0 ) return f;
  int end = start+1;
  for( int i = start+1; i < s.length(); i++){
    char k = s.charAt(i);
    if( Character.isLetter(k) || k == ' ') break;
    end = i;
  }
  return float( s.substring(start+1,end+1) );
}

// SUMS IntList
// Sums contents of an IntList (used in available buffer tracking)
int sumList(IntList w){
  int sum = 0;
  for(int i = 0; i<w.size();i++){
    sum += w.get(i);
  }
  return sum;
}

////////////////////////////////////////////////////////////////////////////////
// RENDERING
////////////////////////////////////////////////////////////////////////////////

// INITIALIZE GCODE PREVIEW
void initPreview(){
  preview = new PShape();
}

// RENDER PREVIEW TO CANVAS
void renderPreview(){
  if( preview == null ) return;
  preview.enableStyle();
  shape(preview, origin.x - (canvas_width*0.5*scalar), origin.y-(canvas_height*0.5*scalar));
}

// GENERATE PREVIEW
void generatePreview(StringList g){
  preview = new PShape();
  PVector last = new PVector(0,0);
  int type;
  color c;
  float o;
  float w;

  for(int i = 0; i<g.size(); i++){
    String cmd = g.get(i);
    type = int(parseNumber(cmd,"G",-1));
    if( type < 0 ) continue;

    // COLOR, LINEWEIGHT, OPACITY SETTINGS
    c = (type==0||type==4) ? blue : red;
    w = (type==0||type==4) ? 2 : 3;
    o = (type==0||type==4) ? 255 : 85;

    switch(type){
      case 0:
      case 1:
        renderLine(last, cmd, c, o, w);
        break;
      case 2:
      case 3:
        renderArc(last, cmd, type, c, o, w);
        break;
      case 4:
        renderPoint(last, c, o, w);
        break;
      default:
        break;
    }
  }
}

// RENDER LINE
// Visualizes GCODE line command (G0/G1)
void renderLine(PVector l, String cmd, color c, float o, float w ){
  PShape ln;
  float x = parseNumber(cmd,"X",l.x);
  float y = parseNumber(cmd,"Y",l.y);
  noFill();
  stroke(c,o);
  strokeWeight(w);
  ln = createShape( LINE, l.x*scalar, -l.y*scalar, x*scalar, -y*scalar );
  // line(origin.x+l.x*scalar, origin.y-l.y*scalar,origin.x+x*scalar, origin.y-y*scalar);
  preview.addChild( ln );
  l.x = x;
  l.y = y;
}

// RENDER ARC
// Visualizes GCODE arc command (G2/G3)
void renderArc( PVector l, String cmd, int dir, color c, float o, float w ){
  PShape a;

  float cx = parseNumber(cmd, "I", 0.0)+l.x;
  float cy = parseNumber(cmd, "J", 0.0)+l.y;
  float x = parseNumber(cmd, "X", l.x);
  float y = parseNumber(cmd, "Y", l.y);
  float dx2 = l.x - cx;
  float dy2 = l.y - cy;
  float dx1 = x - cx;
  float dy1 = y - cy;

  float r = sqrt( pow(dx1,2) + pow(dy1,2) );

  float SA = TWO_PI - atan2(dy1, dx1);
  float EA = TWO_PI - atan2(dy2, dx2);

  if( dir == 3 && SA > EA){
    EA += TWO_PI;
  } else if( dir == 2 && EA > SA){
    SA += TWO_PI;
  }

  noFill();
  stroke(c,o);
  strokeWeight(w);

  if( dir == 2){
    a = createShape(ARC, cx*scalar, -cy*scalar, r*2*scalar, r*2*scalar, EA, SA);
  } else {
    a = createShape(ARC, cx*scalar, -cy*scalar, r*2*scalar, r*2*scalar, SA, EA);
  }
  preview.addChild(a);
  l.x = x;
  l.y = y;
}

// RENDER POINT
// Visualizes GCODE dwell command (G4)
void renderPoint( PVector l, color c, float o, float w){
  PShape p;
  stroke(c,o);
  strokeWeight(w*3);
  p = createShape(POINT, l.x*scalar, -l.y*scalar);
  preview.addChild(p);
}

////////////////////////////////////////////////////////////////////////////////
// SERIAL COMMUNICATION
////////////////////////////////////////////////////////////////////////////////

// OPEN SERIAL PORT
void openSerial(){
  if( portname == null ){
    connected = false;
    return;
  }
  if( port != null ) port.stop();
  try{
    port = new Serial(this, portname, 115200);
    port.bufferUntil('\n');
    connected = true;
  } catch( Exception e){
    closeSerial();
    println("DISCONNECTED: NO SERIAL CONNECTION AVAILABLE");
  }
}

// RESET ALL SERIAL VARIABLES
void closeSerial(){
  portname = null;
  connected = false;
  port = null;
}

// SELECT SERIAL PORT TO OPEN
void selectSerial(){
  int s = Serial.list().length;
  if( s == 0 ){
    JOptionPane.showMessageDialog(null, "No Arduino Connected");
    return;
  }
  if( s > 1){
    String result = (String) JOptionPane.showInputDialog(
      null,
      "Select the serial port connected to Arduino",
      "Select serial port",
      JOptionPane.PLAIN_MESSAGE,
      null,
      Serial.list(),
      0
    );
    if( result != null ) portname = result;
  }
  else portname = Serial.list()[0];
  openSerial();
}

// UPDATE SERIAL CONNECTION & CHECK FOR DATA
void serialRun(){
  if(port.available() > 0){
    String temp = port.readStringUntil('\n');
    if(temp == null) return;
    temp = temp.trim();

    if(temp.matches(CMD_VERSION)){
      if(VERBOSE) print("[STARTUP] "+temp+"\n");
      return;
    }
    if(temp.matches(CMD_STARTUP)){
      if(VERBOSE) print("[STARTUP] "+temp+"\n");
      return;
    }
    if(temp.matches(CMD_STATUS)){
      status = temp;
      extractDim();
      return;
    }
    if(temp.matches(CMD_ERROR)){
      print("[ERROR] " + temp +"\n");
      print("[SENT] " + sent + "\n");
      return;
    }

    if(SIMPLE_MODE){
      if( temp.matches(CMD_OK) ){
        if(VERBOSE) print("[RX] "+temp+"\n");
        line++;
        completed++;
        timeout = 0;
      }
    }
    else {
      if( temp.matches(CMD_OK)){
        if(VERBOSE) print("[RX] "+temp+"\n");
        if(c_line.size()>0){
          c_line.remove(0);
          completed++;
        }
        timeout = 0;
      }
    }
  }
  //SEND GCODE
  stream();
}

// REQUEST MACHINE POSITION REPORT
void statusReport(){
  sendByte( report() );
}

// EXTRACT DIMENSIONS FROM MACHINE REPORT
void extractDim(){
  String[] temp_stat = status.substring(1,status.length()-1).split("\\|");
  //Extract machine status
  idle = temp_stat[0].contains("Idle");
  //Extract Work Position
  String[] temp_pos = temp_stat[1].substring(5).split(",");
  posx = float(temp_pos[0]);
  posy = float(temp_pos[1]);
  //Extract Servo Position
  int servoPos = int( temp_stat[3].substring(4).split(",")[1] );
  spraying = (servoPos == sprayon );
  //Format status message for UX
  status = join(subset(temp_stat,0,4), " | ");
}

// SERIAL SEND
void send( String cmd ){
  if(!connected) return;
  cmd = cmd.trim().replace(" ","");
  sent = cmd;
  port.write(cmd + "\n");

  if( VERBOSE ) print("SENT: " + cmd + '\n');
}

// SERIAL SEND BYTE
void sendByte( Byte b ){
  if(!connected) return;
  port.write( b );
  if(VERBOSE) sent = str(char(b));
}

// RESET SERIAL STREAM STATUS
void resetStatus(){
  line = 0;
  issued = 0;
  completed = 0;
  c_line = new IntList();
}

// SERIAL STREAM
void stream(){
  if(!connected || !streaming) return;

  while(true){
    if( line >= gcode.size() || line < 0 ){
      if( line>0 ){
        print("COMPLETED STREAMING\n");
        //streaming = false;
        line = -1;
      } else if ( c_line.size() == 0 ) {
        print("DRAWING FINISHED\n");
        streaming = false;
        resetStatus();
      }
      return;
    }
    if( gcode.get(line).trim().length() == 0 ){
      line++;
      continue;
    }
    else break;
  }

  String cmd = gcode.get(line).trim().replace(" ","");
  if(SIMPLE_MODE){

    if( !lastSent.contains(cmd) ){
      port.write( cmd + "\n" );
      issued++;
      lastSent = cmd;
      print("SENT "+line+": "+cmd+" : ");
      sent = cmd;
    }
  } else {
    if(VERBOSE) print( str(127 - sumList(c_line)) + " BYTES AVAILABLE\n" );
    if( sumList(c_line) + (cmd.length()+1) <= 127 ){
      c_line.append( cmd.length()+1 );
      port.write(cmd + "\n");
      issued++;
      lastSent = cmd;
      line++;
      print("SENT "+line+": "+cmd+"\n");
      sent = cmd;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// GCODE
////////////////////////////////////////////////////////////////////////////////
// G0/G1 - LINE COMMAND
String gLine(float x, float y, boolean f){
  String cmd = (f) ? "G1" : "G0";
  cmd += "X"+str(x) + "Y"+str(y);
  return cmd;
}

// G2/G3 - ARC COMMANDS
String gArc(float cx, float cy, float x, float y, boolean dir){
  //clockwise = 2 ... counterclockwise = 3
  if( dir ) return "G2I"+str(cx) + "J"+str(cy) + "X"+str(x) + "Y"+str(y) + "F"+str(int(spray_speed));
  else return "G3I" + str(cx) + "J" + str(cy) + "X" + str(x) + "Y" + str(y) + "F"+str(int(spray_speed));
}

// G4 - PAUSE COMMAND
String gDwell( float time ){
  return "G4P" + str(time);
}

// M3 - SPRAY COMMAND
String gSpray( boolean s ){
  return "M3S" + ((s) ? str(sprayon) : str(sprayoff));
}

// Report
Byte report(){
  return byte(0x3f);
}

// JOGGING
String jog(float x, float y){
  String cmd = "G91";
  cmd += gLine(x,y,false);
  return cmd + "\nG90";
}

// SET ORIGIN
String origin(){
  posx = 0.0;
  posy = 0.0;
  return "G10 P1 L20 X0 Y0";
}
// GO HOME
String home(){
  return gLine(0,0,false);
}

Byte gPause(){
  return byte(0x21);
}

Byte gDoor(){
  return byte(0x84);
}

Byte gReset(){
  return byte(0x18);
}

Byte gResume(){
  return byte(0x7e);
}

// PARK (GO TO MACHINE ZERO)
String park(){
  return "G53 X0 Y0";
}

////////////////////////////////////////////////////////////////////////////////
// FILE I/O
////////////////////////////////////////////////////////////////////////////////

// LOAD FILES
void loadSingle(){
  selectInput("Select a file to load:", "fileSelected");
}

void loadFolder(){
  selectFolder("Select a folder of drawings to load:", "folderSelected");
}

void folderSelected( File f ){
  if( f == null ){
    print("Window closed or user cancelled\n");
    return;
  }
  fp = f.getAbsolutePath();
  print("User selected " + fp + "\n");
  String[] files = listFiles(fp);
  if( files == null || !checkDir(files,((type_gcode)?"txt":"json"))){
    loaded = false;
    fp = "";
    print( ((files==null)?"ERROR--EMPTY OR INVALID DIRECTORY\n":"ERROR--NO JSON DRAWING FILES IN DIRECTORY\n"));
    return;
  }
  loaded = true;
  gcode = (type_gcode) ? processGCODEs( files ) : processJSONs( files );
  if( gcode.size() > 0 ){
    print("DRAWINGS LOADED\n");
    print("GCODE LINES GENERATED: " + gcode.size() + "\n");
    generatePreview(gcode);
    print("GCODE PREVIEW GENERATED\n");
    saveStrings( "data/gcode.txt", gcode.array() );
  }
  line = 0;
}

void fileSelected( File f ){
  if( f == null ){
    print("Window closed or user cancelled\n");
    return;
  }
  fp = f.getAbsolutePath();
  print( "User selected "+fp+"\n");
  loaded = true;
  gcode = (type_gcode) ? processGCODE(fp) : processJSON(fp);
  if(gcode.size() > 0){
    print( "DRAWING LOADED\n");
    print( "GCODE LINES GENERATED: "+gcode.size()+"\n");
    generatePreview(gcode);
    if(VERBOSE) print("GCODE PREVIEW GENERATED");
    saveStrings( "data/gcode.txt", gcode.array() );
  }
  line = 0;
}

// LIST FILES IN DIRECTORY
String[] listFiles( String dir ){
  File file = new File(dir);
  if( file.isDirectory() ){
    return file.list();
  }
  return null;
}

Boolean fileCheck( String f, String ext ){
  return f.contains(ext);
}

// CHECK FILE EXTENSION
Boolean checkDir( String[] files, String ext ){
  for( int i = 0; i<files.length; i++){
    if ( fileCheck(files[i], ext) ) return true;
  }
  return false;
}

// PROCESS FILES
StringList processJSONs( String[] f ){
  StringList g = new StringList(); //clear gcode buffer
  PVector p;

  g.append( gSpray(false) );
  g.append( home() );

  for( int i = 0; i < f.length; i++){
    if( !fileCheck(f[i],"json") ) continue;

    JSONArray coords = loadJSONArray( fp + "\\" + f[i] );

    p = extractPos( coords.getFloat(0), -coords.getFloat(1) );
    g.append( gSpray(false) );
    g.append( gLine( p.x, p.y, false ) );
    g.append( gDwell(0.5) );
    g.append( gSpray(true) );

    for( int k = 2; k < coords.size(); k+=2 ){
      p = extractPos( coords.getFloat(k),-coords.getFloat(k+1) );
      g.append( gLine(p.x, p.y, true) );
    }
    g.append( gSpray(false) );
  }
  g.append( gSpray(false) );
  g.append( home() );

  print("GCODE LINES GENERATED: " + g.size() + "\n");
  return g;
}

StringList processGCODEs( String[] f ){
  String[] load;
  StringList g = new StringList();

  g.append( gSpray(false) );
  g.append( home() );

  for(int i = 0; i < f.length; i++){
    if( !fileCheck(f[i],"txt") ) continue;
    load = loadStrings(fp+"\\"+f[i]);

    for(int k = 0; k < load.length; k++){
      //ignore home commands at beginning & end of file
      if( k <= 3 && load[k].contains("G0X0Y0")) continue;
      if( load[k].length() < 1 ) continue;
      if( k >= load.length-3 && load[k].contains("G0X0Y0")) continue;
      g.append(load[k]);
    }
    g.append(gSpray(false));
  }
  g.append( gSpray(false));
  g.append( home() );

  return g;
}

StringList processJSON( String f ){
  StringList g = new StringList();
  if( !fileCheck(f,"json") ){
    print("ERROR - NOT A JSON FILE\n");
    return g;
  }

  PVector p;
  JSONArray coords = loadJSONArray( f );
  p = extractPos(coords.getFloat(0), -coords.getFloat(1));
  g.append( gSpray(false) );
  g.append( home() );

  g.append( gLine( p.x, p.y, false ) );
  g.append( gDwell(0.5) );
  g.append( gSpray(true) );

  for( int i = 2; i < coords.size(); i+=2 ){
    p = extractPos( coords.getFloat(i),-coords.getFloat(i+1) );
    g.append( gLine(p.x, p.y, true) );
  }

  g.append( gSpray(false) );
  g.append( home() );

  return g;
}

StringList processGCODE( String f ){
  StringList g = new StringList();
  if( !fileCheck(f,"txt") ){
    print("ERROR - NOT A GCODE FILE\n");
    return g;
  }
  String[] load = loadStrings(f);
  g.append( gSpray(false) );
  g.append( home() );

  for (int i = 0; i < load.length; i++){
    g.append( load[i] );
  }
  g.append( gSpray(false) );
  g.append( home() );
  return g;
}

PVector extractPos(float x, float y){
  float x_s = (canvas_width*0.5)-canvas_margin;
  float y_s = (canvas_height*0.5)-canvas_margin;
  float x_off = canvas_width*0.5;
  float y_off = canvas_height*0.5;

  return new PVector( x_off + x * x_s, y_off + y * y_s );
}

////////////////////////////////////////////////////////////////////////////////
// UX
////////////////////////////////////////////////////////////////////////////////

// INIT COLORS
void initColors(){
  black = color(0);
  white = color(255);
  grey = color(220);
  charcoal = color(100);
  red = color(237, 28, 36);
  green = color(57, 181, 74);
  blue = color(80, 150, 225);
}

// INIT FONTS
void initFonts(){
  font24 = loadFont("Roboto-Regular-24.vlw");
  font18 = loadFont("Roboto-Regular-18.vlw");
  font16i = loadFont("Roboto-Italic-16.vlw");
  font14 = loadFont("Roboto-Regular-14.vlw");
  font12 = loadFont("Roboto-Regular-12.vlw");
}

// DISPLAY UI
void displayUI() {
  // UI CANVAS DIMENSIONS
  float scaleWidth = canvas_width*scalar;
  float scaleHeight = canvas_height*scalar;
  float scaleMargin = canvas_margin*scalar;

  // SETUP PREVIEW AREA
  // Canvas BG
  noStroke();
  fill(black);
  rect(600, 0, 700, 750);

  pushMatrix();
  rectMode(CENTER);

  // Canvas
  translate(origin.x,origin.y);
  fill(white);
  rect(0,0,scaleWidth,scaleHeight);
  fill(grey);
  rect(0,0,scaleWidth-scaleMargin*2,scaleHeight-scaleMargin*2);

  // Canvas Grid
  noFill();
  stroke(white);
  strokeWeight(1);
  for (float x = 0 ; x < scaleWidth*0.5; x+=scalar*2) {
    line(x, -scaleHeight*0.5, x, scaleHeight*0.5);
    line(-x, -scaleHeight*0.5, -x, scaleHeight*0.5);
  }
  for (float y = 0; y < scaleHeight*0.5; y+=scalar*2) {
    line(-scaleWidth*0.5, y, scaleWidth*0.5, y);
    line(-scaleWidth*0.5, -y, scaleWidth*0.5, -y);
  }

  // Canvas Frame
  strokeWeight(1);
  rect(0, 0, scaleWidth, scaleHeight);

  rectMode(CORNER);
  popMatrix();

  // MANUAL CONTROLS AREA
  // Controls BG
  noStroke();
  fill(grey);
  rect(0,0,600,900);
  noFill();
  stroke(charcoal);
  strokeWeight(1);
  rect(15,40,320,320);
  // Controls Label
  fill(black);
  textFont(font24,24);
  textAlign(LEFT);
  text("MANUAL CONTROLS", 15, 30);
  // File load Area
  fill(black);
  rect(0,375,590,70);

  // Console area
  fill(black);
  rect(0,450,590,305);

  // Loading labels
  fill(white);
  textFont(font14,14);
  textAlign(LEFT);
  text("FILE TYPE",25,435);
  text("LOAD MODE",110,435);

}

// RENDER NOZZLE POSITION
void renderNozzle(){
  pushMatrix();
  //Display Dimensions
  float scaleWidth = canvas_width*scalar;
  float scaleHeight = canvas_height*scalar;

  translate(origin.x-scaleWidth*0.5,origin.y-scaleHeight*0.5);

  // Nozzle Icon
  stroke( (spraying)?red:blue );
  fill(white,50);
  strokeWeight(3);
  ellipse(posx*scalar,-(posy*scalar),10,10);
  noFill();
  strokeWeight(0.5);
  ellipse(posx*scalar, -(posy*scalar),20,20);

  // Nozzle Position Text
  String pos = "( "+nf(posx,0,2)+", "+nf(posy,0,2)+" )";
  rectMode(CENTER);
  noStroke();
  fill(255,100);
  rect(posx*scalar,-posy*scalar+20, textWidth(pos)+10,20,10);
  rectMode(CORNER);

  //fill( (spraying) ? red : blue );
  fill(black);
  textFont(font14,14);
  textAlign(CENTER);
  text(pos,(posx*scalar),-(posy*scalar) + 24.0);

  popMatrix();

}

// DISPLAY STATS
void displayStats(){
  // TX Command
  if(sent != null){
    noStroke();
    fill(green);
    textAlign(LEFT);
    textFont(font24, 24);
    text("TX: "+sent, 15, 560);
  }
  // RX Command
  if(val != null){
    noStroke();
    fill(red);
    textAlign(LEFT);
    textFont(font18, 18);
    text("RX: "+val, 15, 590);
  }

  //COMPLETION
  noStroke();
  fill(white);
  textAlign(LEFT);
  textFont(font18,18);
  text("LINES SENT: "+issued+" / "+gcode.size(), 15, 620);
  text("COMPLETED: "+completed+" / "+gcode.size(), 15, 640);

  // Serial Status
  String serial_status;
  textFont(font18,18);
  fill( ((connected) ? green : red) );
  serial_status = (connected) ? "CONNECTED ON " + portname : "NOT CONNECTED";
  text(serial_status, 15, 740);

  // Machine status
  textFont(font18,18);
  fill( (status.contains("Idle")) ? white : (status.contains("Run")) ? green : red );
  textAlign(CENTER);
  text(status, origin.x, origin.y+375);

  // File Selection
  if(fp.length()>0){
    String[] path = fp.split("\\\\");
    int depth = path.length;
    textFont(font18,18);
    fill( white );
    textAlign(LEFT);
    text(join(subset(path,depth-2),"/"),210,415);
  }
}

// SET UP UX CONTROLS
void setupControls() {
  cP5 = new ControlP5(this);

  // Global Settings
  cP5.setFont( font12 );
  cP5.setColorForeground( black );
  cP5.setColorBackground( white );
  cP5.setColorValueLabel( white );
  cP5.setColorCaptionLabel( white );
  cP5.setColorActive( blue );

  // Serial Connect Button
  cP5.addBang("connect")
  .setPosition(475,710)
  .setSize(100,25)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(white)
  .setColorActive(blue)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(black)
  .setFont(font12)
  .setText("CONNECT")
  ;

  // Y+100 button
  cP5.addBang("y+100")
  .setPosition(150,50)
  .setSize(50,32)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("+100")
  ;
  // Y+10 button
  cP5.addBang("y+10")
  .setPosition(150,87)
  .setSize(50,32)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("+10")
  ;
  // Y+1 button
  cP5.addBang("y+1")
  .setPosition(150,124)
  .setSize(50,32)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("+1")
  ;
  // Y-100 button
  cP5.addBang("y-100")
  .setPosition(150,318)
  .setSize(50,32)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("-100")
  ;
  // Y-10 button
  cP5.addBang("y-10")
  .setPosition(150,281)
  .setSize(50,32)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("-10")
  ;
  // Y-1 button
  cP5.addBang("y-1")
  .setPosition(150,244)
  .setSize(50,32)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("-1")
  ;
  // X-100 button
  cP5.addBang("x-100")
  .setPosition(25,175)
  .setSize(32,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("-100")
  ;
  // X-10 button
  cP5.addBang("x-10")
  .setPosition(62,175)
  .setSize(32,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("-10")
  ;
  // X-1 button
  cP5.addBang("x-1")
  .setPosition(99,175)
  .setSize(32,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("-1")
  ;
  // X+100 button
  cP5.addBang("x+100")
  .setPosition(293,175)
  .setSize(32,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("+100")
  ;
  // X+10 button
  cP5.addBang("x+10")
  .setPosition(256,175)
  .setSize(32,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("+10")
  ;
  // X+1 button
  cP5.addBang("x+1")
  .setPosition(219,175)
  .setSize(32,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("+1")
  ;

  //Go Home Button
  cP5.addBang("home")
  .setPosition(140,165)
  .setSize(70,70)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font12)
  .setText("GO HOME")
  ;

  // Sprayer Off Button
  cP5.addBang("sprayOff")
  .setPosition(345,175)
  .setSize(120,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font14)
  .setText("SPRAY OFF")
  ;
  // Sprayer On Button
  cP5.addBang("sprayOn")
  .setPosition(470,175)
  .setSize(120,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(black)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font14)
  .setText("SPRAY ON")
  ;

  // Set Origin Button
  cP5.addBang("origin")
  .setPosition(255,300)
  .setSize(70,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(white)
  .setColorActive(blue)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(black)
  .setFont(font14)
  .setText("SET (0,0)")
  ;

  // Park Machine Button
  cP5.addBang("park")
  .setPosition(25,300)
  .setSize(70,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(white)
  .setColorActive(red)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(black)
  .setFont(font14)
  .setText("PARK")
  ;

  // Start Button
  cP5.addBang("start")
  .setPosition(345,245)
  .setSize(245,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(green)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font24)
  .setText("RUN FILE")
  ;
  // Pause Button
  cP5.addBang("pause")
  .setPosition(345,300)
  .setSize(245,50)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(red)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(white)
  .setFont(font24)
  .setText("PAUSE")
  ;

  // Load Files Button
  cP5.addBang("load")
  .setPosition(470,375)
  .setSize(120,70)
  .setTriggerEvent(Bang.RELEASE)
  .setColorForeground(blue)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(black)
  .setFont(font18)
  .setText("LOAD")
  ;

  cP5.addToggle("file-type")
  .setPosition(25,390)
  .setSize(80,30)
  .setColorForeground(white)
  .setColorBackground(white)
  .setColorActive(white)
  .setState(type_gcode)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(black)
  .setFont(font18)
  .setText("JSON")
  ;

  cP5.addToggle("load-mode")
  .setPosition(110,390)
  .setSize(80,30)
  .setColorForeground(white)
  .setColorBackground(white)
  .setColorActive(white)
  .setState(load_dir)
  //caption settings
  .getCaptionLabel()
  .align(ControlP5.CENTER, ControlP5.CENTER)
  .setColor(black)
  .setFont(font18)
  .setText("FILE")
  ;

  // Canvas Width Entry
  cP5.addTextfield("width")
  .setPosition( 345, 50 )
  .setSize( 120, 30 )
  .setFont( font18 )
  .setFocus( false )
  .setColor( black )
  .setAutoClear( false )
  .setInputFilter( ControlP5.INTEGER )
  .setValue( nf(canvas_width) )
  //caption settings
  .getCaptionLabel()
  .setColor( black )
  .setFont( font14 )
  .alignX( ControlP5.LEFT )
  .setText( "WIDTH (mm)" )
  ;
  // Canvas Height Entry
  cP5.addTextfield("height")
  .setPosition( 470, 50 )
  .setSize( 120, 30 )
  .setFont( font18 )
  .setFocus( false )
  .setColor( black )
  .setAutoClear( false )
  .setInputFilter( ControlP5.INTEGER )
  .setValue( nf(canvas_height) )
  //caption settings
  .getCaptionLabel()
  .setColor( black )
  .setFont( font14 )
  .alignX( ControlP5.LEFT )
  .setText( "HEIGHT (mm)" )
  ;
  // Canvas Margin Entry
  cP5.addTextfield("margin")
  .setPosition( 345, 110 )
  .setSize( 120, 30 )
  .setFont( font18 )
  .setFocus( false )
  .setColor( black )
  .setAutoClear( false )
  .setInputFilter( ControlP5.INTEGER )
  .setValue( nf(canvas_margin) )
  //caption settings
  .getCaptionLabel()
  .setColor( black )
  .setFont( font14 )
  .alignX( ControlP5.LEFT )
  .setText( "MARGIN (mm)" )
  ;
  // Paint Speed Entry
  cP5.addTextfield("speed")
  .setPosition( 470, 110 )
  .setSize( 120, 30 )
  .setFont( font18 )
  .setFocus( false )
  .setColor( black )
  .setAutoClear( false )
  .setInputFilter( ControlP5.INTEGER )
  .setValue( nf(spray_speed) )
  //caption settings
  .getCaptionLabel()
  .setColor( black )
  .setFont( font14 )
  .alignX( ControlP5.LEFT )
  .setText( "SPEED (mm/min)" )
  ;

  // Manual Command Entry
  cP5.addTextfield("cmdEntry")
  .setPosition( 15, 460 )
  .setSize( 560, 50 )
  .setFont( font24 )
  .setFocus( true )
  .setColor( black )
  .setAutoClear( true )
  //caption settings
  .getCaptionLabel()
  .setColor(white)
  .setFont(font14)
  .alignX(ControlP5.LEFT)
  .setText("MANUAL ENTRY")
  ;
}

// UX CONTROL EVENTS
void controlEvent( ControlEvent theEvent ) {
  if ( theEvent.isController() ) {
    String eventName = theEvent.getName();
    switch( eventName ) {
      case "connect":
        if(connected){
          port.stop();
          portname = null;
        }
        selectSerial();
        break;
      case "park":
        if(!streaming) send( park() );
        break;
      case "y+100":
        if(!streaming) send( jog( 0, 100 ) );
        break;
      case "y+10":
        if(!streaming) send( jog( 0, 10 ) );
        break;
      case "y+1":
        if(!streaming) send( jog(0, 1) );
        break;
      case "y-1":
        if(!streaming) send( jog(0,-1) );
        break;
      case "y-10":
        if(!streaming) send( jog(0, -10) );
        break;
      case "y-100":
        if(!streaming) send( jog(0, -100) );
        break;
      case "x+100":
        if(!streaming) send( jog(100, 0) );
        break;
      case "x+10":
        if(!streaming) send( jog(10, 0) );
        break;
      case "x+1":
        if(!streaming) send( jog(1, 0) );
        break;
      case "x-1":
        if(!streaming) send( jog(-1, 0) );
        break;
      case "x-10":
        if(!streaming) send( jog(-10, 0) );
        break;
      case "x-100":
        if(!streaming) send( jog(-100, 0) );
        break;
      case "home":
        if(!streaming) send( home() );
        break;
      case "sprayOff":
        if(!streaming) send( gSpray(false) );
        break;
      case "sprayOn":
        if(!streaming) send( gSpray(true) );
        break;
      case "origin":
        if(!streaming) send( origin() );
        break;
      case "width":
      case "height":
      case "margin":
        if(!streaming) updateDim();
        break;
      case "speed":
        if(!streaming) updateSpeed();
        break;
      case "cmdEntry":
        if(!streaming) send( manualEntry() );
        break;
      case "load":
        if( load_dir ) loadFolder();
        else loadSingle();
        break;
      case "load-mode":
        if(!streaming) {
          load_dir = !load_dir;
          print("LOADING MODE: "+ ((load_dir)?"DIRECTORY":"SINGLE FILE") + "\n");
        }
        break;
      case "file-type":
        if(!streaming){
          type_gcode = !type_gcode;
          print("INPUT FILETYPE: "+ ((type_gcode)?"GCODE":"JSON") + "\n");
        }
        break;
      case "start":
        if(paused){
          streaming = false;
          resetStatus();
          sendByte( gReset() );
          delay(100);
          send( home() );
          paused = false;
          break;
        }
        if(!streaming){
          updateSpeed();
          streaming = true;
          stream();
        }
        break;
      case "pause":
        paused = !paused;
        if(paused){
          sendByte( gDoor() );
        } else {
          sendByte( gResume() );
          streaming = true;
          stream();
        }
        break;
      default:
        break;
    }
  }
}

// CHECK STATUS
void checkStatus(){
  Bang start = cP5.get(Bang.class, "start");
  Bang pause = cP5.get(Bang.class, "pause");
  Bang load = cP5.get(Bang.class, "load");
  Bang origin = cP5.get(Bang.class, "origin");
  Bang connect = cP5.get(Bang.class, "connect");
  Toggle f_type = cP5.get(Toggle.class, "file-type");
  Toggle l_type = cP5.get(Toggle.class, "load-mode");

  relabelToggle( f_type, ((type_gcode)?"GCODE":"JSON"));
  relabelToggle( l_type, ((load_dir)?"DIR":"FILE"));

  if( !connected ){
    lockButton( start, true, charcoal, grey );
    relabelButton( start, "START" );
    lockButton( pause, true, charcoal, grey );
    relabelButton( pause, "PAUSE" );
    relabelButton( connect, "CONNECT" );
    return;
  }

  if( loaded ){
    relabelButton( load, "RELOAD" );
  }

  if( (streaming && !paused) ){
    lockButton( start, false, blue, white );
    relabelButton( start, "RUNNING" );
    lockButton( pause, false, red, white );
    relabelButton( pause, "PAUSE" );
    lockButton( load, true, charcoal, grey );
    lockButton( origin, true, charcoal, grey );
    lockButton( connect, true, charcoal, grey );
    return;
  }

  if( paused ){
    lockButton( start, false, red, white );
    relabelButton( start, "RESET" );
    lockButton( pause, false, green, white );
    relabelButton( pause, "RESUME" );
    lockButton( load, true, charcoal, grey );
    lockButton( origin, true, charcoal, grey );
    lockButton( connect, true, charcoal, grey );
    return;
  }

  lockButton( start, false, green, white );
  relabelButton( start, "START" );
  lockButton( pause, false, red, white );
  relabelButton( pause, "PAUSE" );
  lockButton( load, false, blue, black );
  lockButton( origin, false, black, white );
  lockButton( connect, false, white, black );

}

// RELABEL BUTTON
void relabelButton(Bang button, String newlabel){
  button.getCaptionLabel().setText(newlabel);
}
void relabelToggle(Toggle button, String newlabel){
  button.getCaptionLabel().setText(newlabel);
}

// LOCK BUTTON
void lockButton(Bang button, boolean lock, color c, color t){
  button.setLock(lock)
  .setColorForeground(c)
  .getCaptionLabel().setColor(t);
}

// MANUAL ENTRY
String manualEntry() {
  return cP5.get(Textfield.class, "cmdEntry").getText().toUpperCase();
}

// UPDATE DIMENSIONS
void updateDim(){
  String w_ = cP5.get(Textfield.class, "width").getText();
  String h_ = cP5.get(Textfield.class, "height").getText();
  String m_ = cP5.get(Textfield.class, "margin").getText();

  canvas_width = (w_ != "" && float(w_)>0) ? float(w_) : canvas_width;
  canvas_height = (h_ != "" && float(h_)>0) ? float(h_) : canvas_height;
  canvas_margin = (m_ != "" && float(m_)<canvas_width/2 && float(m_)<canvas_height/2) ? float(m_) : canvas_margin;
}

// UPDATE SPEED
void updateSpeed(){
  String s_ = cP5.get(Textfield.class, "speed").getText();
  spray_speed = (s_ != "" && float(s_)>0 && float(s_)<=10000) ? float(s_) : spray_speed;
  send("G1F"+str(spray_speed)+"\n");
}