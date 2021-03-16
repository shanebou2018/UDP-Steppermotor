#include <WiFi.h> for wifi
#include <WiFiUDP.h>
#include <AccelStepper.h>
#include <M5Stack.h>

char* ssid = "NETWORKSSID";  //  your network SSID (name)
char* pass = "PASSWORD";       // your network password

WiFiUDP Udp;// A UDP instance to let us send and receive packets over UDP
unsigned int localPort = 12345;      // local port to listen for UDP packets
byte packetBuffer[512]; //buffer to hold incoming and outgoing packets

#include <AccelStepper.h>
 
//User-defined values
long receivedSteps = 0; //Number of steps
long receivedSpeed = 0; //Steps / second
long receivedAcceleration = 0; //Steps / second^2
String receivedCommand = "";

//-------------------------------------------------------------------------------
int directionMultiplier = 1; // = 1: positive direction, = -1: negative direction
bool newData, runallowed = false; // booleans for new data from serial, and runallowed flag
AccelStepper stepper(1,22, 19);// direction Digital 19 (CCW), pulses Digital 22 (CLK)
const byte interruptPin = 33; //pin for the microswitch using attachInterrupt(); 

void setup()
{
  pinMode(interruptPin, INPUT_PULLUP); // internal pullup resistor (debouncing)
  attachInterrupt(digitalPinToInterrupt(interruptPin), stopMotor, FALLING);
  //If you choose FALLING, make sure that the switch connects the pin 2 to the GND when it is pressed.
  //You can change FALLING but make sure that you connect the switch to GND or +5V accordingly!
  
  M5.begin(true, false, true);
  M5.Power.begin();
  
  // setting up Station AP
  WiFi.begin(ssid, pass);
  
  // Wait for connect to AP
  M5.Lcd.print("[Connecting] ");
  M5.Lcd.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }
  M5.Lcd.println();
  
  
  M5.Lcd.println("Connected to WiFi");
  
  // print the SSID of the network you're attached to:
  M5.Lcd.print("SSID: ");
  M5.Lcd.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();

    // print your IP address:
  M5.Lcd.print("IP Address: ");
  M5.Lcd.println(ip);

  // start up UDP
  Udp.begin(localPort);
  
  M5.Lcd.print("UDP server started at port ");
  M5.Lcd.println(localPort);
  
  M5.Lcd.println("Demonstration of AccelStepper Library"); //print a messages
  M5.Lcd.println("Send 'C' for printing the commands.");

  //setting up some default values for maximum speed and maximum acceleration
  M5.Lcd.println("Default speed: 400 steps/s, default acceleration: 800 steps/s^2.");
  stepper.setMaxSpeed(2000); //SPEED = Steps / second
  stepper.setAcceleration(1000); //ACCELERATION = Steps /(second)^2

  stepper.disableOutputs(); //disable outputs
   //M5.dis.drawpix(0, 0x707070);
  
}

void loop()
{
  checkUDP();
  RunTheMotor(); //function to handle the motor  
  M5.update();
}

void RunTheMotor() //function for the motor
{
    if (runallowed == true)
    {
        stepper.enableOutputs(); //enable pins
        stepper.run(); //step the motor (this will step the motor by 1 step at each loop)  
    }
    else //program enters this part if the runallowed is FALSE, we do not do anything
    {
        stepper.disableOutputs(); //disable outputs
        return;
    }
}

void checkUDP(){
  int byteCount = Udp.parsePacket();
  receivedCommand = "";
  if ( byteCount ) {
    M5.Lcd.print(millis());
    M5.Lcd.print(" > Packet of ");
    M5.Lcd.print(byteCount);
    M5.Lcd.print(" bytes received from ");
    M5.Lcd.print(Udp.remoteIP());
    M5.Lcd.print(":");
    M5.Lcd.println(Udp.remotePort());
    
    // We've received a packet, read the data from it
    Udp.read(packetBuffer,byteCount); // read the packet into the buffer

    // display the packet contents in HEX
    for (int i=1;i<=byteCount;i++)
    {
      M5.Lcd.print(packetBuffer[i-1],HEX);
      receivedCommand = receivedCommand + char(packetBuffer[i - 1]);
      if (i % 32 == 0)
      {
        M5.Lcd.println();
      }
      else M5.Lcd.print(' ');
    } // end for
    
    M5.Lcd.println();
    
    //print the whole UDP packet as plain text
    M5.Lcd.println(receivedCommand);
    processCommand();
  }
}

void processCommand(){
  newData = true; //indicate that there is a new data by setting this bool to true
  
  if (newData == true){
    switch (receivedCommand.charAt(0)){ //we check what is the command
      case 'P': //P uses the move() function of the AccelStepper library, which means that it moves relatively to the current position.             
          ParseStepsAndSpeed();
          directionMultiplier = 1; //We define the direction
          M5.Lcd.println("Positive direction."); //print the action
          RotateRelative(); //Run the function
          //example: P2000 400 - 2000 steps (5 revolution with 400 step/rev microstepping) and 400 steps/s speed
          //In theory, this movement should take 5 seconds
          break;         

      case 'N': //N uses the move() function of the AccelStepper library, which means that it moves relatively to the current position.      
          ParseStepsAndSpeed();
          directionMultiplier = -1; //We define the direction
          M5.Lcd.println("Negative direction."); //print action
          RotateRelative(); //Run the function
          //example: N2000 400 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
          //In theory, this movement should take 5 seconds
          break;

      case 'R': //R uses the moveTo() function of the AccelStepper library, which means that it moves absolutely to the current position.            
          ParseStepsAndSpeed();   
          directionMultiplier = 1; //We define the direction
          M5.Lcd.println("Absolute position (+)."); //print the action
          RotateAbsolute(); //Run the function
          //example: R800 400 - It moves to the position which is located at +800 steps away from 0.
          break;

      case 'r': //r uses the moveTo() function of the AccelStepper library, which means that it moves absolutely to the current position.            
          ParseStepsAndSpeed();
          directionMultiplier = -1; //We define the direction
          M5.Lcd.println("Absolute position (-)."); //print the action
          RotateAbsolute(); //Run the function
          //example: r800 400 - It moves to the position which is located at -800 steps away from 0.
          break;

      case 'S': // Stops the motor
          stepper.stop(); //stop motor
          stepper.disableOutputs(); //disable power
          M5.Lcd.println("Stopped."); //print action
          runallowed = false; //disable running
          break;

      case 'A': // Updates acceleration
          runallowed = false; //we still keep running disabled, since we just update a variable
          stepper.disableOutputs(); //disable power
          ParseReceivedAcceleration();
          if(receivedAcceleration > 0){
            stepper.setAcceleration(receivedAcceleration); //update the value of the variable
            M5.Lcd.print("New acceleration value: "); //confirm update by message
            M5.Lcd.println(receivedAcceleration); //confirm update by message
          }
          break;

      case 'L': //L: Location
          runallowed = false; //we still keep running disabled
          stepper.disableOutputs(); //disable power
          M5.Lcd.print("Current location of the motor: ");//Print the message
          M5.Lcd.println(stepper.currentPosition()); //Printing the current position in steps.
          break;
         
      case 'H': //H: Homing
          runallowed = true;     
          M5.Lcd.println("Homing"); //Print the message
          GoHome();// Run the function
          break;

      case 'U':
          runallowed = false; //we still keep running disabled
          stepper.disableOutputs(); //disable power
          stepper.setCurrentPosition(0); //Reset current position. "new home"            
          M5.Lcd.print("The current position is updated to: "); //Print message
          M5.Lcd.println(stepper.currentPosition()); //Check position after reset.
          break; 

      case 'C':
          PrintCommands(); //Print the commands for controlling the motor
          break;

      default:  
          break;
      }
  }
  //after we went through the above tasks, newData is set to false again, so we are ready to receive new commands again.
  newData = false;   
}

void ParseReceivedAcceleration(){
  String commandData = receivedCommand.substring(1);

  int parsedAcceleration = commandData.toInt();
  M5.Lcd.print("Parsed Acceleration: ");
  M5.Lcd.println(parsedAcceleration);
  if(parsedAcceleration > 0){
    receivedAcceleration = parsedAcceleration; 
    return;
  }
  M5.Lcd.println("This command takes one integer i.e A400");
}

void ParseStepsAndSpeed(){
  String commandData = receivedCommand.substring(1);
  M5.Lcd.println("Command Data: " + commandData);
  int spacePosition = commandData.indexOf(' ');
  if(spacePosition > 0){
    M5.Lcd.println("Space Position Found Parsing out values");
   
    int parsedSteps = commandData.substring(0,spacePosition).toInt();
    M5.Lcd.print("Parsed Steps: ");
    M5.Lcd.println(parsedSteps);
    
    int parsedSpeed = commandData.substring(spacePosition + 1).toInt();
    M5.Lcd.print("Parsed Speed: ");
    M5.Lcd.println(parsedSpeed);

    if(parsedSteps > 0 && parsedSpeed > 0){
      receivedSteps = parsedSteps;
      receivedSpeed = parsedSpeed;
      
      M5.Lcd.print("Received Steps: ");
      M5.Lcd.println(receivedSteps);
      M5.Lcd.print("Received Speed: ");
      M5.Lcd.println(receivedSpeed);
    }
    return;
  }
  
  M5.Lcd.println("This command takes two integers seperated by a space i.e P2000 400");
}

 
void GoHome()
{  
    if (stepper.currentPosition() == 0)
    {
        M5.Lcd.println("We are at the home position.");
        stepper.disableOutputs(); //disable power
    }
    else
    {
        stepper.setMaxSpeed(400); //set speed manually to 400. In this project 400 is 400 step/sec = 1 rev/sec.
        stepper.moveTo(0); //set abolute distance to move
    }
}
 
void RotateRelative()
{
    //We move X steps from the current position of the stepper motor in a given direction.
    //The direction is determined by the multiplier (+1 or -1)
   
    runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepper.setMaxSpeed(receivedSpeed); //set speed
    stepper.move(directionMultiplier * receivedSteps); //set relative distance and direction
}

void RotateAbsolute()
{
    //We move to an absolute position.
    //The AccelStepper library keeps track of the position.
    //The direction is determined by the multiplier (+1 or -1)
    //Why do we need negative numbers? - If you drive a threaded rod and the zero position is in the middle of the rod...
 
    runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepper.setMaxSpeed(receivedSpeed); //set speed
    stepper.moveTo(directionMultiplier * receivedSteps); //set relative distance   
}

void PrintCommands()
{  
    //Printing the commands
    M5.Lcd.println(" 'C' : Prints all the commands and their functions.");
    M5.Lcd.println(" 'P' : Rotates the motor in positive (CW) direction, relative.");
    M5.Lcd.println(" 'N' : Rotates the motor in negative (CCW) direction, relative.");
    M5.Lcd.println(" 'R' : Rotates the motor to an absolute positive position (+).");
    M5.Lcd.println(" 'r' : Rotates the motor to an absolute negative position (-).");
    M5.Lcd.println(" 'S' : Stops the motor immediately."); 
    M5.Lcd.println(" 'A' : Sets an acceleration value.");
    M5.Lcd.println(" 'L' : Prints the current position/location of the motor.");
    M5.Lcd.println(" 'H' : Goes back to 0 position from the current position (homing).");
    M5.Lcd.println(" 'U' : Updates the position current position and makes it as the new 0 position. ");   
}

void stopMotor()//function activated by the pressed microswitch
{
  //Stop motor, disable outputs; here we should also reset the numbers if there are any
  runallowed = false; //disable running
       
      stepper.setCurrentPosition(0); // reset position
      Serial.println("STOP "); //print action
      stepper.stop(); //stop motor
      stepper.disableOutputs(); //disable power


  Serial.println("Pressed."); //feedback towards the serial port
  
  //This part might not work properly.
  //M5.dis.drawpix(5, 0xff0000); //turn on LED  #FF0000=green
  delay(2000); //wait a bit
  //M5.dis.drawpix(5, 0x00ff00 ); //turn off the LED     #00FF00=red
  
}
