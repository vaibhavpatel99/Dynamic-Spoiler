// SPI - Version: Latest 
#include <SPI.h>
#include <mcp_can.h>

//comment out if serial debug messages not required
#define SERIAL_DEBUG

//message IDs
#define IDStrWhAng 0x1E5
#define IDBrkPdlPos 0xF1
// define the new ID for spoiler msg #define

// variables for recieving CAN msg 
unsigned long ID;
unsigned char len = 0;
unsigned char buf[8];
double strWhAng; // Unit: deg
double brkPdlPos; // Unit: %
int sig = 0;
int phase = 0;
int new_phase = 0;
int str_phase = 0;
int brk_phase = 0;
int phase_move = 0;

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

// pins for movement of actuator
const int forwards = 4;
const int backwards = 5;
const int power = 8;
  
void setup(){
  Serial.begin(115200);
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
      Serial.println("CAN initialization retrying.");
    }

  pinMode(power, OUTPUT);
  digitalWrite(power, HIGH);
    
  //set mask and filter for CAN transceiver
  //the chip applies the filters in registers 0 & 1 to the mask in register 0.  the mask in register 1 corresponds to filters 2-5
  //mask and filters for receive buffer 0, which will receive msgs we want
  CAN.init_Mask(0, 0, 0x7FF); //0th mask register, 7FF mask to get all ids
  CAN.init_Filt(0, 0, IDStrWhAng); //filter for control msg id, in 0th register
  CAN.init_Filt(1, 0, IDBrkPdlPos);
  //mask and filters for receive buffer 1, which will be setup to reject all ids
  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(2, 0, 0x000); //no CAN message will satisfy this mask and filter
  CAN.init_Filt(3, 0, 0x000);
  CAN.init_Filt(4, 0, 0x000);
  CAN.init_Filt(5, 0, 0x000);

  // actuator stuff
  pinMode(forwards, OUTPUT);
  pinMode(backwards, OUTPUT);
}

void loop(){
  // Received CAN message for 2 signals
  if(CAN_MSGAVAIL == CAN.checkReceive()) 
  {
    // read data,  len: data length, buf: data buf
    CAN.readMsgBufID(&ID, &len, buf); //why no & infront of buf
    
    #ifdef SERIAL_DEBUG
    #endif
    
    // Convert signals
    if (ID == IDStrWhAng) {
      // Any checks we need to do to make sure data in signal is legit?
      int sig = buf[2]|(buf[1] << 8);
//      Serial.print("CAN Received. temp= ");
//      Serial.println(temp);
      strWhAng = abs(0.0625*(sig));

//      Serial.print("Steering Wheel Angle: ");
 //     Serial.println(strWhAng);
    }
    // if above if statement works, repeast for IDBrkPdlPos
    if (ID == IDBrkPdlPos) {
      // Any checks we need to do to make sure data in signal is legit?
      int sig = (buf[1]);
      brkPdlPos = 0.392157*(sig);
//      Serial.print("Brake Pedal Position: ");
 //     Serial.println(brkPdlPos);
    }
  }

  if(strWhAng < 90) {
    str_phase = 0;
  } else if(strWhAng >= 90 and strWhAng < 180) {
    str_phase = 1;
  } else if(strWhAng >= 180 and strWhAng < 270) {
    str_phase = 2;
  } else {
    str_phase = 3;
  }

  if(brkPdlPos < 25) {
    brk_phase = 0;
  } else if(brkPdlPos >= 25 and brkPdlPos < 50) {
    brk_phase = 1;
  } else if(brkPdlPos >= 50 and brkPdlPos < 75) {
    brk_phase = 2;
  } else {
    brk_phase = 3;
  }

  if(brk_phase > str_phase) {
    new_phase = brk_phase;
  } else if(str_phase > brk_phase) {
    new_phase = str_phase;
  } else {
    new_phase = brk_phase;
  }

  phase_move = new_phase - phase;
  
  if(new_phase != phase) {
//    digitalWrite(forwards, HIGH);
//    digitalWrite(backwards, LOW);
//    delay(1000);
    if(phase_move >= 0) {
      digitalWrite(forwards, LOW);
      digitalWrite(backwards, HIGH);
      if(phase_move == 0) {
        delay(0);
      } else if(phase_move == 1) {
        delay(200);
      } else if(phase_move == 2) {
        delay(400);
      } else if(phase_move == 3) {
        delay(600);
      }
    } else if(phase_move < 0) {
      digitalWrite(forwards, HIGH);
      digitalWrite(backwards, LOW);
      if(phase_move == -1) {
        delay(200);
      } else if(phase_move == -2) {
        delay(400);
      } else if(phase_move == -3) {
        delay(600);
      }
    }
    digitalWrite(forwards, HIGH);
    digitalWrite(backwards, HIGH);
    phase = new_phase;
    Serial.println(phase);
  }
//  delay(1000);
}
