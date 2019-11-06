#include <Wire.h>
#include <IridiumSBD.h>
#define IridiumSerial Serial3
#define DIAGNOSTICS false // Change this to see diagnostics
IridiumSBD modem(IridiumSerial);
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VC0706.h>
//#include <SD_t3.h> //needed for teensy to work
#include <SD.h>
/*
  //////////////////////////////////////////////////
  CompleteMinusACS as of 09/01/19:

  TODO:
  verify camera integration with masterstatus/rockblock variables (fix photosize2 issue)
  fault handling should tell which fault has occured (may be solved)
  a global variable should know if a fault has been tripped each loop
  move special report counter to comcontroller 
  add camera reset and resolution commands
  have overwrite variables initial state change with flow so that when overwrite is true, the operations doesnt change abruptly
  assure everything from rockblock dev 6 experimental has ben passed over!

  Changes:
  Deployment code is done
  reformatted loop, more neat

  //////////////////////////////////////////////////
*/

//////////////// RockBlock Setup Data ////////////////

//uint16_t bytesleft; //declared, needs to be integrated into hybrid //added a 2 since it is used by a library //declared at pic downlink
uint16_t counter = 0; //starts as 0, no image has been segmented for downlink yet // needs to be integrated to hybrid
//boolean to know when we are sending the first segment of an image in a downlink -> needs to be created in hybrid
//bool isFirst = true; //declared at pic downlink
//bool imgsent = false; // last msg sent on downlink was an image. Starts as false //declared at pic downlink

int exampleData = 7471236;
uint32_t downlinkSize = 70;
uint8_t buf[340]; //340 is the max for rockblock
int openSpot = 0;
uint8_t lastReceive = 0; //is this number appropriate for startup?
uint8_t lastSend = 0;
bool DLConf = false; //should you be checking to confirm sucessful downlink. true = yes, false = no
uint8_t lastSR = 0; // how many transmissions have passed since the last special report
//uint32_t downlinkPeriod = 900000; // how many millis to wait between each downlink
//uint32_t uplinkPeriod = 900000; // how many millis to wait before each uplink
uint16_t SRFreq = 10; //how many regular downlinks between special reports
uint32_t SRTimeout = 300000; // how many millis will the rockblock attempt to downlink
uint16_t DLFailCounter = 0; //how many times has downlinking failed
int x;
int err;

//////////////// RockBlock Setup Data End ////////////////


Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial5);

//SD Card
#define chipSelect 4
#define DLSize 320
#define WireTransferSize 32

long day = 86400000; // 86400000 milliseconds in a day
long hour = 3600000; // 3600000 milliseconds in an hour
long minute = 60000; // 60000 milliseconds in a minute
long second =  1000; // 1000 milliseconds in a second

unsigned long manualTimeout = 10 * 1000;
int endT = 0;
unsigned long int z; //number of millis in current cycle
uint32_t nextMode = 0; // mode of the spacecraft next cycle

  //downlink msg test!
  uint8_t testmsgbuffer[] = {0,1,5,2,5,2,3};
  size_t testmsgbufferSize = sizeof(testmsgbuffer);

  //for uplink! This time saved!
  uint8_t rxbuffer[70];
  size_t rxBufferSize = sizeof(rxbuffer);
  //Serial.println(sizeof(rxBuffer));

  char rxOutput = "";
  String rxOutputCom = "";
  int receivedSize = -1;


//Deployment commands
long int burnTimer;
int deployStage = 0;
bool DLKey1 = false;
bool DLKey2 = false;
//uint8_t faultReport[40];
int ReportOpenSpot = 0;
bool sendFault = false;
bool faultTripped = false;
const int SenseNumb = 10; //change number of data samples in averaging array
const int faultSize = 40; // always make a multiple of 8 //size of faultReport
bool SensUpdate = false; //is there fresh IMU data
bool IMUBufferFull = false; //must fill w/ data before averages can be taken
int SenseCounter = 0;

//Health check
//bool IMUActive = false; //now inside class
//bool CamActive = false; //now inside class
//bool RBActive = false; //now inside class
//bool SlaveActive = false; //no longer used since we have one teensy,not two computers
//bool SlaveCompFlag = false; //no longer used since we have one teensy,not two computers
//bool SDActive = false; //now inside class

//verify these are used
bool WireConnected = true; //no longer used since we have one teensy, not two computers
boolean newData = false;
String receivedChars = "";
// makeshift command interpreter. read command parse note for better
// explaination. popcommands is up and running.

//Camera
#define chipSelect 4
#define smallImage VC0706_160x120
#define mediumImage VC0706_320x240
#define largeImage VC0706_640x480
long lastCamCheckTime = 0;
int camCheckTime = 4070; // what are you?
bool camStatus = false;
//Camera Functions
char classfilename[9]; //general filename, to be used by camera functions for transmission
uint16_t photosize; //needed to keep track of size of our first image
uint16_t photosize2; //needed to keep track of our true size of the buffer array
uint8_t a[5120]; //array is a general one

//Picture Downlink
uint16_t bytesleft; //declared, needs to be integrated into hybrid //Already declared at RB Init
uint16_t piccounter = 0; //starts as 0, no image has been segmented for downlink yet // needs to be integrated to hybrid
bool isFirst = true; //boolean to know when we are sending the first segment of an image in a downlink -> needs to be created in hybrid
bool imgsent = false; // last msg sent on downlink was an image. Starts as false

//General Downlink
//int exampleData = 7471236; //erase after testing //used by the commCon()
//uint8_t buf[340]; //340 is the max for rockblock
//int openSpot = 0;
//long int lastReceive = 0; //is this number appropriate for startup?
//long int lastSend = 0;
//bool DLConf = false; //should you be checking to confirm sucessful downlink. true = yes, false = no
//int lastSR = 0; // how many transmissions have passed since the last special report
//int DLFailCounter = 0; //how many times has downlinking failed
//int x;
//int err;

static const char b64chars[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"; //Remove before Flight //TODO

void printArray(uint8_t arr[], int s) {
  if (s) {
    for (int i = 0; i < s; i++) {
      print_binary(arr[i], 8);
      Serial.print(" ");
    }
    Serial.println("");
  }
}

void print_binary(int v, int num_places) {
  int mask = 0, n;
  for (n = 1; n <= num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places
  while (num_places) {
    if (v & (0x0001 << num_places - 1)) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    --num_places;
  }
}

void byteread(int value) {// convert a 1 byte int into 1 uint
  uint8_t f = 0;
  f = value & 0xFF;
  buf[openSpot] = f; openSpot++;
}
void twobyteread (int value) {// convert a 2 byte int into 2 uints
  uint8_t f = 0;
  f = value & 0xFF;
  buf[openSpot + 1] = f;
  f = (value >> 8) & 0xFF;
  buf[openSpot] = f; openSpot = (openSpot + 2);
}
void threebyteread (int value) { // convert a 3 byte int into 3 uints
  uint8_t f1 = 0;
  uint8_t f2 = 0;
  uint8_t f3 = 0;
  f1 = value & 0xFF;
  buf[openSpot + 2] = f1;
  f2 = (value >> 8) & 0xFF;
  buf[openSpot + 1] = f2;
  f3 = (value >> 16) & 0xFF;
  buf[openSpot] = f3;
  openSpot = (openSpot + 3);
}
void printBits(uint8_t myByte) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    if (mask  & myByte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}

String Hash_base64( uint8_t *in, int hashlength) {  //only works on strings up to 5120bytes?
  int i, out;
  char b64[(int)(hashlength * (8 / 6.0) + 1)]; // working byte array for sextets....
  String base64;
  for (i = 0, out = 0 ;; in += 3) { // octets to sextets
    i++;
    b64[out++] = b64chars[in[0] >> 2];

    if (i >= hashlength ) { // single byte, so pad two times
      b64[out++] = b64chars[((in[0] & 0x03) << 4) ];
      b64[out++] =  '=';
      b64[out++] =  '=';
      break;
    }

    b64[out++] = b64chars[((in[0] & 0x03) << 4) | (in[1] >> 4)];
    i++;
    if (i >= hashlength ) { // two bytes, so we need to pad one time;
      b64[out++] =  b64chars[((in[1] & 0x0f) << 2)] ;
      b64[out++] =  '=';
      break;
    }
    b64[out++] = b64chars[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
    b64[out++] =   b64chars[in[2] & 0x3f];

    i++;
    if (i >= hashlength ) { // three bytes, so we need no pad - wrap it;
      break;
    }
    //Serial.println(out);
  } // this should make b64 an array of sextets that is "out" in length
  b64[out] = 0;
  base64 = b64;
  return (base64);
}



////////////////////////////////////////////////////////////////////////////////
/////////////////Database of parameters/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class masterStatus {
    //Class to hold entire State of Spacecraft Operation except timers
  public:
    // To others looking at this code:
    //it is important that as few of these variables as possible get arbitrarily assigned a value.
    // these should only be given real data

    bool OVERWRITE; //this boolean allows commands from user to overwrite the normal spacecraft behavior. It is used in PowerMan, ModeSwitch, faultCheck
    uint32_t overdown; //this and the next two ints are used by overwrite commands
    uint32_t overup;
    uint32_t overmode;
    uint32_t State; // what does this do
    uint32_t NextState; // I don't think we need the state stuff
    uint32_t Faults; //number of faults detected over satellite lifetime
    uint8_t faultReport[faultSize]; //What faults are we checking the state of
    //String FaultString; // turn activefaults into a string for easy transmitting stupid idea, strings resrve two bytes per character

    //ROCKBLOCK Message Variables
    bool AttemptingLink; //True if waiting for SBDIX to return
    bool MessageStaged; //True if message waiting in Mobile Originated Buffer
    int RBCheckType; //State of Outgoing Communication with RockBlock. 0=ping, 1=Send SBDIX, 2=Fetch Incomming Command
    int MOStatus; //0 if No Outgoing message, 1 if outgoing message success, 2 if error
    int MOMSN; //Outgoing Message #
    int MTStatus; //0 if No Incoming message, 1 if Incoming message success, 2 if error
    int MTMSN; //Incoming Message #
    int MTLength; //Incoming Message Length in bytes 
    uint32_t MTQueued; //# of messages waiting in iridium //setup this to work
    String SBDRT;
    int LastMsgType; //0 = inval, 1 = ok, 2 = ring, 3 = error, 4 = ready //TODO
    int LastSMsgType; //Only Update on NON EMPTY reads from RB: 0 = inval, 1 = ok, 2 = ring, 3 = error, 4 = ready //TODO
    uint32_t SBDIXFails; //setup this to work
 

    //Binary array variables
    int binArr[1000];
    int binOpenSpot = 0;

    int XGyroThresh;
    int YGyroThresh;
    int ZGyroThresh;
    int XAccelThresh;
    int YAccelThresh;
    int ZAccelThresh;

    //Camera Variables
    bool CamActive;
    bool SDActive;
    volatile int ITStatus;
    uint32_t numPhotos;
    bool CameraBurst;
    uint32_t imageSize; //this is our photosize/photosize2
    unsigned long burstStart;
    unsigned long BurstDuration;


    //  get all the min max vals/percision
    uint32_t Mag[3];
    uint32_t Gyro[3];
    uint32_t Accel[3];


    uint32_t MagAve[3];
    uint32_t GyroAve[3];
    uint32_t AccelAve[3];

    uint32_t MagLog[3][SenseNumb];
    uint32_t GyroLog[3][SenseNumb];
    uint32_t AccelLog[3][SenseNumb];

    uint32_t ImuTemp;
    uint32_t Temp;
    uint32_t TempAccumulator;
    uint32_t DoorButton; // what is the default reading of a button
    uint32_t lightSense;
    uint32_t SolarCurrent;
    uint32_t CurrentZero;
    // may need to store the zeros aswell.

    //COntants for Current Sensor
    //const int SOLAR_PIN;      // Input pin for measuring Vout
    const float RS = 0.1;           // Shunt resistor value (in ohms)
    const float VOLTAGE_REF = 3.3;  // Reference voltage for analog read
    const float RL = 29.75;         // Load Resistor value NOTE: in kOhms
    //float sensorValue;              // Analog Signal Value
    //float volt;                     // Voltage after conversion from analog signal
    //float amp;

    //Inhibitors
    uint32_t Inhibitor_19a;
    uint32_t Inhibitor_10b;
    uint32_t Inhibitor_2;
    uint32_t FREEHUB;
    

    //ADCS State Variables: torque rods do we need pwm?
    uint32_t TorqueXDir; //-1 or 1 for Coil Current Direction
    uint32_t TorqueXPWM; // 0 to 255 for Coil Current Level
    uint32_t TorqueYDir; //-1 or 1 for Coil Current Direction
    uint32_t TorqueYPWM; // 0 to 255 for Coil Current Level
    uint32_t TorqueZDir; //-1 or 1 for Coil Current Direction
    uint32_t TorqueZPWM; // 0 to 255 for Coil Current Level
    uint32_t IMUActive;

    //power board variables
    uint8_t Battery; // battery charge

    //ADCS State Variables: torque rods

    //Radio State Variables:
    //todo flesh out once we get our hands on a radio
    bool RBActive;
    bool DownlinkStaged;
    bool AttemptingDownLink;
    bool StageReport;
    uint32_t lastSR; //last special report, keeps track of number of normal reports since last special report
    uint32_t downlinkPeriod; // how many millis to wait between each downlink
    uint32_t uplinkPeriod; // how many millis to wait before each uplink
    uint32_t SRFreq; //how many regular downlinks between special reports 
    uint32_t SRTimeout;
    uint32_t VarHolder[13]; //I still dont get this

    masterStatus() {

      //Constructor
      State = 1; //normal ops

//      MagLog = {{0,0,0},{0,0,0},{0,0,0}};
//      GyroLog = {{0,0,0},{0,0,0},{0,0,0}};
//      AccelLog = {{0,0,0},{0,0,0},{0,0,0}};
//
//      GyroAve[3] = {0,0,0}; // sent to us by slave comp
//      MagAve[3] = {0,0,0}; // sent to us by slave comp
//      AccelAve[3] = {0,0,0}; // sent to us by slave comp

      TorqueXDir = 0;
      TorqueXPWM = 0;
      TorqueYDir = 0;
      TorqueYPWM = 0;
      TorqueZDir = 0;
      TorqueZPWM = 0;
      IMUActive = false; //used for healthcheck

      ImuTemp = 0; //integrated IMU temp sensor
      Temp = 0; // external thermistor
      DoorButton = 0; //TODO
      lightSense = 0;
      Battery = 3.7; //todo i'd rather not have the default value trip fault detection 50%?
      SolarCurrent = 0;
      CurrentZero = 0;

      CamActive = false; //used for healthcheck
      SDActive = false; //used for healthcheck
      ITStatus = 0; //todo joao
      numPhotos = 0;
      CameraBurst = 0;
      imageSize = 0; //this is our photosize/photosize2
      burstStart = 0;
      BurstDuration = 15000;

      Faults = 0;

//      RS = 0.1;           // Shunt resistor value (in ohms)    //these three are declared right way since they are const and in C++
//      VOLTAGE_REF = 3.3;  // Reference voltage for analog read
//      RL = 29.75;

      Inhibitor_19a = 200;
      Inhibitor_10b = 200;
      Inhibitor_2 = 200; //any value for these three above 0 is taken as true
      FREEHUB = 0; //used for GREENLIGHT function, deterine if cubesat left p-pod //value is zero for false
      
      
      OVERWRITE = false; //default values to start spacecraft operations
      overdown = 900000; 
      overup = 60000;
      overmode = 0;

      downlinkPeriod = 60000; // how many millis to wait between each downlink
      uplinkPeriod = 1000; // how many millis to wait before each uplink
      SRFreq = 10; //how many regular downlinks between special reports
      SRTimeout = 300000;

      RBActive = false;
      DownlinkStaged = false; // have data to transfer
      AttemptingDownLink = false; // radio transmission in progress
      StageReport = false;
      lastSR = 0;
      SRFreq = 9; //how many normal reports before special extra data report

    }

    float roundDecimal(float num, int places) { // is this still needed
      int roundedNum = round(pow(10, places) * num);
      return roundedNum / ((float)(pow(10, places)));
    }

    void PrintStatus() { // all data should be ints (still works w/ floats though..)
      build1Byte(State);
      build3Byte(GyroAve[0]); build3Byte(GyroAve[1]); build3Byte(GyroAve[2]); // change to ints.
      build3Byte(MagAve[0]); build3Byte(MagAve[1]); build3Byte(MagAve[2]);
      build3Byte(AccelAve[0]); build3Byte(AccelAve[1]); build3Byte(AccelAve[2]);
      build1Byte(ImuTemp);
      build1Byte(Temp);
      build1Byte(Battery);
      build1Byte(Faults);
    }

//    String SpecialReport() { //todo make this work w/ binary //Special Report is now a separate function
//      // all fixed values
//      String output = "";
//      output += "{";
//      output += "Fl:" + String(Faults) + ",";
//      output += "FlSt:" + String(FaultString);
//      output += "SRF:" + String(SRFreq) + ",";
//      output += "XGT:" + String(XGyroThresh) + ",";
//      output += "YGT:" + String(YGyroThresh) + ",";
//      output += "ZGT:" + String(ZGyroThresh) + ",";
//      output += "XAT:" + String(XAccelThresh) + ",";
//      output += "YAT:" + String(YAccelThresh) + ",";
//      output += "ZAT:" + String(ZAccelThresh) + "}";
//      return (output);
      // all misc information
    

    /////////////////binary functions//////////////////

    void build3Byte(long int numb) {
      //range +- 8,388,607

      //if we have any values above 8 million we probably have a problem :)

      if (numb >= 0) { //if positive first bit = 1
        binArr[binOpenSpot] = 1;
        binOpenSpot ++;
      } else {
        binArr[binOpenSpot] = 0;
        binOpenSpot ++; //if negative first bit = 0
        numb = abs(numb);
      }

      if (numb > -8388607 && numb < 8388607) {


        if (numb < 2) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 4) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 8) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 16) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 32) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 64) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 128) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 256) {
          binArr[binOpenSpot] = 0; binOpenSpot ++; // 1 byte
        }
        if (numb < 512) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 1024) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 2048) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 4096) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 8192) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 16384) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 32768) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 65536) {
          binArr[binOpenSpot] = 0; binOpenSpot ++; //2byte
        }
        if (numb < 131072) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 262144) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 524288) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 1048576) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 2097152) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 4194304) {
          binArr[binOpenSpot] = 0; binOpenSpot ++; //23 bits of data
        }

      } else {
        numb = 8388607;
      }
      String h = "";
      int a = 0;
      h += String(numb, BIN);
      int msgLen = h.length();

      while (msgLen != 0) {

        String x = h.substring(a, a + 1);
        char d[10];
        x.toCharArray(d, 10);

        if (int(d[0]) == 49) {
          binArr[binOpenSpot] = 1;
        } else {
          binArr[binOpenSpot] = 0;
        }

        binOpenSpot++;
        a++;
        msgLen--;
      }
    }


    void build2Byte(long int numb) {
      //range +- 32,767

      if (numb >= 0) { //if positive first bit = 1
        binArr[binOpenSpot] = 1;
        binOpenSpot ++;
      } else {
        binArr[binOpenSpot] = 0;
        binOpenSpot ++; //if negative first bit = 0
        numb = abs(numb);
      }

      if (numb > -32767 && numb < 32767) {


        if (numb < 2) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 4) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 8) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 16) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 32) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 64) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 128) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 256) {
          binArr[binOpenSpot] = 0; binOpenSpot ++; // 1 byte
        }
        if (numb < 512) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 1024) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 2048) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 4096) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 8192) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 16384) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }

      } else {
        numb = 32767;
      }
      String h = "";
      int a = 0;
      h += String(numb, BIN);
      int msgLen = h.length();

      while (msgLen != 0) {

        String x = h.substring(a, a + 1);
        char d[10];
        x.toCharArray(d, 10);

        if (int(d[0]) == 49) {
          binArr[binOpenSpot] = 1;
        } else {
          binArr[binOpenSpot] = 0;
        }

        binOpenSpot++;
        a++;
        msgLen--;
      }
    }


    void build1Byte(long int numb) {
      //range +- 127

      if (numb >= 0) { //if positive first bit = 1
        binArr[binOpenSpot] = 1;
        binOpenSpot ++;
      } else {
        binArr[binOpenSpot] = 0;
        binOpenSpot ++; //if negative first bit = 0
        numb = abs(numb);
      }

      if (numb > -127 && numb < 127) {


        if (numb < 2) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 4) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 8) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 16) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 32) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }
        if (numb < 64) {
          binArr[binOpenSpot] = 0; binOpenSpot ++;
        }

      } else {
        numb = 127;
      }
      String h = "";
      int a = 0;
      h += String(numb, BIN);
      int msgLen = h.length();

      while (msgLen != 0) {

        String x = h.substring(a, a + 1);
        char d[10];
        x.toCharArray(d, 10);

        if (int(d[0]) == 49) {
          binArr[binOpenSpot] = 1;
        } else {
          binArr[binOpenSpot] = 0;
        }

        binOpenSpot++;
        a++;
        msgLen--;
      }
    }




    /////////////////////////////////////////////////////////////





};
masterStatus MSH; //Declare MSH

////////////////////////////////////////////////////////////////////////////////
/////////////////////////Initialization Functions///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

  void SDInit() {
    Serial.println("Starting SD");
    MSH.SDActive = true;
    if (!SD.begin(BUILTIN_SDCARD))
    {
      Serial.println("SD Failed");
      MSH.SDActive = false;
    }
  }

  void IMUInit() {
    if (!lsm.begin())
    {
      //imu startup failure. trip fault
      MSH.IMUActive = false;
      return;
    }
    MSH.IMUActive = true;
    IMURange(1, 1);
    IMURange(2, 1);
    IMURange(3, 1);
  }

////////////////////////////////////////////////////////////////////////////////
/////////////////////////Ram Image//////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void InitializeSdCam() {
  Serial.println("Starting SD");
  delay(3000);
  MSH.SDActive = true;
  if (!SD.begin(BUILTIN_SDCARD)) //need to check if it is also #4 on teensy 3.5
  {
    Serial.println("SD Failed");
    MSH.SDActive = false;
  }
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  cam.setImageSize(VC0706_160x120); // executed twice, with a cam.reset in the middle, because, apparently, the camera is autistic
  cam.reset();
  cam.setImageSize(VC0706_160x120);

  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  delay(1000);
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");
  delay(1000);
}

void TakePicture() { //TODO this should be based off a timed photoburst
  Serial.println("Snap in 3 secs...");
  delay(3000);
  if (! cam.takePicture()) {
    Serial.println("Failed to snap!");
  }
  else {
    Serial.println("Picture taken!");
  }

  // Create an image with the name Axxx.JPG
  char filename[9]; //used only inside this function to name the file
  strcpy(filename, "A000.JPG"); // start position

  if (SD.exists(filename)) {
    Serial.println("Apparently, it has decided the previous name exists");
    for (int i = 1; i < 1000; i++) { // int starts at 1 since case 0 is already the starter position // size goes up to 999, only three numbers, useful for specific wipe operation

      String picnumber = String(i); //necessary?

      int ilength = picnumber.length();
      int nzeros = (3 - ilength);
      String zero = "0";

      if (nzeros != 0) {
        for (int z = 1; z < nzeros; z++) {
          zero += "0";
        }
        zero += picnumber;
        picnumber = zero;
      }

      String tempname1 = "A";
      String extension = ".JPG";
      String tempname2 = tempname1 + picnumber;
      String finalname = tempname2 + extension;

      char holder[9];
      finalname.toCharArray(holder, 9);
      strcpy(filename, holder);
      Serial.print("It came up, therefore, with a new name: ");
      Serial.println(filename);

      if (!SD.exists(filename)) { //breaks the loop if name chosen is not taken
        break;
      }
    }
  }
  if (!SD.exists(filename)) {
    Serial.print("File created: ");
    Serial.println(filename);
  }


  // Open File for writing
  File imgFile = SD.open(filename, FILE_WRITE); //Open Image File

  // Get the size of the image (frame) taken
  uint16_t jpglen = cam.frameLength();
  photosize = jpglen;
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.println(" byte image.");
  //  uint8_t a[jpglen]; //has already een created in the header

  int32_t time = millis();
  pinMode(8, OUTPUT); //why 8? why do we need to have an output? Does this work well with Teensy? Testing required
  // Read all the data up to # bytes!
  int innerCount = 0; // For counting # of writes inside array a
  int wCount = 0; // For counting number of writes for normal operation
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead); //which library is this?
    //    for (int i=innerCount; i<innerCount+bytesToRead; i++){ //do we want to copy automtically and send picture immediately, or have te option to select an image to send?
    //      a[i] = buffer[i-innerCount];
    //    }
    /// for byte string comparation ///
    for (int y = 0; y < bytesToRead; y++) {
      Serial.print(buffer[y]); Serial.print(",");

      if (wCount % 128 == 0) {
        Serial.println(".");
      }
      wCount++;
      innerCount++;
    }
    ///  ///
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");
}

void ImageBuffer() { //this function is used to copy a file in the SD to an array for transmission //must still be integrated with rockblock commands
  //first, we take the umber from popcommand and open a file
  String popc = "02";
  int number = popc.toInt();
  Serial.print("number: ");
  Serial.println(number);
  //go through all files to search for one with the equivelent number:
  for (int i = 0; i < 1000; i++) { // int starts at 1 since case 0 is already the starter position // size goes up to 999, only three numbers, useful for specific wipe operation

    String picnumber = String(i); //necessary?
    String target = String(number);

    int ilength = picnumber.length();
    int nzeros = (3 - ilength);
    String zero = "0";

    if (nzeros != 0) {
      for (int z = 1; z < nzeros; z++) {
        zero += "0";
      }
      zero += picnumber;
      picnumber = zero;
    }

    String tempname1 = "A";
    String extension = ".JPG";
    String tempname2 = tempname1 + picnumber;
    String finalname = tempname2 + extension;

    char holder[9];
    finalname.toCharArray(holder, 9);
    strcpy(classfilename, holder);

    if (SD.exists(classfilename) && i == number) { //breaks the loop if we found the file with the correspondent number
      Serial.println("Target Picture Found for Buffer");
      break;
    }
    else if (!SD.exists(classfilename) && i == 999) {
      Serial.println("SD does not contain file");
      return; //break the function, like this? Something's missing
    }
  }

  File ImgtoCopy = SD.open(classfilename, FILE_READ); // We are reading the file found
  photosize2 = ImgtoCopy.size(); //here we can give the input to the true size of the array, photosize should alreay exist, though empty
  Serial.print("Photosize: ");
  Serial.println(photosize2, DEC);
  Serial.println("Starting Segmentation");
  //    uint8_t a[photosize]; // array should be initiated outside, so we can keep the value

  for (int i = 0; i < photosize2; i++) {
    a[i] = (uint8_t)ImgtoCopy.read();
    Serial.print(a[i]);
    Serial.print(",");
    if (i % 128 == 0 && i != 0) { // a dot every 128 loops
      Serial.println(".");
    }
  }

  Serial.println("");
  Serial.println("Done!");
  Serial.print("Size of a: ");
  Serial.println(sizeof(a));
  Serial.print("Photosize2: ");
  Serial.println(photosize2);
  ImgtoCopy.close();
}

void OutputB64String(uint8_t *a, int i) { //a is an array with the bytes of the picture we want to see, while i will be the size (stored in one of the photosizes) //should we store different photosizes?
  Serial.println(""); Serial.println("Printing Base 64 String of a: ");
  String b64 = Hash_base64(a, i);
  Serial.println(b64);
  Serial.println("String printed!");
  //this function is missing the code to output it, not into the serial, but for the downlink
}

void WipeAll() { // this function wipes our SD of all pictures
  Serial.println("Wiping SD Card");

  // Create the name basket to use
  char filename[9];
  Serial.println("Starting Loop Removal Operation");
  for (int i = 0; i < 999; i++) { // int starts at 0 for 000

    String picnumber = String(i); //necessary?

    int ilength = picnumber.length();
    int nzeros = (3 - ilength);
    String zero = "0";

    if (nzeros != 0) {
      for (int z = 1; z < nzeros; z++) {
        zero += "0";
      }
      zero += picnumber;
      picnumber = zero;
    }

    String tempname1 = "A";
    String extension = ".JPG";
    String tempname2 = tempname1 + picnumber;
    String finalname = tempname2 + extension;

    char holder[9]; //why does it need to be 10?? Well, it does work...
    finalname.toCharArray(holder, 9);
    strcpy(filename, holder);
    if (SD.exists(filename)) {
      SD.remove(filename);
    }
    Serial.print("Removed the following file: ");
    Serial.println(filename);
  }
}

void WipeOne() { //here as a function temporarily, until integrated into popcommands
  Serial.println("Wiping SD Card");

  // Create the name basket to use
  char filename[9];
  //next, a number received from popcommand
  String popc = "000"; //for testing only
  Serial.println("Starting Removal Operation");

  String picnumber = popc; //necessary?
  String tempname1 = "A";
  String extension = ".JPG";
  String tempname2 = tempname1 + picnumber;
  String finalname = tempname2 + extension;

  char holder[9]; //why does it need to be 10?? Well, but it does work...
  finalname.toCharArray(holder, 9);
  strcpy(filename, holder);
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  Serial.print("Removed the following file: ");
  Serial.println(filename);
  popc = ""; //needed???
}

int getNumPhotos() {
  char filename[9];
  Serial.println("Started Counting");
  int innercounter = 0;
  for (int i = 0; i < 999; i++) { // int starts at 0 for 000

    String picnumber = String(i); //necessary?

    int ilength = picnumber.length();
    int nzeros = (3 - ilength);
    String zero = "0";

    if (nzeros != 0) {
      for (int z = 1; z < nzeros; z++) {
        zero += "0";
      }
      zero += picnumber;
      picnumber = zero;
    }

    String tempname1 = "A";
    String extension = ".JPG";
    String tempname2 = tempname1 + picnumber;
    String finalname = tempname2 + extension;

    char holder[9]; //why does it need to be 10?? Well, it does work...
    finalname.toCharArray(holder, 9);
    strcpy(filename, holder);
    if (SD.exists(filename)) {
      innercounter += 1;
      Serial.print("Found the following file: ");
      Serial.println(filename);
    }
  }
  Serial.print("Total Count: ");
  Serial.println(innercounter);
  return innercounter;
}



////////////////////////////////////////////////////////////////////////////////
/////////////////Command functions//////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////



class commandBuffer {
  public:
    int commandStack[200][2];
    int openSpot;
    commandBuffer() {
      commandStack[200][2] = { -1};
      openSpot = 0;
    }
    void print() {
      //Serial formatting and Serial output
      int i = 0;
      endT = millis() + manualTimeout;
      while (i < 200 && millis() < endT) {
        if (commandStack[i][0] == -1 && commandStack[i][1] == -1) {
          break;
        }
        i++;
      }
    }
};
commandBuffer cBuf;



void commandParse() {

  int commandData;
  int commandType; // changed from int to long int. so if it breaks this is probably why
  bool l = true;
  String i = "";
  i = receivedChars;

  while (l) {

    commandType = (receivedChars.substring(0, receivedChars.indexOf(","))).toInt();

    commandData = (receivedChars.substring(receivedChars.indexOf(",") + 1, receivedChars.indexOf("!"))).toInt();

    cBuf.commandStack[cBuf.openSpot][0] = commandType;
    cBuf.commandStack[cBuf.openSpot][1] = commandData;
    if (receivedChars.indexOf("!") == receivedChars.length() - 1) {
      l = false;

    } else {

      receivedChars = receivedChars.substring(receivedChars.indexOf("!") + 1);
      i = receivedChars;

    }
    cBuf.openSpot++;
  }
}



void recvWithEndMarker() {
  // for serial
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

//  while ( Serial.available() > 0 && newData == false) {
//
//    rc = Serial.read();
//
//    if (rc != endMarker) {
//      receivedChars += (char(rc));
//    }
//    else {
//
//      newData = true;
//    }
//  }
  char copy[50]; //can be made smaller
  receivedChars.toCharArray(copy,7);
  Serial.println(copy);
  //is last place an exclamation mark?
  if (copy[7] == '!'){
    newData = true;
  } else {
    Serial.println("Command does not end in '!'");
  }

}



boolean isInputValid() {
  // todo make timeout
  Serial.print("starting parse on:");
  Serial.println(receivedChars);
  //Check if incoming command string <input> is valid
  int lastPunc = 0; //1 if ",", 2 if "!", 0 Otherwise
  bool valid = true;
  int q = 0;
  int l = receivedChars.length();

  if (l < 4) {
    Serial.println("command too short");
    valid = false;
  }

  while (q < l) {
    char currentChar = receivedChars[q];
    //Serial.println(currentChar,HEX);
    q++;

    if (isPunct(currentChar)) {
      if (currentChar == (',')) {
        //Check if last was a period
        Serial.println("Comma Found");
        if (receivedChars[q - 2] == '!') {
          Serial.println("No First Command Number");
          valid = false;
          break;
        }
        if (lastPunc == 0 || lastPunc == 2) {
          Serial.println("Comma OK");
          lastPunc = 1;
        } else {
          Serial.println("2 Commas");
          valid = false;
          break;
        }
      } else if (currentChar == ('!')) {
        if (receivedChars[q - 2] == ',') {
          Serial.println("No Second Command Number");
          valid = false;
          break;
        }
        Serial.println("Excl Found");
        if (lastPunc == 1) {
          Serial.println("Period ok");
          lastPunc = 2;
        } else {
          Serial.println("2 Excl or No prior comma");
          valid = false;
          break;
        }
      } else if (currentChar == ('-')) {
        Serial.println("Hypen Found");
        if (receivedChars[q - 2] == ',') { //q incremented after value capture
          Serial.println("Negative Sign ok");
        } else {
          Serial.println("Hyphen in wrong place");
          valid = false;
          break;
        }
      } else if (currentChar == ('.')) {
        Serial.println("period ok");
      } else {
        Serial.println("Invalid Punc");
        valid = false;
        break;
      }
    } else if (isAlpha(currentChar)) {
      //Serial.println("Alpha");
      valid = false;
      break;
    } else if (isSpace(currentChar)) {
      //Serial.println("Space");
      valid = false;
      break;
    }

    //Detect no ending exclamation point
    if (q == receivedChars.length() - 1) {
      if (receivedChars[q] != '!') {
        //Serial.println("No Ending");
        valid = false;
        break;
      }
    }
    //Null Character in the middle
    if (currentChar == '\0' && q != receivedChars.length() - 1) {
      Serial.println("null character");
      valid = false;
      break;
    }
  }
  if (valid) {
    Serial.println("valid");
  }
  if (!valid) {
    Serial.println("invalid");
  }
  return valid;
}



void popCommands() {
  //Process all the Incoming Commands
  long start = millis();

  while (cBuf.openSpot > 0 && millis() - start < manualTimeout) {
    if (cBuf.openSpot > 0) {

      //Serial.println (cBuf.openSpot - 1);
      int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0], cBuf.commandStack[cBuf.openSpot - 1][1]};
      cBuf.commandStack[cBuf.openSpot - 1][0] = -1;
      cBuf.commandStack[cBuf.openSpot - 1][1] = -1;
      cBuf.openSpot --;

      //Supported Commands
      switch (currentCommand[0]) {
        case (1):
          MSH.PrintStatus();
          int a;

          while (a < MSH.binOpenSpot) {

            Serial.print(MSH.binArr[a]);
            a++;
          }
          Serial.println("");
          Serial.print("binary array length: ");
          Serial.print(MSH.binOpenSpot / 8);
          Serial.println(" bytes");

          MSH.binOpenSpot = 0;


          break;

        case (2):
          Serial1.print(F("1,1!\n")); // <---found the secret to Serial1 :)
          break;

        case (3):
          Serial1.println(F("1,0!\n"));
          break;

        case (4):
          Serial.print("Magnetometer X:"); Serial.print(MSH.MagAve[0]);
          Serial.print(" Y:");  Serial.print(MSH.MagAve[1]);
          Serial.print(" Z:"); Serial.println(MSH.MagAve[2]);
          Serial.print("Gyroscope X:"); Serial.print(MSH.GyroAve[0]);
          Serial.print(" Y:");  Serial.print(MSH.GyroAve[1]);
          Serial.print(" Z:"); Serial.println(MSH.GyroAve[2]);
          Serial.print("Accelerometer X:"); Serial.print(MSH.AccelAve[0]);
          Serial.print(" Y:");  Serial.print(MSH.AccelAve[1]);
          Serial.print(" Z:"); Serial.println(MSH.AccelAve[2]);
          break;

        case (5):
          Serial.println(currentCommand[1]);
          break;

        case (20): // 20 downlink commands
          MSH.SRFreq = currentCommand[1]; //how many regular dls before a special dl
          break;

        case (21):
          Serial.print("Command recieved, command type is 21, Command data is");
          Serial.println(currentCommand[1]);
          break;

        case (22):
          MSH.SRTimeout = (1000 * currentCommand[1]); //how long should the code wait for the transmission to be sent (seconds)
          break;

        case (23):
          MSH.uplinkPeriod = (1000 * currentCommand[1]);
          break;

        case (30): //toggle active fault //todo test this
          if (MSH.faultReport[currentCommand[1]] = 0) {
            MSH.faultReport[currentCommand[1]] = 1;
          } else {
            MSH.faultReport[currentCommand[1]] = 0;
          }
          break;

        case (40): //Begin Image Downlink

          break;

        case (41): //Take Photos
          delay(1000);
          //MSH.CameraBurst = true; // do we want a burst or a single photo at a time?
          //sendSCommand(F("41,1!"));
          TakePicture();
          Serial.println(F("\nPhotoBurst Initiated"));
          break;

        case (42): { //Set PhotoBurst Time in seconds // do we want this?
            String com = "42," + String(currentCommand[1]) + "!";
            MSH.BurstDuration = currentCommand[1] * 1000;
            Serial.println(F("\nPhotoBurst Time Set"));
            //sendSCommand(com);
            break;
          }

        case (43): { //Get # of Available Photos // function needs to be built. Can depend on how we want to use the variable though
            MSH.numPhotos = getNumPhotos();
            Serial.println("\nPhotos Available: " + String(MSH.numPhotos));
            break;
          }

        case (44): { //switch downlinkSize to 200
            downlinkSize = 200;
            Serial.println("Changed downlinkSize to 200");
            break;
          }

          
        case (45): { //switch downlinkSize to 50
            downlinkSize = 50;
            Serial.println("Changed downlinkSize to 200");
            break;
          }

        case (50): //Wipe SD Card
          Serial.println(F("\nWiping SD Card"));
          //sendSCommand(F("47,1!"));
          WipeAll();
          break;

        case (51): //Wipe Picture 1 from SD Card //discuss how to erase different pictures at our command
          Serial.println(F("\nWiping chosen picture from SD Card"));
          //sendSCommand(F("47,1!"));
          WipeOne(); //function needs modification to take into account popcommand number
          break;

        case (52): //re-init SD card
          SDInit();

        case (60): //accelerometer scale
          IMURange(1, currentCommand[1]);
          break;

        case (61): //magnetometer scale
          IMURange(2, currentCommand[1]);
          break;

        case (62): //gyroscope scale
          IMURange(3, currentCommand[1]);
          break;

        case (63): // attempt to re-initialize IMU
          IMUInit();

        case (80): // 80 gyro commands
          MSH.XGyroThresh = currentCommand[1];
          break;

        case (81):
          MSH.YGyroThresh = currentCommand[1];
          break;

        case (82):
          MSH.ZGyroThresh = currentCommand[1];
          break;

        case (90): // 90 accelerometer commands
          MSH.XAccelThresh = currentCommand[1];
          break;

        case (91):
          MSH.YAccelThresh = currentCommand[1];
          break;

        case (92):
          MSH.ZAccelThresh = currentCommand[1];
          break;

        case (101): //100s are for security commands such as faultcheck and overwrite operations
          MSH.OVERWRITE = true; //activate OVERWRITE mode. Act responsibly!
          break;

        case (102):
          MSH.OVERWRITE = false; //activate OVERWRITE mode. Act responsibly!
          break;

        case (103): //
          MSH.overdown = currentCommand[1]; //makes downlinkPeriod equal to second number of command
          break;

        case (104):
          MSH.overup = currentCommand[1]; //makes uplinkPeriod equal to second number of command
          break;

        case (105):
          MSH.overmode = currentCommand[1]; //makes nextMode equal to second number of command
          break;

        case (110):
          MSH.faultReport[1] = 1; //trip fault protection for imu init failure
          break;
      }
    } else {
      Serial.println("No Command");
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
//////////////////////Setup & loop//////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


void setup() {
  //initialize
  Serial.begin (74880);
  Serial3.begin (74880); //either 1 or 3. Must be consistent with header of the script
  Serial5.begin (38400); //the camera is connected to serial 6 right? No, it's on serial 1
  //Wire.begin();
  Serial.println("Bitch, its starting?!");
  delay(1000);
  cBuf = commandBuffer();
  //buildFaults();
  //sensorInit();
  RBInit();
  Serial.println("Setup complete!");
  delay(1000);
  //  while (millis() <= 2700000){
  //    //wait here for dorman cruise
  //  }
}

void loop() {
  Serial.println("Calling defaultFunctions (Modified)");
  delay(1000);
  defaultFunctions();
  Serial.println("Calling modeCon (Modified)");
  modeCon(); //if we command a mode, we must be able to ignore this line, a boolean?
  Serial.println("Calling commCon (Modified)");
  delay(1000);
  commCon(); //has DLConf inside of it, which calls the downlink.uplink functions
}

////////////////////////////////////////////////////////////////////////////
////////////////Default Functions///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void defaultFunctions() {
  //faultCheck();
  popCommands();
  recvWithEndMarker();

  if (newData) {
    if (isInputValid() && (receivedChars != "")) {
      commandParse();
    }
    //incorrect format, tell ground //missing function here
    newData = false;
    //Serial.println("wiping serial buffer");
    receivedChars = "";
  }
  //ModeSwitch(); 
  //powerMan(); //we must be able to command it if we wish so, unless we get into low power mode. danger is having the rockblock interrupt its work. could it damage the radio?
}


void powerMan() { //should I have a default DL freq for eclipse? //how much power we use on uplink?
  //  int i = map(MSH.Battery,minimum,maximum,100,50); // map battery reading to approx charge %
  int i; //deleteme
  if ((MSH.OVERWRITE) && (i > 20)){ //only works when OVERWRITE = true, and we are above safety voltage (20 for now)
    //OVERWRITE IS TRUE //U BETTA KNOW WHATCHU DUIN
    //int overdown = ; //downlink period from popcomands 
    //int overup = ; //uplink period from popcommands
    
    switch (MSH.overdown) { //takes value given by popcommands for downlink Period
      case 51 ... 60:
        MSH.downlinkPeriod = 1800000;//30 min
        break;

      case 61 ... 70:
        MSH.downlinkPeriod = 600000;//10 min
        break;

      case 71 ... 80:
        MSH.downlinkPeriod = 300000;//5 min
        break;

      case 81 ... 90:
        MSH.downlinkPeriod = 60000;//1 min
        break;

      case 91 ... 110: //max set to 110 incase the range i set up is a little off
        MSH.downlinkPeriod = 5000;// 5 sec
        break;

    }

    switch (MSH.overup) { //takes value given by popcommands for uplink Period
      case 51 ... 60:
        MSH.uplinkPeriod = 1800000;//30 min
        break;

      case 61 ... 70:
        MSH.uplinkPeriod = 600000;//10 min
        break;

      case 71 ... 80:
        MSH.uplinkPeriod = 300000;//5 min
        break;

      case 81 ... 90:
        MSH.uplinkPeriod = 60000;//1 min
        break;

      case 91 ... 110: //max set to 110 incase the range i set up is a little off
        MSH.uplinkPeriod = 5000;// 5 sec
        break;

    }
    
  } else {  //else for overwrite state //normal operations
    
  if (i <= 50) { // i comes from voltage read
    nextMode = 3; // enter low power
    //dlfreq minimum maybe an hour?
    
  } else { //else in case we have enough power
    
    switch (i) { //first number is min value second is max
      case 51 ... 60:
        MSH.downlinkPeriod = 1800000;//30 min
        break;

      case 61 ... 70:
        MSH.downlinkPeriod = 600000;//10 min
        break;

      case 71 ... 80:
        MSH.downlinkPeriod = 300000;//5 min
        break;

      case 81 ... 90:
        MSH.downlinkPeriod = 60000;//1 min
        break;

      case 91 ... 110: //max set to 110 incase the range i set up is a little off
        MSH.downlinkPeriod = 5000;// 5 sec
        break;

    }
      switch (i) { //same as above, but determines the uplink Period instead of Downlink
      case 51 ... 60:
        MSH.uplinkPeriod = 1800000;//30 min
        break;

      case 61 ... 70:
        MSH.uplinkPeriod = 600000;//10 min
        break;

      case 71 ... 80:
        MSH.uplinkPeriod = 300000;//5 min
        break;

      case 81 ... 90:
        MSH.uplinkPeriod = 60000;//1 min
        break;

      case 91 ... 110: //max set to 110 incase the range i set up is a little off
        MSH.uplinkPeriod = 5000;// 5 sec
        break;
    }
  }
  }
}

////////////////////////////////////////////////////////////////////////////
////////////////Mode Control////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void ModeSwitch() { // to do plan out the switching mechanisms
  int i = 50; //delete me
  if ((MSH.OVERWRITE) && (i > 20)){ //only works when OVERWRITE = true, and we are above safety voltage (20 for now) //should we include the voltage safety here?
  //OVERWRITE IS TRUE //U BETTA KNOW WHATCHU DUIN
  // int MSH.overmode = ; //mode user order from popcomands 

  switch (MSH.overmode) {
     case (0):
        nextMode = 0; //MNormal
        break;  //we need the break?
        
     case (1):
        nextMode = 1; //detumble
        break;  //we need the break?

     case (2):
        nextMode = 2; //MSafeHold
        break;  //we need the break?

     case (4):
        nextMode = 4; //eclipse // now just MLowPower
        break;  //we need the break?

     case (5):
        nextMode = 5; //MDeploy
        break;  //we need the break?
  }
    
  } else { //only used if overwrite is off //should we turn it off after a while? Say a week?

    
    //How to get into NORMAL MODE 
    //if voltage is above 3.2
    //if current/power is above a certain threshold-> not recommended
    //will not transition to detumble
    //if a piture is being taken, will drop voltage threshold to 3.0 to complete action, and then it raises back to 3.2
    if (i>50){ //always gets in this if it has enough power // is this needed
      nextMode = 0; //MNormal //important to have it biased towards normal operations
    }

    //How to get into DETUMBLE MODE
    //Functions for inhibitors were activated by change -> recognizes it has been released
    //Or IMU numbers have destabilized
    //Or by expressive order
    float g = .2;
    if (MSH.SolarCurrent >= (MSH.CurrentZero + g)) {//todo determine true g
      nextMode = 4; //eclipse // now just MLowPower
    }
    
    //How to get into MSAFEHOLD MODE
    //Fault tripped
    if (true) {//has a fault been tripped
      nextMode = 2; //MSafeHold
    }

    //How to get into LOWPOWER MODE
    //If voltage drops below 3.2
    //if current/power is above a certain threshold -> not recommended
    
    if (false) { //spinning too fast on any axis
      nextMode = 1; //detumble
    }

    //How to get into DEPLOY MODE
    //Two keys (booleans) transition by manual order
    if (DLKey1 && DLKey2) { //we need to send a command for this one to activate. We don;t want it running the OP by itself without us noticing. (probably DLKey1&2)
      nextMode = 5; //MDeploy
    }
  }
}


void modeCon() { //does not need overwrite mod
  z = millis();
  //  faultCheck(); activate when fault check is ready

  switch (nextMode) {

    case 0:
      // normal ops
      MNormal();
      break;

    case 1:
      // detumble
      MDetumble();
      break;

    case 2:
      // safe hold
      MSafeHold();
      break;

//    case 3:
//      MEclipse(); // to erase when sure
//      break;

    case 4:
      MLowPower();
      break;

    case 5:
      MDeploy();
      break;
  }
  // use to check cycle speed
  //  Serial.println("////////////////////////////////////////");
  //  Serial.print("cycle completed in ");
  //  Serial.print(millis() - z);
  //  Serial.println(" milliseconds");
  //  Serial.println("////////////////////////////////////////");


}

void commCon() { //does not need overwrite mod
    Serial.println("Calling ComCon, creaing fake data!");
    uint8_t testdata[] = {70,0,1,2,3,4,5,6,7,8,9,10,11,12};
    for(int i = 0; i<14; i++){
      buf[i] = testdata[i];
      openSpot++;
    }
    Serial.print("openSpot = ");
    Serial.println(openSpot);
//  if ((lastSend + MSH.downlinkPeriod) <= (millis()) && !DLConf) { // check if we're due for a downlink, also dont run when we're still waiting for DL confirmation
//    Serial.println("Starting Downlink");
//    Downlink();
//  }
  openSpot = 0;

  if (lastReceive + MSH.uplinkPeriod <= millis()) {
    Uplink();
    //sbd sendrecieve text
  }
  // every x minutes initiate a null downlink to update
  // if there are any incoming messages. if so call read function
  // or maybe don't? every time we downlink we'll know if there are
  // incoming messages

  if (DLConf) {
    DownlinkCheck();
  }
}

void MNormal() { //each mode should have a set of initial conditions in case we need to enforce them //perhaps, we should have presets, but the normal operations are run in a more fluid way?
  //Write presets for normla operations
    Serial.println("MNORMAL Starting!");
  //  checkTime();
  //initializeRB();
  //updateSensors();
}

void MDetumble() {
  //Serial.println ("yay im detumbling");
  //ACS Code Here
  nextMode = 0;
}

void MSafeHold() {
  //Serial.println("im safe... for now");
  //Change booleans for usage by modeCon, and also run specific trnamisison to identify fault
  //Here must be code that is able to recieve commands to ignore a certain fault
  popCommands();
  recvWithEndMarker();
  updateSensors();
}

void MLowPower() {
  //Serial.println("low power");
  //change rockbloc frequency //but already done with powerman? //Have PowerMan call MOdeSwitch to set to this stage, which then houses new parameters. This mode is runseveral times through loops, 
  //always checking and making adjustments
  
}

void MDeploy() { //mission related. receives th two keys and executes the commands i the right order. Once data is taken, it analyzes and sends a check to us, as well as the data it used to confirm the mission's success
  int g; //todo determine proper value for g also check door sense logic

  if (deployStage == 0) {
    digitalWrite(35, HIGH); //todo has the burnwire pin changed
    burnTimer = millis();
    deployStage = 1;
  }

  if ((deployStage == 1) && (MSH.lightSense >= g) && (MSH.DoorButton = 1)) { //confirm that this is VERY consistent

    deployStage = 2;
    digitalWrite(35, LOW); //todo has the burnwire pin changed
    MSH.burstStart = millis();
  }

  if (deployStage == 2) {

    if ((MSH.BurstDuration + MSH.burstStart) <= millis()) {
      //success!!!
      deployStage = 0;
      DLKey1 = false;
      DLKey2 = false;
    } else {
      TakePicture();
    }
  }

  if (((burnTimer + 45000) <= millis()) && deployStage == 1) {
    digitalWrite(35, LOW); //todo has the burnwire pin changed
    //burnwire fail fault
    DLKey1 = false;
    DLKey2 = false;
    deployStage = 0;
  }

}

void faultCheck() { // todo make this track which fault was tripped
  // reads fault string and checks that no threshold has been tripped
  // todo if a sensor is off be sure you gave it enough time to init
  int f = 0;
  bool fault = false;

  if (!faultTripped) {
    ReportOpenSpot = 0;
  }


  while (f < faultSize) {

    int CurInt = MSH.faultReport[f];

    if (CurInt = 0) {
      //ignore
    } else {

      switch (f) { //this does not increment f //cases can neverthless be added

        case (1):
          //          Serial.println("case 1 tripped");
          //          if (MSH.Gyro[0] > MSH.XGyroThresh) {
          //            fault = true;
          //          }
          break;

        case (2):
          //Serial.println("case 2 tripped");
          break;

        case (3):
          // camera active?
          if (!MSH.CamActive) {
            MSH.faultReport[ReportOpenSpot + 1]  = MSH.CamActive;
            fault = true;
          }
          break;

        case (4):
          // SD card active?
          //if (SlaveCompFlag) { not needed anymore
            if (MSH.SDActive) {
              break;
            } else {
              MSH.faultReport[ReportOpenSpot + 1]  = MSH.SDActive;
              fault = true;
            }
          
          break;


        case (6):
          if (!MSH.IMUActive) {
            MSH.faultReport[ReportOpenSpot + 1]  = MSH.IMUActive;
            fault = true;
          }
          break;

        case (7):
          if (!MSH.RBActive) {
            MSH.faultReport[ReportOpenSpot + 1]  = MSH.RBActive;
            fault = true;
          }
        break;
        case (8): //IMU Magnetorquer x off
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (9): //IMU Magnetorquer y off
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (10): //IMU Magnetorquer z off
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (11): //Battery reading off
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (12): //Solar Panel Voltage Reading off (?or maybe other issues, need to work with them to notice possible problems))
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (13): //Current Sensor is off
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (14): //Rockblock signal quality is zero for too long
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (15): //Rockblock has failed to downlink for more than 5 times
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (16): //Uplink had not been fully uploaded (need code on each transmit to keep track of packages)
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (17): //Rockblock has no signal at all (ISBD_SUCCESS is false for too long)
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (18): //Temp sensor is off
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (19): //photoresistor is off
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (20): //A door button is being pressed when it shouldn't (there will be a case for each)
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (21): //Nitinol Wire actiavted, but photoresistor does not detect difference in light
          if (true){ //juts to pass compile   //have we failed to open the compartment? Have it take a pic and the rockblock send a picture
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (22):
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (23):
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
        case (24):
          if (true){ //juts to pass compile
            MSH.faultReport[ReportOpenSpot +1] = 1;
          }else{
            fault = true;
          }
        break;
          

      }
      if (fault) {
        faultTripped = true; // enter safe hold mode
        MSH.faultReport[ReportOpenSpot] = f;
        ReportOpenSpot++;
        ReportOpenSpot++;
      }
      fault = false;
    }
    f++;
  
  if (faultTripped) {
    for (int g = 0; g < ReportOpenSpot; g++) {
      buf[g] = MSH.faultReport[g];
    }
    openSpot = ReportOpenSpot;
    if (sendFault == true) {
      Downlink();
    } else {
      openSpot = 0;
    }
  }
  if (faultTripped && !sendFault){
    faultTripped = false;
  }
}
}
void buildFaults() { //fills faultReport with zeroes, with 1s (ones) representing Faults tripped
    int f = 0;
    while (f < faultSize) {
      MSH.faultReport[f] = 0;
      f++;
    }
  }

void FaultDownlink() { //take faultReport and puts into the buf for downlink for routine special report
    //int f = 0;
    //String g = "";
    //assumes openSpot = 0
    while(openSpot<faultSize) {
      buf[openSpot] = MSH.faultReport[openSpot];
      openSpot++;
    }
    //MSH.FaultString = g;
  }

  ////////////////////////////////////////////////////////////////////////////
  ////////////Communication fucntions/////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  // rockblock code is basically finished do not want to integrate quite yet. maybe once we have imu up & running?



  void RBInit() {
    //initalize rockblock
  IridiumSerial.begin(19200);
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  Serial.println("Starting modem...");
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    return;
  }
  for (int i = 0; i<256; i++){ //create a for testing
    a[i]=i;
  }

    
    if (true) {
      MSH.RBActive = true;
    } else {
      MSH.RBActive = false;
    }
    modem.adjustSendReceiveTimeout(60);
  }

  void stageDownLink() { //rework downlink todo  IS THIS DOING ANYTHING USEFUL?
    long int i = micros(); //return microseconds since the arduino board began running the current program. this number will overflow (go back to zero) after approxmately 70 minutes
    //  checkTime();
    String msg;
    MSH.lastSR++; //basically, if the number of max normal reports has been reached, the next if statement builds the special report, and sends that instead, reseting the counter so that next message is a normal report again
    uint32_t SpecialReport[] = {MSH.IMUActive, MSH.MagAve[0], MSH.MagAve[1], MSH.MagAve[2], MSH.GyroAve[0], MSH.GyroAve[1], MSH.GyroAve[2], MSH.AccelAve[0], MSH.AccelAve[1], MSH.AccelAve[2], MSH.MagLog[0][0],MSH.GyroLog[0][0], MSH.AccelLog[0][0], MSH.MagLog[0][1],MSH.GyroLog[0][0], MSH.AccelLog[0][0], MSH.MagLog[0][0],MSH.GyroLog[0][0], MSH.AccelLog[0][0], MSH.ImuTemp, MSH.Temp, MSH.SolarCurrent, MSH.Battery, MSH.DoorButton, MSH.Inhibitor_19a, MSH.Inhibitor_10b, MSH.Inhibitor_2, MSH.FREEHUB, nextMode, downlinkSize, MSH.downlinkPeriod, MSH.uplinkPeriod, MSH.MTQueued, MSH.SBDIXFails};
    if (MSH.lastSR > MSH.SRFreq /*|| booleanforspecialreportrequest*/) { // int can change based on how often you'd like special data.
      MSH.StageReport = true;
      openSpot = 0;//rewriting buf
      FaultDownlink(); // update currently active faults //fills buf with faults first instead
      
      //SpecialReport above has 34 array elemtns, so it goes from 0 to 33
      int openSpotholder = openSpot;
      while(openSpot < 33+openSpotholder){
        buf[openSpot]= SpecialReport[openSpot-33]; //we are filing out the rest of the buf with the rest of the special report
        openSpot++;
        }
    }
    if (MSH.StageReport) { //this should reset the counter
      String u = "";
     // u += SpecialReport();
      msg += u;
      MSH.StageReport = false;
      MSH.lastSR = 0;
    }
    //  downLink(msg);
  }

  void routineDownlinkData() { //updating the varaible data we receive with downlink, useful to save data in case we need to retrieve it
    // do we want to receive MagLog, GyroLog, AccelLog?
    // do we want to include DoorButton, lightSense and CurrentZero?
    
    if (nextMode!=2){
    uint8_t tempholder[14] = {21, MSH.MagAve[0], MSH.MagAve[1], MSH.MagAve[2], MSH.GyroAve[0], MSH.GyroAve[1], MSH.GyroAve[2], MSH.AccelAve[0], MSH.AccelAve[1], MSH.AccelAve[2], MSH.ImuTemp, MSH.Temp, MSH.SolarCurrent, MSH.Battery};
    for (int i = 0; i < 14; i++) {
      MSH.VarHolder[i] = tempholder[i];
      //MSH.lastSR++;
    }
    } else { //scrap this, if we on fault mode, we send a special report everytime we can, instead of typical routine
    uint8_t tempholder[14] = {22, MSH.MagAve[0], MSH.MagAve[1], MSH.MagAve[2], MSH.GyroAve[0], MSH.GyroAve[1], MSH.GyroAve[2], MSH.AccelAve[0], MSH.AccelAve[1], MSH.AccelAve[2], MSH.ImuTemp, MSH.Temp, MSH.SolarCurrent, MSH.Battery};
    for (int i = 0; i < 14; i++) {
      MSH.VarHolder[i] = tempholder[i];
    }
     
    }
  }

  void Downlink() { // outgoing data must be placed in buf before downlink is called
  delay(1000);
  int signalQuality = -1;
  int err;

  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("On a scale of 0 to 5, signal quality is currently ");
  Serial.print(signalQuality);
  Serial.println(".");
  delay(1000);
    if (DLConf == false && signalQuality > 0) {// is there still an outgoing message awaiting confirmation //make sure this is no a waste of time, rockblock may be able to transmit with signal quality zero!
      x = modem.sendSBDBinary(buf, (openSpot - 1)); // sends from 0 to openspot - 1 (all space in array used)
      imgsent = false;
      sendFault = false;
      lastSend = millis();
      // for now assume that we could fit the regular transmission and the special report on the same downlink
      // clear buffer of all sent data. be sure that the message has been sucessfully transmitted.
      DLConf = true;
    }
  }

  void DownlinkCheck() { // Should now be finished
    Serial.print("downlinkcheck: ");
    Serial.println(DLFailCounter);
    Serial.print("ISBD Sucess Status: ");
    Serial.println(ISBD_SUCCESS);
    Serial.print("X value: ");
    Serial.println(x);

    if (x == ISBD_SUCCESS && imgsent) { //imgsent is a boolean used to verify if the last msg sent was a segment of our image
      Serial.println("Last msg was an imag, and we have received confirmation it was successfully transmitted");
      imgsent = false;
      Serial.println("What we received: ");
      for (int i = 0; i < min(downlinkSize, bytesleft); i++) {
        Serial.print(buf[i]); Serial.print(",");
        if (i % downlinkSize == 0) {
          Serial.println("");
        }
      }
      piccounter += min(downlinkSize, bytesleft); //advances the beginning of the copy from array a to buf
      bytesleft -= min(downlinkSize, bytesleft); //tells us how much of th buf will be busy with bytes from image //bytesleft, imgsent, and counter should only be changed by the checker, guaranteeing we are always sending fresh bytes at the right time
      openSpot = min (downlinkSize, bytesleft);
      Serial.print("bytesleft: "); Serial.println(bytesleft);
      Serial.print("piccounter: "); Serial.println(piccounter);


      if (bytesleft == 0) {
        Serial.print("bytesleft is equal to zero. Message sent. Setting counter = 0; isFirst = true;");
        piccounter = 0; //resets counter for next image
        isFirst = true; //resets boolean, since we will be strating again.
      } else {
        PicDownlink();
      }


    }

    if (x == ISBD_SUCCESS) { //Everything worked out, reset everything needed to go again
      Serial.println("Success!");
      openSpot = 0;
      lastSend = millis();
      DLConf = false;
      DLFailCounter = 0;
      exit; //what is this?
    }
    if ((lastSend + MSH.downlinkPeriod + MSH.SRTimeout) <= millis()) {  //sr timeout is the max wait after the supposed transmission
      DLFailCounter++;
      if (!imgsent) {
        openSpot = 0;
        lastSend = millis();
        DLConf = false;
      } else {
        PicDownlink();
        if (DLFailCounter >= 3) {
          DLFailCounter = 0;
          DLConf = 0;
          openSpot = 0;
          imgsent = false;
          piccounter = 0;
          //todo trip downlink fault
        }
      }

    }

  }

  void Uplink() { // function to receive commands
    Serial.println("top of uplink");
    delay(1000);
    int signalQuality = -1;
  int err;
  err = modem.getWaitingMessageCount();
  Serial.print("Message Count: "); Serial.println(err);

  err = modem.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    //exit(1);
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);
  
  // Read/Write the first time or if there are any remaining messages
  if(signalQuality>1){
  //if ( modem.getWaitingMessageCount() > 0){
    Serial.println("There's a message waiting for us!");
    //size_t bufferSize = sizeof(buffer);
    //Serial.println(sizeof(buffer));
    // First time through send+receive; subsequent loops receive only
//    if (!messageSent)
//      err = modem.sendReceiveSBDBinary(testmsgbuffer, testmsgbufferSize, rxBuffer, rxBufferSize);
//    else
      Serial.println("Receiving now!");
      err = modem.sendReceiveSBDText(NULL, rxbuffer, rxBufferSize);
      
    if (err != ISBD_SUCCESS)
    {
      Serial.print("sendReceiveSBD* failed: error ");
      Serial.println(err);
    }
    else // success!
    {
      //messageSent = true;
      Serial.print("Inbound buffer size is ");
      Serial.println(rxBufferSize);
      for (int i=0; i<rxBufferSize; ++i)
      {
        Serial.print(rxbuffer[i], HEX);
        if (isprint(rxbuffer[i]))
        {
          Serial.print("(");
          Serial.write(rxbuffer[i]);
          Serial.print(")");
          Serial.print(" ");
          Serial.print("(");
          Serial.print(rxbuffer[i]);
          Serial.print(")");
          rxOutput = rxbuffer[i];
          rxOutputCom += String(rxOutput);
          Serial.print(" ");
          Serial.print("(");
          Serial.print(rxOutput);
          Serial.print(")");
        }
        Serial.print(" ");
      }
      Serial.println();
      Serial.print("Total String: ");
      Serial.println(rxOutputCom);
      Serial.println();
      Serial.print("Messages remaining to be retrieved: ");
      Serial.println(modem.getWaitingMessageCount());

      receivedChars = rxOutputCom;
      receivedSize = rxBufferSize;
      rxOutput = "";
      rxOutputCom = "";
    }
  //}
  }
  }

  void PicDownlink() {
    Serial.println("We've entered PicDownlink Function");
    //First we start with the segmentation of an image on array a[5120], which happens procedurally, according to the success of the transmission
    if (isFirst) { //for first segment, where we set up bytesleft
      Serial.println("It seems this is the first segment of our image");
      bytesleft = photosize2;
      openSpot = min(340, bytesleft);
      isFirst = false; //set up complete
    }

    //for (uint16_t counter=0; counter<=photosize2; counter += min(340,bytesleft)){ //counter to go through the array, goes up to photosize2(name to change)
    //build correct buf with the correct spots on image array, run once per counter loop
    Serial.println("building correct buf for transmission");
    buf[0] = 71; //serial code to identify this buf has a picture, we load the picture afterwards
    for (int i = 1; i < min(339, bytesleft); i++) {
      //if
      buf[i] = a[piccounter + i];
    }
    Serial.print("How many spots of the buf we are using: ");  Serial.println(min(340, bytesleft));
    x = modem.sendSBDBinary(buf, min(340, bytesleft)); // sends from 0 to openspot - 1 (all space in array used)
    // for now assume that we could fit the regular transmission and the special report on the same downlink
    // clear buffer of all sent data. be sure that the message has been sucessfully transmitted.
    imgsent = true; //last img sent is an image, needed for downlink check!
    DLConf = true; //this will call the checker indirectly

  }

  ////////////////////////////////////////////////////////////////////////////
  ////////////////Sensor Functions////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  void updateSensors() { //do a full sensor sweep
    checkTime();
    checkBatteryVoltage();
    readIMU(); //TODO should I immediately upload theses reading to the slave comp?
    checkBatteryVoltage();
    currentSense();
    zeroCurrent();
    lightSense();
    doorSense();
    checkTemp();
    InhibitorsCheck();
  }

  void checkTime() { //todo
    long timeNow = millis();
    String timeString = "";

    int days = timeNow / day ;//number of days
    int hours = (timeNow % day) / hour;
    int minutes = ((timeNow % day) % hour) / minute ;//and so on...
    int seconds = (((timeNow % day) % hour) % minute) / second;

    // digital clock display of current time
    //  Serial.print(days, DEC);
    //  printDigits(hours);
    //  printDigits(minutes);
    //  printDigits(seconds);
    //  Serial.println();

    timeString += String(days, DEC) + ":";
    timeString += String(hours, DEC) + ":";
    timeString += String(minutes, DEC) + ":";
    timeString += String(seconds, DEC);

    Serial.println(timeString);

  }

  void printDigits(byte digits) {
    // utility function for digital clock display: prints colon and leading 0
    Serial.print(":");
    if (digits < 10) {
      Serial.print('0');
      Serial.print(digits, DEC);
    }
  }

  void readIMU() { //theres no elegant way to do this ;_;
    lsm.read();
    MSH.Mag[0] = ((long int)lsm.magData.x);
    MSH.Mag[1] = ((long int)lsm.magData.y);
    MSH.Mag[2] = ((long int)lsm.magData.z);
    MSH.Gyro[0] = ((long int)lsm.gyroData.x);
    MSH.Gyro[1] = ((long int)lsm.gyroData.y);
    MSH.Gyro[2] = ((long int)lsm.gyroData.z);
    MSH.Accel[0] = ((long int)lsm.accelData.x);
    MSH.Accel[1] = ((long int)lsm.accelData.x);
    MSH.Accel[2] = ((long int)lsm.accelData.x);
    MSH.ImuTemp = ((int)lsm.temperature);
  }

  void fillIMUBuffer() {
    if (SenseCounter <= SenseNumb) {

      for (int j = 0; j < 3; j++) {
        MSH.MagLog[j][SenseCounter] = MSH.Mag[j];
        MSH.GyroLog[j][SenseCounter] = MSH.Gyro[j];
        MSH.AccelLog[j][SenseCounter] = MSH.Accel[j];
      }
      SenseCounter++;
    }
    else {
      IMUBufferFull = true;
    }
    SensUpdate = false;
  }

  void buildIMULog() {

    for (int i = (SenseNumb - 1); i >= 0; i--) {

      for (int j = 0; j < 3; j++) {
        if (i == 0) {
          MSH.MagLog[j][i] = MSH.Mag[j];
          MSH.GyroLog[j][i] = MSH.Gyro[j];
          MSH.AccelLog[j][i] = MSH.Accel[j];
        }
        if (i != 0) {
          MSH.MagLog[j][i] = MSH.MagLog[j][i - 1];
          MSH.GyroLog[j][i] = MSH.GyroLog[j][i - 1];
          MSH.AccelLog[j][i] = MSH.AccelLog[j][i - 1];
        }
      }//close for i

    }//close for j;

    // very helpful for debugging
    // Serial.print(MSH.GyroLog[0][0]); Serial.print(" "); Serial.print(MSH.GyroLog[1][0]); Serial.print(" "); Serial.println(MSH.GyroLog[2][0]) ;
    // Serial.print(MSH.GyroLog[0][1]); Serial.print(" "); Serial.print(MSH.GyroLog[1][1]); Serial.print(" "); Serial.println(MSH.GyroLog[2][1]) ;
    // Serial.print(MSH.GyroLog[0][2]); Serial.print(" "); Serial.print(MSH.GyroLog[1][2]); Serial.print(" "); Serial.println(MSH.GyroLog[2][2]) ;
    // Serial.print(MSH.GyroLog[0][3]); Serial.print(" "); Serial.print(MSH.GyroLog[1][3]); Serial.print(" "); Serial.println(MSH.GyroLog[2][3]) ;
    // Serial.print(MSH.GyroLog[0][4]); Serial.print(" "); Serial.print(MSH.GyroLog[1][4]); Serial.print(" "); Serial.println(MSH.GyroLog[2][4]) ;


  }

  void checkBatteryVoltage() {//TODO determine battery range Map to true values
    MSH.Battery = analogRead(A0);
    MSH.Battery = map(MSH.Battery, 0, 1023, 0, 4.2); //->check this mapping see if it can read the top
  }

  void checkTemp() {
    MSH.Temp = analogRead(A1); //calibrate top voltage of Tempsensor, until then values are off
    MSH.Temp = map(MSH.Temp,0,1023,-40,150); //PS: accuracy diminshes greatly past 125 Celsius, sensor limitation, no fix.
   
  }

  void currentSense() {// this value will need to be zeroed before practical use, also needs the meth
    float sensorValue = analogRead(9);              // Analog Signal Value
    float volt;                     // Voltage after conversion from analog signal
    float amp; 

    volt = (sensorValue * MSH.VOLTAGE_REF) / 1023;
    // Follow the equation given by the INA169 datasheet to
    // determine the current flowing through RS.
    // Reason for this is to obtain 50-100mV per INA169 specifications.
    // Is = (Vout x 1k) / (RS x RL)
    // 20 mA of approximate error error
    amp = volt / (29.75 * MSH.RS);
    
    MSH.SolarCurrent = amp;

    //test this please, and sign off!
  }

  void zeroCurrent() { // TODO debug me to make sure that my averaging is working right
    int accumulator = 0;
    for (int i = 0; i < 10; i++) {
      currentSense();
      accumulator += MSH.SolarCurrent;
    }
    MSH.CurrentZero = (accumulator / 10);
  }

  void lightSense() { //resistance goes down with more light -> 2.5V when light out at least with 0V (or close) when its dark
    // 
    MSH.lightSense = analogRead(A2);
    MSH.lightSense = map(MSH.lightSense, 0, 1023, 0, 3.3);
    //if value above a threshold, say 600, change boolean to positive sunlight. Or maybe just send back the number. r/ANd, add to requirements for mission sucess from CueSat perspective
  }

  void doorSense() {
    MSH.DoorButton = digitalRead(13); //done, its a simple HIGH or LOW-> confirm which means door is closed/open
  }

  void printDirectory(File dir, int numTabs) {
    while (true) {

      File entry =  dir.openNextFile();
      if (! entry) {
        // no more files
        break;
      }
      for (uint8_t i = 0; i < numTabs; i++) {
        Serial.print('\t');
      }
      Serial.print(entry.name());
      if (entry.isDirectory()) {
        Serial.println("/");
        printDirectory(entry, numTabs + 1);
      } else {
        // files have sizes, directories do not
        Serial.print("\t\t");
        Serial.println(entry.size(), DEC);
      }
      entry.close();
    }
  }

  void sensorInit() { //TODO Ensure that every sensor input is declared an input here // Relevant for all sensors: 3.3/1023 = .00322 -> discrete values //verfy pins are correct!
    
    SDInit();
    pinMode(A2, INPUT); //verify its photoresistor
    pinMode(13, INPUT); //verify its for TTL
    pinMode(9, INPUT); //verify its for button
    pinMode(A0, INPUT); //verify its for first inhibitor
    pinMode(A1, INPUT); //verify its for second inhibitor
    //missing third inhibitor
    //missing RBF
    //missing temp sensor //for the Temp sensor, the output range goes from 0V to 1.75V
    //missing voltage sensor
    //missing current sensor
    IMUInit();
  }



  void IMURange(int sensor, int sensitivity) { // TODO double check that I work
    switch (sensor) {
      case (1):
        //accelerometer
        switch (sensitivity) {
          case (1):
            lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
            break;
          case (2):
            lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
            break;
          case (3):
            lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
            break;
          case (4):
            lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
            break;
          case (5):
            lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
            break;
        }
        break;
      case (2):
        //magnetometer
        switch (sensitivity) {
          case (1):
            lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
            break;
          case (2):
            lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
            break;
          case (3):
            lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
            break;
          case (4):
            lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
            break;
        }
        break;
      case (3):
        //gyroscope
        switch (sensitivity) {
          case (1):
            lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
            break;
          case (2):
            lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
            break;
          case (3):
            lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
            break;
        }
        break;
    }
  }


  void InhibitorsCheck() { //read pin, and change value //please check PINs they are all wrong 
    MSH.Inhibitor_19a = digitalRead(12);
    MSH.Inhibitor_10b = digitalRead(14);
    MSH.Inhibitor_2 = digitalRead(15);

    if(MSH.Inhibitor_19a == false && MSH.Inhibitor_10b == false && MSH.Inhibitor_2 == false){
      MSH.FREEHUB = true; //CUBESAT is out of the P-POD! YAY!
    }
    
    
  }

 void GRENNLIGHT(){ //This function is used to specifically detect if a manual GREENLIGHT for mission operation has been given (sail release)
                    //This is normal function which may be overwritten to ignore rest of conditions
                    //ends up switching mode to MDEPLOY
  if(true){//conditions:
       //two keys have been engaged
       //detumble has been switched out - >manual command for this
       //HUB is FREE
    
    
    
    
    MDeploy();
  }
 }
