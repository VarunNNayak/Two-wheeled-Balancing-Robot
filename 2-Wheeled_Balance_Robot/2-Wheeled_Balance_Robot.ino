//2-Wheeled Balance Robot
#include <ChibiOS_ARM.h>

// TODO replace this period
const int TODO_PERIOD_MS = 20;

const int BALANCE_WORK_MICROS = 1800;

const int IMU_WRITE_OP_MICROS = 330;
const int IMU_READ_OP_MICROS = 820;

const int UART_BYTE_MICROS = 95;

const int BLUETOOTH_WORK_MICROS = 1000;

const int LOGGING_WORK_MICROS = 60;
const int SD_WRITING_WORK_NORMAL_MICROS = 3000;
const int SD_WRITING_WORK_LONG_MICROS = 25000;

const int NUM_SD_BUFFERS = 2;

// ===================================================================
// types
// ===================================================================

enum WhichMotor {
  MOTOR_LEFT,
  MOTOR_RIGHT
};

struct SdBuffer {
  char data[512];
};


// ===================================================================
// global variables and synchronization objects
// ===================================================================

bool gRunning = true;

SdBuffer gSdBuffers[NUM_SD_BUFFERS];
int gLoggingBuffer = 0; // which not-full buffer is in use for logging


// ===================================================================
// helper functinons
// ===================================================================

/**
 * simulate a certain number of microseconds of computational work
 */
void simulateWork(unsigned long durMicros) {
  for (unsigned long i = 0; i < (durMicros*4 + durMicros*0); i++) {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }
}


// ===================================================================
// task helper functions...
// ===================================================================

/**
 * simulate the balance control algorithm
 * Note: not synchronized, requires external synchronization if shared data is used
 */
void applyBalanceControl(int left, int right) {
  if (right != (left * 2) - 1) {
    Serial.println("\n***** ERROR: check balance inputs  mismatch => race condition");
  }

  simulateWork(BALANCE_WORK_MICROS);
}

// simulate the write command to Inertial Management Unit (IMU)
void imuWriteCommand() {
  simulateWork(IMU_WRITE_OP_MICROS);
}

// simulate the read command from Inertial Management Unit (IMU)
void imuReadCommand() {
  simulateWork(IMU_READ_OP_MICROS);
}

// simulate writing a single byte of data via the UART to the motor
void writeByteToUart(enum WhichMotor motor) {
  simulateWork(UART_BYTE_MICROS);
}

/**
 * simulate logging an entry into a (not yet full) buffer for later writing to the SD card
 * @param loggingBuffer is the buffer into which data should be logged
 * @param entry -- the logging entry number (0-31) within the buffer. (The buffer is 512
 *                 bytes in total and each entry consumes 16 bytes => 32 entries in total)
 * 
 * Note: this function is unsynchronized so if any data is shared between
 * threads it will need to be synchronised externally
 */
void logEntryToBuffer(SdBuffer* loggingBuffer, int entry) {
  // Note: the following 4 lines slightly affect the simulated timing of this task
  // but we'll ignore that
  int offset = entry * 16;
  int value = random(256);
  for (int i=0; i<16; i++)
    loggingBuffer->data[offset+i] = value;
    
  simulateWork(LOGGING_WORK_MICROS);
}

/** 
 * simulate receiving and processing a command over the Bluetooth link 
 * @param leftSetpointOut -- output param - pointer to left setpoint variable to be set
 * @param rightSetpointOut -- output param - pointer to right setpoint variable to be set
 * Note: not synchronized, requires external synchronization if shared data is accessed
 */
void receiveAndProcessBluetoothCommand(int *leftSetpointOut, int *rightSetpointOut) {
    simulateWork(BLUETOOTH_WORK_MICROS);
    int newSetpoint = random(100);
    *leftSetpointOut = newSetpoint;
    *rightSetpointOut = (newSetpoint * 2) - 1;
}

/**
 * simulate writing a full buffer to the SD card
 * @param writingBuffer -- pointer to the full buffer to write to the SD card
 * @param isLongDuration -- true means this is a long duration write, false means
 *                          it is the normal duration write
 * Note: not syncronized, requires external synchronization if shared data is accessed
 */
void writeBufferToSdCard(SdBuffer* writingBuffer, bool isLongDuration) {
  if (isLongDuration)
    simulateWork(SD_WRITING_WORK_NORMAL_MICROS);
  else
    simulateWork(SD_WRITING_WORK_LONG_MICROS);
}

// ===================================================================
// task functions...
// ===================================================================
//global variables
bool gbluetooth=false;
bool gBuffer=false;


// ===================================================================
//Mutex and conditional variable declaration
MUTEX_DECL(BluetoothTask);
MUTEX_DECL(WaitForSignalMutex);
CONDVAR_DECL(WaitForSignalCond);


// ===================================================================
//Busy Task 
void SimulateWorkmicros(unsigned long microsec){
    float LoopRepeat=microsec*(54.68/100); //Estimating the number of loop repeat 
   for(float i=1;i<LoopRepeat;i++){
    volatile float Rand_Number=i;
    i++;
}
}

//------------------------------------------------------------
//Critical Section
void SimulateCriticalSection(unsigned long microsec){
  SimulateWorkmicros(microsec);
}
// TODO - add your task functions here
// The task functions should then call the task helpers above to
// perform the [simulated] operations described in the Assignment text.

// ===================================================================
// thread functions...
// ===================================================================
//Thread 1
static THD_WORKING_AREA(waThreadOne,50);//Declaring Thread Working Area

static THD_FUNCTION(ControlFunctionThread,arg){
  int i=0; 
  while(true){ 
  Serial.print("Start of the loop");
  Serial.println();
  
    //-------------------- Inertial Sensors Task ----------------------------
    unsigned long StartTime=micros();
   
    SimulateCriticalSection(1150);//add

    unsigned long StopTime=micros()-StartTime;
    Serial.print("Inertial Sensors Task Period = ");
    Serial.println(StopTime);

    //------------------- Balance Control Feedback Task -------------
    StartTime=micros(); 
    chMtxLock(&BluetoothTask);
    bool data=gbluetooth;
    gbluetooth=true;
    chMtxUnlock(&BluetoothTask); 
    SimulateCriticalSection(1800);//add
    StopTime=micros()-StartTime;
    Serial.print("Balance Control Feedback Task Period = ");
    Serial.println(StopTime);

    //---------------------------- UART Task ----------------------------
    StartTime=micros();   
    SimulateCriticalSection(380);//add
    StopTime=micros()-StartTime;
    Serial.print("UART Task Period = ");
    Serial.println(StopTime);
    
    //---------------------------- LoggingTask ----------------------------
    int N = 0;
    for (N = 0; N < 32; N++){
    StartTime=micros();    
    SimulateCriticalSection(60);//add
    if(++i==32){   
      chMtxLock(&WaitForSignalMutex);
      gBuffer=true;
      //chCondSignal(&WaitForSignalCond);//buffer is full signals SDcardThread to start WaitForSignaling
      chMtxUnlock(&WaitForSignalMutex);
      i=0; 
    }
    StopTime=micros()-StartTime;
    Serial.print("SD Entry period = ");
    Serial.println(StopTime);
    
    Serial.print("sN(Signalled/communicated buffer number) = ");
    Serial.println(N);
      
   Serial.print("End of the loop");
  Serial.println();
      }

  }
}


// TODO - add your thread functions here
// One thread is added already as an example - rename it as appropriate
//Thread 2
static THD_WORKING_AREA(waThreadTwo, 32);

static THD_FUNCTION(BluetoothThread, arg) {
  static int charCounter = 0;
  systime_t deadline = chVTGetSystemTimeX(); // initialise deadline for start of first loop
  
  while (gRunning) {
     //chCondSignal(&WaitForSignalCond);
//    deadline += TIME_MS2I(TODO_PERIOD_MS);
    deadline += MS2ST(19); // set deadline to start of next loop -- TODO choose correct period

    unsigned long StartTime=micros();
    
    SimulateCriticalSection(16000);//add

    chMtxLock(&BluetoothTask);
    gbluetooth=true;
    chMtxUnlock(&BluetoothTask);

    unsigned long StopTime=micros()-StartTime;
    Serial.print("Bluetooth Task Period = ");
    Serial.println(StopTime);

    chThdSleepUntil(deadline); // en
    
  }
}

//Thread 3
static THD_WORKING_AREA(waThreadThree,50);//Declaring Thread Working Area

static THD_FUNCTION(SDcardThread,arg){
  int N = 0;
  while(true){
    for (N = 0; N < 32; N++){
    unsigned long StartTime=micros();
    
    chMtxLock(&WaitForSignalMutex);
    while(gBuffer==false)
    chCondSignal(&WaitForSignalCond);
    gBuffer=false;
    chCondSignal(&WaitForSignalCond);
    chMtxUnlock(&WaitForSignalMutex);
    SimulateCriticalSection(25000);//add
    
    unsigned long StopTime=micros()-StartTime;
    Serial.print("SD Copying Task Period = ");
    Serial.println(StopTime);

    
    Serial.print("rN(Received/communicated buffer number) = ");
    Serial.println(N);
      }
 }
}


// main thread -- all it does is launch the other threads and then exit

void mainThread() {
  chThdCreateStatic(waThreadOne,sizeof(waThreadOne),NORMALPRIO,ControlFunctionThread,NULL);//medium priority
  chThdCreateStatic(waThreadTwo,sizeof(waThreadTwo),NORMALPRIO+1,BluetoothThread,NULL);//high priority
  chThdCreateStatic(waThreadThree,sizeof(waThreadThree),NORMALPRIO,SDcardThread,NULL);//low priority
}

// ===================================================================
// setup and loop
// ===================================================================

void setup() {
  // put your setup code here, to run once:

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  // setting the serial port to 115200 ensures that we can print approx
  // 11250 chars/sec => approx one char every 87 microsecs. Therefore, as long
  // as we don't print more than one char every 100 microsecs or so, the call
  // to Serial.print a single char only takes about 2 microsecs to execute and 
  // has negligible effect on the timing of our tasks (in the context of this 
  // assignment)  
  Serial.begin(115200);
  Serial.println("\n\n============= A2_BalanceBot starting ==============\n");

  // the main thread uses the default stack of the program so no working
  // area etc is allocated - it just needs a normal function
  chBegin(mainThread);
  while(1);  // this code should never execute if ChibiOS starts correctly
}

void loop() {
  // this function is unused with ChibiOS
}
