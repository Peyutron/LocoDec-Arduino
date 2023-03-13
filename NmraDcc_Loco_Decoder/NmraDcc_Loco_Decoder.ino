/* Decodificador para locomotoras H0 con Arduino nano y el driver L293
 * Libreria NmraDcc.h 
 * El número por defecto de la locomotora es el 3 
 * La salida PWM en el Arduino Nano es la 9
 * Los pines de salida del L293 en el Arduino Nano son el 10 y 11 
 * Las luces van conectadas a los pines 4 y 5 del Arduino Nano
 * CVś basicas: CV2 = Tension de arranque; CV3  = Tasa de aceleracion; CV4  = Tasa de Frenado; CV5  = Tension de velocidad maxima 
*/
#include <NmraDcc.h>

#define This_Decoder_Address 3 //Cambia este numero para cambiar la dirección de la locomotora
const int FunctionPin0 = 4;
const int FunctionPin1 = 5;
const int FunctionPin2 = 6;
const int FunctionPin3 = 7;
const int FunctionPin4 = 8;

const int PWM_MAX = 254;
const int left = 10;        //Pin de salida L293
const int right = 11;       //Pin de salida L293
const int velPwm = 9;       //Pin de salida Enable L293
const int lucesDelanteras = 4; //pin de salia luces delanteras
const int lucesTraseras = 5;   //pin de salia luces traseras
int currentSpeed = 0;
int rateSteps;
int acSpeed;
int locSpeed = 0;
int rateSpeed = 0;
int dirState = 0;     //Variable cambio de direccion
int dirFlag1 = 0;
int dirFlag2 = 0;
int steps;            //Pasos (14, 28, 128)
int maniobras = 0;
int Luces = 0;        //Variable encendido de luces
//-------------CV de configuracion
int CV2 = 100;        //Tension de arranque
int CV3 = 4;          //Tasa de Aceleración (TA * intervalAcc)
int CV4 = 4;          //Tasa de Frenado (TF * intervalDec)
int CV5 = 254;        //Tension velocidad Maxima
//-------------
int topSpeed =  CV5;
long previousDebug = 0;
long previousAcc = 0;
long previousDec = 0;
long intervalDebug = (500);
long intervalAcc = (25 * CV3);
long intervalDec = (25 * CV4);
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
  // The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, This_Decoder_Address},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, This_Decoder_Address},

  // ONLY uncomment 1 CV_29_CONFIG line below as approprate
  //  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
  //  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps
};

NmraDcc  Dcc ;
DCC_MSG  Packet ;

uint8_t FactoryDefaultCVIndex = 0;

// Uncomment this line below to force resetting the CVs back to Factory Defaults
// FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
};

// Uncomment the #define below to print all Speed Packets
#define NOTIFY_DCC_SPEED
#ifdef  NOTIFY_DCC_SPEED
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  // Serial.print("notifyDccSpeed: Addr: ");
  // Serial.print(Addr, DEC);
  // Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
  // Serial.print(" Speed: ");
  // Serial.print(Speed, DEC);
  //  locSpeed = Speed;
  acSpeed = Speed;
  // Serial.print(" Steps: ");
  int SpeedStep = (SpeedSteps - 1);
  steps = SpeedStep;
  //  Serial.print(SpeedStep, DEC);
  //  Serial.print(" Dir: ");
  //  Serial.println( (Dir == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  dirState = Dir;
};
#endif

// Uncomment the #define below to print all Function Packets
#define NOTIFY_DCC_FUNC
//#ifdef  NOTIFY_DCC_FUNC
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  // Serial.print("notifyDccFunc: Addr: ");
  //erial.print(Addr, DEC);
  //Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  //Serial.print("  Function Group: ");
  //Serial.print(FuncGrp, DEC);
  switch ( FuncGrp )
  {
#ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE
    case FN_0:
      //  Serial.print(" FN0: ");
      //  Serial.println((FuncState & FN_BIT_00) ? "1  " : "0  ");
      break;
#endif
    case FN_0_4:
      exec_function( 0, (FuncState & FN_BIT_00) >> 4 );
      exec_function( 1, (FuncState & FN_BIT_01));
      exec_function( 2, (FuncState & FN_BIT_02) >> 1);
      exec_function( 3, (FuncState & FN_BIT_03) >> 2 );
      exec_function( 4, (FuncState & FN_BIT_04) >> 3 );
      if (Dcc.getCV(CV_29_CONFIG) & CV29_F0_LOCATION) // Only process Function 0 in this packet if we're not in Speed Step 14 Mode
      {
        //  Serial.print(" FN 0: ");
        //  Serial.print((FuncState & FN_BIT_00) ? "1  " : "0  ");
        // delay (100);
      }
      break;

    case FN_5_8:
      exec_function( 5, (FuncState & FN_BIT_05));
      exec_function( 6, (FuncState & FN_BIT_06) >> 1 );
      exec_function( 7, (FuncState & FN_BIT_07) >> 2 );
      exec_function( 8, (FuncState & FN_BIT_08) >> 3 );
      break;

    case FN_9_12:
      exec_function( 9, (FuncState & FN_BIT_09));
      exec_function( 10, (FuncState & FN_BIT_10) >> 1 );
      exec_function( 11, (FuncState & FN_BIT_11) >> 2 );
      exec_function( 12, (FuncState & FN_BIT_12) >> 3 );
      break;
  }
}
void exec_function (int f_index, int FuncState)  
{
  switch (f_index) {
    case 0:
      if (FuncState == 1) {
        Luces = 1;
      }
      else {
        Luces = 0;
      }
    case 1:
      break;
    case 2:
      break;
    case 3:
      if (FuncState == 1) {
        maniobras = 1;
      }
      else {
        maniobras = 0;
      }
      break;
    default:
      break;
  }
}

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read

const int DccAckPin = 15 ;

void notifyCVAck(void)
{
  Serial.println("notifyCVAck") ;

  digitalWrite( DccAckPin, HIGH );
  delay( 8 );
  digitalWrite( DccAckPin, LOW );
}

void setup()
{
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  pinMode(velPwm, OUTPUT);
  pinMode (lucesDelanteras, OUTPUT);
  pinMode (lucesTraseras, OUTPUT);
  pinMode (6, OUTPUT);
  pinMode (7, OUTPUT);
  pinMode (8, OUTPUT);
  pinMode (13, OUTPUT);

  Serial.begin(115200);
  Serial.println("NMRA Dcc Multifunction Decoder Demo 1");

  // Configure the DCC CV Programing ACK pin for an output
  pinMode( DccAckPin, OUTPUT );
  digitalWrite( DccAckPin, LOW );

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(0, 2, 0);

  // Call the main DCC Init function to enable the DCC Receiver
  //Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY, 0 );

  // Uncomment to force CV Reset to Factory Defaults
  notifyCVResetFactoryDefault();
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if ( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }

  if (dirState == 0) {
    if (dirFlag1 == 1) {
      int locSpeedup = locSpeed;
      for (int locSpeed = locSpeedup; locSpeed > CV2; locSpeed--) {
        analogWrite (velPwm, locSpeed);
        delay(intervalDec);
      }
      dirState0();

      for (int locSpeed = CV2; locSpeed < locSpeedup; locSpeed++) {
        analogWrite (velPwm, locSpeed);
        delay(intervalAcc);
      }
      dirFlag1 = 0;
    }
    dirFlag2 = 1;
    dirState0();
  }

if (dirState == 1) {
    if (dirFlag2 == 1) {
      int locSpeedup = locSpeed;
      for (int locSpeed = locSpeedup; locSpeed > CV2; locSpeed--) {
        analogWrite (velPwm, locSpeed);
        delay(intervalDec);
      }
      dirState1();

      for (int locSpeed = CV2; locSpeed < locSpeedup; locSpeed++) {
        analogWrite (velPwm, locSpeed);
        delay(intervalAcc);
      }
      dirFlag2 = 0;
    }
    dirFlag1 = 1;
    dirState1();
  }
  if (currentSpeed <= 1) {
    locSpeed = 0;
    digitalWrite (left, LOW);
    digitalWrite (right, LOW);
  }
  else {
    rateSpeed = ((PWM_MAX - CV2) / steps);
  }
  if (currentSpeed != acSpeed ) { //si currentSteps es diferente de steps
    if (currentSpeed < acSpeed) {
      Acc();
    }
    if (currentSpeed > acSpeed) {
      Dec();
    }
  }
  analogWrite (velPwm, locSpeed);
  Debugger();
}
void Acc() 
{
  unsigned long currentAccMillis = millis();
  if (currentAccMillis - previousAcc > intervalAcc ) {
    previousAcc = currentAccMillis;
    locSpeed = (CV2 + (currentSpeed * rateSpeed));

    if (locSpeed >= CV5) {  //si velocidad de locomotora mayor que
      locSpeed = CV5;
    }
    currentSpeed = (currentSpeed + 1);
    if (currentSpeed >= acSpeed) {
      currentSpeed = acSpeed;
    }
  }
  return;
}
void Dec() 
{
  unsigned long currentDecMillis = millis();
  if (currentDecMillis - previousDec > intervalDec) {
    previousDec = currentDecMillis;
    locSpeed = (CV2 + (currentSpeed * rateSpeed));
    currentSpeed = (currentSpeed - 1);
    if (currentSpeed <= acSpeed) {
      currentSpeed = acSpeed;
   }
    return;
  }
}
void dirState0() 
{
  digitalWrite (left, HIGH);
  digitalWrite (right, LOW);

  if (Luces == 1) {
    digitalWrite (lucesDelanteras, HIGH);
    digitalWrite (lucesTraseras, LOW);
  }
  else {
    digitalWrite (lucesDelanteras, LOW);
    digitalWrite (lucesTraseras, LOW);
  }
  return;
}
void dirState1() 
{
  digitalWrite (right, HIGH);
  digitalWrite (left, LOW);

  if (Luces == 1) {
    digitalWrite (lucesDelanteras, LOW);
    digitalWrite (lucesTraseras, HIGH);
  }
  else {
    digitalWrite (lucesDelanteras, LOW);
    digitalWrite (lucesTraseras, LOW);
  }
  return;
}
void Debugger() 
{
  unsigned long currentDebugMillis = millis();
  if (currentDebugMillis - previousDebug > intervalDebug) {
    previousDebug = currentDebugMillis;
    Serial.print(" Decoder N: ");
    Serial.print(This_Decoder_Address);
    Serial.print(" Velocidad PWM : ");
    Serial.print(locSpeed);
    Serial.print(" Pasos : ");
    Serial.print(steps, DEC);
    Serial.print(" Dir: ");
    Serial.println( (dirState == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  }
  return;
}
