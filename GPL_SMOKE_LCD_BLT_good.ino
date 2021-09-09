#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

#define MQ_PIN                       (0)     //define which analog input channel you are going to use
#define RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
#define CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 

#define GAS_LPG                      (0)
#define GAS_CO                       (1)
#define GAS_SMOKE                    (2)
                                                     //normal operation
                                                     //which is derived from the chart in datasheet

float LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float Ro           =  10;                 //Ro is initialized to 10 kilo ohms





const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 6, d7 = 9;
int bluetoothTx = 2; // TX-O pin of bluetooth, Arduino D2
int bluetoothRx = 3; // RX-I pin of bluetooth, Arduino D3
int redLed = 8;
int greenLed = 7;
int buzzer = 13;
int sensorThresGas = 250;
int sensorThresSmoke = 300;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
SoftwareSerial blue(bluetoothTx, bluetoothRx);

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600); // Begin the serial monitor at 9600bps
  blue.begin(9600);  // Start bluetooth serial at 9600
  blue.print("$"); 

  delay(100); // Short delay, wait for the Mate to send back CMD
  blue.begin(9600); // Start bluetooth serial at 9600
  
  
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(9600);                               //UART setup, baudrate = 9600bps
  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                                  
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

}


void loop() 
{
  Serial.print("SMOKE:"); 
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
  Serial.print( "ppm" );


  Serial.print(" LPG:"); 
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
  Serial.print( "ppm" );
  Serial.print("\n");

//  blue.print("SMOKE VALUE: ");
//  blue.println(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
//  blue.print("LPG VALUE: ");
  //blue.println(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
  

      
  if(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE)>sensorThresSmoke)
  {
      digitalWrite(redLed,HIGH);
      digitalWrite(greenLed,LOW);
      tone(buzzer,1000,2000);
      delay(100);
     
      lcd.setCursor(0, 0);
      lcd.print("SMOKE:");
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
      lcd.setCursor(0, 1);
      lcd.print("SMOKE ALARM!!!");
      blue.println("SMOKE VALUE: ");
      blue.println(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
      blue.println("SMOKE ALARM");
     
      delay(500);
   }

   if(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG)>sensorThresGas)
   {
      digitalWrite(redLed,HIGH);
      digitalWrite(greenLed,LOW);
      tone(buzzer,1000,2000);
      delay(100);

      lcd.setCursor(0, 0);
      lcd.print(" LPG:");
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
      lcd.print(" ppm ");
      lcd.setCursor(0, 1);
      lcd.print("LPG ALARM!!!");
      blue.print("LPG VALUE: ");
      blue.println(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
      blue.println("LPG ALARM");
      delay(500);
   }
   else if(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE)>sensorThresSmoke && MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG)>sensorThresGas)
   {
      digitalWrite(redLed,HIGH);
      digitalWrite(greenLed,LOW);
      tone(buzzer,1000,2000);
      delay(100);

      lcd.setCursor(0, 0);
      lcd.print("SMOKE:");
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
      lcd.print(" LPG:");
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
      lcd.print(" ppm ");
      lcd.setCursor(0, 1);
      lcd.print("SMOKE & LPG ALARM!!");
      blue.println("SMOKE VALUE: ");
      blue.println(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
      blue.print("LPG VALUE: ");
      blue.println(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
      blue.println("SMOKE & LPG ALARM!!");
      delay(500);
      
   }
   else
   {
      digitalWrite(redLed,LOW);
      digitalWrite(greenLed,HIGH);
      noTone(buzzer);
      delay(100);
   } 
}

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   
  val = val/RO_CLEAN_AIR_FACTOR;                      
  return val; 
}


float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
       return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } 
  else if ( gas_id == GAS_CO ) {
       return MQGetPercentage(rs_ro_ratio,COCurve);
  }
  else if ( gas_id == GAS_SMOKE ) {
 
      return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }  

  return 0;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
