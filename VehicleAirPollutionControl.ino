



/************************Hardware Related Macros************************************/
#include <LiquidCrystal.h>
//#include <DHT.h>
#include "DHT.h"
#include <TinyGPS.h>
#include <SoftwareSerial.h>

TinyGPS gps;
//const int calibrationLed = 13;                      //when the calibration start , LED pin 13 will light up , off when finish calibrating
const int MQ_PIN=A0;                                //define which analog input channel you are going to use
int RL_VALUE=5; //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
 
   SoftwareSerial mySerial(9, 10);                                                 //which is derived from the chart in datasheet
   SoftwareSerial myInput(1, 13);
/***********************Software Related Macros************************************/
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_INTERVAL=25;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in 
#define DHTPIN 8
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);                                                    //normal operation
 
/**********************Application Related Macros**********************************/
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    
 
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

LiquidCrystal lcd(12, 11, 5, 4, 3 ,2 ); //LCD pins





  // put your main code here, to run repeatedly:
  
   
   //digitalWrite(7,LOW);
   //digitalWrite(6,HIGH);
   //delay(3000);

void setup()
{ 
   pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  mySerial.begin(9600);
  myInput.begin(9600);
  Serial.begin(9600);
   myInput.println(F("DHTxx test!"));

  dht.begin();
  //dht.begin();
  delay(100);
  
  lcd.begin(40,2);
//  pinMode(calibrationLed,OUTPUT);
  //digitalWrite(calibrationLed,HIGH);
  lcd.print("Calibrating...");                        //LCD display

  
  Ro = MQCalibration(MQ_PIN);                         //Calibrating the sensor. Please make sure the sensor is in clean air         
 // digitalWrite(calibrationLed,LOW);              
  
  lcd.print("done!");                                 //LCD display
  lcd.setCursor(0,1);
  lcd.print("Ro= ");
  lcd.print(Ro);
  lcd.print("kohm");
  delay(3000);
}
 
void loop()
{ 
  int sensorValue = analogRead(A0);
  myInput.println(sensorValue);
     delay(2000);


   float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    myInput.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  myInput.print(F("Humidity: "));
  myInput.print(h);
  myInput.print(F("%  Temperature: "));
  myInput.print(t);
  myInput.print(F("°C "));
  myInput.print(f);
  myInput.print(F("°F  Heat index: "));
  myInput.print(hic);
  myInput.print(F("°C "));
  myInput.print(hif);
  myInput.println(F("°F"));


  
//float voltage = sensorValue * (5.0 / 1023.0);
   //print out the value you read:
  //myInput.println(voltage);
   unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      //Serial.print(c);
      gps.encode(c);
          
    }
  }
   
  long iPPM_LPG = 0;
  long iPPM_CO = 0;
  long iPPM_Smoke = 0;
 

  iPPM_LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  iPPM_CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
  iPPM_Smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
  if(iPPM_CO <= 150 && h <= '80.00%'){
    digitalWrite(7,HIGH);
   digitalWrite(6,LOW);
   delay(1000); 
   lcd.clear();   
   lcd.setCursor( 0 , 0 );
   lcd.print("Concentration of gas ");

   
   lcd.setCursor( 0, 1);
   lcd.print("CO: ");
   lcd.print(iPPM_CO);
   lcd.print(" ppm");
   lcd.print("   LPG:");
   lcd.print(iPPM_LPG);
   lcd.print(" ppm");
   lcd.print("   Smoke:");  
   lcd.print(iPPM_Smoke);
   lcd.print(" ppm");
  }

  else{

     lcd.clear();   
   lcd.setCursor( 0 , 0 );
   lcd.print("WARNING DANGER!!!!");

   
   lcd.setCursor( 0, 1);
   lcd.print("CO: ");
   lcd.print(iPPM_CO);
   lcd.print(" ppm");
   lcd.print("   LPG:");
   lcd.print(iPPM_LPG);
   lcd.print(" ppm");
   lcd.print("   Smoke:");  
   lcd.print(iPPM_Smoke);
   lcd.print(" ppm");  
    sendmessage(iPPM_CO);
    
  }   

   delay(1000);
  
}
void sendmessage(long ppm)
{

 
 
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);  
 


          mySerial.println("AT+CMGF=1");
          delay(1000);
          mySerial.println("AT+CMGS=\"+917980590630\"\r");

          delay(1000);

          digitalWrite(7,LOW);
          digitalWrite(6,LOW);

          delay(1000);

          mySerial.println("Warning!!");
          mySerial.println("You need to service your car asap");
           mySerial.println("Your Location is here");
         

          delay(1000);
          
           mySerial.print("Latitude = ");
          mySerial.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
          mySerial.print(" Longitude = ");
          mySerial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

         // delay(1000);

           delay(2000);
            float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
   mySerial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  mySerial.print(F("Humidity: "));
  mySerial.print(h);
  mySerial.print(F("%  Temperature: "));
  mySerial.print(t);
  mySerial.print(F("°C "));
  mySerial.print(f);
  mySerial.print(F("°F  Heat index: "));
  mySerial.print(hic);
  mySerial.print(F("°C "));
  mySerial.print(hif);
  mySerial.println(F("°F"));

   delay(1000);
          mySerial.println((char)26);

          delay(1000);


  
  }
 
/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
 
    return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {  
    float f;
    //take multiple samples
    f =  MQResistanceCalculation(analogRead(mq_pin));
     lcd.clear();
   lcd.setCursor( 0 , 0 );
   lcd.print(f);
     val += f;
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 

}
 
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
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
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
long MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
