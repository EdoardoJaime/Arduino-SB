//MOTORS//
int driverDIR11 = 6; //dir
int driverPUL11 = 8; //step
int driverDIR23 = 10; //dir
int driverPUL23 = 12; //step
float steps_per_rev_11 = 400.0 * 13.73356401;
float steps_per_rev_23 = 400.0;
int Nema_23_steps_taken=0;
int Nema_11_steps_taken=0;
bool OpenBin = false;

//SERVO//
#include <Servo.h>
Servo myservo;  // create servo object to control a servo

//PIR//
int pirPin = 2;  // PIR Out pin 
int pirStat = 0; // PIR status
bool object_fallen_pir = false;

//LED//
///Positive wire to be connected to digital pin ///
int LEDPIN = 11;

//IR CALIBRATION//
int SENSORPIN = 13;
int sensorState = 0;
int lastState = 0; 
bool NEMA11_cal = false;
bool total_calibration = false;
int number_of_sorting_rounds = 0;

//ULTRASONIC//
int pingPin = 3; // trigger pin in the US sensor
int pongPin = 4; // data pin in the US sensor
//#define NR 10 // number of repetition of readings under each test condition
long duration; // microseconds
float tempAir; // Celsius
float soundSpeed; // m/s
int AT[5]={1,10,50,100,200}; // time interval between consecutive readings, milliseconds
float distance_top_casing; // array with the readings of the repeated measurements, cm
float fullness;
float height = 36.6;
float durCalc(int pinI, int pinO)
{
  // The PING is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(pinI, LOW);
  delayMicroseconds(2);
  digitalWrite(pinI, HIGH);
  delayMicroseconds(12);
  digitalWrite(pinI, LOW);
  //delay(10);
  // The pongPin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  return(pulseIn(pinO, HIGH));
}


//GENERAL//
int bin_type;
String data;
bool bin_operating = false;
String Rpi_input;
bool input_communication_send = false;



//TIME//
unsigned long material_StartTime;
unsigned long material_CurrentTime;
unsigned long material_ElapsedTime;

unsigned long pir_StartTime;
unsigned long pir_CurrentTime;
unsigned long pir_ElapsedTime;


//FUNCTIONS//
  // Function to rotate the NEMA 23 a certain angle (degrees) in a certain time (s)
void rotate_motor_23(int rotate_angle, float time_rotate)
  { 
      // Obtaining the steps to do the iterations for a given steps/rev
    float steps = (rotate_angle * steps_per_rev_23)/360.0;

      // Changing the time to millis
    time_rotate = time_rotate * 1000;

      // While loop for given steps
    while (Nema_23_steps_taken<steps) 
    {
    
       Nema_23_steps_taken = Nema_23_steps_taken+1;
       digitalWrite(driverDIR23,1); // turn to the right
       digitalWrite(driverPUL23,HIGH);
       delayMicroseconds(time_rotate); // 1ms
       digitalWrite(driverPUL23,LOW);
       delayMicroseconds(time_rotate); // 1ms      
    }
      // Restart the count for next time
    Nema_23_steps_taken = 0;
  }

   // Function to rotate the NEMA 23 a amount of steps in a certain time (s)
void rotate_motor_23_steps(int motor_steps, float time_rotate)
  { 

      // Changing the time to millis
    time_rotate = time_rotate * 1000;

      // While loop for given steps
    while (Nema_23_steps_taken<motor_steps) 
    {
     
       Nema_23_steps_taken = Nema_23_steps_taken+1;
       digitalWrite(driverDIR23,1); // turn to the right
       digitalWrite(driverPUL23,HIGH);
       delayMicroseconds(time_rotate); // 1ms
       digitalWrite(driverPUL23,LOW);
       delayMicroseconds(time_rotate); // 1ms 
     
    }
      // Restart the count for next time
    Nema_23_steps_taken = 0;
  }

  // Function to rotate the NEMA 11 a certain angle (degrees) in a certain time (s), stop a certain time (s) and then rotate back to original position
void rotate_motor_11(int rotate_angle, float time_rotate, float time_stop, int polarity)
  { 
      // Obtaining the steps to do the iterations for a given steps/rev
    float steps = (rotate_angle * steps_per_rev_11)/360.0;

      // Changing the time to millis
    time_rotate = time_rotate *1000;
    time_stop= time_stop * 1000;

      // While loop for given steps
    while (Nema_11_steps_taken<steps) 
    {
     
       Nema_11_steps_taken = Nema_11_steps_taken+1;
       digitalWrite(driverDIR11,polarity); 
       digitalWrite(driverPUL11,HIGH);
       delayMicroseconds(time_rotate); 
       digitalWrite(driverPUL11,LOW);   
     
    }
    delay(time_stop);
    digitalWrite(driverPUL11,LOW);
    if (polarity == 1) {
      polarity = 0;
    }
    else {
      polarity = 1;
    }
    while (steps < Nema_11_steps_taken && Nema_11_steps_taken < 2.0 * steps)
    {
     
       Nema_11_steps_taken = Nema_11_steps_taken + 1;
       digitalWrite(driverDIR11,polarity); // turn to the left
       digitalWrite(driverPUL11,HIGH);
       delayMicroseconds(time_rotate);
       digitalWrite(driverPUL11,LOW);
     
    }
      // Restart the count for next time
    Nema_11_steps_taken = 0;
  }
  
  // Function to rotate the NEMA 11 a amount of steps in a certain time (s)
void rotate_motor_11_steps(int motor_steps, float time_rotate)
  { 

      // Changing the time to millis
    time_rotate = time_rotate * 500;

      // While loop for given steps
    while (Nema_11_steps_taken<motor_steps) 
    {
     
       Nema_11_steps_taken = Nema_11_steps_taken+1;
       digitalWrite(driverDIR11,1); // turn to the right
       digitalWrite(driverPUL11,HIGH);
       delayMicroseconds(time_rotate); // 1ms
       digitalWrite(driverPUL11,LOW);
       delayMicroseconds(time_rotate); // 1ms 
     
    }
      // Restart the count for next time
    Nema_11_steps_taken = 0;
  }

// Functin to rotate NEMA 11 only in one direction (particularly useful for bin 4). Inputs are angle, time and polarity
void rotate_motor_11_bin4(int rotate_angle, float time_rotate, int polarity)
  { 
      // Obtaining the steps to do the iterations for a given steps/rev
    float steps = (rotate_angle * steps_per_rev_11)/360.0;

      // Changing the time to millis
    time_rotate = time_rotate * 1000;

      // While loop for given steps
    while (Nema_11_steps_taken<steps) 
    {
     Nema_11_steps_taken = Nema_11_steps_taken+1;
     digitalWrite(driverDIR11,polarity); 
     digitalWrite(driverPUL11,HIGH);
     delayMicroseconds(time_rotate); // 1ms
     digitalWrite(driverPUL11,LOW);
     delayMicroseconds(time_rotate); // 1ms 
    }
      // Restart the count for next time
    Nema_11_steps_taken = 0;
  }


  // Function to execute all the bin_types
void bin_type_action(String bin_type)
  {
    digitalWrite(LEDPIN, LOW); 
    if (bin_type == "General")
         //if (bin_type == 1)
        {
         rotate_motor_23(0,5);
         rotate_motor_11(180,2.0,2.0,0);  
         delay(2000);             
        }
      
     if (bin_type == "Dry")
        //if (bin_type == 2)
        {
          //rotate_motor_11(100,1.5,1.5,0);
          rotate_motor_11_bin4(100,1.5,0);
          rotate_motor_23(90,7);
          rotate_motor_11_bin4(100,1.5,1);  
          delay(2000);             
        }
      
     if (bin_type == "Organic")
        //if (bin_type == 3)
        {
          rotate_motor_23(180,7);
          rotate_motor_11(0,1.5,1.5,0); 
          delay(2000);              
        }
    
     if (bin_type == "Paper")
        //if (bin_type == 4)
        {
          rotate_motor_11_bin4(90,1.5,1);
          rotate_motor_23(270,10);
          rotate_motor_11_bin4(90,1.5,0); 
          delay(2000);             
        }
     else
      {
        
      }
  }

void setup() {

  pinMode (driverDIR11,OUTPUT) ;
  pinMode (driverPUL11,OUTPUT) ;
  pinMode (driverDIR23,OUTPUT) ;
  pinMode (driverPUL23,OUTPUT) ;
  pinMode (pirPin, INPUT);
  pinMode(pingPin, OUTPUT);
  pinMode(pongPin, INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(SENSORPIN, INPUT);     
  digitalWrite(SENSORPIN, LOW); // HIGH means the beam is unbroken
  myservo.attach(9,600,2300);  // (pin, min, max)
  myservo.write(0);
  Serial.begin(9600);
  tempAir = 20;
  soundSpeed = 331.3+0.06*tempAir; // m/s
  soundSpeed = soundSpeed/10000.; // cm per microsec

}


void loop() 
{
    
   
  duration = durCalc(pingPin, pongPin); // measures the sound travelling distance (microseconds)
  distance_top_casing = duration*soundSpeed/2; // converts the time into a distance (cm)

 //Rpi_input = Serial.readStringUntil('\n');

  // If the distance is smaller than 10 & the bin is not operating -> Open the bin
  //if (distance_top_casing < 10 && bin_operating == false || Rpi_input == "Open Bin" && bin_operating == false) 
  if (distance_top_casing < 30 && bin_operating == false) 
  {
    myservo.write(90);
    delay(1000);
    pir_StartTime = millis();
    OpenBin = true;
  }

    // If the Bin has opened wait for the PIR to detect if an object has fallen
  if (OpenBin == true)
  {
    pir_CurrentTime = millis();
    pir_ElapsedTime = pir_CurrentTime - pir_StartTime;

    Serial.println("Waiting for object to fall, time remaining is: " + (String)pir_ElapsedTime);
    
    if (pir_ElapsedTime<10000)
    {
      // If the PIR has detected an object, close the Bin // start a timer to wait for us to input the material // rotate the NEMA 23 for the AI // Change the state of the object_fallen_pir for future operations
      if (digitalRead(pirPin)) 
      {
        delay(1000); // Before it was 3000
        material_StartTime = millis();
        myservo.write(0);
        delay(1000);
        digitalWrite(driverPUL23, HIGH);
        rotate_motor_23(45,3);
        digitalWrite(LEDPIN, HIGH); 
        object_fallen_pir = true;
        OpenBin = false;   
      }
    }

    if (pir_ElapsedTime>10000)
    {
      object_fallen_pir = false;
      myservo.write(0);
      OpenBin = false;
    }

    input_communication_send = false;

  }

    // If the object has fallen and the bin has closed
  if (object_fallen_pir == true && OpenBin == false) 
  {
          
    // Obtain second timer reading & calculate the elapsed time
     material_CurrentTime = millis();   
     material_ElapsedTime = material_CurrentTime - material_StartTime;

     if (input_communication_send == false)
     {
       //Serial.println("Waiting for object input from RPi, time remaining is: " + (String)material_ElapsedTime);
       Serial.println("Waiting for object input");
       input_communication_send = true;
     }

        // If the elapsed time is less than 10 seconds:
     if (material_ElapsedTime<10000)
     {

          // Obtain the Raspberry Pi output
       data = Serial.readStringUntil('\n');

          // If the Raspberry Pi has send smth
       if (data.length()>0)
       {
        Serial.println(data);

          // Do the motor rotation depending on the data
        bin_type_action(data);

          // Rotate to original position
        rotate_motor_23(45,3);

         // Add one round of sorting to the count for the callibration
        number_of_sorting_rounds = number_of_sorting_rounds + 1;

          // Change the state of the object_fallen_pir for future operations
        object_fallen_pir = false;
       }
     }
     if (material_ElapsedTime>10000)
     {
      digitalWrite(LEDPIN, LOW);

        bin_type_action("General");

          // Rotate to original position
        rotate_motor_23(45,3);

         // Add one round of sorting to the count for the callibration
        number_of_sorting_rounds = number_of_sorting_rounds + 1;

          // Change the state of the object_fallen_pir for future operations
        object_fallen_pir = false;
     }   
  }
 
  if (number_of_sorting_rounds == 100) 
  {
  total_calibration = false;
  NEMA11_cal = false;
  rotate_motor_11_bin4(30,3,0);
    while (total_calibration == false) 
    {
        // read the state of the pushbutton value:
      sensorState = digitalRead(SENSORPIN);
        // check if the sensor beam is broken
          // if it is, the sensorState is LOW:
      if (sensorState == LOW && NEMA11_cal == false) 
        {
          Serial.println("Calibrating Nema 11");
          rotate_motor_11_steps(1,0.01);   
        } 
        
      if (sensorState == HIGH) 
      {
        Serial.println("Calibrating Nema 23");  
        NEMA11_cal = true;
        rotate_motor_23_steps(1,0.05); 
      }
         
      if (sensorState == LOW && NEMA11_cal == true) 
      {
        rotate_motor_23(45,6);
        rotate_motor_11_bin4(7,3,0);
        Serial.println("Calibrating Finished");
        total_calibration = true;
        number_of_sorting_rounds = 0;
      }
     }
  }
}

 
