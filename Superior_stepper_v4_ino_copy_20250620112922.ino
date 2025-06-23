//#include <DHT.h>
#include <util/atomic.h>

#define FIRMWARE_VERSION_STRING  "1.0.0.0"
#define STEP_PIN_1_BIT 7  // Pin 7
#define STEP_PIN_2_BIT 4  // Pin 4
#define DIR_PIN_1_BIT 6  // Pin 6
#define DIR_PIN_2_BIT 3  // Pin 3
#define DIS_PIN_1_BIT 5  // Pin 6
#define DIS_PIN_2_BIT 2  // Pin 3

volatile long stepperPosition1 = 0;
volatile long stepperPosition2 = 0;
volatile bool direction = true; // true for forward, false for backward

volatile int targetSpeed = 1000; // Target speed in steps per second
volatile int setSpeed = 1000;
volatile int acceleration = 10; // Acceleration in steps per second^2
volatile int deceleration = 32; // Deceleration in steps per second^2
volatile long stepsToStop = 0;
volatile long inputSteps =0;
volatile unsigned long distance = 100000 ;  
volatile bool motionStarted1 = false;
volatile bool motionStarted2 = false;

volatile bool motorEnabled1 = false;
volatile bool motorEnabled2 = false;


volatile bool stepHigh = false;

// Motion control variables (updated by main loop)
volatile int currentSpeed = 40;
volatile long stepsRemaining = 0;




// Define the DHT sensor pin and type
// #define DHTPIN 11
// #define DHTTYPE DHT11
int FAN_POWER_PIN = 8;
int DET_POWER_PIN=9;
int PEL_POWER_PIN = 10;

// // Create an instance of the DHT sensor
// DHT dht(DHTPIN, DHTTYPE);

void setup() {
 
  Serial.begin(115200);
 // Initialize the DHT sensor
 // dht.begin();

  pinMode(STEP_PIN_1_BIT, OUTPUT);
  pinMode(DIR_PIN_1_BIT, OUTPUT);
  //pinMode(DIS_PIN_1, OUTPUT);
  
  pinMode(STEP_PIN_2_BIT, OUTPUT);
  pinMode(DIR_PIN_2_BIT, OUTPUT);
  //pinMode(DIS_PIN_2, OUTPUT);

  pinMode(FAN_POWER_PIN, OUTPUT);
  pinMode(DET_POWER_PIN, OUTPUT);
  pinMode(PEL_POWER_PIN, OUTPUT);

  digitalWrite(DIS_PIN_1_BIT,  LOW);
  digitalWrite(DIS_PIN_2_BIT,  LOW);
  // Configure Timer1 for motor control
  motorEnabled1 = false;
  motorEnabled1 = false;

  cli(); // Disable interrupts
  TCCR1A = 0; // Set Timer1 to CTC mode
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 16000; // Initial compare value (adjust as needed)
  TCCR1B |= (1 << WGM12) | (1 << CS10); // CTC mode, no prescaler
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
  sei(); // Enable interrupts



  // Initialize other components as needed
}

float read_temp()
{
  unsigned int total = 0; // A/D readings
  float tempC; // Celcius
  float tempF; // Fahrenheit
    // read the sensor
for (int x = 0; x < 16; x++) {
    total = total + analogRead(A0);
}


tempC = total / 16.0;

tempC = (tempC / 1023.0) * 5.0;

tempC = tempC * 100;
return tempC;
}

ISR(TIMER1_COMPA_vect) {
    if (stepsRemaining <= 0) return;  // No motion
    
    if (stepHigh) {
        // Step pulse LOW
        if (motorEnabled1) PORTB &= ~(1 << STEP_PIN_1_BIT);
        if (motorEnabled2) PORTB &= ~(1 << STEP_PIN_2_BIT);
        
        // Count steps
        if (motorEnabled1) stepperPosition1 += direction ? 1 : -1;
        if (motorEnabled2) stepperPosition2 += direction ? 1 : -1;
        
        stepsRemaining--;
        stepHigh = false;
    } else {
        // Step pulse HIGH  
        if (motorEnabled1) PORTB |= (1 << STEP_PIN_1_BIT);
        if (motorEnabled2) PORTB |= (1 << STEP_PIN_2_BIT);
        stepHigh = true;
    }
}

// ISR(TIMER1_COMPA_vect) {

//   //static int stepInterval = 16000000 / currentSpeed; // in microseconds//
//   // Determine the direction why in here ?
//   digitalWrite(DIR_PIN_1, direction ? HIGH : LOW);
//   digitalWrite(DIR_PIN_2, direction ? HIGH : LOW);
  
  
//   // Move the motor one step
//   if(motorEnabled1){digitalWrite(STEP_PIN_1, HIGH);}
//   if(motorEnabled2){digitalWrite(STEP_PIN_2, HIGH);}
//   delayMicroseconds(1); // Ensure the step pulse is long enough
//   if(motorEnabled1){digitalWrite(STEP_PIN_1, LOW);}
//   if(motorEnabled2){digitalWrite(STEP_PIN_2, LOW);}
  


//   if( motionStarted1 || motionStarted2 ){
//     // Update the stepper position
//     if( motionStarted1 ) stepperPosition1 += direction ? 1 : -1;
//     if( motionStarted2 ) stepperPosition2 += direction ? 1 : -1;
    
//     distance --;

    
//     if( distance == 0)
//     {
//       currentSpeed = 40;
//       motionStarted1 = false;
//       motionStarted2 = false;
//     }
//     else if( distance <= stepsToStop  )
//     {
    
//       targetSpeed = 40;
    
//     }
//   }

//   // Accelerate or decelerate
//   if (currentSpeed < targetSpeed) {
//     currentSpeed += acceleration;
//   } else if (currentSpeed > targetSpeed) {
//     currentSpeed -= deceleration;
//     if( currentSpeed < 50 ) {
//       motorEnabled1 = false;
//       motorEnabled2 = false;
//     }
    
//   }

  
//   // Update the step interval
//   //stepInterval = 16000000 / currentSpeed;
  
//   OCR1A = 16000000 / currentSpeed;  // Adjust for the timer resolution and prescaler
// }

void setDirection( int steps)
{
  bool newDirection = (steps >= 0);
    
    // Set direction pin immediately
    if (newDirection) {
        PORTB |= (1 << DIR_PIN_1_BIT);   // Forward
         PORTB |= (1 << DIR_PIN_2_BIT);   // Forward
    } else {
        PORTB &= ~(1 << DIR_PIN_1_BIT);  // Reverse
        PORTB &= ~(1 << DIR_PIN_2_BIT);  // Reverse
        steps = -steps;  // Make steps positive for counting
    }
}
long calculateStepsToStop(int currentSpd, int decel) {
    int targetSpd = 40;
    
    if (currentSpd <= targetSpd) return 0;
    
    long steps = 0;
    int tempSpeed = currentSpd;
    
    // Simulate the deceleration step by step
    while (tempSpeed > targetSpd) {
        tempSpeed -= decel;
        if (tempSpeed < targetSpd) tempSpeed = targetSpd;
        steps++;
    }
    
    return steps;
}

void updateMotionProfile() {
    if (stepsRemaining <= 0) return;
    
    // Calculate if we need to start decelerating
    long stepsToStop = calculateStepsToStop(currentSpeed, deceleration);
    
    if (stepsRemaining <= stepsToStop) {
        targetSpeed = 40;  // Deceleration phase
    }
    
    // Accelerate or decelerate
    if (currentSpeed < targetSpeed) {
        currentSpeed += acceleration;
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= deceleration;
        if (currentSpeed < 50) {
            motorEnabled1 = false;
            motorEnabled2 = false;
        }
    }
    
    // Update timer frequency (safe to do in main loop)
    noInterrupts();
    OCR1A = 16000000 / currentSpeed;
    interrupts();
}

long readPositionData(int flag)
{
  
    long pos;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if( flag == 1){
        pos = stepperPosition1;
      }
      else
      {
        pos= stepperPosition2;
      }
    }
    return pos;

}

void loop() {


 static unsigned long lastSpeedUpdate = 0;
    
    if (millis() - lastSpeedUpdate > 1) {  // Update speed every 1ms
        updateMotionProfile();
        lastSpeedUpdate = millis();
    }
    
    


 if (Serial.available() > 0) {

        //do something
         // Read the input as a string
          String input = Serial.readStringUntil('\n');
          input.trim(); // Remove any leading/trailing whitespace

          if( input.equals("!"))
          {
                      targetSpeed  = 40;
                      distance = 5000;
                      Serial.println("stop: stopped");
          }
          else if (input.startsWith("ALL ")) {
                String command = input.substring(4);
                Serial.print(command);
                Serial.print(": ");
                    if (command.startsWith("stop")){
                      targetSpeed  = 40;
                      distance = 5000;
                      Serial.println("stopped");
                    }
                    else if (command.startsWith("sethome")){
                     
                      stepperPosition1 = 0;
                      stepperPosition2 = 0;
                      Serial.println("home");
                    }
                     else if (command.startsWith("start ")){
                      inputSteps =  strtol(command.substring(6).c_str(), 0, 10);
                      
                      setDirection(inputSteps);
                      distance = abs(inputSteps);
                      if(distance != 0 ){
                      currentSpeed = 1000;
                      targetSpeed = setSpeed;
                      stepsToStop = targetSpeed  / deceleration ; // divide by deceleration;
                      motorEnabled1= true;
                      motorEnabled2= true;
                      motionStarted1 = true;
                      motionStarted2 = true;
                      Serial.print(distance);
                      Serial.println(" started");
                      }
                      else
                      {
                        Serial.println("no distance set");
                      }
                    }
                    else if (command.startsWith("start")){
                      if(distance > 0 ){
                      currentSpeed = 1000;
                      targetSpeed = setSpeed;
                      stepsToStop = targetSpeed  / deceleration ; // divide by deceleration;
                      motorEnabled1= true;
                      motorEnabled2 = true;
                      motionStarted1 = true;
                      motionStarted2 = true;
                      Serial.println("started");
                      }
                      else
                      {
                        Serial.println("no distance set");
                      }
                    }
                    else if (command.startsWith("home")){
                        
                        distance = abs(stepperPosition1);

                        setDirection(stepperPosition1);

                      currentSpeed = 1000;
                      targetSpeed = setSpeed;
                      stepsToStop = targetSpeed  / deceleration ; 
                      motorEnabled1= true;
                      motorEnabled2= true;
                      motionStarted1 = true;
                      motionStarted2 = true;
                      Serial.println("homing");
                    }
                    else if (command.startsWith("speed ")) {
                    
                      setSpeed = command.substring(6).toInt();
                      targetSpeed = setSpeed;
                      Serial.print("Motor speeds set to ");
                      Serial.print(targetSpeed);
                      Serial.println(" steps per second.");
                    }
                    else if( command.startsWith("speed"))
                    {
                      Serial.println(targetSpeed);
                    }
                    else if (command.startsWith("distance ")){
                        

                        int inputDistance = strtol(command.substring(9).c_str(), 0, 10);
                        setDirection(inputDistance);

                        distance = abs(inputDistance);
                        Serial.print("Motor distance set to ");
                        Serial.println(inputDistance);
                    }
                     else if (command.startsWith("distance")){
                        
                        
                        Serial.println(distance);
                    }
          }





          // Process commands for Motor 1
         else if (input.startsWith("M1 ")) {
             String command = input.substring(3);
             Serial.print(command);
             Serial.print(": ");
                      if (command.startsWith("stop")){
                        targetSpeed  = 40;
                        distance = 5000;
                        Serial.println("stopping");
                      }
                      else if (command.startsWith("start ")){
                        inputSteps =  strtol(command.substring(6).c_str(), 0, 10);
                        setDirection(inputSteps);
                        distance = abs(inputSteps);
                        if( distance != 0 ) {
                          
                          targetSpeed = setSpeed;
                          Serial.print(targetSpeed);
                          Serial.print(", ");
                          Serial.print(distance);
                          stepsToStop = targetSpeed  / deceleration ; 
                          currentSpeed = 1000;
                          Serial.print(", ");
                          Serial.println(currentSpeed);
                          motorEnabled1 = true;
                           motionStarted1 = true;
                        }
                      }
                      else if (command.startsWith("start")){
                        if( distance > 0 ) {
                          
                          targetSpeed = setSpeed;
                          Serial.print(targetSpeed);
                          Serial.print(", ");
                          Serial.print(distance);
                          stepsToStop = targetSpeed  / deceleration ; 
                          currentSpeed = 1000;
                          Serial.print(", ");
                          Serial.println(currentSpeed);
                           motorEnabled1 = true;
                           motionStarted1 = true;
                        }
                      }
                      else if (command.startsWith("home")){
                        
                        distance = abs(stepperPosition1);

                        setDirection(stepperPosition1);
                         currentSpeed = 1000;
                         targetSpeed = setSpeed;
                         stepsToStop = targetSpeed  / deceleration ; 

                         motorEnabled1 = true;
                         motionStarted1 = true;
                         Serial.println("homing 1");
                      }
                      else if (command.startsWith("pos")){
                        Serial.println(readPositionData(1));
                      }
                       

            }

            // Process commands for Motor 2
           else if (input.startsWith("M2 ")) {
             String command = input.substring(3);
                       Serial.print(command);
                       Serial.print(": ");
                      if (command.startsWith("stop")){
                        targetSpeed  = 40;
                        distance = 5000;
                      }
                       
                        else if (command.startsWith("start ")){
                        inputSteps =  strtol(command.substring(6).c_str(), 0, 10);
                        setDirection(inputSteps);
                        distance = abs(inputSteps);
                        if( distance != 0 ) {
                         
                          currentSpeed = 1000;
                          targetSpeed= setSpeed;
                          stepsToStop = targetSpeed  / deceleration ; 
                          Serial.print(targetSpeed);
                          Serial.print(", ");
                          Serial.print(distance);
                          Serial.print(", ");
                          Serial.println(currentSpeed);
                          motorEnabled2 = true;
                          motionStarted2 = true;
                        }
                     
                      }
                      else if (command.startsWith("start")){
                        if( distance > 0 ) {
                         
                          currentSpeed = 1000;
                          targetSpeed= setSpeed;
                          stepsToStop = targetSpeed  / deceleration ; 
                          Serial.print(targetSpeed);
                          Serial.print(", ");
                          Serial.print(distance);
                          Serial.print(", ");
                          Serial.println(currentSpeed);
                          motorEnabled2 = true;
                          motionStarted2 = true;
                        }
                     
                      }
                      else if (command.startsWith("home")){
                        
                        distance = abs(stepperPosition2);

                        setDirection(stepperPosition2);
                        
                         currentSpeed = 1000;
                         targetSpeed = setSpeed;
                         stepsToStop = targetSpeed  / deceleration ; 
                         motorEnabled2 = true;
                         motionStarted2 = true;
                         Serial.println("homing 2");
                      }
                      else if (command.startsWith("pos")){
                        Serial.println(readPositionData(2));
                      }

            }
            else if (input.equalsIgnoreCase("temp")) {

                float t = read_temp(); // Temp in celcius
                Serial.print("Temperature: ");
                Serial.println(t);

              // Read temperature as Celsius DHT11
              // float tempC = dht.readTemperature();
              // // Read humidity
              // float humidity = dht.readHumidity();

              // // Check if any reads failed and exit early (to try again)
              // if (isnan(tempC) || isnan(humidity)) {
              //   Serial.println("Failed to read from DHT sensor!");
              // } else {
              //   Serial.print("Temperature: ");
              //   Serial.print(tempC);
              //   Serial.print(" *C, Humidity: ");
              //   Serial.print(humidity);
              //   Serial.println(" %");
              // }
            }
            else if (input.equalsIgnoreCase("$01F"))
            {
              // returns firmware version number
              Serial.print("Firmware : ");
              Serial.println(FIRMWARE_VERSION_STRING);
            }
            else if (input.equalsIgnoreCase("FANON"))
            {
              digitalWrite(FAN_POWER_PIN, HIGH);
              Serial.println("Fan: on");
            }
            else if (input.equalsIgnoreCase("FANOFF"))
            {
              digitalWrite(FAN_POWER_PIN, LOW);
              Serial.println("Fan: off");
            }
            else if (input.equalsIgnoreCase("DETON"))
            {
              digitalWrite(DET_POWER_PIN, HIGH);
              Serial.println("Detector: on");
            }
            else if (input.equalsIgnoreCase("DETOFF"))
            {
              digitalWrite(DET_POWER_PIN, LOW);
              Serial.println("Detector: off");
            }
            else if (input.equalsIgnoreCase("PELON"))
            {
              digitalWrite(PEL_POWER_PIN, HIGH);
              Serial.println("Peltier: on");
            }
            else if (input.equalsIgnoreCase("PELOFF"))
            {
              digitalWrite(PEL_POWER_PIN, LOW);
              Serial.println("Peltier: off");
            }
            else
            {
              Serial.println("Uknown command "+ input);
            }
        }
}
