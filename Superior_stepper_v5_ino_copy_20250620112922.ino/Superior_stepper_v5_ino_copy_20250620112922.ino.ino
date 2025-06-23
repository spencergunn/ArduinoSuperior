// Pin definitions
#define STEP_PIN_1_BIT 7  // Pin 7
#define STEP_PIN_2_BIT 4  // Pin 4
#define TEMP_PIN A0

// Global variables for motor control
volatile long stepperPosition1 = 0;
volatile long stepperPosition2 = 0;
volatile bool motorEnabled1 = false;
volatile bool motorEnabled2 = false;
volatile bool stepHigh = false;
volatile long stepsRemaining = 0;
volatile bool direction = true;  // true = forward, false = reverse

// Motion control variables
volatile int currentSpeed = 40;
int targetSpeed = 40;
int maxSpeed = 15000;
int acceleration = 50;
int deceleration = 100;

// Temperature variable
float motorTemp = 25.0;

void setup() {
    Serial.begin(115200);
    
    // Set step pins as outputs using direct port manipulation
    DDRD |= (1 << STEP_PIN_1_BIT) | (1 << STEP_PIN_2_BIT);
    
    // Setup Timer1 for motor pulses
    setupMotorTimer();
}

void loop() {
    static unsigned long lastTempRead = 0;
    static unsigned long lastReport = 0;
    
    // Read temperature every 1000ms
    if (millis() - lastTempRead >= 1000) {
        motorTemp = readLM35Fast(TEMP_PIN);
        lastTempRead = millis();
    }
    
    // Report position and temperature every 500ms
    if (millis() - lastReport >= 500) {
        printFastStatus();
        lastReport = millis();
    }
    
    // Update motion profile
    updateMotionProfile();
    
    // Handle serial commands if any
    if (Serial.available()) {
        handleSerialCommands();
    }
}

// Ultra-fast LM35 temperature reading
float readLM35Fast(uint8_t pin) {
    // Disable interrupts briefly for clean ADC read
    noInterrupts();
    
    ADMUX = (1 << REFS0) | (pin & 0x07);  // AVCC reference, select pin
    ADCSRA |= (1 << ADSC);                // Start conversion
    while (ADCSRA & (1 << ADSC));         // Wait for completion
    int result = ADC;
    
    interrupts();  // Re-enable interrupts
    
    // Convert to temperature: LM35 = 10mV/°C, 4.88mV per ADC step
    return (result * 4.88) / 10.0;
}

// Ultra-fast status reporting: just position and temperature
void printFastStatus() {
    // Get position atomically
    long pos1;
    noInterrupts();
    pos1 = stepperPosition1;
    interrupts();
    
    // Convert temperature to integer (235 = 23.5°C)
    int tempInt = (int)(motorTemp * 10);
    
    // Minimal output: "position temperature\n"
    Serial.print(pos1);
    Serial.print(" ");
    Serial.println(tempInt);
}

// Motor control ISR - optimized for speed
ISR(TIMER1_COMPA_vect) {
    if (stepsRemaining <= 0) return;  // No motion
    
    if (stepHigh) {
        // Step pulse LOW - using direct port manipulation
        if (motorEnabled1) PORTD &= ~(1 << STEP_PIN_1_BIT);
        if (motorEnabled2) PORTD &= ~(1 << STEP_PIN_2_BIT);
        
        // Count steps on falling edge
        if (motorEnabled1) stepperPosition1 += direction ? 1 : -1;
        if (motorEnabled2) stepperPosition2 += direction ? 1 : -1;
        
        stepsRemaining--;
        stepHigh = false;
    } else {
        // Step pulse HIGH
        if (motorEnabled1) PORTD |= (1 << STEP_PIN_1_BIT);
        if (motorEnabled2) PORTD |= (1 << STEP_PIN_2_BIT);
        stepHigh = true;
    }
}

// Motion profile update (runs in main loop)
void updateMotionProfile() {
    if (stepsRemaining <= 0) return;
    
    // Calculate deceleration point
    long stepsToStop = calculateStepsToStop(currentSpeed, deceleration);
    
    if (stepsRemaining <= stepsToStop) {
        targetSpeed = 40;  // Start deceleration
    } else {
        targetSpeed = maxSpeed;  // Continue acceleration/maintain speed
    }
    
    // Update speed
    if (currentSpeed < targetSpeed) {
        currentSpeed += acceleration;
        if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= deceleration;
        if (currentSpeed < 40) {
            currentSpeed = 40;
            motorEnabled1 = false;
            motorEnabled2 = false;
        }
    }
    
    // Update timer frequency safely
    noInterrupts();
    OCR1A = 16000000 / currentSpeed;
    interrupts();
}

// Calculate steps needed to decelerate to target speed
long calculateStepsToStop(int currentSpd, int decel) {
    int targetSpd = 40;
    if (currentSpd <= targetSpd) return 0;
    
    long speedDifference = currentSpd - targetSpd;
    return (speedDifference * speedDifference) / (2 * decel);
}

// Setup Timer1 for motor control
void setupMotorTimer() {
    // Configure Timer1 for CTC mode
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS10);  // CTC mode, no prescaler
    TIMSK1 = (1 << OCIE1A);               // Enable compare interrupt
    OCR1A = 16000000 / currentSpeed;      // Set initial frequency
}

// Safe position reading function
long getCurrentPosition() {
    long pos;
    noInterrupts();
    pos = stepperPosition1;
    interrupts();
    return pos;
}

// Example function to start motion
void moveSteps(long steps) {
    // Stop current motion
    stepsRemaining = 0;
    motorEnabled1 = false;
    
    if (steps == 0) return;
    
    // Set direction
    direction = (steps > 0);
    
    // Setup motion
    stepsRemaining = abs(steps);
    currentSpeed = 40;
    targetSpeed = maxSpeed;
    motorEnabled1 = true;
    
    // Start timer
    TCCR1B |= (1 << CS10);
}

// Basic serial command handler
void handleSerialCommands() {
    if (Serial.available()) {
        long steps = Serial.parseInt();
        if (steps != 0) {
            moveSteps(steps);
        }
    }
}