/*
 * Arduino Input/Output Control with Latch Functionality and Input Debounce
 * 
 * This program uses 5 input pins and 4 output pins:
 * - masterInputPin: When HIGH, allows other inputs to toggle outputs
 * - 4 other input pins: Each toggles the state of its corresponding output pin
 * - 4 output pins: Each latches/holds its state when toggled by its input
 */
#include <Arduino.h>
// Define pin numbers for inputs
const int masterInputPin = A0;   // Master control input
const int inputPin1 = 3;
const int inputPin2 = 4;
const int inputPin3 = 5;
const int inputPin4 = 6;

// Define custom voltage threshold for master input (assuming 5V reference)
const int masterThreshold = 400;

// Master input debounce variables
const int masterDebounceCount = 10;  // Number of consecutive readings needed to change state
int masterReadings = 0;             // Counter for consistent readings
int stableMasterState = LOW;        // Current stable state of master input

// Define pin numbers for outputs
const int outputPin1 = 8;
const int outputPin2 = 9;
const int outputPin3 = 10;
const int outputPin4 = 11;

const int buzzerPin = 2;

// Variables to store previous input states for edge detection
int prevInput1 = LOW;
int prevInput2 = LOW;
int prevInput3 = LOW;
int prevInput4 = LOW;

// Variables to store output states
int outputState1 = LOW;
int outputState2 = LOW;
int outputState3 = LOW;
int outputState4 = LOW;

void buzz()
{
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
}

void setup() {
  // Initialize input pins
  pinMode(masterInputPin, INPUT);
  pinMode(inputPin1, INPUT);
  pinMode(inputPin2, INPUT);
  pinMode(inputPin3, INPUT);
  pinMode(inputPin4, INPUT);
  
  // Initialize output pins
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(outputPin3, OUTPUT);
  pinMode(outputPin4, OUTPUT);

  pinMode(buzzerPin, OUTPUT);
  
  // Initialize all outputs to LOW
  digitalWrite(outputPin1, LOW);
  digitalWrite(outputPin2, LOW);
  digitalWrite(outputPin3, LOW);
  digitalWrite(outputPin4, LOW);
  
  digitalWrite(buzzerPin, LOW);
  // Optional: Start serial for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the analog value of master input pin
  int masterValue = analogRead(masterInputPin);
  int rawMasterState = (masterValue > masterThreshold) ? HIGH : LOW;
  
  // Debounce the master input
  if (rawMasterState == stableMasterState) {
    // Input matches current state, reset the counter
    masterReadings = 0;
  } else {
    // Input differs from current state, increment the counter
    masterReadings++;
    
    // If we've seen enough consistent readings, change the state
    if (masterReadings >= masterDebounceCount) {
      stableMasterState = rawMasterState;
      masterReadings = 0;
      
      Serial.print("Stable master state changed to: ");
      Serial.println(stableMasterState ? "HIGH" : "LOW");
      digitalWrite(buzzerPin, HIGH);
      delay(100);
      digitalWrite(buzzerPin, LOW);
      delay(100);
      digitalWrite(buzzerPin, HIGH);
      delay(100);
      digitalWrite(buzzerPin, LOW);
      delay(100);
      digitalWrite(buzzerPin, HIGH);
      delay(100);
      digitalWrite(buzzerPin, LOW);
    }
  }
  
  // Read current input states
  int currentInput1 = digitalRead(inputPin1);
  int currentInput2 = digitalRead(inputPin2);
  int currentInput3 = digitalRead(inputPin3);
  int currentInput4 = digitalRead(inputPin4);
  
  // If master input is HIGH, check for input transitions to toggle outputs
  if (stableMasterState == HIGH) {
    // Check for transitions
    if (currentInput1 == HIGH && prevInput1 == LOW) {
      outputState1 = !outputState1;  // Toggle output state
      digitalWrite(outputPin1, outputState1);
      Serial.print("Output 1 toggled to: ");
      Serial.println(outputState1 ? "HIGH" : "LOW");
      buzz();
    }
    
    if (currentInput2 == HIGH && prevInput2 == LOW) {
      outputState2 = !outputState2;
      digitalWrite(outputPin2, outputState2);
      Serial.print("Output 2 toggled to: ");
      Serial.println(outputState2 ? "HIGH" : "LOW");
      buzz();
    }
    
    if (currentInput3 == HIGH && prevInput3 == LOW) {
      outputState3 = !outputState3;
      digitalWrite(outputPin3, outputState3);
      Serial.print("Output 3 toggled to: ");
      Serial.println(outputState3 ? "HIGH" : "LOW");
      buzz();
    }
    
    if (currentInput4 == HIGH && prevInput4 == LOW) {
      outputState4 = !outputState4;
      digitalWrite(outputPin4, outputState4);
      Serial.print("Output 4 toggled to: ");
      Serial.println(outputState4 ? "HIGH" : "LOW");
      buzz();
    }
  } else if (stableMasterState == LOW) {
    // If master input is LOW, all outputs are set to LOW
    outputState1 = LOW;
    outputState2 = LOW;
    outputState3 = LOW;
    outputState4 = LOW;
    
    digitalWrite(outputPin1, LOW);
    digitalWrite(outputPin2, LOW);
    digitalWrite(outputPin3, LOW);
    digitalWrite(outputPin4, LOW);
  }
  
  // Store current input states for next iteration
  prevInput1 = currentInput1;
  prevInput2 = currentInput2;
  prevInput3 = currentInput3;
  prevInput4 = currentInput4;
  
  // Small delay to prevent excessive looping
  delay(20);
}