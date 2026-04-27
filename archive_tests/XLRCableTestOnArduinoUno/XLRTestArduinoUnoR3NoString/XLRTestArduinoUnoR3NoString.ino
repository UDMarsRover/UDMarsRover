const int SEND_PIN = 11;    // Outgoing signal
const int RECEIVE_PIN = 10; // Incoming signal
const int STATUS_LED = 13;  // The built-in LED on your Uno

void setup() {
  Serial.begin(9600);
  
  pinMode(SEND_PIN, OUTPUT);
  pinMode(RECEIVE_PIN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  
  Serial.println("--- Starting Simple DC Loop Test ---");
}

void loop() {
  // 1. Send a HIGH signal
  digitalWrite(SEND_PIN, HIGH);
  delay(10); // Wait for the voltage to stabilize
  
  // 2. Read the Receiving Pin
  int signal = digitalRead(RECEIVE_PIN);
  
  if (signal == HIGH) {
    //digitalWrite(STATUS_LED, HIGH); // Turn on built-in LED
    Serial.println("SIGNAL DETECTED: The loop is electrically closed.");
  } else {
    //digitalWrite(STATUS_LED, LOW);  // Turn off built-in LED
    Serial.println("NO SIGNAL: The loop is electrically open.");
  }
  
  delay(500); // Check twice a second
}
