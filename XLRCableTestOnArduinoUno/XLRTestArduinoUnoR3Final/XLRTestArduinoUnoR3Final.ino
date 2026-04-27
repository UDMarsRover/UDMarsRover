
#include <SoftwareSerial.h>

/* * WIRING RECAP:
 * Arduino Pin 11 (TX) -> Rover Pin 2 (red) -> [XLR Cable] -> Vault Pin 2 (red)
 * [Breadboard Bridge] -> Vault Pin 2 (yellow) to Vault Pin 3 (yellow)
 * Vault Pin 3 (yellow) -> [XLR Cable] -> Rover Pin 3 (yellow) -> Arduino Pin 10 (RX)
 * All Pin 1s (Grounds; black) connected to Arduino GND
 */

// Define the "Software" serial port
// RX is Pin 10 (Listener), TX is Pin 11 (Voice)
SoftwareSerial xlrLoop(10, 11); 

// We will send this unique string to make sure we aren't just hearing "noise"
String testMessage = "HEIST_CABLE_OK_123"; 

void setup() {
  // Initialize communication with your Mac (USB)
  Serial.begin(9600);
  
  // Initialize communication with the XLR Cable
  // We use 9600 because it's very stable for testing long/spliced cables
  xlrLoop.begin(9600);
  
  Serial.println("========================================");
  Serial.println("  XLR 6-WIRE INTEGRITY TEST STARTING    ");
  Serial.println("========================================");
  delay(1000); 
}

void loop() {
  Serial.print("Sending signal down the line... ");

  // 1. CLEAR the buffer - throw away any old "trash" data
  while(xlrLoop.available() > 0) { 
    xlrLoop.read(); 
    }

  // 2. SEND the message out of Pin 11
  xlrLoop.println(testMessage);

  // 3. WAIT a moment for the electricity to travel the loop
  delay(100); 

  // 4. CHECK if it came back into Pin 10
  if (xlrLoop.available() > 0) {
    // Read the incoming string until the newline character
    String response = xlrLoop.readStringUntil('\n');
    response.trim(); // Remove any invisible spaces/extra characters

    if (response == testMessage) {
      Serial.println(" [ SUCCESS ] ");
      Serial.println(">>> Message matched! All 6 wires and joints are solid.");
    } else {
      Serial.println(" [ DATA CORRUPTION ] ");
      Serial.print(">>> Sent: " + testMessage);
      Serial.println(" | Received: " + response);
      Serial.println("Tip: Check for loose connections or noise.");
    }
  } 
  else {
    Serial.println(" [ FAILURE ] ");
    Serial.println(">>> No signal returned. The loop is broken.");
    Serial.println("Check: Solder joints, middle jumpers, or GND connection.");
    delay(1000);
    exit(0);
  }

  Serial.println("----------------------------------------");
  delay(2000); // Wait 2 seconds before the next check
}
