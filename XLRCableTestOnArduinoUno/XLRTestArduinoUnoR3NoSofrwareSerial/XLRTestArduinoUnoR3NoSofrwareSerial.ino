/*
 * HARDWARE SERIAL CLASH TEST
 * Wiring: 
 * Pin 1 (TX) -> Rover XLR Pin 2 -> [Cable] -> Vault XLR Pin 2/3 Bridge -> [Cable] -> Rover XLR Pin 3 -> Pin 0 (RX)
 * All GNDs connected.
 */

void setup() {
  // This initializes the hardware serial port shared by USB and Pins 0/1
  Serial.begin(9600); 
  Serial.println("--- SYSTEM ONLINE: PREPARING FOR CLASH ---");
  delay(2000);
}

void loop() {
  // 1. Send the message. 
  // This goes to MAC and THE XLR CABLE simultaneously.
  Serial.println("Heist_Task_Test");

  // 2. Give the signal time to travel the physical loop
  delay(100);

  // 3. Check the "mailbox"
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();

    // 4. Print the result back to the same line
    // This will likely cause a feedback loop!
    Serial.print("ECHO RECEIVED: ");
    Serial.println(incoming);
  }

  delay(1000);
}
