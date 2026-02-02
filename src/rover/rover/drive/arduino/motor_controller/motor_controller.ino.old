#include <CANSAME5x.h>

#define PACKET_SIZE 25
#define SENSOR_COUNT 6
#define SERIAL_BAUD 115200

CANSAME5x CAN;

enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

enum control_mode {
  Duty_Cycle_Set = 0x2050080,
  Speed_Set = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set = 0x2050C80,
  Voltage_Set = 0x2051080,
  Current_Set = 0x20510C0,
  Smart_Motion_Set = 0x2051480
};

enum UDMRT_status_modes {
  velocity = 0x85,
  other = 0x86
};

enum UDMRT_control_modes {
  set_velocity = 0x01
};

// Heartbeat Frame
const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t HEARTBEAT_DATA[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };

// Control Frame
const uint8_t CONTROL_SIZE = 8;

// Status Frame
const uint8_t STATUS_SIZE = 8;

// Status Packet
uint8_t packet[PACKET_SIZE];  // Buffer for outgoing packet
bool packet_ready = false;

// Receive packer
uint8_t control_packet[PACKET_SIZE];
float velocities[6];

// Status Buffers from REV
uint8_t motor_1_velocity[4];
uint8_t motor_1_other[4];

void setup() {
    Serial.begin(SERIAL_BAUD);
    // Setup CAN pins and bus for Adafruit Feather M4 CAN
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, false);  // turn off STANDBY
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, true);   // turn on booster
    // Serial.println("Initializing CAN...");
    if (!CAN.begin(1000000)) {
      // Serial.println("Starting CAN failed!");
      while (1) delay(10);
    }
    // Serial.println("CAN bus started");
    delay(2000);
}

void loop() {
    // Example sensor data
    // uint8_t temp[SENSOR_COUNT] = {25, 26, 24, 27, 23, 22};  // Temperatures in °C
    // float voltage[SENSOR_COUNT] = {3.3, 3.2, 3.1, 3.4, 3.0, 2.9};  // Volts
    // float current[SENSOR_COUNT] = {3.3, 1.3, 1.1, 1.4, 1.0, 0.9};  // Amps
    
    // constructPacket(0x86, temp, voltage, current);  // Build the packet
    // sendPacket();  // Send packet over Serial
    send_heartbeat();
    listen_for_can_packet(1);
    if (Serial.available() >= PACKET_SIZE) {
        Serial.readBytes(reinterpret_cast<char*>(control_packet), PACKET_SIZE);

        parseVelocitySetPacket(control_packet);
    }

    delay(50);  // Send every 20ms
}

// Convert float to 12-bit fixed point
uint16_t floatToFixed12(float value) {
    return (uint16_t)(value * 4095 / 12.0);  // Scale and round
}

void form_status_packet(uint8_t type) {
  memset(packet, 0, PACKET_SIZE);  // Clear the packet
  packet[0] = type;
}

void set_status_in_packet(uint8_t status[4], uint8_t index) {
  memcpy(packet + 1 + index * 4, status, 4);  // Fix: Correct memory copy logic
}

// Function to construct the packet
void constructPacket(uint8_t messageType, uint8_t temp[], float voltage[], float current[]) {
    packet[0] = messageType;  // Set identifier
    
    int index = 1;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        uint16_t fixedVoltage = floatToFixed12(voltage[i]);
        uint16_t fixedCurrent = floatToFixed12(current[i]);

        packet[index++] = temp[i];                     // 8-bit temperature
        packet[index++] = (fixedVoltage >> 4) & 0xFF;  // First 8 bits of 12-bit voltage
        packet[index] = (fixedVoltage & 0x0F) << 4;    // Last 4 bits of voltage
        packet[index++] |= (fixedCurrent >> 8) & 0x0F; // First 4 bits of current
        packet[index++] = fixedCurrent & 0xFF;         // Last 8 bits of current
    }
}

// Function to send packet over Serial
void sendPacket() {
    Serial.write(packet, PACKET_SIZE);
    Serial.flush();
}

void listen_for_can_packet(uint8_t id) {
    if (CAN.parsePacket()) {  // Check if CAN packet is available
        uint32_t packet_id = CAN.packetId();
        uint8_t buffer[STATUS_SIZE];
        uint32_t status_1_id = status_1 + id;
        if (packet_id == status_1_id) {
            // Serial.print("Received status_1 packet with ID: 0x");
            // Serial.println(packet_id, HEX);

            for (int i = 0; i < STATUS_SIZE; i++) {
                buffer[i] = CAN.read();
            }
            form_status_packet(0x86);
            memcpy(motor_1_other, buffer + 4, 4);  // Fill the motor_1_other buffer with data
            
            set_status_in_packet(motor_1_other, 0);
            sendPacket();
            packet_ready = true; 
        }
    }
    // End the packet processing logic outside of the condition
    packet_ready = false;
}

void send_heartbeat() {
  CAN.beginExtendedPacket(HEARTBEAT_ID);
  CAN.write(HEARTBEAT_DATA, sizeof(HEARTBEAT_DATA));
  CAN.endPacket();
}

void parseVelocitySetPacket(uint8_t* packet) {
    if (packet[0] != set_velocity) {
        return;
    }

    
    for (int i = 0; i < 6; i++) {
        velocities[i] = *reinterpret_cast<float*>(&packet[1 + i * 4]);
    }

    for (int i = 0; i < 6; i++) {
        send_control_frame(i + 1, Smart_Velocity_Set, velocities[i]);
    }
}

void send_control_frame(const uint32_t device_id, const control_mode mode, const float setpoint) {
  // Serial.print("Sending control frame... ");
  
  uint32_t control_id = mode + device_id;

  uint8_t control_data[CONTROL_SIZE] = {CONTROL_SIZE};
  // control_data[0] = 0;
  // control_data[1] = 0;
  // control_data[2] = 32;
  // control_data[3] = 65;
  // control_data[4] = 0;
  // control_data[5] = 0;
  // control_data[6] = 0;
  // control_data[7] = 0;
  // floatToLittleEndian(setpoint, control_data);
  memcpy(control_data, &setpoint, sizeof(setpoint));

  // Serial.print("Sending CAN message with ID: 0x");
  // Serial.println(control_id, HEX);
  // print_control_frame(control_id, control_data, CONTROL_SIZE);
  // for (int i = 0; i < 7; i++) {
  //     Serial.print(control_data[i]);
  //     Serial.print(" ");
  //   }
  // Serial.println(control_data[7]);

  // Send the control frame over CAN
  CAN.beginExtendedPacket(control_id);
  CAN.write(control_data, CONTROL_SIZE);
  if (CAN.endPacket()) {
        // Serial.println("CAN packet sent successfully!");
    } else {
        // Serial.println("CAN packet failed to send.");
    }

  // Serial.println("done");
}
