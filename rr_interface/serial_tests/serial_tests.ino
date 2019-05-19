//typedef struct Receiver {
//  byte speed;
//  byte steer;
//  byte position;
//};
//
//typedef struct Transmitter {
//  byte payload;  
//};
//
//union Uni {
//  Transmitter frame;
//  uint8_t serialized_array[512];  
//};
//
//Receiver rx_byte;
//union Uni payloadunion;
//Transmitter tx_byte;

int receivedByte;
String input;
unsigned char output = 'w';

void setup() {
  // Setup serial communication with a baud rate of 115200
  // and 8 data bits, no parity bits, 1 stop bit (default if not specified)
  Serial.begin(115200, SERIAL_8N1);
}

void read()
{
  if (Serial.available() > 0) 
  {
    receivedByte = Serial.read();
    Serial.print("Received data: ");
    Serial.print(receivedByte);
    Serial.println();
  }
}

void write()
{
  Serial.write(output);
//  payloadunion.frame.payload = 1;
//  tx_byte.payload = 1;
//  Serial.write('P');
//  Serial.write((uint8_t*)&tx_byte,sizeof(tx_byte));
//  Serial.write('S');
//  return;
}

void loop() {
  //read();
  write();
  //delay(500);
}
