typedef struct Receiver {
  byte speed;
  byte steer;
  byte position;
};

typedef struct Transmitter {
  byte payload;  
};

union Uni {
  Transmitter frame;
  uint8_t serialized_array[512];  
};

Receiver rx_byte;
union Uni payloadunion;
Transmitter tx_byte;

void setup() {
  Serial.begin(115200);
}

//void read()
//{
//  if (Serial.available()) 
//  {
//    // Can only read one byte at a time
//    rx_byte = Serial.read();
//    Serial.print("Received data: ");
//    Serial.print(rx_byte);
//    Serial.println();
//  }
//}

void write()
{
  //payloadunion.frame.payload = 1;
  tx_byte.payload = 1;
  Serial.write('P');
  Serial.write((uint8_t*)&tx_byte,sizeof(tx_byte));
  Serial.write('S');
  return;
}

void loop() {
  //read();
  write();
  delay(10);
}


