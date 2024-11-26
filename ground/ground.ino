#define RXD 3
#define TXD 1
#define HC12 Serial

void setup() {
  Serial.begin(9600);
  HC12.begin(9600, SERIAL_8N1, 3, 1);
}

void loop() {
	if (HC12.available()) {
		String message = HC12.readStringUntil('/n');
		Serial.print(message);
	}
}
