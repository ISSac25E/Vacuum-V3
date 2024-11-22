volatile const uint8_t start[] PROGMEM = {0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1};


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("init");
  Serial.println(sizeof(start));

  readArr(start, sizeof(start));
}

void loop() {
  // put your main code here, to run repeatedly:

}

uint8_t *ptr;

void readArr(uint8_t *c, uint16_t len) {
  ptr = c;
  for (uint16_t x = 0; x < len; x++)
    Serial.println(pgm_read_byte_near(ptr + x));
}

void readArr(bool *c, uint16_t len) {
  ptr = (uint8_t*)c;
  for (uint16_t x = 0; x < len; x++)
    Serial.println((bool)pgm_read_byte_near(ptr + x));
}