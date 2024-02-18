

void setup() 
  {

    Serial.begin(9600);

    pinMode(9,OUTPUT); digitalWrite(9,HIGH);

    Serial1.begin(38400);

  }

void loop()

  {

    if (Serial1.available())

    Serial.write(Serial1.read());

    if (Serial.available())

    Serial1.write(Serial.read());

  }
