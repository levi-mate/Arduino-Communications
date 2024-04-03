#define STX 0x70
#define ETX 0x71

char txButton, txTilt, txPot, txA, txB, txC, txD;
char txBuffer[8] = {0,0,0,0,0,0,0,ETX};
char rxBuffer[7];
char rxButton, rxTilt, rxPot, rxA, rxB, rxC, rxD;
int  rx_index;

const int buttonPin = A4;
const int buzzerPin = A3;

const int tiltPin = 4;
const int outLEDpin = 13;

int potPin = A5;
const int digitA = 7;
const int digitB = 6;
const int digitC = 10;
const int digitD = 11;
const int digitE = 12;
const int digitF = 8;
const int digitG = 9;

const int joyX = A1;
//const int joyY = A2;
const int motorPin = 5;


void readInputs()
{
  // Reads the inputs in the mini-projects
  // Uses txButton, txTilt, txPot, txA, txB, txC, txD;
  
  if (digitalRead(buttonPin) == LOW) {
    txButton = 1;
  }
  if (digitalRead(buttonPin) == HIGH) {
    txButton = 0;
  }
  
  // Read in tilt switch and convert output to 1 or 0
  if (digitalRead(tiltPin) == HIGH) {
    txTilt = 1;
  }
  if (digitalRead(tiltPin) == LOW) {
    txTilt = 0;
  }
  
  // Read in potentiometer and map it to txPot (0...7)
  int potVal = analogRead(potPin);
  txPot = map(potVal, 0, 1023, 0, 7);
 
  // Read joystick X axis and map it to txA (0...99)
  int val = analogRead(joyX);
  txA = map(val, 0, 1023, 0, 99);

  txB = map(val, 0, 1023, 0, 99);
  
  txC = map(val, 0, 1023, 0, 99);
  
  txD = map(val, 0, 1023, 0, 99);
}

void writeOutputs()
{
  // Writes the outputs in the mini-projects
  // Uses rxButton, rxTilt, rxPot, rxA, rxB, rxC, rxD;
  
  // Buzzer
  if (rxButton == 1) {
    digitalWrite(buzzerPin, HIGH);
    tone(buzzerPin, 440);
    delay(2000);
    digitalWrite(buzzerPin, LOW);
  }
  if (rxButton == 0) {
    digitalWrite(buzzerPin, LOW);
    noTone(buzzerPin);
  }
  
  // LED
  if (rxTilt == 1) {
		digitalWrite(outLEDpin, HIGH);
	}
	if (rxTilt == 0) {
		digitalWrite(outLEDpin, LOW);
	}

	// 1 digit 7 segment display
  switch (rxPot) {
    case 0:
      digitalWrite(digitA, HIGH);
      digitalWrite(digitB, HIGH);
      digitalWrite(digitC, HIGH);
      digitalWrite(digitD, HIGH);
      digitalWrite(digitE, HIGH);
      digitalWrite(digitF, HIGH);
      digitalWrite(digitG, LOW);
      break;
    case 1:
      digitalWrite(digitA, LOW);
      digitalWrite(digitB, HIGH);
      digitalWrite(digitC, HIGH);
      digitalWrite(digitD, LOW);
      digitalWrite(digitE, LOW);
      digitalWrite(digitF, LOW);
      digitalWrite(digitG, LOW);
      break;
    case 2:
      digitalWrite(digitA, HIGH);
      digitalWrite(digitB, HIGH);
      digitalWrite(digitC, LOW);
      digitalWrite(digitD, HIGH);
      digitalWrite(digitE, HIGH);
      digitalWrite(digitF, LOW);
      digitalWrite(digitG, HIGH);
      break;
    case 3:
      digitalWrite(digitA, HIGH);
      digitalWrite(digitB, HIGH);
      digitalWrite(digitC, HIGH);
      digitalWrite(digitD, HIGH);
      digitalWrite(digitE, LOW);
      digitalWrite(digitF, LOW);
      digitalWrite(digitG, HIGH);
      break;
    case 4:
      digitalWrite(digitA, LOW);
      digitalWrite(digitB, HIGH);
      digitalWrite(digitC, HIGH);
      digitalWrite(digitD, LOW);
      digitalWrite(digitE, LOW);
      digitalWrite(digitF, HIGH);
      digitalWrite(digitG, HIGH);
      break;
    case 5:
      digitalWrite(digitA, HIGH);
      digitalWrite(digitB, LOW);
      digitalWrite(digitC, HIGH);
      digitalWrite(digitD, HIGH);
      digitalWrite(digitE, LOW);
      digitalWrite(digitF, HIGH);
      digitalWrite(digitG, HIGH);
      break;
    case 6:
      digitalWrite(digitA, HIGH);
      digitalWrite(digitB, LOW);
      digitalWrite(digitC, HIGH);
      digitalWrite(digitD, HIGH);
      digitalWrite(digitE, HIGH);
      digitalWrite(digitF, HIGH);
      digitalWrite(digitG, HIGH);
      break;
    case 7:
      digitalWrite(digitA, HIGH);
      digitalWrite(digitB, HIGH);
      digitalWrite(digitC, HIGH);
      digitalWrite(digitD, LOW);
      digitalWrite(digitE, LOW);
      digitalWrite(digitF, LOW);
      digitalWrite(digitG, LOW);
      break;
  }

	// Fan motor
	int speedVal = rxA;
	int mSpeed = map(speedVal, 0, 99, 0, 255);
	analogWrite(motorPin, mSpeed);
}

int ledState = LOW;

void setup()
{
  Serial.begin(9600);
  
  pinMode(3, OUTPUT);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  
  pinMode(tiltPin, INPUT);
  digitalWrite(tiltPin, LOW);
  pinMode(outLEDpin, OUTPUT);
  
  pinMode(potPin, INPUT);
  pinMode(digitA, OUTPUT);
  pinMode(digitB, OUTPUT);
  pinMode(digitC, OUTPUT);
  pinMode(digitD, OUTPUT);
  pinMode(digitE, OUTPUT);
  pinMode(digitF, OUTPUT);
  pinMode(digitG, OUTPUT);
  
  pinMode(joyX, INPUT);
  pinMode(motorPin, OUTPUT);
}


const long txInterval = 200;
int tx_state = 0;
char tx_char = 'H';
char chr;
unsigned long previousTxMillis = 0;

char tx_string[] = "Hello World";
#define TX_START_OF_TEXT  -1
int tx_string_state = TX_START_OF_TEXT;

char getTxChar()
{
  char chr;
  
  switch (tx_string_state)
  {
    case TX_START_OF_TEXT:
    tx_string_state = 0;
    txBuffer[0] = txButton;
    txBuffer[1] = txTilt;
    txBuffer[2] = txPot;
    txBuffer[3] = txA;
    txBuffer[4] = txB;
    txBuffer[5] = txC;
    txBuffer[6] = txD;
    return STX;
    break;
    
    default:
    chr = txBuffer[tx_string_state];
    tx_string_state++;
    if (chr == ETX)
    {
      tx_string_state = TX_START_OF_TEXT;
      return ETX;
    }
    else
    {
      return chr;
    }
    break;
  }
}

void txChar()
{
  unsigned long currentTxMillis = millis();

  if (currentTxMillis - previousTxMillis >= txInterval)
  {
    previousTxMillis = previousTxMillis + txInterval;

    switch (tx_state)
    {
      case 0:
        chr = getTxChar();
        digitalWrite(3, HIGH);
        tx_state++;
        break;

      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
        if ((chr & 0x40) != 0)
        {
          digitalWrite(3, HIGH);
        }
        else
        {
          digitalWrite(3, LOW);
        }
        chr = chr << 1;
        tx_state++;
        break;

      case 8:
        digitalWrite(3, HIGH);
        tx_state++;
        break;

      default:
        digitalWrite(3, LOW);
        tx_state++;
        if (tx_state > 20) tx_state = 0;
        break;
    }
  }
}



const long rxInterval = 20;
int rx_state = 0;
char rx_char;
unsigned long previousRxMillis = 0;
int rx_bits[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


void rxChar()
{
  unsigned long currentRxMillis = millis();
  int sensorValue;
  int i;

  if (currentRxMillis - previousRxMillis >= rxInterval)
  {
    previousRxMillis = previousRxMillis + rxInterval;

    sensorValue = analogRead(A0);
    //Serial.println(rx_state);

    switch (rx_state)
    {
      case 0:
        if (sensorValue >= 900)
        {
          rx_bits[0]++;
          rx_state++;
        }
        break;

      case 100:
        if ((rx_bits[0] >= 6) && (rx_bits[8] >= 6))
        {
          rx_char = 0;

          for (i = 1; i < 8; i++)
          {
            rx_char = rx_char << 1;
            if (rx_bits[i] >= 6) rx_char = rx_char | 0x01;
          }
          
          rx_char = rx_char;
          
          switch (rx_char)
          {
            case STX:
            rx_index = 0;
            break;
            
            case ETX:
            rxButton = rxBuffer[0];
            rxTilt = rxBuffer[1];
            rxPot = rxBuffer[2];
            rxA = rxBuffer[3];
            rxB = rxBuffer[4];
            rxC = rxBuffer[5];
            rxD = rxBuffer[6];
            rx_index = 0;
            break;
            
            default:
            rxBuffer[rx_index] = rx_char;
            rx_index++;
            break;
          }
        }
        else
        {
          Serial.println("Rx error");
        }
//        for (i = 0; i < 10; i++)
//        {
//          Serial.println(rx_bits[i]);
//        }
        for (i = 0; i < 10; i++)
        {
          rx_bits[i] = 0;
        }
        rx_state = 0;
        break;

      default:
        if (sensorValue >= 900)
        {
          rx_bits[rx_state / 10]++;
        }
        rx_state++;
        break;
    }
  }

}

void loop()
{
  readInputs();
  txChar();
  rxChar();
  writeOutputs();
}