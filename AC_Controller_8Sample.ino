#include <IRremote.h>
#include <arduinoFFT.h>

#define butPin 4

// -------LIGHTS SHOW-----------

#define sensorPin0 A0
#define sensorPin1 A1
#define potPin0 A2
#define potPinThreshold A3
#define potPinAmplifier A4
#define potPinDelay A5
#define DOUT1 8
#define DOUT2 9   // pwm
#define DOUT3 10  // pwm
#define DOUT4 11  // pwm
#define DOUT5 12 
#define DOUT6 13
#define SAMPLES 16             //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC

// -------IR CONTROLLER-----------

#define LIGHTS_MODE     2
#define APPLIANCE_MODE  1

#define RELAY_1   8
#define RELAY_2   9
#define RELAY_3   10
#define RELAY_4   11
#define RELAY_5   12
#define RELAY_6   13
#define KEY_1     506
#define KEY_2     502
#define KEY_3     498
#define KEY_4     494
#define KEY_5     490
#define KEY_6     486
#define KEY_POWER 442

// -------LIGHTS SHOW-----------

arduinoFFT FFT = arduinoFFT();

int brightness_divisor = 250;

short num_outputs = 3;
 
unsigned int sampling_period_us;
unsigned long microseconds; // current time since the Arduino board started 
 
double vReal[SAMPLES];
double vImag[SAMPLES];

int potThreshold = 0;     // Between 500 and 10,000
float potAmplifier = 2;   // Between 1 and 8?
int potDelay = 20;        // Between 1 and 400

double total = 0;
double fullMultiplier = 0;

double module1 = 0;
double module2 = 0;
double module3 = 0;
double module4 = 0;

int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;

// -------IR CONTROLLER-----------

const int IR_RECEIVE_PIN = 5;

decode_results results;

bool relay_1 = false, relay_2 = false, relay_3 = false, relay_4 = false, relay_5 = false, relay_6 = false;

int operation_mode = LIGHTS_MODE;

double convBrightness(double b) {
  double c = b; // The maximun intensity value in theory is 31713 (but we are never having the volume that high)
  if( c < (analogRead(potPinThreshold))*.75) c = 0;
  else c = 1;
  return c;
}

void set_brightness(int delta_brightness) {
  brightness_divisor += delta_brightness;
}

void _switch_relay(int relay_pin, bool *relay) {
  if (not *relay) {
    *relay = true;
    digitalWrite(relay_pin, HIGH);
    return;
  } else {
    *relay = false;
    digitalWrite(relay_pin, LOW);
    return;
  }
}

void switch_relay(int relay_pin) {
  if (relay_pin == RELAY_1) {
    _switch_relay(relay_pin, &relay_1);
  }
  if (relay_pin == RELAY_2) {
    _switch_relay(relay_pin, &relay_2);
  }
  if (relay_pin == RELAY_3) {
    _switch_relay(relay_pin, &relay_3);
  }
  if (relay_pin == RELAY_4) {
    _switch_relay(relay_pin, &relay_4);
  }
  if (relay_pin == RELAY_5) {
    _switch_relay(relay_pin, &relay_5);
  }
  if (relay_pin == RELAY_6) {
    _switch_relay(relay_pin, &relay_6);
  }
}

bool do_command(long IR_code){
  if (IR_code == KEY_1) {
    switch_relay(RELAY_1);
    return true;
  } else if (IR_code == KEY_2) {
    switch_relay(RELAY_2);
    return true;
  } else if (IR_code == KEY_3) {
    switch_relay(RELAY_3);
    return true;
  } else if (IR_code == KEY_4) {
    switch_relay(RELAY_4);
    return true;
  } else if (IR_code == KEY_5) {
    switch_relay(RELAY_5);
    return true;
  } else if (IR_code == KEY_6) {
    switch_relay(RELAY_6);
    return true;
  } else if (IR_code == KEY_POWER) {
    set_all_leds(LOW);
    relay_1 = relay_2 = relay_3 = relay_4 = relay_5 = relay_6 = false;
  } else if (IR_code == 0) {
      return true;
  }
  return false;
}

void set_all_leds(int value) {
  digitalWrite(DOUT1, value);
  digitalWrite(DOUT2, value);
  digitalWrite(DOUT3, value);
  digitalWrite(DOUT4, value);
  digitalWrite(DOUT5, value);
  digitalWrite(DOUT6, value);
}

void blink_all(int pins[], int arrSize, int duration) {
  for (int i = 0; i<arrSize; i++) {
    digitalWrite(pins[i], LOW);
  }
  delay(duration);
  for (int i = 0; i<arrSize; i++) {
    digitalWrite(pins[i], HIGH);
  }
  delay(duration);
  for (int i = 0; i<arrSize; i++) {
    digitalWrite(pins[i], LOW);
  }
}

void printToGraph(int val) {
  Serial.print(val);
  Serial.print(' ');
}

void setup(){

  Serial.begin(115200); // open the serial port at 115200 bps:
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  pinMode(sensorPin0, INPUT);
  pinMode(sensorPin1, INPUT);
  pinMode(potPin0, INPUT);
  pinMode(potPinThreshold, INPUT);
  pinMode(potPinAmplifier, INPUT);
  pinMode(potPinDelay, INPUT);
  pinMode(butPin, INPUT_PULLUP);

  // Lights outputs
  pinMode(DOUT1, OUTPUT);
  pinMode(DOUT2, OUTPUT);
  pinMode(DOUT3, OUTPUT);
  pinMode(DOUT4, OUTPUT);
  pinMode(DOUT5, OUTPUT);
  pinMode(DOUT6, OUTPUT);
  
  // Relay outputs
  pinMode(13, OUTPUT);  // Digital, Relay 6
  pinMode(12, OUTPUT);  // Digital, Relay 5
  pinMode(11, OUTPUT);  // Analog, Relay 4
  pinMode(10, OUTPUT);  // Analog, Relay 3
  pinMode(9, OUTPUT);   // Analog, Relay 2
  pinMode(8, OUTPUT);   // Digital, Relay 1

  // Serial and IR Receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
}

void loop(){
  if (digitalRead(butPin) == LOW) {
    relay_1 = relay_2 = relay_3 = relay_4 = relay_5 = relay_6 = false;
    int pins[6] = {DOUT2, DOUT3, DOUT4, DOUT1, DOUT5, DOUT6};
    for (int i = 0; i<6; i++) {
      digitalWrite(pins[i], LOW);
    }

    if (operation_mode == LIGHTS_MODE) {
      // 1 blink
      blink_all(pins, 1, 75); 
      blink_all(pins, 1, 75); 
      delay(1000);
      if (digitalRead(butPin) == HIGH) {
        num_outputs = 1;
        return;
      }
  
      // 2 blinks
      blink_all(pins, 2, 75); 
      blink_all(pins, 2, 75); 
      delay(1000);
      if (digitalRead(butPin) == HIGH) {
        num_outputs = 2;
        return;
      }
  
      // 3 blinks
      blink_all(pins, 3, 75);
      blink_all(pins, 3, 75);
      delay(1000);
      if (digitalRead(butPin) == HIGH) {
        num_outputs = 3;
        return;
      }
  
      // 4 blinks
      blink_all(pins, 4, 75);
      blink_all(pins, 4, 75);
      delay(1000);
      if (digitalRead(butPin) == HIGH) {
        num_outputs = 4;
        return;
      }
    }
    
    // Long Blink
    blink_all(pins, 6, 1000);
    relay_1 = relay_2 = relay_3 = relay_4 = relay_5 = relay_6 = false;
    if (digitalRead(butPin) == HIGH) {
      operation_mode += 1;
      if (operation_mode > 2) operation_mode = 1;
      Serial.print("Operation mode: ");
      Serial.println(operation_mode);
      IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
      return;
    }
  }
  
  if (operation_mode == LIGHTS_MODE) {
    for(int i=0; i<SAMPLES; i++) {
      microseconds = micros();    //Overflows after around 70 minutes
      vReal[i] = analogRead(sensorPin0) * analogRead(potPin0)/300; // Uses distanced mic by default. Modifies by pot
      vImag[i] = 0;
      
      while(micros() < (microseconds + sampling_period_us)){ // We take sampling_period_us microseconds to read the next sample
      }
    }
  /*FFT*/
  
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    
    val1 = 0;
    val2 = 0;
    val3 = 0;
    val4 = 0;

    total = 0;
    
    module1 = 0;
    module2 = 0;
    module3 = 0;
    module4 = 0;
    /* MODULES of the frequency vectors computed */
    
    for(int i=1; i < SAMPLES/2; i++) {
      double a = pow(vReal[i], 2);
      double b = pow(vImag[i], 2);

      float sqrtVal = sqrt(a+b);
      
      
      // Now we assign each frequency value to its corresponding light range:
      if (num_outputs == 1) {
        if (i > 2) module2 += sqrtVal;
      } else if (num_outputs == 2) {
        if(i >= (2) && i <= (4)) module2 += sqrtVal;
        else if(i >= (6) && i <= (8)) module3 += sqrtVal;
      } else if (num_outputs == 3) {        
        if(i >= 2 && i <= 2) module2 += sqrtVal;
        else if(i >= 4 && i <= 4) module3 += sqrtVal;
        else if(i >= 7 && i <= 7) module4 += sqrtVal;
      } else if (num_outputs == 4) {
        if(i >= 2 && i <= 2) module1 += sqrtVal;
        else if(i >= 4 && i <= 4) module2 += sqrtVal;
        else if(i >= 6 && i <= 6) module3 += sqrtVal;
        else if(i >= 7 && i <= 8) module4 += sqrtVal;
      }
    }

    // REDUCE ALL MODULES TO MORE MANAGEABLE NUMBERS, SO THEY CAN BE RAISED TO 6th POWER OR MORE!


    potAmplifier = float(analogRead(potPinAmplifier)) / 200.0 + 1; // will be between 1 and 3
//    Serial.println(potAmplifier);

    total = module1 + module2 + module3 + module4;
    fullMultiplier = pow(module1, potAmplifier) + pow(module2, potAmplifier) + pow(module3, potAmplifier) + pow(module4, potAmplifier);

    module1 = pow(module1, potAmplifier)*(total/fullMultiplier);
    module2 = pow(module2, potAmplifier)*(total/fullMultiplier);
    module3 = pow(module3, potAmplifier)*(total/fullMultiplier);
    module4 = pow(module4, potAmplifier)*(total/fullMultiplier);


    printToGraph(module2);
    printToGraph(module3);
    printToGraph(module4);
    Serial.println(' ');
    
    val2 = 255 * convBrightness(module2);
    digitalWrite(DOUT2, val2);
    if (num_outputs >= 2) {
      val3 = 255 * convBrightness(module3);
      digitalWrite(DOUT3, val3);
    }
    if (num_outputs >= 3) {
      val4 = 255 * convBrightness(module4);
      digitalWrite(DOUT4, val4);
    }
    if (num_outputs >= 4) {
      val1 = 255 * convBrightness(module1);
      digitalWrite(DOUT1, val1);
    }
  
    delay(analogRead(potPinDelay)+1);

    // delay,threshold,sensitivity

  } else if (operation_mode == APPLIANCE_MODE) {
    if (IrReceiver.decode()){
      Serial.println("Receiving signal");
      Serial.println(IrReceiver.decodedIRData.decodedRawData);
      IrReceiver.resume(); // Enable receiving of the next value
      bool response = do_command(IrReceiver.decodedIRData.decodedRawData);
      if (not response) {
        Serial.println("Key not bound");
      }
    }
  }
}
