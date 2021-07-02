//
// JLC v1.0  2021/07/02  Data acquisition at a fixed rate
//
//

#include <assert.h>

/*
  port D : digital pins #0 to #7  on Arduino UNO board
  port B : digital pins #8 to #13 on Arduino UNO board
    (bits 6 & 7 of port B map to the crystal pins and are not available)
  
  PORTD,2 -> D2   PORTB,0 -> D8
  PORTD,3 -> D3   PORTB,1 -> D9
  PORTD,4 -> D4   PORTB,2 -> D10
  PORTD,5 -> D5   PORTB,3 -> D11
  PORTD,6 -> D6   PORTB,4 -> D12
  PORTD,7 -> D7   PORTB,5 -> D13
*/

///////////////////////////////////////////////////
///////      Pins Layout          /////////////////
///////////////////////////////////////////////////

// Arduino UNO:  Timer0 drives PWM on pin 5 & 6
// Arduino Mega: Timer0 drives PWM on pin 4 & 13

#define pinPWM    5        // Timer0 of ARDUINO drives PWM on pin 5 & 6
#define pinSamplingTop 4   // Output a top to check sampling period with an oscilloscope

//
// global variables used by the timer/counter
//
uint16_t counterPeriod_musec;           // the period of the timer counter in micro-secondes.
volatile uint16_t COUNTER;              // counters for periods
volatile uint16_t TARGET_COUNT;                  // period to count for motor
volatile int N = 0;                              // sample number

//
// miscellanous global variables:
//
byte debug;           // the debug level (0 ... 20)
const float sampling_period_ms = 10;     // the desired sampling period
uint16_t sampling_count;                 // the corresponding timer count

//
// sampling stuff
//
const int nb_sample = 500;
int data[nb_sample];

#define CLR(x,y) (x &= (~(1 << y)))
#define SET(x,y) (x |= (1 << y))
#define LF String("\n")

// 
// the interrup fucntion associated with timer1
//
ISR(TIMER1_COMPA_vect)
{
  if (TARGET_COUNT == 0)
  {
    COUNTER = 0;
  }
  else
  {
    COUNTER++;
    if (COUNTER >= TARGET_COUNT)
    { 
      // generate a 10 usec top to check sampling period:
      SET(PORTD, pinSamplingTop);         
      delayMicroseconds(10);
      CLR(PORTD, pinSamplingTop);
      COUNTER = 0;

      // output the PWM signe:
      // todo....

      // write the read value to the data table:
      if (N < nb_sample) data[N] = analogRead(A0);
      N +=1;
    }
  }
}

void setup()
{  
  Serial.begin(115200);
  
  //
  // To control the stepper motors we use Timer1 interrupt running at 25Khz.
  //
  
  // STEPPER MOTORS INITIALIZATION
  
  cli(); // desactivate global interruption
  
  // TIMER1 CTC MODE (Clear Timer on Compare Match Mode)
  TCCR1A &= ~(1 << WGM10);   // set bit WGM10 to 0 in TCCR1A
  TCCR1A &= ~(1 << WGM11);   // set bit WGM11 to 0 in TCCR1A 
  TCCR1B |=  (1 << WGM12);   // set bit WGM12 to 1 in TCCR1B
  TCCR1B &= ~(1 << WGM13);   // set bit WGM13 to 0 in TCCR1B

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0); 
  TCCR1A &= ~(3 << COM1B0); 
  
  // Set the timer pre-scaler : we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B &= ~(0x07 << CS10);  // clear CS12, CS11 & CS10
  TCCR1B |=  (2 << CS10);     // set "CS12 CS11 CS10" to "0 1 0"

  // initalize counter value
  TCNT1 = 0;      

  // OCRIA with frequency 2 MHz:
  // 1000 ->   2 kHz : 0.500  ms
  //  500 ->   4 kHz : 0.250  ms
  //  250 ->   8 kHz : 0.125  ms
  //  125 ->  16 kHz : 0.0625 ms
  //  100 ->  20 kHz : 0.050  ms
  //   80 ->  25 kHz : 0.040 ms
  //   50 ->  40 kHz : 0.025  ms
  
  OCR1A = 100;                    // 
  counterPeriod_musec = OCR1A/2;  // OCR1A / 2 MHz -> counter period in micro-sec = 0.05 ms (50 usec)
  sampling_count = uint16_t(sampling_period_ms/(counterPeriod_musec*1.e3));

  // Enable Timer1 interrupt
  TIMSK1 |= (1 << OCIE1A);  

  sei(); // Activate global interrupt

  // IN/OUT PINS 
  pinMode(pinSamplingTop,OUTPUT);  // ENABLE MOTOR 
  pinMode(pinPWM,  OUTPUT);        // PWM to drive the actutator

  debug = 1;
  if (debug)
  {
    Serial.print("\n counter period     [micro sec.]: "); Serial.print(counterPeriod_musec);
    Serial.print("\n sampling_period_ms [milli sec.]: "); Serial.print(sampling_period_ms);
    Serial.print("\n sampling_count                 : "); Serial.print(sampling_count);
  }
  
  TARGET_COUNT  = 0;
  COUNTER       = 0;
  
  delay(1000);
}

///////////////////
// The Main loop //
///////////////////

bool done = false;

void loop() 
{     
  // start time counting
  if ( ! done) 
  {
    N= 0;
    TARGET_COUNT = sampling_count;
    done = true;
  }

  if (N == nb_sample)
  {
    Serial.print("End of sampling 1000 values\n");
    Serial.print("Ts:"); Serial.print(sampling_period_ms); Serial.print(" ms\n");
    for (int i=0; i < nb_sample; i++)
    {
      Serial.println(data[i]);
    }
    Serial.println("-1");
    
    // stop the loop:
    while(true){;};
  }

}
