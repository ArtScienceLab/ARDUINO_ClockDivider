// An interrupt driven clock divider for Arduino.
// 
// Attach the input clock to the interrupt pin 2 and the output (divided)
// clock to pin 3.
//
// The interrupt routine is kept as short as possible and simply sets a
// toggle. This toggle is used in the main loop. This means that there 
// should be no delays in the loop in order to react to the toggle. 
//
// Possible improvements: 
//     * use a gate to start the system
//     * use a rotary switch to select divider
//     * an input to start the clock, as a gate (how does mocap react?)

// Every time this numer of input pulses has passed, an output pulse is send
const int MAX_COUNTER = 400;//divider
const int HALF_COUNTER = 200;//when to set the signal to low

const int MAX_PERIOD_MICROS = 8333+8;//
const int MIN_PERIOD_MICROS = 8333-8;//

// Do not use an external 48kHz clock but generate the clock internally 
const boolean self_gen_48kHz_clock = true;
IntervalTimer internal_48kHz_clock;
// pin with interrupt capabilities
// Attach the clock signal to this pin
const byte interrupt_pin_48kHz_clock = 2; 

// Do not use an external 1Hz clock but generate the clock internally 
const boolean self_gen_1Hz_clock = false;
IntervalTimer internal_1Hz_clock;
const byte interrupt_pin_1Hz_clock = 3; 

//print debug statements
const boolean debug_output = false;

// The output signal (divided clock), stable 120Hz 
const byte out_pin_plain = 4;
// The output signal with the first of the 120 periods slightly shorter
const byte out_pin_enc = 5;

// Status LED pin
const byte led_pin = LED_BUILTIN;
// current status LED state
byte led_state = LOW;

// Send output pulse to the output pin?
volatile boolean check_timing = false;

//the counter incremented in the interrupt
int counter_48kHz_plain = 0;
int counter_48kHz_enc = 0;

volatile int counter_120Hz = 0;

int led_counter = 0;
int max_led_counter = 120;

//current time in microseconds
unsigned long time_in_microseconds;
unsigned long prev_time_in_microseconds;

unsigned long max_delta = 0;
unsigned long min_delta = 160000;

void setup() {
  //Only use serial if needed
  if(debug_output) Serial.begin(115200);
  
  //initialize output pins
  pinMode(led_pin, OUTPUT);
  pinMode(out_pin_plain, OUTPUT);
  pinMode(out_pin_enc, OUTPUT);
  
  //Either listen to a clock or generate it
  if(self_gen_48kHz_clock){
    internal_48kHz_clock.begin(tick_48kHz, 20.833333333333333333333333);
  }else{
    //see here https://www.arduino.cc/en/Tutorial/DigitalPins
    pinMode(interrupt_pin_48kHz_clock, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin_48kHz_clock), tick_48kHz, RISING);
  }

  //Either listen to a clock or generate it
  if(self_gen_1Hz_clock){
    internal_1Hz_clock.begin(tick_1Hz, 1000000);
  }else{
    //see here https://www.arduino.cc/en/Tutorial/DigitalPins
    pinMode(interrupt_pin_1Hz_clock, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin_1Hz_clock), tick_1Hz, CHANGE);
  }
}


void loop() {
  //120 times per second this boolean is true
  if(check_timing){
    check_timing = false;
    
    prev_time_in_microseconds = time_in_microseconds;
    time_in_microseconds = micros();

    //do not take in accout the longer and shorter first two periods
    
    // calculate the input and output frequency
    unsigned long delta_in_micros = time_in_microseconds-prev_time_in_microseconds;

    if(counter_120Hz != 0 && counter_120Hz != 1){
      max_delta = max(max_delta,delta_in_micros);
      min_delta = min(min_delta,delta_in_micros);
      led_indicator(delta_in_micros);
    }

    // Print serial output
    if(debug_output){
        if(counter_120Hz < 10)
          Serial.print(" ");
        if(counter_120Hz < 100)
          Serial.print(" ");
        Serial.print(counter_120Hz);
        Serial.print("# min ");
        Serial.print(min_delta );
        Serial.print("µ, max ");
        Serial.print(max_delta);
        Serial.print("µ, diff ");
        Serial.print((max_delta - min_delta));
        Serial.print("µ, current ");
        Serial.print(delta_in_micros);
        Serial.print("µ ");
        Serial.print(counter_48kHz_plain);
        Serial.print(" ");
        Serial.print(counter_48kHz_enc);
        Serial.print(" ");
        Serial.println();
    }
  }
}


void led_indicator(unsigned long current_delta){
  //check the input, output frequency
  if(current_delta > MIN_PERIOD_MICROS && current_delta < MAX_PERIOD_MICROS ){
    //change led every second if freq in expected range
    //max_led_counter = 120;
  }else {
    //change led five times per second if freq out of range
    max_led_counter = 12;
  }

  // Set indicator led
  toggle_led();
}

// Heartbeat led
void toggle_led(){
  led_counter++;
  if(led_counter >= max_led_counter){
    led_counter = 0;
    //change led heartbeat / error indicator
    //led_state = !led_state;
    //digitalWrite(led_pin, led_state);
  }
}

volatile int second_indicator_diff = 0;
volatile bool first_second = true;

void tick_1Hz(){
  second_indicator_diff = 1;
  if(first_second){
    //syncs counter and LTC messages
    counter_48kHz_plain = 0;
    counter_48kHz_enc = 0;
    counter_120Hz=0;
    first_second = false;
    max_led_counter = 120;
    max_delta = 0;
    min_delta = 160000;
    check_timing = true;
  }
  led_state = !led_state;
  digitalWrite(led_pin, led_state);
}

// Interrupt routine, called +- 48000 times each second
void tick_48kHz() {

  // The straight 120Hz output
  if(counter_48kHz_plain == MAX_COUNTER){
    digitalWrite(out_pin_plain,HIGH);
    counter_48kHz_plain = 0;
  }else if(counter_48kHz_plain == HALF_COUNTER){
    // Set the output port to low,
    digitalWrite(out_pin_plain,LOW);
  }
  
  // The encoded output is one sample longer for the first 
  // period after detecting the second, and one sample shorter the next
  // the other periods are in sync with the plain output
  if(counter_48kHz_enc == MAX_COUNTER + second_indicator_diff){
    // Set the output pin to HIGH
   digitalWrite(out_pin_enc,HIGH);

   //Where in the SMPTE LTC frame are we?
   counter_120Hz++;
   if(counter_120Hz == 120){
      counter_120Hz = 0;
   }

   // If the current period is one sample longer: make the
   // next one sample one sample shorter
   if(second_indicator_diff == 1){
     second_indicator_diff = -1;
     counter_120Hz = 0;
   }else if(second_indicator_diff == -1){
    // If the current period is one sample shorter: the next
    // one should have nominal lenght
    second_indicator_diff = 0;
   }
   //reset the counter
   counter_48kHz_enc = 0;
   
   //indicate that a 120Hz tick passed
   check_timing = true;
  } else if(counter_48kHz_enc == HALF_COUNTER){
    // Set the output port to low,
    digitalWrite(out_pin_enc,LOW);
  }

  //increment the sample counters
  counter_48kHz_plain++;
  counter_48kHz_enc++;
}
