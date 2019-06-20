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

//print debug statements
const boolean debug_output = true;

// pin with interrupt capabilities
// Attach the clock signal to this pin
const byte input_clock_interrupt_pin = 2; 

// The output signal (divided clock) 
const byte out_pin = 3;

// Status LED pin
const byte led_pin = 13;
// current status LED state
byte led_state = LOW;

// Send output pulse to the output pin?
volatile boolean check_timing = false;

//the counter incremented in the interrupt
int counter = 0;

int led_counter = 0;
int max_led_counter = 120;

//current time in microseconds
unsigned long time_in_microseconds;
unsigned long prev_time_in_microseconds;

const int max_nr_of_deltas = 240;
unsigned long deltas_in_micros[240];

int deltas_index = 0;
int print_counter = 0;

void setup() {
  //only use serial if needed
  if(debug_output)
    Serial.begin(115200);
  
  //initialize output
  pinMode(led_pin, OUTPUT);
  pinMode(out_pin, OUTPUT);
  //see here https://www.arduino.cc/en/Tutorial/DigitalPins
  pinMode(input_clock_interrupt_pin, INPUT_PULLUP);

  //start the interrupt code
  attachInterrupt(digitalPinToInterrupt(input_clock_interrupt_pin), increment, RISING);
}


void loop() {
  //120 times per second this boolean is true
  if(check_timing){
    check_timing = false;
    
    prev_time_in_microseconds = time_in_microseconds;
    time_in_microseconds = micros();
    
    // calculate the input and output frequency
    unsigned long delta_in_micros = time_in_microseconds-prev_time_in_microseconds;

    led_indicator(delta_in_micros);

    // Print serial output
    if(debug_output){

      //store the current delta in a list
      deltas_in_micros[deltas_index] = delta_in_micros;
      //increment and reset the counter if needed
      deltas_index++;
      if(deltas_index == max_nr_of_deltas){
        deltas_index = 0;
      }

      //print counter aims to print a message every second (120 pulses)
      print_counter++;
      if(print_counter==120){
        print_counter = 0;

        unsigned long max_delta = 0;
        unsigned long min_delta = 160000;
        for(int i = 0 ; i < max_nr_of_deltas ; i++){
          max_delta = max(max_delta,deltas_in_micros[i]);
          min_delta = min(min_delta,deltas_in_micros[i]);
        }

        Serial.print("min ");
        Serial.print(min_delta );
        Serial.print("µ, max ");
        Serial.print(max_delta);
        Serial.print("µ, diff ");
        Serial.print((max_delta - min_delta));
        Serial.print("µ, current ");
        Serial.print(delta_in_micros);
        Serial.print("µ");
        if(max_led_counter == 12) Serial.print(" out of expected range");
        Serial.println();
      }
    }
  }
}


void led_indicator(unsigned long current_delta){
  //check the input, output frequency
  if(current_delta > MIN_PERIOD_MICROS && current_delta < MAX_PERIOD_MICROS ){
    //change led every second if freq in expected range
    max_led_counter = 120;
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
    led_state = !led_state;
    digitalWrite(led_pin, led_state);
  }
}

// Interrupt routine, called +- 48000 times each second
void increment() {
  //check if the counter reached the max
  if(counter == MAX_COUNTER){
    // Set the output pin to HIGH
   digitalWrite(out_pin,HIGH);

    //reset the counter
    counter = 0;

    // set a boolean that is used in the loop to 
    // check timing
    check_timing = true;
    
  } else if(counter == HALF_COUNTER){
    // Set the output port to low,
    // 
    digitalWrite(out_pin,LOW);
  }

  //increment the counter
  counter++;
}
