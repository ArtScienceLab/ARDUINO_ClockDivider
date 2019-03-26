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

// The duration of a pulse in microseconds
const int PULSE_DURATION_IN_MICROS = 3160;//this makes the output more or less as long low as high

// The min and max allowed input frequency
const long MAX_FREQ = 48500;//Hz
const long MIN_FREQ = 47500;//Hz

//print debug statements
const boolean debug_output = false;

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
volatile boolean send_output_pulse = false;

//the counter incremented in the interrupt
int counter = 0;

int led_counter = 0;
int max_led_counter = 120;

//current time in microseconds
unsigned long time_in_microseconds;
unsigned long prev_time_in_microseconds;

void setup() {
  //only use serial if needed
  if(debug_output)
    Serial.begin(115200);
  
  //initialize output
  pinMode(led_pin, OUTPUT);
  pinMode(out_pin, OUTPUT);
  pinMode(input_clock_interrupt_pin, INPUT);

  //start the interrupt code
  attachInterrupt(digitalPinToInterrupt(input_clock_interrupt_pin), increment, RISING);
}

void loop() {
  if(send_output_pulse){
    //wait and set output port to low
    prev_time_in_microseconds = time_in_microseconds;
    time_in_microseconds = micros();
    delayMicroseconds(PULSE_DURATION_IN_MICROS);

    // Set the output port to low,
    // the output port is set to high in the interrupt.
    // Do not use slow digitalWrite() but fast register switching.
    // This means that this code is device DEPENDENT. Do not use on leonardo but only on UNO!
    PORTD &= ~_BV(PD3);
    
    send_output_pulse = false;

    // Less timing dependent 
    // calculate the input and output frequency
    unsigned long delta_in_mircros = time_in_microseconds-prev_time_in_microseconds;
    double delta = delta_in_mircros / (float) MAX_COUNTER;
    long intput_freq_hz = (long) (1.0 / delta * 1000000.0);
    float output_freq_hz = (float) (1.0 / delta_in_mircros * 1000000.0);

    //check the input, output frequency
    if(intput_freq_hz > MIN_FREQ && intput_freq_hz < MAX_FREQ ){
      //change led every second if freq in expected range
      max_led_counter = 120; 
    }else {
      //change led five times per second if freq out of range
      max_led_counter = 12;
    }

    // Set indicator led
    toggle_led();

    // Print serial output
    if(debug_output){
      Serial.print(intput_freq_hz );
      Serial.print("Hz => ");
      Serial.print(output_freq_hz);
      Serial.print("Hz");
      if(max_led_counter == 24) Serial.print(" out of expected range");
      Serial.println();
    }
  }
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
    // Set digital port 3 as quickly as possible
    // So do not use digitalWrite but direct register manupilation
    // otherwise it adds a couple of us.
    PORTD |= _BV(PD3);

    // set a boolean that is used in the loop to pull
    // the pin to LOW
    send_output_pulse = true;

    
    counter = 0;
  }
  
  counter++;
}
