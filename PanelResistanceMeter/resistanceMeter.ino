// jfMeter.ino
//
// Cole Kampa, Aina Zulkifli, Luiz Fernando Andrade, Jean-Francois Caron
//
// v2.0
//
// description: used for measuring pin-pin and straw-straw resistances in the completed panels
// hardware: at-08 buzzer, 150 Ohm reference resistor, LCD Shield,  2 Probes, 1 Button(Main)
//
//
// pin chart:
// buzzer: + pin to dig12, gnd to gnd --- OK
// ohmmeter: probe 1 (black banana jack) to +5V, probe 2 (red banana jack) to A0/ reference resistor:150 ohm
// main button: + pin to dig3, gnd to gnd.
//
//
//How to use it:
//Change position with: LEFT(-1) and RIGHT(+1) buttons and UP(95) and DOWN(48) buttons.
//Take measurement with: SELECT button or MAIN button(attached to red probe)
//Measurement order: Straw -> Wire

#include <avr/interrupt.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

// Define pin numbers
#define mainButton 3 // pin for the measurement button, PCINT19, in Port D
#define buzzerPin 12 // set to an output pin below
#define ADCPin 0  // no need to declare input/output for analog pins

// Uncomment this if you want to silence the beeps.
//#define DISABLEBEEP

//Initialize object for lcd
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// Number of straw/wire positions.
const int max_count = 95;

float strawMeasurements[96];
float wireMeasurements[96];
int counter = 0;// keeps track of the straw/wire index
bool at_wire = false; // keeps track of whether we are at a wire or a straw

// This flag is set when the mainButton gets pushed, in an interrupt.
volatile bool buttonPushed = false;

// These values are used to prevent high-rate button pushes.
unsigned long last_button_time = 0;
const unsigned long button_timeout = 500000; // microseconds

// This struct is made to hold the parameters of a beep.
struct Beep
{
  int freq; // Frequency of beep (Hz)
  int num;  // Number of beeps
  int dur;  // Duration of each beep (ms)
};

// Now I define three beeps for use in the program.
const Beep low_beep {
  261, 3, 250
};  // Beep for a measurement that is below the acceptable range.
const Beep high_beep {
  523, 3, 250
}; // Beep for a measurement that is above the acceptable range.
const Beep good_beep {
  349, 1, 1000
};// Beep that is within the acceptable range.


const int num_meas = 100; // number of measurements to make
// delay (milliseconds) between measurements...total measurement time should be ~ num_meas * meas_delay
const int meas_delay = 3;
const float resistance_ref = 150.4; // value of reference resistor in ohms
const float lower_limit = 145; // lower limit of 'acceptable' resistance range
const float upper_limit = 155; // upper limit of 'acceptable' resistance range

char inByte; // variable to store serial input from user or python code

void setup() {
  //Set the lcd screen
  lcd.begin(16, 2);
  lcd.print(F("Mu2e Panel QC"));
  lcd.setCursor(0, 1);
  lcd.print(F("ResistanceTester"));
  lcd.cursor();
  lcd.blink();

  // Continue setting up while LCD welcome screen is shown.

  // Set pins for buzzer and probe button.
  pinMode(buzzerPin, OUTPUT); // set buzzer (digital pin 12) to an output pin
  pinMode(mainButton, INPUT_PULLUP);

  // Enable "pin-change interrupt" on pin mainButton.
  cli(); // Disable interrupts.
  PCICR |= 0b00000100;
  PCMSK2 |= 0b00001000;
  sei();// Re-enable interrupts.

  // Initialize the measurement arrays.
  for (int i = 0; i < 96; i++) {
    wireMeasurements[i] = 0;
    strawMeasurements[i] = 0;
  }

  Serial.begin(115200); // start the USB serial connection
  Serial.println(F("# Position, wire/straw, ADC values..., resistance, PASS?"));

  // Delay a little bit, so the welcome screen is visible to humans.
  delay(1000);
  lcd.clear();
  draw_LCD();
}

void loop() {
  // This flag can be set in multiple places to make a measurement happen.
  bool do_measurement = false;

  // buttonPushed gets set in the interrupt when the probe button is pushed.
  if (buttonPushed)
  {
    // Only react to buttonPushed if it hasn't been pushed in button_timeout.
    unsigned long now = micros();
    if ((now - last_button_time) > button_timeout)
    {
      do_measurement = true;
      last_button_time = now;
    }
  }

  if (Serial.available() > 0)
  {
    inByte = Serial.read(); //reads the incoming data, an 'r' will trigger a measurement.
    if (inByte == 'r')
    {
      do_measurement = true;
    }
  }

  // Read built-in buttons on the LCD shield.
  uint8_t buttons = lcd.readButtons();

  if (buttons) // check to see if LCD buttons were pressed.
  {
    // Only react to buttons if they haven't been pushed in button_timeout.
    unsigned long now = micros();
    if ((now - last_button_time) > button_timeout)
    {
      if (buttons & (BUTTON_UP | BUTTON_DOWN | BUTTON_LEFT | BUTTON_RIGHT))
      {
        at_wire = false;
      }
      if (buttons & BUTTON_UP) {
        counter = 95;
      }
      if (buttons & BUTTON_DOWN) {
        counter = 48;
      }
      if (buttons & BUTTON_LEFT) {
        // Decrement counter, but wrap back up to max_count from zero.
        counter = (counter - 1) % max_count;
        counter = (counter < 0) ? max_count : counter;
      }
      if (buttons & BUTTON_RIGHT) {
        // Increment counter, but wrap back to zero after max_count.
        counter = (counter + 1) % max_count;
      }
      if (buttons & BUTTON_SELECT)
      {
        do_measurement = true;
      }
      lcd.clear();
      draw_LCD();
      last_button_time = now;
    }
  }


  if (do_measurement)
  {
    // All the action happens in manage_measurement
    manage_measurement();
    lcd.clear();
    draw_LCD();
    if (at_wire)
    { // Give enough time for the users to see the screen before it changes to the next counter.
      delay(1000);
    }

    // at_wire is false(0) or true(1), so this only increments by 1 if at_wire is true.
    counter = (counter + at_wire) % max_count;
    at_wire = !at_wire; // Toggle at_wire
    lcd.clear();
    draw_LCD();

    buttonPushed = false;
  }
} //ends the loop


unsigned long measure_sequence()
{
  // we must use long rather than int because int has a maximum value
  // of 65,536. 100 measurements could yield a value of up to 102,300!
  unsigned long adc_total = 0;

  int i = 0;
  // Do num_meas measurements and sum them into a total.
  while (i < num_meas) {
    unsigned int rawADC = analogRead(ADCPin);
    Serial.print(rawADC); Serial.print(", "); // print every individual measurement to Serial.
    adc_total += rawADC;
    i++;
    delay(meas_delay);
  }
  return adc_total;
}

void manage_measurement()
{
  Serial.print(counter);
  Serial.print(F(", "));
  Serial.print(at_wire);
  Serial.print(F(", "));

  unsigned long adc_total = measure_sequence();
  float resistance = resistance_ref * (1023 / (1.0 * adc_total / num_meas) - 1);

  Serial.print(resistance);
  Serial.print(F(", "));

  // Store measurements into our arrays.
  if (at_wire == false) {
    strawMeasurements[counter] = resistance;
  }
  else if (at_wire == true) {
    wireMeasurements[counter] = resistance;
  }

  // Determine which beep to be used, and print status
  if (resistance < lower_limit) { // low resistance
    beep(low_beep);
    Serial.print(F("LOW"));
  }
  else if (resistance > upper_limit) { // high resistance
    beep(high_beep);
    Serial.print(F("HIGH"));
  }
  else { // if we make it this far into the loop, we know the resistance is 'acceptable'
    beep(good_beep);
    Serial.print(F("PASS"));
  }
  Serial.println();
}

void beep(const Beep& the_beep) {
  // The #ifndef block here will disable this code if the DISABLEBEEP macro is set
#ifndef DISABLEBEEP
  for (int i = 0; i < the_beep.num; i++) { // loop through however many times we want to beep
    tone(buzzerPin, the_beep.freq); // turn the buzzer on at a given frequency
    delay(the_beep.dur); // keep the buzzer on for the beep duration (in milliseconds)
    noTone(buzzerPin); // turn the buzzer off
    delay(the_beep.dur); // keep the buzzer off for the beep duration
  }
#endif
}

void draw_LCD()
{
  lcd.setCursor(0, 0);
  lcd.print(F("P :"));
  lcd.print(counter);
  lcd.setCursor(0, 1);
  lcd.print(F("S: "));
  lcd.print(strawMeasurements[counter]);
  lcd.setCursor(8, 1);
  lcd.print(F("W: "));
  lcd.print(wireMeasurements[counter]);
  // We leave the flashing cursor on top of the S or W depending on at_wire.
  lcd.setCursor(8 * at_wire, 1);
}

// Interrupt service routine for mainButton push.
ISR(PCINT2_vect)
{
  // This interrupt fires on any change of the button input (LOW->HIGH or HIGH->LOW),
  // but we only want to react when it was a HIGH->LOW transition.
  if (digitalRead(3) == LOW)
  {
    buttonPushed = true;
  }
}

