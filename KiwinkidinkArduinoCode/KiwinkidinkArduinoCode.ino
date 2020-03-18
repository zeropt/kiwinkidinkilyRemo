#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define ROLL_PIN      11 // roll channel from RC receiver
#define PITCH_PIN     10 // pitch channel from RC receiver
#define THROTTLE_PIN   9 // throttle channel from RC receiver
#define YAW_PIN        8 // yaw channel from RC receiver

#define MAX_SPEED             127 // max motor speed
#define PULSE_WIDTH_DEADBAND   25 // pulse width difference from 1500 us (microseconds) to ignore (to compensate for control centering offset)
#define PULSE_WIDTH_RANGE     350 // pulse width difference from 1500 us to be treated as full scale input (for example, a value of 350 means
                                  //   any pulse width <= 1150 us or >= 1850 us is considered full scale)

int final_active = 0;
int final_roll = 0;
int final_pitch = 0;
int final_throttle = 0;
int final_yaw = 0;

#define LED_PIN 7
#define PIX_NUM 12

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

int mode = 0;


void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  
  Serial.begin(9600);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  int active = 0;
  int roll = pulseIn(ROLL_PIN, HIGH);
  int pitch = pulseIn(PITCH_PIN, HIGH);
  int throttle = pulseIn(THROTTLE_PIN, HIGH);
  int yaw = pulseIn(YAW_PIN, HIGH);

  if (roll > 0 && pitch > 0 && throttle > 0 && yaw > 0) {
    // all RC signals are good

    // RC signals encode information in pulse width centered on 1500 us (microseconds); subtract 1500 to get a value centered on 0
    roll -= 1500;
    pitch -= 1500;
    throttle -= 1500;
    yaw -= 1500;

    // apply deadband
    if (abs(roll) <= PULSE_WIDTH_DEADBAND)
      roll = 0;
    if (abs(pitch) <= PULSE_WIDTH_DEADBAND)
      pitch = 0;
    if (abs(throttle) <= PULSE_WIDTH_DEADBAND)
      throttle = 0;
    if (abs(yaw) <= PULSE_WIDTH_DEADBAND)
      yaw = 0;

    roll = ((long)roll * MAX_SPEED / PULSE_WIDTH_RANGE);
    pitch = ((long)pitch * MAX_SPEED / PULSE_WIDTH_RANGE);
    throttle = ((long)throttle * MAX_SPEED / PULSE_WIDTH_RANGE);
    yaw = ((long)yaw * MAX_SPEED / PULSE_WIDTH_RANGE);

    active = 100;
    roll = constrain(roll, -MAX_SPEED, MAX_SPEED);
    pitch = constrain(pitch, -MAX_SPEED, MAX_SPEED);
    throttle = constrain(throttle, -MAX_SPEED, MAX_SPEED);
    yaw = constrain(yaw, -MAX_SPEED, MAX_SPEED);
  } else {
    // at least one RC signal is not good
    active = 0;
    roll = 0;
    pitch = 0;
    throttle = 0;
    yaw = 0;
  }

  final_active = active;
  final_roll = roll;
  final_pitch = pitch;
  final_throttle = throttle;
  final_yaw = yaw;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  byte buf[] = {(byte)(final_active), (byte)(final_roll+127), (byte)(final_pitch+127), (byte)(final_throttle+127), (byte)(final_yaw+127)};
  Wire.write(buf, 5); // respond with message of 5 bytes
  // as expected by master
  Serial.println(buf[2]);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  int x;
  while (1 < Wire.available()) { // loop through all but the last
    x = Wire.read(); // receive byte as an int
  }
  mode = x;
}
