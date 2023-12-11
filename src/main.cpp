/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-neopixel-led-strip
 */
#include <Arduino.h>
#include "BluetoothSerial.h"

#include <Adafruit_NeoPixel.h>

#define PIN_NEO_PIXEL 14 // The ESP32 pin GPIO14 connected to NeoPixel
#define NUM_PIXELS 24    // The number of LEDs (pixels) on NeoPixel LED strip

TaskHandle_t Task1;
TaskHandle_t Task2;
BluetoothSerial SerialBT;

Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);
const int soundPin = 34;
int VoltageAnalogInputPin = 34; // Which pin to measure voltage Value (Pin A0 is
                                // reserved for button function)
const int ledPin = 2;
static int lastState = HIGH; // the previous state from the input pin
static int currentState;     // the current reading from the input pin
const int interrupUs = 1000000;
static bool rstCntSound = false;
const int lmtLvl = 60;
const int soundLvl[lmtLvl] = {254, 249, 244, 239, 234, 229, 224, 219, 214, 209, 204, 199, 194, 189, 184, 179, 174, 169, 164, 159, 154, 149, 144, 139, 134, 129, 124, 119, 114, 109, 104, 99, 94, 89, 84, 79, 74, 69, 64, 59, 54, 49, 44, 39, 34, 29, 24, 19, 14, 9, 4};

void colorWipe(uint32_t c, uint8_t wait);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);
void theaterChase(uint32_t c, uint8_t wait);
void theaterChaseRainbow(uint8_t wait);
void iron_normal(uint16_t d, int time);
void whiteOverRainbow(int whiteSpeed, int whiteLength);
void pulseWhite(uint8_t wait);
void rainbowFade2White(int wait, int rainbowLoops, int whiteLoops);
void fillGradient(uint32_t color1, uint32_t color2, int wait);
uint32_t Wheel(byte WheelPos);
void theaterChaseNoDelay(uint32_t color, int wait);
void rainbowNodelay(uint8_t wait);
void theaterChaseRainbowNoDelay(uint8_t wait);
void iron_man2();
uint16_t readSound();
void Task1code(void *pvParameters);
void Task2code(void *pvParameters);
void callRedLaser(uint8_t brig, uint8_t w);
void callOrgLaser(uint8_t brig, uint8_t w);
void callCyanLaser(uint8_t brig, uint8_t w);

static int red = 0;
static int green = 0;
static int blue = 240;
static uint8_t Brig = 10;
static bool revBrig = false;
static bool revRed = false;
static bool revGreen = false;
static bool soundCal = false;
static int hue = 35000;
static int saturation = 10;
static int value = 10;
static u_int8_t cntSdSpeedSave = 0;
static u_int8_t cntSdSpeed = 0;
static u_int8_t speed = 0;
static int stepMode = 0;
static int rpMode = 0;
char buffer[64]; // buffer must be big enough to hold all the message

static uint32_t blueScale[6] = {strip.Color(0, 0, 255), strip.Color(0, 64, 255), strip.Color(51, 102, 255), strip.Color(102, 140, 255), strip.Color(153, 179, 255), strip.Color(204, 217, 255)};

hw_timer_t *My_timer = NULL;
u_int8_t wait_sound = 0;
u_int8_t wait_save = 0;
String message = "";
char incomingChar;
static u_int8_t divv = 10;
const u_int8_t w_Max = 20;

enum stateMode
{
  music,
  laser,
  repos,
  lamp,
  idle,
};
static enum stateMode state = music;

void IRAM_ATTR onTimer()
{
  if (soundCal)
  {
    Serial.printf("The sound has been detected = %d , divv = %d \n", cntSdSpeed, cntSdSpeed / divv);
    SerialBT.printf("The sound has been detected = %d , divv = %d\n", cntSdSpeed, cntSdSpeed / divv);
  }
  rstCntSound = true;
  digitalWrite(ledPin, !digitalRead(ledPin));
}

void setup()
{
  strip.begin(); // initialize NeoPixel strip object (REQUIRED)
  strip.setBrightness(20);

  strip.show(); // Initialize all pixels to 'off'
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  Serial.write("Hello world \n");

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer);
  // analogReadResolution(10);
  // adcAttachPin(acPin);
  pinMode(soundPin, INPUT_PULLUP);

  xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      0,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */
}
// Task2code: blinks an LED every 700 ms
void Task1code(void *pvParameters)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  SerialBT.begin("ESP32test"); // Bluetooth device name

  for (;;)
  {
    speed = readSound();
    if (rstCntSound)
    {
      // if (speed >= 65 && speed <= 50) {
      // } else if (speed >= 49 && speed <= 35) {
      //   wait_sound = 0;
      // } else if (speed >= 34 && speed <= 20) {
      //   wait_sound = 20;
      // } else if (speed >= 19 && speed <= 4) {
      //   wait_sound = 30;
      // } else if (speed >= 4 && speed <= 0) {
      //   wait_sound = 40;
      // } else {
      //   wait_sound = 50;
      // }

      if (speed / divv > w_Max)
      {
        wait_save = 0;
      }
      else
      {
        wait_save = w_Max - speed / divv;
      }
      // printf("speed = %d \n", speed);
      // printf("wait_sound = %d \n", wait_sound);

      cntSdSpeed = 0;
      rstCntSound = false;
      // printf("wait_sound = %d \n",wait_sound);
    }

    if (SerialBT.available())
    {
      char bt = SerialBT.read();

      switch (bt)
      {
      case 'm':
        state = music;
        Serial.printf("Mode changed to MUSIC\n");
        break;

      case 'n':
        state = repos;
        Serial.printf("Mode changed to REPOS\n");
        break;
      case 'l':
        state = laser;
        Serial.printf("Mode changed to laser\n");
        break;
      case 'c':
        strip.updateLength(12);
        strip.clear();
        strip.show();
        Serial.printf("12 leds\n");

        break;

      case 'h':
        strip.updateLength(24);
        strip.clear();
        strip.show();
        Serial.printf("24 leds\n");
        break;

      case 'f':
        state = lamp;
        Serial.printf("flashgrip \n");
        break;

      case '+':
        Brig = Brig + 10;
        strip.setBrightness(Brig);
        printf("Brightness = %d \n", Brig);
        SerialBT.printf("Brightness = %d \n", Brig);
        break;
      case '-':
        Brig = Brig - 10;
        strip.setBrightness(Brig);
        printf("Brightness = %d \n", Brig);
        SerialBT.printf("Brightness = %d \n", Brig);
        break;
      case '*':
        divv++;
        printf("divv = %d \n", divv);
        SerialBT.printf("divv = %d \n", divv);
        break;
      case '!':
        divv--;
        printf("divv = %d \n", divv);
        SerialBT.printf("divv = %d \n", divv);
        break;
      case 'd':
        soundCal = !soundCal;
        printf("sound cal = %d \n", soundCal);
        SerialBT.printf("sound cal = %d \n", soundCal);

        break;
      default:
        // Handle other messages if needed
        break;
      }
    }
  }
}

void loop()
{
  uint8_t w;
  wait_sound = wait_save;
  switch (state)
  {
  case laser:

    for (int j = 0; j < 1; j++)
    {
      callRedLaser(45, 40);
      callOrgLaser(45, 40);
    }

    for (int j = 0; j < 3; j++)
    {
      callRedLaser(75, 10);
      callOrgLaser(75, 10);
    }
    for (int j = 0; j < 5; j++)
    {
      callRedLaser(80, 5);
      callOrgLaser(80, 5);
    }
    Serial.printf("laser For mode case  \n");

    callCyanLaser(255, 0);
    delay(2000);
    state = repos;

    break;
  case repos:
    iron_man2();
    Serial.printf("repos mode \n");
    //  SerialBT.printf("repos mode \n");
    // state = idle;
    break;

  case music:
    // SerialBT.printf("Music mode wait_sound = %d \n", wait_sound);
    switch (stepMode)
    {
    case 0:
      if (wait_sound > 5)
      {
        wait_sound = 5;
      }
      printf("mode rainbow rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode rainbow rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      rainbow(wait_sound);
      rpMode++;
      break;
    case 1:
      if (wait_sound > 5)
      {
        wait_sound = 5;
      }
      printf("mode rainbowCycle rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode rainbowCycle rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      rainbowCycle(wait_sound);
      rpMode++;
      break;
    case 2:
      if (wait_sound > 5)
      {
        wait_sound = 5;
      }
      printf("mode theaterChaseRainbow rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChaseRainbow rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChaseRainbow(wait_sound);
      rpMode++;
      break;
    case 3:
      printf("mode theaterChase blue rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChaseRainbow rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChase(strip.Color(0, 0, 127), wait_sound); // Blue
      rpMode++;

      break;
    case 4:
      printf("mode theaterChase Red rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChase Red rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChase(strip.Color(127, 0, 0), wait_sound); // Red
      rpMode++;

      break;
    case 5:
      printf("mode theaterChase white rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChase white rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChase(strip.Color(127, 127, 127), wait_sound); // white
      rpMode++;

      break;
    case 6:
      printf("mode theaterChase cyan rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChase cyan rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChase(strip.Color(0, 255, 255), wait_sound); // cyan
      rpMode++;

      break;
    case 7:
      printf("mode theaterChase purple rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChase purple rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChase(strip.Color(255, 0, 255), wait_sound); // purple
      rpMode++;

      break;
    case 8:
      printf("mode theaterChase white rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChase white rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChase(strip.Color(255, 255, 0), wait_sound); // yellow
      rpMode++;

      break;
    case 9:
      printf("mode theaterChase orange rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode theaterChase orange rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      theaterChase(strip.Color(255, 128, 0), wait_sound); // orange
      rpMode++;

      break;
    default:
      printf("mode colorWipe rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      SerialBT.printf("mode colorWipe rpMode =%d , wait_sound = %d \n", rpMode, wait_sound);
      colorWipe(strip.Color(0, 0, 255), wait_sound); // Blue
      rpMode++;

      break;
    }

    if (rpMode >= 2)
    {
      stepMode = rand() % 10;
      rpMode = rand() % 2;
    }

    // state = idle;

    break;

  case lamp:
    strip.fill(blueScale[5], 0, NUM_PIXELS);
    strip.setBrightness(255);
    strip.show();
    break;
  case idle:

    break;
  }
}

uint16_t readSound()
{

  currentState = digitalRead(soundPin);
  if (lastState == HIGH && currentState == LOW)
  {
    cntSdSpeed++;
  }
  else if (lastState == LOW && currentState == HIGH)
  {
    //  Serial.println("The sound has disappeared");
  }

  // save the the last state
  lastState = currentState;
  return cntSdSpeed;
}

void iron_normal(uint16_t d, int time)
{
  for (int j = 0; j < time; j++)
  {
    for (int i = 0; i < 73; i++)
    {
      strip.setBrightness(Brig);
      strip.fill(strip.Color(red, green, blue), 0, NUM_PIXELS);
      strip.show();
      delay(d);

      if (revGreen == false)
      {
        red += 2;
        green += 3;
      }
      else
      {
        red -= 2;
        green -= 3;
      }
      if (green > 220)
      {
        revGreen = true;
      }
      else if (green == 0)
      {
        revGreen = false;
      }
      if (Brig > 20)
      {
        revBrig = true;
      }
      if (Brig == 2)
      {
        revBrig = false;
      }
      if (revBrig == false)
      {
        Brig++;
      }
      else
      {
        Brig--;
      }
      Serial.printf("red = %d , green = %d , blue = %d , brig = %d , time = %d \n", red, green, blue, Brig, j);
    }
  }
}

void iron_man2()
{
  for (int i = 0; i < 5; i++)
  {

    strip.setBrightness(5 + (i * 3));
    strip.fill(blueScale[i], 0, NUM_PIXELS);
    strip.show();
    delay(300);
  }

  for (int i = 5; i > 0; i--)
  {

    strip.setBrightness(5 + (i * 3));
    strip.fill(blueScale[i], 0, NUM_PIXELS);
    strip.show();
    delay(300);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait)
{
  for (uint16_t i = 0; i < NUM_PIXELS; i++)
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256; j++)
  {
    // Serial.printf("block rainbow j =%d , i = %d \n",);

    for (i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++)
  { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait)
{
  for (int j = 0; j < 10; j++)
  { // do 10 cycles of chasing
    for (int q = 0; q < 3; q++)
    {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
      {
        strip.setPixelColor(i + q, c); // turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
      {
        strip.setPixelColor(i + q, 0); // turn every third pixel off
      }
    }
  }
}

// Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait)
{
  for (int j = 0; j < 256; j++)
  { // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++)
    {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
      {
        strip.setPixelColor(i + q, Wheel((i + j) % 255)); // turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
      {
        strip.setPixelColor(i + q, 0); // turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void whiteOverRainbow(int whiteSpeed, int whiteLength)
{

  if (whiteLength >= strip.numPixels())
    whiteLength = strip.numPixels() - 1;

  int head = whiteLength - 1;
  int tail = 0;
  int loops = 3;
  int loopNum = 0;
  uint32_t lastTime = millis();
  uint32_t firstPixelHue = 0;

  for (;;)
  { // Repeat forever (or until a 'break' or 'return')
    for (int i = 0; i < strip.numPixels(); i++)
    {                                     // For each pixel in strip...
      if (((i >= tail) && (i <= head)) || //  If between head & tail...
          ((tail > head) && ((i >= tail) || (i <= head))))
      {
        strip.setPixelColor(i, strip.Color(0, 0, 0, 255)); // Set white
      }
      else
      { // else set rainbow
        int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
      }
    }

    strip.show(); // Update strip with new contents
    // There's no delay here, it just runs full-tilt until the timer and
    // counter combination below runs out.

    firstPixelHue += 40; // Advance just a little along the color wheel

    if ((millis() - lastTime) > whiteSpeed)
    { // Time to update head/tail?
      if (++head >= strip.numPixels())
      { // Advance head, wrap around
        head = 0;
        if (++loopNum >= loops)
          return;
      }
      if (++tail >= strip.numPixels())
      { // Advance tail, wrap around
        tail = 0;
      }
      lastTime = millis(); // Save time of last movement
    }
  }
}

void pulseWhite(uint8_t wait)
{
  for (int j = 0; j < 256; j++)
  { // Ramp up from 0 to 255
    // Fill entire strip with white at gamma-corrected brightness level 'j':
    strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
    strip.show();
    delay(wait);
  }

  for (int j = 255; j >= 0; j--)
  { // Ramp down from 255 to 0
    strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
    strip.show();
    delay(wait);
  }
}

void rainbowFade2White(int wait, int rainbowLoops, int whiteLoops)
{
  int fadeVal = 0, fadeMax = 100;

  // Hue of first pixel runs 'rainbowLoops' complete loops through the color
  // wheel. Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to rainbowLoops*65536, using steps of 256 so we
  // advance around the wheel at a decent clip.
  for (uint32_t firstPixelHue = 0; firstPixelHue < rainbowLoops * 65536;
       firstPixelHue += 256)
  {

    for (int i = 0; i < strip.numPixels(); i++)
    { // For each pixel in strip...

      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      uint32_t pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());

      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the three-argument variant, though the
      // second value (saturation) is a constant 255.
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue, 255,
                                                          255 * fadeVal / fadeMax)));
    }

    strip.show();
    delay(wait);

    if (firstPixelHue < 65536)
    { // First loop,
      if (fadeVal < fadeMax)
        fadeVal++; // fade in
    }
    else if (firstPixelHue >= ((rainbowLoops - 1) * 65536))
    { // Last loop,
      if (fadeVal > 0)
        fadeVal--; // fade out
    }
    else
    {
      fadeVal = fadeMax; // Interim loop, make sure fade is at max
    }
  }

  for (int k = 0; k < whiteLoops; k++)
  {
    for (int j = 0; j < 256; j++)
    { // Ramp up 0 to 255
      // Fill entire strip with white at gamma-corrected brightness level 'j':
      strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
      strip.show();
    }
    delay(1000); // Pause 1 second
    for (int j = 255; j >= 0; j--)
    { // Ramp down 255 to 0
      strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
      strip.show();
    }
  }

  delay(500); // Pause 1/2 second
}

void callRedLaser(uint8_t brig, uint8_t w)
{
  strip.setBrightness(brig);
  // theaterChase(strip.Color(255, 0, 0), (w));
  colorWipe(strip.Color(178, 0, 255), (w)); // Red
  // strip.clear();
}
void callOrgLaser(uint8_t brig, uint8_t w)
{
  strip.setBrightness(brig);
  // theaterChase(strip.Color(255, 0, 0), (w));
  colorWipe(strip.Color(72, 0, 200), (w)); // Red
  // strip.clear();
}

void callCyanLaser(uint8_t brig, uint8_t w)
{
  strip.setBrightness(brig);
  // theaterChase(strip.Color(255, 0, 0), (w));
  colorWipe(strip.Color(165, 255, 255), (w)); // Cyqn
  // strip.clear();
}