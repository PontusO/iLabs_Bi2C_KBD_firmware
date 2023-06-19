/*
  NUMPAD

  This is an example on how to create an atonomous keyboard application
  on the Bi2C KBD board.

  Settings you need to build it for the ATTiny1616 are:

    * Chip: ATtiny1616
    * Clock: 20MHz internal
    * Millis timer: Enable (default timer)
    * Startup time: 8mS
    * PWM Pins: PB3-5, PC3, PA4/PA5 No TCD PWM even on 1-series

  In order to debug the system it can be built for the ATMEGA processor and executed
  on an Arduini MEGA board which gets you tons of debug options.
*/

#include <Wire.h>
#include <CircularBuffer.h>
#include <megaTinyCore.h>

#if defined(ARDUINO_AVR_MEGA2560)
  #define BUILD_FOR_ATMEGA
  #if defined(BUILD_FOR_ATMEGA)
    #define DEBUG
  #endif
#endif

CircularBuffer<uint8_t, 16> key_queue;

#define NUM_ROWS  4
#define NUM_COLS  4
#define NUM_LEDS  5

#if defined(BUILD_FOR_ATMEGA)
static const uint8_t columns[NUM_COLS] = {22, 23, 24, 25};
static const uint8_t rows[NUM_ROWS] = {26, 27, 28, 29};
static const uint8_t led_pins[NUM_LEDS] = {30, 31, 32, 33, 34};
const uint8_t INT_PIN = 30;
const uint8_t LED_CTRL_PIN = 31;
#else
static const uint8_t columns[NUM_COLS] = {PIN_PC1, PIN_PC2, PIN_PC3, PIN_PA1};
static const uint8_t rows[NUM_ROWS] = {PIN_PB2, PIN_PB3, PIN_PB4, PIN_PB5};
static const uint8_t led_pins[NUM_LEDS] = {PIN_PA6, PIN_PA1, PIN_PC3, PIN_PC2, PIN_PC1};
const uint8_t INT_PIN = PIN_PC0;
const uint8_t LED_CTRL_PIN = PIN_PA7;
#undef LED_BUILTIN
#define LED_BUILTIN PIN_PA6
#endif
static const uint8_t keytab[NUM_COLS][NUM_ROWS] = 
          { {'*', '7', '4', '1'}, { '0', '8', '5', '2' },
            {'#', '9', '6', '3'}, { 'D'+0x80, 'C'+0x80, 'B'+0x80, 'A'+0x80} };
            
static uint8_t previousState[NUM_COLS][NUM_ROWS];
static uint8_t pressed[NUM_COLS][NUM_ROWS];

const int I2C_SLAVE_ADDRESS = 0x32;
volatile byte i2cRegister = 0xff;
volatile bool reg_update = false;
volatile uint8_t updated_register;
volatile uint8_t int_pin_mode = 0;
/*
 * Register map of the I2C registers.
 * 
 * addr: 0 (Write)  - Control register (CTRL_REG)
 * 
 * This register sets the operation mode of the numpad.
 *
 *    +-+-+-+-+-+-+-+-+
 *    |7|6|5|4|3|2|1|0|
 *    +-+-+-+-+-+-+-+-+
 *     | | | | | | | |
 *     | | | | | | | +- Interrupt enable
 *     | | | | | | +--- Interrupt pin mode 0
 *     | | | | | +----- Interrupt pin mode 1
 *     | | | | +------- Numeric keys enable
 *     | | | +--------- Alpha numeric keys enable
 *     | | +----------- No function (Write 0)
 *     | +------------- No function (Write 0)
 *     +--------------- Reset device
 *     
 * Bit 0 : Interrupt enable (Default : 0)
 *   Setting this bit high will enable interrupts from the keyboard 
 *   controller. A single interrupt will be triggered from each key press.
 *   
 * Bit 2 and 1 : Interrupt pin mode (Default : 00)
 *   Sets the operating mode of the interrupt pin according to the followin
 *   table:
 *    00 = Open drain pulled high internally.
 *    01 = Open drain with external pull up.
 *    10 = Push pull active high.
 *    11 = Push pull active low.
 *
 * Bit 3 : Disable numeric keys (Default : 0)
 *   Setting this bit high will disable the numeric keys on the keyboard.
 *   
 * Bit 4 : Disable  alpha numeric keys (A-D) (Default : 0)
 *   Setting this bit high will disable the alpha numeric keys on the keyboard.
 *   
 * Bits 5 and 6 : No function
 *   These are unimplemented control bits but should be written 0 to ensure
 *   compatability with future firmware releases.
 *   
 * Bit 7 : Reset device.
 *   This bit is used to reset the device to a known stat.
 * 
 */
#define CTRL_REG                    0
#define CTRL_REG_INTERRUPT_ENABLE   0x01
#define CTRL_REG_INT_PIN_MODE_0     0x02
#define CTRL_REG_INT_PIN_MODE_1     0x04
#define CTRL_REG_NUM_KEYS_DISABLE   0x08
#define CTRL_REG_ALPHA_KEYS_DISABLE 0x10
#define CTRL_REG_RESET              0x80

/* addr: 0 (Read)   - Status register (STAT_REG)
 * 
 * Returns the current status of the device.
 * 
 *    +-+-+-+-+-+-+-+-+
 *    |7|6|5|4|3|2|1|0|
 *    +-+-+-+-+-+-+-+-+
 *     | | | | | | | |
 *     | | | | | | | +- Interrupt status
 *     | | | | | | +--- Keypress in buffer
 *     | | | | | +----- No function
 *     | | | +--------- No function
 *     | +------------- No function
 *     +--------------- No function
 */
#define STAT_REG                      0
#define STAT_REG_INTERRUPT_STATUS     0x01
#define STAT_REG_KEY_PRESS            0x02

/* addr: 1 (Read)   - Key code (In ascii) (KEY_REG)
 * 
 * Returns the pressed key. The user should read the status register to 
 * ensure that there is a valid keypress in the buffer. 0xFF will be returned
 * if there is no key press entry in the buffer.
 * 
 * Key presses are buffered in a 16 entry FIFO.
 * 
 */
#define KEY_REG                       1

/* addr: 2 (Read)   - Returns the number of key entries in the buffer (NUM_KEYS_REG)
 * 
 */
#define NUM_KEYS_REG                  2

/*
 * addr: 3 (Write)  - Status LED control register (LED_CTRL_REG_1)
 * 
 * This register controls the behavior of the status LED.
 * 
 *    +-+-+-+-+-+-+-+-+
 *    |7|6|5|4|3|2|1|0|
 *    +-+-+-+-+-+-+-+-+
 *     | | | | | | | |
 *     | | | | | | | +- mode bit 0
 *     | | | | | | +--- mode bit 1
 *     | | | | | +----- mode bit 2
 *     | | | | +------- mode bit 3
 *     | | | +--------- No function
 *     | | +------------No function
 *     | +------------- No function
 *     +--------------- No function
 *     
 *  operation modes:
 *    mode 0 - The LED is turned off.
 *    mode 1 - The LED is turned on with a fixed light.
 *    mode 2 - The LED is blinking with 1 Hz.
 *    mode 3 - The LED is blinking with 2 Hz.
 *    mode 4 - The LED is blinking with 4 Hz.
 *    mode 5 - Not implemented
 *    mode 6 - Not implemented
 *    mode 7 - Not implemented
 * 
 */
#define LED_CTRL_REG_1  3

/*
 * addr: 4 (Write)  - Status LED control register (LED_CTRL_REG_2)
 * 
 * This register controls the behavior of the indicator LED's 1 and 2.
 * 
 *    +-+-+-+-+-+-+-+-+
 *    |7|6|5|4|3|2|1|0|
 *    +-+-+-+-+-+-+-+-+
 *     | | | | | | | |
 *     | | | | | | | +- LED 1 mode bit 0
 *     | | | | | | +--- LED 1 mode bit 1
 *     | | | | | +----- LED 1 mode bit 2
 *     | | | | +------- LED 1 mode bit 3
 *     | | | +--------- LED 2 mode bit 0
 *     | | +------------LED 2 mode bit 1
 *     | +------------- LED 2 mode bit 2
 *     +--------------- LED 2 mode bit 3
 *     
 *  operation modes:
 *    mode 0 - The LED is turned off.
 *    mode 1 - The LED is turned on with a fixed light.
 *    mode 2 - The LED is blinking with 1 Hz.
 *    mode 3 - The LED is blinking with 2 Hz.
 *    mode 4 - The LED is blinking with 4 Hz.
 *    mode 5 - Not implemented
 *    mode 6 - Not implemented
 *    mode 7 - Not implemented
 * 
 */
#define LED_CTRL_REG_2  4

/*
 * addr: 5 (Write)  - Status LED control register (LED_CTRL_REG_3)
 * 
 * This register controls the behavior of the indicator LED's 3 and 4.
 * 
 *    +-+-+-+-+-+-+-+-+
 *    |7|6|5|4|3|2|1|0|
 *    +-+-+-+-+-+-+-+-+
 *     | | | | | | | |
 *     | | | | | | | +- LED 3 mode bit 0
 *     | | | | | | +--- LED 3 mode bit 1
 *     | | | | | +----- LED 3 mode bit 2
 *     | | | | +------- LED 3 mode bit 3
 *     | | | +--------- LED 4 mode bit 0
 *     | | +------------LED 4 mode bit 1
 *     | +------------- LED 4 mode bit 2
 *     +--------------- LED 4 mode bit 3
 *     
 *  operation modes:
 *    mode 0 - The LED is turned off.
 *    mode 1 - The LED is turned on with a fixed light.
 *    mode 2 - The LED is blinking with 1 Hz.
 *    mode 3 - The LED is blinking with 2 Hz.
 *    mode 4 - The LED is blinking with 4 Hz.
 *    mode 5 - Not implemented
 *    mode 6 - Not implemented
 *    mode 7 - Not implemented
 * 
 */
#define LED_CTRL_REG_3  5

#define NUM_REGS        6
// Two complete register files (control and status registers) are maintained.
uint8_t ctrl_registers[NUM_REGS];
uint8_t stat_registers[NUM_REGS];

typedef enum led_mode_e {
  LED_OFF = 0,
  LED_ON,
  LED_BLINK_1HZ,
  LED_BLINK_2HZ,
  LED_BLINK_4HZ,
  LED_CANDLE
} led_mode_t;

class LED
{
  public:
    void setup(int pin, led_mode_t mode, int alpha = 20, int delay = 100);
    void setmode(led_mode_t newmode);
    void process();
    void restore();

  private:
    led_mode_t _mode;
    int _pin;
    int _brightness = 0;  // soft start
    int _oldBrightness = 0;
    int _updateDelay;
    unsigned long _nextUpdate;
    int _alpha; // filter coefficient (0..100). Low means slow changes
};

void LED::setup(int pin, led_mode_t mode, int alpha, int delay) {
  _mode = mode;
  constrain(alpha, 0, 100);
  _pin = pin;
  _alpha = alpha;
  _updateDelay = delay;
  // random first upate to prevent multiple sequences looking synchronous
  _nextUpdate  = millis() + random(_updateDelay);
}

void LED::setmode(led_mode_t newmode) {
  _mode = newmode;
  _nextUpdate = millis() + 1;  // Ensures a quick update.
}

void LED::restore() {
  if (_mode == LED_CANDLE)
    analogWrite(_pin, _brightness);
  else
    digitalWrite(_pin, _brightness);
}

void LED::process() {
  switch (_mode) {
    case LED_OFF:
      digitalWrite(_pin, LOW);
      _brightness = 0;
    break;

    case LED_ON:
      digitalWrite(_pin, HIGH);
      _brightness = 1;
    break;

    case LED_BLINK_1HZ:
      if (millis() >= _nextUpdate) {
        _nextUpdate = millis() + 500;
        digitalWrite(_pin, _brightness);
        _brightness = _brightness ? 0 : 1;
      }
    break;

    case LED_BLINK_2HZ:
      if (millis() >= _nextUpdate) {
        _nextUpdate = millis() + 250;
        digitalWrite(_pin, _brightness);
        _brightness = _brightness ? 0 : 1;
      }
    break;

    case LED_BLINK_4HZ:
      if (millis() >= _nextUpdate) {
        _nextUpdate = millis() + 125;
        digitalWrite(_pin, _brightness);
        _brightness = _brightness ? 0 : 1;
      }
    break;
    
    case LED_CANDLE:
      if (millis() >= _nextUpdate) {
        _nextUpdate += _updateDelay; 
        _brightness = random(0, 255);
        // low pass filter the brightness changes
        _brightness = (_alpha * _brightness + (100 - _alpha) * _oldBrightness) / 100;
        _oldBrightness = _brightness;
        analogWrite(_pin, _brightness);
      }
    break;
  }
}

LED leds[NUM_LEDS];

void i2cReceive(int bytesReceived) {
  // When receiving more than 1 bytes the transaction is determined to be
  // a write register operation.
  // If we only get one byte it is a read operation.
  i2cRegister = Wire.read();
  if (bytesReceived > 1) {
    // Write operation, transfer incoming data to the register file
    ctrl_registers[i2cRegister] = Wire.read();
    reg_update = true;
    updated_register = i2cRegister;
  }
}

void i2cRequest() {
  // Single register read.
  switch (i2cRegister) {
    case STAT_REG:
      Wire.write(stat_registers[i2cRegister]);
      break;
      
    case LED_CTRL_REG_1:
    case LED_CTRL_REG_2:
    case LED_CTRL_REG_3:
      // LED control registers are simply returned here
      Wire.write(ctrl_registers[i2cRegister]);
      break;

    case NUM_KEYS_REG:
      // Returns the number of key presses in the buffer.
      Wire.write(key_queue.size());
      break;

    case KEY_REG:
      if (key_queue.isEmpty()) {
        Wire.write(0xff);
      } else {
        Wire.write(key_queue.pop());
      }
      if (key_queue.isEmpty()) {
        stat_registers[STAT_REG] &= 0xfd;  // Clear the key available flag
      }
      break;    
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
#if defined(DEBUG)
  Serial.begin(115200);
  Serial.println("Serial DEBUG enabled, version x.xx");
#endif
  // initialize digital pin LED_BUILTIN as an output and set it low.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize interrupt output pin to input.
  // When it is time to use it we simply set it to output mode and the
  // external pull upp will be pulled low.
  pinMode(INT_PIN, INPUT_PULLUP);
  digitalWrite(INT_PIN, LOW);

  // Control pin for the multiplexed LED's
  // Default high so no LED's will light up.
  pinMode(LED_CTRL_PIN, OUTPUT);
  digitalWrite(LED_CTRL_PIN, HIGH);

  // Configure registers
  for (int i=0; i < NUM_REGS; i++) {
    ctrl_registers[i] = 0;
    stat_registers[i] = 0;
  }
  
  // Configure numpad pins
  for (int i=0; i < NUM_COLS; i++) {
    pinMode(columns[i], OUTPUT);
    digitalWrite(columns[i], HIGH);
    pinMode(rows[i], INPUT_PULLUP);
    for (uint8_t j=0; j < NUM_ROWS; j++) {
      previousState[i][j] = HIGH;  // Default non pressed state
      pressed[i][j] = false;
    }
  }

  Wire.begin(I2C_SLAVE_ADDRESS);

  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);

  for (int i=0; i < NUM_LEDS; i++)
    leds[i].setup(led_pins[i], LED_OFF);
}

// The main loop
static unsigned long previousMillis = 0;
static const long debounce = 25;   // Debounce time
void loop() {
  unsigned long currentMillis = millis();
  uint8_t i, j;

  if (currentMillis - previousMillis >= debounce) {
    // Turn off the LED's while scanning the keyboard
    digitalWrite(LED_CTRL_PIN, HIGH);
    // Make all column pins high before starting.
    for (i = 0; i< NUM_COLS; i++)
      digitalWrite(columns[i], HIGH);
    delay(1);
    // Update time stamp for debouncing
    previousMillis = currentMillis;
    for (i = 0; i< NUM_COLS; i++) {
      digitalWrite(columns[i], LOW);
      for (j = 0; j < NUM_ROWS; j++) {
        uint8_t pin_state = digitalRead(rows[j]);
        if (pin_state == LOW && previousState[i][j] == LOW && pressed[i][j] == false) {
          pressed[i][j] = true;
          // This happens when a key is pressed down.
          if (!(ctrl_registers[CTRL_REG] & CTRL_REG_NUM_KEYS_DISABLE) && !(keytab[i][j] & 0x80) || 
             (!(ctrl_registers[CTRL_REG] & CTRL_REG_ALPHA_KEYS_DISABLE) && keytab[i][j] & 0x80)) {
            key_queue.unshift(keytab[i][j] & 0x7F);
            // Did the user request interupts ?
            if (ctrl_registers[CTRL_REG] & CTRL_REG_INTERRUPT_ENABLE) {
              // Depending on the interrupt pin mode we need to treat it differently
              switch (int_pin_mode) {
                case 0:
                case 1:
                  pinMode(INT_PIN, OUTPUT);   // Short interrupt pulse
                  pinMode(INT_PIN, INPUT_PULLUP);
                  break;
      
                case 2:
                  digitalWrite(INT_PIN, HIGH);
                  digitalWrite(INT_PIN, LOW);     // Short high pulse
                  break;
      
                case 3:
                  digitalWrite(INT_PIN, LOW);
                  digitalWrite(INT_PIN, HIGH);    // Short low pulse
                  break;
              }
            }
            stat_registers[STAT_REG] |= STAT_REG_KEY_PRESS;
          }
        } else if (pin_state == HIGH && previousState[i][j] == HIGH && pressed[i][j] == true) {
          pressed[i][j] = false;
        }
        // Save current state for next time around.
        previousState[i][j] = pin_state;
      }
      digitalWrite(columns[i], HIGH);
    }
    // Restore the state of all LED pins.
    for (int i=0; i<NUM_LEDS; i++)
      leds[i].restore();
    // And turn them back on
    digitalWrite(LED_CTRL_PIN, LOW);
  } else {
    for (int i=0; i<NUM_LEDS; i++)
      leds[i].process();
    // Turn LED's back on again
  }
  
  if (reg_update) {
    reg_update = false;
    switch(updated_register) {
      case CTRL_REG:
        // First check if the user wants to reset the device
        if (ctrl_registers[CTRL_REG] & CTRL_REG_RESET)
          ResetViaWDT();
        // Something has changed in the main control register
        // So we need to set the interrupt pin mode.
        int_pin_mode = (ctrl_registers[CTRL_REG] & (CTRL_REG_INT_PIN_MODE_0 | CTRL_REG_INT_PIN_MODE_1)) >> 1;
        switch (int_pin_mode) {
          case 0:
            pinMode(INT_PIN, INPUT_PULLUP); // Use internall pullup
            break;

          case 1:
            pinMode(INT_PIN, INPUT);        // Needs external pullup
            break;

          case 2:
            pinMode(INT_PIN, OUTPUT);
            digitalWrite(INT_PIN, LOW);     // Push pull, active high interrupt
            break;

          case 3:
            pinMode(INT_PIN, OUTPUT);
            digitalWrite(INT_PIN, HIGH);    // Push pull, active low interrupt
            break;
        }
      break;

      case LED_CTRL_REG_1:
        leds[0].setmode((led_mode_t)(ctrl_registers[LED_CTRL_REG_1] & 0x0f));
      break;
      
      case LED_CTRL_REG_2:
        leds[1].setmode((led_mode_t)(ctrl_registers[LED_CTRL_REG_2] & 0x0f));
        leds[2].setmode((led_mode_t)((ctrl_registers[LED_CTRL_REG_2] >> 4) & 0x0f));
      break;
      
      case LED_CTRL_REG_3:
        leds[3].setmode((led_mode_t)(ctrl_registers[LED_CTRL_REG_3] & 0x0f));
        leds[4].setmode((led_mode_t)((ctrl_registers[LED_CTRL_REG_3] >> 4) & 0x0f));
      break;
    }
  }
}
