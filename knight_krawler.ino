
#define LEFT_MIN 425
// origin
// less is to the right
// 510 was slightly to the right
// 550 was slightly to the right
// 580 was to the left but could reverse
// 565 was slightly to the left
#define LEFT_ORIGIN 580
#define LEFT_MAX 700

#define RIGHT_MIN 321
// origin
// more is to the right
// 428 was slightly to the right.
// 380 was slightly to the right
// 350 was to the left but could reverse
// 365 was slightly to the left
#define RIGHT_ORIGIN 350
#define RIGHT_MAX 645

class Logger {
  private:
  boolean enabled;
  boolean disable_all;
  
  public:
  Logger(boolean enabled=false) {
    this->enabled = enabled;
    this->disable_all = false;
  }
  
  void enable() {
    this->enabled = true;
  }
  
  void disable() {
    this->enabled = false;
  }
  
  boolean isEnabled() {
    return this->enabled && !this->disable_all;
  }
  
  void print(const char *value) {
    if (this->isEnabled()) {
      Serial.print(value);
    }
  }
  
  void print(char *value) {
    if (this->isEnabled()) {
      Serial.print(value);
    }
  }
  
  void print(int value) {
    if (this->isEnabled()) {
      Serial.print(value);
    }
  }
  
  void print(float value) {
    if (this->isEnabled()) {
      Serial.print(value);
    }
  }
  
  void println(const char *value) {
    if (this->isEnabled()) {
      Serial.println(value);
    }
  }
  
  void println(char *value) {
    if (this->isEnabled()) {
      Serial.println(value);
    }
  }
  
  void println(int value) {
    if (this->isEnabled()) {
      Serial.println(value);
    }
  }
  
  void println(float value) {
    if (this->isEnabled()) {
      Serial.println(value);
    }
  }
};

class Actuator {
  private:
  Logger* logger;
  int pin_in;
  int pin_out_0;
  int pin_out_1;
  int target_position;
  int threshold;
  
  public:
  Actuator(Logger* logger, int pin_in, int pin_out_0, int pin_out_1, int threshold, int target_position) {
    this->logger = logger;
    
    this->pin_in = pin_in;
    this->pin_out_0 = pin_out_0;
    this->pin_out_1 = pin_out_1;
    
    this->threshold = threshold;
    this->target_position = target_position;
    
    pinMode(pin_out_0, OUTPUT);
    pinMode(pin_out_1, OUTPUT);
    pinMode(pin_in, INPUT);
    
    this->stay();
  }
  
  int getPosition() {
    return analogRead(this->pin_in);
  }
  
  int getThreshold() {
    return this->threshold;
  }
  
  int getTargetPosition() {
    return this->target_position;
  }
  
  void setTargetPosition(int value) {
    this->target_position = value;
  }
  
  void retract() {
    digitalWrite(this->pin_out_0, HIGH);
    digitalWrite(this->pin_out_1, LOW);
  }
  
  void extract() {
    digitalWrite(this->pin_out_0, LOW);
    digitalWrite(this->pin_out_1, HIGH);
  }
  
  void stay() {
    digitalWrite(this->pin_out_0, HIGH);
    digitalWrite(this->pin_out_1, HIGH);
  }
  
  void update() {
    int current_position = this->getPosition();
    
    if (current_position < (this->target_position - this->threshold)) {
      this->extract();
    } else if (current_position > (this->target_position + this->threshold)) {
      this->retract();
    } else {
      this->stay();
    }
  }
};

class Joystick {
  private:
  Logger* logger;
  int pin_h;
  int pin_v;
  int origin_h;
  int origin_v;
  int threshold_h;
  int threshold_v;
  int min_h;
  int max_h;
  int min_v;
  int max_v;
  
  public:
  Joystick(Logger* logger, int pin_h, int pin_v, int origin_h, int origin_v, int threshold_h, int threshold_v) {
    this->logger = logger;
    this->pin_h = pin_h;
    this->pin_v = pin_v;
    this->origin_h = origin_h;
    this->origin_v = origin_v;
    this->threshold_h = threshold_h;
    this->threshold_v = threshold_v;
    this->min_h = 0;
    this->min_v = 0;
    this->max_h = 1023;
    this->max_v = 1023;

    pinMode(pin_v, INPUT);
    pinMode(pin_h, INPUT);
  }
  
  int getHorizontalPosition() {
    int value = analogRead(this->pin_h);
    
    return value;
  }
  
  int getVerticalPosition() {
    return analogRead(this->pin_v);
  }
  
  boolean isUp() {
    return this->getVerticalPosition() < (this->origin_v - this->threshold_v);
  }
  
  boolean isDown() {
    return this->getVerticalPosition() > (this->origin_v + this->threshold_v);
  }
  
  boolean isLeft() {
    return this->getHorizontalPosition() < (this->origin_h - this->threshold_h);
  }
  
  boolean isRight() {
    return this->getHorizontalPosition() > (this->origin_h + this->threshold_h);
  }

  float getLeft() {
    return 1 - (this->getHorizontalPosition() / (float)(this->origin_h - this->min_h));
  }
  
  float getRight() {
    return (this->getHorizontalPosition() - this->origin_h) / (float)(this->max_h - this->origin_h);
  }

  float getUp() {
    return 1 - (this->getVerticalPosition() / (float)(this->origin_v - this->min_v));
  }
  
  float getDown() {
    return (this->getVerticalPosition() - this->origin_v) / (float)(this->max_v - this->origin_v);
  }
};

class Button {
  private:
  int pin;
  
  public: 
  Button(int pin) {
    this->pin = pin;
    
    pinMode(pin, INPUT_PULLUP);
  }
  
  boolean isPressed() {
    return digitalRead(this->pin) == HIGH;
  }
};

class Wheel {
  private:
  Logger* logger;
  Actuator* actuator;
  int min;
  int origin;
  int max;
  
  public:
  Wheel(Logger* logger, Actuator* actuator, int min, int origin, int max) {
    this->logger = logger;
    this->actuator = actuator;
    this->min = min;
    this->origin = origin;
    this->max = max;
  }
  
  void turnInside(float percent) {
    this->actuator->setTargetPosition(this->origin - (this->origin - this->min) * percent);
  }
  
  void turnOutside(float percent) {
    this->actuator->setTargetPosition((this->max - this->origin) * percent + this->origin);
  }
  
  int getTargetPosition() {
    return this->actuator->getTargetPosition();
  }
  
  int getCurrentPosition() {
    return this->actuator->getPosition();
  }
  
  void stay() {
    this->actuator->stay();
  }
  
  void update() {
    this->actuator->update();
  }
};

class LimitSwitch {
  private:
  int pin;
  
  public:
  LimitSwitch(int pin) {
    this->pin = pin;
    
    pinMode(pin, INPUT_PULLUP);
  }
  
  boolean isOn() {
    return digitalRead(this->pin) == HIGH;
  }
  
  boolean isOff() {
    return digitalRead(this->pin) != HIGH;
  }
};

class Relay {
  private:
  int pin;
  
  public:
  Relay(int pin) {
    this->pin = pin;
    
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  
  void on() {
    digitalWrite(this->pin, LOW);
  }
  
  void off() {
    digitalWrite(this->pin, HIGH);
  }
};

class Motor {
  private:
  Logger* logger;
  
  public:
  Motor(Logger *logger) {
    this->logger = logger;
    
    Serial.write(128);
  }
  
  void drive(int value) {
    value = map(value, 0, 1023, 0, 255);
    this->logger->println(value);
    Serial.write(value);
//    Serial.flush();
  }
};

class CartState {
  public:
  virtual void loop() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
};

class DriveState : public CartState {
  private:
  Wheel *left;
  Wheel *right;
  Joystick *joystick;
  Motor *motor;
  Logger *logger;
  
  public:
  DriveState(Wheel *left, Wheel *right, Joystick *joystick, Motor *motor, Logger *logger) {
    this->left = left;
    this->right = right;
    this->joystick = joystick;
    this->motor = motor;
    this->logger = logger;
  }
  
  void loop() {
    if (this->joystick->isRight()) {
      this->logger->print("right ");
      
      float value = this->joystick->getRight();

      this->left->turnInside(value);
      this->right->turnOutside(value);
    } else if (this->joystick->isLeft()) {
      this->logger->print("left ");
      
      float value = this->joystick->getLeft();

      this->left->turnOutside(value);
      this->right->turnInside(value);
    } else {
      this->left->turnInside(0);
      this->right->turnInside(0);
    }
    
    this->logger->print(this->joystick->getLeft());
    this->logger->print(" ");
    this->logger->print(this->left->getCurrentPosition());
    this->logger->print(" ");
    this->logger->println(this->right->getCurrentPosition());

    this->left->update();
    this->right->update();
    /*    
    this->motor->drive(vertical);
    /**/

    delay(10);
  }
  
  void start() {
    this->logger->println("Starting drive state");
  }
  
  void stop() {
    this->logger->println("Stopping drive state");
    
    this->left->stay();
    this->right->stay();
    /*
    this->motor->drive(512);
    /**/
  }
};

class RampState : public CartState {
  private:
  LimitSwitch* limit_top;
  LimitSwitch* limit_bottom;
  Joystick *joystick;
  Relay* relay_up;
  Relay* relay_down;
  Logger* logger;
  Relay* relay_joystick_switch1;
  Relay* relay_joystick_switch2;
  
  public:
  RampState(LimitSwitch *limit_top, LimitSwitch *limit_bottom, Joystick *joystick, Relay *relay_up, Relay *relay_down, Logger *logger, Relay* relay_joystick_switch1, Relay* relay_joystick_switch2) {
    this->limit_top = limit_top;
    this->limit_bottom = limit_bottom;
    this->joystick = joystick;
    this->relay_up = relay_up;
    this->relay_down = relay_down;
    this->logger = logger;
    this->relay_joystick_switch1 = relay_joystick_switch1;
    this->relay_joystick_switch2 = relay_joystick_switch2;
  }
  
  void loop() {
    this->logger->println(this->joystick->getVerticalPosition());    
    
    if (this->limit_bottom->isOff()) {
      this->relay_up->off();
      this->relay_down->off();
    } else if (this->joystick->isUp()) {
      this->relay_down->off();
      this->relay_up->on();
    } else if (this->joystick->isDown()) {
      this->relay_up->off();
      this->relay_down->on();
    } else {
      this->relay_up->off();
      this->relay_down->off();
    }
    
    delay(10);
  }
  
  void start() {
    this->logger->println("Starting ramp state");
    this->relay_joystick_switch1->on();
    this->relay_joystick_switch2->on();
  }
  
  void stop() {
    this->logger->println("Stopping ramp state");

    this->relay_up->off();
    this->relay_down->off();
    this->relay_joystick_switch1->off();
    this->relay_joystick_switch2->off();
  }
};

Wheel* left;
Wheel* right;
Joystick* joystick;
Button* button;
LimitSwitch* limit_top;
LimitSwitch* limit_bottom;
Relay* relay_up;
Relay* relay_down;
Motor* motor;
CartState *cart_states[2];
int last_state;

void setup() {
  Serial.begin(9600);

  int joystick_horizontal_pin = A3; // ?
  int joystick_vertical_pin = A2; // ?
  int joystick_horizontal_threshold = 3;
  int joystick_vertical_threshold = 3;
  int joystick_horizontal_origin = analogRead(joystick_horizontal_pin);
  int joystick_vertical_origin = 532; //analogRead(joystick_vertical_pin);
  int joystick_relay_switch1 = 8;
  int joystick_relay_switch2 = 9;

  int button_pin = 13;
  int limit_top_pin = -2; // ?
  int limit_bottom_pin = 12;
  int relay_up_pin = 6; // ?
  int relay_down_pin = 7; // ?
  
  int left_pin_in = A1; // ?
  int left_pin_out_0 = 5; // ?
  int left_pin_out_1 = 4; // ?
  int left_threshold = 7;
  int left_min = LEFT_MIN;
  int left_origin = LEFT_ORIGIN;
  int left_max = LEFT_MAX;
  
  int right_pin_in = A0;
  int right_pin_out_0 = 3;
  int right_pin_out_1 = 2;
  int right_threshold = 4;
  int right_min = RIGHT_MIN;
  int right_origin = RIGHT_ORIGIN;
  int right_max = RIGHT_MAX;
  
  last_state = -1;
  left = new Wheel(new Logger(), new Actuator(new Logger(), left_pin_in, left_pin_out_0, left_pin_out_1, left_threshold, left_origin), left_min, left_origin, left_max);
  right = new Wheel(new Logger(), new Actuator(new Logger(), right_pin_in, right_pin_out_0, right_pin_out_1, right_threshold, right_origin), right_min, right_origin, right_max);
  joystick = new Joystick(new Logger(), joystick_horizontal_pin, joystick_vertical_pin, joystick_horizontal_origin, joystick_vertical_origin, joystick_horizontal_threshold, joystick_vertical_threshold);
  button = new Button(button_pin);
  limit_top = new LimitSwitch(limit_top_pin);
  limit_bottom = new LimitSwitch(limit_bottom_pin);
  relay_up = new Relay(relay_up_pin);
  relay_down = new Relay(relay_down_pin);
  motor = new Motor(new Logger());
  
  cart_states[0] = new DriveState(left, right, joystick, motor, new Logger());  
  cart_states[1] = new RampState(limit_top, limit_bottom, joystick, relay_up, relay_down, new Logger(true), new Relay(joystick_relay_switch1), new Relay(joystick_relay_switch1));
}

void loop() {
  int current_state = button->isPressed() ? 0 : 1;
  
  if (current_state == last_state) {
    cart_states[current_state]->loop();
  } else if (last_state != -1) {
    cart_states[last_state]->stop();
    
    delay(2000);
    cart_states[current_state]->start();
  } else {
    cart_states[current_state]->start();
  }
  
  last_state = current_state;
}

