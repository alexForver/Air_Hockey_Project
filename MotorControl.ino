#define EN 8 // port8=stepper enable/disable, active low
#define ENZ 1 // enable stepper motor, active low
#define Y_DIR 6 // PE4
#define Y_STP 3 // PE1
#define X_DIR 5 // PE3
#define X_STP 2 // PE0

#define MAX_SPEED 150
#define ZERO_SPEED 1000 //30000 //65535

#define X_step_per_mm 5
#define Y_step_per_mm 5

volatile unsigned int max_speed = MAX_SPEED;
volatile int16_t abs_target_m1_step = 0;
volatile int16_t abs_target_m2_step = 0;
volatile int16_t m1_stepCount = 0;
volatile int16_t m2_stepCount = 0;
volatile int16_t m1_stepPosition = 0;
volatile int16_t m2_stepPosition = 0;
volatile float factor1 = 1.0;
volatile float factor2 = 1.0;
volatile float factor11 = 1.0;
volatile float factor22 = 1.0;
volatile int16_t m1_dir = 0;
volatile int16_t m2_dir = 0;
volatile bool m1_moveDone = false;
volatile bool m2_moveDone = false;
volatile int16_t m1_rampUpStepCount = 0;
volatile int16_t m2_rampUpStepCount = 0;
volatile int16_t n1 = 0;
volatile int16_t n2 = 0;

volatile unsigned int d1 = 0;
volatile unsigned int d2 = 0;

uint32_t timer_old;

void setup() {
  pinMode(X_STP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_STP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(ENZ, OUTPUT);
  
  Serial.begin(115200);   // PC debug connection

  Serial.println("Ready：");
  while (!(Serial.available())) {}
  char a  = Serial.read();
  if (!(a == 'a'))
    while (1);

  digitalWrite(ENZ, LOW);
  digitalWrite(EN, LOW);

  TCCR1A = 0;
  TCCR1B  &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= ((1 << CS10) | (1 << CS11)); // Start timer at Fcpu/64
  TCNT1 = 0;

  TCCR3A = 0;
  TCCR3B  &= ~(1 << WGM13);
  TCCR3B  |= (1 << WGM12);
  TCCR3B |= ((1 << CS10) | (1 << CS11)); // Start timer at Fcpu/64
  TCNT3 = 0;

  sei();

  timer_old = micros();
}

void loop() {
  moveToPosition(200,200);
  delay(500);
  moveToPosition(0,100);
  delay(500);
  moveToPosition(-200,0);
  delay(500);
  moveToPosition(200,-100);
  delay(500);
  moveToPosition(-400,-100);
  delay(500);
  moveToPosition(200,-100);
  while (true) {};
}

ISR(TIMER1_COMPA_vect)
{
  if ( m1_stepCount < abs_target_m1_step )
  {
    digitalWrite(X_STP, HIGH);
    __asm__ __volatile__ (
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop");  // Wait for step pulse
    digitalWrite(X_STP, LOW);
    m1_stepCount ++;
    m1_stepPosition += m1_dir;
  } else {
    m1_moveDone = true;
    TIMSK1 &= ~(1 << OCIE1A);
  }

  if ( m1_rampUpStepCount == 0 ) {
    n1 ++;
    d1 = d1 - int( 2 * float(d1) / (4 * float(n1) + 1));
    //d1 = d1 - 2 * d1 / (4 * n1 + 1);
    if ( d1 <= max_speed ) { //reach the max speed
      d1 = max_speed;
      m1_rampUpStepCount = m1_stepCount;
    }
    if ( m1_stepCount >= (abs_target_m1_step / 2 )) { //should start decel
      m1_rampUpStepCount = m1_stepCount;
    }
  }
  else if (m1_stepCount > (abs_target_m1_step - m1_rampUpStepCount)) { //decel
    n1 --;
    d1 = d1 * (4 * n1 + 1) / (4 * n1 - 1);
  }

  OCR1A = d1*factor11;
}

ISR(TIMER3_COMPA_vect)
{
  if ( m2_stepCount < abs_target_m2_step )
  {
    digitalWrite(Y_STP, HIGH);
    __asm__ __volatile__ (
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop");  // Wait for step pulse
    digitalWrite(Y_STP, LOW);
    m2_stepCount ++;
    m2_stepPosition += m2_dir;
  } else {
    m2_moveDone = true;
    TIMSK3 &= ~(1 << OCIE1A);
  }

  if ( m2_rampUpStepCount == 0 ) {
    n2 ++;
    d2 = d2 - int( 2 * float(d2) / (4 * float(n2) + 1));
    //d2 = d2 - 2 * d2 / (4 * n2 + 1);
    if ( d2 <= max_speed ) {
      d2 = max_speed;
      m2_rampUpStepCount = m2_stepCount;
    }
    if ( m2_stepCount >= (abs_target_m2_step / 2) ) {
      m2_rampUpStepCount = m2_stepCount;
    }
  }
  else if (m2_stepCount > (abs_target_m2_step - m2_rampUpStepCount)) {
    n2 --;
    d2 = d2 * (4 * n2 + 1) / (4 * n2 - 1);
  }

  OCR3A = d2*factor22;
}

void moveNSteps( int16_t target_y_mm,int16_t target_x_mm)
{
  int16_t target_m1_step = X_step_per_mm * target_x_mm - Y_step_per_mm * target_y_mm;
  int16_t target_m2_step = -X_step_per_mm * target_x_mm - Y_step_per_mm * target_y_mm;



  digitalWrite(X_DIR, target_m1_step < 0 ? LOW : HIGH);
  digitalWrite(Y_DIR, target_m2_step < 0 ? LOW : HIGH);

  m1_dir = target_m1_step < 0 ? -1 : 1;
  m2_dir = target_m2_step < 0 ? -1 : 1;

  abs_target_m1_step = abs(target_m1_step);
  abs_target_m2_step = abs(target_m2_step);

  d1 = ZERO_SPEED * 0.676;
  d2 = ZERO_SPEED * 0.676;

  OCR1A = d1;
  OCR3A = d2;

  n1 = 0;
  n2 = 0;

  m1_stepCount = 0;
  m2_stepCount = 0;

  m1_rampUpStepCount = 0;
  m2_rampUpStepCount = 0;

  m1_moveDone = false;
  m2_moveDone = false;

  factor11 = 1.0;
  factor22 = 1.0;

  TIMSK1 |= (1 << OCIE1A);
  TIMSK3 |= (1 << OCIE1A);

}

void moveToPosition(int16_t target_x_mm, int16_t target_y_mm )
{
  int16_t diff1;
  int16_t diff2;
  moveNSteps(target_x_mm, target_y_mm);
  while ( !m1_moveDone || !m2_moveDone ) {
    diff1 = abs(abs_target_m1_step) - m1_stepCount;
    diff2 = abs(abs_target_m2_step) - m2_stepCount;
    if ( (diff2 > diff1) && (diff1!=0) ) {
      factor11 = diff2 / diff1;
    } else if((diff2 < diff1) && (diff2 !=0) ) {
      factor22 = diff1 / diff2;
    }else{
      factor11 = 1.0;
      factor22 = 1.0;  
    }
  }
}
