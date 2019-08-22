#define CHANNELS 6
#define KILLCH 5

uint8_t kill = 1;
int8_t killcnt = 0;

const uint8_t intPin[CHANNELS] = {0, 1, 8, 9, A1, A2};
const uint8_t motorPin[4] = {4, 5, 6, 7};
uint8_t is_up[CHANNELS];
uint32_t timeStamp[CHANNELS];
uint16_t cmd[CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000};
uint16_t motor_cmd[4] = {1000, 1000, 1000, 1000};

// read rc signal data
void pinChangeInt(uint8_t ch) {
  uint32_t temp;
  if(is_up[ch]) {
    if(!digitalRead(intPin[ch])) {
      temp = micros();
      cmd[ch] = temp - timeStamp[ch];
      timeStamp[ch] = temp;
      is_up[ch] = 0;
    }

    // check for kill sw
    // more than three consecutive kill on ==> kill motors
    if(ch == KILLCH) {
      if(cmd[ch] < 1500) {
        killcnt += 1;
        if(killcnt > 3) {
          killcnt = 3;
          kill = 1;
        }
      }
      else {
        killcnt -= 1;
        if(killcnt < 0) {
          killcnt = 0;
          kill = 0;
        }
      }
    }
  }
  else {
    if(digitalRead(intPin[ch])) {
      timeStamp[ch] = micros();
      is_up[ch] = 1;
    }
  }
}

void pinChangeInt0() { pinChangeInt(0); }
void pinChangeInt1() { pinChangeInt(1); }
void pinChangeInt2() { pinChangeInt(2); }
void pinChangeInt3() { pinChangeInt(3); }
void pinChangeInt4() { pinChangeInt(4); }
void pinChangeInt5() { pinChangeInt(5); }

// Output 250kHz PWM on timer TCC0 (6-bit resolution)
void setup() 
{ 
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  for(uint8_t j = 0; j < 4; ++j) {
    // Enable the port multiplexer for the digital pins
    PORT->Group[g_APinDescription[motorPin[j]].ulPort].PINCFG[g_APinDescription[motorPin[j]].ulPin].bit.PMUXEN = 1;  
  }

  // Connect the TCC0 timer to digital output D4-D7 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F; 
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |           // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |       // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;     // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |              // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;        // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                  // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC0_PER = 57600;                             // Set the frequency of the PWM on TCC0 to 250kHz
  while (TCC0->SYNCBUSY.bit.PER);                   // Wait for synchronization
  
  // Set the PWM signal to 1000 ms
  REG_TCC0_CCB0 = 24000;                            // TCC0 CC3 - on D4
  while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
  REG_TCC0_CCB1 = 24000;                            // TCC0 CC3 - on D5
  while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
  REG_TCC0_CCB2 = 24000;                            // TCC0 CC3 - on D6
  while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
  REG_TCC0_CCB3 = 24000;                            // TCC0 CC3 - on D7
  while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  // Enable the port multiplexer for the 4 PWM channels: timer TCC0 outputs

  Serial.begin(115200);

  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(intPin[0]), pinChangeInt0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intPin[1]), pinChangeInt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intPin[2]), pinChangeInt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intPin[3]), pinChangeInt3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intPin[4]), pinChangeInt4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intPin[5]), pinChangeInt5, CHANGE);
}

void pwmWrite(const uint8_t motor_num, uint16_t sig) {
  if(kill) {
    sig = 1000;
  }
  if(sig >= 2000) {
    sig = 1000;
  }
  if(sig < 1000) {
    sig = 1000;
  }
  switch(motor_num) {
    case 1:
      REG_TCC0_CCB0 = 24 * sig;         // TCC0 CCB0 - on D4
      break;
    case 2:
      REG_TCC0_CCB1 = 24 * sig;         // TCC0 CCB1 - on D5
      break;
    case 3:
      REG_TCC0_CCB2 = 24 * sig;         // TCC0 CCB2 - on D6
      break;
    case 4:
      REG_TCC0_CCB3 = 24 * sig;         // TCC0 CCB3 - on D7
      break;
  }
  while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization
}

char incoming_str[32];
uint16_t motor_data;

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t ok_flag = 1;
  for(int i = 0; i < CHANNELS; i++) {
    if(cmd[i] >= 1000) Serial.print(cmd[i]);
    else if(cmd[i] < 1000) Serial.print(1000);
    Serial.print(" ");
  }
  Serial.println("");
  if(Serial.available() > 0) {
    if(Serial.readBytesUntil(' ', incoming_str, 32) == 4) {
      
      // check if first byte is correct
      if(incoming_str[0] > '4') ok_flag = 0;
      
      // check if incoming string is a number
      for(int j = 0; j < 4; ++j) {
        if(!isDigit(incoming_str[j])) {
          ok_flag = 0;
          break;
        }
      }
      if(ok_flag) {
        
        // parse incoming string (xyyy -> motor_num = x, cmd = 1yyy)
        motor_data = (incoming_str[0] - '0') * 1000 + (incoming_str[1] - '0') * 100 + 
                     (incoming_str[2] - '0') * 10 + (incoming_str[3] - '0');

        // motor 4
        if(motor_data >= 4000) {
          motor_cmd[3] = motor_data - 3000;
          pwmWrite(4, motor_cmd[3]);
        }
        // motor 3
        else if(motor_data >= 3000) {
          motor_cmd[2] = motor_data - 2000;
          pwmWrite(3, motor_cmd[2]);
        } 
        // motor 2
        else if(motor_data >= 2000) {
          motor_cmd[1] = motor_data - 1000;
          pwmWrite(2, motor_cmd[1]);
        }
        // motor 1
        else if(motor_data >= 1000) {
          motor_cmd[0] = motor_data;
          pwmWrite(1, motor_cmd[0]);
        }
        else {
          
        }
      }
    }
  }
}
