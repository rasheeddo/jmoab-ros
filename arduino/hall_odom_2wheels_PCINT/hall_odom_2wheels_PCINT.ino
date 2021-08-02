#include <ArduinoJson.h>

byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;
unsigned long counter_1, counter_2, counter_3, counter_4, current_time;
unsigned long last_time_fall_1, last_time_fall_2, last_time_fall_3, last_time_fall_4;
const byte ledPin = 13;
volatile byte led_state = LOW;
int magnets = 8;
/////////////////////////////////
/////////// Left wheel //////////
/////////////////////////////////
bool ch1_fall = false;
bool ch2_fall = false;
bool fwd_dir_L = true;
int count_L = 0;
int last_count_L = 0;
volatile float rpm_L = 0.0;
volatile float prev_rpm_L = 0.0;
float time_passed_L;
float first_count_time_L;
float hall_timeout_L;

//////////////////////////////////
/////////// Right wheel //////////
//////////////////////////////////
bool ch3_fall = false;
bool ch4_fall = false;
bool fwd_dir_R = true;
int count_R = 0;
int last_count_R = 0;
volatile float rpm_R = 0.0;
volatile float prev_rpm_R = 0.0;
float time_passed_R;
float first_count_time_R;
float hall_timeout_R;
float out_rpm_L;
float out_rpm_R;

////////////////////////////////////////////
/////////////// JSON object ////////////////
////////////////////////////////////////////
const size_t CAPACITY = JSON_OBJECT_SIZE(4);
StaticJsonDocument<CAPACITY> doc;

void setup() {
  
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  pinMode(ledPin, OUTPUT);
  
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.                                               
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change. 

  Serial.begin(115200); 

}

void loop() {

  if (fwd_dir_L == 0){
    out_rpm_L = -rpm_L;
  } else{
    out_rpm_L = rpm_L;
  }

  if (fwd_dir_R == 0){
    out_rpm_R = -rpm_R;
  } else{
    out_rpm_R = rpm_R;
  }

  digitalWrite(ledPin, led_state);
  
  //Serial.print("out_rpm_L: ");
  //Serial.print(out_rpm_L);
  //Serial.print(" | out_rpm_R: ");
  //Serial.println(out_rpm_R);

  doc["rpm_L"] = out_rpm_L;
  doc["rpm_R"] = out_rpm_R;
  doc["count_L"] = count_L;
  doc["count_R"] = count_R;
  serializeJson(doc, Serial);
  Serial.println();

  // timeout if last trigger is too long then set to 0 rpm_L
  hall_timeout_L = (micros() - last_time_fall_1)/1000000.0;
  hall_timeout_R = (micros() - last_time_fall_3)/1000000.0;
  // this timeout depends on what is the slowest we can read
  // 1.2~1.5 seconds could read around 4~5 rpm_L minimum
  if (hall_timeout_L > 1.2){
    rpm_L = 0.0;
    prev_rpm_L = 0.0;
    count_L = 0;
  }

  if (hall_timeout_R > 1.2){
    rpm_R = 0.0;
    prev_rpm_R = 0.0;
    count_R = 0;
  }

  
  delay(10);

}

ISR(PCINT0_vect){

  current_time = micros();

  //////////////////////////
  /////////// D8 ///////////
  //////////////////////////
  if (PINB & B00000001){
    if (last_CH1_state == 0){
      last_CH1_state = 1;
      led_state = !led_state;
      ch1_fall = false;
    }
  }
  else if(last_CH1_state == 1){
    last_CH1_state = 0;
     led_state = !led_state;
     last_time_fall_1 = current_time;
     ch1_fall = true;
     
     //////////////////////////
     /////// rpm_L cal //////////
     //////////////////////////
     count_L += 1;
     if (count_L == 1){
      first_count_time_L = current_time;
     } else if (count_L > 1 && count_L <= 9){
      // Get time_passed_L count from the first magnet 
      time_passed_L = (current_time - first_count_time_L)/1000000.0;
      // Calculate rpm_L according to how many magnets it passed...
      rpm_L = (float(count_L-1)/magnets)/(time_passed_L/60.0);
      prev_rpm_L = rpm_L;
     } else if (count_L >= (magnets+1)){
      count_L = 0;
     } else{
      rpm_L = prev_rpm_L;
     }
     
  }
  
  //////////////////////////
  /////////// D9 ///////////
  //////////////////////////
  if (PINB & B00000010){
    if (last_CH2_state == 0){
      last_CH2_state = 1;
      //led_state = !led_state;
      ch2_fall = false;
    }
  }
  else if(last_CH2_state == 1){
    last_CH2_state = 0;
    //led_state = !led_state;
    last_time_fall_2 = current_time;
    ch2_fall = true;
  }

  //////////////////////////
  /////////// D10 ///////////
  //////////////////////////
  if (PINB & B00000100){
    if (last_CH3_state == 0){
      last_CH3_state = 1;
      //led_state = !led_state;
      ch3_fall = false;
    }
  }
  else if(last_CH3_state == 1){
    last_CH3_state = 0;
     //led_state = !led_state;
     last_time_fall_3 = current_time;
     ch3_fall = true;
     
     //////////////////////////
     /////// rpm_R cal //////////
     //////////////////////////
     count_R += 1;
     if (count_R == 1){
      first_count_time_R = current_time;
     } else if (count_R > 1 && count_R <= 9){
      // Get time_passed_R count from the first magnet 
      time_passed_R = (current_time - first_count_time_R)/1000000.0;
      // Calculate rpm_L according to how many magnets it passed...
      rpm_R = (float(count_R-1)/magnets)/(time_passed_R/60.0);
      prev_rpm_R = rpm_R;
     } else if (count_R >= (magnets+1)){
      count_R = 0;
     } else{
      rpm_R = prev_rpm_R;
     }
     
  }

  //////////////////////////
  /////////// D12 ///////////
  //////////////////////////
  if (PINB & B00010000){
    if (last_CH4_state == 0){
      last_CH4_state = 1;
      //led_state = !led_state;
      ch4_fall = false;
    }
  }
  else if(last_CH4_state == 1){
    last_CH4_state = 0;
    //led_state = !led_state;
    last_time_fall_4 = current_time;
    ch4_fall = true;
  }
  
  ////////////////////////////
  ///////// check_dir_L //////
  ///////////////////////////
  // 2 sensors must be very close,
  // so even magnet is between sensors, it should trig both sensor stage
  if (ch1_fall && ch2_fall){
    if (last_time_fall_1 < last_time_fall_2){
     fwd_dir_L = true;
    }
    else{
      fwd_dir_L = false;
    }
    ch1_fall = false;
    ch2_fall = false;
  }

  ////////////////////////////
  ///////// check_dir_R //////
  ///////////////////////////
  // 2 sensors must be very close,
  // so even magnet is between sensors, it should trig both sensor stage
  if (ch3_fall && ch4_fall){
    if (last_time_fall_3 < last_time_fall_4){
     fwd_dir_R = true;
    }
    else{
      fwd_dir_R = false;
    }
    ch3_fall = false;
    ch4_fall = false;
  }
  
  

  
}
