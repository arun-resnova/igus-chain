#include <Encoder.h>             // Library For Rotary Encoder
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <String.h>
#include <TimerOne.h>
#include <EEPROM.h>

#define ENC_1      A1              // Encoder Pin
#define ENC_2      A2              // One of encoder pin should be connected to interrupt for better performance
#define ENC_SW     A3              // Encoder Switch 

#define PAUSE_SW   3               // Pause Switch
#define START_SW   8               // Start Switch
#define RESET_SW   A0               // Reset Switch

#define LIMIT_IN   4

#define SOLENOID_A        5
#define SOLENOID_B        6
#define SOLENOID_C        7

#define IR_IN             2

#define STEP_DIR          9
#define STEP_PUL          10

#define PISTON_1          11
#define PISTON_2          12

#define stepsPerRevoltion   800

Encoder myEnc(ENC_1, ENC_2);      // Initializing Encoder with encoder library
LiquidCrystal_I2C lcd(0x27, 20, 4);   // Initializing LCD using I2C 

//bool first_free=false;
const int LONG_PRESS_TIME  = 3000;                       // defined time for a long press =2000 milliseconds 
const int SHORT_PRESS_TIME = 500; 
static long old_position = 0;

unsigned long oldtime,newtime,diff;
unsigned long microSecnds=1000;
//int rpm;
volatile bool pause_interrupt=false,pi_interrupt=false;
bool runningFlag=0;


unsigned int blocks_per_len,target,offset,increment,offload,block_num=1,target_temp=0;
float piston_1_delay =0, piston_2_delay = 0, solenoid_delay = 0;
int step_count=0;
int password=100;
int lastState = HIGH;  // the previous state from the input pin
int currentState = HIGH;     // the current reading from the input pin
int processNum=0;
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
bool lcd_preset=false;

bool first_pass=true;
void setup() {
  // put your setup code here, to run once:

  pinMode(START_SW, INPUT_PULLUP);  
  pinMode(PAUSE_SW, INPUT_PULLUP);
  pinMode(RESET_SW, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  
  pinMode(LIMIT_IN, INPUT_PULLUP);
  pinMode(IR_IN, INPUT_PULLUP);
 
  pinMode(STEP_PUL, OUTPUT);
  pinMode(STEP_DIR, OUTPUT);

  pinMode(SOLENOID_A, OUTPUT);
  pinMode(SOLENOID_B, OUTPUT);
  pinMode(SOLENOID_C, OUTPUT);
  
  pinMode(PISTON_1, OUTPUT);
  pinMode(PISTON_2,OUTPUT);

  digitalWrite(PISTON_1,HIGH);
  digitalWrite(PISTON_2,HIGH);
  digitalWrite(SOLENOID_A,HIGH);
  digitalWrite(SOLENOID_B,HIGH);
  digitalWrite(SOLENOID_C,HIGH);
  
  attachInterrupt(digitalPinToInterrupt(PAUSE_SW), isr_pause_fun, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_IN), isr_rasp_pi_fun, RISING);
  
  lcd.begin();
  lcd.backlight();

  Serial.begin(9600);
  old_position= myEnc.read();
  lcd.clear();
  lcd_update("","      RESNOVA","","   www.resnova.in");
  delay(5000);
  lcd.clear();
  lcd_update("PRESS HOME","","","");
  while(!reset_sw_input());
  initial_settings(); 
}

void loop() {
  
  if(lcd_preset){
    lcd_preset=false;
    lcd.clear();
    lcd_update("PRESS START","",home_display_status("NUM OF BLOCKS",block_num-1,blocks_per_len),home_display_status("TARGET",target_temp,target));
  }
  if(pause_interrupt){
    runningFlag=false;
    pause_interrupt=false;
    Serial.println("Pause sw input");
    process_pause_func();
  }
  if(reset_sw_input()){
    initial_settings(); 
  }
  if(runningFlag){
    Serial.println("Running");
    fun_process();
  }
  else{
    
    currentState = digitalRead(ENC_SW);
    if(lastState == HIGH && currentState == LOW)        // button is pressed
      pressedTime = millis();
    else if(lastState == LOW && currentState == HIGH) { // button is released
      releasedTime = millis();
      long pressDuration = releasedTime - pressedTime;
      if( pressDuration > LONG_PRESS_TIME ){
        Serial.println("A long press is detected");
        menu_function();
      }
      else if( pressDuration < SHORT_PRESS_TIME ){
        Serial.println("A short press is detected");
        short_press();
      }
    }
    lastState = currentState;

    if(start_sw_input()){
      Serial.println("In start");
      runningFlag=true;
      lcd_clear_line(0);
      lcd_update("RUNNING","","","");
    }
  }
}

void isr_pause_fun(){
  pause_interrupt=true;
}
void isr_rasp_pi_fun(){
  pi_interrupt=true;
}
void releasing_block(){
    while(!pi_interrupt);
    {
       pi_interrupt=false;
       digitalWrite(SOLENOID_B,LOW);
       delay(solenoid_delay*1000);
       digitalWrite(SOLENOID_B,HIGH);
       delay(solenoid_delay*1000);
       Serial.println("Releasing");
       digitalWrite(SOLENOID_A,LOW);
       delay(solenoid_delay*1000);
       digitalWrite(SOLENOID_A,HIGH);
    }
    
}
void plunging_process(){
   Serial.println("Plunging");
   digitalWrite(SOLENOID_C,LOW);
   delay(solenoid_delay*1000);
   digitalWrite(PISTON_1,LOW);
   delay(piston_1_delay*1000);
   digitalWrite(PISTON_1,HIGH);
   delay(piston_1_delay*1000);
   digitalWrite(SOLENOID_C,HIGH);
   delay(solenoid_delay*1000);
}
void incrementing_process(){
    Serial.println("Incrementing");
    digitalWrite(PISTON_2,LOW);
    delay(piston_2_delay*1000);
    step_count=round(increment*((float)stepsPerRevoltion/30.0));
    stepper_inc_dec(1,step_count);
    digitalWrite(PISTON_2,HIGH);
    digitalWrite(SOLENOID_C,LOW);
    delay(solenoid_delay*1000);
    stepper_inc_dec(0,step_count);
    digitalWrite(SOLENOID_C,HIGH);
    delay(solenoid_delay*1000);
    block_num++;
}
void offloading_process(){
    Serial.println("Offloading");
    digitalWrite(PISTON_2,LOW); 
    delay(piston_2_delay*1000);
    step_count=round(offload*((float)stepsPerRevoltion/30.0));
    stepper_inc_dec(1,step_count);
    digitalWrite(PISTON_2,HIGH);
    delay(piston_2_delay*1000);
    digitalWrite(SOLENOID_C,LOW);
    delay(solenoid_delay*1000);
    stepper_inc_dec(0,step_count);
    digitalWrite(SOLENOID_C,HIGH);
    delay(solenoid_delay*1000);
    block_num=1;
    first_pass=true;
    target_temp++;
    lcd_update("","",home_display_status("NUM OF BLOCKS",block_num,blocks_per_len),home_display_status("TARGET",target_temp+1,target));
    home_stepper_motor();
    if(target_temp==target){
       runningFlag=false;
       target_temp=0;
       lcd_update("COMPLETED","PRESS START",home_display_status("NUM OF BLOCKS",blocks_per_len,blocks_per_len),home_display_status("TARGET",target,target));
    }
}
int fun_process(){
  if(pause_interrupt)
    return 0;
  if(!first_pass){
    lcd_update("","",home_display_status("NUM OF BLOCKS",block_num,blocks_per_len),"");
    if(processNum == 0){
        releasing_block();
        if(runningFlag){
          processNum=1;
          if(pause_interrupt)
              return 0;
        }
        else 
          return 0;
    }
    if(processNum ==1){
        plunging_process();
        processNum=2;
        if(pause_interrupt)
          return 0; 
    }
    if(processNum ==2){
        processNum=0;
        if(block_num<blocks_per_len){
          incrementing_process();
        }
        else{
          offloading_process();   
        }
    }       
  }
  else{
    if(pause_interrupt)
      return 0;
    lcd_update("","",home_display_status("NUM OF BLOCKS",block_num,blocks_per_len),home_display_status("TARGET",target_temp+1,target));
    if(processNum == 0){
        Serial.println("Releasing");
        digitalWrite(SOLENOID_A,LOW);
        delay(solenoid_delay*1000);
        digitalWrite(SOLENOID_A,HIGH);
        delay(solenoid_delay*1000);
        releasing_block();
        if(runningFlag){
          processNum=1;
          if(pause_interrupt)
              return 0;
        }
        else 
          return 0;
    }
    if(processNum ==1){
        plunging_process();
       first_pass=false;
       processNum=0;
    }
  }
  return 1;
}

void stepper_inc_dec(bool dir,int steps){
  if(dir)
    digitalWrite(STEP_DIR,HIGH);
  else
    digitalWrite(STEP_DIR,LOW);
  for(int i=0;i<steps;i++){
    digitalWrite(STEP_PUL,LOW);
    delayMicroseconds(microSecnds);
    digitalWrite(STEP_PUL,HIGH);
    delayMicroseconds(microSecnds);
  }
}

/*
FUNCTION   : process_pause_func()
DESCRIPTION : "Pauses every runnig program"
PARAMETERS  : NONE
RETURN TYPE : void 
*/
void process_pause_func(){
  lcd_clear_line(0);
  lcd_update("PAUSED","","",""); 
  delay(100);
}
/*
FUNCTION   : initial_settings()
DESCRIPTION : "Function to call when pause interrupt is pressed"
PARAMETERS  : NONE
RETURN TYPE : void 
*/
void initial_settings(){
  lcd.clear();
  lcd_update("INITIALIZING","","","");
  lcd_preset=true;
  
  EEPROM.get(10,offset);
  if(offset >300 )
    offset=300;
  delay(10);

  EEPROM.get(20,increment);
  if(increment >200 )
    increment=200;
  delay(10);

  EEPROM.get(30,offload);
  if(offload >400 )
    offload=400;
  delay(10);
  
  EEPROM.get(40,piston_1_delay);
 //EEPROM.put(40,0.1);
  if(piston_1_delay >4 )
    piston_1_delay=4;
  delay(10);

  EEPROM.get(50,piston_2_delay);
// EEPROM.put(50,0.1);
  if(piston_2_delay >4 )
    piston_2_delay=4;
  delay(10);

  EEPROM.get(60,solenoid_delay);
// EEPROM.put(60,0.1);
  if(solenoid_delay >4 )
    solenoid_delay=4;
  delay(10);
  
  EEPROM.get(70,blocks_per_len);
  if(blocks_per_len >255 )
    blocks_per_len=255;
  delay(10);
  EEPROM.get(80,target);
  if(target >300 )
    target=300;
  delay(10);
  
  pause_interrupt=false;
  step_count=round(offset*((float)stepsPerRevoltion/30.0));
  Serial.print("step count = ");
  Serial.println(step_count);
  home_stepper_motor();
  
}
void home_stepper_motor(){
  digitalWrite(STEP_DIR,LOW);
  digitalWrite(SOLENOID_C,LOW);
   delay(solenoid_delay*1000);
  
  while(1){
    if(!check_home_sensor()){
      digitalWrite(STEP_PUL,HIGH);
      delayMicroseconds(microSecnds);
      digitalWrite(STEP_PUL,LOW);
      delayMicroseconds(microSecnds);
    }
    else
      break;
  }
  delay(100);
  digitalWrite(STEP_DIR,HIGH);
  for(int i=0;i<step_count;i++){
    digitalWrite(STEP_PUL,HIGH);
    delayMicroseconds(microSecnds);
    digitalWrite(STEP_PUL,LOW);
    delayMicroseconds(microSecnds);
  }
  digitalWrite(SOLENOID_C,HIGH);
  runningFlag=false;
}
void short_press(){
  float option=2;
  int change;
  bool menuExit=false,menuLcd=false;
  lcd.clear();
  lcd_update(" SETTINGS","><<BACK",construct_line_string(" BLOCKS PER LEN",blocks_per_len,1),construct_line_string(" TARGET",target,1));
  while(!menuExit){
    if(menuLcd){
      lcd.clear();
      lcd_update(" SETTINGS"," <<BACK",construct_line_string(" BLOCKS PER LEN",blocks_per_len,1),construct_line_string(" TARGET",target,1));
      menuLcd=false;
      lcd_selection(option);
    }
    if (encoder_change(&option, 1) )
    {
      if (option <2) option = 2;
      if (option >4) option = 4; 
      lcd_selection(option);
    }
    change=(int)option;
    switch(change){
        case 2: return_longpress(&menuExit);break;
        case 3: bpl_edit_menu(&menuLcd);break;
        case 4: target_edit_menu(&menuLcd);break;
      }
  }
  lcd_preset=true;  
}
void bpl_edit_menu(bool * ptr){
  float temp_bpl;
  temp_bpl=blocks_per_len;
  if(common_edit_fun("BLOCKS PER LEN ",&temp_bpl,2,255,1,1)){
      int ret =save_choice_menu();
      if( ret == 3){
        blocks_per_len=temp_bpl;
      }
      else if( ret ==4){
        blocks_per_len=temp_bpl;
        EEPROM.put(70,blocks_per_len);
      }
      *ptr= true; 
  }
}
void target_edit_menu(bool * ptr){
  float temp_target;
  temp_target=target;
  if(common_edit_fun("TARGET ",&temp_target,1,300,1,1)){
      int ret =save_choice_menu();
      if( ret == 3){
        target=temp_target;
      }
      else if( ret ==4){
        target=temp_target;
        EEPROM.put(80,target);
      }
      *ptr= true; 
  }
}

/*
FUNCTION   : long_press()
DESCRIPTION : "Function to control eeprom values"
PARAMETERS  : NONE
RETURN TYPE : void 
*//*
void long_press(){
  float option=2;
  int change;
  bool exitFlag=false,update_lcd=false;
  lcd.clear();
  lcd_update(" SETTINGS","><<BACK"," ENTER PASSWORD"," FORGOT PASSWORD");
  while(!exitFlag){
    if(update_lcd){
      option=2;
      lcd.clear();
      lcd_update(" SETTINGS","><<BACK"," ENTER PASSWORD"," FORGOT PASSWORD");
      update_lcd=false;
      lcd_selection(option);
    }
    if (encoder_change(&option, 1) )
    {
      if (option <2) option = 2;
      if (option >4) option = 4;
      lcd_selection(option);
    }
    change=(int)option;
    switch(change){
        case 2: return_longpress(&exitFlag);break;
        case 3: enter_password(&update_lcd);break;
        case 4: forgot_password(&exitFlag);break;
        
      }
  }
  lcd_preset=true;
}
*/
/*
FUNCTION   : return_longpress(bool *ptr)
DESCRIPTION : "take back to the home page"
PARAMETERS  : NONE
RETURN TYPE : void 
*/
void return_longpress(bool *ptr){
  if (enc_sw_read(0))
  {
    enc_sw_read(1);
    
    *ptr =true;
  }
}
/*
void enter_password(bool * ptr){
  if (enc_sw_read(0))
  {
    enc_sw_read(1);
    float temp_pass=100;
    lcd.clear();
    lcd_update("","PASSWORD","","");
    lcd_update_value_only(temp_pass,1,1);
    
    while(true){
      if (encoder_change(&temp_pass, 1) )
      {
        if (temp_pass <= 100) temp_pass = 100;
        if (temp_pass >= 999) temp_pass = 999;
        Serial.print("EDIT Value: ");
        Serial.println(temp_pass);
        lcd_update_value_only(temp_pass,1,1);
      }
      if(enc_sw_read(0)){
        enc_sw_read(1);
        if(temp_pass== password){
          menu_function();
        }
        else{
          lcd.clear();
          lcd_update("","WRONG PASSWORD","","");
          delay(2000);
        }
        break;   
      }
    }
    *ptr =true;
  }
}*/
void menu_function(){
  float option=2;
  int change;
  bool menuExit=false,menuLcd=false;
  lcd.clear();
  lcd_update(" SETTINGS","><<BACK"," STEPPER SETTINGS"," DELAY SETTINGS");
  while(!menuExit){
    if(menuLcd){
      lcd.clear();
      lcd_update(" SETTINGS"," <<BACK"," STEPPER SETTINGS"," DELAY SETTINGS");
      menuLcd=false;
      lcd_selection(option);
    }
    if (encoder_change(&option, 1) )
    {
      if (option <2) option = 2;
      if (option >4) option = 4;
      lcd_selection(option);
    }
    change=(int)option;
    switch(change){
        case 2: return_longpress(&menuExit);break;
        case 3: stepper_settings_menu(&menuLcd);break;
        case 4: delay_settings_menu(&menuLcd);break;
      }
  }
  lcd_preset=true; 
}
void stepper_settings_menu(bool *ptr){
  if (enc_sw_read(0))
  {
    enc_sw_read(1);
    float option=1;
    int change;
    bool exitFlag=false,lcdFlag=false;
    lcd.clear();
    lcd_update("><<BACK",construct_line_string(" OFFSET",offset,1),construct_line_string(" INCREMENT",increment,1),construct_line_string(" OFFLOAD",offload,1));
    while(!exitFlag){
      if(lcdFlag){
        lcd.clear();
        lcd_update(" <<BACK",construct_line_string(" OFFSET",offset,1),construct_line_string(" INCREMENT",increment,1),construct_line_string(" OFFLOAD",offload,1));
        lcdFlag=false;
        lcd_selection(option);
      }
      if (encoder_change(&option, 1))
      {
        if (option <1) option = 1;
        if (option >4) option = 4;
        lcd_selection(option);
      }
      change=(int)option;
      switch(change){
          case 1: return_longpress(&exitFlag);break;
          case 2: offset_edit_menu(&lcdFlag);break;
          case 3: increment_edit_menu(&lcdFlag);break;
          case 4: offload_edit_menu(&lcdFlag);break;
        }
    }
    *ptr =true;
  }
}
void delay_settings_menu(bool *ptr){
  if (enc_sw_read(0))
  {
    enc_sw_read(1);
    float option=1;
    int change;
    bool exitFlag=false,lcdFlag=false;
    lcd.clear();
    lcd_update("><<BACK",construct_line_string(" PISTON 1 DELAY",piston_1_delay,0),construct_line_string(" PISTON 2 DELAY",piston_2_delay,0),construct_line_string(" SOLENOID DELAY",solenoid_delay,0));
    while(!exitFlag){
      if(lcdFlag){
        lcd.clear();
        lcd_update(" <<BACK",construct_line_string(" PISTON 1 DELAY",piston_1_delay,0),construct_line_string(" PISTON 2 DELAY",piston_2_delay,0),construct_line_string(" SOLENOID DELAY",solenoid_delay,0));        
        lcdFlag=false;
        lcd_selection(option);
      }
      if (encoder_change(&option, 1))
      {
        if (option <2) option = 1;
        if (option >4) option = 4;
        lcd_selection(option);
      }
      change=(int)option;
      switch(change){
          case 1: return_longpress(&exitFlag);break;
          case 2: piston_1_edit_menu(&lcdFlag);break;
          case 3: piston_2_edit_menu(&lcdFlag);break;
          case 4: solenoid_edit_menu(&lcdFlag);break;
        }
    }
    *ptr =true;
  }
}
void offset_edit_menu(bool * ptr){
  float temp_offset;
  temp_offset=offset;
  if(common_edit_fun("OFFSET ",&temp_offset,1,300,1,1)){
      int ret =save_choice_menu();
      if( ret == 3){
        offset=temp_offset;
      }
      else if( ret ==4){
        offset=temp_offset;
        EEPROM.put(10,offset);
      }
      *ptr= true; 
  }
}
void increment_edit_menu(bool * ptr){
  float temp_increment;
  temp_increment=increment;
  if(common_edit_fun("INCREMENT ",&temp_increment,1,200,1,1)){
      int ret =save_choice_menu();
      if( ret == 3){
        increment=temp_increment;
      }
      else if( ret ==4){
        increment=temp_increment;
        EEPROM.put(20,increment);
      }
      *ptr= true; 
  }
}
void offload_edit_menu(bool * ptr){
  float temp_offload;
  temp_offload=offload;
  if(common_edit_fun("OFFLOAD ",&temp_offload,1,400,1,1)){
      int ret =save_choice_menu();
      if( ret == 3){
        offload=temp_offload;
      }
      else if( ret ==4){
        offload=temp_offload;
        EEPROM.put(30,offload);
      }
      *ptr= true; 
  }
}
void piston_1_edit_menu(bool * ptr){
  float temp_p1_delay;
  temp_p1_delay=piston_1_delay;
  if(common_edit_fun("P1 DELAY ",&temp_p1_delay,0.1,4,0.1,0)){
      int ret =save_choice_menu();
      if( ret == 3){
        piston_1_delay=temp_p1_delay;
      }
      else if( ret ==4){
        piston_1_delay=temp_p1_delay;
        EEPROM.put(40,piston_1_delay);
      }
      *ptr= true; 
  }
}
void piston_2_edit_menu(bool * ptr){
  float temp_p2_delay;
  temp_p2_delay=piston_2_delay;
  if(common_edit_fun("P2 DELAY ",&temp_p2_delay,0.1,4,0.1,0)){
      int ret =save_choice_menu();
      if( ret == 3){
        piston_2_delay=temp_p2_delay;
      }
      else if( ret ==4){
        piston_2_delay=temp_p2_delay;
        EEPROM.put(50,piston_2_delay);
      }
      *ptr= true; 
  }
}
void solenoid_edit_menu(bool * ptr){
  float temp_solenoid_delay;
  temp_solenoid_delay=solenoid_delay;
  if(common_edit_fun("OFFLOAD ",&temp_solenoid_delay,0.1,4,0.1,0)){
      int ret =save_choice_menu();
      if( ret == 3){
        solenoid_delay=temp_solenoid_delay;
      }
      else if( ret ==4){
        solenoid_delay=temp_solenoid_delay;
        EEPROM.put(60,solenoid_delay);
      }
      *ptr= true; 
  }
}
/*
bool forgot_password( bool *ptr){
  if(enc_sw_read(0)){
    enc_sw_read(1);
    lcd.clear();
    lcd_update("","ENTER FIRST INPUT","","");
    while(!pause_sw_input()){
      if(start_sw_input() | !digitalRead(ENC_SW)){
        lcd.clear();
        lcd_update("","WRONG SEQUENCE","","");
        *ptr=true;
        pause_interrupt=false;
        delay(2000);
        return 0;
      }
    }
    delay(200);
    lcd.clear();
    lcd_update("","ENTER SECOND INPUT","","");
    while(!start_sw_input()){
      if(pause_sw_input() | !digitalRead(ENC_SW)){
        lcd.clear();
        lcd_update("","WRONG SEQUENCE","","");
        *ptr=true;
        pause_interrupt=false;
        delay(2000);
        return 0;
      }
    }
    delay(200);
    lcd.clear();
    lcd_update("","ENTER THIRD INPUT","","");
    while(!start_sw_input()){
      if(pause_sw_input() | !digitalRead(ENC_SW)){
        lcd.clear();
        lcd_update("","WRONG SEQUENCE","","");
        *ptr=true;
        pause_interrupt=false;
        delay(2000);
        return 0;
      }
    }
    delay(200);
    lcd.clear();
    lcd_update("","ENTER FOURTH INPUT","","");
    while(digitalRead(ENC_SW)){
      if(pause_sw_input() | start_sw_input()){
        lcd.clear();
        lcd_update("","WRONG SEQUENCE","","");
        delay(2000);
        *ptr=true;
        pause_interrupt=false;
        return 0;
      }
    }
    delay(200);
    lcd.clear();
    lcd_update("","ENTER FIFTH INPUT","","");
    while(!pause_sw_input()){
      if(start_sw_input() | !digitalRead(ENC_SW)){
        lcd.clear();
        lcd_update("","WRONG SEQUENCE","","");
        delay(2000);
        *ptr=true;
        pause_interrupt=false;
        return 0;
      }
    }
    delay(200);
    lcd.clear();
    float temp_pass;
    temp_pass=password;
    lcd_update("<< EDIT PASSWORD","","","");
    lcd_update_value_only(temp_pass,1,1);
    while(1){
      if (encoder_change(&temp_pass, 1) )
      {
        if (temp_pass <= 100) temp_pass = 100;
        if (temp_pass >= 999) temp_pass = 999;
        Serial.print("EDIT Value: ");
        Serial.println(temp_pass);
        lcd_update_value_only(temp_pass,1,1);
      }
      if(enc_sw_read(0)){
        enc_sw_read(1);
        lcd.clear();
        lcd_update("","SAVING NEW PASSWORD","","");
        password=temp_pass;
        EEPROM.put(50,password); 
        delay(1000);
        *ptr=true;
        pause_interrupt=false;
        return 1;  
      }
    }
  }
}

*/
/*
void convert_rpm_to_microsec(unsigned int spd){
  unsigned int temp = 1;
  Serial.print("spd : ");
  Serial.println(spd);
  temp *= spd ;
  temp *= (stepsPerRevoltion/100)  ;
  Serial.print("temp : ");
  Serial.println(temp);
  //Serial.print("rpm : ");
  //Serial.println(spd);
  double value;
  value = (60.0*10)/temp;
  Serial.print("value : ");
  Serial.println(value);
  
  microSecnds = (value*1000)/2;
  Serial.print("microseconds : ");
  Serial.println(microSecnds);
}
*/
int save_choice_menu(){
  lcd.clear();
  lcd_update(" CHOOSE",">DON'T SAVE"," SAVE"," SAVE AS DEFAULT");
  float choice=2;
  while(1){
    if (encoder_change(&choice, 1) )
    {
      if (choice <2) choice = 2;
      if (choice >4) choice = 4;
      lcd_selection(choice);
    }
    if (enc_sw_read(0)){
      enc_sw_read(1);
      return choice;
    }
  }
}


/*
FUNCTION   : lcd_selection(int arrow)
DESCRIPTION : "To print the arrow"
PARAMETERS  : 
    int arrow : lcd selection to print the arrow
RETURN TYPE : void 
*/
void lcd_selection(int arrow){
  for(int i=0;i<4;i++)
  {
    lcd.setCursor (0,i);
    lcd.print(" ");
  }
  if(arrow<4){
    lcd.setCursor (0,arrow-1);
    lcd.print(">");
  }
  else{
    lcd.setCursor (0,3);
    lcd.print(">");
  }
}
/*
FUNCTION   : lcd_clear_line(int line)
DESCRIPTION : "To print the arrow"
PARAMETERS  : 
    int arrow : lcd selection to print the arrow
RETURN TYPE : void 
*/
void lcd_clear_line(int line){
  for(int i=0;i<20;i++)
  {
    lcd.setCursor (i,line);
    lcd.print(" ");
  }
}
/*
FUNCTION   : start_sw_input()
DESCRIPTION : "To check whether start sw is pressed"
PARAMETERS  : NONE
RETURN TYPE : bool, return 1 when switch is pressed else return 0 
*/

bool start_sw_input()
{
  if (digitalRead(START_SW) == 0)
  {
    delay(50);
    if (digitalRead(START_SW) == 0)
    {
      while (digitalRead(START_SW) == 0);
      return 1;
    }
  }
  return 0;
}
/*
FUNCTION   : pause_sw_input()
DESCRIPTION : "To check whether start sw is pressed"
PARAMETERS  : NONE
RETURN TYPE : bool, return 1 when switch is pressed else return 0 
*/

bool pause_sw_input()
{
  if (digitalRead(PAUSE_SW) == 0)
  {
    delay(50);
    if (digitalRead(PAUSE_SW) == 0)
    {
      while (digitalRead(PAUSE_SW) == 0);
      return 1;
    }
  }
  return 0;
}
bool reset_sw_input()
{
  if (digitalRead(RESET_SW) == 0)
  {
    delay(50);
    if (digitalRead(RESET_SW) == 0)
    {
      while (digitalRead(RESET_SW) == 0);
      return 1;
    }
  }
  return 0;
}
bool check_home_sensor()
{
  if (digitalRead(LIMIT_IN) == 0)
  {
    delay(100);
    return 1;
  }
  return 0;
}
bool check_ir_sensor()
{
  if (digitalRead(IR_IN) == 0)
  {
    delay(100);
    return 1;
  }
  return 0;
}

/*
FUNCTION   : common_edit_fun(String parameter,int * value,int minimum_value,int maximum_value)
DESCRIPTION : "to edit using encoder ,common function"
PARAMETERS  : NONE
RETURN TYPE : bool, return 1 when switch is pressed else return 0 
*/
bool common_edit_fun(String parameter,float * value,float minimum_value,float maximum_value,float increment,int type){
  if (enc_sw_read(0))
  {
    enc_sw_read(1);
    lcd.clear();
    lcd_update("<< EDIT MODE",parameter,"","");
    lcd_update_value_only(*value,1,type);
    while(1){
      if (encoder_change(value, increment) )
      {
        if (*value <= minimum_value) *value = minimum_value;
        if (*value >= maximum_value) *value = maximum_value;
        Serial.print("EDIT Value: ");
        Serial.println(*value);
        lcd_update_value_only(*value,1,type);
      }
      if(enc_sw_read(0)){
        enc_sw_read(1);
        //init_lcd_update_flag=true;
        return 1;   
      }
    }
  }
  return 0;
}
/*
FUNCTION   : lcd_update_value_only(int value, int line)
DESCRIPTION : "Update a value at the end of the line"
PARAMETERS  : 
  int value : value to be updated
  int line  : on which line value to be updated
RETURN TYPE : NONE 
*/

void lcd_update_value_only(float value, int line,int type){
  String inVal="";
  if(type)
    inVal=String((int)value);
  else
    inVal=String(value);
  for(int i=0;i<6-inVal.length();i++){
      lcd.setCursor(14+i,line);
      lcd.print(" ");
  }
  lcd.setCursor(20-inVal.length(),line);
  lcd.print(inVal);
  
}
/*
FUNCTION    : lcd_update(String line1, String line2)
DESCRIPTION : "To update the LCD display"
PARAMETERS  : 
  String line1  :  String to be printed on the first line  
  String line2  :  String to be printed on the second line  
RETURN TYPE : int , returns number of digits 
*/
void lcd_update(String line1, String line2, String line3, String line4){
  lcd.setCursor (0,0);
  lcd.print(line1);
  lcd.setCursor (0,1);
  lcd.print(line2);
  lcd.setCursor (0,2);
  lcd.print(line3);
  lcd.setCursor (0,3);
  lcd.print(line4);
}

/*
FUNCTION    : encoder_change(unsigned int * input, unsigned int inc_dec_val)
DESCRIPTION : "To update the LCD display"
PARAMETERS  : 
  unsigned int * input      :  input value that change   
  unsigned int inc_dec_val  :  inc or dec value  
RETURN TYPE : bool , return 1 if completes
*/

bool encoder_change(float * input, float inc_dec_val)
{
  
  long new_position = myEnc.read();
  
  if (new_position >= (old_position + 4))
  {
    newtime=millis();
    diff=newtime-oldtime;
    Serial.print(diff);
    Serial.print("    ");
    oldtime=newtime;
    old_position = new_position;
    if(diff < 30){
      *input += inc_dec_val*10;
    }
    else if( diff> 30 & diff <90){
      *input += inc_dec_val*5;
    }
    else{
      *input += inc_dec_val;
    }
    Serial.println(*input);
    return 1;
  }
  else if (new_position <= (old_position - 4))
  {
    newtime=millis();
    diff=newtime-oldtime;
    Serial.print(diff);
    Serial.print("    ");
    oldtime=newtime;
    old_position = new_position;
    if(diff < 30){
      *input -= inc_dec_val*10;
    }
    else if( diff> 30 & diff <90){
      *input -= inc_dec_val*5;
    }
    else{
      *input -= inc_dec_val;
    }
    Serial.println(*input);
    return 1;
  }
  return 0;
}
/*
FUNCTION    : enc_sw_read(bool manual_update)
DESCRIPTION : "To update the LCD display"
PARAMETERS  : 
  unsigned int * input      :  input value that change   
  unsigned int inc_dec_val  :  inc or dec value  
RETURN TYPE : bool , return 1 if completes
*/

bool enc_sw_read(bool manual_update)
{

  static bool state = 0;
  if (manual_update == 1) state = 0;
  if (digitalRead(ENC_SW) == 0)
  {
    delay(50);
    if (digitalRead(ENC_SW) == 0)
    {
      state = !state;
      while (digitalRead(ENC_SW) == 0);
    }
  }
  return state;
}
/*
FUNCTION    : String construct_line_string(String input ,unsigned int value)
DESCRIPTION : "To update the LCD display"
PARAMETERS  : 
  unsigned int * input      :  input value that change   
  unsigned int inc_dec_val  :  inc or dec value  
RETURN TYPE : bool , return 1 if completes
*/

String construct_line_string(String input ,float value,bool type){
  int space=0;
  String newLine="";
  newLine=newLine+input;
  String inVal="";
  if (type)
    inVal=String((int)value);
  else
    inVal=String(value);
  space =20-(input.length()+inVal.length());
  for(int i=0;i<space;i++){
    newLine +=" ";
  }
  newLine+=inVal;
  return newLine;
}
String home_display_status(String input ,unsigned int currentVal,unsigned int finalVal){
  int space=0;
  String newLine="";
  newLine=newLine+input;
  String inVal="";
  inVal+=String(currentVal);
  inVal+="/";
  inVal+=String(finalVal);
  Serial.println(inVal);
  space =20-(input.length()+inVal.length());
  for(int i=0;i<space;i++){
    newLine +=" ";
  }
  newLine+=inVal;
  return newLine;
}
