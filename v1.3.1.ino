#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

int o2_raw;

#include <ADS1X15.h>
#include <EEPROM.h>
struct MyObject{
  float field1;
  byte field2;
  char name[10];
};

#include <avr/wdt.h>


ADS1115 ADS(0x48);

// RELAY PIN ASSIGNMENT
//**************************************************************************
int Sieve_A_Valve = 5; //Defined Pin as Variable
int Sieve_B_Valve = 6; //Defined Pin as Variable

int Fan = 8; //Defined Pin as Variable

// VARIABLE CREATION
//**************************************************************************
unsigned long Relay_Test_Delay; //delay variable creation
unsigned long Startup_Purge_Delay; //delay variable creation

unsigned long Production_Delay; //delay variable creation
unsigned long Flash_Delay; //delay variable creation

int Production_Delay_1; //delay variable creation
int Flash_Delay_1; //delay variable creation

int Production_Delay_2; //delay variable creation
int Flash_Delay_2; //delay variable creation
//**************************************



int pause_button = 2;
int setting_button = 3;
int pause_state;
int prev_pause_state;
int save_second;
int machine_state;
int compressor = 11;
int alarm = 13;
unsigned long currentMillis;
unsigned long seconds;
float seconds_millis;
unsigned long down_seconds;
unsigned long down_seconds_buffer;
unsigned long down_seconds_actual;
unsigned long prev_seconds;
unsigned long seconds_raw;
unsigned long minutes;
unsigned long hours;
int max_hours = 1;
long max_hours_seconds;
unsigned long days, pause_time, total_pause_time;
int elapsed;

unsigned long run_seconds;
unsigned long run_millis;
unsigned long run_millis_prev;
unsigned long run_minutes;
float run_minutes_eeprom;
int run_hours;
unsigned long run_minutes_actual;
unsigned long run_hours_actual;

unsigned long time_millis;
unsigned long time_cycle;
unsigned long prev_time_cycle;

unsigned long o2_time;
unsigned long o2_millis;
unsigned long o2_millis_prev;

unsigned long alarm_millis;
unsigned long alarm_millis_prev;
unsigned long alarm_time;

unsigned long eeprom_millis;
unsigned long eeprom_millis_prev;
unsigned long eeprom_time;

int cycle;
int o2;

int eeAddress = 0;
int eeAddress_seconds = 10;
int maintenance_time = 9000;
void setup() {
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  
  
  cli();                      //stop interrupts 
  //reset tccr 
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
 
  //set prescalar
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  
  //compare
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  
  //set register val
  OCR1A = 31250;              
  sei();                     //Enable back the interrupts

  
  

  while (!Serial) { }
  Serial.println("Begin conversion from millis to readable time");
  pinMode(pause_button, INPUT_PULLUP);
  pinMode(setting_button, INPUT_PULLUP);
  pinMode(alarm, OUTPUT);

  // STARTUP
  //**************************************************************************
  // Serial Port initialization
  Serial.begin(9600);


  // SET PIN MODE FOR PINS IN PROGRAM
  //**************************************************************************
  pinMode(Sieve_A_Valve, OUTPUT);
  pinMode(Sieve_B_Valve, OUTPUT);
  
  pinMode(Fan, OUTPUT);
  pinMode(compressor, OUTPUT);
  digitalWrite(compressor, LOW);
  //  SET DELAY TIMING HERE
  //**************************************************************************
  Relay_Test_Delay = 0;
  Startup_Purge_Delay = 1000;
  Production_Delay = 7500; //7000 - 8000 , 7400 best : 7000 //default 7500
  Flash_Delay = 1000; //best : 800 //default 1000

  // VALVE RELAY TEST SEQUENCE
  //**************************************************************************
  Serial.println("Relay Test Sequence");
  digitalWrite(Sieve_A_Valve, HIGH); //Turn on relay
  delay(Relay_Test_Delay);
  digitalWrite(Sieve_B_Valve, HIGH); //Turn on relay
  delay(Relay_Test_Delay);

  Serial.println("Valve Relay Test Sequence Complete");
  delay(Relay_Test_Delay);


  // STARTUP PURGE
  //**************************************************************************
  Serial.println("Relay Test Sequence");
  digitalWrite(Sieve_A_Valve, HIGH); //Turn on relay
  digitalWrite(Sieve_B_Valve, HIGH); //Turn on relay
  
  delay(Startup_Purge_Delay);


  // FAN CONTROL
  //**************************************************************************
  Serial.println("Program Starting...");
  delay(Relay_Test_Delay);
  digitalWrite(Fan, HIGH);
  Serial.println("Fan Switched On");



 ADS.begin();
 ADS.setGain(0);


 EEPROM.get( eeAddress, run_minutes_eeprom );
 EEPROM.get( eeAddress_seconds, seconds_millis);
 seconds = seconds_millis*1000;
 wdt_enable(WDTO_1S);

 Serial.println(seconds_millis,3);
}

void loop() {
// EEPROM.put(eeAddress_seconds, seconds_millis);
if(machine_state == 0){

  o2_millis_prev = millis();
  prev_time_cycle = millis();
  
  cycle = 0;

  digitalWrite(compressor, LOW);
  digitalWrite(Sieve_A_Valve, LOW);
  digitalWrite(Sieve_B_Valve, LOW);
  
  int setting_state = digitalRead(setting_button);
  if (setting_state==0) {
    lcd.setCursor(16, 2);
    lcd.print("set");
    max_hours = map(analogRead(A0), 0, 1022, 1, 9);
    if (max_hours > 8){
      max_hours = 8;
    }
    seconds = 0;
    EEPROM.put(eeAddress_seconds, 0.f);
    lcd.setCursor(0,3);
    lcd.print("total: h:");

    run_minutes_actual = run_minutes_eeprom*100;
    run_hours_actual = run_minutes_actual/60;
    run_minutes_actual %= 60;
    lcd.setCursor(16, 3);
    lcd.print("m:");
    lcd.setCursor(18, 3);
    lcd.print(run_minutes_actual);
    
    
    if (run_hours_actual < 10){
    lcd.setCursor(9, 3);
    lcd.print(run_hours_actual);  
    lcd.setCursor(10, 3);
    lcd.print("      ");  
    }
    if (run_hours_actual >= 10 && run_hours_actual < 100){
    lcd.setCursor(9, 3);
    lcd.print(run_hours_actual);  
    lcd.setCursor(11, 3);
    lcd.print("     "); 
    }
    if (run_hours_actual >= 100 && run_hours_actual < 1000){
    lcd.setCursor(9, 3);
    lcd.print(run_hours_actual);  
    lcd.setCursor(12, 3);
    lcd.print("    "); 
    }
    if (run_hours_actual >= 1000 && run_hours_actual < 10000){
    lcd.setCursor(9, 3);
    lcd.print(run_hours_actual);  
    lcd.setCursor(13, 3);
    lcd.print("   "); 
    }
    if (run_hours_actual > 10000){
    run_hours_actual = 10000;
    lcd.setCursor(9, 3);
    lcd.print(run_hours_actual);  
    lcd.setCursor(14, 3);
    lcd.print("  "); 
    }

        

  } else{
    //o2
    o2_process();

    lcd.setCursor(16, 2);
    lcd.print("   ");
  }

  //lcd.setCursor(15, 3);
  //lcd.print("      ");
  run_millis_prev = millis();
}

if (machine_state == 1) {

run_millis = millis();
run_seconds = (run_millis - run_millis_prev)/1000;
if(run_seconds > 60){
  //read eeprom

   EEPROM.get( eeAddress, run_minutes_eeprom );
 // EEPROM.get(0, run_minutes_eeprom);
  //count up
  run_minutes_eeprom = run_minutes_eeprom + 0.01;
  
 //write eeprom
  EEPROM.put(eeAddress, run_minutes_eeprom);


  EEPROM.put( eeAddress_seconds, (seconds_millis/1000));
  
  //stop timer
  run_millis_prev = millis();
}

eeprom_millis = millis();
eeprom_time = (eeprom_millis - eeprom_millis_prev) / 1000;


  

run_minutes_actual = run_minutes_eeprom*100;
run_hours_actual = run_minutes_actual/60;
run_minutes_actual %= 60;


digitalWrite(compressor, HIGH);
time_millis = millis();
time_cycle = time_millis - prev_time_cycle;

Production_Delay_1 = Production_Delay;
Flash_Delay_1 = Production_Delay + Flash_Delay;
Production_Delay_2 = Production_Delay + Flash_Delay + Production_Delay; 
Flash_Delay_2 = Production_Delay + Flash_Delay + Production_Delay + Flash_Delay;


//Serial.println(Production_Delay_2);
//Serial.println(time_cycle);

//o2
o2_process();

if (time_cycle < Production_Delay_1){
  cycle = 1;
  digitalWrite(Sieve_A_Valve, HIGH); //prev h
  digitalWrite(Sieve_B_Valve, LOW); //prev l

  
}
if (time_cycle > Production_Delay_1 && time_cycle < Flash_Delay_1){
  cycle = 2;
  digitalWrite(Sieve_A_Valve, HIGH); //prev h
  digitalWrite(Sieve_B_Valve, HIGH); //prev h 
}

if (time_cycle > Flash_Delay_1 && time_cycle < Production_Delay_2){
  cycle = 3;
  digitalWrite(Sieve_A_Valve, LOW); //prev l
  digitalWrite(Sieve_B_Valve, HIGH); //prev h
}

if (time_cycle > Production_Delay_2 && time_cycle < Flash_Delay_2){
  cycle = 4;
  digitalWrite(Sieve_A_Valve, HIGH); //prev h
  digitalWrite(Sieve_B_Valve, HIGH); //prev h
}


if(time_cycle > Flash_Delay_2){
  prev_time_cycle = millis();
//  lcd.clear();
}


}

//lcd display

if (run_hours_actual >= maintenance_time){ //>

  lcd.setCursor(0,0);
  lcd.print("MAINTENANCE TIME"); 
} else {
  lcd.setCursor(0,0);
  lcd.print("     OXYCON-I   "); 
  
}






lcd.setCursor(0,2);
lcd.print("set hours on:");
lcd.setCursor(13, 2);
lcd.print(max_hours);

lcd.setCursor(0,1);
lcd.print("ontime=");




//hours
lcd.setCursor(7, 1);  //prev 17,1
lcd.print("h:");
lcd.setCursor(9, 1); //prev 19,1
lcd.print(hours);


//minutes
lcd.setCursor(11, 1);
lcd.print("m:");
if (minutes < 10){
lcd.setCursor(13, 1);
lcd.print(minutes);
lcd.setCursor(14, 1);
lcd.print(" ");
} 
if (minutes >= 10){
lcd.setCursor(13, 1);
lcd.print(minutes);  
}



//seconds
lcd.setCursor(16,1);  //prev 7,1
lcd.print("s:");

if (down_seconds < 10){
lcd.setCursor(18,1); //prev 9,1
lcd.print(down_seconds);
lcd.setCursor(19,1); //prev 10,1
lcd.print(" ");
}

if (down_seconds >= 10){
lcd.setCursor(18,1); //prev 9,1
lcd.print(down_seconds);
}


/*

 Serial.println(Flash_Delay_2);
  if (machine_state == 1){
  time_cycle = (machine_state * elapsed) + time_cycle;    
  Serial.println((Production_Delay_2));
  
  if (time_cycle < Production_Delay_1){
    //CYCLE 1
  //**************************************************************************
  Serial.println("Sieve A Charge / Sieve B Purge");
  digitalWrite(Sieve_A_Valve, HIGH);
  digitalWrite(Sieve_B_Valve, LOW);
  
  //delay(Production_Delay);
  }

  if (time_cycle > Production_Delay_1 && time_cycle < Flash_Delay_1){
  //CYCLE 2
  //**************************************************************************
  Serial.println("Sieve A Charge / Sieve B Purge / Flush/PreCharge");
  digitalWrite(Sieve_A_Valve, HIGH);
  digitalWrite(Sieve_B_Valve, HIGH);
 
  //delay(Flash_Delay) ;
  }

  if (time_cycle > Flash_Delay_1 && time_cycle < Production_Delay_2){
  
    //CYCLE 3
  //**************************************************************************
  Serial.println("Sieve A Charge / Sieve B Purge");
  digitalWrite(Sieve_A_Valve, LOW);
  digitalWrite(Sieve_B_Valve, HIGH);
  
  //delay(Production_Delay);
  }

  if(time_cycle < Production_Delay_2 && time_cycle < Flash_Delay_2){
  //CYCLE 4
  //**************************************************************************
  Serial.println("Sieve A Charge / Sieve B Purge / Flush/PreCharge");
  digitalWrite(Sieve_A_Valve, HIGH);
  digitalWrite(Sieve_B_Valve, HIGH);
 
  //delay(Flash_Delay) ;
  }
  if (time_cycle > Flash_Delay_2){
    time_cycle = 0;
  }
  
  */
//  delay(100000000);

wdt_reset(); 
}



void o2_process (){
  //o2
int16_t val_01 = ADS.readADC_Differential_0_1();  
float volts_01 = ADS.toVoltage(val_01);
o2_raw = map(val_01, 68, 313, 21, 100);
o2 = (o2_raw*0.9) + (o2*0.1);

o2_millis = millis();
o2_time = o2_millis - o2_millis_prev;

if (o2_time > 60000){
  if (o2 < 80){
    alarm_millis = millis();
    alarm_time = alarm_millis - alarm_millis_prev;
    if (alarm_time < 500){
    digitalWrite(alarm, HIGH);  
    } else {
    digitalWrite(alarm, LOW); 
    }

    if (alarm_time > 1000){
     alarm_millis_prev = millis();
    }
    
    lcd.setCursor(13, 3);
    lcd.print("o2 low");
  } else {
    digitalWrite(alarm, LOW);
    lcd.setCursor(13, 3);
    lcd.print("       ");
  }
} else {
    lcd.setCursor(13, 3);
    lcd.print("       ");
}

if (o2 > 99){
  o2 = 99;
}

lcd.setCursor(0, 3);
lcd.print("oxygen: ");
if(o2 < 100){
lcd.setCursor(8, 3);
lcd.print(o2);
lcd.setCursor(10,3);
lcd.print(" ");
}
lcd.setCursor(11, 3);
lcd.print("% ");

}


ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  currentMillis = millis(); 
  max_hours_seconds = max_hours * 3600;
  //Serial.println(down_seconds_actual);
  //seconds = (currentMillis / 1000);
  seconds_raw = currentMillis/1000;
  seconds = ((machine_state * elapsed) + seconds);
  seconds_millis = seconds;
 
  down_seconds_actual = seconds;
  down_seconds = max_hours_seconds - seconds;
  
  //down_seconds_actual = max_hours_seconds - seconds;
//  minutes = (seconds / 60)+minutes;
    minutes = (down_seconds / 60);
//  hours = (minutes / 60)+hours;
    hours = minutes/60;
  days = hours / 24;
  currentMillis %= 1000;
//  seconds %= 60;
  down_seconds %= 60;
 
  minutes %= 60;
  hours %= 24;

  //down_seconds = down_seconds_buffer;
/*
  Serial.print("ts : ");
  Serial.print(currentMillis);
  Serial.print(" seconds : ");
  Serial.print(seconds);  
  Serial.print(" minutes : ");
  Serial.print(minutes);
  Serial.print(" hours : ");
  Serial.print(hours);
  Serial.print(" t : ");
  */
  elapsed = seconds_raw - prev_seconds;
  Serial.print(elapsed);
  Serial.print(" ");
  Serial.print(seconds_millis/1000,4); 
  pause_state = digitalRead(pause_button);
 //if (pause_state != prev_pause_state){
 
 
 if ((pause_state == 0) && ((hours > 0) || (minutes > 0) || (down_seconds > 0)) && (run_hours_actual < maintenance_time)){ //<
    machine_state = 1;
   
 }
 else{
   machine_state = 0;    
    if ((hours > 0) || (minutes> 0) || (down_seconds > 0)){
      digitalWrite(alarm, LOW);
    } else {
      digitalWrite(alarm, HIGH);
      EEPROM.put(eeAddress_seconds, 0.f);
    }
 }
  
 
 prev_pause_state = pause_state;
 prev_seconds = seconds_raw;
 Serial.println(" ");
//
}
