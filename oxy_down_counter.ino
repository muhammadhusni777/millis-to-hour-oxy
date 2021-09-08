#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);


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
unsigned long down_seconds;
unsigned long down_seconds_buffer;
unsigned long down_seconds_actual;
unsigned long prev_seconds;
unsigned long seconds_raw;
unsigned long minutes;
unsigned long hours;
int max_hours = 8;
long max_hours_seconds;
unsigned long days, pause_time, total_pause_time;
int elapsed;

unsigned long time_millis;
unsigned long time_cycle;
unsigned long prev_time_cycle;

int cycle;

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


}

void loop() {

if(machine_state == 0){
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
    
    seconds = 0;
    
    
  } else{
    lcd.setCursor(16, 2);
    lcd.print("   ");
  }

  
}

if (machine_state == 1) {
digitalWrite(compressor, HIGH);
time_millis = millis();
time_cycle = time_millis - prev_time_cycle;

Production_Delay_1 = Production_Delay;
Flash_Delay_1 = Production_Delay + Flash_Delay;
Production_Delay_2 = Production_Delay + Flash_Delay + Production_Delay; 
Flash_Delay_2 = Production_Delay + Flash_Delay + Production_Delay + Flash_Delay;


//Serial.println(Production_Delay_2);
//Serial.println(time_cycle);

if (time_cycle < Production_Delay_1){
  cycle = 1;
  digitalWrite(Sieve_A_Valve, HIGH);
  digitalWrite(Sieve_B_Valve, LOW);

  
}
if (time_cycle > Production_Delay_1 && time_cycle < Flash_Delay_1){
  cycle = 2;
  digitalWrite(Sieve_A_Valve, HIGH);
  digitalWrite(Sieve_B_Valve, HIGH);
}

if (time_cycle > Flash_Delay_1 && time_cycle < Production_Delay_2){
  cycle = 3;
  digitalWrite(Sieve_A_Valve, LOW);
  digitalWrite(Sieve_B_Valve, HIGH);
}

if (time_cycle > Production_Delay_2 && time_cycle < Flash_Delay_2){
  cycle = 4;
  digitalWrite(Sieve_A_Valve, HIGH);
  digitalWrite(Sieve_B_Valve, HIGH);
}


if(time_cycle > Flash_Delay_2){
  prev_time_cycle = millis();
//  lcd.clear();
}


}


//lcd display
lcd.setCursor(0,0);
lcd.print("     OXYCON-I"); 



lcd.setCursor(0,2);
lcd.print("max hours on:");
lcd.setCursor(13, 2);
lcd.print(max_hours);

lcd.setCursor(0,1);
lcd.print("ontime=");
lcd.setCursor(7,1);
lcd.print("s:");

if (down_seconds < 10){
lcd.setCursor(9,1);
lcd.print(down_seconds);
lcd.setCursor(10,1);
lcd.print(" ");
}

if (down_seconds >= 10){
lcd.setCursor(9,1);
lcd.print(down_seconds);

}

lcd.setCursor(12, 1);
lcd.print("m:");
if (minutes < 10){
lcd.setCursor(14, 1);
lcd.print(minutes);
lcd.setCursor(15, 1);
lcd.print(" ");
} 
if (minutes >= 10){
lcd.setCursor(14, 1);
lcd.print(minutes);  
}
lcd.setCursor(17, 1);
lcd.print("h:");
lcd.setCursor(19, 1);
lcd.print(hours);


lcd.setCursor(0, 3);
lcd.print("cycle:");
lcd.setCursor(7, 3);
lcd.print(cycle);
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



}


ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  currentMillis = millis(); 
  max_hours_seconds = max_hours * 3600;
  Serial.println(down_seconds_actual);
  //seconds = (currentMillis / 1000);
  seconds_raw = currentMillis/1000;
  seconds = ((machine_state * elapsed) + seconds);
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
  Serial.print("ts : ");
  Serial.print(currentMillis);
  Serial.print(" seconds : ");
  Serial.print(seconds);  
  Serial.print(" minutes : ");
  Serial.print(minutes);
  Serial.print(" hours : ");
  Serial.print(hours);
  Serial.print(" t : ");
  elapsed = seconds_raw - prev_seconds;
  Serial.print(elapsed);
  Serial.print(" ");
  Serial.print(cycle); 
  pause_state = digitalRead(pause_button);
 //if (pause_state != prev_pause_state){
 
 
 if ((pause_state == 0) && ((hours > 0) || (minutes > 0) || (down_seconds > 0))){
    machine_state = 1;
   
 }
 else{
   machine_state = 0;    
    if ((hours > 0) || (minutes> 0) || (down_seconds > 0)){
      digitalWrite(alarm, LOW);
    } else {
      digitalWrite(alarm, HIGH);
    }
 }
  
 
 prev_pause_state = pause_state;
 prev_seconds = seconds_raw;
 Serial.println(" ");

}
