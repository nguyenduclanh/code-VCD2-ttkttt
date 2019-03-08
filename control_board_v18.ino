#include <PinChangeInt.h>
#include <PID_v1.h>
#include <math.h>
#include <EEPROM.h>
#define ENA              22//9
#define INT1             23
#define INT2             24
#define encodPinA1       3 // 2                      // Quadrature encoder A pin
#define encodPinA2       8 //8                       // Quadrature encoder B pin                            // PWM outputs to L298N H-Bridge motor driver module
#define ENB              9 //10   
#define INT3             25 
#define INT4             26
#define ENC              13 //10   
#define INT5             11 
#define INT6             12
#define encodPinB1       2                     // Quadrature encoder A pin
#define encodPinB2       10   
#define stopPin          47
#include <SPI.h>
#include <Wire.h> 
#include <MCP342x.h>
//=================================== get string ======================================

String Stringmode, Stringsat, Stringrx;
String StringACU= "";
String String1, Stringaz, Stringel, StringLa, StringLo, Stringstop, Stringelman, Stringazman;
float mode, mode_old, sat, rx_power, modeACK=5, stop_stt, elman, azman,mode_old2;
//String String1, Stringaz, Stringel, StringLa, StringLo;
double kpA = 50 , kiA = 12 , kdA = 0.003;             // modify for optimal performance
double inputA = 0, outputA = 0, setpointA = 130, setpoint_maxA =0, outputel=0;
double kpB = 10, kiB = 12 , kdB = 0.002; //kpB = 0, kiB = 0.1 , kdB = 0.0006; bam tinh
double inputB = 0, outputB = 0, setpointB = 130, setpoint_maxB =0, outputaz=0;
double kpC = 50 , kiC = 12 , kdC = 0.003;             // modify for optimal performance
double step_constA=1, step_constB=1;
double inputC = 0, outputC = 0, setpointC = 130, setpoint_maxC =0;
double inputstow;
long temp,n=0, value;
double R,l,z,re=6378.14, Als=35786, rs, ee=0.08182, H, Late,Longe,Ale, phie, Long, B,d, pi=3.1416, Lat, beta_angle, el_angle, azi_angle; //d=distance station to satellite
double current_heading, rotation_angle, elevator_angle,k_resistor=200/90,k_tilt=106/33;
int stopState, hot_old, hot_state, hot_val;

//Serial.println();
//int i, k;
volatile long encoderPosA = 0;
volatile long encoderPosB = 0;
PID myPIDA(&inputA, &outputA, &setpointA, kpA, kiA, kdA, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPIDB(&inputB, &outputB, &setpointB, kpB, kiB, kdB, DIRECT);
PID myPIDC(&inputC, &outputC, &setpointC, kpC, kiC, kdC, DIRECT);
uint8_t address = 0x68;
MCP342x adc = MCP342x(address);
long el_max, pol, po_const = 10, el_max_pos, el_max_pos_en, az_max, az_max_pos, az_max_pos_en, el_const=1, az_const=1;
// Configuration settings
MCP342x::Config config(MCP342x::channel1, MCP342x::oneShot,
		       MCP342x::resolution18, MCP342x::gain1);

// Configuration/status read back from the ADC
MCP342x::Config status;

// Inidicate if a new conversion should be started
bool startConversion = false;
int data=0;
bool ledLevel = false;
void setup() 
{
  String1.reserve(50);
  pinMode(INT1, OUTPUT);
  pinMode(INT2, OUTPUT);
  pinMode(INT3, OUTPUT);
  pinMode(INT4, OUTPUT);
  pinMode(INT5, OUTPUT);
  pinMode(INT6, OUTPUT);
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinA2, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB2, INPUT_PULLUP);  
  pinMode(stopPin, INPUT); 
  TCCR2B=0x04;
  TIMSK2 = 0x01;
   interrupts(); 
  attachInterrupt(1, encoderA, FALLING);               // update encoder position
  attachInterrupt(0, encoderB, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPIDA.SetMode(AUTOMATIC);
  myPIDA.SetSampleTime(1);
  myPIDA.SetOutputLimits(-250,250);
  myPIDB.SetMode(AUTOMATIC);
  myPIDB.SetSampleTime(1);
  myPIDB.SetOutputLimits(-100, 100);
  myPIDC.SetMode(AUTOMATIC);
  myPIDC.SetSampleTime(1);
  myPIDC.SetOutputLimits(-150, 150);
  Serial.begin (4800); 
  Serial1.begin (9600);
 Serial2.begin(4800); 
  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed
  //Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
// Enable power for MCP342x (needed for FL100 shield only)
  //pinMode(9, OUTPUT);
  //digitalWrite(9, HIGH);
  
 // pinMode(led, OUTPUT);
    
  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle
  
  // Check device present
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1)
      ;
      }

  // First time loop() is called start a conversion
  startConversion = true;
   delay(1500);
   Serial.println( "test_read_ACU");
}
unsigned long lastLedFlash = 0;
void loop() {  
//elevator(420); //371= stow(tilt sensor), 90 degree=520
//while(1);
//az_const=0;;
//azimuth(600); //center=512
//while(1);
 //{pwmOutA(255);}
//polarization(512);

// while(1)
// {
//   inputA = analogRead(0) ;
//    Serial.print("inputA");
//  Serial.println(inputA);
// delay(500);
// inputB = analogRead(1) ;
//    Serial.print("inputB");
//  Serial.println(inputB);
//  delay(500);
//  inputC = analogRead(4) ;
//    Serial.print("inputC");
//  Serial.println(inputC);
//  delay(500);
//  receiver_indicator();
//   inputstow=analogRead(2);
//   Serial.print("inputstow");
//   Serial.print(inputstow);
//   
// }
read_ACUboard();
//keyscan();
if((stop_stt==0)&&(mode==2)&&(mode_old2!=2)) // thu hoi
{ 
Stow();// Khi trien khai bo 2 lenh nay, khi thu hoi thi dung 2 lenh nay
if (stopState ==0)
 {
   modeACK=2;
// mode = 0;
  send_ACUboard();
 }
}
if((stop_stt==0)&&(mode==1)&&(mode_old2!=1)) // trien khai
 {  
   stopState=0;
 //azifast(504); //Stow temp
elevator_new(450);
 polarization(706);
//while(1);
// delay(5000);
 send_sensorboard();
calculate_angle();
Serial.println("Vinasat 1");
calculate_angle();
//polarization(750);
polarization(706);
scan(el_angle);
/*
while(1);
elefast(el_angle);
azifast(azi_angle);
scan_max_azi();
delay(1000);
//kpB = 50; kiB = 120 ; kdB = 1;
//az_const=0;
//step_constB=20;
Serial.print("az max");
Serial.print(az_max_pos);
azifast(az_max_pos);
scan_max_el();
Serial.print("el max");
Serial.print(el_max_pos);
//elefast_scan(el_max_pos);
//elevator_new(380);
//elevator_new(450);
//azimuth_new(506);
//elevator_new(372);
while(1);
  inputA = analogRead(0) ;
    Serial.print("inputA");
  Serial.println(inputA);
 delay(500);
 inputB = analogRead(1) ;
    Serial.print("inputB");
  Serial.println(inputB);
  delay(500);
  inputC = analogRead(4) ;
    Serial.print("inputC");
  Serial.println(inputC);
  delay(500);
  receiver_indicator();
  //compare_el();
*/
  modeACK=1; 
  mode = 0;
  send_ACUboard();
  } 
}
ISR(TIMER2_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  if(mode >0) stopState=0;
  if(digitalRead(stopPin)==0) stopState=1;
}
void pwmOutA(int out) {        // to H-Bridge board
 if(stopState==0)
 {
  if (out > 0) {
  digitalWrite(INT2, LOW);
  digitalWrite(INT1, HIGH);
  analogWrite(ENA, out);
  }
  else {
digitalWrite(INT1, LOW);
  digitalWrite(INT2, HIGH);  // drive motor CCW
  analogWrite(ENA, abs(out));
  }
 
 }else
 
  analogWrite(ENA, 0);
}

void encoderA()  {                                     // pulse and direction, direct port reading to save cycles
  if(digitalRead(encodPinA2)==HIGH)   encoderPosA++;             // PINB = status register from 6 to 13 and 6&7 can not use : if(digitalRead(encodPinB1)==HIGH)   count ++;
 if(digitalRead(encodPinA2)==LOW)     encoderPosA--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}
void encoderB()  {                                     // pulse and direction, direct port reading to save cycles
  if(digitalRead(encodPinB2)==HIGH)   encoderPosB++;             // PINB = status register from 6 to 13 and 6&7 can not use : if(digitalRead(encodPinB1)==HIGH)   count ++;
 if(digitalRead(encodPinB2)==LOW)     encoderPosB--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}

void setpoint_cal()
{
set_fun: setpoint_maxA = analogRead(1); //cong tac truot
if(setpoint_maxA >550)
{
digitalWrite(INT1, LOW);
digitalWrite(INT2, HIGH);
analogWrite(ENA, 255);
goto set_fun;
}
else
{
digitalWrite(INT1, HIGH);
digitalWrite(INT2, HIGH);
analogWrite(ENA, 0);
}
}
void pwmOutB(int out1) 
{    
   if(stopState==0)  // to H-Bridge board
  {
   if (out1 > 0) {
    digitalWrite(INT4, LOW);
    digitalWrite(INT3, HIGH);
    analogWrite(ENB, out1);
  
    }
     else {
     digitalWrite(INT3, LOW);
      digitalWrite(INT4, HIGH);  // drive motor CCW
     analogWrite(ENB, abs(out1));
   }
  }
  else analogWrite(ENB, 0);
  
}
void pwmOutC(int out2) 
{  
 if(stopState==0)  // to H-Bridge board
  {  
  if (out2 > 0) {
  digitalWrite(INT5, LOW);
  digitalWrite(INT6, HIGH);
  analogWrite(ENC, out2);
  
  }
  else {
digitalWrite(INT5, HIGH);
  digitalWrite(INT6, LOW);  // drive motor CCW
  analogWrite(ENC, abs(out2));
  }
  }
   else analogWrite(ENC,0);
  
}
void elevator (int el)
{
  setpointA = el; // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpointA");
  Serial.println(setpointA);
  modeACK=5;
  //while(inputA>el-20||inputA<el+20)
//{
el_loop: 
//modeold=mode;
inputA = analogRead(0) ;// data from encoder
keyscan();
read_ACUboard();
//if(modeold!=mode) {inputA=el; outputA=0; modeACK=6;}
//if(mode==3) (setpointA=elman);
receiver_indicator();
  Serial.print("inputA");
  Serial.println(inputA);
  Serial.println("encoderPos");
  Serial.println(encoderPosA);   // monitor motor position
   myPIDA.Compute();  // calculate new output
  Serial.print("outputA");
  Serial.print(outputA);
  if(Wire.available()<4)
  {pwmOutA(outputA);}
  if(outputA<30) {pwmOutA(outputA*3);}
  //}
  inputA = analogRead(0) ;
  Serial.print("inputA");
  Serial.println(inputA);
  Serial.print("outputA");
  Serial.print(outputA);
   // pwmOutA(0);
   
  if(inputA<el-el_const||inputA>el+el_const||abs(outputA)>step_constA)
  {  
   if(stopState==0)
  {goto el_loop;}
  }
  pwmOutA(0);
}
void azimuth (int azi)
{
  setpointB = azi;  // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpointB");
  Serial.println(setpointB);
 //if(inputB>azi-3||inputB<azi+3)
  //{
azi_loop:
inputB = analogRead(1) ;// data from encoder
//read_ACUboard();
//keyscan();
//if(modeold!=mode) {inputB=azi; outputB=0; modeACK=6;}
//if(mode==3) (setpointB=azman);
  Serial.print("inputB");
  Serial.println(inputB);
  Serial.println("encoderPosB");
  Serial.println(encoderPosB);                      // monitor motor position
   myPIDB.Compute();  // calculate new output
  Serial.print("outputB");
  Serial.print(outputB);
  if(Wire.available()<4)
  {pwmOutB(outputB);}
  if(outputB<20) {pwmOutB(outputB*3);}
  inputB = analogRead(1) ;   // data from encoder
  Serial.print("inputB");
  receiver_indicator();
  compare_az();
  if(inputB<azi-az_const||inputB>azi+az_const||abs(outputB)>step_constB)
{
if(stopState==0)
{
goto azi_loop;
}
}
pwmOutB(0);
  //}
}
void send_sensorboard(void)
{
//Serial1.println("?");
//delay(1000);
Serial1.println("?");
while (Serial1.available()) {
    char c = Serial1.read();  //gets one byte from serial buffer
    String1 += c; //makes the string readString
    
  }
 for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='a'&&String1[i+1]=='z'&&String1[i+k-1]=='e')
    {
    Stringaz=String1.substring(i+3,i+k-1);
    Serial.println(Stringaz);
    rotation_angle=Stringaz.toFloat();
    Serial.println(rotation_angle);
    goto el_cal1;
    }
    }
    }
    el_cal1:
    for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='e'&&String1[i+1]=='l'&&String1[i+k-1]=='L'&&String1[i+k]=='A')
    {
    Stringel=String1.substring(i+3,i+k-1);
    Serial.println(Stringel);
    elevator_angle=Stringel.toFloat();
    Serial.println(elevator_angle);
    goto Lat_cal1;
    
    //goto end_cal;
}
  }
    }
    Lat_cal1:
    for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='L'&&String1[i+1]=='A'&&String1[i+k-1]=='L'&&String1[i+k]=='O')
    {
    StringLa=String1.substring(i+3,i+k-1);
    Serial.println(StringLa);
    Lat=StringLa.toFloat();
    Serial.println(Lat);
    goto Long_cal1;
    }
    }
    }
    Long_cal1:
    for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='L'&&String1[i+1]=='O'&&String1[i+k-1]=='$')
    {
    StringLo=String1.substring(i+4,i+k-1);
    Serial.println(StringLa);
    Long=StringLo.toFloat();
    
    Serial.println(Long);
    String1=""; //clear String1
    }
    }
    }
    //end_cal: break; break;
}
void calculate_angle()
{
Serial.println("elevator_angle");
Serial.println(elevator_angle);
Serial.println("rotation_angle");
Serial.println(rotation_angle);
wait_gps: if(elevator_angle==90||elevator_angle==0)
{
Serial.println("Wait for GPS:");
send_sensorboard();
delay(1000);
if (stopState ==0) {goto wait_gps;}
}
if(rotation_angle<180)
{
azi_angle=(512-k_resistor*rotation_angle);
}
if(rotation_angle>180)
{
azi_angle=(512+k_resistor*(360-rotation_angle));
}
//if(rotation_angle>360-136.5)
//{
//azi_angle=(508+k_resistor*abs(rotation_angle-360));
//}
el_angle=478+k_tilt*(62- elevator_angle);
Serial.println("el_angle");
Serial.println(el_angle);
Serial.println("azi_angle");
Serial.println(azi_angle);
}
void Stow()
{
polarization(512);
azifast(505); //Stow temp
elevator_new(380);
inputstow=analogRead(2);
while(inputstow>100)
{
inputstow=analogRead(2);
Serial.println("inputstow");
Serial.println(inputstow);
pwmOutA(-150);
Serial.println("Stowing2");
if (stopState ==1) break;
}
while(inputstow<100)
{
inputstow=analogRead(2);
Serial.println("inputstow");
Serial.println(inputstow);
pwmOutA(-150);
Serial.println("Stowing1");
if (stopState ==1) break;
}
while(inputstow>100)
{
inputstow=analogRead(2);
Serial.println("inputstow");
Serial.println(inputstow);
pwmOutA(-150);
Serial.println("Stowing3");
if (stopState ==1) break;
}
if(inputstow<100)
{
pwmOutA(0);
}
}
void fine_elevator(long i)
{ kpA = 0, kiA = 0 , kdA = 0.00001;
  setpointA = i; // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpoint");
  Serial.println(setpointA);
  inputA = encoderPosA ;   // data from encoder
  Serial.print("input");
  Serial.println(inputA);
  //while(inputA>el-20||inputA<el+20)
//{
fine_el_loop: inputA = encoderPosA ;   // data from encoder
  Serial.print("input");
  Serial.println(inputA);
  Serial.println("encoderPos");
  Serial.println(encoderPosA);  // monitor motor position
  myPIDA.SetOutputLimits(-60,60);
  myPIDB.SetTunings(kpA, kiA, kdA);
   myPIDA.Compute();  // calculate new output
  Serial.print("output");
  Serial.print(outputA);
  pwmOutA(outputA);
  //}
  Serial.print("input");
  Serial.println(inputA);
  if(inputA<i-100||inputA>i+100||abs(outputA)>10)
  {
  goto fine_el_loop;
  }
}
void fine_azimuth(long i)
{
  kpB = 0, kiB = 0.1 , kdB = 0.0006;
 setpointB = i;  // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpoint");
  Serial.println(setpointB);
 //if(inputB>azi-3||inputB<azi+3)
  //{
fine_azi_loop: inputB = encoderPosB ;   // data from encoder
  Serial.print("input");
  Serial.println(inputB);
  Serial.println("encoderPos");
  Serial.println(encoderPosB); 
myPIDB.SetOutputLimits(-100,100);  // monitor motor position
myPIDB.SetTunings(kpB, kiB, kdB);
   myPIDB.Compute();  // calculate new output
  Serial.print("output");
  Serial.print(outputB);
  pwmOutB(outputB);
  //inputB = analogRead(1) ;   // data from encoder
  //Serial.print("input");
  if(inputB<i-50||inputB>i+50||abs(outputB)>15)
{goto fine_azi_loop;}
  //} 
}
void receiver_indicator(void)
{
 //value = 0;
  uint8_t err;

  if (startConversion) {
    Serial.println("Convert");
    err = adc.convert(config);
    if (err) {
      Serial.print("Convert error: ");
      Serial.println(err);
    }
    startConversion = false;
  }
  err = adc.read(value, status);
  if (!err && status.isReady()) { 
    // For debugging purposes print the return value.
    Serial.print("Value: ");
    Serial.println(value);
    Serial.print("Config: 0x");
    Serial.println((int)config, HEX);
    Serial.print("Convert error: ");
    Serial.println(err);
    startConversion = true;
  }

  // Do other stuff here, such as flash an LED
//  if (millis() - lastLedFlash > 50) {
//    ledLevel = !ledLevel;
//    digitalWrite(led, ledLevel);
//    lastLedFlash = millis();
//  }
    
}
void compare_el(void)
{
//el_max_pos=el_angle;
if(el_max<value&&value>40000)
{
el_max=value;
el_max_pos=inputA;
el_max_pos_en=encoderPosA;
}
Serial.println("el_max");
Serial.println(el_max);
Serial.println("el_max_pos");
Serial.println(el_max_pos);
Serial.println("el_max_pos_en");
Serial.println(el_max_pos_en);
}
void compare_az(void)
{
//az_max_pos=azi_angle;
if(az_max<value&&value>40000)
{
az_max=value;
az_max_pos=inputB;
az_max_pos_en=encoderPosB;
}
Serial.println("az_max");
Serial.println(az_max);
Serial.println("az_max_pos");
Serial.println(az_max_pos);
Serial.println("az_max_pos_en");
Serial.println(az_max_pos_en);
}
void elevator_fine(void)
{
//calculate_angle();
//elevator(el_angle);
delay(1000);
if(el_max!=0)
{
elevator(el_max_pos);
fine_elevator(el_max_pos_en);
}
}
void azimuth_fine(void)
{
az_const=1;
az_const=1;
//azimuth(azi_angle);
delay(1000);
if(az_max!=0)
{
azimuth(az_max_pos);
fine_azimuth(az_max_pos_en);
}
}
void polarization(long pol)
{
  po_const=10;
  kpC = 5; kiC = 12 ; kdC = 1;
  setpointC = pol;  // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpointC");
  Serial.println(setpointC);
pol_loop: 
inputC = analogRead(4) ;// data from encoder
  Serial.print("inputC");
  Serial.println(inputC);
   myPIDC.Compute();  // calculate new output
  Serial.print("outputC");
  Serial.print(outputC);
  pwmOutC(outputC);
  inputC = analogRead(4) ;   // data from encoder+
  Serial.print("inputC");
  if(inputC<pol-po_const||inputC>pol+po_const||abs(outputC)>80)
{
if(stopState==0)
{
goto pol_loop;
}
}
pwmOutC(0); 
}
//---------------------------------------------====================== chuong trinh ngat EVENT
void serialEvent2() {
  while (Serial2.available()) 
{    
    char c = Serial2.read();  //gets one byte from serial buffer
    String1 += c; //makes the string readStrin
   // Serial.println(String1);
  
 // if (c == '\n') {
   //   stringComplete = true;
   // }
  }
}


void read_ACUboard(void)
{/*
while (Serial2.available()) 
{
    char c = Serial2.read();  //gets one byte from serial buffer
    String1 += c; //makes the string readStrin
   // Serial.println(String1);
  }
*/
  Serial.println("String1 length");
  Serial.println(String1.length());
  Serial.println(String1);
  for(int i=0; i<40; i++)
    {
    for(int k=0;k<40;k++)
    {
    if(String1[i]=='s'&&String1[i+1]=='t'&&String1[i+k-1]=='m'&&String1[i+k]=='o')
    {
    Stringstop=String1.substring(i+5,i+k-1);
     Serial.println("Stringstop");
    Serial.println(Stringstop);
    stop_stt=Stringstop.toFloat();
     Serial.println("stop_stt=");
    Serial.println(stop_stt);
    goto mode_cal;
    }
    }
    }   
mode_cal:
    mode_old2=mode;
    for(int i=0; i<40; i++)
    { //Serial.print("mode_cal");
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='m'&&String1[i+1]=='o'&&String1[i+k-1]=='s'&&String1[i+k]=='a')
    {
    Stringmode=String1.substring(i+5,i+k-1);
     Serial.println("Stringmode");
    Serial.println(Stringmode);
   
    mode=Stringmode.toFloat();
     Serial.println("mode=");
    Serial.println(mode);
    goto sat_cal;
    }
    }
    }
    sat_cal:
    for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='s'&&String1[i+1]=='a'&&String1[i+k-1]=='r'&&String1[i+k]=='x')
    {
    Stringsat=String1.substring(i+4,i+k-1);
    Serial.println(Stringsat);
    sat=Stringsat.toFloat();
    Serial.println("sat=");
    Serial.println(sat);
    goto rx_cal;
}
  }
    }
    rx_cal:
    for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='r'&&String1[i+1]=='x'&&String1[i+k-1]=='e'&&String1[i+k]=='l')
    {
    Stringrx=String1.substring(i+9,i+k-1);
    Serial.println(Stringrx);
    Serial.println("rx_power=");
    rx_power=Stringrx.toFloat();
    Serial.println(rx_power);
    goto elman_cal;
    }
    }
    } 
elman_cal: 
for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='e'&&String1[i+1]=='l'&&String1[i+k-1]=='r'&&String1[i+k]=='o')
    {
    Stringelman=String1.substring(i+14,i+k-1);
    Serial.println(Stringelman);
    Serial.println("elman=");
    elman=Stringelman.toFloat();
    Serial.println(elman);
    goto azman_cal;
    }
    }
    } 
azman_cal:   
for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='r'&&String1[i+1]=='o'&&String1[i+1]=='t'&&String1[i+k-1]=='$')
    {
    Stringazman=String1.substring(i+14,i+k-1);
    Serial.println(Stringazman);
    Serial.println("azman=");
    azman=Stringazman.toFloat();
    Serial.println(azman);
    }
    }
    }
    String1="";
    //Serial2.end();
    //Serial2.begin(4800);
    send_ACUboard();
}
void command_exec(void)
{
if(mode==3)
{
kpB = 100; kiB = 120 ; kdB = 1;
kpA = 150 ; //50 lau
kiA = 120 ;
kdA = 1;
el_const=5;
az_const=5;
step_constA=50;
step_constB=20;
elevator(elman);
azimuth(azman);
}
if(mode==2)
{
 kpB = 100; kiB = 120 ; kdB = 2;
kpA = 50 ; //50 lau
kiA = 120 ;
kdA = 1;
el_const=5;
az_const=5;
step_constA=50;
step_constB=20;
Stow();
//kpB = 5; kiB = 12 ; kdB = 0.002;
//kpA = 5; kiA = 12 ; kdA = 0.003;
//el_const=5;
//az_const=5;
//step_constA=20;
//step_constB=20;
//Stow();
modeACK=2;
if(stopState==1) {modeACK=6;}
while(mode==5)
{
send_ACUboard();
read_ACUboard();
}
}
if (mode==1&&sat==1)
{
Serial.println("Vinasat 1");
calculate_angle();
polarization(750);
kpB = 1000; kiB = 120 ; kdB = 2;
kpA = 150 ;
kiA = 120 ;
kdA = 2;
el_const=1;
az_const=15;
step_constA=50;
step_constB=50;
elevator(el_angle-1);
azimuth(azi_angle);
//kpB = 5; kiB = 12 ; kdB = 0.002;
//kpA = 5; kiA = 12 ; kdA = 0.003;
//el_const=2;
//az_const=10;
//step_constA=20;
//step_constB=20;
//elevator(550);
//azimuth(730);
scan_max_azi();
delay(1000);
kpB = 50; kiB = 120 ; kdB = 1;
az_const=0;
step_constB=20;
Serial.print("az max");
Serial.print(az_max_pos);
azimuth(az_max_pos);
Serial.println("azimuth(az_max_pos)");
modeACK=1;
if(stopState==1) {modeACK=6;}
while(mode==5)
{
send_ACUboard(); //gui lenh cho den khi ACU nhan duoc
read_ACUboard();
}
}
if (mode==1&&sat==2)
{
calculate_angle();
polarization(250);
kpB = 50; kiB = 120 ; kdB = 2;
kpA = 100 ;
kiA = 120 ;
kdA = 2;
el_const=2;
az_const=15;
step_constA=50;
step_constB=50;
elevator(el_angle);
azimuth(azi_angle);
//kpB = 5; kiB = 12 ; kdB = 0.002;
//kpA = 5; kiA = 12 ; kdA = 0.003;
//el_const=2;
//az_const=10;
//step_constA=20;
//step_constB=20;
//elevator(550);
//azimuth(730);
scan_max_azi();
delay(1000);
kpB = 150; kiB = 120 ; kdB = 2;
az_const=1;
step_constB=20;
Serial.print("az max");
Serial.print(az_max_pos);
azimuth(az_max_pos);
Serial.println("azimuth(az_max_pos)");
modeACK=12;
modeACK=2;
send_ACUboard();
delay(500);
send_ACUboard();
modeACK=5;
}
}
void fine_azimuth_v1(void)
{
step_inc:
azi_angle+=3;
receiver_indicator();
az_max=value;
azimuth(azi_angle);
Serial.print("azi_angle:");
Serial.println(azi_angle);
receiver_indicator();
if(az_max+10<value)
{
goto step_inc;
}
step_dec: 
azi_angle-=3;
receiver_indicator();
az_max=value;
azimuth(azi_angle);
Serial.print("azi_angle:");
Serial.println(azi_angle);
receiver_indicator();
if(az_max+10<value)
{
goto step_inc;
}
}
void fine_elevator_v1(void)
{
step_inc:
el_angle-=1;
receiver_indicator();
el_max=value;
elevator(el_angle+1);
while(Wire.available()<4);
receiver_indicator();
if(el_max+10<value)
{
goto step_dec;
}
step_dec: 
el_angle-=1;
receiver_indicator();
az_max=value;
elevator(el_angle);
receiver_indicator();
if(el_max+10<value)
{
goto step_dec;
}
}
void scan_max_azi()
{
Serial.print("scan_max_azi");
kpB = 100; kiB = 120 ; kdB = 3;
kpA = 100 ;
kiA = 120 ;
kdA = 3;
el_const=1;
az_const=15;
step_constA=50;
step_constB=50;
Serial.print("azi_angle+k_resistor*20");
Serial.print(azi_angle+k_resistor*20);

if(az_max_pos==0)
{
azimuth_new(azi_angle+k_resistor*30);
azimuth_new(azi_angle-k_resistor*30);
}
if(az_max_pos!=0)
{
azimuth_new(az_max_pos+k_resistor*10);
azimuth_new(az_max_pos-k_resistor*10);
}
if(az_max_pos==0)
{
//scan_max_el();
//elevator(el_max_pos);
//azimuth(azi_angle+k_resistor*60);
//azimuth(azi_angle-k_resistor*60);
az_max_pos=azi_angle;
el_max_pos=el_angle;
}
}
void scan_max_el()
{
elevator_new_scan(el_angle+k_tilt*5);
elevator_new_scan(el_angle-k_tilt*5);
}
void send_ACUboard(void)
{
Serial2.print("modeACK");
Serial2.print(modeACK);
Serial2.print("LAT");
Serial2.print(Lat);
Serial2.print("LONG");
Serial2.print(Long);
Serial2.print("$");
}
void refer(void)
{
delay(10000);
if(n<1)
{//delay(20000);
  send_sensorboard();
delay(1000);
n+=1;
}
el_angle=608;
azi_angle=512;

//azimuth(k_resistor*rotation_angle+500);
//elevator(608-k_tilt*(90-elevator_angle));
//azimuth(512);
//elevator(630); //angle=(600-400)/60
//while(1);
//azimuth(717);
//fine_elevator(-75530);
//polarization(512);
//while(1);
calculate_angle();
el_angle=550;
azi_angle=730;
kpB = 50; kiB = 120 ; kdB = 1;
kpA = 50 ;
kiA = 120 ;
kdA = 1;
el_const=3;
az_const=5;
step_constA=50;
step_constB=50;
Serial.println("fine");
//elevator(el_angle);
//azimuth(azi_angle);
Stow();
kpB = 50; kiB = 120 ; kdB = 0.01;
kpA = 50; kiA = 120 ; kdA = 0.01;
el_const=1;
az_const=1;
step_constA=20;
step_constB=20;
Stow();
//elevator(el_angle);
//step_const=20;
//azimuth(azi_angle);
delay(1000);
//azimuth(az_max_pos);
//fine_elevator_v1();
//polarization(750);

//while(1);
//delay(1000);
//azimuth(512); //angle=(750-500)/90
//fine_azimuth(1427);
//inputC=analogRead(3);
//Serial.println("inputC");
//Serial.println(inputC);
//elevator_fine();
//azimuth_fine();
//azimuth(azi_angle);
while(1);
}
void loop_func(void)
{ delay(5000);
read_ACUboard();
  keyscan();
if(stopState==1) {EEPROM.write(0,0);stopState=0; mode=0; delay(5000);az_max=0; az_max_pos=0;}
delay(1000);
  //elevator_tuning();


//elefast(554);
//azifast(512);
//azifast(725);
  //}
//while(1);
mode_old=EEPROM.read(0);
 Serial.println("Stop value");
 Serial.println(hot_val);
 Serial.print("n");
 Serial.print(n);
Serial.println("mode");
Serial.println(mode);
if(n<2)
{//delay(20000);
Serial.print("send_sensorboard");
  send_sensorboard();
delay(1000);
n+=1;
}
if(n<2)
{//delay(20000);
  send_ACUboard();
delay(1000);
n+=1;
}
//if(n<2)
//{//delay(20000);
//  read_ACUboard();
//delay(1000);
//n+=1;
//}
Serial.println("mode");
Serial.println(mode);
Serial.println("mode old");
Serial.println(mode_old);
sat=1;
mode=1;
if(mode!=mode_old&mode!=0)
{
command_execfast();
}
}
void elevator_tuning (int el)
{
  setpointA = el; // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpointA");
  Serial.println(setpointA);
  //while(inputA>el-20||inputA<el+20)
//{
  el_const=20; 
  step_constA=50;
eltun_loop: 
inputA = analogRead(0) ;// data from encoder
receiver_indicator();
  Serial.print("inputA");
  Serial.println(inputA);
  Serial.println("encoderPos");
  Serial.println(encoderPosA);   // monitor motor position
   myPIDA.Compute();  // calculate new output
  Serial.print("outputA");
  Serial.print(outputA);
  if(inputA==el) outputA=0;
  if(stop_stt==1) {inputA=el; outputA=0;}
  if(Wire.available()<4)
  {pwmOutA(outputA);}
  //if(outputA<30) {pwmOutA(outputA*3);}
  //}
  inputA = analogRead(0) ;
  receiver_indicator();
  compare_el();
  Serial.print("inputA");
  Serial.println(inputA);
  Serial.print("outputA");
  Serial.print(outputA);
   // pwmOutA(0);
  if(inputA<el-el_const||inputA>el+el_const||abs(outputA)>step_constA)
  {  
  goto eltun_loop;
  }
  pwmOutA(0);
  el_const=1;
    step_constA=30;
    myPIDA.SetTunings(50, 120, 0.1);
  eltun1_loop:
  inputA = analogRead(0) ;// data from encoder
receiver_indicator();
  Serial.print("inputA");
  Serial.println(inputA);
  Serial.println("encoderPos");
  Serial.println(encoderPosA);   // monitor motor position
   myPIDA.Compute();  // calculate new output
  Serial.print("outputA");
  Serial.print(outputA);
  if(inputA==el) outputA=0;
  if(stop_stt==1) {inputA=el; outputA=0;}
  if(Wire.available()<4)
  {pwmOutA(outputA);}
  //if(outputA<30) {pwmOutA(outputA*3);}
  //}
  inputA = analogRead(0) ;
  receiver_indicator();
  compare_el();
  Serial.print("inputA");
  Serial.println(inputA);
  Serial.print("outputA");
  Serial.print(outputA);
   // pwmOutA(0);
  if(inputA<el-el_const||inputA>el+el_const||abs(outputA)>step_constA)
  {  
  goto eltun1_loop;
  }
  pwmOutA(0);
}
void azimuth_tuning (int azi)
{
  setpointB = azi;  // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpointB");
  Serial.println(setpointB);
 az_const=20; 
  step_constB=30;
azituning_loop:
inputB = analogRead(1) ;// data from encoder
//read_ACUboard();
//if(mode==3) (setpointB=azman);
//if(stop_stt==1) {inputB=azi; outputB=0;}
receiver_indicator();
  Serial.print("inputB");
  Serial.println(inputB);
  Serial.println("encoderPosB");
  Serial.println(encoderPosB);                      // monitor motor position
   myPIDB.Compute();  // calculate new output
  Serial.print("outputB");
  Serial.print(outputB);
   if(inputB==azi) outputB=0;
  if(stop_stt==1) {inputB=azi; outputB=0;}
  if(Wire.available()<4)
  {pwmOutB(outputB);}
  if(outputB<20) {pwmOutB(outputB*3);}
  inputB = analogRead(1) ;   // data from encoder
  Serial.print("inputB");
  receiver_indicator();
  compare_az();
  if(inputB<azi-az_const||inputB>azi+az_const||abs(outputB)>step_constB)
{goto azituning_loop;}
pwmOutB(0);
  //}
  az_const=1; 
  step_constB=10;
  myPIDB.SetTunings(50, 120, 0.05);
azituning1_loop:
inputB = analogRead(1) ;// data from encoder
//read_ACUboard();
//if(mode==3) (setpointB=azman);
//if(stop_stt==1) {inputB=azi; outputB=0;}
receiver_indicator();
  Serial.print("inputB");
  Serial.println(inputB);
  Serial.println("encoderPosB");
  Serial.println(encoderPosB);                      // monitor motor position
   myPIDB.Compute();  // calculate new output
  Serial.print("outputB");
  Serial.print(outputB);
   if(inputB==azi) outputB=0;
  //if(stop_stt==1) {inputB=azi; outputB=0;}
  if(Wire.available()<4)
  {pwmOutB(outputB);}
  if(outputB<20) {pwmOutB(outputB*3);}
  inputB = analogRead(1) ;   // data from encoder
  Serial.print("inputB");
  receiver_indicator();
  compare_az();
  if(inputB<azi-az_const||inputB>azi+az_const||abs(outputB)>step_constB)
{goto azituning1_loop;}
pwmOutB(0);
}

void azifast (int azi)
{
  Serial.println("azimuth_new");
  azimuth_new(azi);
  step_constB=10;
  //myPIDB.SetTunings(50, 120, 0.05);
   Serial.println("azimuth_old");
  azimuth(azi);
}
void elefast_scan (int el)
{
 Serial.println("elevator_new");
 elevator_new_scan(el);
   el_const=0;
    step_constA=20;
    //myPIDA.SetTunings(50, 120, 0.1);
    Serial.println("elevator_old");
  elevator(el);
}
void elefast (int el)
{
 Serial.println("elevator_new");
 elevator_new(el);
   el_const=0;
    step_constA=20;
    //myPIDA.SetTunings(50, 120, 0.1);
    Serial.println("elevator_old");
  elevator(el);
}
void scan_max_azifast()
{
Serial.print("azi_angle+k_resistor*20");
Serial.print(azi_angle+k_resistor*20);

if(az_max_pos==0)
{
azifast(azi_angle+k_resistor*50);
azifast(azi_angle-k_resistor*50);
}
if(az_max_pos!=0)
{
azifast(az_max_pos+k_resistor*30);
azifast(az_max_pos-k_resistor*30);
}
if(az_max_pos==0)
{
//scan_max_el();
//elevator(el_max_pos);
//azimuth(azi_angle+k_resistor*60);
//azimuth(azi_angle-k_resistor*60);
az_max_pos=azi_angle;
el_max_pos=el_angle;
}
}
void command_execfast(void)
{
if(mode==3)
{
elefast(elman);
azifast(azman);
}
if(mode==2)
{
Stowfast();
//kpB = 5; kiB = 12 ; kdB = 0.002;
//kpA = 5; kiA = 12 ; kdA = 0.003;
//el_const=5;
//az_const=5;
//step_constA=20;
//step_constB=20;
//Stow();
EEPROM.write(0,2);
modeACK=2;
if(stopState==1) {EEPROM.write(0,0); mode=0; delay(10000);}
//if(stop_stt==1) {modeACK=6; }
//while(rx_power==1)
//{
//send_ACUboard(); //gui lenh cho den khi ACU nhan duoc
//delay(500);
//read_ACUboard();
//}
}
if (mode==1&&sat==1)
{
Serial.println("Vinasat 1");
calculate_angle();
polarization(750);
elefast(el_angle+2);
azifast(azi_angle);
//kpB = 5; kiB = 12 ; kdB = 0.002;
//kpA = 5; kiA = 12 ; kdA = 0.003;
//el_const=2;
//az_const=10;
//step_constA=20;
//step_constB=20;
scan_max_azifast();
delay(1000);
Serial.print("az max");
Serial.print(az_max_pos);
azifast(az_max_pos);
Serial.println("azifast(az_max_pos)");
modeACK=1;
EEPROM.write(0,1);
if(stopState==1) {EEPROM.write(0,0); mode=0; delay(10000);}
//while(1);
//if(stop_stt==1) {modeACK=6; }
//while(rx_power==1)
//{
//send_ACUboard(); //gui lenh cho den khi ACU nhan duoc
//read_ACUboard();
//}
}
if (mode==1&&sat==2)
{
calculate_angle();
polarization(250);
kpB = 50; kiB = 120 ; kdB = 2;
kpA = 100 ;
kiA = 120 ;
kdA = 2;
el_const=2;
az_const=15;
step_constA=50;
step_constB=50;
elevator(el_angle);
azimuth(azi_angle);
//kpB = 5; kiB = 12 ; kdB = 0.002;
//kpA = 5; kiA = 12 ; kdA = 0.003;
//el_const=2;
//az_const=10;
//step_constA=20;
//step_constB=20;
//elevator(550);
//azimuth(730);
scan_max_azi();
delay(1000);
kpB = 150; kiB = 120 ; kdB = 2;
az_const=1;
step_constB=20;
Serial.print("az max");
Serial.print(az_max_pos);
azimuth(az_max_pos);
Serial.println("azimuth(az_max_pos)");
//if(stop_stt==1) {modeACK=6; }
//while(rx_power==1)
//{
//send_ACUboard(); //gui lenh cho den khi ACU nhan duoc
//read_ACUboard();
//}
}
}
void Stowfast()
{
azifast(512);
el_const=5;
step_constA=50;
elevator(608);
polarization(512);
}
//void read_ACUboard1(void)
//{
//while (Serial2.available()) 
//{
//    char c = Serial2.read();  //gets one byte from serial buffer
//    String1 += c; //makes the string readStrin
//    ///Serial.println(String1);
//  }
//   Serial.println("String1 length");
//    Serial.println(String1.length());
//     Serial.println(String1);
//  for(i=0; i<30; i++)
//    {
//    for(k=0;k<30;k++)
//    {
//    if(String1[i]=='s'&&String1[i+1]=='t'&&String1[i+k-1]=='m'&&String1[i+k]=='o')
//    {
//    Stringstop=String1.substring(i+5,i+k-1);
//     Serial.println("Stringstop");
//    Serial.println(Stringstop);
//    stop_stt=Stringstop.toFloat();
//     Serial.println("stop_stt=");
//    Serial.println(stop_stt);
// //break;
//    }
//    }
//   // if(String1[i]=='s'&&String1[i+1]=='t'&&String1[i+k-1]=='m'&&String1[i+k]=='o')
//    }   
//mode_cal: Serial.println("mode_cal");
//    for(i=0; i<30; i++)
//    {
//    for(k=0;k<30;k++)
//    {
//    if(String1[i]=='m'&&String1[i+1]=='o'&&String1[i+k-1]=='s'&&String1[i+k]=='a')
//    {
//    Stringmode=String1.substring(i+5,i+k-1);
//     Serial.println("Stringmode");
//    Serial.println(Stringmode);
//    mode=Stringmode.toFloat();
//     Serial.println("mode=");
//    Serial.println(mode);
//    //break;
//    //goto sat_cal;
//    }
//    }
//    if(String1[i]=='m'&&String1[i+1]=='o'&&String1[i+k-1]=='s'&&String1[i+k]=='a')
//    break;
//    }
//    sat_cal:
//    for(i=0; i<3; i++)
//    {
//    for(k=0;k<30;k++)
//    {
//    if(String1[i]=='s'&&String1[i+1]=='a'&&String1[i+k-1]=='r'&&String1[i+k]=='x')
//    {
//    Stringsat=String1.substring(i+4,i+k-1);
//    Serial.println(Stringsat);
//    sat=Stringsat.toFloat();
//    Serial.println("sat=");
//    Serial.println(sat);
//    break;
//    //goto rx_cal;
//}
//  }
//   if(String1[i]=='s'&&String1[i+1]=='a'&&String1[i+k-1]=='r'&&String1[i+k]=='x') break;
//    }
//    rx_cal:
//    for(i=0; i<3; i++)
//    {
//    for(k=0;k<30;k++)
//    {
//    if(String1[i]=='r'&&String1[i+1]=='x'&&String1[i+k-1]=='e'&&String1[i+k]=='l')
//    {
//    Stringrx=String1.substring(i+8,i+k-1);
//    Serial.println(Stringrx);
//    Serial.println("rx_power=");
//    rx_power=Stringrx.toFloat();
//    Serial.println(rx_power);
//    break;
//    //goto elman_cal;
//    }
//    }
//     if(String1[i]=='r'&&String1[i+1]=='x'&&String1[i+k-1]=='e'&&String1[i+k]=='l') break;
//    } 
//elman_cal: 
//for(i=0; i<3; i++)
//    {
//    for(k=0;k<30;k++)
//    {
//    if(String1[i]=='e'&&String1[i+1]=='l'&&String1[i+k-1]=='r'&&String1[i+k]=='o')
//    {
//    Stringelman=String1.substring(i+14,i+k-1);
//    Serial.println(Stringelman);
//    Serial.println("elman=");
//    elman=Stringelman.toFloat();
//    Serial.println(elman);
//    break;
//   // goto azman_cal;
//    }
//    }
//     if(String1[i]=='e'&&String1[i+1]=='l'&&String1[i+k-1]=='r'&&String1[i+k]=='o') break;
//    } 
//azman_cal:   
//for(i=0; i<3; i++)
//    {
//    for(k=0;k<30;k++)
//    {
//    if(String1[i]=='r'&&String1[i+1]=='o'&&String1[i+1]=='t'&&String1[i+k-1]=='$')
//    {
//    Stringazman=String1.substring(i+14,i+k-1);
//    Serial.println(Stringazman);
//    Serial.println("azman=");
//    azman=Stringazman.toFloat();
//    Serial.println(azman);
//    break;
//    }
//    }
//    if(String1[i]=='r'&&String1[i+1]=='o'&&String1[i+1]=='t'&&String1[i+k-1]=='$') break;
//    }
//    String1="";
//    Serial.print("String1 length");
//    Serial.print(String1.length());
//    Serial2.end();
//    Serial2.begin(4800);
//    send_ACUboard();
//}
void keyscan(void)
{
hot_state= digitalRead(stopPin);
if(hot_state== HIGH)
{
delay(20);
hot_old=1;
}
if(hot_old&&(hot_state==LOW))
{
hot_old=0;
delay(20);
hot_val+=1;
stopState=1;
}
}
void elevator_new(int el)
{ 
setpointA=el;
Serial.print("setpointA");
Serial.print(setpointA);
inputA=analogRead(0);
Serial.println("inputA");
Serial.print(inputA);
 myPIDA.Compute();
delay(10); 
  myPIDA.Compute(); 
  delay(10); 
   myPIDA.Compute(); 
   delay(10); 
Serial.println("outputA");
Serial.println(outputA); 
while(inputA>setpointA+2||inputA<setpointA-2)
{
//myPIDA.Compute();
keyscan();

  inputA=analogRead(0);
Serial.println("inputA");
Serial.print(inputA);
pwmOutA(outputA/abs(outputA)*200);
//pwmOutA(250);
Serial.println("outputA");
Serial.println(outputA/abs(outputA)*200);
receiver_indicator();
compare_el();
Serial.print("setpointA");
Serial.print(setpointA);
   if(stopState ==1) break;
}
pwmOutA(0);
}
void azimuth_new(int azi)
{ 
 setpointB=azi;
 Serial.print("setpointB");
 Serial.print(setpointB);
 inputB=analogRead(1);
 Serial.println("inputB");
 Serial.print(inputB);
 for(int i=0; i<9; i++)
   {
    Serial.print("setpointB");
    Serial.print(setpointB);
    inputA=analogRead(1);
    Serial.println("inputB");
    Serial.print(inputB);
    myPIDB.Compute();
    outputaz+=outputB;
  }
  Serial.println("outputaz");
  Serial.println(outputaz); 
  while(inputB>setpointB+2||inputB<setpointB-2)
    {
//myPIDA.Compute();
      keyscan();
      inputB=analogRead(1);
      Serial.println("inputB");
      Serial.print(inputB);
      pwmOutB(outputaz/abs(outputaz)*150);
//pwmOutA(250);
      Serial.println("outputaz");
      Serial.println(outputaz);
      Serial.println(outputaz/abs(outputaz)*150);
      receiver_indicator();
      compare_az();
      Serial.print("setpointB");
      Serial.print(setpointB);
      if (stopState ==1) break; // neu phim dung duoc nhan thi thoat
    }
outputaz=0;
pwmOutB(0);
}
void elevator_new_scan(int el)
{ 
setpointA=el;
Serial.print("setpointA");
Serial.print(setpointA);
inputA=analogRead(0);
Serial.println("inputA");
Serial.print(inputA);
for(int i=0; i<9; i++)
{
 Serial.print("setpointA");
Serial.print(setpointA);
inputA=analogRead(0);
Serial.println("inputA");
Serial.print(inputA);
 myPIDA.Compute();
 outputel+=outputA;
}
Serial.println("outputel");
Serial.println(outputel); 
while(inputA>setpointA+2||inputA<setpointA-2)
{
myPIDA.Compute();
receiver_indicator();
compare_el();
inputA=analogRead(0);
Serial.println("inputA");
Serial.print(inputA);
pwmOutA(outputA/abs(outputA)*145);
//pwmOutA(250);
Serial.println("outputel");
Serial.println(outputA/abs(outputA)*145);
receiver_indicator();
compare_el();
Serial.print("setpointA");
Serial.print(setpointA);
if (stopState ==1) break;
}
outputel=0;
pwmOutA(0);
outputA=0;
}
void elevator_scan (int el)
{
  setpointA = el; // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.print("setpointA");
  Serial.println(setpointA);

el_loop: 
//modeold=mode;
inputA = analogRead(0) ;// data from encoder
   myPIDA.Compute();  // calculate new output
  Serial.print("outputA");
  Serial.print(outputA);
  if(Wire.available()<4)
  {pwmOutA(outputA);}
  if(outputA<30) {pwmOutA(outputA*3);}
  //}Z
  inputA = analogRead(0) ;
  Serial.print("inputA");
  Serial.println(inputA);
  Serial.print("outputA");
  Serial.print(outputA);
receiver_indicator();
compare_el();
   
  if(inputA<el-el_const||inputA>el+el_const||abs(outputA)>step_constA)
  {  
   if(stopState==0)
  {goto el_loop;}
  }
  pwmOutA(0);
}
void scan(int el)
{
for(int i=0; i<10; i++)
{
el+=2;
elevator_new_scan(el);
scan_max_azi();
Serial.println("Change to elevator angle:");
Serial.println(el);
delay(1000);
if(az_max>40000){ el_angle=el; break;}

}
Serial.print("az max");
Serial.print(az_max_pos);
azifast(az_max_pos);
scan_max_el();
Serial.print("el max");
Serial.print(el_max_pos);
elefast_scan(el_max_pos+1);
azifast(az_max_pos);

inputA = analogRead(0) ;
    Serial.print("inputA");
  Serial.println(inputA);
 delay(500);
 inputB = analogRead(1) ;
    Serial.print("inputB");
  Serial.println(inputB);
  delay(500);
  inputC = analogRead(4) ;
    Serial.print("inputC");
  Serial.println(inputC);
  delay(500);
  receiver_indicator();
}

