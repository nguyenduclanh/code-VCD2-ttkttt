
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
const int right = 49;     // the number of the pushbutton pin
const int down =  53;      // the number of the LED pin
const int left = 51;     // the number of the pushbutton pin
const int up =  41;      // the number of the LED pin
const int func = 39;     // the number of the pushbutton pin
const int ent =  43;      // the number of the LED pin
const int hot =  47;
const int track_led = 52;
const int GPS_led=50;
const int deploy_led =48;
const int stow_led=46;
const int error_led=38;
// down 52, 50 left, 48 right, 44 enter, 42 up, 40 func, hot 38, gps 53, track 51, error 49, start 47, stop 46
// variables will change:
int timer;
int right_state = 0,right_old=0, right_val = 0 ;         // variable for reading the pushbutton status
int down_state = 0, down_old=0, down_val=0;
int left_state = 0, left_old=0, left_val=0;
int up_state=0, up_old=0, up_val=0;
int func_state=0, func_old=0, func_val=0;
int ent_state=0, ent_old=0, ent_val=0;
int hot_state=0, hot_old=0, hot_val=0;
int n=0, m=0, l=0;
int k=0, o=0, p=0;
byte current_sat;
float stop_stt;
float rx_power=0;
float Lat, Long;
float el, azi;
float mode, old_mode, modeACK; //1 deploy; 2 stow; 3: manual, 4: stop;
float sat_state=2, sat_oldstate;
String String1, Stringmo, StringLo, StringLa;
void setup()
{
 Serial1.begin(4800); //connect to 
 Serial2.begin(4800); //connect to modem
 Serial3.begin(4800);
 Serial.begin(4800);
  pinMode(right, INPUT);
  pinMode(left, INPUT);
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(func, INPUT);
  pinMode(ent, INPUT);
  pinMode(hot, INPUT);
  pinMode(track_led, OUTPUT);
  pinMode(GPS_led, OUTPUT);
  pinMode(deploy_led, OUTPUT);
  pinMode(stow_led, OUTPUT);
  pinMode(error_led, OUTPUT);
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.print("PLEASE WAIT!");
  delay(1000);
  lcd.clear();
  delay(1000);
  login_modem();
  read_modem_data();
  delay(100);
  read_modem_data();
   delay(100);
  read_modem_data();
  current_sat=EEPROM.read(0);
  Serial.print("current sat");
  Serial.print(current_sat);
  digitalWrite(track_led, LOW);
  digitalWrite(GPS_led, LOW);
  digitalWrite(deploy_led, LOW);
  digitalWrite(stow_led, LOW);
  digitalWrite(error_led, LOW);
}

void loop() {
loop_func();
}
void keyscan(void)
{
right_state= digitalRead(right);
if(right_state== HIGH)
{
delay(20);
//right_state= digitalRead(right);
right_old=1;
}
if(right_old&&(right_state==LOW))
{
right_old=0;
delay(20);
right_val+=1;
Serial.println("right_key");
}
left_state= digitalRead(left);
if(left_state== HIGH)
{
delay(20);
left_old=1;
}
if(left_old&&(left_state==LOW))
{
left_old=0;
delay(20);
left_val+=1;
Serial.println("left_key");
}
up_state= digitalRead(up);
if(up_state== HIGH)
{
delay(20);
up_old=1;
}
if(up_old&&(up_state==LOW))
{
up_old=0;
delay(20);
up_val+=1;
Serial.println("up_key");
nhanphimup:up_state=digitalRead(up); 
if(up_state==LOW)
{
delay(20);
if(up_state==LOW) {timer+=1; //Serial.println(timer);
if(timer>10) 
{
  up_val+=1;
Serial.println(up_val);
} 
goto nhanphimup;} 
if(timer>10) 
{
  up_val+=1;
Serial.println(up_val);
goto nhanphimup;
}  
}
if(up_state==HIGH);
{timer=0;}
}
down_state= digitalRead(down);
if(down_state== HIGH)
{
delay(20);
down_old=1;
}
if(down_old&&(down_state==LOW))
{
down_old=0;
delay(20);
down_val+=1;
Serial.println("down_key");
nhanphimdown:down_state=digitalRead(down); 
if(up_state==LOW)
{
delay(20);
if(up_state==LOW) {timer+=1; goto nhanphimdown;} 
if(timer>10) 
{
  down_val+=1; 
goto nhanphimdown;
Serial.print(down_val);
}
}
if(down_state==HIGH);
{timer=0;}

}
func_state= digitalRead(func);
if(func_state== HIGH)
{
delay(20);
func_old=1;
}
if(func_old&&(func_state==LOW))
{
func_old=0;
delay(20);
func_val+=1;
Serial.println("func_key");
}
ent_state= digitalRead(ent);
if(ent_state== HIGH)
{
delay(20);
ent_old=1;
}
if(ent_old&&(ent_state==LOW))
{
ent_old=0;
delay(20);
ent_val+=1;
Serial.println("ent_key");
}
hot_state= digitalRead(hot);
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
stop_stt=1;
Serial.println("stop_key");
}
}
void main_screen(void)
{
current_sat=EEPROM.read(0);
Serial.print("current_sat");
Serial.print(current_sat);
lcd.setCursor(0,0);
if(current_sat==1)
lcd.print ("   VINASAT-1   ");
if(current_sat==2)
lcd.print ("   VINASAT-2   ");
lcd.setCursor(0,1);
lcd.print(">T.khai   >T.hoi");
keyscan();
while(right_val>0&right_val%2==1&func_val%3==0&stop_stt==0)
{
ent_val=0;
//delay(1000);
//lcd.setCursor(0,0);
//lcd.print("                ");
keyscan();
lcd.setCursor(0,1);
keyscan();
if(func_val%3!=0) break;
lcd.print("                ");
for(int i=0; i<5; i++)
{
keyscan();
if(ent_val>0) break;
stow_label: if(func_val%3!=0) {ent_val=0; break;}
}
keyscan();
lcd.setCursor(0,1);
lcd.print("        >T.hoi");
for(int i=0; i<5; i++)
{
keyscan();
if(ent_val>0) break;
if(func_val%3!=0) {ent_val=0; break;}
}
if(ent_val>0)
{
if(func_val%3!=0) break;
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
//lcd.print("  Dang T.khai  ");
}
while(ent_val>0&stop_stt==0)
{
if(func_val%3!=0) {ent_val=0; break;}
lcd.setCursor(0,1);
keyscan();
//lcd.print("                ");
lcd.print("  Dang T.hoi     ");
Serial.println("Dang T.hoi");
digitalWrite(stow_led, HIGH);
mode=2;
send_controlboard();
delay(500);
read_controlboard();
delay(500);
while(modeACK==2&&stop_stt==0) 
{ 
lcd.setCursor(0,1);
//lcd.print("                ");
lcd.print("   Da T.hoi   ");
Serial.println("   Da T.hoi   ");
digitalWrite(stow_led, LOW);
keyscan();
}
}
keyscan();
}
deploy_label: while(right_val>0&right_val%2==0&func_val%3==0&stop_stt==0)
{ent_val=0;
keyscan();
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
for(int i=0; i<5; i++)
{
keyscan();
if(ent_val>0) break;
if(func_val%3!=0) {ent_val=0; break;}
}
lcd.setCursor(0,1);
lcd.print(">T.khai            ");
for(int i=0; i<5; i++)
{
keyscan();
if(ent_val>0) break;
if(func_val%3!=0) {ent_val=0; break;}
}
if(ent_val>0)
{
if(func_val%3!=0) break;
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
//lcd.print("  Dang T.khai  ");
}
while(ent_val>0&&stop_stt==0)
{
if(func_val%3!=0) {ent_val=0; break;}
lcd.setCursor(0,1);
keyscan();
//lcd.print("                ");
lcd.print("   Dang T.khai  ");
mode=1;
send_controlboard();
delay(500);
read_controlboard();
delay(500);
digitalWrite(deploy_led, HIGH);
while(modeACK==1&&stop_stt==0) 
{ 
lcd.setCursor(0,1);
//lcd.print("                ");
lcd.print("   Da T.khai    ");
digitalWrite(deploy_led, LOW);
digitalWrite(track_led, HIGH);
keyscan();
}
//ent_val=0;
}
}
}
void sat_select_screen(void)
{
lcd.setCursor(0,0);
lcd.print("   >VINASAT-1");
lcd.setCursor(0,1);
lcd.print("   >VINASAT-2");
keyscan();
Serial.println("right_val");
Serial.println(right_val);
if(right_val>0&right_val%2==0) Serial.println("sao deo chay?");
while(right_val>0&&right_val%2==1&&(func_val-1)%3==0&stop_stt==0)
{
  Serial.print("log while");
ent_val=0;
//delay(1000);
lcd.setCursor(0,1);
lcd.print("                ");
keyscan();
lcd.setCursor(0,0);
keyscan();
lcd.print("                ");
for(int i=0; i<15; i++)
{
keyscan();
if(ent_val>0) break;
if((func_val-1)%3!=0) {ent_val=0; break;}
}
lcd.setCursor(0,0);
lcd.print("    VINASAT-1  ");
for(int i=0; i<15; i++)
{
keyscan();
if(ent_val>0) break;
if((func_val-1)%3!=0) {ent_val=0; break;}
}
lcd.setCursor(0,1);
lcd.print("              ");
keyscan();
Serial.print("ent=1");
if(ent_val>0)
{
 Serial.print("ent=13");
if((func_val-1)%3!=0) {ent_val=0; break;}
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
//lcd.print("  Dang T.khai  ");
}
while(ent_val>0&stop_stt==0)
{
 Serial.print("ent=12");
if((func_val-1)%3!=0) {ent_val=0; break;}
lcd.setCursor(0,0);
keyscan();
lcd.print("    VINASAT-1 ");
EEPROM.write(0,1);
current_sat=EEPROM.read(0);
Serial.print("read");
Serial.print(current_sat);
}
}
while(right_val>0&right_val%2==0&(func_val-1)%3==0&stop_stt==0)
{
Serial.print("log while");
//lcd.clear();
//delay(1000);
lcd.setCursor(0,0);
lcd.print("                ");
keyscan();
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
for(int i=0; i<15; i++)
{keyscan();
if(ent_val>0) break;
if((func_val-1)%3!=0) {ent_val=0; break;}
}
lcd.setCursor(0,1);
lcd.print("    VINASAT-2 ");
for(int i=0; i<15; i++)
{keyscan();
if(ent_val>0) break;
if((func_val-1)%3!=0) {ent_val=0; break;}
}
//keyscan();
if(ent_val>0)
{
if((func_val-1)%3!=0) {ent_val=0; break;}
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
}
while(ent_val>0&stop_stt==0)
{
if((func_val-1)%3!=0) {ent_val=0; break;}
lcd.setCursor(0,1);
keyscan();
//lcd.print("                ");
lcd.print("    VINASAT-2 ");
EEPROM.write(0,2); 
current_sat=EEPROM.read(0);
Serial.print("current sat");
Serial.print(current_sat);
}
}
}
void mode_select_screen(void)
{
right_val=0;
lcd.setCursor(0,0);
lcd.print(">Manual mode");
lcd.setCursor(0,1);
lcd.print(">Automatic mode");
keyscan();
while(right_val>0&right_val%2==1&(func_val-2)%3==0&stop_stt==0)
{
ent_val=0;
//delay(1000);
lcd.setCursor(0,1);
lcd.print("                ");
keyscan();
lcd.setCursor(0,0);
keyscan();
lcd.print("                ");
for(int i=0; i<15; i++)
{keyscan();
if (ent_val==0) break;
if((func_val- 2)%3!=0) {ent_val=0; break;}
}
keyscan();
lcd.setCursor(0,0);
lcd.print("Manual Mode");
for(int i=0; i<15; i++)
{keyscan();
if (ent_val==0) break;
if((func_val-2)%3!=0) {ent_val=0; break;}
}
lcd.setCursor(0,1);
lcd.print("              ");
if(ent_val>0)
{
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
//lcd.print("  Dang T.khai  ");
}
while(ent_val>0&stop_stt==0)
{
if((func_val-2)%3!=0) {ent_val=0; break;}  
lcd.setCursor(0,0);
keyscan();
//lcd.print("                ");
//lcd.print("Manual Mode");
mode=3;
delay(500);
manual_screen();
}
while(right_val>0&right_val%2==0&(func_val-2)%3==0&stop_stt==0)
{
ent_val=0;
//lcd.clear();
//delay(1000);
//lcd.setCursor(0,0);
//lcd.print("                ");
keyscan();
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
for(int i=0; i<10; i++)
{keyscan();
if (ent_val==0) break;
if((func_val-2)%3!=0) {ent_val=0; break;}
}
lcd.setCursor(0,1);
lcd.print("Automatic Mode");
for(int i=0; i<10; i++)
{keyscan();
if (ent_val==0) break;
if((func_val-2)%3!=0) {ent_val=0; break;}
}
keyscan();
if(ent_val>0)
{
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
}
while(ent_val>0&stop_stt==0)
{
if((func_val-2)%3!=0) {ent_val=0; break;}
lcd.setCursor(0,1);
keyscan();
//lcd.print("                ");
lcd.print("Automatic Mode");
}
}
}
}
void read_modem_data(void)
{
retry_read: Serial2.println("rx power");
delay(100);
//Serial2.println("latlong 21 N 106 E");
//Serial2.print("latlong ");
//Serial2.print(Lat);
//Serial2.print(" N ");
//Serial2.print(Long);
//Serial3.print(" E");
//Serial.println("latlong 21 N 106 E");
//Serial.print("latlong ");
//Serial.print(Lat);
//Serial.print(" N ");
//Serial.print(Long);
//Serial.print(" E");
  //delay(3000);
   //Serial1.print("\n");
  // read from port 0, send to port 1:
    //Serial1.println("\n");
    String String1, Stringrx;
  while (Serial2.available()) {
    char inByte = Serial2.read();
    String1 +=inByte;
    Serial.print(inByte);
  }
  Serial.println(String1);
   for(int i=0; i<50; i++)
    {
    for(int k=0;k<50;k++)
    {
    if(String1[i]=='R'&&String1[i+1]=='x'&&String1[i+k-1]=='[')
    {
    Stringrx=String1.substring(i+10,k);
    Serial.println(Stringrx);
     rx_power =Stringrx.toFloat();
    Serial.println(rx_power);
}
    }
    }
    for(int i=0; i<50; i++)
    {
    for(int k=0;k<50;k++)
    {
    if(String1[i]=='U'&&String1[i+1]=='n'&&String1[i+k-1]=='d')
    {
   goto retry_read;
}
    }
    }
}
void read_controlboard(void)
{
while (Serial1.available()) {
    char c = Serial1.read();  //gets one byte from serial buffer
    String1 += c; //makes the string readString
    
  }
  for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='m'&&String1[i+1]=='o'&&String1[i+k-1]=='L'&&String1[i+k]=='A')
    {
    Stringmo=String1.substring(i+7,i+k-1);
    Serial.println(Stringmo);
    Serial.println("modeACK");
    modeACK=Stringmo.toFloat();
    Serial.println(modeACK);
    goto Lat_cal;
    }
    }
    }
    
 Lat_cal: for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='L'&&String1[i+1]=='A'&&String1[i+k-1]=='L'&&String1[i+k]=='O')
    {
    StringLa=String1.substring(i+3,i+k-1);
    Serial.println(StringLa);
    Lat=StringLa.toFloat();
    Serial.println(Lat);
    goto Long_cal;
    }
    }
    }
    Long_cal:
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
    String1="";
    Serial1.end();
    Serial1.begin(4800);
    if(Lat!=0&Long!=0)
{
digitalWrite(GPS_led, HIGH);
}
if(modeACK==2) 
{
digitalWrite(stow_led, LOW);
}
if(modeACK==1) 
{
digitalWrite(deploy_led, LOW);
}
}
void send_controlboard(void)
{
Serial1.print("step=");
Serial1.print(stop_stt);
Serial1.print("mode=");
Serial1.print(mode);
Serial1.print("sat=");
Serial1.print(current_sat);
Serial1.print("rx power=");
Serial1.print(rx_power);
Serial1.print("elevator_angle=");
Serial1.print(el);
Serial1.print("rotation_angle=");
Serial1.print(azi);
Serial1.print("$");
Serial.print("stop=");
Serial.print(stop_stt);
Serial.print("mode=");
Serial.print(mode);
Serial.print("sat=");
Serial.print(current_sat);
Serial.print("rx power=");
Serial.print(rx_power);
Serial.print("elevator_angle=");
Serial.print(el);
Serial.print("rotation_angle=");
Serial.print(azi);
Serial.print("$");
//Serial.println("mode");
//Serial.println(mode);
//Serial.print("sat=");
//Serial.print(current_sat);
read_controlboard();

}
void login_modem(void)
{
 //Serial.begin(9600);
  //Serial2.begin(9600);
  Serial2.println("/n");
  delay(500);
  Serial2.println("/n");
  delay(1500);
  Serial.println("logging modem");
  while (!Serial2) {
  lcd.print("Wait for modem");
  }
delay(1000);
retry: Serial2.println("root");
Serial.println("root");
 delay(1000);
  Serial2.println("iDirect");
Serial.println("iDirect");
  delay(1000);
 Serial2.println("telnet 0");
   delay(1000);
   Serial2.println("user");
  Serial.println("user");
  delay(1000);
 Serial2.println("iDirect");
 Serial.println("iDirect");
  delay(1000);
   String Stringlog, Stringlog1;
  while (Serial2.available()) {
    char inByte = Serial2.read();
    Stringlog +=inByte;
    Serial.print(inByte);
  }
  Serial.println(Stringlog);
   for(int i=0; i<50; i++)
    {
    for(int k=0;k<50;k++)
    {
    if(Stringlog[i]=='i'&&Stringlog[i+1]=='n'&&Stringlog[i+2]=='c')
    {
   lcd.setCursor(0,0);
      lcd.print("                  ");
   lcd.print("Login incorrect");
   goto retry;
}
    }
    }
}
void control_board_data(void)
{
while (Serial1.available()) {
    char c = Serial1.read();  //gets one byte from serial buffer
    String1 += c; //makes the string readString
    
  }
    for(int i=0; i<30; i++)
    {
    for(int k=0;k<30;k++)
    {
    if(String1[i]=='m'&&String1[i+1]=='o'&&String1[i+k-1]=='L'&&String1[i+k]=='A')
    {
    Stringmo=String1.substring(i+7,i+k-1);
    Serial.println(Stringmo);
    modeACK=Stringmo.toFloat();
    Serial.println("modeACK");
    Serial.println(modeACK);
    goto Lat_cal;
    
    //goto end_cal;
}
  }
    }
    Lat_cal:
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
    goto Long_cal;
    }
    }
    }
    Long_cal:
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
}
void manual_screen(void)
{
ent_val=1;
lcd.setCursor(0,0);
lcd.print(">Elevator");
lcd.setCursor(0,1);
lcd.print(">Azimuth");
keyscan();
while((right_val-1)%2==1)
{
if(k<1)
{//delay(20000);
lcd.clear();
k+=1;
}
lcd.setCursor(0,0);
lcd.print("               ");
for(int i=0; i<10; i++)
{keyscan();}
Serial.println("Elevator");
lcd.setCursor(0,0);
lcd.print("Elevator");
for(int i=0; i<10; i++)
{keyscan();}
keyscan();
if(ent_val>1&ent_val%2==0&stop_stt==0)
{
lcd.clear();
Serial.println("clear");
keyscan();
lcd.setCursor(0,0);
lcd.print("El Angle:");
Serial.println("El Angle:");
lcd.setCursor(0,1);
lcd.print(el);
Serial.println(el);
}
while(ent_val>1&&ent_val%2==1&stop_stt==0)
{
keyscan();
//lcd.print("                ");
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
for(int i=0; i<30; i++)
{keyscan();}
lcd.setCursor(0,1);
lcd.print(el);
if(ent_val%2==0) break;
}
while(ent_val>1&&ent_val%2==0&stop_stt==0)
{
for(int i=0; i<5; i++)
{keyscan();}
lcd.setCursor(0,1);
//azi=aziup_val/3;
lcd.print(el+up_val/3-down_val/3);
if(ent_val%2==1) break;
if((right_val-1)%2!=1) break;
}
if(right_val%2!=1) break;
}
while((right_val-1)%2==0&stop_stt==0)
{
if(o<1)
{//delay(20000);
lcd.clear();
o+=1;
}
lcd.setCursor(0,0);
lcd.print("               ");
for(int i=0; i<10; i++)
{keyscan();}
Serial.println("Azimuth");
lcd.setCursor(0,0);
lcd.print("Azimuth");
for(int i=0; i<10; i++)
{keyscan();}
keyscan();
if(ent_val>1&ent_val%2==0)
{
lcd.clear();
Serial.println("clear");
keyscan();
lcd.setCursor(0,0);
lcd.print("Azi Angle:");
Serial.println("El Angle:");
lcd.setCursor(0,1);
lcd.print(azi);
Serial.println(azi);
}
while(ent_val>1&&ent_val%2==1&stop_stt==0)
{
keyscan();
//lcd.print("                ");
lcd.setCursor(0,1);
keyscan();
lcd.print("                ");
for(int i=0; i<30; i++)
{keyscan();}
lcd.setCursor(0,1);
lcd.print(azi);
if(ent_val%2==0) break;
}
while(ent_val>1&&ent_val%2==0&stop_stt==0)
{
for(int i=0; i<5; i++)
{keyscan();}
lcd.setCursor(0,1);
//azi=aziup_val/3;
lcd.print(el+up_val/3-down_val/3);
if(ent_val%2==1) break;
if((right_val-1)%2!=0) break;
}
if((right_val-1)%2!=0) break;
}
}
void loop_func(void)
{ 
  if(stop_stt!=0)
 {
  digitalWrite(track_led, LOW);
  //digitalWrite(GPS_led, LOW);
  digitalWrite(deploy_led, LOW);
  digitalWrite(stow_led, LOW);
  digitalWrite(error_led, LOW);
  ent_val=0;
 right_val=0;
 func_val=0;
 rx_power=0;
 stop_stt=0;
 lcd.clear();
 mode=0;
 modeACK=0;
 read_controlboard();
 }
 ent_val=0;
 right_val=0;
keyscan();
current_sat=EEPROM.read(0);
//send_controlboard();
//while(stop_stt==1&&modeACK!=6)
//{
//rx_power=0;
//send_controlboard();
//}
//if(modeACK==6) {stop_stt=0; modeACK=5;}
Serial.println("send_controlboard");
 if(stop_stt!=0)
 {
   digitalWrite(track_led, LOW);
  //digitalWrite(GPS_led, LOW);
  digitalWrite(deploy_led, LOW);
  digitalWrite(stow_led, LOW);
  digitalWrite(error_led, LOW);
  ent_val=0;
 right_val=0;
 func_val=0;
 rx_power=0;
 stop_stt=0;
 lcd.clear();
 mode=0;
 modeACK=0;
 read_controlboard();
 send_controlboard();
 }
while(stop_stt==0)
{ //rx_power=1;
keyscan();
modeACK=0;
read_controlboard();
send_controlboard();
if (func_val%3==0) { 
if(n<1)
{//delay(20000);
lcd.clear();
Serial.println("lcd.clear");
n+=1;
}
main_screen(); }
if((func_val-1)%3==0) 
{ 
if(m<1)
{//delay(20000);
lcd.clear();
m+=1;
}
sat_select_screen();}
if((func_val-2)%3==0) 
{ 
if(l<1)
{//delay(20000);
lcd.clear();
l+=1;
}
mode_select_screen();}
}
}
