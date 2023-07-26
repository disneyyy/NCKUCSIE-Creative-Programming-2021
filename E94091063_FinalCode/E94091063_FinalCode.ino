//*******************************************************************************
//    Arduino Robot car Multi-Sensors Control 
//   
//    Arduino IDE 1.6.5 and newer
//    Arduino UNO R3 / Arduino Genuino
//*******************************************************************************
#include <IRremote.h> //用tone的話要改timer1

// L298N 馬達驅動板
// 宣告 MotorA 為右邊
// 宣告 MotorB 為左邊

#define MotorA_I1     8  //宣告 I1 接腳
#define MotorA_I2     9  //宣告 I2 接腳
#define MotorB_I3    10  //宣告 I3 接腳
#define MotorB_I4    11  //宣告 I4 接腳
#define MotorA_PWMA    5  //宣告 ENA (PWM調速) 接腳
#define MotorB_PWMB    6  //宣告 ENB (PWM調速) 接腳
int speedF = 200;
float speedF2 = 200;

// IRremote 紅外線
#define IR_Recv      3   // 宣告紅外線接收接腳
IRrecv irrecv(IR_Recv);  // 宣告 IRrecv 物件來接收紅外線訊號　
decode_results results;  // 宣告解碼變數

// 抓取套件中紅外線遙控器所對應的 IR Code 請勿直接使用本範例中抓到的 IR Code
// 請務必先使用範例資料夾中 IRremote_Test 的範例碼燒入 Arduino 開發板後，使用套件內的紅外線遙控器實際去按下您所
// 要宣告的各按鈕，並透過序列埠監控視窗 Serial Monitor 所顯示對應的數值，實際抓出對應的 IR Code　再宣告下列指令參數

#define IR_Forwards    0xFF629D  // 遙控器方向鍵 上, 前進
#define IR_Back        0xFFA857  // 遙控器方向鍵 下, 後退
#define IR_Stop        0xFF02FD  // 遙控器 OK 鍵, 停止
#define IR_Fast        0xFF6897  // 遙控器方向鍵 1, 快
#define IR_Slow        0xFF9867  // 遙控器方向鍵 2, 慢
#define IR_Tracking    0xFF30CF  // 遙控器 4 鍵, 循跡模式
#define IR_Ultrasonic  0xFF18E7  // 遙控器 5 鍵, 超音波停車模式

// 循線模組
#define SensorLeft    A0  //宣告 左側感測器 輸入腳
#define SensorMiddle  A1  //宣告 中間感測器 輸入腳
#define SensorRight   A2  //宣告 右側感測器 輸入腳
int off_track = 0;        //宣告變數

//sonar HC-SR04 超音波測距模組 HC-SR04 
//sonar pins and datas
int US_Trig = A3;  //宣告超音波模組 Trig 腳位
int US_Echo = A4;  //宣告超音波模組 Echo 腳位
unsigned long sonar_previous_time = 0;
unsigned long sonar_current_time, sonar_pass_time;
float sonar_duty, sonar_distance = 0;

//Laser Module
int lasertrig = A5;
unsigned long laser_on_time = 0;
unsigned long laser_last_time;
bool laserOn= false;

//IR Obstacle pins
const int isObstaclePin = 2;            //紅外線模組 isObstaclePin(OUT) 腳位 連接至腳位2 will use interrupt
volatile bool isObstacle = false;      // HIGH MEANS NO OBSTACLE
const int ledPin = 13;     

void pinFalled()  //ISR for IR Obstacle Module
{
 isObstacle = true;
}  // end of ISR

//Buzzer
const int buzzerPin = 4;

float sonar(unsigned long delayTime){ 
  sonar_current_time = micros();
  sonar_pass_time = sonar_current_time - sonar_previous_time;
  digitalWrite(US_Trig, HIGH);
  if(sonar_pass_time >=delayTime*1000) {
    digitalWrite(US_Trig, LOW);
    sonar_previous_time = micros();
    sonar_duty = pulseIn(US_Echo,HIGH);
    sonar_distance = (sonar_duty/2)/29.4;
    }
  return sonar_distance;
}
void alarm(int pin, int a){
  tone(pin, 4000);
  digitalWrite(lasertrig, HIGH);
  delay(a*50);
  noTone(pin);
  digitalWrite(lasertrig, LOW);
  delay(a*50);
}
void ringTone(int pin) {
  for (int i=0; i<2; i++) { //repeat 10 times
    tone(pin, 2000);
    delay(50);
    tone(pin, 1500);
    delay(50);
    }
  noTone(pin);
  } 
void ringTone2(int pin) {
    tone(pin, 4000);    delay(10);
    tone(pin, 3500);    delay(9);
    tone(pin, 3000);    delay(8);
    tone(pin, 3500);    delay(7);
    tone(pin, 3000);    delay(6);
    tone(pin, 2500);    delay(5);
    tone(pin, 2000);    delay(4);
    tone(pin, 1500);    delay(30);
    noTone(pin);
} 

void laserfired(unsigned long delayTime) {
  if (!laserOn){
      digitalWrite(lasertrig, HIGH);
      laser_on_time= micros();
      laserOn=true; 
   }    
  laser_last_time = micros()-laser_on_time;
  if(laser_last_time >= delayTime*1000)
   {
    digitalWrite(lasertrig, LOW);
    laserOn=false;
   } 
}
void back(int a)    // 小車前進
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）順時針轉動
    digitalWrite(MotorA_I2,LOW);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）逆時針轉動
    digitalWrite(MotorB_I4,LOW);
    delay(a * 100);
}

void turnL(int d)    // 小車左轉
{
    digitalWrite(MotorA_I1,LOW);    //馬達（右）逆時針轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）逆時針轉動
    digitalWrite(MotorB_I4,LOW);
    delay(d * 100);
}

void turnR(int e)    // 小車右轉
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）順時針轉動
    digitalWrite(MotorA_I2,LOW);
    digitalWrite(MotorB_I3,LOW);    //馬達（左）順時針轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(e * 100);
}    

void stopRL(int f)  // 小車停止
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）停止轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）停止轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(f * 100);
}

void advance(int g)    // 小車後退
{
    digitalWrite(MotorA_I1,LOW);    //馬達（右）逆時針轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,LOW);    //馬達（左）順時針轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(g * 100);     
}
    

void setup()
{
  Serial.begin(9600); 
  
  pinMode(MotorA_I1,OUTPUT);
  pinMode(MotorA_I2,OUTPUT);
  pinMode(MotorB_I3,OUTPUT);
  pinMode(MotorB_I4,OUTPUT);
  pinMode(MotorA_PWMA,OUTPUT);
  pinMode(MotorB_PWMB,OUTPUT);
  
  pinMode(isObstaclePin, INPUT); //Receive IR Obstacle interrupt Signal
  pinMode(lasertrig, OUTPUT);
  digitalWrite (lasertrig, LOW);

  pinMode(US_Trig, OUTPUT);
  pinMode(US_Echo, INPUT);
 
  pinMode(SensorLeft,   INPUT); 
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight,  INPUT);
  
  
  irrecv.enableIRIn();  // 啟動 IR Controller Receiver 解碼讀取
  
  attachInterrupt (digitalPinToInterrupt (isObstaclePin), pinFalled, FALLING);  // attach interrupt handler
}

void blueToothCommand()
{
  int cmd = Serial.read();  // 讀取藍芽指令
    
  switch(cmd)  // 執行藍芽指令
  {
    case 'M':
      speedF = 200;
      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速 
      break;
    case 'm':
      speedF =70;
      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
      break; 
    case 'B':  // 倒車
      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
      back(5);
      break;
    case 'L':  // 左轉
      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
      turnL(5);
      break;       
    case 'R':  // 右轉
      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
      turnR(5);
      break;     
    case 'F':  // 前進
      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
      advance(5);
      break;        
    case 'S':  // 停止
        stopRL(5);
        break;
    case 'W':  // Laser
        digitalWrite(lasertrig, HIGH);
        break;
        
    case 'w':  // 關Laser
        digitalWrite(lasertrig, LOW);
        break;
        
    case 'X':  // 終止所有程序
        stopRL(5);
        digitalWrite(lasertrig, LOW);
        break;        
  }
}

void loop()
{
    blueToothCommand();  // Read if there is a command from BlueTooth module
   
    if(irrecv.decode(&results)) 
    {
        // 解碼成功，收到一組紅外線訊號
        switch(results.value)
        {
          case IR_Stop:
            //Serial.print(" Stop");
            digitalWrite(lasertrig, LOW);
            stopRL(1);
            break;
          case IR_Fast:
            speedF = 200;
            analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
            analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
            break;
          case IR_Slow:
            speedF = 100;
            analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
            analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
            break;
          case IR_Tracking: //Enter Line Tracking mode
            //Serial.print(" Tracking Mode");
            
            irrecv.resume();
            
            while(true)
            {
              // 讀取感測器狀態
              int SL = digitalRead(SensorLeft);
              int SM = digitalRead(SensorMiddle);
              int SR = digitalRead(SensorRight);
              
              if((SM == LOW) && (SL == LOW) && (SR == LOW))  // 小車脫離黑線
              {
                analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
                analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
                
                // 小車有時會因為循線感測器誤判或黑線轉角太大, 認為脫離黑線了而停車
                // 加上尋回黑線機制, 避免小車誤動作
                // 您可以修改程式讓 循線/尋線 機制更完美
				
                if(off_track < 3)
                {
                  switch(off_track)
                  {
                    case 0:
                      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                      back(1);
                      break;
                    
                    case 1:
                      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                      turnR(1);
                      break;
                      
                    case 2:
                      analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                      analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                      turnL(2);
                      break;
                  }
                  
                  off_track++;
                }
                else
                {
                  stopRL(0);
                }
              }
              else
              {
                off_track = 0;
                               
                if(SM == HIGH)  //中感測器在黑色區域
                {
                  if((SL == LOW) && (SR == HIGH))  // 左白右黑, 車體偏右校正
                  {
                    analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB, speedF*9/10);    //設定馬達 (左) 轉速
                    advance(0);
                  } 
                  else if((SL == HIGH) && (SR == LOW))  // 左黑右白, 車體偏左校正
                  {
                    analogWrite(MotorA_PWMA, speedF*9/10);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                    advance(0);
                  }
                  else  // 其他, 直走
                  {
                    analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                    advance(0);
                  }
                } 
                else // 中感測器在白色區域, 車體已大幅偏離黑線
                {
                  if((SL == LOW) && (SR == HIGH))  // 左白右黑, 車體快速右轉
                  {
                    analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                    turnR(0);
                  }
                  else if((SL == HIGH) && (SR == LOW))  // 左黑右白, 車體快速左轉
                  {
                    analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                    analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                    turnL(0);
                  }
                }
              }
              
              if(irrecv.decode(&results)) //See if IR remote controller sent any command?
              {
                irrecv.resume();
                
                if(results.value == IR_Stop)
                {
                  stopRL(1);
                  analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
                  analogWrite(MotorB_PWMB,speedF);    //設定馬達 (左) 轉速
                  break;
                }else if(results.value == IR_Fast){
                  tone(buzzerPin, 2000);
                  delay(100);
                  noTone(buzzerPin);
                  speedF = 200;
                }else if(results.value == IR_Slow){
                  tone(buzzerPin, 4000);
                  delay(100);
                  noTone(buzzerPin);
                  speedF = 100;
                }
              }
            }
            break;
            
          case IR_Ultrasonic:
            irrecv.resume();
            speedF=200;
            while (true){
            if(sonar(2) >=60){
              analogWrite(MotorA_PWMA,speedF);    //設定馬達 (右) 轉速
              analogWrite(MotorB_PWMB,speedF);
              advance(0);
              digitalWrite(lasertrig, LOW);
              //alarm(buzzerPin, 10);
            }
            else if(sonar(2) < 60.0 && sonar(2)>5.0){
              advance(0);
              speedF2 = (sonar(2)-5)/55.0*140.0+60.0;
              analogWrite(MotorA_PWMA,speedF2);    //設定馬達 (右) 轉速
              analogWrite(MotorB_PWMB,speedF2);    //設定馬達 (左) 轉速
              digitalWrite(lasertrig, HIGH);
              //ringTone(buzzerPin);
              if(sonar(2)>35)
                alarm(buzzerPin, 10);
              else
                alarm(buzzerPin, 5);
            }else if(sonar(2)<5.0){
              stopRL(0);
              ringTone2(buzzerPin);
              //alarm(buzzerPin, 1);
              digitalWrite(lasertrig, HIGH);
              
            }
              if(irrecv.decode(&results))
              {
                irrecv.resume();
                
                if(results.value == IR_Stop)
                {
                  //Serial.print("\r\nStop Ultrasonic detector");
                  digitalWrite(lasertrig, LOW);
                  stopRL(1);
                  break;
                }else if(results.value == IR_Back){
                  back(1);
                }
              }
            }
            break;
            
          default:
          break;
        }
    
        irrecv.resume(); // Receive the next value
    }
 }
