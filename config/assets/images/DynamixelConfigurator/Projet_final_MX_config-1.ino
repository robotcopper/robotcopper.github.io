/*
Copyright 2022 Jousselin.F

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

//======Library======//

#include <Adafruit_TFTLCD.h> 
#include <Adafruit_GFX.h>    
#include <TouchScreen.h>

#include <Dynamixel_Serial.h>   // Library needed to control Dynamixel servo


//======GPIO======//

#define LCD_CS A3 
#define LCD_CD A2 
#define LCD_WR A1 
#define LCD_RD A0 
#define LCD_RESET A4 

#define YP A2  // must be an analog pin, use "An" notation!
#define XM A3  // must be an analog pin, use "An" notation!
#define YM 8   // can be a digital pin
#define XP 9   // can be a digital pin


//======Screen Calibration======//

#define TS_MINX 200
#define TS_MINY 130
#define TS_MAXX 950
#define TS_MAXY 900


//======Colors======//

#define BLACK   0x0000
#define DBLUE   0x083F
#define BLUE    0x001F
#define LBLUE   0xAE5F
#define RED     0xF800
#define DRED    0xF902
#define GREEN   0x07E0
#define DGREEN  0x0EE5
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define LPURPLE 0xE37F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


//======Servo parameters======//

#define SERVO_ControlPin 10       // Control pin of buffer chip
#define LED13 0x0D                // Pin of Visual indication for runing "heart beat" using onboard LED


//======tft object declaration======//

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


//======Variable======//

//LCD//
char currentPage;
char ip[3];
int ipCurrentLen=0;
int Id=1;
long BaudRate;
int AlowClickOnSetup=1;
int AlowClickOnTest=0;

//Servo//
long Baud[9] = {9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000};
bool MODE=1;    //ServoMode by default
int Torque = 100;   //100% torque by default
unsigned int CWLimit=0x000*0.088;    //converts the hexadecimal input to degrees
unsigned int CCWLimit=0xFFF*0.088;


//######Setup######//

void setup() {
  
  //Initialisation of serial//
  Serial.begin(9600);
  Serial.print("Starting...");

  //Initial setup//
  tft.reset();
  tft.begin(0x9341);
  tft.setRotation(3);

  DrawHomeScreen();
  currentPage = '0';    // Indicates that we are at Home Screen

  //ResetServo();
  
}



//######Loop######//

void loop(){
  if(currentPage=='0'){
    TSPoint p = ts.getPoint();    //Get touch point
  
    if (p.z > ts.pressureThreshhold) {
      p.x = map(p.x, TS_MAXX, TS_MINX, 0, 320);
      p.y = map(p.y, TS_MAXY, TS_MINY, 0, 240);
       
      if(p.x>30 && p.x<285 && p.y>130 && p.y<200){    // The user has pressed inside the red rectangle
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        currentPage='1';
        DrawBaudRateSetup();     
      }
      
    }
    
  }

  if(currentPage=='1'){
    TSPoint q = ts.getPoint();  //Get touch point
    
    if (q.z > ts.pressureThreshhold){
      q.x = map(q.x, TS_MAXX, TS_MINX, 0, 320);
      q.y = map(q.y, TS_MAXY, TS_MINY, 0, 240);
      
      //Click on 9600//
      if(q.x>3 && q.x<106 && q.y>60 && q.y<100){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=9600;
        currentPage='2';
        DrawIpSetup();
      }

      //Click on 19200//
      if(q.x>108 && q.x<211 && q.y>60 && q.y<100){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=19200;
        currentPage='2';
        DrawIpSetup();
      }
      
      //Click on 57600//
      if(q.x>213 && q.x<316 && q.y>60 && q.y<100){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=57600;
        currentPage='2';
        DrawIpSetup();
      }

      //Click on 115.2k//
      if(q.x>3 && q.x<106 && q.y>110 && q.y<150){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=115200;
        currentPage='2';
        DrawIpSetup();
      }

      //Click on 200k//
      if(q.x>108 && q.x<211 && q.y>110 && q.y<150){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=200000;
        currentPage='2';
        DrawIpSetup();
      }
      
      //Click on 250k//
      if(q.x>213 && q.x<316 && q.y>110 && q.y<150){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=250000;
        currentPage='2';
        DrawIpSetup();
      }

      //Click on 400k//
      if(q.x>3 && q.x<106 && q.y>160 && q.y<200){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=400000;
        currentPage='2';
        DrawIpSetup();
      }

      //Click on 500k//
      if(q.x>108 && q.x<211 && q.y>160 && q.y<200){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=500000;
        currentPage='2';
        DrawIpSetup();
      }

      //Click on 1M//
      if(q.x>213 && q.x<316 && q.y>160 && q.y<200){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        BaudRate=1000000;
        currentPage='2';
        DrawIpSetup();
      }
      //Click on Return//
      if(q.x>115 && q.x<205 && q.y>210 && q.y<235){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        currentPage='0';
        DrawHomeScreen();
      }

    }

  }

  if(currentPage=='2'){
    TSPoint r = ts.getPoint();  //Get touch point
    
    if (r.z > ts.pressureThreshhold){
      r.x = map(r.x, TS_MAXX, TS_MINX, 0, 320);
      r.y = map(r.y, TS_MAXY, TS_MINY, 0, 240);

      //Click on 1//
      if(r.x>120 && r.x<145 && r.y>75 && r.y<104){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('1');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 2//
      if(r.x>148 && r.x<173 && r.y>75 && r.y<104){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('2');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 3//
      if(r.x>176 && r.x<201 && r.y>75 && r.y<104){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('3');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 4//
      if(r.x>120 && r.x<145 && r.y>107 && r.y<139){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('4');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }

      //Click on 5//
      if(r.x>148 && r.x<173 && r.y>107 && r.y<139){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('5');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 6//
      if(r.x>176 && r.x<201 && r.y>107 && r.y<139){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('6');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 7//
      if(r.x>120 && r.x<145 && r.y>142 && r.y<174){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('7');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 8//
      if(r.x>148 && r.x<173 && r.y>142 && r.y<174){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('8');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 9//
      if(r.x>176 && r.x<201 && r.y>142 && r.y<174){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('9');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on 0//
      if(r.x>120 && r.x<145 && r.y>177 && r.y<209){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpIncrement('0');
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }
      
      //Click on del//
      if(r.x>148 && r.x<201 && r.y>177 && r.y<209){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        strIpDecrement();
        printIp();
        delay(250);   //delay too avoid spam from a nomber to another
      }

      //Click on Confirm//
      if(r.x>214 && r.x<314 && r.y>177 && r.y<209){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        currentPage='3';
        DrawModeAndTorqueSetup();
        delay(250);   //delay too avoid spam from a nomber to another
      }

      
      //Click on Return//
      if(r.x>115 && r.x<205 && r.y>210 && r.y<235){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        currentPage='1';
        DrawBaudRateSetup();
      }
    
    }
    /*//Debuging//
    Serial.print(ipCurrentLen);
    Serial.print('\n');
    Serial.print(ip[0]);
    Serial.print('\t');
    Serial.print(ip[1]);
    Serial.print('\t');
    Serial.print(ip[2]);
    Serial.print('\n');
    */
  }

    if(currentPage=='3'){
    TSPoint p = ts.getPoint();  //Get touch point
    
    if (p.z > ts.pressureThreshhold){
      p.x = map(p.x, TS_MAXX, TS_MINX, 0, 320);
      p.y = map(p.y, TS_MAXY, TS_MINY, 0, 240);

      //Click on ServoMode//
      if(p.x>30 && p.x<150 && p.y>55 && p.y<85){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        MODE=1;
        tft.drawRect(30,55, 120, 30, RED);
        tft.drawRect(170,55, 120, 30, WHITE);

        DrawJointMode();
        }
      }

      //Click on - CW//
      if(p.x>108 && p.x<128 && p.y>100 && p.y<120 && MODE){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        if(CWLimit>0){
          CWLimit=CWLimit-1;
          tft.fillRect(53,100,50,20, BLACK);
          tft.setCursor(55,107);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.print(CWLimit);
          tft.print(" DEG");
        }
      }

      //Click on + CW//
      if(p.x>130 && p.x<150 && p.y>100 && p.y<120 && MODE){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        if(CWLimit<360){
          CWLimit=CWLimit+1;
          tft.fillRect(53,100,50,20, BLACK);
          tft.setCursor(55,107);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.print(CWLimit);
          tft.print(" DEG");
        }
      }
      
      //Click on WheelMode//
      if(p.x>170 && p.x<290 && p.y>55 && p.y<85){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        MODE=0;
        tft.drawRect(170,55, 120, 30, RED);
        tft.drawRect(30,55, 120, 30, WHITE);

        tft.fillRect(1,87, 317, 35, BLACK);
      }

      //Click on - CCW//
      if(p.x>240 && p.x<260 && p.y>100 && p.y<120 && MODE){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        if(CCWLimit>0){
          CCWLimit=CCWLimit-1;
          tft.fillRect(193,100,50,20, BLACK);
          tft.setCursor(195,107);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.print(CCWLimit);
          tft.print(" DEG");
        }
      }

      //Click on + CCW//
      if(p.x>270 && p.x<290 && p.y>100 && p.y<120 && MODE){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        if(CCWLimit<360){
          CCWLimit=CCWLimit+1;
          tft.fillRect(193,100,50,20, BLACK);
          tft.setCursor(195,107);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.print(CCWLimit);
          tft.print(" DEG");
        }
      }

      //Click on - Torque//
      if(p.x>140 && p.x<160 && p.y>185 && p.y<205){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        if(Torque>0){
          Torque=Torque-1;
          tft.fillRect(140,165,50,20, BLACK);
          tft.setCursor(140,165);
          tft.setTextColor(WHITE);
          tft.setTextSize(2);
          tft.print(Torque);
          tft.print("%");
        }
      }

      //Click on + Torque//
      if(p.x>162 && p.x<182 && p.y>185 && p.y<205){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        if(Torque<100){
          Torque=Torque+1;
          tft.fillRect(140,165,50,20, BLACK);
          tft.setCursor(140,165);
          tft.setTextColor(WHITE);
          tft.setTextSize(2);
          tft.print(Torque);
          tft.print("%");
        }
      }

      //Click on Confirm//
      if(p.x>214 && p.x<314 && p.y>177 && p.y<209){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        currentPage='4';
        DrawValidateAndConfigSetup();
        delay(250);   //delay too avoid spam from a nomber to another
      }

      //Click on Return//
      if(p.x>115 && p.x<205 && p.y>210 && p.y<235){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        currentPage='2';
        DrawIpSetup();
      }
  }
  
  if(currentPage=='4'){
    TSPoint s = ts.getPoint();  //Get touch point
    
    if (s.z > ts.pressureThreshhold){
      s.x = map(s.x, TS_MAXX, TS_MINX, 0, 320);
      s.y = map(s.y, TS_MAXY, TS_MINY, 0, 240);

      //Click on Set up//
      if(s.x>70 && s.x<250 && s.y>160 && s.y<200 && AlowClickOnSetup){
        AlowClickOnSetup=0;
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        tft.fillRect(70,160,180,40,BLACK);
        ResetServo();
        ProgramBaudrateID(BaudRate,Id,MODE,Torque,CWLimit,CCWLimit);
        AlowClickOnTest=1;
        Test();
       }
       
     }

     //Click on Test//
     if(s.x>247 && s.x<317 && s.y>210 && s.y<235 && AlowClickOnTest){
      //This is important, because the libraries are sharing pins
      pinMode(XM, OUTPUT);
      pinMode(YP, OUTPUT);
          
      //Retrace Test Button
      tft.fillRect(247,210, 70, 25, LPURPLE);
      tft.drawRect(247,210, 70, 25, WHITE);
      tft.setCursor(257,215);
      tft.setTextColor(WHITE);
      tft.setTextSize(2);
      tft.print("Test");
           
      if(MODE==1){
        CheckServoMode(Id,CWLimit,CCWLimit);
      }
      else{
        CheckWheelMode(Id);
      }
      
     }

      //Click on Return//
      if(s.x>115 && s.x<205 && s.y>210 && s.y<235){
        //This is important, because the libraries are sharing pins
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        currentPage='3';
        AlowClickOnSetup =1;
        DrawModeAndTorqueSetup();      
      }
    
    }
  
}



//######Custom Fonctions######//

void DrawHomeScreen(){

  //Backgroud in black//
  tft.fillScreen(BLACK);

  //Draw white frame
  tft.drawRect(0,0,319,240,WHITE);
  
  //Print "Dynamixel" Text
  tft.setCursor(110,30);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Dynamixel");

  //Print "Configuration of MX servo" Text
  tft.setCursor(10,50);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Configuration of MX servo");
  
  //Create Red Button
  tft.fillRect(30,130, 260, 70, DRED);
  tft.drawRect(30,130, 260, 70,WHITE);
  tft.setCursor(118,138);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.print("Start");
  tft.setCursor(40,168);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.print("configuration!");
}


void DrawBaudRateSetup(){
    
    //Erase the screen
    tft.fillScreen(BLACK);
    
    //Draw frame
    tft.drawRect(0,0,319,240,WHITE);

    //Title of baud rate setting
    tft.setCursor(10,10);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Baud rate speed which the");
    tft.setCursor(30,30);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("motor will be set too :");

    //Create 9600 Button
    tft.fillRect(3,60, 103, 40, DBLUE);
    tft.drawRect(3,60, 103, 40, WHITE);
    tft.setCursor(19,70);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("9600");
    
    //Create 19200 Button
    tft.fillRect(108,60, 103, 40, DBLUE);
    tft.drawRect(108,60, 103, 40, WHITE);
    tft.setCursor(114,70);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("19200");
    
    //Create 57600 Button
    tft.fillRect(213,60, 103, 40, DBLUE);
    tft.drawRect(213,60, 103, 40, WHITE);
    tft.setCursor(221,70);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("57600");

    //Create 115200 Button
    tft.fillRect(3,110, 103, 40, DBLUE);
    tft.drawRect(3,110, 103, 40, WHITE);
    tft.setCursor(2,120);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("115.2k");

    //Create 200k Button
    tft.fillRect(108,110, 103, 40, DBLUE);
    tft.drawRect(108,110, 103, 40, WHITE);
    tft.setCursor(127,120);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("200k");
    
    //Create 250k Button
    tft.fillRect(213,110, 103, 40, DBLUE);
    tft.drawRect(213,110, 103, 40, WHITE);
    tft.setCursor(235,120);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("250k");


    //Create 400k Button
    tft.fillRect(3,160, 103, 40, DBLUE);
    tft.drawRect(3,160, 103, 40, WHITE);
    tft.setCursor(20,170);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("400k");

    //Create 500k Button
    tft.fillRect(108,160, 103, 40, DBLUE);
    tft.drawRect(108,160, 103, 40, WHITE);
    tft.setCursor(127,170);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("500k");
    
    //Create 1M Button
    tft.fillRect(213,160, 103, 40, DBLUE);
    tft.drawRect(213,160, 103, 40, WHITE);
    tft.setCursor(247,170);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("1M");

    ReturnButton();

    delay(500);   //delay too avoid spam from a page to another
}


void DrawIpSetup(){
  
  //Erase the screen
  tft.fillScreen(BLACK);
    
  //Draw frame
  tft.drawRect(0,0,319,240,WHITE);

  //Title of Ip setting
  tft.setCursor(5,10);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("ID at which the motor will");
  tft.setCursor(120,30);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("be set :");

  printIp();

  //Reminder text
  tft.setCursor(15,80);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("Reminder :");
  tft.setCursor(15,100);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("The ID have");
  tft.setCursor(15,115);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("to be between");
  tft.setCursor(15,130);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("0 and 253.");
  tft.setCursor(15,145);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("Otherwise it");
  tft.setCursor(15,160);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("will be set");
  tft.setCursor(15,175);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("to 1.");

  //Entry 
  tft.drawRect(140,49, 40, 20, WHITE);

  //Create 1 Button
  tft.fillRect(120,72, 25, 32, DBLUE);
  tft.drawRect(120,72, 25, 32, WHITE);
  tft.setCursor(127,80);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("1");
    
  //Create 2 Button
  tft.fillRect(148,72, 25, 32, DBLUE);
  tft.drawRect(148,72, 25, 32, WHITE);
  tft.setCursor(155,80);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("2");
    
  //Create 3 Button
  tft.fillRect(176,72, 25, 32, DBLUE);
  tft.drawRect(176,72, 25, 32, WHITE);
  tft.setCursor(185,80);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("3");

  //Create 4 Button
  tft.fillRect(120,107, 25, 32, DBLUE);
  tft.drawRect(120,107, 25, 32, WHITE);
  tft.setCursor(127,117);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("4");
  
  //Create 5 Button
  tft.fillRect(148,107, 25, 32, DBLUE);
  tft.drawRect(148,107, 25, 32, WHITE);
  tft.setCursor(155,117);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("5");
    
  //Create 6 Button
  tft.fillRect(176,107, 25, 32, DBLUE);
  tft.drawRect(176,107, 25, 32, WHITE);
  tft.setCursor(185,117);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("6");


  //Create 7 Button
  tft.fillRect(120,142, 25, 32, DBLUE);
  tft.drawRect(120,142, 25, 32, WHITE);
  tft.setCursor(127,150);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("7");

  //Create 8 Button
  tft.fillRect(148,142, 25, 32, DBLUE);
  tft.drawRect(148,142, 25, 32, WHITE);
  tft.setCursor(155,150);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("8");
  
  //Create 9 Button
  tft.fillRect(176,142, 25, 32, DBLUE);
  tft.drawRect(176,142, 25, 32, WHITE);
  tft.setCursor(185,150);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("9");

  //Create 0 Button
  tft.fillRect(120,177, 25, 32, DBLUE);
  tft.drawRect(120,177, 25, 32, WHITE);
  tft.setCursor(127,187);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("0");

  //Create del Button
  tft.fillRect(148,177, 53, 32, DBLUE);
  tft.drawRect(148,177, 53, 32, WHITE);
  tft.setCursor(158,187);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("del");

  //Create confirm Button
  tft.fillRect(214,177, 100, 32, DRED);
  tft.drawRect(214,177, 100, 32, WHITE);
  tft.setCursor(224,187);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Confirm");

  ReturnButton();

  delay(500);   //delay too avoid spam from a page to another

}


void ReturnButton(){
    //Create return Button
    tft.fillRect(115,210, 90, 25, DGREEN);
    tft.drawRect(115,210, 90, 25, WHITE);
    tft.setCursor(125,215);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Return");  
}


void strIpIncrement(char val){
  if(ipCurrentLen<3){
  ip[ipCurrentLen]=val;
  ipCurrentLen=1+ipCurrentLen;
  }
}


void strIpDecrement(){
  if(ipCurrentLen>0){
  ip[ipCurrentLen-1]='\0';
  tft.fillRect(133+(10*ipCurrentLen),52, 15, 15, BLACK);
  ipCurrentLen=ipCurrentLen-1;
  }
}


void printIp(){
  tft.setCursor(145,52);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(ip[0]);
  tft.setCursor(155,52);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(ip[1]);
  tft.setCursor(165,52);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(ip[2]);
}


void DrawModeAndTorqueSetup(){

  //Erase the screen
  tft.fillScreen(BLACK);
    
  //Draw frame
  tft.drawRect(0,0,319,240,WHITE);

  //Title of Mode setting
  tft.setCursor(5,10);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("In which mode do you want");
  tft.setCursor(25,30);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("your motor to be set ?");

  //Create ServoMode Button
  tft.fillRect(30,55, 120, 30, DBLUE);
  tft.drawRect(30,55, 120, 30, WHITE);
  tft.setCursor(37,62);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("ServoMode");

  //Create WheelMode Button
  tft.fillRect(170,55, 120, 30, DBLUE);
  tft.drawRect(170,55, 120, 30, WHITE);
  tft.setCursor(177,62);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("WheelMode");

  if(MODE==1){
    tft.drawRect(30,55, 120, 30, RED);
    DrawJointMode();
  }
  else{
    tft.drawRect(170,55, 120, 30, RED);    
  }
  
  //Title of Torque setting
  tft.setCursor(10,125);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Which torque do you want?");
  tft.setCursor(87,145);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("Recommended: maximum torque");

  //Torque
  tft.setCursor(140,165);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(Torque);
  tft.print("%");

  //Create - Button
  tft.fillRect(140,185, 20, 20, DBLUE);
  tft.drawRect(140,185, 20, 20, WHITE);
  tft.setCursor(145,188);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("-");

  //Create + Button
  tft.fillRect(162,185, 20, 20, DBLUE);
  tft.drawRect(162,185, 20, 20, WHITE);
  tft.setCursor(167,188);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("+");

  //Create confirm Button
  tft.fillRect(214,177, 100, 32, DRED);
  tft.drawRect(214,177, 100, 32, WHITE);
  tft.setCursor(224,187);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Confirm");

  ReturnButton();

  delay(500);   //delay too avoid spam from a page to another

}

void DrawJointMode(){
  
  //Title of JointMode
  tft.setCursor(80,90);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("With which JointMode limits?");

  //Title of Min
  tft.setCursor(28,107);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("CW:");

  //Angle min
  tft.setCursor(55,107);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print(CWLimit);
  tft.print(" DEG");

  //Create - Button min
  tft.fillRect(108,100, 20, 20, DBLUE);
  tft.drawRect(108,100, 20, 20, WHITE);
  tft.setCursor(113,103);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("-");

  //Create + Button min
  tft.fillRect(130,100, 20, 20, DBLUE);
  tft.drawRect(130,100, 20, 20, WHITE);
  tft.setCursor(135,103);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("+");

  //Title of CCW
  tft.setCursor(168,107);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("CCW:");

  //Angle max
  tft.setCursor(195,107);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print(CCWLimit);
  tft.print(" DEG");

  //Create - Button max
  tft.fillRect(248,100, 20, 20, DBLUE);
  tft.drawRect(248,100, 20, 20, WHITE);
  tft.setCursor(253,103);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("-");

  //Create + Button max
  tft.fillRect(270,100, 20, 20, DBLUE);//262,100, 20, 20,
  tft.drawRect(270,100, 20, 20, WHITE);
  tft.setCursor(275,103);//267
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("+");
  
}

void DrawValidateAndConfigSetup(){
  
  //Erase the screen
  tft.fillScreen(BLACK);
    
  //Draw frame
  tft.drawRect(0,0,319,240,WHITE);

  //Title of settings
  tft.setCursor(25,10);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Do you want to set your");
  tft.setCursor(90,30);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("servomotor in :");

  //Settings chosen
  tft.setCursor(5,60);
  tft.setTextColor(LPURPLE);
  tft.setTextSize(2);
  if(MODE==1){
    tft.print("ServoMode ");
  }
  else{
    tft.print("WheelMode ");
  }
  tft.setTextColor(WHITE);
  tft.print("with ");
  tft.setTextColor(LPURPLE);
  tft.print(Torque);
  tft.setTextColor(WHITE);
  tft.print("% torque");
  tft.setCursor(85,90);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("at ");
  tft.setTextColor(LPURPLE);
  tft.print(BaudRate,DEC);
  tft.print(" bps");
  tft.setCursor(72,120);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("with ");
  tft.setTextColor(LPURPLE);
  Id=strtol(ip,NULL,10);
  if(Id>253){
    Id=1;
  }
  tft.print(Id);
  tft.setTextColor(WHITE);
  tft.print(" for ID ?");

  //Create Set up Button
  tft.fillRect(70,160, 180, 40, DRED);
  tft.drawRect(70,160, 180, 40, WHITE);
  tft.setCursor(105,170);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.print("Set up");
  
  ReturnButton();

}

void Test(){
  
  //Create Test Button
  tft.fillRect(247,210, 70, 25, MAGENTA);
  tft.drawRect(247,210, 70, 25, WHITE);
  tft.setCursor(257,215);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Test");

  //Warning about the test
  tft.setCursor(258,158);
  tft.setTextColor(RED);
  tft.setTextSize(1);
  tft.print("!Warnning!");
  tft.setTextColor(WHITE);
  tft.setCursor(268,170);
  tft.print("Be sure");
  tft.setCursor(258,180);
  tft.print("to release");
  tft.setCursor(262,190);
  tft.print("the motor");
  tft.setCursor(252,200);
  tft.print("for testing");
  
}


void ResetServo(){
  
  // Pin setup for Visual indication of runing (heart beat) program using onboard LED
  pinMode(LED13, OUTPUT);
  digitalWrite(LED13, HIGH);
  
  int j=0;    //Loading bar progress indicator
  
  for (int b=0; b<9; b++){    // This "for" loop will take about 20 Sec to compelet and is used to loop though all speeds that Dynamixel can be and send reset instuction 
    long Baudrate_BPS = 0;
    Baudrate_BPS  = Baud[b];
    /*//Debuging//
    Serial.print(Baudrate_BPS, DEC);
    Serial.print("\n");*/
    
    if(Baudrate_BPS==57600){
      Dynamixel.begin(Baudrate_BPS ,SERVO_ControlPin);    // Set Ardiuno Serial speed and control pin
      Dynamixel.reset(0xFE);    // Broadcast to all Dynamixel IDs(0xFE is the ID for all Dynamixel to responed) and Reset Dynamixel to factory default
    }
    
    else{
      Dynamixel.begin(Baudrate_BPS ,SERVO_ControlPin);    // Set Ardiuno Serial speed and control pin
      
      for (int i=1; i<0xFF; i++){
        /*//Debuging//
        Serial.print(i);
        Serial.print("\n");*/
        Dynamixel.reset(i);
        j++;
        
        //Loading bar//
        tft.drawRect(57,170,200,20,WHITE);
        tft.fillRect(58,171,int(0.0979*j),18,GREEN);
        
      }
      
      delay(5);
    }
    delay(100);    // Time needed for Dynamixel to Broadcast
  }
  
  //Loading bar//
  tft.setCursor(117,173);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Reset...");
  
  digitalWrite(LED13, LOW);
  delay(3000);    // Give time for Dynamixel to reset
}


void ProgramBaudrateID(long SERVO_SET_Baudrate, int SERVO_ID,bool MODE,int Torque,unsigned int CWLimit,unsigned int CCWLimit){// Baud rate and ID of which we will set Dynamixel too 
  
  // Now that the Dynamixel is reset to factory setting we will program its Baudrate and ID
  
  //Loading bar//
  tft.fillRect(58,171,198,18,GREEN);
  tft.setCursor(87,173);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Programming...");
  
  Dynamixel.begin(57600,SERVO_ControlPin);                // Set Ardiuno Serial speed to factory default speed of 57600
  Dynamixel.setID(0xFE,SERVO_ID);                         // Broadcast to all Dynamixel IDs(0xFE) and set with new ID
  delay(10);                                              // Time needed for Dynamixel to set it's new ID before next instruction can be sent
  Dynamixel.setStatusPaket(SERVO_ID,READ);                // Tell Dynamixel to only return status packets when a "read" instruction is sent e.g. Dynamixel.readVoltage();
  delay(30);
  Dynamixel.setBaudRate(SERVO_ID,SERVO_SET_Baudrate);     // Set Dynamixel to new serial speed 
  delay(30);                                              // Time needed for Dynamixel to set it's new Baudrate

  Dynamixel.begin(SERVO_SET_Baudrate,SERVO_ControlPin);   // We now need to set Ardiuno to the new Baudrate speed 
  Dynamixel.ledState(SERVO_ID, ON);                       // Turn Dynamixel LED on
  delay(5);
  Dynamixel.setMode(SERVO_ID, MODE,int(CWLimit*11.375),int(CCWLimit*11.375));         // Turn mode to SERVO, must be WHEEL if using wheel mode
  delay(30);
  Dynamixel.setMaxTorque(SERVO_ID,int(Torque*7,67));                // Set Dynamixel to max torque limit

  //Loading bar//
  tft.drawRect(57,170,200,20,WHITE);
  tft.fillRect(58,171,198,18,LBLUE);
  tft.setCursor(137,173);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("DONE");
  tft.setCursor(67,192);
  tft.setTextColor(CYAN);
  tft.setTextSize(1);
  tft.print("Dynamixel LED should be on now");

}


void CheckServoMode(int SERVO_ID,unsigned int CWLimit,unsigned int CCWLimit){
  
  digitalWrite(LED13, HIGH);                // Turn Arduino onboard LED on
  Dynamixel.ledState(SERVO_ID, ON);         // Turn Dynamixel LED on
  delayMicroseconds(1);
  Dynamixel.servo(SERVO_ID,int(CWLimit*11.375),0x7FF);    // Comman for servo mode, Move servo to angle 1(0.088 degree) at speed 100
  delay(3000);
  
  digitalWrite(LED13, LOW);                 // Turn Arduino onboard LED off
  Dynamixel.ledState(SERVO_ID, OFF);        //Turn Dynamixel LED off
  delayMicroseconds(1);
  Dynamixel.servo(SERVO_ID,int(CCWLimit*11.375),0x7FF);    // Comman for servo mode, Move servo to max angle at max speed 
  
  //Reretrace Test Button
  tft.fillRect(247,210, 70, 25, MAGENTA);
  tft.drawRect(247,210, 70, 25, WHITE);
  tft.setCursor(257,215);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Test");
  
}

void CheckWheelMode(int SERVO_ID){
  
  digitalWrite(LED13, HIGH);                // Turn Arduino onboard LED on
  Dynamixel.ledState(SERVO_ID, ON);         // Turn Dynamixel LED on
  delayMicroseconds(1);
  Dynamixel.wheel(SERVO_ID,LEFT,0x3FF);     // Comman for Wheel mode, Move left at max speed  
  delay(4000);
  
  digitalWrite(LED13, LOW);                 // Turn Arduino onboard LED off
  Dynamixel.ledState(SERVO_ID, OFF);        //Turn Dynamixel LED off
  delayMicroseconds(1);
  Dynamixel.wheel(SERVO_ID,RIGHT,0x3FF);    // Comman for Wheel mode, Move right at max speed 
  delay(4000);
  Dynamixel.wheel(SERVO_ID,RIGHT,0x0);      // Comman for Wheel mode, Stop

  //Reretrace Test Button
  tft.fillRect(247,210, 70, 25, MAGENTA);
  tft.drawRect(247,210, 70, 25, WHITE);
  tft.setCursor(257,215);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Test");
}
