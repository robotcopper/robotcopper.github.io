---
title: "Dynamixel Servomotor Configurator"
time: 2022-08-01
---

# Dynamixel MX Series Servomotor Configurator

Dynamixel servo motors can be controlled in different ways : by [Wizard2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) the Dynamixel software or Raspberry with [pypot](https://docs.poppy-project.org/en/) each requiring a [USB2Dynamixel](https://www.eirlab.net/wp-content/uploads/2022/07/removal.ai_tmp-62e3cef3d053e.png) or by Raspberry or Arduino with [Dynamixel shields](https://www.eirlab.net/wp-content/uploads/2022/07/shield-dynamixel-arduino-robotis-3.jpg). However, to set the fundamental parameters of the dynamixel servo, it is generally the Dynamixel software on the computer that is used. This is very restrictive and that is why I was led to develop a small embedded system to facilitate the setting of Dynamixel MX series servos. 

## Table of Contents

## Equipment At My Disposal

- An Arduino Uno as motherboard
- An Arduino 2.8 TFT LCD Shield touch screen as user interface
- One MX-106 servo motor and one MX-28 servo motor for testing my project

## Simple Communication With The Servo Motor

To avoid having to purchase one of the above-mentioned connectors, the choice was made to control the servo via TTL serial communication from an Arduino Uno (i.e. using the Rx and Tx ports of the Arduino). However, reverting to the motor’s intrinsic communication protocol requires a good understanding of it and sending the right binary data packets. Another difficulty is that the servo data pin is a duplex for transmitted and received information. However, on the arduino the serial communication pins are separate. 
<br>

A thorough review of the state of the art led me to note that a similar project had already been done on a proprietary card by JosueGutierrez from [SavageElectronics](https://savageelectronics.com/dynamixel-smart-motor-configurator/). However, it is the work of [Mahyar Abdeetedal](https://www.etedal.net/2014/07/controlling-dynamixel-mx-64t.html) that has contributed to the development of the following elements. Indeed, despite the fact that Dynamixel has developed an Arduino library in partnership with its community, it was designed for the use of Dynamixel shields and was not conclusive for the desired use. To avoid developing a new library, research led me to the work of [J.Teda](https://code.google.com/archive/p/slide-33/) who created the [Dynamixel_Serial](https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/slide-33/Dynamixel_Serial%20V2_2.zip) library on Arduino.
<br>

Therefore, to communicate with the Dynamixel, the [SN74LS241](https://www.ti.com/lit/gpn/sn74ls241) 3-state buffer is required, allowing half-duplex communication according to the following circuit:

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/DynamixelConfigurator/circuit-arduino3.jpg" style="background: transparent;" width="60%" >
</div>
<p align="center" style="color:gray;">Communication circuit with the servomotor</p>

## Parameter Setting Function

Now able to communicate with the servo motor, it will be necessary to write the function to configure the servo motor whatever its current parameters.

<!-- a11y-dark theme colors -->
<div style="background:linear-gradient(135deg,rgb(0,0,0) 0%,rgb(59,18,18) 45%,rgb(51,12,30) 100%); padding: 10px;">
<pre><code lang="c">
<span style="color:#d4d0ab;">//======Library======//</span>

<span style="color:#ffa07a;">#</span><span style="color:#00e0e0;">include</span> <span style="color:#abe338;">&lt;Dynamixel_Serial.h&gt;</span>     <span style="color:#d4d0ab;">// Library needed to control Dynamixel servo</span>

<span style="color:#d4d0ab;">//======Servo parameters======//</span>

<span style="color:#ffa07a;">#</span><span style="color:#00e0e0;">define</span> <span style="color:#ffa07a;">SERVO_ControlPin</span> <span style="color:#00e0e0;">10</span>       <span style="color:#d4d0ab;">// Control pin of buffer chip</span>
<span style="color:#ffa07a;">#</span><span style="color:#00e0e0;">define</span> <span style="color:#ffa07a;">LED13</span> <span style="color:#00e0e0;">0x0D</span>                <span style="color:#d4d0ab;">// Pin of Visual indication for running "heartbeat" using onboard LED</span>

<span style="color:#d4d0ab;">//======Variable======//</span>

<span style="color:#00e0e0;">long</span> Baud[<span style="color:#00e0e0;">9</span>] = {<span style="color:#00e0e0;">9600</span>, <span style="color:#00e0e0;">19200</span>, <span style="color:#00e0e0;">57600</span>, <span style="color:#00e0e0;">115200</span>, <span style="color:#00e0e0;">200000</span>, <span style="color:#00e0e0;">250000</span>, <span style="color:#00e0e0;">400000</span>, <span style="color:#00e0e0;">500000</span>, <span style="color:#00e0e0;">1000000</span>};
bool MODE <span style="color:#00e0e0;">= 1</span>;                    <span style="color:#d4d0ab;">// ServoMode by default</span>
<span style="color:#00e0e0;">int</span> Torque <span style="color:#00e0e0;">= 100</span>;                 <span style="color:#d4d0ab;">// 100% torque by default</span>
<span style="color:#00e0e0;">unsigned int</span> CWLimit <span style="color:#00e0e0;">= 0x000 * 0.088</span>;  <span style="color:#d4d0ab;">// converts the hexadecimal input to degrees</span>
<span style="color:#00e0e0;">unsigned int</span> CCWLimit <span style="color:#00e0e0;">= 0xFFF * 0.088</span>;
</code></pre>
</div>

<br>

<div style="background:linear-gradient(135deg,rgb(0,0,0) 0%,rgb(59,18,18) 45%,rgb(51,12,30) 100%); padding: 10px;">
<pre><code lang="c">
<span style="color:#00e0e0;">void</span> <span style="color:#ffd700;">ResetServo</span>(){                               <span style="color:#d4d0ab;">// Pin setup for Visual indication of running (heart beat) program using onboard LED</span>
  <span style="color:#ffd700;">pinMode</span>(LED13, <span style="color:#00e0e0;">OUTPUT</span>);
  <span style="color:#ffd700;">digitalWrite</span>(LED13, <span style="color:#00e0e0;">HIGH</span>);

  <span style="color:#00e0e0;">for</span> (<span style="color:#00e0e0;">int</span> b = <span style="color:#00e0e0;">0</span>; b < <span style="color:#00e0e0;">9</span>; b++){                     <span style="color:#d4d0ab;">// This "for" loop will take about 20 Sec to complete and is used to loop through all speeds that Dynamixel can be and send reset instruction</span>
    <span style="color:#00e0e0;">long</span> Baudrate_BPS = <span style="color:#00e0e0;">0</span>;
    Baudrate_BPS  = Baud[b];
    <span style="color:#00e0e0;">if</span>(Baudrate_BPS == <span style="color:#00e0e0;">57600</span>){
      Dynamixel.<span style="color:#ffd700;">begin</span>(Baudrate_BPS, SERVO_ControlPin); <span style="color:#d4d0ab;">// Set Arduino Serial speed and control pin</span>
      Dynamixel.<span style="color:#ffd700;">reset</span>(<span style="color:#00e0e0;">0xFE</span>);                           <span style="color:#d4d0ab;">// Broadcast to all Dynamixel IDs and reset to factory default</span>
    }
    <span style="color:#00e0e0;">else</span>{
      Dynamixel.<span style="color:#ffd700;">begin</span>(Baudrate_BPS, SERVO_ControlPin); <span style="color:#d4d0ab;">// Set Arduino Serial speed and control pin</span>
      <span style="color:#00e0e0;">for</span> (<span style="color:#00e0e0;">int</span> i = <span style="color:#00e0e0;">1</span>; i < <span style="color:#00e0e0;">0xFF</span>; i++){
        Dynamixel.<span style="color:#ffd700;">reset</span>(i);
      }
      <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">5</span>);
    }
    <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">100</span>);                                      <span style="color:#d4d0ab;">// Time needed for Dynamixel to broadcast</span>
  }
  <span style="color:#ffd700;">digitalWrite</span>(LED13, <span style="color:#00e0e0;">LOW</span>);
  <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">3000</span>);                                     <span style="color:#d4d0ab;">// Give time for Dynamixel to reset</span>
}
</code></pre>
</div>

<br>

The ResetServo() function has the role of « making the servo programmable ». In fact, let’s consider at the beginning that the servomotor is of unknown ID and Baudrate. In this case, it is necessary to be able to impose working parameters before imposing those desired by the user. This is what this function does by using the reset instruction, imposing 57600bps as the working frequency.

<br>

<div style="background:linear-gradient(135deg,rgb(0,0,0) 0%,rgb(59,18,18) 45%,rgb(51,12,30) 100%); padding: 10px;">
<pre><code lang="c">
<span style="color:#00e0e0;">void</span> <span style="color:#ffd700;">ProgramBaudrateID</span>(<span style="color:#00e0e0;">long</span> SERVO_SET_Baudrate, <span style="color:#00e0e0;">int</span> SERVO_ID, <span style="color:#00e0e0;">bool</span> MODE, <span style="color:#00e0e0;">int</span> Torque, <span style="color:#00e0e0;">unsigned int</span> CWLimit, <span style="color:#00e0e0;">unsigned int</span> CCWLimit){
  <span style="color:#d4d0ab;">// Now that the Dynamixel is reset to factory setting we will program its Baudrate and ID</span>

  Dynamixel.<span style="color:#ffd700;">begin</span>(<span style="color:#00e0e0;">57600</span>, SERVO_ControlPin);         <span style="color:#d4d0ab;">// Set Arduino Serial speed to factory default speed of 57600</span>
  Dynamixel.<span style="color:#ffd700;">setID</span>(<span style="color:#00e0e0;">0xFE</span>, SERVO_ID);                  <span style="color:#d4d0ab;">// Broadcast to all Dynamixel IDs(0xFE) and set with new ID</span>
  <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">10</span>);                                        <span style="color:#d4d0ab;">// Time needed for Dynamixel to set its new ID before next instruction can be sent</span>
  Dynamixel.<span style="color:#ffd700;">setStatusPaket</span>(SERVO_ID, <span style="color:#00e0e0;">READ</span>);         <span style="color:#d4d0ab;">// Tell Dynamixel to only return status packets when a "read" instruction is sent</span>
  <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">30</span>);

  Dynamixel.<span style="color:#ffd700;">setBaudRate</span>(SERVO_ID, SERVO_SET_Baudrate);   <span style="color:#d4d0ab;">// Set Dynamixel to new serial speed</span>
  <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">30</span>);                                             <span style="color:#d4d0ab;">// Time needed for Dynamixel to set its new Baudrate</span>
  Dynamixel.<span style="color:#ffd700;">begin</span>(SERVO_SET_Baudrate, SERVO_ControlPin); <span style="color:#d4d0ab;">// We now need to set Arduino to the new Baudrate speed</span>

  Dynamixel.<span style="color:#ffd700;">ledState</span>(SERVO_ID, <span style="color:#00e0e0;">ON</span>);                      <span style="color:#d4d0ab;">// Turn Dynamixel LED on</span>
  <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">5</span>);
  Dynamixel.<span style="color:#ffd700;">setMode</span>(SERVO_ID, MODE, <span style="color:#00e0e0;">int</span>(CWLimit * <span style="color:#00e0e0;">11.375</span>), <span style="color:#00e0e0;">int</span>(CCWLimit * <span style="color:#00e0e0;">11.375</span>));  <span style="color:#d4d0ab;">// Set mode to SERVO, must be WHEEL if using wheel mode</span>
  <span style="color:#ffd700;">delay</span>(<span style="color:#00e0e0;">30</span>);
  Dynamixel.<span style="color:#ffd700;">setMaxTorque</span>(SERVO_ID, <span style="color:#00e0e0;">int</span>(Torque * <span style="color:#00e0e0;">7.67</span>));  <span style="color:#d4d0ab;">// Set Dynamixel to max torque limit</span>
}
</code></pre>
</div>

<br>

Now programmable, user parameters are applied to the servomotor : Baudrate, ID, Operating mode, torque, min and max angle.

## User Interface

Now that the major function is done, the user must be able to impose its parameters. To do this, it was chosen to use the 2.8 TFT LCD Shield touch screen. The GFX library from Adafruit was used to create the graphical interface and is based on the following idea :

<div style="background:linear-gradient(135deg,rgb(0,0,0) 0%,rgb(59,18,18) 45%,rgb(51,12,30) 100%); padding: 10px;">
<pre><code lang="c">
<span style="color:#d4d0ab;">//======Library======//</span>

<span style="color:#ffa07a;">#</span><span style="color:#00e0e0;">include</span> <span style="color:#abe338;">&lt;Adafruit_TFTLCD.h&gt;</span> 
<span style="color:#ffa07a;">#</span><span style="color:#00e0e0;">include</span> <span style="color:#abe338;">&lt;Adafruit_GFX.h&gt;</span>
<span style="color:#ffa07a;">#</span><span style="color:#00e0e0;">include</span> <span style="color:#abe338;">&lt;TouchScreen.h&gt;</span>

<span style="color:#d4d0ab;">//======GPIO======//</span>
<span style="color:#d4d0ab;">[...]</span>

<span style="color:#d4d0ab;">//======Screen Calibration======//</span>
<span style="color:#d4d0ab;">[...]</span>

<span style="color:#d4d0ab;">//======Colors======//</span>
<span style="color:#d4d0ab;">[...]</span>

<span style="color:#d4d0ab;">//======tft object declaration======//</span>

<span style="color:#00e0e0;">Adafruit_TFTLCD</span> tft(<span style="color:#ffa07a;">LCD_CS</span>, <span style="color:#ffa07a;">LCD_CD</span>, <span style="color:#ffa07a;">LCD_WR</span>, <span style="color:#ffa07a;">LCD_RD</span>, <span style="color:#ffa07a;">LCD_RESET</span>);
<span style="color:#00e0e0;">TouchScreen</span> ts = <span style="color:#00e0e0;">TouchScreen</span>(<span style="color:#ffa07a;">XP</span>, <span style="color:#ffa07a;">YP</span>, <span style="color:#ffa07a;">XM</span>, <span style="color:#ffa07a;">YM</span>, <span style="color:#00e0e0;">300</span>);

<span style="color:#d4d0ab;">//======Variable======//</span>

<span style="color:#00e0e0;">char</span> currentPage;
</code></pre>
</div>

<br>

<div style="background:linear-gradient(135deg,rgb(0,0,0) 0%,rgb(59,18,18) 45%,rgb(51,12,30) 100%); padding: 10px;"><pre><code lang="c">
<span style="color:#00e0e0;">void</span> <span style="color:#ffd700;">setup</span>() {                       
  <span style="color:#d4d0ab;">// Initial LCD setup //</span><br>
  tft.<span style="color:#ffd700;">reset</span>();
  tft.<span style="color:#ffd700;">begin</span>(<span style="color:#00e0e0;">0x9341</span>);
  tft.<span style="color:#ffd700;">setRotation</span>(<span style="color:#00e0e0;">3</span>);<br>
  <span style="color:#ffd700;">DrawHomeScreen</span>();
  currentPage = <span style="color:#abe338;">'0'</span>;         <span style="color:#d4d0ab;">// Indicates that we are at Home Screen</span>
}
</code></pre>
</div>

<br>

The setup displays the wallpaper once on switch-on after initializing it.

<div style="background:linear-gradient(135deg,rgb(0,0,0) 0%,rgb(59,18,18) 45%,rgb(51,12,30) 100%); padding: 10px;">
<pre><code lang="c">
<span style="color:#00e0e0;">void</span> <span style="color:#ffd700;">loop</span>() {                          
  <span style="color:#00e0e0;">if</span>(currentPage == <span style="color:#abe338;">'0'</span>) {                 
    TSPoint p = ts.<span style="color:#ffd700;">getPoint</span>();    <span style="color:#d4d0ab;">// Get touch point</span><br>
    <span style="color:#00e0e0;">if</span> (p.z > ts.pressureThreshhold) {
        p.x = <span style="color:#ffd700;">map</span>(p.x, TS_MAXX, TS_MINX, <span style="color:#00e0e0;">0</span>, <span style="color:#00e0e0;">320</span>);
        p.y = <span style="color:#ffd700;">map</span>(p.y, TS_MAXY, TS_MINY, <span style="color:#00e0e0;">0</span>, <span style="color:#00e0e0;">240</span>);<br>
        <span style="color:#00e0e0;">if</span>(p.x > <span style="color:#00e0e0;">30</span> && p.x < <span style="color:#00e0e0;">285</span> && p.y > <span style="color:#00e0e0;">130</span> && p.y < <span style="color:#00e0e0;">200</span>) {    <span style="color:#d4d0ab;">// The user has pressed inside the red rectangle</span>
        <span style="color:#d4d0ab;">// This is important, because the libraries are sharing pins</span>
        <span style="color:#ffd700;">pinMode</span>(XM, <span style="color:#00e0e0;">OUTPUT</span>);
        <span style="color:#ffd700;">pinMode</span>(YP, <span style="color:#00e0e0;">OUTPUT</span>);
        currentPage = <span style="color:#abe338;">'1'</span>;
        <span style="color:#ffd700;">DrawBaudRateSetup</span>();     
      }
    }
  }
  
  <span style="color:#d4d0ab;">// [...]</span>
}
</code></pre>
</div>

<br>

The loop is responsible for checking at any time whether there has been an interaction on the screen by looping over « ifs » and, if necessary, displaying a new feature.

<div style="background:linear-gradient(135deg,rgb(0,0,0) 0%,rgb(59,18,18) 45%,rgb(51,12,30) 100%); padding: 10px;">
<pre><code lang="c">
<span style="color:#00e0e0;">void</span> <span style="color:#ffd700;">DrawHomeScreen</span>() {                             
  <span style="color:#d4d0ab;">// Background in black //</span>
  tft.<span style="color:#ffd700;">fillScreen</span>(BLACK);<br>
  <span style="color:#d4d0ab;">// Draw white frame</span>
  tft.<span style="color:#ffd700;">drawRect</span>(<span style="color:#00e0e0;">0</span>, <span style="color:#00e0e0;">0</span>, <span style="color:#00e0e0;">319</span>, <span style="color:#00e0e0;">240</span>, WHITE);<br>
  <span style="color:#d4d0ab;">// Print "Dynamixel" Text</span>
  tft.<span style="color:#ffd700;">setCursor</span>(<span style="color:#00e0e0;">110</span>, <span style="color:#00e0e0;">30</span>);
  tft.<span style="color:#ffd700;">setTextColor</span>(WHITE);
  tft.<span style="color:#ffd700;">setTextSize</span>(<span style="color:#00e0e0;">2</span>);
  tft.<span style="color:#ffd700;">print</span>(<span style="color:#abe338;">"Dynamixel"</span>);<br>
  <span style="color:#d4d0ab;">// Print "Configuration of MX servo" Text</span>
  tft.<span style="color:#ffd700;">setCursor</span>(<span style="color:#00e0e0;">10</span>, <span style="color:#00e0e0;">50</span>);
  tft.<span style="color:#ffd700;">setTextColor</span>(WHITE);
  tft.<span style="color:#ffd700;">setTextSize</span>(<span style="color:#00e0e0;">2</span>);
  tft.<span style="color:#ffd700;">print</span>(<span style="color:#abe338;">"Configuration of MX servo"</span>);<br>
  <span style="color:#d4d0ab;">// Create Red Button</span>
  tft.<span style="color:#ffd700;">fillRect</span>(<span style="color:#00e0e0;">30</span>, <span style="color:#00e0e0;">130</span>, <span style="color:#00e0e0;">260</span>, <span style="color:#00e0e0;">70</span>, DRED);
  tft.<span style="color:#ffd700;">drawRect</span>(<span style="color:#00e0e0;">30</span>, <span style="color:#00e0e0;">130</span>, <span style="color:#00e0e0;">260</span>, <span style="color:#00e0e0;">70</span>, WHITE);
  tft.<span style="color:#ffd700;">setCursor</span>(<span style="color:#00e0e0;">118</span>, <span style="color:#00e0e0;">138</span>);
  tft.<span style="color:#ffd700;">setTextColor</span>(WHITE);
  tft.<span style="color:#ffd700;">setTextSize</span>(<span style="color:#00e0e0;">3</span>);
  tft.<span style="color:#ffd700;">print</span>(<span style="color:#abe338;">"Start"</span>);
  tft.<span style="color:#ffd700;">setCursor</span>(<span style="color:#00e0e0;">40</span>, <span style="color:#00e0e0;">168</span>);
  tft.<span style="color:#ffd700;">setTextColor</span>(WHITE);
  tft.<span style="color:#ffd700;">setTextSize</span>(<span style="color:#00e0e0;">3</span>);
  tft.<span style="color:#ffd700;">print</span>(<span style="color:#abe338;">"configuration!"</span>);
}
</code></pre>
</div>

<br>

To draw, loop and setup independent functions are created for each page being displayed that need to be called at the desired time as shown above.

## Implementation Of The Project

Almost finished, it is only a question of making the electrical circuit and its box.

###  Electrical Circuit
<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/DynamixelConfigurator/Schema_Kicad-1024x578.jpg" style="background: transparent;" width="37%" >
    <img src="/config/assets/images/DynamixelConfigurator/PCB_Kicad.jpg" style="background: transparent;" width="30%" >
</div>
<p align="center" style="color:gray;">Electronic schematic and PCB</p>

###  Enclosure
<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/DynamixelConfigurator/Boitier3D4.png" style="background: transparent; height: 100%; object-fit: contain;" width="30%">
    <img src="/config/assets/images/DynamixelConfigurator/Boitier3D2.png" style="background: transparent; height: 100%; object-fit: contain;" width="30%">
    <img src="/config/assets/images/DynamixelConfigurator/Boitier3D_eclate.png" style="background: transparent; height: 100%; object-fit: contain;" width="30%">
</div>

<p align="center" style="color:gray;">3D model</p>

## Final Test
<div style="display: flex; justify-content: center;">
    <video width="100%" height="480" controls>
        <source src="/config/assets/images/DynamixelConfigurator/deployment_configurator.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>
</div>

## Downloadable Resources

<a href="/config/assets/images/DynamixelConfigurator/Projet_final_MX_config-1.ino" download>
    Full project code
</a><br>
<a href="/config/assets/images/DynamixelConfigurator/TopProjetDynamixel.svg" download>
    Top of the enclosure made by laser cutter
</a><br>
<a href="/config/assets/images/DynamixelConfigurator/Electrical_circuit.7z
" download>
    Electrical circuit
</a><br>
[3D model](https://cad.onshape.com/documents/94720394670121bc889dd342/w/a171c5eb550a3dcf7c7859b6/e/4ac920b9838d332c02344668?renderMode=0&uiState=62e7da4e2cd35275a5ea0e38)