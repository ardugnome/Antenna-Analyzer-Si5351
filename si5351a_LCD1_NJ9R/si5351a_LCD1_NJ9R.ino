 // 
// Author: Hans Summers, 2015
// Website: http://www.hanssummers.com
//
// A very very simple Si5351a demonstration
// using the Si5351a module kit http://www.hanssummers.com/synth
// Please also refer to SiLabs AN619 which describes all the registers to use
//
//
// Update by Relu Jianu NJ9R 2/20/2019
// Adds Scan width between 100kHz and 5 MHz on A3
// Also added touch functionality for later update.
//


#include <inttypes.h>
#include "i2c.h"
#include "i2c.c"
#include "si5351a.h"

#include "SPI.h"
#include "TFT_ILI9341.h"

#include <URTouch.h>
URTouch  myTouch( 5, 6, 7, 3, 4);

int xCoord, yCoord;
char stCurrent[20]="";
int stCurrentLen=0;
char stLast[20]="";

#define TFT_GREY 0x7BEF

TFT_ILI9341 myGLCD = TFT_ILI9341(); 

// Variablen für Messung
float Fstart_MHz = 1.0;      // Start Frequency for sweep
//double Fstop_MHz = 10;    // Stop Frequency for sweep
float Fstop_MHz = 30.0;      // Stop Frequency for sweep
float current_freq_MHz;    // Temp variable used during sweep
long serial_input_number;   // Used to build number from serial stream
int num_steps = 101;        // Number of steps to use in the sweep
char incoming_char;         // Character read from serial stream
  float x;
float freq;                // Frequency
bool pc_flag;               // Use PC values and not Analog Value

// Variablen für Display
double vswrArray[120];       // Array for SWR
int z = 0;                   // Index for Array
float SwrFreq = 14.00;         // Variable for freq. with SWR Min.
double SwrMin = 100;         // Variable for SWR Min. 
float Freq1 = 1.00;            // Freq. Left bottom line display
float Freq2 = 15.00;           // Freq. Left center line display
float Freq3 = 30.00;           // Freq. Left right line display
unsigned long milliold = 0;  // Milliseconds for debouncing interrupt
bool flag = 0;               // we set to 1 at interrupt, in void loop perform_sweep
double counter = 0;          // Counter to ignore first interrupts
bool flag_graph = 1;         // Marker - Was Graph drawn?
bool deleteflag = 0;         // Marker Display Clear
float span=0;
// double VSWR;

/*************************
**   Custom functions   **
*************************/

void updateStr(int val)
{
  if (stCurrentLen<20)
  {
    stCurrent[stCurrentLen]=val;
    stCurrent[stCurrentLen+1]='\0';
    stCurrentLen++;
  }
}

// Draw a red frame while a button is touched
void waitForIt(int x1, int y1, int x2, int y2)
{
  while (myTouch.dataAvailable())
    myTouch.read();
}


void setup() {  

  Serial.begin(57600);

//Touch
// Initial setup
 
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_HI);

  
  //Used for Reset TFT
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  delay(100);
  digitalWrite(12, HIGH);
  delay(100);

  //TFT D/C = A3 in Setup.h
  
 // Setup the LCD
  myGLCD.init();
  myGLCD.setRotation(3); 
  myGLCD.fillScreen(TFT_BLACK);
  myGLCD.setTextColor(TFT_GREEN, TFT_BLACK);
  myGLCD.setCursor(0, 50);
  myGLCD.setTextSize(3);
  myGLCD.println("DG7EAO");
  myGLCD.println("   Antenna");
  myGLCD.println("      Analyser");
  myGLCD.setTextSize(2);
  myGLCD.println("");
  myGLCD.println("");
  myGLCD.println("");
  myGLCD.println("           by Relu Jianu");
  myGLCD.println("              NJ9R");
  delay(1000);
  counter = counter + 1;
  
  myGLCD.fillRect(0, 50, 300, 300,TFT_BLACK); 

  i2cInit();
  si5351aSetFrequency(10000000);

  // Set up analog inputs on A0 and A1, internal reference voltage
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  analogReference(INTERNAL); //1.1v on aref pin
  
 
  //Initialise the incoming serial number to zero
  serial_input_number=0; 

  //Tasten Interrupt an PIN 2
  pinMode(2,OUTPUT);
  digitalWrite(2, HIGH);
  attachInterrupt(0, key2, FALLING);
  unsigned long milliold = millis();       
}


void loop() 
{
while (true)
  {
  
      myTouch.read();
      
      xCoord=myTouch.getX();
      yCoord=myTouch.getY();
      
      Serial.print("X ");
      Serial.print(xCoord);
      Serial.print(", Y ");
      Serial.print(yCoord);
      Serial.println();
    
  
   //Touch anywhere to bring menu up
      
      if ((yCoord>=5) && (yCoord<=230))  
      {
        if ((xCoord>=5) && (xCoord<=310))  
        {
    waitForIt(5, 5, 230, 320);   //whole screen
    myGLCD.setTextSize(2);
    myGLCD.drawRect(1,180,155,319, TFT_GREEN);
    myGLCD.setCursor(25, 203);
    myGLCD.println("SET WIDTH");
    myGLCD.drawRect(165,180,230,319, TFT_GREEN);
    myGLCD.setCursor(215, 203);
    myGLCD.println("SWEEP");
    
      }
        if ((xCoord>=180) && (xCoord<=220))  
        {
          //width rectangle touch
          // x1,y1,x2,y2
          waitForIt(1,180,85,230);
          myGLCD.fillRect(1,180,155,319, TFT_RED);
          myGLCD.fillRect(165,180,230,319, TFT_RED);
          myGLCD.setTextColor(TFT_CYAN);
         
         //do something
        }
        else if ((xCoord>=180) && (xCoord<=319))  
        {
          //sweep rectangle touch
          // x1,y1,x2,y2
          waitForIt(165,180,300,230);
          myGLCD.fillRect(1,180,155,319, TFT_BLUE);
          myGLCD.fillRect(165,180,230,319, TFT_BLUE);
          //do something
        }
        
      }

 
    
  // Set Frequency
  si5351aSetFrequency(14100000);


String str;
if(Serial.available() > 0)
    {str = Serial.readStringUntil('\n'); }


// if pc_flag = true; take PC values and not analog setting    

// 10000000A Start
if (str.endsWith("A")){ 
  Fstart_MHz = str.substring(0, str.length() -1).toFloat()/1000000;
      pc_flag = true;}

// 20000000B Stop
if (str.endsWith("B")){
  Fstop_MHz = str.substring(0, str.length() -1).toFloat()/1000000;
      pc_flag = true;}

// C tune to this freq
if (str == "C"){
      //Turn frequency into FStart and set DDS output to single frequency
      Fstart_MHz = str.substring(1).toFloat()/1000000;
      //SetDDSFreq(Fstart_MHz);
      si5351aSetFrequency(Fstart_MHz * 1000000);}

 // 102N Steps
 if (str.endsWith("N")){
      // Set number of steps in the sweep
      num_steps = str.substring(0, str.length() -1).toFloat();
      pc_flag = true;
      }       

// s Start sweep
if (str == "s" | str == "S"){ Perform_sweep();}


// ? Print config
if (str == "?"){
     // Report current configuration to PC    
      Serial.print("Start Freq:");
      Serial.println(Fstart_MHz*1000000);
      Serial.print("Stop Freq:");
      Serial.println(Fstop_MHz*1000000);
      Serial.print("Num Steps:");
      Serial.println(num_steps);
}
    Serial.flush(); 

  // Perform Sweep Interrupt PIN2
  
 if (flag == 1 && counter >1)  
{
  Perform_sweep();  
  flag_graph = 1; 
} 

else
{  
 flag_graph = 0;

 if (deleteflag == 0){
 myGLCD.fillScreen(TFT_BLACK); 
 }
 
}

delay(100);


 //set span at X-1 or X+1 corresponding to A3 and add pot
float rawwidth = analogRead(A3);
 span=map(rawwidth,1,1023,1,50);

  // Read Soll on Pot A2
float soll = analogRead(A2);

  x = (soll / 30)+span/10;
  if (x > 30) x = 31; // full band scan
  if (soll > 850) x = 48.00; // 850 min
  if (soll > 890) x = 50.00; // 850 min
  if (soll > 925) x = 52.00; // 850 min
  if (soll > 970) x = 144.00; // 2m
  if (soll > 1000) x = 146.00;
  if (soll > 1020) x = 148.00;  
    

 // bei pc_flag = true; take PC values and not analog setting 
 
 
 // Setze Frequenzbereich
 if (pc_flag == false)
 {
 Fstart_MHz = (x-(span/10));  // Start Frequency for sweep
 Fstop_MHz = x+span/10;   // Stop Frequency for sweep 
 num_steps = 102;   // Steps
 }
 
 Freq1 = x-span/10;      // Bottom line Display Freq. Left
 Freq2 = x;        // Bottom line Display Freq. center
 Freq3 = x+span/10;      // Bottom line Display Freq. Right

  
 if (pc_flag == true)
 {
 Freq1 = Fstart_MHz;      // Bottom line Display Freq. Left
 Freq2 = Fstart_MHz + (Fstop_MHz - Fstart_MHz)/2; // Bottom line Display Freq. center
 Freq3 = Fstop_MHz;     // Bottom line Display Freq. Right
 }

 // Special with x = 0 do big scan
 if (x==31 && pc_flag == false)
 {
 Fstart_MHz = 1.0;  // Start Frequency for sweep
 Fstop_MHz = 30.0;   // Stop Frequency for sweep

 Freq1 = 1.0;      // Bottom line Display Freq. Left
 Freq2 = 15.0;     // Bottom line Display Freq. Middle  
 Freq3 = 30.0;     // Bottom line Display Freq. Right
 }
  
 
  if (flag_graph == 0 || counter == 1)
  {   

    myGLCD.setTextSize(3);
    myGLCD.setCursor(3, 10);
    myGLCD.println("Set Center Freq");
    myGLCD.setCursor(130,40);
    myGLCD.println(String(x) + " MHz");
       
    myGLCD.setTextSize(3);
    myGLCD.setCursor(3, 90);
    myGLCD.println("Set Scan Width");
    myGLCD.setCursor(130,120);
    myGLCD.println(String(span/10) + " MHz");
    deleteflag = 1;
  }
 } 
}

 
void Perform_sweep(){
  double FWD=0;//was1
  double REV=0;//was1
  double VSWR;
  double Fstep_MHz = (Fstop_MHz-Fstart_MHz)/num_steps;
  
  z = 0;
  SwrMin = 100; // was 100 maybe 10
   
  // Start loop 
  for(int i=0;i<=num_steps;i++){
    // Calculate current frequency
    current_freq_MHz = Fstart_MHz + i*Fstep_MHz;
    // Set DDS to current frequency
    // Serial.print(current_freq_MHz);
    si5351aSetFrequency(current_freq_MHz*1000000); 
    // Wait a little for settling
    delay(20);
    // Read the forward and reverse voltages
    
    REV = analogRead(A0);
    FWD = analogRead(A1);
     
     //Offset Korrektur ????added
  //REV = REV - 5;

        
    if(REV>=FWD){REV = FWD-1;}
    
    if (REV <1) {REV = 1;}
  
    VSWR = (FWD+REV)/(FWD-REV);
          
    //Skalieren für Ausgabe (Scaling for Output)
    VSWR = VSWR * 1000;  //was 1000
    
    // Kalibrierung
    // Widerstände > 50 Ohm = Load / 50 = SWR
    // Widerstände < 50 Ohm = 50 Ohm / Load = SWR
    
    // 100 Ohm >> SWR 2
    // 150 Ohm >> SWR 3
    // 470 Ohm >> SWR 9,4
    // 330 Ohm >> SWR 6.6
    // 250 Ohm >> SWR 5      
    
 // Corrections 
   if (x==31)
   {
  //   Serial.print("Full Sweep:");
     if ((VSWR <40500)&&(VSWR >11000))
      {VSWR=VSWR*0.45;}
     else 
     {VSWR=VSWR*1.68;}
   }
   //HF
   if ((Fstart_MHz>=1) && ( Fstop_MHz<35))
   {
   //Serial.print("HF:");
    if (VSWR <1100)
      {VSWR=1001;}
  else if ((VSWR>1401)&& (VSWR <7001))
     {VSWR=VSWR*1.31;}
  /* else if ((VSWR>7000 )&&(VSWR <10999))
      {VSWR=VSWR*0.85;} 
    else if ((VSWR>11000 )&&(VSWR<20500 ))
      {VSWR=VSWR*0.65;}*/
    else if ((VSWR >20501)&&(VSWR<45501 ))
      {VSWR=VSWR*0.4;}
    

   }
 
    //6m
    if ((Fstart_MHz>40) && ( Fstop_MHz<66))
   {
  //  Serial.print("6m:");
    if (VSWR <1500){VSWR=1001;}
     else if ((VSWR <18999)&&(VSWR >7000))
      {
        VSWR=VSWR*0.88;
       
      } 
      else if ((VSWR <29500)&&(VSWR >11000))
      {VSWR=VSWR*0.5;}
    else
    {
      VSWR=VSWR*1.75;
    }
   }

//   2m

   if ((Fstart_MHz>140) && ( Fstop_MHz<160))
   {
  //   Serial.print("2m:");
 
   if ( VSWR <1500 )
      { VSWR=1001;    }
   else if ((VSWR >7000)&&(VSWR <12299))
      { VSWR=VSWR*0.7;      }
   else if ((VSWR <45501)&&(VSWR >20501))
      { VSWR=VSWR*0.43;}   
   else if ((VSWR <6999)&&(VSWR >12300))
      { VSWR=VSWR*0.88;   }
   else{ VSWR=VSWR*1.45;  }
 
   }
   // Send current line back to PC over serial bus
    Serial.print(current_freq_MHz*1000000);
    Serial.print(",0,");
    Serial.print(VSWR);
    Serial.print(",");
    Serial.print(FWD);
    Serial.print(",");
    Serial.println(REV);
   
  
    // Übergebe SWR an Array (Pass SWR Array)
    // Ermittele Freq bei niedrigsten SWR(Determine freq at lowest SWR)
    vswrArray[z] = VSWR/1000;// was 1000
        
    if (vswrArray[z] > 10) vswrArray[z] = 10;
    
    if (vswrArray[z] < SwrMin && vswrArray[z] > 1) 
    {
    SwrMin = vswrArray[z];
    SwrFreq = current_freq_MHz;    
    }    
    
    z = z + 1;
  
  }
  // Send "End" to PC to indicate end of sweep
  Serial.println("End");
  Serial.flush(); 

     //graphic();
     CreateGrid();

    // double last = 10;
     double last = vswrArray[0];
     double xx = 6;
     int j = 1;

    for (int i = 1 ;i < 103; i++){
      xx = vswrArray[i];
      
  myGLCD.drawLine(j,210-last*18, j+1, 210-xx*18,TFT_RED);
  myGLCD.drawLine(j+1,210-last*18, j+2, 210-xx*18,TFT_RED);
 
      j = j + 3;
      last = xx;  
      }
}



// Interrupt Service Routine
// Abfrage Low an Pin 2 (Query on pin 2)
void key2()
{
  
  if ((millis() - milliold) < 1000) return;
  
  counter = counter + 1;   

  flag = !flag; 

  deleteflag= 0;

milliold = millis();
  
}




void CreateGrid()
{
  
  myGLCD.setTextSize(2);
  myGLCD.fillScreen(TFT_BLACK);
  myGLCD.setTextColor(TFT_GREEN, TFT_BLACK);

  myGLCD.setCursor(0, 0);

  myGLCD.print("SWR ");
  myGLCD.print(SwrMin);
  myGLCD.print("     Freq ");
  myGLCD.print(SwrFreq,3);
  
    double maxSwr = 10; //was10

        // Horizontale Linien (Horizontal Line)
        myGLCD.drawLine(0, 30, 310, 30,TFT_WHITE);
        myGLCD.drawLine(0, 120, 310, 120,TFT_WHITE);
        myGLCD.drawLine(0, 196, 310, 196,TFT_WHITE);
        myGLCD.drawLine(0, 210, 310, 210,TFT_WHITE);

        // Vertikale Linien (Vertical Line)
        myGLCD.drawLine(0, 30, 0, 210,TFT_WHITE);
        myGLCD.drawLine(78, 30, 78, 210,TFT_WHITE);
        myGLCD.drawLine(155, 30, 155, 210,TFT_WHITE);
        myGLCD.drawLine(233, 30, 233, 210,TFT_WHITE);
        myGLCD.drawLine(310, 30, 310, 210,TFT_WHITE);

        // 3 frequency labels
        myGLCD.setCursor(0, 215);
        myGLCD.print(Freq1,2);
        myGLCD.setCursor(123, 215);
        myGLCD.print(Freq2,2);
        myGLCD.setCursor(240, 215);
        myGLCD.print(Freq3,2);

        myGLCD.setTextSize(1);
        myGLCD.setCursor(5, 35);
        myGLCD.print(maxSwr);
        myGLCD.setCursor(5, 125);
        myGLCD.print("5");
}



// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//
void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t P1;          // PLL config register P1
  uint32_t P2;          // PLL config register P2
  uint32_t P3;          // PLL config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 1, (P3 & 0x000000FF));
  i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
  i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 4, (P1 & 0x000000FF));
  i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}

//
// Set up MultiSynth with integer divider and R divider
// R divider is the bit value which is OR'ed onto the appropriate register, it is a #define in si5351a.h
//
void setupMultisynth(uint8_t synth, uint32_t divider, uint8_t rDiv)
{
  uint32_t P1;          // Synth config register P1
  uint32_t P2;          // Synth config register P2
  uint32_t P3;          // Synth config register P3

  P1 = 128 * divider - 512;
  P2 = 0;             // P2 = 0, P3 = 1 forces an integer value for the divider
  P3 = 1;

  i2cSendRegister(synth + 0,   (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 1,   (P3 & 0x000000FF));
  i2cSendRegister(synth + 2,   ((P1 & 0x00030000) >> 16) | rDiv);
  i2cSendRegister(synth + 3,   (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 4,   (P1 & 0x000000FF));
  i2cSendRegister(synth + 5,   ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  i2cSendRegister(synth + 6,   (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 7,   (P2 & 0x000000FF));
}

//
// Switches off Si5351a output
// Example: si5351aOutputOff(SI_CLK0_CONTROL);
// will switch off output CLK0
//
void si5351aOutputOff(uint8_t clk)
{
  i2cSendRegister(clk, 0x80);   // Refer to SiLabs AN619 to see bit values - 0x80 turns off the output stage
}

// 
// Set CLK0 output ON and to the specified frequency
// Frequency is in the range 1MHz to 150MHz
// Example: si5351aSetFrequency(10000000);
// will set output CLK0 to 10MHz
//
// This example sets up PLL A
// and MultiSynth 0
// and produces the output on CLK0
//
void si5351aSetFrequency(uint32_t frequency)
{
  uint32_t pllFreq;
  uint32_t xtalFreq = XTAL_FREQ;
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t divider;

  divider = 900000000 / frequency;// Calculate the division ratio. 900,000,000 is the maximum internal 
                  // PLL frequency: 900MHz
  if (divider % 2) divider--;   // Ensure an even integer division ratio

  pllFreq = divider * frequency;  // Calculate the pllFrequency: the divider * desired output frequency

  mult = pllFreq / xtalFreq;    // Determine the multiplier to get to the required pllFrequency
  l = pllFreq % xtalFreq;     // It has three parts:
  f = l;              // mult is an integer that must be in the range 15..90
  f *= 1048575;         // num and denom are the fractional parts, the numerator and denominator
  f /= xtalFreq;          // each is 20 bits (range 0..1048575)
  num = f;            // the actual multiplier is  mult + num / denom
  denom = 1048575;        // For simplicity we set the denominator to the maximum 1048575

 
                  // Set up PLL A with the calculated multiplication ratio
  setupPLL(SI_SYNTH_PLL_A, mult, num, denom);
                  // Set up MultiSynth divider 0, with the calculated divider. 
                  // The final R division stage can divide by a power of two, from 1..128. 
                  // reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
                  // If you want to output frequencies below 1MHz, you have to use the 
                  // final R division stage
  setupMultisynth(SI_SYNTH_MS_0, divider, SI_R_DIV_1);
                  // Reset the PLL. This causes a glitch in the output. For small changes to 
                  // the parameters, you don't need to reset the PLL, and there is no glitch
  i2cSendRegister(SI_PLL_RESET, 0xA0);  
                  // Finally switch on the CLK0 output (0x4F) 
                  // and set the MultiSynth0 input to be PLL A
  i2cSendRegister(SI_CLK0_CONTROL, 0x4c | SI_CLK_SRC_PLL_A); ///keep this 4C setting
  
  //0x4C = lowest power yields better results
  /*
   *  i2cSendRegister(SI_CLK0_CONTROL,76);  // CLK0 strength = 2mA; power level ~ -10dB
   *   i2cSendRegister(SI_CLK0_CONTROL,77);  // CLK0 strength = 4mA; power level ~ -6dB
   *   i2cSendRegister(SI_CLK0_CONTROL,78);  // CLK0 strength = 6mA; power level ~ -3dB
   *   i2cSendRegister(SI_CLK0_CONTROL,79);  // CLK0 strength = 8mA; power level := 0dB
   */
    // in the future we can change the library to si5351.h and use the line below to set power
    // i2cSendRegister(SI5351_CLK0, SI5351_DRIVE_2MA);
}


  
