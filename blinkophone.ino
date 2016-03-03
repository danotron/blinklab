#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

// all of the neopattern class shit here is from the adafruit 'multitasking the arduino' tutorial
// i've dicked around with it a little bit but mostly it's from there

// Pattern types:
enum  pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, SCANNER_CHASE, COLOR_WIPE, SCANNER, DIRECTIONAL_SCANNER, FIRE };
// Patern directions:
enum  direction { FORWARD, REVERSE };


// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
	public:
	// Member Variables:  
	pattern ActivePattern;		// which pattern is running
	direction Direction;		// direction to run the pattern

	unsigned long Interval;		// milliseconds between updates
	unsigned long lastUpdate;	// last update of position

	uint32_t Color1, Color2;	// What colors are in use
	uint16_t TotalSteps;		// total number of steps in the pattern
	uint16_t Index;				// current step within the pattern
	void (*OnComplete)();		// Callback on completion of pattern

	// Constructor - calls base-class constructor to initialize Sides
	NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
	:Adafruit_NeoPixel(pixels, pin, type)
	{
		OnComplete = callback;
	}

	// Update the pattern
	void Update()
	{
		if((millis() - lastUpdate) > Interval) // time to update
		{
			lastUpdate = millis();
			switch(ActivePattern)
			{
  				case FIRE:
					FireUpdate();
					break;
				case RAINBOW_CYCLE:
					RainbowCycleUpdate();
					break;
				case THEATER_CHASE:
					TheaterChaseUpdate();
					break;
				case SCANNER_CHASE:
					ScannerChaseUpdate();
					break;
				case COLOR_WIPE:
					ColorWipeUpdate();
					break;
				case SCANNER:
					ScannerUpdate();
					break;
				case DIRECTIONAL_SCANNER:
					DirectionalScannerUpdate();
					break;
				default:
					break;
			}
		}
	}
	
	// Increment the Index and reset at the end
	void Increment()
	{
        if (Direction == FORWARD)
        {
           Index++;
           if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
        else //(Direction == REVERSE)
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps-1;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
	}
    
	// Reverse pattern direction
	void Reverse()
	{
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps-1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
	}
    
	// ------- vv Fire pattern vv -------
	void Fire(uint8_t interval, direction dir = FORWARD)
	{
          ActivePattern = FIRE;
          Interval = interval;
          TotalSteps = numPixels();
          Index = 0;
          Direction = dir;
	}
    
	void FireUpdate()
	{
          for(int i=0; i< numPixels(); i++)
          {
            int FireR = 255;
            int FireG = FireR-100;
            int FireB = 75;
            int flicker = random(0,150);
            int FireR1 = FireR-flicker;
            int FireG1 = FireG-flicker;
            int FireB1 = FireB-flicker;
            if(FireR1<0) FireR1=0;
            if(FireG1<0) FireG1=0;
            if(FireB1<0) FireB1=0;
            setPixelColor(i, FireR1, FireG1, FireB1);
          }
          show();
          Increment();
	}
	// ------- ^^ Fire pattern ^^ -------
	

	// ------- vv RainbowCycle pattern vv -------
	void RainbowCycle(uint8_t interval, direction dir = FORWARD)
	{
          ActivePattern = RAINBOW_CYCLE;
          Interval = interval;
          TotalSteps = 255;
          Index = 0;
          Direction = dir;
	}

	void RainbowCycleUpdate()
	{
          for(int i=0; i< numPixels(); i++)
          {
              setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
          }
          show();
          Increment();
	}
	// ------- ^^ RainbowCycle pattern ^^ -------
    
	
	
	// ------- vv TheaterChase pattern vv -------
	void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
	{
          ActivePattern = THEATER_CHASE;
          Interval = interval;
          TotalSteps = numPixels();
          Color1 = color1;
          Color2 = color2;
          Index = 0;
          Direction = dir;
	}

	void TheaterChaseUpdate()
	{
          for(int i=0; i< numPixels(); i++)
          {
              if ((i + Index) % 3 == 0)
              {
                  setPixelColor(i, Color1);
              }
              else
              {
                  setPixelColor(i, Color2);
              }
          }
          show();
          Increment();
	}
	// ------- ^^ TheaterChase pattern ^^ -------

	
	
	
	// ------- vv ScannerChase pattern vv -------
	void ScannerChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
	{
          ActivePattern = SCANNER_CHASE;
          Interval = interval;
          TotalSteps = (numPixels() - 1);
          Color1 = color1;
          Color2 = color2;
          Index = 0;
          Direction = dir;
	}

	void ScannerChaseUpdate()
	{
          for(int i=0; i< numPixels(); i++)
          {
              if (i == TotalSteps - Index)
              {
                  setPixelColor(i, Color1);
              }
              else
              {
                   setPixelColor(i, DimColor(getPixelColor(i)));
              }
          }
          show();
          Increment();
	}
	// ------- ^^ ScannerChase pattern ^^ -------
	
	// ------- vv ColorWipe pattern vv -------
	void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
	{
          ActivePattern = COLOR_WIPE;
          Interval = interval;
          TotalSteps = numPixels();
          Color1 = color;
          Index = 0;
          Direction = dir;
	}

	void ColorWipeUpdate()
	{
          setPixelColor(Index, Color1);
          show();
          Increment();
	}
	// ------- ^^ ColorWipe pattern ^^ -------
	
	
	
	
	// ------- vv Scanner pattern vv -------
	void Scanner(uint32_t color1, uint8_t interval)
	{
          ActivePattern = SCANNER;
          Interval = interval;
          TotalSteps = (numPixels() - 1) * 2;
          Color1 = color1;
          Index = 0;
	}

	void ScannerUpdate()
	{ 
          for (int i = 0; i < numPixels(); i++)
          {
            if (i == Index)  // Scan Pixel to the right
              {  setPixelColor(i, Color1);  }
            else if (i == TotalSteps - Index) // Scan Pixel to the left
              {  setPixelColor(i, Color1);  }
            else // Fading tail
              {  setPixelColor(i, DimColor(getPixelColor(i)));  }
          }
          show();
          Increment();
	}
	// ------- ^^ Scanner pattern ^^ -------

	
	
	
	// ------- vv DirectionalScanner pattern vv -------
	void DirectionalScanner(uint32_t color1, uint8_t interval, direction dir = FORWARD)
	{
          ActivePattern = DIRECTIONAL_SCANNER;
          Interval = interval;
          TotalSteps = (numPixels() - 1);
          Color1 = color1;
          Index = 0;
          Direction = dir;
	}

	void DirectionalScannerUpdate()
	{ 
          for (int i = 0; i < numPixels(); i++)
          {
              if (i == Index)  // Scan Pixel to the right
                {  setPixelColor(i, Color1);  }
              else // Fading tail
              {  setPixelColor(i, DimColor(getPixelColor(i)));  }
              if (i == TotalSteps){
                setPixelColor(i, Color1);
                setPixelColor((i - 1), DimColor(getPixelColor((i - 1))));
                setPixelColor((i - 2), DimColor(getPixelColor((i - 2))));
              }
          }
          show();
          Increment();
	}
	// ------- ^^ DirectionalScanner pattern ^^ -------
	

    
    
    

	// ------- vv Pattern utilities vv -------
	// Calculate 50% dimmed version of a color (used by ScannerUpdate)
	uint32_t DimColor(uint32_t color)
	{
          // Shift R, G and B components one bit to the right
          uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
          return dimColor;
	}

	// Set all pixels to a color (synchronously)
	void ColorSet(uint32_t color)
	{
          for (int i = 0; i < numPixels(); i++)
            {  setPixelColor(i, color);  }
          show();
	}

	// Returns the Red component of a 32-bit color
	uint8_t Red(uint32_t color)
        {
          return (color >> 16) & 0xFF;
        }

	// Returns the Green component of a 32-bit color
	uint8_t Green(uint32_t color)
        {
          return (color >> 8) & 0xFF;
        }

	// Returns the Blue component of a 32-bit color
	uint8_t Blue(uint32_t color)
        {
          return color & 0xFF;
        }

	// Input a value 0 to 255 to get a color value.
	// The colours are a transition r - g - b - back to r.
	uint32_t Wheel(byte WheelPos)
	{
          WheelPos = 255 - WheelPos;
          if(WheelPos < 85)
          {
              return Color(255 - WheelPos * 3, 0, WheelPos * 3);
          }
          else if(WheelPos < 170)
          {
              WheelPos -= 85;
              return Color(0, WheelPos * 3, 255 - WheelPos * 3);
          }
          else
          {
              WheelPos -= 170;
              return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
          }
	}
	// ------- ^^ Pattern utilities ^^ -------
};

// hardware definitions
#define BrightPin A0		// potentiometer
#define MicPin A2		// microphone
const int LeftButtonPin = 8;    // green button on left side of handle
const int RightButtonPin = 4;   // red button on right side of handle
const int MainPin = 6;		// main light array in cone
const int CenterPin = 5;	// ring on center pokey-thingy in middle of cone
const int num_MainLeds = 80;    // # of leds in main array
const int num_CenterLeds = 12;  // # of leds in center ring

// vars for brightness control
int val = 0;                     // brightness pot value
int brightLevel = 0;             // brightness level, mapped to pot value

// vars for left button, program cycler
int leftButtonPushCounter = 0;   // counter for the number of button presses
int leftButtonState = 0;         // current state of the button
int lastLeftButtonState = 0;     // previous state of the button
int leftButtonMax = 5;           // button counter max value, resets to 0 when value reached

// vars for right button, mode selector
int rightButtonPushCounter = 0;   // counter for the number of button presses
int rightButtonState = 0;         // current state of the button
int lastRightButtonState = 0;     // previous state of the button
int rightButtonMax = 2;           // button counter max value, resets to 0 when value reached
int soundMode=0;

// microphone sensor related bullshit
#define DC_OFFSET  0              // DC offset in mic signal - if unusure, leave 0
#define NOISE     350             // Noise/hum/interference in mic signal
#define SAMPLES   60              // Length of buffer for dynamic level adjustment
#define TOP       (225)           // Allow dot to go slightly off scale # now used as brightness setting max

byte
  peak      = 0,                  // Used for falling dot
  dotCount  = 0,                  // Frame counter for delaying dot-falling speed
  volCount  = 0;                  // Frame counter for storing past volume data
  
int
  vol[SAMPLES],                   // Collection of prior volume samples
  lvl       = 15,                 // Current "dampened" audio level
  minLvlAvg = 0,                  // For dynamic adjustment of graph low & high
  maxLvlAvg = 768;

// declare led objects
NeoPatterns Main(num_MainLeds, MainPin, NEO_GRB + NEO_KHZ800, &MainComplete);
NeoPatterns Center(num_CenterLeds, CenterPin, NEO_GRB + NEO_KHZ800, &CenterComplete);

// component array
const int NumComponents = 2;
NeoPatterns *Components[NumComponents];









void setup() {
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);  // some clocking bullshit i guess
  Serial.begin(115200);	                                   // start serial for some reason
  OCR0A = 0xAF;                                            // interrupt timer or some shit
  TIMSK0 |= _BV(OCIE0A);                                   // i don't know
  pinMode(LeftButtonPin, INPUT);	                   // set button input mode
//  pinMode(RightButtonPin, INPUT);	// set button input mode

  // microphone sensor thing probably
  memset(vol, 0, sizeof(vol));
  randomSeed(analogRead(3));

  // add leds to component array  
  Components[0] = &Main;
  Components[1] = &Center;

  // initialize leds
  for (int i=0; i<NumComponents; ++i)
  {
    Components[i]->begin();
    Components[i]->Scanner(Main.Color(255,0,0), 55);
  }
}

// hardware interrupt handler
SIGNAL(TIMER0_COMPA_vect){
  
    // left button handler
    leftButtonState = digitalRead(LeftButtonPin);         // read state of momentary switch
    if (leftButtonState != lastLeftButtonState)  {        // if it's not the same as it was the last time you read it...
      if (leftButtonState == HIGH)  {                     // and if it's HIGH....
        leftButtonPushCounter++;                          // increment the button counter...
        if (leftButtonPushCounter == leftButtonMax)  {    // and if the button counter is now at the counter maximum....
          leftButtonPushCounter = 0;  }                   // reset the counter to 0.
      }
    }
    lastLeftButtonState = leftButtonState;                // now set the last button state to the current button state for examination the next time around
  
    // right button handler
    rightButtonState = digitalRead(RightButtonPin);       // read state of momentary switch
    if (rightButtonState != lastRightButtonState)  {      // if it's not the same as it was the last time you read it...
      if (rightButtonState == HIGH)  {                    // and if it's HIGH....
        rightButtonPushCounter++;                         // increment the button counter...
        if (rightButtonPushCounter == rightButtonMax)  {  // and if the button counter is now at the counter maximum....
          rightButtonPushCounter = 0;  }                  // reset the counter to 0.
      }
    }
    lastRightButtonState = rightButtonState;              // now set the last button state to the current button state for examination the next time around
    
    // microphone reading
    uint16_t minLvl, maxLvl;
    uint32_t t;
    uint32_t t2;
    int      n, height;
    n   = analogRead(MicPin);                 // Raw reading from mic 
    n   = abs(n - 512 - DC_OFFSET);            // Center on zero
    n   = (n <= NOISE) ? 0 : (n - NOISE);      // Remove noise/hum
    lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
    
    // Calculate bar height based on dynamic min/max levels (fixed point):
    height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
  
    if(height < 0L)       height = 0;      // Clip output
    else if(height > TOP) height = TOP;
    if(height > peak)     peak   = height; // Keep 'peak' dot at top
    
    // toggle sound mode
    if(rightButtonPushCounter == 0)  {    soundMode = 0;  }
    if(rightButtonPushCounter == 1)  {    soundMode = 1;  }  
  
  
  
    // default to sound mode enabled at power on.  set led brightness to 'height' from microphone reading.
    if (soundMode == 0){
      for (int i=0; i<NumComponents; ++i)
      {
        Components[i]->setBrightness(height);
        Components[i]->Update();
      }
    }
  
    // read pot value and map to brightness range; 0 to 1023 is the analog value, 0 to 100 is the mapped brightness value
    if (soundMode == 1){
      val = analogRead(BrightPin);
      brightLevel = map(val, 0, 1023, 1, 200);
      for (int i=0; i<NumComponents; ++i)
      {
        Components[i]->setBrightness(brightLevel);
        Components[i]->Update();
      }
    }

    // and here's some more microphone bullshit
    vol[volCount] = n;                      // Save sample for dynamic leveling
    if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
    // Get volume range of prior frames
    minLvl = maxLvl = vol[0];
    for(int i=1; i<SAMPLES; i++) {
      if(vol[i] < minLvl)      minLvl = vol[i];
        else if(vol[i] > maxLvl) maxLvl = vol[i];
    }
    if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
    minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
    maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
    
 
 
 
  
}

void loop() {
    ChangePattern();                          // call the changepattern function.
}

void ChangePattern(){

        if( leftButtonPushCounter ==  0)
        {
          Main.ActivePattern = THEATER_CHASE;
          Main.Interval = 50;
          Main.TotalSteps = Main.numPixels();
      
          Center.ActivePattern = COLOR_WIPE;
          Center.Interval = random(30,100);
          Center.TotalSteps = Center.numPixels();
          Center.Direction = FORWARD;
      	
        }
        
        if( leftButtonPushCounter ==  1)
        {
          Main.ActivePattern = COLOR_WIPE;
          Main.Interval = random(30,100);
          Main.TotalSteps = Main.numPixels();
      
          Center.ActivePattern = COLOR_WIPE;
          Center.Interval = random(30,100);
          Center.TotalSteps = Center.numPixels();
          Center.Direction = FORWARD;
      	
        }
        
        if( leftButtonPushCounter ==  2)
        {
          Main.ActivePattern = RAINBOW_CYCLE;
          Main.Interval = random(2,50);
          Main.TotalSteps = 255;
      
          Center.ActivePattern = RAINBOW_CYCLE;
          Center.TotalSteps = 255;
          Center.Interval = random(2,50);
          Center.Direction = REVERSE;
        }
        
        if( leftButtonPushCounter ==  3)
        {  
          Main.ActivePattern = FIRE;
          Main.Interval = random(30,100);
          Main.TotalSteps = Main.numPixels();
          
          Center.ActivePattern = FIRE;
          Center.Interval = random(30,100);
          Center.TotalSteps = Center.numPixels();
          Center.Direction = REVERSE;
        }
}



//------------------------------------------------------------
//Completion Routines - get called on completion of a pattern
//------------------------------------------------------------

// Head Completion Callback
void MainComplete()
{
      Main.Color1 = Main.Wheel(random(255));
}

	
// Mohawk Completion Callback
void CenterComplete()
{
    // Random color change for next scan
    Center.Color1 = Center.Wheel(random(255));
}
