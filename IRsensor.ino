#include  
#include 

// Set the LCD address to 0x27 for a 16 chars and 2 line display


const uint8_t I2C_ADDRESS =0x27;
const uint8_t LCD_CHAR= 16;
const uint8_t LCD_ROW= 2;
LiquidCrystal_I2C lcd(I2C_ADDRESS, LCD_CHAR,LCD_ROW);

#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

int distance=0;
void setup()
{
	Wire.begin();
	lcd.begin();

	// Turn on the blacklight and print a message.
	lcd.backlight();


   if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
      lcd.print("Sensor failed");
      lcd.setCursor(0,1);
      lcd.print("check wiring!");      
    while (1)
      ;
  }

   //distanceSensor.setDistanceModeShort();
  distanceSensor.setDistanceModeLong();
      lcd.print("Robojax VL53L1X");
      lcd.setCursor(0,1);
      lcd.print("Mode: Medium"); 
      //lcd.print("Mode: Long");          
      delay(2000);       
      clearCharacters(LCD_ROW-1,0, LCD_CHAR-1);
      lcd.setCursor(0,1);
      lcd.print("Distance: ");         

}//setup() end



void loop()
{
  getDistance();
	printDistance('m');

  // if distance is greator thatn 185mm do something
  if(distance >185)
  {
    // somethig here
  }
 delay(300);
}




/*
 * getDistance()
 * displays gets the distance for VL53L1X laser sensor
 * updates the "distance" variable with distance in mm
 * 
 */
int getDistance()
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement

  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();

  distanceSensor.stopRanging();
  
}//getDistance()  end


/*
 * printDistance(char unit)
 * displays value and title on LCD1602
 * How to use:
 * printDistance('m');

 */
void printDistance(char unit)
{
   // Robojax.com LCD1602 for VL53L1X d
  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;
  float distanceMeter = distance/1000.00;
  float distanceCm= distance/10.00;

  
   clearCharacters(1,9, LCD_CHAR-1 );
   lcd.setCursor (9,1); //
   if(unit =='i')
   {
    lcd.print(distanceInches);    
    lcd.print("in");
    
   }
   if(unit =='m')
   {
    lcd.print(distance);    
    lcd.print("mm");
    
   }
   if(unit =='f')
   {
    lcd.print(distanceFeet);    
    lcd.print("ft");
   }
   if(unit =='c')
   {
    lcd.print(distanceCm);    
    lcd.print("cm");
   }  
   if(unit =='a')
   {
    lcd.print(distanceMeter);    
    lcd.print("m");
   }         
 // Robojax.com LCD1602 for LM35 Demo
}  

/*
   clearCharacters(uint8_t row,uint8_t start, uint8_t stop)
 * @brief clears a line of display (erases all characters)
 * @param none
 * @return does not return anything
 * Written by Ahmad Shamshiri
 * www.Robojax.com code May 28, 2020 at 16:21 in Ajax, Ontario, Canada
 */
void clearCharacters(uint8_t row,uint8_t start, uint8_t stop )
{
    for (int i=start; i<=stop; i++)
    {
    lcd.setCursor (i,row); //  
    lcd.write(254);
    } 

}//clearCharacters


 
