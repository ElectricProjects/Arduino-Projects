/********************************************
 * Zen Radio - A modern version of an old Zenith radio with an MP3 player inside
 * For instant tunes on the go with no ads like on real radio.
 * Powered by 6 AA batteries.
 * 60 songs on 4 channels (new wave, rock, reggae, ambient)
 * + 1 'song' for radio static.
 * Using 2 pots.  1 pot for station selection and 1 for on/off & volume.
 * Also added blue LED behind on/volume knob as an on/off indicator.
 * Based on Sparkfun example with the FAT library.
 * Used Audacity & iTunes to convert MP3 files to 192kbps
 * PWM: 3, 5, 6, 9, 10, and 11
 * http://code.google.com/p/sdfatlib/
 * You MUST change uint8_t const SS_PIN = 10; 
 * on Sd2PinMap.h of the SDfatlib to pin 9;
 * Zen Radio Version 3.8 4/4/12 http://electricprojects.wordpress.com
 ********************************************/
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
Sd2Card card;
SdVolume volume;
SdFile root;
SdFile track;
char trackName[] = "track001.mp3";
int trackNumber = 1;
char errorMsg[100]; //This is a generic array used for sprintf of error messages
#define TRUE  0
#define FALSE  1
#define MP3_XCS 6
#define MP3_XDCS 7
#define MP3_DREQ 2
#define MP3_RESET 8
//Remember you have to edit the Sd2PinMap.h of the sdfatlib library to correct control the SD card.
#define SCI_MODE 0x00
#define SCI_STATUS 0x01
#define SCI_BASS 0x02
#define SCI_CLOCKF 0x03
#define SCI_DECODE_TIME 0x04
#define SCI_AUDATA 0x05
#define SCI_WRAM 0x06
#define SCI_WRAMADDR 0x07
#define SCI_HDAT0 0x08
#define SCI_HDAT1 0x09
#define SCI_AIADDR 0x0A
#define SCI_VOL 0x0B
#define SCI_AICTRL0 0x0C
#define SCI_AICTRL1 0x0D
#define SCI_AICTRL2 0x0E
#define SCI_AICTRL3 0x0F
int volRead=0;
int volPin = 0;
int vol=25;
int Left=30;
int Right=0;
int prevVolume=30;
int randNumber=0;
int stationPin=1;
int stationRead=0;
int station=1;
int stationValue=1;
int prevStation=1;
byte trackCounter=0;

int BLUEPin = 11;   // BLUE pin of the LED to PWM pin 6

volatile int state = 0;
int x=0;
void setup() {
  pinMode(BLUEPin, OUTPUT);
  pinMode(MP3_DREQ, INPUT);
  pinMode(MP3_XCS, OUTPUT);
  pinMode(MP3_XDCS, OUTPUT);
  pinMode(MP3_RESET, OUTPUT);
  digitalWrite(MP3_XCS, HIGH); //Deselect Control
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
  digitalWrite(MP3_RESET, LOW); //Put VS1053 into hardware reset
  Serial.begin(57600); //Use serial for debugging
  Serial.println("MP3 Testing");
  pinMode(10, OUTPUT);       //Pin 10 must be set as an output for the SD communication to work.
  if (!card.init(SPI_FULL_SPEED))  Serial.println("Error: Card init"); //Initialize the SD card and configure the I/O pins.
  if (!volume.init(&card)) Serial.println("Error: Volume ini"); //Initialize a volume on the SD card.
  if (!root.openRoot(&volume)) Serial.println("Error: Opening root"); //Open the root directory in the volume.
  SPI.setClockDivider(SPI_CLOCK_DIV16); //Set SPI bus speed to 1MHz (16MHz / 16 = 1MHz)
  SPI.transfer(0xFF); //Throw a dummy byte at the bus
  delay(10);
  digitalWrite(MP3_RESET, HIGH); //Bring up VS1053
  Mp3SetVolume(35, 35); //Set initial volume (0 is loud 50 is quiet)
  int MP3Mode = Mp3ReadRegister(SCI_MODE);
  int MP3Status = Mp3ReadRegister(SCI_STATUS);
  int MP3Clock = Mp3ReadRegister(SCI_CLOCKF);

  int vsVersion = (MP3Status >> 4) & 0x000F; //Mask out only the four version bits

  Mp3WriteRegister(SCI_CLOCKF, 0x60, 0x00); //Set multiplier to 3.0x
  SPI.setClockDivider(SPI_CLOCK_DIV4); //Set SPI bus speed to 4MHz (16MHz / 4 = 4MHz)
  MP3Clock = Mp3ReadRegister(SCI_CLOCKF);

  randomSeed(analogRead(5));  
  analogWrite(BLUEPin, 255);

}
void loop(){

  if (x==0) // First time through  - read station and select random song
  {
    stationRead = analogRead(stationPin);
    stationChange();
    prevStation=station;
    randNumber=random(1, 14);
    // randNumber=11;
    trackNumber=stationValue+randNumber;
    trackCounter=stationValue+randNumber;
    x++;
    Serial.print("First time through.....");
  }
  if(station==99)
  {
    stationValue=99;
    trackNumber=99;
  }
  Serial.print("Track number is ");
  Serial.println(trackNumber);
  sprintf(trackName, "track%03d.mp3", trackNumber);
  playMP3(trackName);
  trackCounter++;
  trackNumber++;
  if (trackCounter>15)
  {
    trackCounter=0; 
    trackNumber=stationValue;
  }
   Serial.print("Track number is ");
  Serial.println(trackNumber);
 Serial.print("---------------------"); 
}
void playMP3(char* fileName) {
  if (!track.open(&root, fileName, O_READ)) { //Open the file in read mode.
    sprintf(errorMsg, "Failed to open %s", fileName);
    Serial.println(errorMsg);
    return;
  }

  uint8_t mp3DataBuffer[32]; //Buffer of 32 bytes. VS1053 can take 32 bytes at a go.
  int need_data = TRUE;
  long replenish_time = millis();
  while(1) {
    while(!digitalRead(MP3_DREQ)) {
      //DREQ is low while the receive buffer is full
      //You can do something else here, Maybe set the volume or test how much we can delay before we hear glitches
      for (int i=0; i < 5; i++)
      {
        volRead = volRead+analogRead(volPin);    
      }
      volRead=volRead/5;  // get average to stop fluctuation
      //volRead = analogRead(volPin);
      if (volRead != prevVolume)
      {
        volChange();  
        vol=Left;
        Mp3SetVolume(Left, Left);
      }
      Serial.print("Volume = ");
      Serial.print(volRead);
      Serial.print(" Actual Volume = ");
      Serial.println(vol);
      for (int i=0; i < 5; i++)
      {
        stationRead = stationRead+analogRead(stationPin);    
      }
      stationRead=stationRead/5;  // get average to stop fluctuation
      stationRead = analogRead(stationPin);
      Serial.print("X = ");
      Serial.print(x);
      Serial.print(" Prev Station = ");
      Serial.print(prevStation);
      Serial.print(" Current Station = ");
      Serial.println(station);
      Serial.print("Track # = ");
      Serial.print(trackNumber);
      Serial.print(" Track name = ");
      Serial.println(trackName);
      Serial.print("Random # = ");
      Serial.println(randNumber);
      if(need_data == TRUE) {
        if(!track.read(mp3DataBuffer, sizeof(mp3DataBuffer))) { //Try reading 32 new bytes of the song
          //Oh no! There is no data left to read!
          //Time to exit
          break;
        }
        need_data = FALSE;
      }


      if(stationRead != prevStation)
      {
        stationChange();
        if (prevStation!=station)
        {
          // track.close();// stop track
          Serial.println("In 2nd Change Station");
          // Mp3WriteRegister(SCI_MODE, 0x48, SM_RESET);
          //  track.close();

          randNumber=random(1, 14);
          if(station==99)
          {
            trackNumber=99;
            stationValue=99;
            randNumber=0;
          }

          stopTrack();
          prevStation=station;
          trackNumber=stationValue+randNumber;
          trackCounter=stationValue+randNumber;
          sprintf(trackName, "track%03d.mp3", trackNumber);
          playMP3(trackName);
          
         
        }
      }

      //Test to see just how much we can do before the audio starts to glitch
      long start_time = millis();
      //delay(120); //Do NOTHING - barely audible glitches
      delay(50); //Do NOTHING - sounds fine
      replenish_time = millis();
    }
    if(need_data == TRUE){ //This is here in case we haven't had any free time to load new data
      if(!track.read(mp3DataBuffer, sizeof(mp3DataBuffer))) { //Go out to SD card and try reading 32 new bytes of the song
        //Oh no! There is no data left to read!
        //Time to exit
        break;
      }
      need_data = FALSE;
    }
    //Once DREQ is released (high) we now feed 32 bytes of data to the VS1053 from our SD read buffer
    digitalWrite(MP3_XDCS, LOW); //Select Data
    for(int y = 0 ; y < sizeof(mp3DataBuffer) ; y++) {
      SPI.transfer(mp3DataBuffer[y]); // Send SPI byte
    }
    digitalWrite(MP3_XDCS, HIGH); //Deselect Data
    need_data = TRUE; //We've just dumped 32 bytes into VS1053 so our SD read buffer is empty. Set flag so we go get more data
  }
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating transfer is complete
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
  track.close(); //Close out this track
  sprintf(errorMsg, "Track %s done!", fileName);
  Serial.println(errorMsg);
}
void Mp3WriteRegister(unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte){
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(MP3_XCS, LOW); //Select control
  SPI.transfer(0x02); //Write instruction
  SPI.transfer(addressbyte);
  SPI.transfer(highbyte);
  SPI.transfer(lowbyte);
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  digitalWrite(MP3_XCS, HIGH); //Deselect Control
}
unsigned int Mp3ReadRegister (unsigned char addressbyte){
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(MP3_XCS, LOW); //Select control
  SPI.transfer(0x03);  //Read instruction
  SPI.transfer(addressbyte);
  char response1 = SPI.transfer(0xFF); //Read the first byte
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  char response2 = SPI.transfer(0xFF); //Read the second byte
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  digitalWrite(MP3_XCS, HIGH); //Deselect Control
  int resultvalue = response1 << 8;
  resultvalue |= response2;
  return resultvalue;
}
void Mp3SetVolume(unsigned char leftchannel, unsigned char rightchannel)
{
  Mp3WriteRegister(SCI_VOL, leftchannel, rightchannel);
}
void volChange()
{
  if (volRead<=100)
  {
    Left=5;
    prevVolume=Left;
  }
  else if (volRead<=200)
  {
    Left=10;
    prevVolume=Left;
  }
  else if (volRead<=300)
  {
    Left=15;
    prevVolume=Left;
  }
  else if (volRead<=400)
  {
    Left=20;
    prevVolume=Left;
  }
  else if (volRead<=500)
  {
    Left=25;
    prevVolume=Left;
  }
  else if (volRead<=600)
  {
    Left=30;
    prevVolume=Left;
  }
  else if (volRead<=700)
  {
    Left=35;
    prevVolume=Left;
  }
  else if (volRead<=800)
  {
    Left=40;
    prevVolume=Left;
  }
  else if (volRead<=900)
  {
    Left=45;
    prevVolume=Left;
  }
  else if (volRead<=970)
  {
    Left=50;
    prevVolume=Left;
  }
  prevVolume=Left;
}
void stationChange()
{
  if (stationRead <=250)
  {
    station=1;
    stationValue=1;
  }
  else if (stationRead <=350)
  {
    station=99;
    stationValue=99;
  }
  else if (stationRead <=500)
  {
    station=2;
    stationValue=16;
  }
  else if (stationRead <=650)
  {
    station=99;
    stationValue=99;
  }
  else if (stationRead <=750)
  {
    station=3;
    stationValue=31;
  }
  else if (stationRead <=800)
  {
    station=99;
    stationValue=99;
  }
  else
  {
    station=4;
    stationValue=46;
  }
}


void stopTrack()
{
   while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating transfer is complete
  //digitalWrite(MP3_XDCS, HIGH); //Deselect Data
 // Mp3WriteRegister(SCI_MODE, 0x48, MP3_RESET);
  //detachInterrupt(0);
  track.close(); //Close out this track
  //sprintf(errorMsg, "Track %s done!", fileName);
  //Serial.println(errorMsg);
  // if(playing == FALSE)
  // return;
  // track.seekEnd((-128 + offset));
  Serial.println("In Stop Track");
 // detachInterrupt(0);
  //tell MP3 chip to do a soft reset. Fixes garbles at end, and clears its buffer.
//Mp3WriteRegister(SCI_MODE, 0x48, MP3_RESET);
  //digitalWrite(MP3_XDCS, HIGH);
  //track.close(); //Close out this track

}



