#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <SoftwareServo.h>
#include <math.h>
#include <EEPROM.h>
#include <avr/interrupt.h>

/*
 * This code is in the public domain.
 */

#define RXPIN 2
#define TXPIN 3

#define POLULUPIN 5

#define LIDPIN 6
#define DRAWERPIN 4
#define IRPINA 0

#define EARTH_RADIUS 6378.1f
#define DEG_TO_RAD 0.0174532925f

#define TOFINO_LAT 49.122444f
#define TOFINO_LON 125.900871f

#define PEARTREE_LAT 49.28095f
#define PEARTREE_LON 123.01355f

#define HOME_LAT 49.5f
#define HOME_LON 120.5f

#define GPS_DELAY_MS 60000

/* 5km is pretty generous, but
 better safe than sorry, right? */
#define DIST_THRESHOLD 5.0f

#define ADDR_TRIES 0
#define ADDR_STAGE1 1
#define ADDR_STAGE2 2
#define ADDR_BACKDOOR 3

TinyGPS gps;
SoftwareServo drawerServo;
SoftwareServo lidServo;

/* Set up the SoftwareSerial to talk to the GPS */
SoftwareSerial nss(RXPIN,TXPIN);

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

unsigned long waitUntil;
int attemptsRemaining;

void setup() {
  pinMode(POLULUPIN, OUTPUT);
  Serial.begin(9600);
  Serial.print( "In setup.\n" );
  nss.begin(4800);
  lcd.begin(2,16);
  lcd.clear();
  lcd.setCursor(0,0);

  /* Check how many tries are left */
  attemptsRemaining = EEPROM.read(ADDR_TRIES);
  Serial.print( "Attempts Remaining: " );
  Serial.print( attemptsRemaining, DEC );
  Serial.print("\n");
  if ( attemptsRemaining == 0xFF )
  {
    // First time through
    Serial.print( "First time through, initializing EEPROM\n" );
    initializeEEPROM();
    Serial.print( "Locking lid\n" );
    lockLid();
    shutdown();
  }
  lcd.print( " ...Loading..." );
}

void loop()
{
  waitUntil = millis() + 2000;
  if ( waitForBackdoor() )
  {
    /* If we get here, then the backdoor was triggered */
    unlockLid();
    shutdown();
  }
  else if ( EEPROM.read( ADDR_BACKDOOR ) != 0 )
  {
    // clear the backdoor flag, if it was set from a previous
    // run. this lets us try to unlock the backdoor multiple times
    // in case the first try jammed or something.
    lockLid();
    EEPROM.write( ADDR_BACKDOOR, 0 );
    shutdown();
  }

  if ( attemptsRemaining <= 0 )
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( " No More Tries!" );
    delay(5000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( " Return to" );
    lcd.setCursor(0,1);
    lcd.print( " Russ" );
    delay(5000);
    shutdown();
  }

  // ... Normal Operation begins here
  bool stage1Complete = (EEPROM.read(ADDR_STAGE1) != 0);
  bool stage2Complete = (EEPROM.read(ADDR_STAGE2) != 0);

  waitUntil = millis() + 120000;
  if ( !stage1Complete )
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( " Searching for " );
    lcd.setCursor(0,1);
    lcd.print( " Signal " );
    float stage1Dist = 100.0f;
    if ( distanceTo( TOFINO_LAT, TOFINO_LON, &stage1Dist ) )
    {
      /* If we get here, we received a valid distance measurement */
      if ( stage1Dist < DIST_THRESHOLD )
      {
        EEPROM.write( ADDR_STAGE1, 0xFF );
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( "Stage One" );
        lcd.setCursor(0,1);
        lcd.print( "Complete!");
        unlockDrawer();
      }
      else
      {
        /* Sorry, try again */
        decrAttemptsRemaining();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( "Access Denied!" );
        lcd.setCursor(0,1);
        lcd.print( attemptsRemaining );
        lcd.print( " tries left" );
        delay(7500);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( "Out of Range" );
        lcd.setCursor(0,1);
        lcd.print( stage1Dist );
        lcd.print( " km" );
      }
      delay(7500);
      shutdown();
    }
    else
    {
      /* We never received a good GPS signal */
      failNoSignal();
    }
  }
  else if ( !stage2Complete )
  {
    /* This chunk of code is just different enough from stage 1
     to make it awkward to write in a loop. */
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( " Searching for " );
    lcd.setCursor(0,1);
    lcd.print( " Signal " );
    float stage2Dist = 100.0f;
    if ( distanceTo( PEARTREE_LAT, PEARTREE_LON, &stage2Dist ) )
    {
      if ( stage2Dist < DIST_THRESHOLD )
      {
        EEPROM.write( ADDR_STAGE2, 0xFF );
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( "Stage Two" );
        lcd.setCursor(0,1);
        lcd.print( "Complete!" );
        unlockLid();
      }
      else
      {
        decrAttemptsRemaining();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( "Access Denied!" );
        lcd.setCursor(0,1);
        lcd.print( attemptsRemaining );
        lcd.print( " tries left" );
        delay(7500);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( "Out of Range" );
        lcd.setCursor(0,1);
        lcd.print( stage2Dist );
        lcd.print( " km" );
      }
      delay(7500);
      shutdown();
    }
    else
    {
      failNoSignal();
    }
  }
  else
  {
    // Game Over!
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( " Game" );
    lcd.setCursor(0,1);
    lcd.print( " Over!" );
    delay(10000);
    shutdown();
  }
  // We should never get here, but just in case…
  shutdown();
}

/*
* Tries to calculate the distance to the specified lat+lon, using the GPS sensor.
 * Returns true iff the sensor returned a valid reading.
 * On successful return, result will hold the great circle distance to the specified point.
 */
bool distanceTo( float targetLat, float targetLon, float* result)
{
  int numFixes = 0;
  while ( millis() < waitUntil )
  {
    if (nss.available())
    {
      int c = nss.read();
      if (gps.encode(c))
      {
        unsigned long fix_age;
        float flat,flon;
        gps.f_get_position(&flat, &flon, &fix_age);
        if ( fix_age > 60000 ) continue;
        if ( numFixes < 5 )
        {
          numFixes++;
          continue;
        }
        flat = fabs(flat);
        flon = fabs(flon);

        float dist = gcd( flat * DEG_TO_RAD, flon * DEG_TO_RAD, targetLat * DEG_TO_RAD, targetLon * DEG_TO_RAD );
        *result = dist;
        return true;
      }
    }
  }
  return false;
}

/*
* This is a super-kludge to turn off all pin-change interrupts.
 * It will disable all serial functionality. Pretty much the
 * only way to restore the Arduino to a usable state after calling this
 * function is to cut the power and reboot. Which is a little tricky to do
 * if you’re running on USB _and_ a battery pack.
 *
 * I use this as a hack to get SoftwareSerial and SoftwareServo to play nice
 * with each other, but I think that the latest version of SoftwareSerial has
 * a much cleaner fix for this.
 */
void disablePCI()
{
  PCICR = 0;
  PCMSK2 = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
}

// Blocks until IR code received
// or expiry is reached (returns true iff code received)
// reads the waitUntil variable to determine how long
// to wait for the backdoor.
bool waitForBackdoor()
{
  uint8_t pulseCount = 0;
  unsigned long pulseStart;
  bool pulseActive;

  while ( millis() < waitUntil )
  {
    int irval = analogRead(IRPINA);
    if (irval < 0x0F)
    {
      // pulse started
      if ( !pulseActive )
      {
        pulseActive = true;
        pulseStart = millis();
      }
    }
    else
    {
      if ( pulseActive )
      {
        pulseActive = false;
        unsigned long duration = millis() - pulseStart;
        if ( (duration > 5) && (duration < 15) )
        {
          pulseCount++;
        }
      }
    }
  }
  /* 4 or more pulses within the duration specified by waitUntil will unlock the backdoor. */
  if (pulseCount >= 4)
  {
    EEPROM.write( ADDR_BACKDOOR, 0xFF );
    return true;
  }
  return false;
}

/*
* This function only works when the Arduino is running off a battery pack.
 * When running on USB, maybe stick an infinite loop at the end?
 */
void shutdown()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print( " Shutting Down.");
  delay(2000);
  digitalWrite( POLULUPIN, HIGH );
}

void unlockDrawer()
{
  // disablePCI _MUST_ be called prior to any servo operations!
  disablePCI();
  drawerServo.attach(DRAWERPIN);
  // this "stepping" is only necessary due to limitations of the Software Servo library,
  // which is an obsolete version of the Servo library that I used before
  // I diagnosed the problem with the pin change interrupts and never
  // had time to replace.
  for ( int i = 150; i >= 5; i-- )
  {
    drawerServo.write(i);
    SoftwareServo::refresh();
    delay(15);
  }
  drawerServo.detach();
}

void rotateLidServo(bool lock, int steps)
{
  disablePCI();
  // In my setup, the lid servo is a continuous rotation servo and so writing an angular
  // value to it makes it go a little crazy.
  lidServo.attach(LIDPIN);
  lidServo.write((lock) ? 0 : 180);
  for ( int i = 0; i < steps; i++ )
  {
    SoftwareServo::refresh();
    delayMicroseconds(15000);
  }
  lidServo.detach();
}

void unlockLid()
{
  // give it a bit more juice on the "unlock" just in case
  rotateLidServo(false, 10);
}

void lockLid()
{
  rotateLidServo(true, 7);
}

/*
* Great Circle Distance calculation. If anyone knows a fixed-point version
 * of this function I’d love to see it.
 */
float gcd(float lat_a, float lon_a, float lat_b, float lon_b)
{
  float d = acos(sin(lat_a)*sin(lat_b)+cos(lat_a)*cos(lat_b)*cos(lon_a-lon_b));
  return fabs(EARTH_RADIUS * d);
}

void decrAttemptsRemaining()
{
  attemptsRemaining--;
  // save the new state.
  EEPROM.write(ADDR_TRIES, attemptsRemaining);
}

void failNoSignal()
{
  decrAttemptsRemaining();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print( "Access Denied!" );
  lcd.setCursor(0,1);
  lcd.print( attemptsRemaining );
  lcd.print( " tries left" );
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print( "No Signal!" );
  delay(5000);
  shutdown();
}

/* This function resets the box to its original state. */
void initializeEEPROM()
{
  // 101 Tries remaining
  EEPROM.write(ADDR_TRIES, 101);
  EEPROM.write(ADDR_STAGE1, 0);
  EEPROM.write(ADDR_STAGE2, 0);
  EEPROM.write(ADDR_BACKDOOR, 0);
}

