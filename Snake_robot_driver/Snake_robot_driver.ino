#include "Arduino.h"

//Arduino PWM Speed Control：
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

int i=0;
int value=0;
bool condition=true;


const float SensorOffset = 46.0;  //pressure sensor

/* Not used in polled mode. Stub function necessary for library compilation */
void ads_data_callback(float * sample, uint8_t sample_type)
{
  
}

void setup() {

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
    
  Serial.begin(115200);
  
}

void loop() {

    
    // read the input on analog pin 0:
    float sensorValue = (analogRead(A0)-SensorOffset); //Do maths for calibration
    digitalWrite(M1,HIGH);
    digitalWrite(M2,HIGH);
    
    if (condition== true)
    {
      Serial.println("prendido");  // Stretch data

      if (sensorValue <=550)
      {
      value=255;
      }
      else
      {
      value=0;
        }
      
      condition= false;
      }
    else
    {
      Serial.println("apagado");  // Stretch data
      value=0;
      condition= true;
      }
      
    analogWrite(E1, value);   //PWM Speed Control
    analogWrite(E2, value);   //PWM Speed Control
    
    delay(1300);
    i++;
    if (i==5)
    {
    i=0;
    analogWrite(E2, 255);   //PWM Speed Control
    }

    Serial.println(sensorValue);  // Stretch data

  
  
  // Check for received hot keys on the com port
  if(Serial.available())
  {
    parse_com_port();
  }

  // Delay 5ms to keep 100 Hz sampling rate between bend and stretch, Do not increase rate past 500 Hz.
  delay(5);
}

/* Function parses received characters from the COM port for commands */
void parse_com_port(void)
{
  char key = Serial.read();

  switch(key)
  {
    case '0':
      // Take first calibration point at zero degrees
      //ads_calibrate(ADS_CALIBRATE_FIRST, 0);
      break;
    case '9':
      // Take second calibration point at ninety degrees
      //ads_calibrate(ADS_CALIBRATE_SECOND, 90);
      break;
    case 'c':
      // Restore factory calibration coefficients
      //ads_calibrate(ADS_CALIBRATE_CLEAR, 0);
      break;
    case 'r':
      // Start sampling in interrupt mode
      //ads_run(true);
      break;
    case 's':
      // Place ADS in suspend mode
      //ads_run(false);
      break;
    case 'f':
      // Set ADS sample rate to 200 Hz (interrupt mode)
      //ads_set_sample_rate(ADS_200_HZ);
      break;
    case 'u':
      // Set ADS sample to rate to 10 Hz (interrupt mode)
      //ads_set_sample_rate(ADS_10_HZ);
      break;
    case 'n':
      // Set ADS sample rate to 100 Hz (interrupt mode)
      //ads_set_sample_rate(ADS_100_HZ);
      break;
    case 'b':
      // Calibrate the zero millimeter linear displacement
      //ads_calibrate(ADS_CALIBRATE_STRETCH_ZERO, 0);
      break;
    case 'e':
      // Calibrate the 30 millimeter linear displacement (stretch), Make certain the sensor is at 0 degrees angular displacement (flat)
      //ads_calibrate(ADS_CALIBRATE_STRETCH_SECOND, 30);
      break;
    default:
      break;
  }
}

/* 
 *  Second order Infinite impulse response low pass filter. Sample freqency 100 Hz.
 *  Cutoff freqency 20 Hz. 
 */
void signal_filter(float * sample)
{
    static float filter_samples[2][6];

    for(uint8_t i=0; i<2; i++)
    {
      filter_samples[i][5] = filter_samples[i][4];
      filter_samples[i][4] = filter_samples[i][3];
      filter_samples[i][3] = (float)sample[i];
      filter_samples[i][2] = filter_samples[i][1];
      filter_samples[i][1] = filter_samples[i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
        0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

      sample[i] = filter_samples[i][0];
    }
}

/* 
 *  If the current sample is less that 0.5 degrees different from the previous sample
 *  the function returns the previous sample. Removes jitter from the signal. 
 */
void deadzone_filter(float * sample)
{
  static float prev_sample[2];
  float dead_zone = 0.75f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}
