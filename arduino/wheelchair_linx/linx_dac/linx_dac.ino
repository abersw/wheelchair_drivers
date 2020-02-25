#include <Wire.h>
#include <Adafruit_MCP4725.h>
#define FB_voltsIn 20
#define RL_voltsIn 20

Adafruit_MCP4725 FB_dac; // constructor
Adafruit_MCP4725 RL_dac;


void setup(void) {
  Serial.begin(9600);
  FB_dac.begin(0x60); // The I2C Address: Run the I2C Scanner if you're not sure  
  RL_dac.begin(0x61);
  
}

/*void setForwardBackwardDAC() {
  dac_expected_output = (5.0/4096.0) * dac_value;
  dac.setVoltage(dac_value, false);
  delay(250);
  adcValueRead = analogRead(voltsIn);
  voltageRead = (adcValueRead * 5.0 )/ 1024.0;
}

void setLeftRightDAC() {
  
}*/

void loop(void) {
  
    uint32_t FB_dac_value = 2250;
    int FB_adcValueRead = 0;
    float FB_voltageRead = 0;
    float FB_dac_expected_output;
    
    FB_dac_expected_output = (5.0/4096.0) * FB_dac_value;
    FB_dac.setVoltage(FB_dac_value, false);
    delay(250);
    FB_adcValueRead = analogRead(FB_voltsIn);
    FB_voltageRead = (FB_adcValueRead * 5.0 )/ 1024.0;


    uint32_t RL_dac_value = 2250;
    int RL_adcValueRead = 0;
    float RL_voltageRead = 0;
    float RL_dac_expected_output;
    
    RL_dac_expected_output = (5.0/4096.0) * FB_dac_value;
    RL_dac.setVoltage(RL_dac_value, false);
    delay(250);
    RL_adcValueRead = analogRead(RL_voltsIn);
    RL_voltageRead = (RL_adcValueRead * 5.0 )/ 1024.0;
    
   /*
    for (dac_value = 0; dac_value < 4096; dac_value = dac_value + 15)
    {
      dac_expected_output = (5.0/4096.0) * dac_value;
      dac.setVoltage(dac_value, false);
      delay(250);
      adcValueRead = analogRead(voltsIn);
      voltageRead = (adcValueRead * 5.0 )/ 1024.0;
      
      Serial.print("DAC Value: ");
      Serial.print(dac_value);
      
      Serial.print("\tExpected Voltage: ");
      Serial.print(dac_expected_output,3);
      
      Serial.print("\tArduino ADC Value: ");
      Serial.print(adcValueRead);
      
      Serial.print("\tArduino Voltage: ");      
      Serial.println(voltageRead,3);      
    }
    */
}
