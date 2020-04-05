#include "arduinoFFT.h" //include the library

arduinoFFT FFT = arduinoFFT(); //Creating FFT object

int ADCval;
const uint16_t samples = 128;
const double samplingfrequency = 76800 ; //19kHz

double vReal[samples];
double vImag[samples];

uint8_t sampleCount = 0;

void setup() {
  // put your setup code here, to ruwn once:
  
  cli(); // disable interrupts
  
  ADCSRA &= ( bit(ADPS0) | bit(ADPS1)| bit(ADPS2) ); //clear prescalar bits
  ADCSRA = 0;  // clear ADCSRA register
  ADCSRB = 0;  // clear ADCSRB register
  
  // we need the ADC's to be running in the Free running mode 
  // This is a mode in which the ADC continuously converts the input and throws an 
  // interrupt at the end of each conversion
  // 1. Do not waste time waiting for the next sample allowing to execute additional logic
  // 2. Improve accuracy of sampling reducing jitter
  
  ADMUX |= (0 & 0x07);   // set A0 analog input pin
  ADMUX |= (1 << REFS0); // set reference voltage AVcc as the reference voltage
  ADMUX &= ~(1 << ADLAR); // clear for 10 bit resolution
  
// ADCSRA |= (1 << ADPS0); // 2 prescalar instead of 128 -- 608000 Hz
// ADCSRA |= (1 << ADPS1); // 4 prescalar instead of 128 -- 304000 Hz
// ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // 8 prescalar instead of 128 -- 152000 Hz

 
  //We can only bump the bandwidth from 4.50Hz to 38kHz

  
 ADCSRA |= (1 << ADPS2); // 16 prescalar instead of 128 -- 76000 Hz
// ADCSRA |= (1 << ADPS2) | (1 << ADPS0);  // 32 prescalar instead of 128 -- 38000 Hz
// ADCSRA |= (1 << ADPS2) | (1 << ADPS1);  // 64 prescalar instead of 128 -- 19000 Hz
// ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) ;
//  ADMUX = (1 << ADLAR); // left align ADC values for 8 bit which can be collected from ADCH register
//  ADCval = ADCH // collecting value directly from the ADCH it will be written in ISR 
  
  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enables interrupts when measurement is complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  sei(); //enable interrupts
                                                
  Serial.begin(115200);
 
}

void loop() {
    delay(1000);
  // put your main code here, to run repeatedly:
  
  while(1){
    while( ADCSRA & _BV(ADIE)); // wait for the samples to be collected


/*for (int i=0; i<samples;i++){
   // microseconds = micros();

    Serial.print(vReal[i]);
    Serial.print("  ");
    // while(micros() < (microseconds + sampling_period_us)){ //this is just to wait till the time for next reading to be taken
      // in our case this is waste because we want to use arduino at its best
    }
    Serial.println();*/
    FFT.Windowing( vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD );
    FFT.Compute( vReal, vImag, samples, FFT_FORWARD ); // Computes FFT
    FFT.ComplexToMagnitude( vReal,vImag, samples );     // Compute magnitudes
    double peak = FFT.MajorPeak( vReal, samples, samplingfrequency ); 

    //Serial Printing Starts

//    Serial.println(peak,6);
    //Serial.println();
    
    for(uint16_t i=3; i < samples/2; i++){
      float x_coordinate = (i * 1.0 * samplingfrequency) / samples;
      float y_coordinate = vReal[i];
      Serial.print(x_coordinate);
      Serial.print(",");
      //Serial.print(" Hz :");
      Serial.print(y_coordinate);
      Serial.print(",");
      //Serial.print("   ");
    }

    Serial.println();
    
    sampleCount = 0;
    ADCSRA |= (1 << ADIE);  //Interrupt on
    
    }
}

ISR(ADC_vect){
  
  //when new ADC value ready
  if(sampleCount<samples){
    ADCval = ADCL;
    ADCval = (ADCH << 8) + ADCval;
    vReal[sampleCount] = ADCval;  //should look whether we need ADCval variable or not
    vImag[sampleCount] = 0.0;
    sampleCount++;
  }
  else{
  //  ADCSRA &= ~(1 << ADIE);
  ADCSRA &= ~_BV(ADIE); //alternate syntax to the above
  } 
}
