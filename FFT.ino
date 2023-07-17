#include <FastLED_NeoPixel.h>
#define NUM_LEDS 24
#define DATA_PIN 8

#define DCT_SIZE 128
#define NUMBER_OF_SAMPLES DCT_SIZE

FastLED_NeoPixel<NUM_LEDS, DATA_PIN, NEO_GRB> strip;

auto color_purple = strip.Color(255, 0, 255);
auto color_black = strip.Color(0, 0, 0);

uint8_t buffer[2][NUMBER_OF_SAMPLES];
uint8_t dct_coefficients[DCT_SIZE];
// float cos_table[DCT_SIZE][NUMBER_OF_SAMPLES];
volatile uint8_t buffer_current = 1;
volatile uint8_t buffer_next = 0;
volatile uint8_t buffer_index = 0;
volatile bool run_fft = false;


void setup(){
  // Serial.begin(115200);
  Serial.begin(1000000);
  // Serial.begin(9600);
  strip.begin();
  init_cos_table();
  init_adc();
  DDRB |= (1<<7); //LED
  Serial.println("started");
}

void init_cos_table(){
  for(int f=0; f<DCT_SIZE; f++){
    for(int s=0; s<NUMBER_OF_SAMPLES; s++){
      // cos_table[f][s] = cos((f*s * PI) / NUMBER_OF_SAMPLES);
    }
  }
}

float get_cos(int f, int s){
  return cos((f*s * PI) / NUMBER_OF_SAMPLES);
  // return cos_table[f][s];
}

void dct(){
  //discrete cosine transform. O(n^2).
  Serial.println("computing dct");
  int sum;
  for(int f=0; f<DCT_SIZE; f++){
    sum = 0;
    for(int s=0; s<NUMBER_OF_SAMPLES; s++){
      sum += buffer[!buffer_current][s] * get_cos(f,s);
    }
    dct_coefficients[f] = sum / NUMBER_OF_SAMPLES;
  }
}

void print_buffer(){
  Serial.println("buffer:");
  for(int s=0; s<NUMBER_OF_SAMPLES; s++){
    Serial.print(buffer[buffer_current][s]);
    Serial.print(",");
  }
  Serial.println(";");
}

void output_buffer_to_leds(){
  //let's show buffer maximum instead:
  // uint8_t max = 0;
  // for(int i=0; i<256; i++) if(buffer[buffer_current][i] > max) max = buffer[buffer_current][i];
  // int v = map(max, 0, 255, 0 ,NUM_LEDS);
  // for(int i=0; i<v; i++) strip.setPixelColor(i, color_purple);
  // for(int i=v; i<NUM_LEDS; i++) strip.setPixelColor(i, color_black);
  // strip.show();

  int samples_per_led = floor(NUMBER_OF_SAMPLES/NUM_LEDS); //lossing high frequnecy if divides with remainder
  for(int led=0; led<NUM_LEDS; led++){
    int avg = 0;
    for(int s = 0; s<samples_per_led; s++){
      avg += buffer[buffer_current][led*samples_per_led + s];
    }
    avg /= samples_per_led;
    strip.setPixelColor(led,strip.Color(avg, 0, avg));
  }
  strip.show();
}

void print_coefficients(){
  Serial.println("fft coefficients:");
  for(int f=0; f<DCT_SIZE; f++){
    Serial.print(dct_coefficients[f]);
    Serial.print(",");
  }
  Serial.println(";");
}

void output_coefficients_to_leds(){
  // Serial.println("outputing coefficients to leds");
  int samples_per_led = floor(NUMBER_OF_SAMPLES/NUM_LEDS); //lossing high frequnecy if divides with remainder
  for(int led=0; led<NUM_LEDS; led++){
    int avg = 0;
    for(int s = 0; s<samples_per_led; s++){
      avg += dct_coefficients[led*samples_per_led + s];
    }
    avg /= samples_per_led;
    strip.setPixelColor(led,strip.Color(avg, 0, avg));
  }
  strip.show();
}

void loop(){
    if(run_fft){
      adc_disable_interrupt(); //wait to finish transform before accuiring more samples.
      // ADCSRA &= ~(1<<ADATE); //enable auto trigger which is in free running mode by default
      dct();
      print_buffer();
      print_coefficients();
      output_coefficients_to_leds();
      run_fft = false;
      adc_enable_interrupt();
    }
}

inline void buffer_move_to_next(){
  buffer_current = buffer_next;
  buffer_next = (buffer_next+1) % 2;
  // Serial.print("moved to buffer ");
  // Serial.println(buffer_current);
}

ISR(ADC_vect){
  // PORTB ^= (1<<7); used to measure adc rate with oscillosope
  buffer[buffer_current][buffer_index] = ADCH;
  if(++buffer_index == NUMBER_OF_SAMPLES){
    buffer_index = 0;
    run_fft = true;
    buffer_move_to_next();
  }
}

inline void adc_enable_interrupt(){
  // ADCSRA |= (1<< ADIE);
  sei();
}

inline void adc_disable_interrupt(){
  //cli() also disables serial so this has no side effects.
  cli();
  // ADCSRA &= ~(1<< ADIE);
}

void init_adc(){
  PRR0 &= ~(1<<PRADC);

  ADMUX = 0; //this is needed because
  ADCSRA = 0; // arduino probably setup the adc before and thats why it wasn't all 0 at the start.

  ADMUX |= (1<<ADLAR); //left adjust result
  ADMUX |= (0x0 << MUX0); //adc channel 0
  // ADMUX |= (0b01 << REFS0); //AVCC as reference
  ADMUX |= (0b10 << REFS0); //Internal 1.1V reference
  ADCSRA |= (1<<ADEN) | (1<< ADIE);//enable ADC, enable adc interrupt
  ADCSRA |= (1<<ADATE); //enable auto trigger which is in free running mode by default
  ADCSRA |= (0b101<<ADPS0); //32 prescaler (500 kHz clock for ~38kHz sampling rate)
  
  ADCSRA |= (1<<ADSC); //start adc conversion
}