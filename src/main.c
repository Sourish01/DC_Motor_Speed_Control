#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#define F_CPU 8000000UL  // 8 MHz internal clock
#define BAUD_RATE 9600
#define TX_PIN PB3       // Software UART TX

#define ENC_A PB2  // Encoder A (INT0)
#define EN_A PB0   // Motor enable
#define IN_1 PB1   // Motor PWM (OC0B)

// PI Controller Variables
volatile uint32_t pulseCount = 0;
volatile float error = 0, integral = 0, duty = 128.0;
//const float referenceRPM = 3000.0;
float referenceRPM = 0.0;
uint16_t adcVal = 0;
const float ADC_min = 600.0;
const float ADC_max = 1023.0;
const float RPM_min = 1000.0;
const float RPM_max = 3000.0;
const float Kp = 0.07, Ki = 0.0375;
const int pulsesPerRevolution = 2048;

// Custom millis() implementation
volatile unsigned long millisCounter = 0;
unsigned long prevTime = 0;
float rpm = 0;
//Edit
float prev_rpm = 0;
float rpm_avg = 0;
//

// ISR(ADC_vect){
//     adcVal = ADC;
//     referenceRPM = (adcVal-ADC_min)*(RPM_max-RPM_min)/(ADC_max-ADC_min)+1000.0;
//     if(referenceRPM<1000){
//         referenceRPM = 1000.0;
//     }
// }
// Encoder Interrupt Service Routine (ISR)
void setupMillis() {
    TCCR1 = (1 << CTC1) | (1 << CS12) | (1 << CS10); // CTC mode with OCR1A as TOP, Prescaler 64
    OCR1A = 124;  // (8 MHz / 64) / 125 = 1000 Hz (1 ms tick)
    //OCR1A = 62;
    //OCR1A = 249; //trying to get 2x delay
    TIMSK |= (1 << OCIE1A); // Enable Timer1 compare interrupt
}

// Configure External Interrupt for Encoder
void setupInterrupt() {
    GIMSK |= (1 << INT0); // Enable INT0 interrupt
    MCUCR |= (1 << ISC00) | (1 << ISC01); // Rising edge trigger
    sei(); // Enable global interrupts
}

ISR(INT0_vect) {
    pulseCount++;
}

// Timer1 ISR (fires every 1ms to simulate millis())
ISR(TIMER1_COMPA_vect) {
    millisCounter++;
    PORTB ^= (1<<PB4);
}

// Returns the current time in milliseconds
unsigned long millis() {
    uint8_t oldSREG = SREG;
    cli(); // Disable interrupts to read atomic value
    unsigned long currentMillis = millisCounter;
    SREG = oldSREG; // Restore interrupt state
    return currentMillis;
}

// Configure Timer0 for PWM on PB1 (OC0B)
void setupPWM() {
    TCCR0A = (1 << COM0B1) | (1 << WGM00) | (1 << WGM01); // Fast PWM, non-inverting on OC0B
    TCCR0B = (1 << CS01); // Prescaler 8
    OCR0B = (uint8_t)duty; // Set initial duty cycle
}

// // Configure Timer1 to generate 1ms ticks (simulate millis()) //Faced problems as interval was less than 1 s
// void setupMillis() {
//     TCCR1 = (1 << CTC1) | (1 << CS12) | (1 << CS10); // CTC mode, Prescaler 64
//     OCR1C = 124;  // (8 MHz / 64) / 125 = 1000 Hz (1 ms)
//     TIMSK |= (1 << OCIE1A); // Enable Timer1 compare interrupt
// }
// Configure GPIOs
void setupGPIO() {
    DDRB |= (1 << EN_A) | (1 << IN_1) | (1 << TX_PIN)|(1 << PB4); // Set motor control and TX pins as output
    PORTB |= (1 << EN_A); // Enable motor driver
    PORTB |= (1 << PB4); //Check millis function toggling
}

// void ADC_init() {
//     ADMUX = (0 << REFS1) | (0 << REFS0) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0); // Use Vcc (5V) as reference, select ADC0 (PB5)
    
//     ADCSRA = (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) |  // Enable ADC, Prescaler = 128
//              (1 << ADIE)  | (1 << ADATE) | (1 << ADSC); // Enable interrupt, Auto Trigger, Start Conversion
    
//     ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0); // Select Free-Running Mode (000)
// }
void ADC_init(){
    ADMUX = (0 << REFS1) | (0 << REFS0) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0); // Use Vcc (5V) as reference, select ADC0 (PB5)

    ADCSRA = (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t readADC(){
    ADCSRA |= (1<<ADSC);
    while(ADCSRA&(1<<ADSC));
    return ADC;
}

// Software UART: Send one character
void softUART_sendChar(char c) {
    uint8_t i;
    cli();
    PORTB &= ~(1 << TX_PIN); // Start bit
    _delay_us(1000000 / BAUD_RATE);
    
    for (i = 0; i < 8; i++) {
        if (c & (1 << i)) PORTB |= (1 << TX_PIN);
        else PORTB &= ~(1 << TX_PIN);
        _delay_us(1000000 / BAUD_RATE);
    }
    
    PORTB |= (1 << TX_PIN); // Stop bit
    _delay_us(1000000 / BAUD_RATE);
    sei();
}

// Software UART: Send a string
void softUART_sendString(const char* str) {
    while (*str) {
        softUART_sendChar(*str++);
    }
}

// Convert integer to string and send over UART
void softUART_sendInt(int num) {
    char buffer[10];
    itoa(num, buffer, 10);
    softUART_sendString(buffer);
}

int main() {
    cli(); // Disable interrupts during setup

    ADC_init();
    setupGPIO();
    setupPWM();
    setupMillis();
    setupInterrupt();

    sei(); // Enable global interrupts

    while (1) {
        unsigned long currentTime = millis();
        if (currentTime - prevTime >= 1000) { // Calculate RPM every second
            cli();
            //rpm = (pulseCount / (float)pulsesPerRevolution) * 60.0;
            rpm = ((pulseCount*60.0)/((float)pulsesPerRevolution))*2;
            //Edit
            rpm_avg = (rpm+prev_rpm)/2;
            prev_rpm = rpm;
            //
            pulseCount = 0;
            sei();
            adcVal = readADC();
            referenceRPM = (adcVal-ADC_min)*(RPM_max-RPM_min)/(ADC_max-ADC_min)+1000.0;
            if(referenceRPM<1000){
                referenceRPM = 1000.0;
            }

            // PI Controller
            //error = referenceRPM - rpm;
            //Edit
            error = referenceRPM - rpm_avg;
            //
            duty = Kp * error + Ki * integral;
            integral += error;
            if (integral > 5000) integral = 5000;
            if (integral < -5000) integral = -5000;
            if (duty < 0) duty = 0;
            if (duty > 255) duty = 255;

            OCR0B = (uint8_t)duty; // Update PWM duty cycle
            
            softUART_sendInt((int)referenceRPM);
            softUART_sendChar(',');
            //Edit
            softUART_sendInt((int)rpm_avg);
            //softUART_sendInt((int)rpm);
            softUART_sendChar('\n');


            prevTime = currentTime;
        }
    }
}
