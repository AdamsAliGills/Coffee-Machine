#include <LiquidCrystal_I2C.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Servo.h>


LiquidCrystal_I2C lcd(0x27,20,4);

// Globals
Servo servo1;
volatile int state = 1;
int *i=0;
static unsigned long last_interrupt_time = 0;
volatile uint16_t thing = 0;
int *x;
int price;
int payment;
int diff;//reset to zero
int fiveSec;
int *degree;


enum states {
  beginning=1,
  clear1=2,
  flavorSelction=3,
  clear2=4,
  Dipread1=5,
  clear3=6,
  displaydip1=7,
  paying=8,
  clear4=9,
  checkPayment=10,
  updatePrice=11,
  backTopaying=12,
  serveCoffe=13,
};

class coffMachine{

public:
  // moves servo to the correct degree
  void servoMover(){
    if (degree == 45) {
      servo1.write(45);
    } 
      else if (degree == 90) {
      servo1.write(90);
      } 
      else if (degree == 135) {
      servo1.write(135);
      }
       else if(degree == 180){
       servo1.write(180);
      }
  }

  //dip switch value s5 and s6 is read and they return the
  // payment amount
  int pay(){
    int s6 = (PINB & (1 << PINB4)) >> PINB4;
    int  s5 = (PINB & (1 << PINB5)) >> PINB5;

    if (s5 == 0 && s6 == 0) {
       return 1;
      } 
    else if (s5 == 0 && s6 == 1) {
        return 5;
      } 
    else if (s5 == 1 && s6 == 0) {
        return 10;
      }
    else {
        return 20;
      }
   }

  //result of the payment is displayed
  void paycheck(){
    int payment = pay();
    diff = price - payment;
    if (diff > 0) {
        lcd.setCursor(0,0);
        lcd.print(payment);
        lcd.setCursor(2,0);
        lcd.print("DKK enter");
        lcd.setCursor(12,0);
        lcd.print(diff);
        state++;
      } 


    if(diff==0){
      lcd.setCursor(0,0);
      lcd.print("served");
      state=13;
    }

    if(diff<0){
      lcd.setCursor(0,0);
      lcd.print("served");
      lcd.setCursor(0,1);
      lcd.print("refund");
      lcd.setCursor(7,1);
      lcd.print(abs(diff));
      lcd.setCursor(9,1);
      lcd.print("DKK");
      state=13;
    }
   }

  // value of the Dip switch is being read to set price and find type of coffee
  int readDip(){
    int s8 = (PINB & (1 << PINB2)) >> PINB2;
    int s7 = (PINB & (1 << PINB3)) >> PINB3;

      if (s7 == 0 && s8 == 0) {
        price = 16;
        return 0;
      } 
      else if (s7 == 0 && s8 == 1) {
        price = 19;
        return 1;
      } 
      else if (s7 == 1 && s8 == 0) {
        price = 25;
        return 2;
      }
       else {
        price = 50;
        return 3;
      }
  }
  //the chosen coffee is displayed and servo degree is set
  void displayDipval(){
    if (x==0) {
        lcd.setCursor(0,0);
        lcd.print("Regular 16 00=1");
        lcd.setCursor(0,1);
        lcd.print("01=5 10=10 11=20");
        degree=45;
        state++;
        
      } 
      else if (x==1) {
        lcd.setCursor(0,0);
        lcd.print("Espresso 19 00=1");
        lcd.setCursor(0,1);
        lcd.print("01=5 10=10 11=20");
        degree=90;
        state++;
      } 
      else if (x==2) {
        lcd.setCursor(0,0);
        lcd.print("cappocin 25 00=1");
        lcd.setCursor(0,1);
        lcd.print("01=5 10=10 11=20");
        degree=135;
        state++;
      }
       else if (x==3) {
        lcd.setCursor(0,0);
        lcd.print("latte 50 00=1");
        lcd.setCursor(0,1);
        lcd.print("01=5 10=10 11=20");
        degree=180;
        state++;
      }
  }

  // displays the wanted flavor by sliding the potentiometer
  void flavors(){
    ADCSRA|=1<< ADSC;
    int potval =map(thing,0,56000,0,3);// Map the potentiometer results onto4values.

    if(potval==0){
    lcd.setCursor(0,0);
    lcd.print("Select Flavor");
    lcd.setCursor(0,1);
    lcd.print("mild");
    }

    else if(potval==1){
      while(i == 0){
    lcd.clear();
     i++;
    }
    lcd.setCursor(0,0);
    lcd.print("Select Flavor");
    lcd.setCursor(0,1);
    lcd.print("bold");
    
    }
    
    else if(potval==2){
   
    lcd.setCursor(0,0);
    lcd.print("Select Flavor");
    lcd.setCursor(0,1);
    lcd.print("strong");
    i=0;
    }

    else if(potval==3){
    while(i == 0){
    lcd.clear();
     i++;
    }
    lcd.setCursor(0,0);
    lcd.print("Select Flavor");
    lcd.setCursor(0,1);
    lcd.print("full");
    
   }
  }
  // main flow of program
  void coffee(){
    switch (state){

    case beginning:

    servo1.write(-180);
    lcd.setCursor(0,0);
    lcd.print("Coffee Machine");
    lcd.setCursor(0,1);
    lcd.print("Press Enter");
   
    break;

    case clear1:
    lcd.clear();
    state++;
    break;

    case flavorSelction:
     flavors();
    break;

    case clear2:
    lcd.clear();
    state++;
    break;

    case Dipread1:
     lcd.setCursor(0,0);
     lcd.print("SelectCoffee 00=");
     lcd.setCursor(0,1);
     lcd.print("r 01=e 10=c 11=l");
     x = readDip();
    break;

    case clear3:
    lcd.clear();
    state++;
    break;

    case displaydip1:
    displayDipval();
    break;

    case paying:
    // empty state for payment
    break;
  
    case clear4:
    lcd.clear();
    state++;
    break;

    case checkPayment:
    paycheck();
    break;

    case updatePrice:
    price = diff;
    state++;
    break;

    case backTopaying:
    state = paying;
    break;

    case serveCoffe:
    servoMover();
    PORTD |= (1<<PORTD4);
    if(fiveSec>=155){
      PORTD &= ~(1<<PORTD4);
      int diff=0;//reset to zero
      fiveSec=0;
      lcd.clear();
      state=beginning;
    }
   }
  }
};

// update the state by pressing the button
ISR(INT0_vect){
   if (millis() - last_interrupt_time > 200) 
  {
  state++;
}
last_interrupt_time = millis() ;
}
// read potentiometer values
ISR(ADC_vect){
if(state==3){
  thing = ADC;
 }
}

// calculates 5 second time
ISR(TIMER2_OVF_vect){
  if(state==13){
    fiveSec= fiveSec + 1;
  }
}

int main (void){
  // LCD Is initialized 
  init();
  lcd.init();
  lcd.clear();
  lcd.backlight();

  //object coff is made
  coffMachine coff;

  // Rising edge of INT0 generates interrupt 
  EICRA |= (1 << ISC01); 
  // Enable interrupts for INT0 
  EIMSK |= (1 <<INT0);
  
  // Enableaprescaler-determined by the internal/external clock
  ADCSRA |=1<< ADPS2;
  // 8-bit results (ALDAR bit is used in the ADMUX register)
  ADMUX |=1<< ADLAR;
  //Enable interrupts
  ADCSRA |=1<< ADIE;
  // Turn on the ADC feature by turning the ADEN bit into 1 in the ADCSRA register 
  ADCSRA |= 1 << ADEN;

  // this means that the all the 4 pins(13 to 10) are inputs with pull up
  DDRB &= ~(1 << PINB3);
  PORTB |= (1 <<PINB3) ;
  DDRB &= ~(1 << PINB2);
  PORTB |= (1 <<PIN2) ;
  DDRB &= ~(1 << PINB4);
  PORTB |= (1 <<PINB4) ;
  DDRB &= ~(1 << PINB5);
  PORTB |= (1 <<PIN5) ;
 
  // pin 4 is output led
  DDRD |= (1 << PIND4);

  //prescaler 1024
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);
  // initialize counter
  TCNT2 = 0;
  // overflow interrupt
  TIMSK2 |= (1 << TOIE2);

  // shows that pin 3 has a servo
  servo1.attach(3);

  // Enable global interrupts 
  sei(); 
  
  while(true){
  coff.coffee();
  }
}