// Learning embedded coding
// Task: define pins 2 to 13 as output and then set pins 11,12 and 13 as HIGH and other pins LOW
void setup() {
  // put your setup code here, to run once:
  /*
   int pin;
   for (pin = 2; pin <= 13; ++pin){
       pinMode(pin,OUTPUT);
   }
   for (pin = 2; pin <=10; ++pin) {
       digitalWrite(pin,LOW);
   }
   for (pin = 11; pin<= 13; ++pin){
       digitalWrite(pin,HIGH);
   }
   */
   // DDR: tells whether pin is an input or output
   // PORT: it registers whether the pin is a HIGH or LOW
   // PIN: to read the state of the pins which are defined as the inputs
   
   // Control registers DDRD and DDRB each contain 8 bits.
   // set the output 2 TO 7
   DDRD = B11111110; // D7, D6, D5, D4, D3, D2, D1, D0

   // The upper two bits in DDRB are not used. empty. 
   // set the output 8 to 13
   DDRB = B00111111; // 15, 14, D13, D12, D11, D10, D9, D8

   // port registers, PORTB and PORTD also contain 8 bit. 
   // set 11 to 13 as HIGH 
   PORTB = B00111000; // 

   // set D7,...D0 as LOW
   PORTD = PORTD & B00000011; // turns off 7 to 2... but leaves pins 0 and 1 alone
   // can also be written as PORTD &= B00000011
}

void loop() {
  // put your main code here, to run repeatedly:

}
