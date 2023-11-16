#include <LiquidCrystal.h>
#include <LiquidCrystal_Config.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <uart.h>
#include <util/delay.h>

// IO
// P1 (On/Off) -> PD0
// P2 (Sentido) -> PD1

// Pines usados por la librería lcd_2560.h: 
#define RS eS_PORTA0 // Pin RS = PA0 (22) (Reset). 
#define EN eS_PORTA1 // Pin EN = PA1 (23) (Enable). 
#define D4 eS_PORTA2 // Pin D4 = PA2 (24) (Data D4). 
#define D5 eS_PORTA3 // Pin D5 = PA3 (25) (Data D5). 
#define D6 eS_PORTA4 // Pin D6 = PA4 (26) (Data D6). 
#define D7 eS_PORTA5 // Pin D7 = PA5 (27) (Data D7). 

#define BOUNCE_DELAY 10
#define BAUD_RATE 9600UL

uint8_t flag = 0;
uint8_t motorIsOn = 0;
uint8_t trx_buffer = 0;
uint8_t ninth_bit = 0;
char buffer[16]; // Vector de caracteres que almacena string (16 = Nº de filas del LCD)
volatile uint8_t phase[] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
volatile uint8_t phase_index = 0;
volatile uint8_t direction = 0;

int main(void) {

  // Configuración de puertos
  DDRC  = 0xFF; // Puerto C como salida
  PORTC = 0x00; // Inicializa en 0 el puerto C
  DDRA  = 0xFF; // Puerto A como salida
  PORTA = 0x00; // Inicializa en 0 el puerto A
  DDRD &= ~((1 << DDD0) | (1 << DDD1)); // puertos D como entrada 
  PORTD |= ((1 << PORTD0) | (1 << PORTD1)); // habilita resistencias de pullup 
 
  // Configura las interrupciones 
  EICRA |= (1 << ISC01); // config. interrup. INT0 sensible a flanco asc. 
  EIMSK |= (1 << INT0);  // habilita interrp. interna INT0 
  EIFR = 0x00;           // borra flag INTF0 para evitar interrup. espúrea 
  sei();                 // activa interrupciones globalmente 

  // Timer 1 (16 bits)
  TCCR1B = (1 << WGM12);
  TIMSK1 = (1 << OCIE1A);
  OCR1A = 6249; // 6250 * 256 * (1/16MHz) = 100ms

  // Modulo UART
  UBRR1 = (16000000UL / (16 * BAUD_RATE)) - 1;
  UCSR1B = (1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1) | (1 << UCSZ12);
  UCSR1C = (1 << UCSZ10) | (1 << UCSZ11); // 9 bit wide

  void start_motor();
  void stop_motor();

  while (1) {
    if (flag) {
      if (motorIsOn)
        stop_motor();
      else
        start_motor();
      flag = 0;
    }

  }

  return 0;
}

void start_motor() {
  /* UCSR1A |= (1 << TXB81); */
  // TODO: chequear que no se este recibiendo datos
  trx_buffer = 0x01;
  UDR1 = trx_buffer;
  while(!(UCSR1A & (1 << TXC1))) {};
  motorIsOn = 1;
}

void stop_motor() {
  /* UCSR1A &= ~(1 << TXB81); */
  // TODO: chequear que no se este recibiendo datos
  trx_buffer = 0x00;
  UDR1 = trx_buffer;
  while(!(UCSR1A & (1 << TXC1))) {};
  motorIsOn = 0;
}

ISR(INT0_vect) {
    _delay_ms(BOUNCE_DELAY);
    if (!(PIND & (1 << PD0))) // Comprobar si P1 sigue en BAJO
        flag = 1; // Establecer la bandera para indicar la interrupción
}

ISR(USART1_RX_vect) {
  while(!(UCSR1A & (1 << RXC1))) {};
  trx_buffer = UDR1;
  //ninth_bit = (UCSR1A & (1 << RXC1));
}

ISR(TIMER1_COMPA_vect) {
  if (direction) {
    phase_index++;
    if (phase_index > 7)
      phase_index = 0;
  }
  else {
    phase_index--;
    if (phase_index < 0)
      phase_index = 7;
  }
  PORTC = phase[phase_index];
}
