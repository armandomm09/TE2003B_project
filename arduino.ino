#include <Arduino.h>
#include <Wire.h>          
#include <MeMegaPi.h>      
#include <Arduino_FreeRTOS.h> 
#include <string.h>        
#include "semphr.h"       

SemaphoreHandle_t xMutex;

#define F_CPU 16000000UL            // Frecuencia del CPU a 16 MHz
#define USART_BAUDRATE 9600         // Baudrate para comunicación serial
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) // Valor del registro UBRR para configurar velocidad UART


bool detected = false;   

// Handle para la tarea que controla el movimiento
TaskHandle_t moveHandle;

unsigned char mybuffer[50];

SemaphoreHandle_t interruptSemaphore;

/////////// CONTROL DE MOTORES //////////////////////

MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);

// Funciones para controlar la velocidad y dirección de cada motor
void motor_foward_left_run(int16_t speed)
{
  motor_10.run(-speed); 
}

void motor_foward_right_run(int16_t speed)
{
  motor_1.run(speed);
}

void motor_back_left_run(int16_t speed)
{
  motor_2.run(-speed);
}

void motor_back_right_run(int16_t speed)
{
  motor_9.run(speed);
}

// Función para controlar el movimiento del robot basado en velocidades vx, vy y vw
void move_control(int16_t vx_raw, int16_t vy_raw, int16_t vw_raw)
{
  
  float scaleFactor = 255.0 / 100.0;
  int16_t vx = round(vx_raw * scaleFactor);
  int16_t vy = round(vy_raw * scaleFactor);
  int16_t vw = round(vw_raw * scaleFactor);

  int16_t foward_left_speed;
  int16_t foward_right_speed;
  int16_t back_left_speed;
  int16_t back_right_speed;

  foward_left_speed = vy + vx + vw;
  foward_right_speed = vy - vx - vw;
  back_left_speed = vy - vx + vw;
  back_right_speed = vy + vx - vw;

  motor_foward_left_run(constrain(foward_left_speed, -255, 255));
  motor_foward_right_run(constrain(foward_right_speed, -255, 255));
  motor_back_left_run(constrain(back_left_speed, -255, 255));
  motor_back_right_run(constrain(back_right_speed, -255, 255));
}

////////// FUNCIONES DE TRANSMISIÓN UART ///////////////

// Envía un byte por UART
void USART_Transmit(unsigned char data)
{
  // Espera hasta que el buffer de transmisión esté vacío
  while (!(UCSR0A & (1 << UDRE0)))
    ;

  // Coloca el dato a transmitir en el registro UDR0
  UDR0 = data;
}

// Envía una cadena por UART
void USART_Transmit_String(unsigned char *pdata)
{
  unsigned char i;

  unsigned char len = strlen(pdata);

  // Transmite byte a byte hasta el final de la cadena
  for (i = 0; i < len; i++)
  {
    while (!(UCSR0A & (1 << UDRE0)))
      ;
    UDR0 = pdata[i];
  }
}

//////////////////////////////////////////////////////////////

// Tarea para controlar el movimiento leyendo datos desde UART
void move_task(void *pvParameters)
{
  char inputBuffer[11]; // Buffer para recibir comandos (11 caracteres)
  uint8_t index = 0;

  while (1)
  {
    // Revisa si hay datos 
    if ((UCSR0A & (1 << RXC0)) != 0)
    {
      // Lee el carácter recibido
      char c = UDR0;

      // Guarda el carácter en el buffer si no se ha llenado
      if (index < 11)
      {
        inputBuffer[index++] = c;
      }

      // Cuando se reciben 11 caracteres o un salto de línea
      if (index == 11 || c == '\n')
      {
        inputBuffer[index] = '\0'; // Finaliza  cadena

        // extraer tres números en formato vx,vy,vw
        int vx = 0, vy = 0, vw = 0;
        if (sscanf(inputBuffer, "%d,%d,%d", &vx, &vy, &vw) == 3)
        {
          move_control(vx, vy, vw);

          // Enviar respuesta por UART 
          sprintf(mybuffer, "%d,%d,%d\n", vx, vy, vw);
          USART_Transmit_String((unsigned char *)mybuffer);
        }
        
        index = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Tarea para manejar la barrera/sensor y la detección
void vHandlerTaskBarrier(void *pvParameters){
  while(1){
    // Espera a que el semáforo de interrupción sea tomado
    if(xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdTRUE){
      if(detected == false){
        // Si no se detectó antes, indica que detecta
        sprintf(mybuffer, "Sí detecta");
        USART_Transmit_String((unsigned char *)mybuffer);

        // Detener motores 
        move_control(0,0,0);

        // Suspender tarea de movimiento
        vTaskSuspend(moveHandle);

        detected = true;
      } else {
        // Si ya estaba detectado, indica que no detecta
        sprintf(mybuffer, "No detecta");
        USART_Transmit_String((unsigned char *)mybuffer);

        detected = false;

        // Reanuda tarea 
        vTaskResume(moveHandle);
      }     
    }
    // Espera 20 ms antes de volver a revisar semáforo
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

////////// CONFIGURACIÓN INICIAL ///////////////////////////////
void setup()
{
  // Configuración UART: baudrate, formato de trama (8 bits, 1 stop bit)
  UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  UCSR0C = 0x06;                         // 8 bits de datos, 1 bit de parada
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // Habilita transmisión y recepción UART
  UCSR0A = 0x00;                         // Limpia flags


  xTaskCreate(move_task, "Mover", 256, NULL, 1, &moveHandle);
  xTaskCreate(vHandlerTaskBarrier, "Sensor Handler Task", 100, NULL, 1, NULL);

  interruptSemaphore = xSemaphoreCreateBinary();

  // Si el semáforo se creó, configurar la interrupción PCINT16 
  if (interruptSemaphore != NULL)
  {
    // Configura PK0 (Pin K0) como entrada
    DDRK &= ~(1 << PK0);

    // Habilita interrupción por cambio en PORTC 
    PCICR |= (1 << PCIE2);

    // Habilita interrupción PCINT16 
    PCMSK2 |= (1 << PCINT16);

    // Habilita interrupciones 
    sei();
  }

}

ISR(PCINT2_vect)
{
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
}

/////////////////////////////////////////////////

void loop() {}
