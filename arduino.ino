#include <Arduino.h>
#include <Wire.h>
#include <MeMegaPi.h>
#include <Arduino_FreeRTOS.h>
#include <string.h>
#include "semphr.h"

// semáforo del mutex
SemaphoreHandle_t xMutex;

// declaraciones de la tasa de comunicación serial
#define F_CPU 16000000UL
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// handle para un queue
// QueueHandle_t myQueue;

bool detected = false;

TaskHandle_t moveHandle;

// buffer para el UART
unsigned char mybuffer[50];

// semáforo
SemaphoreHandle_t interruptSemaphore;

/////////// MOTORES //////////////////////

MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);

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
//////////funciones de transmisión del UART///////////////

void USART_Transmit(unsigned char data)
{
  // wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)))
    ;

  // put data into buffer, send data
  UDR0 = data;
}

void USART_Transmit_String(unsigned char *pdata)
{
  unsigned char i;
  // calculate string length
  unsigned char len = strlen(pdata);

  // transmit byte for byte
  for (i = 0; i < len; i++)
  {
    // wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)))
      ;
    // put data into buffer, send data
    UDR0 = pdata[i];
  }
}
//////////////////////////////////////////////////////////////
/////////////////////////////////////////////////

void move_task(void *pvParameters)
{
  char inputBuffer[12]; // 11 caracteres + terminador nulo
  uint8_t index = 0;

  while (1)
  {
    // revisa si hay un nuevo dato
    if ((UCSR0A & (1 << RXC0)) != 0)
    {
      // tomar el semáforo
      char c = UDR0;

      // Si no se ha excedido el buffer, guardar el carácter
      if (index < 11)
      { // hacer tarea para que cuando el puerto serial detecte los 11 caracteres ya pase a la siguiente operacion
        inputBuffer[index++] = c;
      }
      // Si se ha recibido 11 caracteres o se detecta un '\n'
      if (index == 11 || c == '\n')
      {
        inputBuffer[index] = '\0'; // Terminar cadena

        // Parsear los datos: esperados formato "123,123,123"
        int vx = 0, vy = 0, vw = 0;
        if (sscanf(inputBuffer, "%d,%d,%d", &vx, &vy, &vw) == 3)
        {
          move_control(vx, vy, vw);

          // mandar respuesta a la rasp
          sprintf(mybuffer, "%d,%d,%d\n", vx, vy, vw);
          USART_Transmit_String((unsigned char *)mybuffer);
        }
        index = 0;
      }
    }
    // dar el semáforo
    // Esperar 10 ms antes de revisar de nuevo (frecuencia de 100 Hz)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vHandlerTaskBarrier(void *pvParameters){
  while(1){
    if(xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdTRUE){
      if(detected == false){
        sprintf(mybuffer, "Sí detecta");
        USART_Transmit_String((unsigned char *)mybuffer);
        move_control(0,0,0);
        vTaskSuspend(moveHandle);
        detected = true;
      } else {
        sprintf(mybuffer, "No detecta");
        USART_Transmit_String((unsigned char *)mybuffer);
        detected = false;
        vTaskResume(moveHandle);
      }     
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

////////// SET UP ///////////////////////////////
void setup()
{
  // configuración del puerto serial
  UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  UCSR0C = 0x06;                         // Set frame format: 8data, 1stop bit
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // TX and RX enables and RX interrupt enabled
  UCSR0A = 0x00;                         // Limpia banderas

  // creación del mutex
  xMutex = xSemaphoreCreateMutex();
  xTaskCreate(move_task, "Mover", 256, NULL, 1, &moveHandle);
  // creación de Handler Task
  xTaskCreate(vHandlerTaskBarrier, "Sensor Handler Task", 100, NULL, 1, NULL);

  // creación del semáforo binario
  interruptSemaphore = xSemaphoreCreateBinary();

  // revisar que se ha creado
  if (xMutex == NULL)
  {
    sprintf(mybuffer, "No se ha creado");
    USART_Transmit_String((unsigned char *)mybuffer);
  }

  // si el semáforo es creado, inicializa interrupción PCINT16
  if (interruptSemaphore != NULL)
  {
    // se hace PK0
    DDRK &= ~(1 << PK0);
    // se habilita interrupción por cambio de estado en PORTC
    PCICR |= (1 << PCIE2);
    // se habilita interrupción PCINT16
    PCMSK2 |= (1 << PCINT16);
    // habilita interrupciones
    sei();
  }

  vTaskStartScheduler();
}

ISR(PCINT2_vect)
{
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
}

/////////////////////////////////////////////////

void loop() {}