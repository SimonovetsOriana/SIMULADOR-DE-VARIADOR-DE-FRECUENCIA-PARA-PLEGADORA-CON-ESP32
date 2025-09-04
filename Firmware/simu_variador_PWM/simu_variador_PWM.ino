#include <Arduino.h>
#include "lcd.h"

// --- Configuración de pines ---
#define PWM1_PIN 27
#define PWM2_PIN 14
#define P1_PIN 22
#define P2_PIN 21
#define LLS1_PIN 26
#define LLS2_PIN 25
#define X1_PIN 34 // Girar hacia un lado
#define X2_PIN 35 // Girar hacia el otro lado
#define X3_PIN 32 // Velocidad baja
#define X4_PIN 33 // Velocidad alta
#define MENU_SIZE 5

// --- Constantes--- //
const byte ENCODER_A_PIN = 4;
const byte ENCODER_B_PIN = 19;

//  ------------------------------------------------------------------------------------ //
// -------------------------------- Variables globales --------------------------------- //
//  ------------------------------------------------------------------------------------ //

// FLAGS
bool flagMM=1;
bool programming_mode = false;
volatile bool counter = false;  // Bandera timer
volatile bool motor_on= false;
volatile bool motor_off =false;

// Configuraciones
uint8_t speed=100; uint8_t acceleration=3; uint8_t deceleration=3; uint8_t speedHigh=100; uint8_t speedLow=60;
uint8_t dutyCycleMin = 128; // Valor mínimo del ciclo de trabajo (0 a 255)-- 0=100% , 204= 20%, 128=50%, 255=0% (PWM invertido)
bool controlMode=1; // 1: modo manual, 0: mediante entradas digitales (X1,X2,X3,X4)

// Handler del timer
uint8_t PWM1_CH=0; uint8_t PWM2_CH=1;
hw_timer_t * timer = NULL;

enum ParamType {
  SPEED, SPEED_HIGH, SPEED_LOW,
  ACCELERATION, DECELERATION
};

//  ------------------------------------------------------------------------------------ //
// ------------------------------------ Prototipos  ------------------------------------ //
//  ------------------------------------------------------------------------------------ //
// Inicialización
void init_config(void); 
void config_PWM(void);
// Control del motor
void switch_on(uint8_t PWM_channel, uint8_t speed_limit);  // Arranque suave
void switch_off(uint8_t PWM_channel, uint8_t speed_limit); // Apagado suave
// Funciones del variador
bool controlModeChange(); // Seleccionar el modo de control
void manualControl(void); 
void digiControl(void);
char programming(void);
// Funciones auxiliares
void rotaryEncoder(int8_t &delta);
bool paramChange(uint8_t* param, ParamType type, const char* paramName,const char* paramUnit);
// Encuentas a los pulsadores
bool pollP1(void);
bool pollP2(void);

// ISR del timer (cada 100 ms)
void IRAM_ATTR onTimer() {
  counter = true;
}

//  ------------------------------------------------------------------------------------ //
// --------------------------------------- SET UP -------------------------------------- //
//  ------------------------------------------------------------------------------------ //
void setup() {

  Serial.begin(115200);

  // Entradas Pulsadores
  pinMode(P1_PIN, INPUT_PULLUP);
  pinMode(P2_PIN, INPUT_PULLUP);

  // Entradas Encoder
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  // Entradas Llave selectora
  pinMode(LLS1_PIN, INPUT_PULLUP);
  pinMode(LLS2_PIN, INPUT_PULLUP);

  // Entradas Digitales
  pinMode(X1_PIN, INPUT);
  pinMode(X2_PIN, INPUT);
  pinMode(X3_PIN, INPUT_PULLUP);
  pinMode(X4_PIN, INPUT_PULLUP);

  // Inicializar la pantalla
  u8g2.begin();
  u8g2_prepare();

  //Inicializar PWM
  config_PWM();
  Serial.println("Sistema inicializado.");
}

//  ------------------------------------------------------------------------------------ //
// ---------------------------------------- LOOP --------------------------------------- //
//  ------------------------------------------------------------------------------------ //
void loop() {
  char option = NULL; const char* mode;
  pollP1();
  pollP2();

  if(controlMode) mode= "Manual";
  else mode= "Entradas digitales";

  if(flagMM){
    u8g2_MainMenu(speed,speedHigh, speedLow, acceleration, deceleration, mode );
    flagMM=0;
  }

  if (programming_mode) {
    Serial.println("Modo Programación");
    option = programming();
    switch (option) {
      case 0: // Volver
        programming_mode = false;
        flagMM=1;
        break;
      case 1: // Modificar modo de control
        Serial.println("Entrando a modificar modo de control");
        controlModeChange();
        break;
      case 2: // Variar velocidad con encoder
        Serial.println("Entrando a variar velocidad con encoder");
        paramChange(&speed,SPEED,"Vel.","%");
        break;
      case 3: // Modificar tiempo de aceleración
        paramChange(&acceleration,ACCELERATION,"Acel.","s");
        break;
      case 4: // Modificar tiempo de desaceleración
        paramChange(&deceleration,DECELERATION,"Desacel.","s");
        break;
      case 5: // Modificar velocidad alta configurada
        paramChange(&speedHigh,SPEED_HIGH,"Vel.","%");
        break;
      case 6: // Modificar velocidad baja configurada
        paramChange(&speedLow,SPEED_LOW,"Vel.","%");
        break;
    }
  }else{
    if(controlMode){
      manualControl(); // Control manual
    }else{
      digiControl(); // Control Digital
    }
  }
}
//  ------------------------------------------------------------------------------------ //
// ------------------------------------- FUNCIONES ------------------------------------- //
//  ------------------------------------------------------------------------------------ //

// ----------------------- Inicialización ----------------------- //

void init_config() {// Restaurar configuración de fábrica
  speed=100; 
  acceleration=3; 
  deceleration=3; 
  speedHigh=100; 
  speedLow=30;
  controlMode=1; // Manual
  flagMM=1;
  Serial.println("RESET - Valores por defecto cargados.");
}

void config_PWM(void){
  const int frequency = 1000;  // Frecuencia en Hz
  const int resolution = 8;    // Resolución en bits (de 1 a 15)

   // Config PWM
  ledcAttachChannel(PWM1_PIN, frequency, resolution, PWM1_CH);
  ledcAttachChannel(PWM2_PIN, frequency, resolution, PWM2_CH);

  // Config TIMER
  timer = timerBegin(10000);   // resolución
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000, true, 0); // periodo 1000 ticks, autoreload
  timerStop(timer);                 // arranca apagado

  // Inicializa salidas PWM en alto (Motor apagado)
  ledcWriteChannel(PWM1_CH, 255);
  ledcWriteChannel(PWM2_CH, 255);
}

// ----------------------- Control del motor ----------------------- //

void switch_on(uint8_t PWM_channel, uint8_t speed_limit){  // Arranque suave
  uint8_t count=0;
  float step = (float)speed_limit / (10.0f * acceleration);
  float duty = 0.0f;

  timerRestart(timer);     // Reset del timer antes de arrancar
  timerStart(timer); // Arranca el conteo

    while (count < acceleration*10)
    { 
      if (counter){ // Cuando se dispara la int. del contador se realiza el incremento
        counter = false;
        count += 1;
        duty += step;
        if (duty >= speed_limit) {
          duty = speed_limit;  // Evita overflow
        }
        // Se pasa a entero para escribir el PWM
        ledcWriteChannel(PWM_channel, (uint32_t)(dutyCycleMin + (255-dutyCycleMin)*(100.0f-duty)*0.01f));
      }
    }
  timerStop(timer); // Detenemos el timer al final
  ledcWriteChannel(PWM_channel, 
    (uint32_t)(dutyCycleMin + (255-dutyCycleMin)*(100.0f-speed_limit)*0.01f));
  motor_on = true;
}

void switch_off(uint8_t PWM_channel, uint8_t speed_limit){ // Apagado suave
  uint8_t count=0;
  float step = (float)speed_limit / (10.0f * deceleration);
  float duty = (float)speed_limit;

  timerRestart(timer);     // Reset del timer antes de arrancar
  timerStart(timer); // Arranca el conteo

    while (count < deceleration*10)
    { 
      if (counter){ // Cuando se dispara la int. del contador se realiza el decremento
        counter = false;
        count += 1;

        if (duty >= step) duty -= step; //Evita underflow
        else {
          duty = 0.0f;
        }
        ledcWriteChannel(PWM_channel,(uint32_t)(dutyCycleMin + (255-dutyCycleMin)*(100.0f-duty)*0.01f));
      }
    }
  timerStop(timer); // Detenemos el timer al final
  ledcWriteChannel(PWM_channel, 
    (uint32_t)(dutyCycleMin + (255-dutyCycleMin)*(100.0f-0)*0.01f));
  motor_off = true;
}

// ----------------------- Funciones del variador ----------------------- //

bool controlModeChange() { // Modificar Modo de Control 
  int8_t ControlMenu_size = 1;
  int8_t selector = 0;
  int8_t laststate=0;

  u8g2_ControlMenu(selector); // imprime la pantalla la primera vez

  while (true) {
    int8_t state = 0;
    
    if (pollP2()) return 0;

    rotaryEncoder(state);
    if(state!=laststate) {
      u8g2_ControlMenu(selector); // actualiza la pantalla solo si hubo un cambio
      laststate=state;
    }

    if (state == 1) {
      if(selector==ControlMenu_size) selector = 0;
      else selector ++;
    }
    if (state == -1) {
      if(selector==0) selector = ControlMenu_size;
      else selector--;
    }
    if (selector == 0) {
      if (pollP2()) return 0;
      if (pollP1()) {
        controlMode=1; // Modo Manual
        return 1;
      }
    }

    if (selector == 1) {
      if (pollP2()) return 0;
      if (pollP1()) {
        controlMode=0; // Entradas Digitales
        return 1;
      }
    }
  }
}

void manualControl(void){ // Control Manual

  if (digitalRead(LLS1_PIN) == LOW) {
    delay(10); // Delay para evitar rebote
    while (digitalRead(LLS1_PIN) == LOW) {
      motor_off=false;
      if(!motor_on) {
        switch_on(PWM2_CH,speed); // Canal 1 : PWM2
      }
    }
    motor_on=false;
    if(!motor_off) {
     switch_off(PWM2_CH,speed); // Vuelve a apagar el motor
    }
  }
  else if (digitalRead(LLS2_PIN) == LOW){
    delay(10); // Delay para evitar rebote
    while(digitalRead(LLS2_PIN) == LOW) {
      motor_off=false;
      if(!motor_on){
        switch_on(PWM1_CH,speed); // Canal 0 : PWM1
      }
    }
    motor_on=false;
    if(!motor_off){
      switch_off(PWM1_CH,speed); // Vuelve a apagar el motor
    }
  }
}

void digiControl(void){ // Control Digital
  uint8_t auxSpeed=0;

  if ((digitalRead(X3_PIN) == LOW)&& (digitalRead(X4_PIN) == HIGH)) {
    delay(20); // Delay para evitar rebote
    if ((digitalRead(X3_PIN) == LOW)&& (digitalRead(X4_PIN) == HIGH) ) {
      auxSpeed=speedHigh;
    }
  }
  else if ((digitalRead(X4_PIN) == LOW)&& (digitalRead(X3_PIN) == HIGH)) {
    delay(20); // Delay para evitar rebote
    if ((digitalRead(X4_PIN) == LOW)&& (digitalRead(X3_PIN) == HIGH) ) {
      auxSpeed=speedLow;
    }
  }
  else if ((digitalRead(X4_PIN) == LOW)&& (digitalRead(X3_PIN) == LOW)) {
    delay(20); // Delay para evitar rebote
    if ((digitalRead(X4_PIN) == LOW)&& (digitalRead(X3_PIN) == HIGH) ) {
      auxSpeed=0;
    }
  }else{
    auxSpeed=speed;
  }

  if ((digitalRead(X1_PIN) == LOW) && (digitalRead(X2_PIN) == HIGH)) {
    delay(10); // Delay para evitar rebote
    while ((digitalRead(X1_PIN) == LOW) && (digitalRead(X2_PIN) == HIGH)) {
      motor_off=false;
      if(!motor_on) {
        switch_on(PWM2_CH,auxSpeed); // Canal 1 : PWM2
      }
    }
    motor_on=false;
    if(!motor_off){
      switch_off(PWM2_CH,auxSpeed); // Vuelve a apagar el motor
    }
  }
  else if ((digitalRead(X2_PIN) == LOW) && (digitalRead(X1_PIN) == HIGH)){
    delay(10); // Delay para evitar rebote
    while((digitalRead(X2_PIN) == LOW) && (digitalRead(X1_PIN) == HIGH)) {
      motor_off=false;
      if(!motor_on) {
        switch_on(PWM1_CH,auxSpeed); // Canal 1 : PWM2
      }
    }
    motor_on=false;
    if(!motor_off){
      switch_off(PWM1_CH,auxSpeed); // Vuelve a apagar el motor
    }
  }
}

char programming() { // Modo Configuración
  int8_t selector = 0;
  int8_t laststate=0;

  u8g2_ProgramMenu(selector); // imprime la pantalla la primera vez

  while (true) {
    int8_t state = 0;
    
    if (pollP2()) return 0;

    rotaryEncoder(state);
    if(state!=laststate) {
      u8g2_ProgramMenu(selector); // actualiza la pantalla solo si hubo un cambio
      laststate=state;
    }

    if (state == 1) {
      if(selector==MENU_SIZE) selector = 0;
      else selector ++;
    }
    if (state == -1) {
      if(selector==0) selector = MENU_SIZE;
      else selector--;
    }
    if (selector == 0) {
      if (pollP2()) return 0;
      if (pollP1()) return 1;
    }

    if (selector == 1) {
      if (pollP2()) return 0;
      if (pollP1()) return 2;
    }

    if (selector == 2) {
      if (pollP2()) return 0;
      if (pollP1()) return 3;
    }

    if (selector == 3) {
      if (pollP2()) return 0;
      if (pollP1()) return 4;
    }

    if (selector == 4) {
      if (pollP2()) return 0;
      if (pollP1()) return 5;
    }

    if (selector == 5) {
      if (pollP2()) return 0;
      if (pollP1()) return 6;
    }

    if (selector == 6) {
      if (pollP2()) return 0;
      if (pollP1()) return 7;
    }
  }
}

// ----------------------- Funciones axiliares ----------------------- //

bool paramChange(uint8_t* param, ParamType type, const char* paramName,const char* paramUnit) {
  int8_t laststate = 0;
  uint8_t temp_param = *param;
  uint8_t lowlim=0;
  uint8_t upplim=0;

  // Definir límites según el tipo de parámetro
  if(type == SPEED || type == SPEED_HIGH || type == SPEED_LOW){
     upplim = 100;
  } else if(type == ACCELERATION || type == DECELERATION) {
     upplim = 10;
     lowlim=1;
  }

  u8g2_ParamChange(temp_param, paramName,paramUnit); // actualiza pantalla la primera vez

  while(!pollP2()){
    int8_t state = 0;

    rotaryEncoder(state);

    if(state != laststate) {
      u8g2_ParamChange(temp_param, paramName,paramUnit); // solo si hay cambio
      laststate = state;
    }

    if(state == 1 && temp_param < upplim){
      temp_param++;
    }
    if(state == -1 && temp_param > lowlim){
      temp_param--;
    }

    if (pollP1()) {
      *param = temp_param; // actualizar valor real si se acepta
      return true;
    }
  }

  return false; // Cancelar
}

void rotaryEncoder(int8_t &delta) { // Encoder
  delta = 0;
  enum {STATE_LOCKED, STATE_TURN_RIGHT_START, STATE_TURN_RIGHT_MIDDLE, STATE_TURN_RIGHT_END, STATE_TURN_LEFT_START, STATE_TURN_LEFT_MIDDLE, STATE_TURN_LEFT_END, STATE_UNDECIDED};
  static uint8_t encoderState = STATE_LOCKED;
  bool a = !digitalRead(ENCODER_A_PIN);
  bool b = !digitalRead(ENCODER_B_PIN);
  switch (encoderState) {
    case STATE_LOCKED:
      if (a && b) {
        encoderState = STATE_UNDECIDED;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_LEFT_START;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_RIGHT_START;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
    case STATE_TURN_RIGHT_START:
      if (a && b) {
        encoderState = STATE_TURN_RIGHT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_RIGHT_END;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_RIGHT_START;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
    case STATE_TURN_RIGHT_MIDDLE:
    case STATE_TURN_RIGHT_END:
      if (a && b) {
        encoderState = STATE_TURN_RIGHT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_RIGHT_END;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_RIGHT_START;
      }
      else {
        encoderState = STATE_LOCKED;
        delta = -1;
      };
      break;
    case STATE_TURN_LEFT_START:
      if (a && b) {
        encoderState = STATE_TURN_LEFT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_LEFT_START;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_LEFT_END;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
    case STATE_TURN_LEFT_MIDDLE:
    case STATE_TURN_LEFT_END:
      if (a && b) {
        encoderState = STATE_TURN_LEFT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_LEFT_START;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_LEFT_END;
      }
      else {
        encoderState = STATE_LOCKED;
        delta = 1;
      };
      break;
    case STATE_UNDECIDED:
      if (a && b) {
        encoderState = STATE_UNDECIDED;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_RIGHT_END;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_LEFT_END;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
  }
}

// ----------------------- Encuestas a los pulsadores ----------------------- //

bool pollP1() {
  if (digitalRead(P1_PIN) == LOW) {
    delay(10); // Pequeño delay para evitar rebote
    if (digitalRead(P1_PIN) == LOW) {
      while (digitalRead(P1_PIN) == LOW); // Espera hasta que se suelte
      if (!programming_mode) {
        programming_mode = true;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool pollP2() {
  if (digitalRead(P2_PIN) == LOW) {
    delay(10); // Pequeño delay para evitar rebote
    if (digitalRead(P2_PIN) == LOW) {
      while (digitalRead(P2_PIN) == LOW); // Espera hasta que se suelte
      if (!programming_mode) {
        init_config();
      }else{
        return true;
      }
    }
  }
  return false;
}

