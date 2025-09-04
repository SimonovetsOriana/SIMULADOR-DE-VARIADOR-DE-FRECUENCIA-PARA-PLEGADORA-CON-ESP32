#include <Arduino.h>
#include "lcd.h"
#include <U8g2lib.h>
#include <SPI.h>

//---------------------Driver de la pantalla y configuracion de pines para el ESP32-----------------------------

U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, CLK, MOSI, CS, RESET );           // ESP32  y pantalla


void u8g2_prepare(void) {

  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.setFontRefHeightExtendedText();

    u8g2.setDrawColor(1);

  u8g2.setFontPosTop();

  u8g2.setFontDirection(0);

}

// Pantalla Principal
void u8g2_MainMenu( uint8_t v, uint8_t vh, uint8_t vl, uint8_t a, uint8_t d, const char* mode){
  char buffer1[64];
  char buffer2[64];
  char buffer3[128];
  char buffer4[64];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  u8g2.setDrawColor(1);                 // Blanco
  u8g2.drawBox(0, 0, 128, 12);         // Fondo
  u8g2.setDrawColor(0);                 // Negro sobre blanco
  u8g2.drawUTF8(0, 0, "KUTS S.A.S - Simu Variador");
  u8g2.setDrawColor(1); 
  
  sprintf(buffer1,"Modo: %s",mode);
  u8g2.drawUTF8(0, 20, buffer1);
  sprintf(buffer2,"Vel. normal: %d %s",v,"%");
  u8g2.drawUTF8(0, 30, buffer2);
  sprintf(buffer3,"V1: %d , V2: %d %s",vh,vl,"%");
  u8g2.drawUTF8(0, 40, buffer3);
  sprintf(buffer4,"ac.: %d s, des.: %d s",a,d);
  u8g2.drawUTF8(0, 50, buffer4);


  u8g2.sendBuffer();
}

// Pantalla Modo Programacion
void u8g2_ProgramMenu(int select){
  char buffer[20];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  if(select<4){
    u8g2.drawUTF8(0, 0, "Configuración");
    if(select == 0){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 10, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawStr(0, 10, "> Modo de control");
      u8g2.setDrawColor(1);                 // Restaurar a blanco normal
    }else u8g2.drawStr(0, 10, "Modo de control");

    if(select == 1){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 20, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawStr(0, 20, "> Velocidad actual");
      u8g2.setDrawColor(1);                 // Restaurar a blanco normal
    }else u8g2.drawStr(0, 20, "Velocidad actual");
    
    if(select == 2){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 30, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawUTF8(0, 30, "> Tiempo aceleración");
      u8g2.setDrawColor(1);                 // Restaurar a blanco normal
    }else u8g2.drawUTF8(0, 30, "Tiempo aceleración");

    if(select == 3){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 40, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawUTF8(0, 40, "> Tiempo desaceleración");
      u8g2.setDrawColor(1);      
    }else u8g2.drawUTF8(0, 40, "Tiempo desaceleración");

    u8g2.drawStr(0, 50, "Volver");
    u8g2.drawStr(86, 50, "Aceptar");
    u8g2.sendBuffer();
  }else{
    u8g2.drawUTF8(0, 0, "Configuración");

    if(select == 4){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 20, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawStr(0, 20, "> Velocidad alta");
      u8g2.setDrawColor(1);     
    }else u8g2.drawStr(0, 20, "Velocidad alta");

    if(select == 5){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 30, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawStr(0, 30, "> Velocidad baja");
      u8g2.setDrawColor(1);     
    }else u8g2.drawStr(0, 30, "Velocidad baja");
    
    u8g2.drawStr(0, 50, "Volver");
    u8g2.drawStr(86, 50, "Aceptar");
    u8g2.sendBuffer();
  }
}

// Pantalla en Modo Control
void u8g2_ControlMenu(int select){
  char buffer[20];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

    u8g2.drawUTF8(0, 0, "Modificar modo de control");
    if(select == 0){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 10, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawStr(0, 10, "> Modo manual");
      u8g2.setDrawColor(1);                 // Restaurar a blanco normal
    }else u8g2.drawStr(0, 10, "Modo manual");

    if(select == 1){
      u8g2.setDrawColor(1);                 // Blanco
      u8g2.drawBox(0, 20, 128, 11);         // Fondo
      u8g2.setDrawColor(0);                 // Negro sobre blanco
      u8g2.drawStr(0, 20, "> Entradas digitales");
      u8g2.setDrawColor(1);                 // Restaurar a blanco normal
    }else u8g2.drawStr(0, 20, "Entradas digitales");
    
    u8g2.drawStr(0, 50, "Volver");
    u8g2.drawStr(86, 50, "Aceptar");
    u8g2.sendBuffer();
}

// Pantalla en Variar Parámetro
void u8g2_ParamChange(uint8_t value, const char* valueName,const char* unit){
  char buffer1[64];
  char buffer2[64];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  sprintf(buffer1,"Variar %s",valueName);
  u8g2.drawStr(0, 0, buffer1);
  sprintf(buffer2,"%s actual: %d %s",valueName,value,unit);
  u8g2.drawStr(0, 30, buffer2);
  u8g2.drawStr(0, 50, "Cancelar");
  u8g2.drawStr(86, 50, "Aceptar");
  u8g2.sendBuffer();
}


