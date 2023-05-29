#include <BluetoothSerial.h>
BluetoothSerial ESP_BT;


//DECLARACIÓN VARIABLES CHASÍS ROBOT
int const sensor_linea_derecha = 34;
int const sensor_linea_izquierda = 32;

int const activacion_puente = 5;

int const izquierda_adelante = 19;
int const izquierda_atras = 18;

int const derecha_adelante = 16;
int const derecha_atras = 17;

int const pin_control_potencia_izquierda = 4; // Pin PWM válido
int const pin_control_potencia_derecha =15; // Pin PWM válido
int const potencia =100;
int const potenciaGiro = 30;

//--------------------------------------------------------------------

//DECLARACIÓN VARIABLES SENSOR DE COLOR
#define S0 2		// S0 a pin 4
#define S1 21		// S1 a pin 5
#define S2 22		// S2 a pin 6
#define S3 23		// S3 a pin 7
#define salidaTCS 0	// salidaTCS a pin 8

void setup() {
  //Bluetooth
  Serial.begin(9600);
  ESP_BT.begin("ESP_bluetooth_proyecto");

  //Carro seguidor de línea
  pinMode(sensor_linea_derecha, INPUT);
  pinMode(sensor_linea_izquierda, INPUT);

  pinMode(activacion_puente, OUTPUT);
  pinMode(izquierda_adelante, OUTPUT);
  pinMode(izquierda_atras, OUTPUT);
  pinMode(derecha_adelante, OUTPUT);
  pinMode(derecha_atras, OUTPUT);
  ledcSetup(0, 5000, 8); // Configura el canal 0 de PWM con una frecuencia de 5000 Hz y resolución de 8 bits
  ledcSetup(1, 5000, 8); // Configura el canal 1 de PWM con una frecuencia de 5000 Hz y resolución de 8 bits
  ledcAttachPin(pin_control_potencia_izquierda, 0); // Asocia el pin de control de potencia izquierda con el canal 0 de PWM
  ledcAttachPin(pin_control_potencia_derecha, 1); // Asocia el pin de control de potencia derecha con el canal 1 de PWM

  //Sensor Color
  pinMode(S0, OUTPUT);		// pin 4 como salida
  pinMode(S1, OUTPUT);		// pin 5 como salida
  pinMode(S2, OUTPUT);		// pin 6 como salida
  pinMode(S3, OUTPUT);		// pin 7 como salida
  pinMode(salidaTCS, INPUT);	// pin 8 como salida
  digitalWrite(S0,HIGH);	// establece frecuencia de salida
  digitalWrite(S1,LOW);		// del modulo al 20 por ciento
}



void loop() {
  //Bluetooth  
  if (ESP_BT.available())
  {
    int dato = ESP_BT.read();
    Serial.println(dato);  
  }

//Carro Seguidor de Línea  

  int estado_sensor_derecho = digitalRead(sensor_linea_derecha);
  int estado_sensor_izquierdo = digitalRead(sensor_linea_izquierda);
  Serial.print("IZ ");
  Serial.print(estado_sensor_izquierdo);
  Serial.print(" DE ");
  Serial.println(estado_sensor_derecho);

  //validación sensores->tabla de verdad
  if (estado_sensor_derecho == 0 && estado_sensor_izquierdo == 0) {
    avanzar();

  } else if (estado_sensor_derecho == 0 && estado_sensor_izquierdo == 1) {
    girar_derecha();

  } else if (estado_sensor_derecho == 1 && estado_sensor_izquierdo == 0) {
    girar_izquierda();

  } else {
    parar();

    delay(500);
    avanzar();
    avanzar();
    avanzar();
    avanzar();
    avanzar();
  }
  
//Sensor Color
  //calculadoraColor();  
  analisisColor();
}
void analisisColor(){
  digitalWrite(S3,LOW);
  int rojo = pulseIn(salidaTCS, LOW);
  delay(200);
  
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  int verde = pulseIn(salidaTCS, LOW);
  delay(200);
  
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  int azul = pulseIn(salidaTCS, LOW);
  delay(200);

  
  //verde
  if(rojo>=165 && rojo<=200 && 
    verde>=100 && verde<=130 &&
    azul>=165 && azul<=190){
      
    ESP_BT.print("0");
    Serial.println("DETECTANDO VERDE");
    parar();
    parar();
    parar();
  }

  
  //rojo
  else if(rojo>=0 && rojo<=200 && 
    verde>=100 && verde<=300 &&
    azul>=0 && azul<=100){ 
    
    Serial.println("DETECTANDO ROJO");
    ESP_BT.print("1");
   parar();
    parar();
    parar();
  }


  //amarillo
  else if(rojo>=30 && rojo<=50 && 
    verde>=35 && verde<=60 &&
    azul>=30 && azul<=50){ 
    
    Serial.println("DETECTANDO AMARILLO");
    ESP_BT.print("2");
    parar();
    parar();
    parar();
  }


  //azul
  else if(rojo>=150 && rojo<=300 && 
    verde>=50 && verde<=200 &&
    azul>=100 && azul<=300){ 
    
    Serial.println("DETECTANDO AZUL");
    ESP_BT.print("3");
    parar();
    parar();
    parar();
  }


    //naranja
  else if(rojo>=35 && rojo<=50 && 
    verde>=115 && verde<=130 &&
    azul>=35 && azul<=50){ 
    
    Serial.println("DETECTANDO NARANJA");
    ESP_BT.print("4");
    parar();
    parar();
    parar();
  }
  



//Sensor Color-> muestra de intensidad por cada canal RGB  
  Serial.print("R:");			// muestra texto
  Serial.print(rojo);			// muestra valor de variable rojo

  Serial.print("\t");			// espacio de tabulacion

  Serial.print("V:");			// muestra texto
  Serial.print(verde);			// muestra valor de variable verde

  Serial.print("\t");			// espacio de tabulacion

  Serial.print("A:");			// muestra texto
  Serial.println(azul);			// muestra valor de variable azul
}



//Carro Seguidor de Línea
void girar_derecha() {
  digitalWrite(activacion_puente, HIGH);
  digitalWrite(izquierda_adelante, LOW);
  digitalWrite(izquierda_atras, HIGH);
  digitalWrite(derecha_adelante, HIGH);
  digitalWrite(derecha_atras, LOW);
  ledcWrite(0, potencia); // Asigna la potencia al canal 0 de PWM (pin_control_potencia_izquierda)
  ledcWrite(1, potencia); // Asigna la potencia al canal 1 de PWM (pin_control_potencia_derecha)
  delay(potenciaGiro);
  Serial.println("Girando a la derecha");
}

void girar_izquierda() {
digitalWrite(activacion_puente, HIGH);
digitalWrite(izquierda_adelante, HIGH);
digitalWrite(izquierda_atras, LOW);
digitalWrite(derecha_adelante, LOW);
digitalWrite(derecha_atras, HIGH);
ledcWrite(0, potencia); // Asigna la potencia al canal 0 de PWM (pin_control_potencia_izquierda)
ledcWrite(1, potencia); // Asigna la potencia al canal 1 de PWM (pin_control_potencia_derecha)
delay(potenciaGiro);
Serial.println("Girando a la izquierda");
}

void avanzar() {
digitalWrite(activacion_puente, HIGH);
digitalWrite(izquierda_adelante, HIGH);
digitalWrite(izquierda_atras, LOW);
digitalWrite(derecha_adelante, HIGH);
digitalWrite(derecha_atras, LOW);
ledcWrite(0, potencia); // Asigna la potencia al canal 0 de PWM (pin_control_potencia_izquierda)
ledcWrite(1, potencia); // Asigna la potencia al canal 1 de PWM (pin_control_potencia_derecha)
delay(50);
Serial.println("Avanzando");
}

void parar() {
digitalWrite(activacion_puente, LOW);
ledcWrite(0, 0); // Detiene el canal 0 de PWM (pin_control_potencia_izquierda)
ledcWrite(1, 0); // Detiene el canal 1 de PWM (pin_control_potencia_derecha)
delay(50);
Serial.println("Final");
}





