
// ----------------------------- SENSOR HIGROMETRO ----------------------------------------------------------
int tempoIrrig = 50000;
int sensorUmidadeSolo_1 = A0;
int sensorUmidadeSolo_2 = A1;
int sensorUmidadeSolo_3 = A2;
int sensorUmidadeSolo_4 = A3;
int portaRele_12 = 12;
int portaRele_7 = 7;
int portaRele_3 = 3;
int portaRele_4 = 4;
int valorLimiteUmidade = 1000;
bool soloUmido_1;
bool soloUmido_2;
bool soloUmido_3;
bool soloUmido_4;
// ----------------------------------------------------------------------------------------------------------
// ----------------------------- SENSOR DHT22 ----------------------------------------------------------
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTTYPE     DHT22

#define DHTPIN 2
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS = 500;
float valorLimiteUmidadeAr = 85;            // valor de comparação da umidade do ar para identificar se está chovendo
float valorLimiteTemperatura = 38;          // valor decomparação para não regar quando estiver muito quente
bool climaUmido;
bool temperaturaAlta;
// ------------------------------------------------------------------------------------------------------


/* MUDAR PARA O PINO DIGITAL
// ------------------------------- SENSOR LDR -----------------------------------------------------------
#define AnalogLDR A0     // define o pino analógico A0 -- SENSOR LDR --
#define Limiar 2.0      // define a constante igual a 4.5 - valor obtido a partir de testes em ambiente controlado -- SENSOR LDR --
#define ledPin 13       // define o pino digital D13 -- SENSOR LDR --

int leitura = 0;        // variável inteiro com valor inicial igual à zero -- SENSOR LDR --
float voltageLDR;       // variável que vai receber o valor da tensão do LDR -- SENSOR LDR --
bool luminosidade;
// -------------------------------------------------------------------------------------------------------

*/

// -------------------------------------ESP01-8266-------------------------------------------------------------
#include <A2a.h>
#define endereco 0x08
A2a arduinoMaster;



//----------------------------------------------------------------------------------------------------------

void setup() {
//--------------------------------ESP01-8266------------------------------------
  arduinoMaster.begin(endereco);
  arduinoMaster.onReceive(receberDados);
  arduinoMaster.onRequest(enviarDados);
//-------------------------------------------------------------------------------


// ----------------------------- SENSOR HIGROMETRO ----------------------------------------------------------
  pinMode(sensorUmidadeSolo_1, INPUT);
  pinMode(sensorUmidadeSolo_2, INPUT);
  pinMode(sensorUmidadeSolo_3, INPUT);
  pinMode(sensorUmidadeSolo_4, INPUT);
  pinMode(portaRele_12, OUTPUT);
  pinMode(portaRele_7, OUTPUT);
  pinMode(portaRele_3, OUTPUT);
  pinMode(portaRele_4, OUTPUT);
  digitalWrite(portaRele_12, HIGH);     //desliga a bomba conctada no pino digital 12
  digitalWrite(portaRele_7, HIGH);      //desliga a bomba conctada no pino digital 7
  digitalWrite(portaRele_3, HIGH);      //desliga a bomba conctada no pino digital 3
  digitalWrite(portaRele_4, HIGH);      //desliga a bomba conctada no pino digital 4
  Serial.begin(9600);
// ---------------------------------------------------------------------------------------------------------  
  
// ----------------------------- SENSOR DHT22 ----------------------------------------------------------
  Serial.begin(9600);
  dht.begin();
  Serial.println("Usando o Sensor DHT");
  sensor_t sensor;
// -----------------------------------------------------------------------------------------------------


/* MUDAR PARA O PINO DIGITAL
// ------------------------------- SENSOR LDR -----------------------------------------------------------  
  pinMode(ledPin,OUTPUT);       // configura D13 como saída digital -- SENSOR LDR --
  Serial.begin(9600);           // monitor serial - velocidade 9600 Bps -- SENSOR LDR --
  delay(100);                   // atraso de 100 milisegundos -- SENSOR LDR --
// ------------------------------------------------------------------------------------------------------

*/
}
// FUNÇÃO DO SENSOR DE UMIDADE DO SOLO-----------------------------------------------
void SensorDeUmidade_1 (){
  int valorSensorUmidadeSolo_1 = analogRead(sensorUmidadeSolo_1);

// ------------------ESP01-8266-------------------------------------
   arduinoMaster.varWireWrite(0, highByte(valorSensorUmidadeSolo_1));
   arduinoMaster.varWireWrite(1, lowByte(valorSensorUmidadeSolo_1));
//----------------------------------------------------------------
     
  Serial.print(" Sensor de umidade do solo = ");
  Serial.print(valorSensorUmidadeSolo_1);
  if (valorSensorUmidadeSolo_1 < valorLimiteUmidade) {
    Serial.println(" O solo está úmido");
    soloUmido_1 = 1;
  } else {
    Serial.println(" O solo está seco");
    soloUmido_1 = 0;
  }
}
void SensorDeUmidade_2 (){
  int valorSensorUmidadeSolo_2 = analogRead(sensorUmidadeSolo_2);

// -------------------- ESP01-8266 ---------------------------
   arduinoMaster.varWireWrite(0, highByte(valorSensorUmidadeSolo_2));
   arduinoMaster.varWireWrite(1, lowByte(valorSensorUmidadeSolo_2));
//----------------------------------------------------------------
  
  Serial.print(" Sensor de umidade do solo = ");
  Serial.print(valorSensorUmidadeSolo_2);
  if (valorSensorUmidadeSolo_2 < valorLimiteUmidade) {
    Serial.println(" O solo está úmido");
    soloUmido_2 = 1;
  } else {
    Serial.println(" O solo está seco");
    soloUmido_2 = 0;
  }
}
void SensorDeUmidade_3 (){
  int valorSensorUmidadeSolo_3 = analogRead(sensorUmidadeSolo_3);

// ------------------ESP01-8266-------------------------------------
   arduinoMaster.varWireWrite(0, highByte(valorSensorUmidadeSolo_3));
   arduinoMaster.varWireWrite(1, lowByte(valorSensorUmidadeSolo_3));
//----------------------------------------------------------------

  
  Serial.print(" Sensor de umidade do solo = ");
  Serial.print(valorSensorUmidadeSolo_3);
  if (valorSensorUmidadeSolo_3 < valorLimiteUmidade) {
    Serial.println(" O solo está úmido");
    soloUmido_3 = 1;
  } else {
    Serial.println(" O solo está seco");
    soloUmido_3 = 0;
  }
}

void SensorDeUmidade_4 (){
  int valorSensorUmidadeSolo_4 = analogRead(sensorUmidadeSolo_4);

// ------------------ESP01-8266-------------------------------------
   arduinoMaster.varWireWrite(0, highByte(valorSensorUmidadeSolo_4));
   arduinoMaster.varWireWrite(1, lowByte(valorSensorUmidadeSolo_4));
//----------------------------------------------------------------

  
  Serial.print(" Sensor de umidade do solo = ");
  Serial.print(valorSensorUmidadeSolo_4);
  if (valorSensorUmidadeSolo_4 < valorLimiteUmidade) {
    Serial.println(" O solo está úmido");
    soloUmido_4 = 1;
  } else {
    Serial.println(" O solo está seco");
    soloUmido_4 = 0;
  }
}

// -------------------------------------------------------------------------------


// -------------------------------------------- FUNÇÃO DHT22 -------------------------------
void SensorDHT22 (){ 
  delay(delayMS);
  sensors_event_t event;
  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)){
    Serial.println("Erro na leitura da Temperatura!");
  } else {
    Serial.print("Temperatura: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }

  float valorTemperaturaAmbiente = event.temperature;
  int valorTemperaturaAmbiente_feed = round(valorTemperaturaAmbiente);

   arduinoMaster.varWireWrite(0, highByte(valorTemperaturaAmbiente_feed));
   arduinoMaster.varWireWrite(1, lowByte(valorTemperaturaAmbiente_feed));
  
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)){
    Serial.println("Erro na leitura da Umidade!");
  } else {
    Serial.print("Umidade: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }

  float valorUmidadeAr = event.relative_humidity;

  int valorUmidadeAr_feed = round(valorUmidadeAr);
  
    
  arduinoMaster.varWireWrite(0, highByte(valorUmidadeAr_feed));
   arduinoMaster.varWireWrite(1, lowByte(valorUmidadeAr_feed));

  if (valorUmidadeAr > valorLimiteUmidadeAr) {
    Serial.println(" => Clima úmido (possível chuva)");
    climaUmido = 1;
  } else {
    Serial.println(" => Clima seco");
    climaUmido = 0;
  }
  if (valorTemperaturaAmbiente > valorLimiteTemperatura) {
    Serial.println(" => Temperatura muito alta");
    temperaturaAlta = 1;
  } else {
    Serial.println(" => Temperatura ideal");
    temperaturaAlta = 0;
  }


}
// ------------------------------------------------------------------------------------------------


/*

// ------------------------------------------------------- FUNÇÃO SENSOR LDR-----------------------------------
void SensorLDR (){
  leitura = analogRead(AnalogLDR);                // leitura da tensão no pino analogico A0 -- SENSOR LDR --
  voltageLDR = leitura * (5.0/1024);              // cálculo da tensão no LDR -- SENSOR LDR --
  Serial.print("Leitura sensor LDR = ");          // imprime no monitor serial -- SENSOR LDR --
  Serial.println(voltageLDR);                     // imprime a tensão do LDR -- SENSOR LDR --

  if (voltageLDR < Limiar){                        // se a tensão do LDR maior que Limiar -- SENSOR LDR --
    digitalWrite(ledPin, HIGH);                   // liga o LED com 5V -- SENSOR LDR --
    luminosidade = 1;                             // indica que a luminosidade é adequada para regar
  } else {                                            // senão -- SENSOR LDR --
    digitalWrite(ledPin, LOW);                    // desliga LED com 0V -- SENSOR LDR --
    luminosidade = 0;
  }
    // delay(10000);                                     // atraso de 500 milisegundos -- SENSOR LDR --
}
// ----------------------------------------------------------------------------------------------------------------
  
*/

void loop() {
  
  SensorDHT22 ();
//  SensorLDR ();
  SensorDeUmidade_1 ();
  SensorDeUmidade_2 ();
  SensorDeUmidade_3 ();
  SensorDeUmidade_4 ();

// -----------------------------------------------------------

   
  if (climaUmido == 0 && temperaturaAlta == 0 && soloUmido_1 == 0) {              //&& luminosidade == 1 FOI RETIRADO ESSE PARAMETRO DE TODOS OS IFS
    Serial.println("Irrigando o setor ...");
    digitalWrite(portaRele_12, LOW);
    delay(tempoIrrig);
    digitalWrite(portaRele_12, HIGH);
  } else {
    Serial.println("Setor irrigado ...");
  }

  if (climaUmido == 0 && temperaturaAlta == 0 && soloUmido_2 == 0) {
    Serial.println("Irrigando o setor ...");
    digitalWrite(portaRele_7, LOW);
    delay(tempoIrrig);
    digitalWrite(portaRele_7, HIGH);
  } else {
    Serial.println("Setor irrigado ...");
  }

  if (climaUmido == 0 && temperaturaAlta == 0 && soloUmido_3 == 0) {
    Serial.println("Irrigando o setor ...");
    digitalWrite(portaRele_3, LOW);
    delay(tempoIrrig);
    digitalWrite(portaRele_3, HIGH);
  } else {
    Serial.println("Setor irrigado ...");
  }

  if (climaUmido == 0 && temperaturaAlta == 0 && soloUmido_4 == 0) {
    Serial.println("Irrigando o setor ...");
    digitalWrite(portaRele_4, LOW);
    delay(tempoIrrig);
    digitalWrite(portaRele_4, HIGH);
  } else {
    Serial.println("Setor irrigado ...");
  }
  delay(30000);
}

// ---------------ESP01-8266 ----------------------------

void receberDados() {
  arduinoMaster.receiveData(); 
}

void enviarDados() {
  arduinoMaster.sendData(); 
}
