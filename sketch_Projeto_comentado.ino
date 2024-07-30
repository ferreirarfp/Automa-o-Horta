// ----------------------------- SENSOR UMIDADE DO SOLO ----------------------------------------------------------
// Define o intervalo de tempo para irrigação em milissegundos
int tempoIrrig = 10000;

// Define os pinos analógicos para os sensores de umidade do solo
int sensorUmidadeSolo_1 = A1;
int sensorUmidadeSolo_2 = A2;
int sensorUmidadeSolo_3 = A3;
int sensorUmidadeSolo_4 = A4;

// Define os pinos digitais para os relés que controlam as bombas
int portaRele_12 = 12;
int portaRele_7 = 7;
int portaRele_3 = 3;
int portaRele_4 = 4;

// Define o valor limite para a umidade do solo
int valorLimiteUmidade = 500;

// Variáveis booleanas para verificar a umidade do solo em cada setor
bool soloUmido_1;
bool soloUmido_2;
bool soloUmido_3;
bool soloUmido_4;
// ----------------------------------------------------------------------------------------------------------

// ----------------------------- SENSOR DHT22 ----------------------------------------------------------
// Inclui as bibliotecas necessárias para o sensor DHT22
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Define o tipo de sensor DHT como DHT22
#define DHTTYPE     DHT22

// Define o pino digital onde o sensor DHT22 está conectado
#define DHTPIN 2
DHT_Unified dht(DHTPIN, DHTTYPE); // Cria um objeto para o sensor DHT22
uint32_t delayMS = 500; // Tempo de atraso para leitura do sensor DHT22 em milissegundos
float valorLimiteUmidadeAr = 85; // Limite de umidade do ar para identificar clima úmido
float valorLimiteTemperatura = 38; // Limite de temperatura para identificar clima quente
bool climaUmido; // Variável booleana para verificar se o clima está úmido
bool temperaturaAlta; // Variável booleana para verificar se a temperatura está alta
// ------------------------------------------------------------------------------------------------------

// ------------------------------- SENSOR LDR -----------------------------------------------------------
#define AnalogLDR A0 // Define o pino analógico onde o sensor LDR está conectado
#define Limiar 2.0 // Valor de limiar para a radiação solar (obtido a partir de testes)
#define ledPin 13 // Define o pino digital para o LED

int leitura = 0; // Variável para armazenar a leitura do sensor LDR
float voltageLDR; // Variável para armazenar a tensão medida pelo LDR
bool luminosidade; // Variável booleana para verificar se a luminosidade é adequada
// -------------------------------------------------------------------------------------------------------

void setup() {
  // ----------------------------- SENSOR UMIDADE DO SOLO ----------------------------------------------------------
  pinMode(sensorUmidadeSolo_1, INPUT); // Configura o pino do sensor de umidade do solo como entrada
  pinMode(sensorUmidadeSolo_2, INPUT);
  pinMode(sensorUmidadeSolo_3, INPUT);
  pinMode(sensorUmidadeSolo_4, INPUT);
  pinMode(portaRele_12, OUTPUT); // Configura o pino do relé como saída
  pinMode(portaRele_7, OUTPUT);
  pinMode(portaRele_3, OUTPUT);
  pinMode(portaRele_4, OUTPUT);
  digitalWrite(portaRele_12, HIGH); // Desliga a bomba conectada ao pino 12
  digitalWrite(portaRele_7, HIGH); // Desliga a bomba conectada ao pino 7
  digitalWrite(portaRele_3, HIGH); // Desliga a bomba conectada ao pino 3
  digitalWrite(portaRele_4, HIGH); // Desliga a bomba conectada ao pino 4
  Serial.begin(9600); // Inicia a comunicação serial com a taxa de 9600 bps
  // ---------------------------------------------------------------------------------------------------------

  // ----------------------------- SENSOR DHT22 ----------------------------------------------------------
  Serial.begin(9600); // Inicia a comunicação serial para o sensor DHT22
  dht.begin(); // Inicializa o sensor DHT22
  Serial.println("Usando o Sensor DHT"); // Mensagem inicial no monitor serial
  sensor_t sensor; // Cria um objeto para o sensor
  // -----------------------------------------------------------------------------------------------------

  // ------------------------------- SENSOR LDR -----------------------------------------------------------
  pinMode(ledPin, OUTPUT); // Configura o pino do LED como saída
  Serial.begin(9600); // Inicia a comunicação serial para o sensor LDR
  delay(100); // Atraso de 100 milissegundos para estabilização
  // ------------------------------------------------------------------------------------------------------
}

// FUNÇÃO DO SENSOR DE UMIDADE DO SOLO-----------------------------------------------
void SensorDeUmidade_1 () {
  int valorSensorUmidadeSolo_1 = analogRead(sensorUmidadeSolo_1); // Lê o valor do sensor de umidade do solo no setor 1
  Serial.print("Setor 1 - Sensor de umidade do solo = "); // Imprime o valor lido no monitor serial
  Serial.print(valorSensorUmidadeSolo_1);
  if (valorSensorUmidadeSolo_1 < valorLimiteUmidade) { // Compara o valor lido com o limite
    Serial.println("Setor 1 => O solo está úmido"); // Se o solo estiver úmido
    soloUmido_1 = 1; // Atualiza a variável booleana para indicar solo úmido
  } else {
    Serial.println("Setor 1 => O solo está seco"); // Se o solo estiver seco
    soloUmido_1 = 0; // Atualiza a variável booleana para indicar solo seco
  }
}

void SensorDeUmidade_2 () {
  int valorSensorUmidadeSolo_2 = analogRead(sensorUmidadeSolo_2); // Lê o valor do sensor de umidade do solo no setor 2
  Serial.print("Setor 2 - Sensor de umidade do solo = "); // Imprime o valor lido no monitor serial
  Serial.print(valorSensorUmidadeSolo_2);
  if (valorSensorUmidadeSolo_2 < valorLimiteUmidade) { // Compara o valor lido com o limite
    Serial.println("Setor 2 => O solo está úmido"); // Se o solo estiver úmido
    soloUmido_2 = 1; // Atualiza a variável booleana para indicar solo úmido
  } else {
    Serial.println("Setor 2 => O solo está seco"); // Se o solo estiver seco
    soloUmido_2 = 0; // Atualiza a variável booleana para indicar solo seco
  }
}

void SensorDeUmidade_3 () {
  int valorSensorUmidadeSolo_3 = analogRead(sensorUmidadeSolo_3); // Lê o valor do sensor de umidade do solo no setor 3
  Serial.print("Setor 3 - Sensor de umidade do solo = "); // Imprime o valor lido no monitor serial
  Serial.print(valorSensorUmidadeSolo_3);
  if (valorSensorUmidadeSolo_3 < valorLimiteUmidade) { // Compara o valor lido com o limite
    Serial.println("Setor 3 => O solo está úmido"); // Se o solo estiver úmido
    soloUmido_3 = 1; // Atualiza a variável booleana para indicar solo úmido
  } else {
    Serial.println("Setor 3 => O solo está seco"); // Se o solo estiver seco
    soloUmido_3 = 0; // Atualiza a variável booleana para indicar solo seco
  }
}

void SensorDeUmidade_4 () {
  int valorSensorUmidadeSolo_4 = analogRead(sensorUmidadeSolo_4); // Lê o valor do sensor de umidade do solo no setor 4
  Serial.print("Setor 4 - Sensor de umidade do solo = "); // Imprime o valor lido no monitor serial
  Serial.print(valorSensorUmidadeSolo_4);
  if (valorSensorUmidadeSolo_4 < valorLimiteUmidade) { // Compara o valor lido com o limite
    Serial.println("Setor 4 => O solo está úmido"); // Se o solo estiver úmido
    soloUmido_4 = 1; // Atualiza a variável booleana para indicar solo úmido
  } else {
    Serial.println("Setor 4 => O solo está seco"); // Se o solo estiver seco
    soloUmido_4 = 0; // Atualiza a variável booleana para indicar solo seco
  }
}
// -------------------------------------------------------------------------------

// -------------------------------------------- FUNÇÃO DHT22 -------------------------------
void SensorDHT22 () {
  delay(delayMS); // Atraso para estabilizar o sensor DHT22
  sensors_event_t event; // Cria um objeto para armazenar os eventos do sensor

  dht.temperature().getEvent(&event); // Obtém o evento de temperatura
  if (isnan(event.temperature)) { // Verifica se a leitura falhou
    Serial.println("Erro na leitura da Temperatura!"); // Mensagem de erro
  } else {
    Serial.print("Temperatura ambiente: "); // Imprime a temperatura no monitor serial
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  float valorTemperaturaAmbiente = event.temperature; // Armazena o valor da temperatura

  dht.humidity().getEvent(&event); // Obtém o evento de umidade
  if (isnan(event.relative_humidity)) { // Verifica se a leitura falhou
    Serial.println("Erro na leitura da Umidade!"); // Mensagem de erro
  } else {
    Serial.print("Umidade realtiva do ar: "); // Imprime a umidade no monitor serial
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
  float valorUmidadeAr = event.relative_humidity; // Armazena o valor da umidade

  if (valorUmidadeAr > valorLimiteUmidadeAr) { // Compara a umidade com o limite
    Serial.println(" => Clima úmido (possível chuva)"); // Se o clima estiver úmido
    climaUmido = 1; // Atualiza a variável booleana para indicar clima úmido
  } else {
    Serial.println(" => Temperatura ideal"); // Se o clima estiver seco
    climaUmido = 0; // Atualiza a variável booleana para indicar clima seco
  }
  if (valorTemperaturaAmbiente > valorLimiteTemperatura) { // Compara a temperatura com o limite
    Serial.println(" => Temperatura muito alta"); // Se a temperatura estiver alta
    temperaturaAlta = 1; // Atualiza a variável booleana para indicar temperatura alta
  } else {
    Serial.println(" => Temperatura ideal"); // Se a temperatura estiver ideal
    temperaturaAlta = 0; // Atualiza a variável booleana para indicar temperatura ideal
  }
}
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------- FUNÇÃO SENSOR LDR-----------------------------------
void SensorLDR () {
  leitura = analogRead(AnalogLDR); // Lê o valor do sensor LDR no pino analógico A0
  voltageLDR = leitura * (5.0 / 1024); // Converte a leitura para a tensão correspondente
  Serial.print("Leitura sensor LDR = "); // Imprime a leitura do sensor LDR no monitor serial
  Serial.println(voltageLDR); // Imprime a tensão do LDR

  if (voltageLDR > Limiar) { // Verifica se a tensão do LDR é maior que o limiar
    digitalWrite(ledPin, HIGH); // Liga o LED para indicar que a luminosidade é adequada
    luminosidade = 1; // Atualiza a variável booleana para indicar luminosidade adequada
  } else {
    digitalWrite(ledPin, LOW); // Desliga o LED se a luminosidade não for adequada
    luminosidade = 0; // Atualiza a variável booleana para indicar luminosidade inadequada
  }
  // delay(10000); // Comentado: atraso de 10 segundos (não utilizado)
}
// ----------------------------------------------------------------------------------------------------------------

void loop() {

  SensorDHT22 (); // Chama a função para ler o sensor DHT22
  SensorLDR (); // Chama a função para ler o sensor LDR
  SensorDeUmidade_1 (); // Chama a função para ler o sensor de umidade do solo no setor 1
  SensorDeUmidade_2 (); // Chama a função para ler o sensor de umidade do solo no setor 2
  SensorDeUmidade_3 (); // Chama a função para ler o sensor de umidade do solo no setor 3
  SensorDeUmidade_4 (); // Chama a função para ler o sensor de umidade do solo no setor 4

  // Verifica se as condições são adequadas para irrigar o setor 1
  if (climaUmido == 0 && luminosidade == 1 && temperaturaAlta == 0 && soloUmido_1 == 0) {
    Serial.println("Irrigando o setor 1..."); // Mensagem de início da irrigação
    digitalWrite(portaRele_12, LOW); // Liga a bomba conectada ao pino 12
    delay(tempoIrrig); // Espera pelo tempo definido para irrigação
    digitalWrite(portaRele_12, HIGH); // Desliga a bomba
  } else {
    Serial.println("Setor 1 irrigado!"); // Mensagem indicando que o setor 1 foi irrigado
  }

  // Verifica se as condições são adequadas para irrigar o setor 2
  if (climaUmido == 0 && luminosidade == 1 && temperaturaAlta == 0 && soloUmido_2 == 0) {
    Serial.println("Irrigando o setor 2..."); // Mensagem de início da irrigação
    digitalWrite(portaRele_7, LOW); // Liga a bomba conectada ao pino 7
    delay(tempoIrrig); // Espera pelo tempo definido para irrigação
    digitalWrite(portaRele_7, HIGH); // Desliga a bomba
  } else {
    Serial.println("Setor 2 irrigado!"); // Mensagem indicando que o setor 2 foi irrigado
  }

  // Verifica se as condições são adequadas para irrigar o setor 3
  if (climaUmido == 0 && luminosidade == 1 && temperaturaAlta == 0 && soloUmido_3 == 0) {
    Serial.println("Irrigando o setor 3..."); // Mensagem de início da irrigação
    digitalWrite(portaRele_3, LOW); // Liga a bomba conectada ao pino 3
    delay(tempoIrrig); // Espera pelo tempo definido para irrigação
    digitalWrite(portaRele_3, HIGH); // Desliga a bomba
  } else {
    Serial.println("Setor 3 irrigado!"); // Mensagem indicando que o setor 3 foi irrigado
  }

  // Verifica se as condições são adequadas para irrigar o setor 4
  if (climaUmido == 0 && luminosidade == 1 && temperaturaAlta == 0 && soloUmido_4 == 0) {
    Serial.println("Irrigando o setor 4..."); // Mensagem de início da irrigação
    digitalWrite(portaRele_4, LOW); // Liga a bomba conectada ao pino 4
    delay(tempoIrrig); // Espera pelo tempo definido para irrigação
    digitalWrite(portaRele_4, HIGH); // Desliga a bomba
  } else {
    Serial.println("Setor 4 irrigado!"); // Mensagem indicando que o setor 4 foi irrigado
  }
  delay(30000); // Atraso de 30 segundos antes de repetir o loop
}
