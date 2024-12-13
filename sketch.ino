#include <WiFi.h>
#include <HTTPClient.h>

#define led_azul 9 // Pino utilizado para controle do led azul
int led_verde = 39; // Pino utilizado para controle do led verde
int led_vermelho = 47; // Pino utilizado para controle do led vermelho
int led_amarelo = 9; // Pino utilizado para controle do led azul

const int pinoBotao = 15;  // Pino utilizado para controle do Botão
int estadoBotao = 0;  // Variavel para armazenar o estado do botão

const int pinoLdr = 4;  // Pino utilizado para leitura do LDR
int threshold = 600; //Definição de baixa luminosidade

const char* ssid = "Wokwi-GUEST";
const char* senha = "";

long tempoAnterior = 0; 
long tempo = 1000;      

WiFiClient espClient;

// Variáveis globais
unsigned long ultimaLeituraLdr = 0;
const unsigned long intervaloLdr = 1000; // 1 segundo
int ldrStatus = 0;

// Classe Semaforo
class Semaforo {
private:
  int *led_vermelho, *led_amarelo, *led_verde; // Ponteiros para os pinos dos LEDs
public:
  // Construtor da classe
  Semaforo(int *vermelho, int *amarelo, int *verde) 
    : led_vermelho(vermelho), led_amarelo(amarelo), led_verde(verde) {}

  // Inicialização dos LEDs e do LCD
  void iniciar() {
    pinMode(*led_vermelho, OUTPUT);
    pinMode(*led_amarelo, OUTPUT);
    pinMode(*led_verde, OUTPUT);

  }

  // Método para ativar a fase vermelha
  void faseVermelho(int tempo) {
    ligar(*led_vermelho);
    desligar(*led_amarelo);
    desligar(*led_verde);
    delay(tempo);
  }

  // Método para ativar a fase amarela
  void faseAmarelo(int tempo) {
    desligar(*led_vermelho);
    ligar(*led_amarelo);
    desligar(*led_verde);
    delay(tempo);
  }

  // Método para ativar a fase verde
  void faseVerde(int tempo) {
    desligar(*led_vermelho);
    desligar(*led_amarelo);
    ligar(*led_verde);
    delay(tempo);
  }

private:
  // Liga o LED
  void ligar(int pino) {
    digitalWrite(pino, HIGH);
  }

  // Desliga o LED
  void desligar(int pino) {
    digitalWrite(pino, LOW);
  }
};

Semaforo semaforo(&led_vermelho, &led_amarelo, &led_verde);


void setup() {
  // Inicializa o LDR como entrada (Input)
  pinMode(pinoLdr, INPUT);

  // Inicialização das entradas
  pinMode(pinoBotao, INPUT_PULLUP); // Inicializa o botão como valor de entrada (input)

  Serial.begin(9600); // Configuração para debug por interface serial entre ESP e computador com baud rate de 9600

  WiFi.begin(ssid, senha); // Conexão à rede WiFi aberta com SSID Wokwi-GUEST

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado ao WiFi com sucesso!"); // Considerando que saiu do loop acima, o ESP32 agora está conectado ao WiFi (outra opção é colocar este comando dentro do if abaixo)

  // Verifica estado do botão
  estadoBotao = digitalRead(pinoBotao);

  if(WiFi.status() == WL_CONNECTED){ // Se o ESP32 estiver conectado à Internet
    HTTPClient http;

    String serverPath = "http://www.google.com.br"; // Endpoint da requisição HTTP

    http.begin(serverPath.c_str());

    int httpResponseCode = http.GET(); // Código do Resultado da Requisição HTTP

    if (httpResponseCode>0) {
      Serial.print("HTTP codigo de Resposta: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
      }
    else {
      Serial.print("Codigo de Erro: ");
      Serial.println(httpResponseCode);
      }
      http.end();
    }

  else {
    Serial.println("WiFi Desconectado");
  }

  semaforo.iniciar();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - ultimaLeituraLdr >= intervaloLdr) {
    ultimaLeituraLdr = currentMillis;
    ldrStatus = analogRead(pinoLdr);
    Serial.print("LDR Value: ");
    Serial.println(ldrStatus);

    // Controle dos LEDs baseado no valor do LDR
    if (ldrStatus <= threshold) {
      Serial.print("Está escuro ligue a luz");
      semaforo.faseAmarelo(1000);
      digitalWrite(led_amarelo, LOW);
    } else {
        if(estadoBotao == HIGH && led_vermelho == HIGH){
          if ((millis() >= tempoAnterior + tempo)) {
            delay(1000);
            semaforo.faseVerde(3000);
            tempoAnterior = millis();
          }
        }
      Serial.print("Está claro desligue a Luz");
      semaforo.faseVermelho(5000);       // 5 segundos no vermelho
      semaforo.faseAmarelo(2000); // 2 segundos no amarelo
      semaforo.faseVerde(3000); // 2 segundos no verde
    }
  }
}