/* 
  Projeto.: Incubadora.
  Autor...: Icaro Alencar de Oliveira.
  Curso...: Eng. Computação.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include "DHT.h"
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Credenciais WiFi
#define WIFI_SSID     "Virus"     // "ALENCAR_2G"                        
#define WIFI_PASSWORD "12345678"  //"99429048Ii@"     

// Firebase API Key
#define API_KEY "AIzaSyAk3TsV_xfr-k8F2t1Nhs3eOKI0CIzV77U"
// Firebase URL Database */
#define DATABASE_URL "https://incubadora-61c3c-default-rtdb.firebaseio.com" 

// ****************** Sensores ******************
#define DHT11PIN  5           // GPIO aonde esta conectado.
#define DHT11TYPE DHT11       // DHT 11.
DHT dht11(DHT11PIN, DHT11TYPE);

#define DHT22PIN  18               // GPIO aonde esta conectado.
#define DHT22TYPE DHT22            // DHT 22.
DHT dht22(DHT22PIN, DHT22TYPE);
 
#define POWER_PIN  15     // Pino de energia do sensor
#define SIGNAL_PIN 34     // Pino de sinal do nivel da agua
#define SENSOR_MIN 2000   // Valor minimo
#define SENSOR_MAX 2900   // Valro maximo

#define COOLERPIN 35  // COOLER

// Parametros PWM / PID
#include <PID_v1_bc.h>
#define pwmPin 26
int pwmValor = 0;
double TempDesejada = 37.7;
double Input = 0, Output = 0;
//double Kp = 2, Ki = 1, Kd = 0.5;
double Kp = 2, Ki = 0.4, Kd = 0.2;
PID myPID(&Input, &Output, &TempDesejada, Kp, Ki, Kd, DIRECT);

// Sensor DS18B20
const int oneWireBus = 4;             // GPIO aonde esta conectado 
OneWire oneWire(oneWireBus);          // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature sensor 
// *********************************************

// Display LCD 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Firebase Banco de dados
FirebaseData db;
// Firebase Autenticacao
FirebaseAuth auth;
// Configuração Firebase
FirebaseConfig config;

WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP);


// COOLER
volatile uint32_t pulso = 0;
unsigned long tempoAtual = 0, tempoAnterior = 0;
unsigned long tempoAtuaB20 = 0, tempoAnteriorB20 = 0;
unsigned long tempoAtuaT11 = 0, tempoAnteriorT11 = 0;
unsigned long tempoAtuaT22 = 0, tempoAnteriorT22 = 0;
int rpm = 0;


unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK;

float temp_B20_C, temp_B20_C_Aux, temp_B20_F, temp_B20_F_Aux;
float temp_T11_C, temp_T11_C_Aux, temp_T11_F, temp_T11_F_Aux;
float temp_T22_C, temp_T22_C_Aux, temp_T22_F, temp_T22_F_Aux, umid_T22, umid_T22_Aux;;
float umid_T11, umid_T11_Aux;

int valor   = 0;
int nivel, nivel_aux;

String nivelStr, nivelStrAux;

bool err_B20, err_T11, err_T22;
bool InfoGerais;

String dia_mes_ano, hr_min_seg;

void conectWifi(){

  int cont = 0;

  // Conectando ao WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //Serial.print("Conectando com o Wi-Fi");
  lcd.clear();
  lcd.print("CONECTANDO");
  while (WiFi.status() != WL_CONNECTED){
    //Serial.print(".");

    if(cont == 6){
      lcd.setCursor(0,1);
    } 

    if(cont == 22){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CONECTANDO");
      cont = 0;
    }

    lcd.print(".");
    cont++;
    delay(500);
  }

  lcd.clear();
  lcd.print("CONECTADO AO IP");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(1000);

}

void conectFirebase(){
  // Conectando ao Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Serial.print("Conectando ao Firebase.: ");
  lcd.clear();
  lcd.print("CONECTANDO AO ");
  lcd.setCursor(0, 1);
  lcd.print("BANCO:");
  lcd.setCursor(6, 1);

  /* Sign up */
  while (!Firebase.signUp(&config, &auth, "", "")){
    // Serial.println("FALHOU");
    lcd.setCursor(0, 1);
    lcd.print("BANCO: FALHOU");
    signupOK = false;
    delay(500);
  }
  
  //Serial.printf("%s\n", config.signer.signupError.message.c_str());
  signupOK = true;
  lcd.setCursor(6, 1);
  lcd.print("OK");
  // Serial.print("OK");

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  delay(1000);
}

void inicializarSensores(){

  // Serial.println("Iniciando os Sensores de Temperatura");
  lcd.clear();
  lcd.print("Iniciando os"); 
  lcd.setCursor(0, 1);
  lcd.print("Sensores");
  sensors.begin();    // DS18B20 sensor
  dht11.begin();      // DHT11
  dht22.begin();      // DHT22

}

void setup(){
  Serial.begin(9600);

  // Inicializando LCD
  lcd.init();
  lcd.backlight();

  lcd.print("INICIALIZANDO...");
  delay(1000);

  conectWifi();
  conectFirebase();
  inicializarSensores();

  // Valindando se os sensores estao funcionando
  // if(!dht11.read() || !dht22.read() || !sensors.isConnected(&oneWire)){
  // }
  
  ntp.begin();                 // Data e hora
  ntp.setTimeOffset(-14400);  // -4 = -14400 (BRASIL-CUIABA) -3 10800 (BRASIL-BRASILIA)

  // Sensor do nivel de agua
  pinMode(POWER_PIN, OUTPUT);   
  digitalWrite(POWER_PIN, LOW); 

  // COOLER
  pinMode(COOLERPIN, INPUT);
  // Habilita a interrupção
  attachInterrupt(COOLERPIN, ctt, RISING);

  InfoGerais = false;

  delay(500);
  lcd.clear();
}


void loop(){

  if(!WiFi.isConnected()){
    conectWifi();
  }

  if(!Firebase.ready() && WiFi.isConnected()){
    conectFirebase();
  }

  BuscaTemperatura();
  NivelAgua();
  BuscaDataHora();
  
  // Erro na leitura do sensor de temperatura
  err_B20 = false;
  if ((temp_B20_C > 125 || temp_B20_C < -55) && !err_B20)  {
    // Serial.println(F("Verifique o sensor DS18B20"));    
    //GravarLog("ERROS/LEITURA/SENSORES", "DS18B20", 0, 0, 0); 
    err_B20 = true;
  }

  err_T11 = false;
  if((isnan(temp_T11_C) || isnan(temp_T11_F)) && !err_T11){
    // Serial.println(F("Verifique o sensor DHT11"));    
    //GravarLog("ERROS/LEITURA/SENSORES", "DHT11", 0, 0, 0);
    err_T11 = true; 
  }

  // Calculo do PID e atualizaçao do PWM
  Input = temp_T22_C;
  myPID.Compute();
  pwmValor = (int)Output;
  analogWrite(pwmPin, pwmValor);

  SensorDS18B20();
  SensorDHT11();
  SensorDHT22();
  Cooler();

  if(nivelStr != nivelStrAux && Firebase.ready() && signupOK){
    nivelStrAux = nivelStr;
    GravarSensores("WATER", 0, 0, 0);
  }

  if(!InfoGerais){
    GravaInfoGerais("CHOCADEIRA", dia_mes_ano, 37.7, 60, 21);
    InfoGerais = true;
  }
  

}

void BuscaTemperatura(){
  sensors.requestTemperatures(); 
  temp_B20_C  = sensors.getTempCByIndex(0);
  temp_B20_F  = sensors.getTempFByIndex(0);

  temp_T11_C  = dht11.readTemperature();
  temp_T11_F  = dht11.readTemperature(true);
  umid_T11    = dht11.readHumidity();

  temp_T22_C  = dht22.readTemperature();
  temp_T22_F  = dht22.readTemperature(true);
  umid_T22    = dht22.readHumidity();
}

void BuscaDataHora(){
  if (ntp.update()) {
    dia_mes_ano = ntp.getDataFormatada("-");
    hr_min_seg = ntp.getFormattedTime();
  } else {
    // Serial.println("!Erro ao atualizar NTP!\n");
  }
}

bool GravarLog(String atividade, String modelo, float valor1, float valor2, float valor3){
  bool err = false;
  if(atividade == "LEITURA/SENSORES"){
    err = (Firebase.RTDB.setFloat(&db, "INCUBADORA/LOGS/"+dia_mes_ano+"/"+atividade+"/"+modelo+"/"+hr_min_seg+"/C", valor1)
          && Firebase.RTDB.setFloat(&db, "INCUBADORA/LOGS/"+dia_mes_ano+"/"+atividade+"/"+modelo+"/"+hr_min_seg+"/F", valor2) ? false : true);

    if((modelo == "DHT11" || modelo == "DHT22") && !err){
      err = (Firebase.RTDB.setFloat(&db, "INCUBADORA/LOGS/"+dia_mes_ano+"/"+atividade+"/"+modelo+"/"+hr_min_seg+"/U", valor3) ? false : true);
    }
  }

  if(atividade == "ERROS/LEITURA/SENSORES"){
    err = (Firebase.RTDB.setString(&db, "INCUBADORA/LOGS/"+dia_mes_ano+"/"+atividade+"/"+modelo+"/"+hr_min_seg+"/TIPO", "Erro de Conexão")
          && Firebase.RTDB.setString(&db, "INCUBADORA/LOGS/"+dia_mes_ano+"/"+atividade+"/"+modelo+"/"+hr_min_seg+"/OBS", "Verifique o sensor") 
          ? false : true);

    if(!err){
      GravarSensores(modelo, -500, -500, -500);
    }
  }


  return err;
}

bool GravarSensores(String modelo, float celsius, float fahren, float umidade){
  bool err = false;

  if(modelo != "WATER"){
    err = (Firebase.RTDB.setFloat(&db, "INCUBADORA/SENSORES/"+modelo+"/C", celsius) 
          && Firebase.RTDB.setFloat(&db, "INCUBADORA/SENSORES/"+modelo+"/F", fahren) 
          && Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/"+modelo+"/DATA", dia_mes_ano)
          && Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/"+modelo+"/HORA", hr_min_seg) ? false : true); 
  }else{
    err = (Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/"+modelo+"/NIVEL", nivelStr)
          && Firebase.RTDB.setInt(&db, "INCUBADORA/SENSORES/"+modelo+"/VALOR", valor)
          && Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/"+modelo+"/DATA", dia_mes_ano)
          && Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/"+modelo+"/HORA", hr_min_seg) ? false : true); 
  }
  if((modelo == "DHT11" || modelo == "DHT22") && !err){
    err = (Firebase.RTDB.setFloat(&db, "INCUBADORA/SENSORES/"+modelo+"/U", umidade)  ? false : true);
  }
  return err;
}

bool GravarHistoricoTemperatura(String modelo, float c, float f, float u){
  bool err = false;

  err = (Firebase.RTDB.setFloat(&db, "INCUBADORA/HISTORICO/"+modelo+"/"+dia_mes_ano+"/"+hr_min_seg+"/C", c)
        && Firebase.RTDB.setFloat(&db, "INCUBADORA/HISTORICO/"+modelo+"/"+dia_mes_ano+"/"+hr_min_seg+"/F", f) ? false : true);
  
  if((modelo == "DHT11" || modelo == "DHT22") && !err){
    err = (Firebase.RTDB.setFloat(&db, "INCUBADORA/HISTORICO/"+modelo+"/"+dia_mes_ano+"/"+hr_min_seg+"/U", u) ? false : true);
  }

  return err;
}

bool GravaInfoGerais(String Nome, String Data_Inicio, float Temp_Ideal, int Umid_Ideal, int Tempo_Incubacao){
  bool err = false;

  err = (Firebase.RTDB.setString(&db, "INCUBADORA/INFO_GERAIS/NOME", Nome)
        && Firebase.RTDB.setString(&db, "INCUBADORA/INFO_GERAIS/DATA_INICIO_INCUBACAO", Data_Inicio)
        && Firebase.RTDB.setFloat(&db, "INCUBADORA/INFO_GERAIS/CONFIG/TEMPERATURA_IDEAL", Temp_Ideal) 
        && Firebase.RTDB.setInt(&db, "INCUBADORA/INFO_GERAIS/CONFIG/UMIDADE_IDEAL", Umid_Ideal) 
        && Firebase.RTDB.setInt(&db, "INCUBADORA/INFO_GERAIS/CONFIG/TEMPO_INCUBACAO", Tempo_Incubacao) ? false : true);


  return err;
}

bool NivelAgua(){
  digitalWrite(POWER_PIN, HIGH);  
  delay(10);                    // 
  valor = analogRead(SIGNAL_PIN); // 
  delay(10);
  digitalWrite(POWER_PIN, LOW);   // 
  digitalWrite(POWER_PIN, LOW);

  nivel = map(valor, SENSOR_MIN, SENSOR_MAX, 0, 3); // Percentual

  switch(nivel){  
    case 1: 
      // Serial.println("Nivel de Agua esta Baixo");
      nivelStr = "BAIXO";
      break;
    
    case 2: 
      // Serial.println("Nivel de Agua esta Medio");
      nivelStr = "MEDIO";
      break;
    
    case 3:
    case 4:
    case 5:
      // Serial.println("Nivel de Agua está Alto");
      nivelStr = "ALTO";
      break;
    
    default:
      // Serial.println("Nivel de Agua esta Vazio");
      nivelStr = "VAZIO";
      break;
  }


  return true;
}

void GravaCooler(){
  bool err = false;
    err = (Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/COOLER/STATUS", rpm > 0 ? "LIGADO" : "DESLIGADO")
        && Firebase.RTDB.setInt(&db, "INCUBADORA/SENSORES/COOLER/RPM", rpm) 
        && Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/COOLER/DATA", dia_mes_ano)
        && Firebase.RTDB.setString(&db, "INCUBADORA/SENSORES/COOLER/HORA", hr_min_seg) ? false : true);
}

void ctt(){
   pulso++;  //Incrementa a váriavel.  
}


void SensorDS18B20(){
  tempoAtuaB20 = millis();
  if((temp_B20_C != temp_B20_C_Aux) && Firebase.ready() && signupOK && !err_B20 && (tempoAtuaB20 - tempoAnteriorB20 >= 2000)){

    GravarSensores("DS18B20", temp_B20_C, temp_B20_F, 0);
    GravarHistoricoTemperatura("DS18B20", temp_B20_C, temp_B20_F, 0);
    //GravarLog("LEITURA/SENSORES", "DS18B20", temp_B20_C, temp_B20_F, 0);
    temp_B20_C_Aux = temp_B20_C;

    // Serial.println("###### TEMPERATURA DAS PATILHAS ########");
    //Serial.println("PATH: " + db.dataPath());
    //Serial.println("TYPE: " + db.dataType());
    // Serial.print("DS18B20_T.: ");
    // Serial.print(temp_B20_C);
    // Serial.println("C");
    // Serial.print("DS18B20_F.: ");
    // Serial.print(temp_B20_F);
    // Serial.println("F");
    //Firebase.RTDB.setBool(&db, "historico/erro", false);

    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("TEMP PASTILHAS");
    lcd.setCursor(4,1);
    lcd.print("T:");
    lcd.setCursor(6,1);
    lcd.print(temp_B20_C);
    lcd.setCursor(11,1);
    lcd.print("C");

    tempoAnteriorB20 = tempoAtuaB20;
  }
}

void SensorDHT11(){
  
  tempoAtuaT11 = millis();
  if((temp_T11_C != temp_T11_C_Aux) && Firebase.ready() && signupOK && !err_T11 && (tempoAtuaT11 - tempoAnteriorT11 >= 2000)){

    GravarSensores("DHT11", temp_T11_C, temp_T11_F, umid_T11);
    GravarHistoricoTemperatura("DHT11", temp_T11_C, temp_T11_F, umid_T11);
    //GravarLog("LEITURA/SENSORES", "DHT11", temp_T11_C, temp_T11_F, umid_T11);
    temp_T11_C_Aux = temp_T11_C;

    // Serial.println("####### TEMPERATURA EXTERNA #######");
    // Serial.print("DHT_T.: ");
    // Serial.print(temp_T11_C);
    // Serial.println("C");
    // Serial.print("DHT_F.: ");
    // Serial.print(temp_T11_F);
    // Serial.println("F");
    // Serial.print("DHT_U: ");
    // Serial.print(umid_T11);
    // Serial.println("%");

    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("TEMP EXTERNA");
    lcd.setCursor(0,1);
    lcd.print("T:");
    lcd.setCursor(2,1);
    lcd.print(temp_T11_C);
    lcd.setCursor(7,1);
    lcd.print("C");

    lcd.setCursor(9,1);
    lcd.print("U:");
    lcd.setCursor(11,1);
    lcd.print(umid_T11);
    lcd.setCursor(15,1);
    lcd.print("%");

    tempoAnteriorT11 = tempoAtuaT11;
  }
}

void SensorDHT22(){

  err_T22 = false;
  if((isnan(temp_T22_C) || isnan(temp_T22_F)) && !err_T22){
    // Serial.println(F("Verifique o sensor DHT2"));    
    //GravarLog("ERROS/LEITURA/SENSORES", "DHT22", 0, 0, 0);
    err_T22 = true; 
  }

  tempoAtuaT22 = millis();

  if((temp_T22_C != temp_T22_C_Aux) && Firebase.ready() && signupOK && !err_T22 && (tempoAtuaT22 - tempoAnteriorT22 >= 2000)){
    
    GravarSensores("DHT22", temp_T22_C, temp_T22_F, umid_T22);
    GravarHistoricoTemperatura("DHT22", temp_T22_C, temp_T22_F, umid_T22);
    //GravarLog("LEITURA/SENSORES", "DHT22", temp_T22_C, temp_T22_F, umid_T22);
    temp_T22_C_Aux = temp_T22_C;

    // Serial.println("####### TEMPERATURA INCUBADORA #######");

    Serial.print("DHT_22_C.: ");
    Serial.println(temp_T22_C);
    // Serial.print("temp_T22_F.: ");
    // Serial.println(temp_T22_F);
    Serial.print("umid_T22.: ");
    Serial.print(umid_T22);
    Serial.println("%");

    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("INCUBADORA");
    lcd.setCursor(0,1);
    lcd.print("T:");
    lcd.setCursor(2,1);
    lcd.print(temp_T22_C);
    lcd.setCursor(7,1);
    lcd.print("C");

    lcd.setCursor(9,1);
    lcd.print("U:");
    lcd.setCursor(11,1);
    lcd.print(umid_T22);
    lcd.setCursor(15,1);
    lcd.print("%");

    tempoAnteriorT22 = tempoAnteriorT22;  
  }
}

void Cooler(){
  // Cooler
  tempoAtual = millis();
  if((tempoAtual - tempoAnterior) >= (60000)){
    // Desabilita interrupção
    detachInterrupt(COOLERPIN);

    // Calculo do RPM.
    rpm = (pulso / 2);

    Serial.print("RPM.: ");
    Serial.println(rpm);
    GravaCooler();
    pulso = 0;
    tempoAnterior = tempoAtual;

   // Habilita a interrupção
   attachInterrupt(COOLERPIN, ctt, RISING);
  }
}

