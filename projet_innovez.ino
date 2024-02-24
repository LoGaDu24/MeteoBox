#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

// MQ135
// résistance (kOhm) calibrée pour mesurer le CO2 avec le MQ135
const int R0 = 65; //38; // 20 kOhm pour 100ppm
// numéro du GPIO (on prend le pin GPIO34 car les pins de l'ADC2 ne peuvent pas être utilisées quand le wifi est utilisé !)
const int mq135_pin = 34;
/// paramètres pour la correction 
const float CORA = 0.00035;
const float CORB = 0.02718;
const float CORC = 1.39538;
const float CORD = 0.0018;
const float CORE = -0.003333333;
const float CORF = -0.001923077;
const float CORG = 1.130128205;

// wifi
const char* ssid = "freebox_GD";
const char* password = "BaPaLoSiVi";

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

// URL pour envoyer le SMS via free
String serverName =  "https://smsapi.free-mobile.fr/sendmsg?user=97860920&pass=aI8BDPSP2cXt76&msg="; 

void setup() {

  Serial.begin(9600);
delayTime = 3600000; // 1h  

  Wire.begin(21, 22);
  // démarre le BME280
  bool test;
  test = bme.begin(0x76);  
  if (!test) {
    Serial.println("probleme avec le BME280 !");
    while (1);
  }
  
  WiFi.begin(ssid, password);
  Serial.println("Connexion...");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connexion OK. Adresse IP du reseau wifi : ");
  Serial.println(WiFi.localIP());
 
}

void loop() { 
  preparer_envoyer_SMS();
  delay(delayTime);
}

uint16_t getAnalogMean(uint8_t pin, uint8_t n) {
  int sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return sum / n;
}

float mq135_resistance(uint8_t pin) {
  int val = getAnalogMean(pin, 5);
  // RLOAD = 10 kohms (notation 102 sur le mq135)
  // resolution des GPIO est 12 bits donc diviser par 4095 (et pas 1023 comme pour un port analogique arduino)
  // formule pour avoir la résistance d'un capteur : https://electronoobs.com/eng_arduino_tut10.php
return ((4095./(float)val) - 1.)*10.0; 
}

// renvoie les ppm CO2 sans prise en compte de la tempé et humidite de l'air
float mq135_readCO2(uint8_t pin, uint8_t R0) {
  float R = mq135_resistance(pin);
  return 116.6020682 * pow((R/R0), -2.769034857);
}

// correction selon tempé et humidité (https://github.com/ViliusKraujutis/MQ135/blob/master/MQ135.cpp)
float mq135_correction(float t, float h) {
    // Linearization of the temperature dependency curve under and above 20 degree C
    // below 20degC: fact = a * t * t - b * t - (h - 33) * d
    // above 20degC: fact = a * t + b * h + c
    // this assumes a linear dependency on humidity
    if(t < 20){
        return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

float mq135_resistance_corr(uint8_t pin, float t, float h) {
  return mq135_resistance(pin)/mq135_correction(t, h);
}

double calcule_indice_chaleur(double temperatureCelsius, double humidityRelative) {
    // Coefficients de la formule de Steadman
    double c1 = -42.379;
    double c2 = 2.04901523;
    double c3 = 10.14333127;
    double c4 = -0.22475541;
    double c5 = -6.83783e-03;
    double c6 = -5.481717e-02;
    double c7 = 1.22874e-03;
    double c8 = 8.5282e-04;
    double c9 = -1.99e-06;

    // Conversion de la température en degrés Fahrenheit
    double T = (temperatureCelsius * 9.0/5.0) + 32.0;

    // Conversion de l'humidité relative en décimales
    double R = humidityRelative / 100.0;

    // Calcul de l'indice de chaleur
    double heatIndex = c1 + (c2 * T) + (c3 * R) + (c4 * T * R) + (c5 * T * T) + (c6 * R * R) + (c7 * T * T * R) + (c8 * T * R * R) + (c9 * T * T * R * R);

    // Conversion du ICH en degrés Celsius
    double heatIndex_C =  (heatIndex - 32.0) * (5.0/9.0); 

    return heatIndex_C;
}

// renvoie les ppm CO2 avec prise en compte de la tempé et humidite de l'air
float mq135_readCO2_corr(uint8_t pin, uint8_t R0, float t, float h) {
  float R = mq135_resistance_corr(pin, t, h);
  return 116.6020682 * pow((R/R0), -2.769034857);
}

void envoyerSMS(char msg[100]) {
        // envoi du message si connexion OK
      if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;
      
      // URL à connecter
      String serverPath = serverName + msg;
      http.begin(serverPath.c_str());
      
      // Envoie la requête HTTP GET
      int httpResponseCode = http.GET();

      // vérifie que la requête a fonctionné
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
}

void preparer_envoyer_SMS() {

  static char fstr[15];
  
  // lecture BME180
  float BME180_temp = bme.readTemperature();
  float BME180_humi = bme.readHumidity();
  float BME180_pres = bme.readPressure();

  // calcule indice chaleur
  double ICH = calcule_indice_chaleur(BME180_temp,BME180_humi);

  // concentration CO2
  double co2ppm_corr = mq135_readCO2_corr(mq135_pin,R0,BME180_temp,BME180_humi);

  // création du message
  char message[100];
  char BME180_temp_txt[8];
  char BME180_humi_txt[8];
  char BME180_pres_txt[8];
  char ICH_txt[8];
  dtostrf(BME180_temp, 4, 1, BME180_temp_txt);
  dtostrf(BME180_humi, 3, 1, BME180_humi_txt);
  dtostrf(BME180_pres, 6, 0, BME180_pres_txt);
  dtostrf(ICH, 4, 1, ICH_txt);
  
//  sprintf(message,"température : %s humidité : %s ICH : %s", BME180_temp_txt, BME180_humi_txt, ICH_txt);

// envoie un sms en cas de basse pression atmosphérique indiquant une averse possible

  if (BME180_pres <= 101000.0) {
    if ((BME180_pres <=101000.0) && (BME180_pres > 99000.0)) {
        sprintf(message,"Attention : risque de précipitations moyen (70%% environ).");
    }
    else {
        sprintf(message,"Attention : risque de précipitations élevé (80%% au moins).");
    }

    // envoie
    envoyerSMS(message);
    
  }

// on marque une petite pause entre les deux messages possibles
 delay(1000);

// envoie un sms en cas de basse température avec risque de gel

  if (BME180_temp <= 0.0) {
    
        sprintf(message,"Attention : risque de gel.");
        
        // envoie
        envoyerSMS(message);
    
  }

// on marque une petite pause entre les deux messages possibles
 delay(1000);

// envoie un sms en fonction de la valeur de l'ICH
  if (ICH >= 27.0) {
      // prépare le message à envoyer selon l'indice de chaleur
      if ((ICH >= 27.0) && (ICH < 32.0)) {
        sprintf(message,"Attention : la fatigue est possible à la suite d'une activité et d'une exposition prolongées.");
      }
      else if ((ICH >= 32.0) && (ICH < 41.0)) {
        sprintf(message,"Vigilance extrême : des crampes de chaleur et un épuisement par la chaleur sont possibles.");        
      }
      else if ((ICH >= 41.0) && (ICH < 54.0)) {
        sprintf(message, "Danger : les crampes de chaleur et l'épuisement par la chaleur sont probables.");
      }
      else {
        sprintf(message, "Danger extrême : l'hyperthermie est imminente.");
      }

      // envoie
      envoyerSMS(message);
  }

// on marque une petite pause entre les deux messages possibles
 delay(1000);

// envoie un sms en fonction de concentration en C02
  if (co2ppm_corr >= 800) {
    if (co2ppm_corr < 1000) {
      sprintf(message,"CO2 élevé : aérez !");
      }
    else {
      sprintf(message,"CO2 très elevé : attention !");        
      }
   // envoie
   envoyerSMS(message);
  }

  Serial.print("température (°C): ");
  Serial.println(BME180_temp);
  Serial.print("humidité (%) : ");   
  Serial.println(BME180_humi);
  Serial.print("pression (Pa) : ");      
  Serial.println(BME180_pres);
  Serial.print("ICH (°C) : ");
  Serial.println(ICH_txt);
  Serial.print("MQ135 CO2 ppm (non corrigé) :");
  Serial.println(mq135_readCO2(mq135_pin,R0));
  Serial.print("MQ135 CO2 ppm (corrigé) :");
  Serial.println(co2ppm_corr);
  Serial.print("voltage pin 34 (ne doit pas dépasser 3,3V!!) :");
  Serial.println(5.0*(float)getAnalogMean(mq135_pin,10)/4095.0); // 5V max en sortie du MQ
}
