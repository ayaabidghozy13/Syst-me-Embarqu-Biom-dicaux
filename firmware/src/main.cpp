#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <string.h>
#include <stdlib.h>

// =====================================================
// PINS - ARDUINO UNO R4 WIFI
// =====================================================
const int LAMPE = 9;
const int BOUTON = 2;
const int BUZZER = 3;
const int ECG = A0;
const int LO_PLUS = 5;
const int LO_MOINS = 4;

#define RFID_SS 10
#define RFID_RST 7
MFRC522 rfid(RFID_SS, RFID_RST);

// =====================================================
// WIFI / MQTT
// =====================================================
const char* WIFI_SSID = "VOTRE_SSID";
const char* WIFI_PASSWORD = "VOTRE_MOT_DE_PASSE";
const char* MQTT_SERVER = "broker.emqx.io";
const int MQTT_PORT = 1883;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

const char* TOPIC_ECG = "health/ecg";
const char* TOPIC_RFID = "health/rfid";
const char* TOPIC_BUTTON = "health/button";
const char* TOPIC_STATUS = "health/status";
const char* TOPIC_COMMAND = "health/command";

// =====================================================
// TIMING
// =====================================================
const unsigned long PERIODE_ECG = 20;                // 50 Hz
const unsigned long DEBOUNCE_BOUTON = 50;
const unsigned long DELAI_WIFI = 10000;
const unsigned long DELAI_MQTT = 3000;
const unsigned long ANTI_REPETITION_RFID = 3000;
const unsigned long DELAI_IDENTIFICATION = 60000;    // 60 s pour appuyer après le badge
const unsigned long PERIODE_STATUT = 2000;           // heartbeat vers Node-RED

unsigned long dernierEchantillon = 0;
unsigned long derniereTentativeWiFi = 0;
unsigned long derniereTentativeMQTT = 0;
unsigned long dernierChangementBouton = 0;
unsigned long dernierAppui = 0;
unsigned long dernierBadgeMillis = 0;
unsigned long debutIdentification = 0;
unsigned long dernierStatut = 0;

int dernierBoutonLu = HIGH;
int etatStableBouton = HIGH;

char dernierUID[32] = "";
char uidPatient[32] = "";

bool etatLampe = false;
bool etatBuzzer = false;
int frequenceBuzzer = 600;

// =====================================================
// MACHINE D'ÉTATS
// ATTENTE_BADGE  --badge-->  PATIENT_IDENTIFIE  --bouton-->  ACQUISITION
//       ^                         |  (timeout 60 s)               |
//       +-------------------------+<----------bouton (stop)-------+
// =====================================================
enum EtatSysteme { ATTENTE_BADGE, PATIENT_IDENTIFIE, ACQUISITION };
EtatSysteme etat = ATTENTE_BADGE;

const char* texteEtat() {
  switch (etat) {
    case ATTENTE_BADGE:     return "Attente badge";
    case PATIENT_IDENTIFIE: return "Patient identifie";
    case ACQUISITION:       return "Acquisition en cours";
  }
  return "Inconnu";
}

void publierStatut() {
  if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, texteEtat());
  dernierStatut = millis();
}

void changerEtat(EtatSysteme nouvelEtat) {
  etat = nouvelEtat;
  Serial.print("Etat -> ");
  Serial.println(texteEtat());
  publierStatut();
}

// Bip court sans couper une alarme déjà active.
void bip(int frequence, unsigned long duree) {
  if (!etatBuzzer) tone(BUZZER, frequence, duree);
}

// =====================================================
// WIFI - NON BLOQUANT
// =====================================================
void gererWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long maintenant = millis();
  if (maintenant - derniereTentativeWiFi < DELAI_WIFI) return;

  derniereTentativeWiFi = maintenant;
  Serial.println("Tentative de connexion Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// =====================================================
// COMMANDES NODE-RED
// Format: lamp=1;buzzer=0;frequency=800
// =====================================================
void recevoirCommande(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_COMMAND) != 0) return;

  char message[128];
  if (length == 0 || length >= sizeof(message)) return;

  memcpy(message, payload, length);
  message[length] = '\0';

  char* champLampe = strstr(message, "lamp=");
  char* champBuzzer = strstr(message, "buzzer=");
  char* champFrequence = strstr(message, "frequency=");

  if (champLampe != nullptr) etatLampe = atoi(champLampe + 5) != 0;
  if (champBuzzer != nullptr) etatBuzzer = atoi(champBuzzer + 7) != 0;

  if (champFrequence != nullptr) {
    int f = atoi(champFrequence + 10);
    if (f >= 100 && f <= 5000) frequenceBuzzer = f;
  }

  digitalWrite(LAMPE, etatLampe ? HIGH : LOW);

  if (etatBuzzer) tone(BUZZER, frequenceBuzzer);
  else noTone(BUZZER);

  Serial.print("Commande Node-RED : ");
  Serial.println(message);
}

// =====================================================
// MQTT - NON BLOQUANT (+ Last Will)
// =====================================================
void gererMQTT() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;

  unsigned long maintenant = millis();
  if (maintenant - derniereTentativeMQTT < DELAI_MQTT) return;
  derniereTentativeMQTT = maintenant;

  char clientId[40];
  snprintf(clientId, sizeof(clientId), "UNO-R4-%lu", (unsigned long)millis());

  Serial.print("Connexion MQTT...");
  // Le broker publiera "Arduino deconnecte" si la carte disparaît brutalement.
  if (mqtt.connect(clientId, TOPIC_STATUS, 0, false, "Arduino deconnecte")) {
    Serial.println(" OK");
    mqtt.subscribe(TOPIC_COMMAND);
    publierStatut();
  } else {
    Serial.print(" echec, code = ");
    Serial.println(mqtt.state());
  }
}

// Heartbeat : sans lui, Node-RED afficherait "déconnecté"
// dès qu'aucun échantillon ECG n'est envoyé (avant le badge / après stop).
void envoyerHeartbeat() {
  if (millis() - dernierStatut >= PERIODE_STATUT) publierStatut();
}

// =====================================================
// RFID - IDENTIFICATION DU PATIENT
// =====================================================
void lireRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  char uid[32] = "";
  char octet[4];

  for (byte i = 0; i < rfid.uid.size; i++) {
    snprintf(octet, sizeof(octet), "%02X", rfid.uid.uidByte[i]);
    if (i > 0) strncat(uid, ":", sizeof(uid) - strlen(uid) - 1);
    strncat(uid, octet, sizeof(uid) - strlen(uid) - 1);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // Pendant l'acquisition, on ne change pas de patient.
  if (etat == ACQUISITION) {
    Serial.println("Badge ignore : acquisition en cours");
    return;
  }

  unsigned long maintenant = millis();
  bool nouveauBadge = (strcmp(uid, dernierUID) != 0) ||
                      (maintenant - dernierBadgeMillis >= ANTI_REPETITION_RFID);
  if (!nouveauBadge) return;

  strncpy(dernierUID, uid, sizeof(dernierUID) - 1);
  dernierUID[sizeof(dernierUID) - 1] = '\0';
  strncpy(uidPatient, uid, sizeof(uidPatient) - 1);
  uidPatient[sizeof(uidPatient) - 1] = '\0';
  dernierBadgeMillis = maintenant;
  debutIdentification = maintenant;

  Serial.print("Patient identifie, UID : ");
  Serial.println(uid);

  if (mqtt.connected()) mqtt.publish(TOPIC_RFID, uid);
  bip(1800, 120);

  if (etat != PATIENT_IDENTIFIE) changerEtat(PATIENT_IDENTIFIE);
  else publierStatut();   // re-badge : on relance simplement le délai
}

// =====================================================
// ACTION DU BOUTON SELON L'ÉTAT
// =====================================================
void actionBouton() {
  switch (etat) {
    case ATTENTE_BADGE:
      Serial.println("Appui refuse : badge RFID requis");
      if (mqtt.connected()) mqtt.publish(TOPIC_BUTTON, "refused");
      bip(400, 300);
      break;

    case PATIENT_IDENTIFIE:
      if (!mqtt.connected()) {
        Serial.println("Demarrage refuse : MQTT non connecte");
        bip(400, 300);
        break;
      }
      mqtt.publish(TOPIC_BUTTON, "start");
      dernierEchantillon = millis();
      bip(2200, 80);
      changerEtat(ACQUISITION);
      break;

    case ACQUISITION:
      if (mqtt.connected()) mqtt.publish(TOPIC_BUTTON, "stop");
      uidPatient[0] = '\0';
      bip(1000, 150);
      changerEtat(ATTENTE_BADGE);   // le patient suivant doit se badger
      break;
  }
}

// =====================================================
// BOUTON - ANTI-REBOND
// =====================================================
void lireBouton() {
  int lecture = digitalRead(BOUTON);
  unsigned long maintenant = millis();

  if (lecture != dernierBoutonLu) {
    dernierBoutonLu = lecture;
    dernierChangementBouton = maintenant;
  }

  if (maintenant - dernierChangementBouton >= DEBOUNCE_BOUTON &&
      lecture != etatStableBouton) {
    etatStableBouton = lecture;

    if (etatStableBouton == LOW && maintenant - dernierAppui >= 300) {
      dernierAppui = maintenant;
      Serial.println("Bouton appuye");
      actionBouton();
    }
  }
}

// =====================================================
// DÉLAI D'IDENTIFICATION
// =====================================================
void verifierDelaiIdentification() {
  if (etat != PATIENT_IDENTIFIE) return;
  if (millis() - debutIdentification < DELAI_IDENTIFICATION) return;

  Serial.println("Delai depasse : nouveau badge requis");
  uidPatient[0] = '\0';
  changerEtat(ATTENTE_BADGE);
}

// =====================================================
// ECG - UNIQUEMENT PENDANT L'ACQUISITION
// =====================================================
void envoyerECG() {
  if (etat != ACQUISITION) return;

  unsigned long maintenant = millis();
  if (maintenant - dernierEchantillon < PERIODE_ECG) return;
  dernierEchantillon = maintenant;

  int valeurECG = analogRead(ECG);
  int loPlus = digitalRead(LO_PLUS);
  int loMoins = digitalRead(LO_MOINS);

  char message[100];
  snprintf(message, sizeof(message),
           "{\"raw\":%d,\"lo_plus\":%d,\"lo_moins\":%d,\"time\":%lu}",
           valeurECG, loPlus, loMoins, maintenant);

  if (mqtt.connected()) mqtt.publish(TOPIC_ECG, message);
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  pinMode(LAMPE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BOUTON, INPUT_PULLUP);
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MOINS, INPUT);

  digitalWrite(LAMPE, LOW);
  noTone(BUZZER);

  Serial.begin(9600);
  delay(1000);
  Serial.println("SYSTEME ECG + RFID + MQTT");

  SPI.begin();
  rfid.PCD_Init();
  delay(100);

  Serial.print("Version RC522 : 0x");
  Serial.println(rfid.PCD_ReadRegister(MFRC522::VersionReg), HEX);

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(recevoirCommande);
  mqtt.setBufferSize(256);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(2);

  derniereTentativeWiFi = millis() - DELAI_WIFI;
  gererWiFi();

  Serial.println("En attente d'un badge RFID...");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  gererWiFi();
  gererMQTT();

  if (mqtt.connected()) {
    mqtt.loop();
    envoyerHeartbeat();
  }

  lireRFID();
  lireBouton();
  verifierDelaiIdentification();
  envoyerECG();
}
