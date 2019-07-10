// ****************************************************************************
// Glacière à chocolat
// Pour maintenanir une température constante et adaptée
// à la conservation du chocolat dans une glacière électrique

// (c) Jojo Bricolo 06/2019
// V 2.1

// V1.0 - 07/2016 - Essai avec Pid (non fonctionnel)
// V2.0 - 02/2019 - Test Thermostat refonte complète du schémas
// V2.1 - 06/2019 - Thermostat et émission RF433 ok

// Affichage de la température sur LCD 2 lignes
// Emission en RF433 de la température
// Capteur de température DS18B20
// (pour Domoticz)
// Compatible Oregon V2.1
// ****************************************************************************



// =================================================
// Le ventilateur n'est jamais coupé, sauf dans le
// cas ou la température de l'environnement est plus
// basse que la consigne
// =================================================



// ****************************************************************************
// connectingStuff, Oregon Scientific v2.1 Emitter
// http://www.connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/

// Copyright (C) 2013 olivier.lebrun@gmail.com
// Modified by Jonathan Martin <therouquinblanc@gmail.com>, August 2015
// ****************************************************************************



// ****************************************************************************
// Connexions:

// Arduino Pro Mini

// Capteur de température DS18B20 Connecté pin A1

// Emetteur RF433 connecté pin 2

// Glacière "OnOff" Pin 3
// La pin 3 commande A LA FOIS l'arrêt complet du Peletier
// et aussi du ventilateur
// Si on ne veut pas arrêter le ventilateur il faut envoyer
// 0 au potentiomètre électronique
// ce qui fera environ 3,2v au Peletier

// LCD 2 lignes 16 colonnes connecté pins 7, 8, 9, 6, 5, 4

// Ne rien connecter en A0 car utilisé pour initialiser le générateur aléatoire

// Protocole SPI utilisé pour le potentiomètre électronique:
// CS pin 10
// SCK pin 13
// MOSI (SI) pin 11

// ***************************************************************************


/*
  Glacière à chocolat

  Le fil souple qui alimente le module Pelletier:
  coté rouge= Masse
  coté sans couleur +

  Fils rouge vert bleu venant de l'ancien potentiomètre:

  Fil vert = masse
  Fil rouge: patte du TL
  Fil bleu: Il va directement à la base d'un transistor NPN 2SC 1831
  La base est polarisée par une résiatance

  En fonctionnement avec le potentiometre mécanique:
  le fil vert et bleu sont en court circuit: La glacière est alimentée
*/



/**********************************************************
  Sketch pour contrôler un potentiomètre numérique
  MCP41100 au moyen d'un Arduino Uno.
  http://electroniqueamateur.blogspot.ca/2013/04/faire-varier-une-resistance-au-moyen-de.html

  Branchements
  MCP 41050
  Ce potentiomètre numérique prend la forme d'un circuit intégré DIP à 8 broches.
  Deux pins servent à alimenter la puce:  la pin 4 se branche dans une des connection "GND" de l'Arduino,
  et la pin 8 se branche dans la sortie 5 V de l'Arduino.

  Trois pins se chargent de la réception des données numérique émises par l'Arduino.
  Comme pour tous les périphériques utilisant le protocole SPI, la pin 1 (chip select)
  se branche de préférence sur la sortie 10 de l'Arduino, la pin 2 (serial clock) sur la sortie 13 de l'Arduino,
  et la pin 3 (données numérique) va dans la sortie MOSI (11) de l'Arduino (les connexions sont différentes si vous utilisez un Arduino Mega).

  Finalement, les trois autres pins du circuit intégré constituent les sorties du potentiomètre:
  la résistance entre les pins 5 et 7 demeurera constante (à la valeur maximale d'environ 100 kΩ),
  mais on pourra varier à volonté la résistance entre la pin 6 (l'équivalent du curseur d'un potentiomètre conventionnel)
  et la pin 5, et la résistance entre la pin 6 et la pin 7.

  la tension maximale absolue est de 7V et le courant maximum de 1mA

  Pour la glacière à chocolat, la tension max est de 2,5V et la mini de 0V car on commande
  un TL 431, c'est donc tout à fait compatible

  Résumé des connexions:

  MCP 41050      Pro mini   Pro micro
  1 (CS)          10          15
  2 (SCK)         13
  3 (MOSI)        11
  4 GND           GND
  5 +5V
  6 Curseur       A1          A1
  7 GND
  8 +5V           5 V

  Programmation

  À la lecture de la fiche technique, on constate que, pour régler la résistance, il faut envoyer au potentiomètre
  un premier message d'un octet indiquant la commande à effectuer.
  Pour la commande "write data" (la seule qui me semble utile), il faut envoyer le nombre binaire
  xx01xx01 dans lequel la valeur des x n'a aucune importance.
  Si on remplace les x par des 0, on envoie donc le nombre binaire
  00010001 qui correspond au nombre hexadécimal 11, ou tout simplement au nombre décimal 17.

  Il faut ensuite envoyer un deuxième message d'un octet:  un nombre entre 0 et 255 qui indique à quelle valeu
  la résistance doit être réglée (0 pour la valeur minimale, 255 pour la valeur maximale).


**********************************************************
  Correspondance résistance (électronique) - tension sur le Peletier:
  NB: le potentiomètre de la glacière est logarithmique
  et la version électronique est linéaire

  255 12,6v
  240 12,6v
  230 9,3v
  220 7,4v
  210 6,3v
  200 5,6v
  170 4,5v
  160 4,3v
  150 4,1v
  140 4,0v
  130 3,9v
  120 3,8v
  110 3,7v
  100 3,6v
  70 3,4v
  50 3.3v
  0 3,2v
**********************************************************/

// les périphériques SPI utiliseront les pins 10(CS/SS), 11 (MOSI/SI), 13 (SCK)

#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <SPI.h>

const int slaveSelectPin = 10;

// Arduino pro-mini
// Habituellement je branche l'afficheur ainsi
// Afficheur 2 lignes branché en 7,8,9,10,11,12 sur l'arduino micro
// LiquidCrystal lcd(7,8,9,10,11,12);

// Mais pour utiliser le SPI je dois modifier le branchement:
// LiquidCrystal lcd(7,8,9,6,5,4);

// NON UTILISE ICI (pour Arduino pro-micro)
// Afficheur branché en 4,5,6,7,8,9 pour l'arduino pro-micro


// =================================================
// Déclaration des constantes et variables principales
// =================================================


// =================================================
//             Températures glacière
// =================================================
// En testant je me suis aperçu qu'en fait la température oscillait non pas
// autour de CONSIGNE mais plutot et quasi exactement entre CONSIGNE et CONSIGNE_MOINS
// Donc pour avoir une oscillation légère autour de la température voulue
// (par exemple 15°) il faut que la température voulue soit exactement ENTRE CONSIGNE et CONSIGNE_MOIS
// Donc pour 15° de référence je mets:
// CONSIGNE_PLUS à 15,4
// CONSIGNE à 15,2
// CONSIGNE_moins à 14,8

// Avec CONSIGNE_MOINS = 14.9 CONSIGNE = 15.1 CONSIGNE_PLUS = 15.3
// La température oscille entre 14,9 et 15,1° et c'est impeccable
// (température ambiante 25 à 27)

// Valeur de la température voulue pour la glacière
// Typiquement 15° pour que le chocolat se conserve bien
const double CONSIGNE = 15.1;

// La valeur inférieure de la consigne pour arreter le refroidissement
// Si c'est inférieur à cela on arrete de refroidir
const double CONSIGNE_MOINS = 14.9;

// La valeur supérieure de la consigne pour passer en refroidissement maximum:
// Si c'est supérieur à cela on refroidit à fond
const double CONSIGNE_PLUS = 15.3;

// Et entre les deux on refroidit en réduit

// Variable de température inférieure
// à la consigne, en cas d'environnement plus froid que la glacière
// (ici 2 degrés sous la température de consigne)
// Dans ce cas on arrête le ventilateur et la glacière
double temperature_mini = CONSIGNE - 2;

// Valeur du potentiomètre électronique quand on refroidit
// "moyen" entre les 2 valeurs maximum de consigne
const byte POT_MOYEN = 220;


// =================================================
//             Emission RF433
// =================================================

// Canal d'émission: 3 canaux possibles:
// canal 1: 0x10   canal 2: 0x20   canal 3: 0x30
const byte canal = 0x20;

// Identifiant unique de la sonde (hexadecimal)
#define ID_SONDE 0xCB

// Emetteur RF433 connecté pin 2
const byte TX_PIN = 2;

// Modifier cette valeur pour changer le nombre de secondes
// entre 2 émissions de la température en RF433
// Une minute au moins (indiqué en secondes)
unsigned int tempsRF433 = 70;

// =================================================
//             Pin Arduino branchement
// =================================================

// Pin "OnOff" commande du transistor pour on/off glacière
// int OnOff = 3;
const byte OnOff = 3;

// LCD 2 lignes 16 colonnes
// Arduino pro-mini
LiquidCrystal lcd(7, 8, 9, 6, 5, 4);

// Capteur de température DS18B20 Connecté pin A1
// (avec résistance de rappel de 4,7k au +)
#define ONE_WIRE_BUS A1


// =================================================
// =================================================
// =================================================



uint32_t delayMS;

// ====================== Variables température et temps  ======================

// la variable ou on mettra la température mesurée
// (arrondi à 1 décimale)
double temperature;

// La valeur du potentiomètre électronique 0-255 :
byte Pot_Elec;

// Température via le capteur avec 2 décimales
float temp_c;

// Temps stockage intermediaire pour les calculs emission RF433
unsigned long time_rf;

// Flag pour l'hysteresis de température
// On stockera à -1 ou +1 suivant que la température
// à été dépassé en moins, en plus ou pas
int flag_temp;


// ================================================================


// Dessin du symbole degré
byte degreesymbol[8] = {B01100, B10010, B10010, B01100, B00000, B00000, B00000, B00000};


// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************


// Le mode Oregon qui n'envoie que la température
#define MODE 0 // Temperature only



const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME * 2;

#define SEND_HIGH() digitalWrite(TX_PIN, HIGH)
#define SEND_LOW() digitalWrite(TX_PIN, LOW)

// Buffer for Oregon message
byte OregonMessageBuffer[8];


/**
   \brief    Send logical "0" over RF
   \details  azero bit be represented by an off-to-on transition
   \         of the RF signal at the middle of a clock period.
   \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
inline void sendZero(void)
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}

/**
   \brief    Send logical "1" over RF
   \details  a one bit be represented by an on-to-off transition
   \         of the RF signal at the middle of a clock period.
   \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
inline void sendOne(void)
{
  SEND_LOW();
  delayMicroseconds(TIME);
  SEND_HIGH();
  delayMicroseconds(TWOTIME);
  SEND_LOW();
  delayMicroseconds(TIME);
}

/**
  Send a bits quarter (4 bits = MSB from 8 bits value) over RF

  @param data Source data to process and sent
*/

/**
   \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
   \param    data   Data to send
*/
inline void sendQuarterMSB(const byte data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}

/**
   \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
   \param    data   Data to send
*/
inline void sendQuarterLSB(const byte data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
   \brief    Send a buffer over RF
   \param    data   Data to send
   \param    size   size of data to send
*/
void sendData(byte *data, byte size)
{
  for (byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}

/**
   \brief    Send an Oregon message
   \param    data   The Oregon message
*/
void sendOregon(byte *data, byte size)
{
  sendPreamble();
  //sendSync();
  sendData(data, size);
  sendPostamble();
}

/**
   \brief    Send preamble
   \details  The preamble consists of 16 "1" bits
*/
inline void sendPreamble(void)
{
  byte PREAMBLE[] = {0xFF, 0xFF};
  sendData(PREAMBLE, 2);
}

/**
   \brief    Send postamble
   \details  The postamble consists of 8 "0" bits
*/
inline void sendPostamble(void)
{
#if MODE == MODE_0
  sendQuarterLSB(0x00);
#else
  byte POSTAMBLE[] = {0x00};
  sendData(POSTAMBLE, 1);
#endif
}

/**
   \brief    Send sync nibble
   \details  The sync is 0xA. It is not use in this version since the sync nibble
   \         is include in the Oregon message to send.
*/
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
   \brief    Set the sensor type
   \param    data       Oregon message
   \param    type       Sensor type
*/
inline void setType(byte *data, byte* type)
{
  data[0] = type[0];
  data[1] = type[1];
}

/**
   \brief    Set the sensor channel
   \param    data       Oregon message
   \param    channel    Sensor channel (0x10, 0x20, 0x30)
*/
inline void setChannel(byte *data, byte channel)
{
  data[2] = channel;
}

/**
   \brief    Set the sensor ID
   \param    data       Oregon message
   \param    ID         Sensor unique ID
*/
inline void setId(byte *data, byte ID)
{
  data[3] = ID;
}

/**
   \brief    Set the sensor battery level
   \param    data       Oregon message
   \param    level      Battery level (0 = low, 1 = high)
*/
void setBatteryLevel(byte *data, byte level)
{
  if (!level) data[4] = 0x0C;
  else data[4] = 0x00;
}

/**
   \brief    Set the sensor temperature
   \param    data       Oregon message
   \param    temp       the temperature
*/
void setTemperature(byte *data, float temp)
{
  // Set temperature sign
  if (temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;
  }
  else
  {
    data[6] = 0x00;
  }

  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt / 10 - (float)td) * 10);

  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);

  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;

  // Set temperature float part
  data[4] |= (tempFloat << 4);
}

/**
   \brief    Set the sensor humidity
   \param    data       Oregon message
   \param    hum        the humidity
*/
void setHumidity(byte* data, byte hum)
{
  data[7] = (hum / 10);
  data[6] |= (hum - data[7] * 10) << 4;
}

/**
   \brief    Set the sensor temperature
   \param    data       Oregon message
   \param    temp       the temperature
*/
void setPressure(byte *data, float pres)
{
  if ((pres > 850) && (pres < 1100)) {
    data[8] = (int)round(pres) - 856;
    data[9] = 0xC0;
  }
}

/**
   \brief    Sum data for checksum
   \param    count      number of bit to sum
   \param    data       Oregon message
*/
int Sum(byte count, const byte* data)
{
  int s = 0;

  for (byte i = 0; i < count; i++)
  {
    s += (data[i] & 0xF0) >> 4;
    s += (data[i] & 0xF);
  }

  if (int(count) != count)
    s += (data[count] & 0xF0) >> 4;

  return s;
}

/**
   \brief    Calculate checksum
   \param    data       Oregon message
*/
void calculateAndSetChecksum(byte* data)
{
#if MODE == MODE_0
  int s = ((Sum(6, data) + (data[6] & 0xF) - 0xa) & 0xff);

  data[6] |=  (s & 0x0F) << 4;     data[7] =  (s & 0xF0) >> 4;
#elif MODE == MODE_1
  data[8] = ((Sum(8, data) - 0xa) & 0xFF);
#else
  data[10] = ((Sum(10, data) - 0xa) & 0xFF);
#endif
}



// ***************************************************************************
// ***************************************************************************
// ***************************************************************************



unsigned long secondMillis() {

  // Retourne le nombre de secondes depuis le démarrage du programme sous la forme d'un
  // nombre entier sur 32 bits (unsigned long).

  // Grace à cette fonction on à pas le problème du débordement (rollover)
  // de la fonction millis au bout de 50 jours.
  // Le calcul interne est effectué sur 64 bits d'après le site:
  // https://www.carnetdumaker.net/articles/la-gestion-du-temps-avec-arduino/
  // modifié par joel pour le retour en unsigned long en secondes
  // "return finalMillis /1000"

  static unsigned long nbRollover = 0;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis < previousMillis) {
    nbRollover++;
  }
  previousMillis = currentMillis;

  unsigned long long finalMillis = nbRollover;
  finalMillis <<= 32;
  finalMillis +=  currentMillis;

  return finalMillis / 1000;
}





void thermostat()
{

  // *****************************************************
  // La température souhaitée :
  // CONSIGNE (exemple 15°)

  // La valeur supérieure de la consigne pour passer en refroidissement maximum:
  // Si c'est supérieur à CONSIGNE_PLUS on refroidit à fond
  // CONSIGNE_PLUS (exemple 15.5°)

  // La valeur Inférieure de la consigne pour arreter le refroidissement
  // Si c'est inférieur à CONSIGNE_MOINS on arrete de refroidir
  // CONSIGNE_MOINS (exemple 14.5°)

  // Entre les deux on refroidit en réduit (50% en gros)
  // *****************************************************

  // La température réelle mesurée (arrondi à 1 décimale)
  // temperature

  // La valeur du potentiomètre électronique 0-255 :
  // Pot_Elec


  // *****************************************************
  // ******************* Thermostat **********************
  // *****************************************************

  // calcul refroidissement, la variable Pot_Elec contient
  // ensuite la valeur calculée

  // *****************************************************
  // Calcul de la tension à envoyer en fonction de la température

  // Il faut ajouter un hysteresis pour ne pas que le peltier
  // se mette en marche et se coupe toutes les quelques secondes quand la temperature
  // minimum est atteinte (valable aussi dans l'autre sens en température trop élevée)

  // On stocke un flag temperature minimum et maximum atteinte
  // et on ne le remettra en marche que quand la consigne sera de nouveau atteinte
  // ce qui fera un hysteresis de 0,5 degré de chaque coté

  // Même si on ne coupe pas le ventilateur ici, on l'allume à chaque fois que
  // l'on refroidit, et ceci au cas ou il aurait été arreté avant: par exemple
  // température ambiante plus froide que la glacière.
  // Donc ne pas changer cela, on allume le ventilateur même si on ne l'éteint
  // pas ici

  if (temperature >= CONSIGNE_PLUS) {
    // Si la température est plus élevée que la consigne maximum on refroidit à fond
    // On est au dessus de la température maximum, on mets donc flag_temp à +1
    flag_temp = +1;
    // On allume la glacière à fond et le ventilateur
    digitalWrite(OnOff, HIGH);
    Pot_Elec = 255;
  }

  if (temperature <= CONSIGNE_MOINS) {
    // Si la température est plus basse que la consigne minimum on coupe tout
    // (mais on laisse marcher le ventilateur)
    // On est passé sous la température minimum, on mets donc flag_temp à -1
    flag_temp = -1;
    // Extinction de la glacière
    // digitalWrite(OnOff, LOW);
    Pot_Elec = 0;
  }



  // Au cas ou la glacière serait dans un environnement plus froid
  // que sa température de consigne, on coupe tout (ventilateur aussi)
  if (temperature <= temperature_mini) {
    // Si la température est plus basse que la consigne minimum on coupe tout
    flag_temp = 0;
    // Extinction de la glacière et du ventilateur
    digitalWrite(OnOff, LOW);
    Pot_Elec = 0;
  }




  // *****************************************************************************************
  // On teste ici entre la température minimum et la maximum
  // et on joue sur un hysteresis de 0,5°
  // entre CONSIGNE_MOINS et CONSIGNE
  // et
  // CONSIGNE et CONSIGNE_PLUS
  // *****************************************************************************************


  // Si la température est entre CONSIGNE_PLUS et CONSIGNE_MOINS
  if ((temperature <= CONSIGNE_PLUS) && (temperature >= CONSIGNE_MOINS)) {

    // *****************************************************************************************
    if (flag_temp == -1) {
      // La température est descendue en dessous du minimum
      // On va donc attendre qu'elle soit remontée jusque CONSIGNE
      // on va laisser la glacière coupée jusqu'à ce que la température soit remontée à CONSIGNE
      // Extinction de la glacière
      Pot_Elec = 0;

      // Glacière coupée, on teste si la température est remontée au dessus de CONSIGNE
      if (temperature >= CONSIGNE) {
        // la température est repassée au dessus de CONSIGNE
        // On refroidit de nouveau à 50%
        // On remets le flag température à 0
        flag_temp = 0;
        // On refroidit de nouveau à 50%
        digitalWrite(OnOff, HIGH);
        Pot_Elec = POT_MOYEN;
      }
    }


    if (flag_temp == 1) {
      // La température est montée au dessus du maximum
      // On va donc refroidir à fond jusqu'à ce que la température soit redescendue à CONSIGNE
      // On allume la glacière à fond et le ventilateur
      digitalWrite(OnOff, HIGH);
      Pot_Elec = 255;

      // Glacière à fond, on teste si la température est redescendue au dessous de CONSIGNE
      if (temperature <= CONSIGNE) {
        // la température est repassée au dessous de CONSIGNE
        // On refroidit à 50%
        // On remets le flag température à 0
        flag_temp = 0;
        // On refroidit à 50%
        digitalWrite(OnOff, HIGH);
        Pot_Elec = POT_MOYEN;
      }
    }


  }

  // *****************************************************************************************
  // Fin CONSIGNE_PLUS et CONSIGNE_MOINS
  // *****************************************************************************************

  // On envoie la valeur de Pot_Elec (0-255) au potentiomètre électronique
  // pour refroidir le module Peletier:
  // 255 12,6v
  // 230 9,3v
  // 220 7,4v
  // 210 6,3v

  // On sélectionne le MCP 41050
  digitalWrite(slaveSelectPin, LOW);

  // On envpoie la valeur au potentiomètre électronique
  // La commande se fait sur deux octets, le premier étant la commande et le second la valeur.
  SPI.transfer(B00010001); // Commande
  SPI.transfer(Pot_Elec); // Valeur
  digitalWrite(slaveSelectPin, HIGH);


  // ***************************************************************************
  // ***************************************************************************
  // ***************************************************************************


}


void capteur_et_lcd()
{


  // ************** Interrogation du capteur de température ***************

  // Delay between measurements.
  delay(delayMS);

  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures

  // On stocke tout de suite la température pour l'envoyer ensuite via rf433
  // Si on ne le fait pas ici ça ne marche pas
  temp_c = sensors.getTempCByIndex(0);

  // Arrondi à 1 décimale pour l'affichage
  temperature = round(temp_c * 10) / 10.0;

  // ************** FIN Interrogation du capteur de température ***************



  lcd.setCursor(0, 0);
  // Si la température est de -127 c'est qu'il ya un problème de capteur
  if ((int)temp_c == -127)
  {

    lcd.print("Erreur capteur !");
  }
  else
  {
    // on affiche un espace avant et après pour bien effacer la ligne
    // au cas ou il y aurait eu le message d'erreur avant
    lcd.print(" Temp.: ");
    lcd.print(temperature, 1);

    //lcd.print(" ");
    // On affiche le caractère spécial degré définit en haut du programme
    lcd.write(byte(0));
    lcd.print("C ");


  }

}


// ***************************************************************************
// ***************************************************************************


void emission_433()
{


  // ------------- Emission de la température en RF433 --------------


  // Get Temperature, humidity and battery level from sensors
  // (ie: 1wire DS18B20 for température, ...)
  setBatteryLevel(OregonMessageBuffer, 1); // 0 : low, 1 : high
  setTemperature(OregonMessageBuffer, temperature);

  // Calculate the checksum
  calculateAndSetChecksum(OregonMessageBuffer);

  // Show the Oregon Message
  // Serial.println("Oregon -> ");

  for (byte i = 0; i < sizeof(OregonMessageBuffer); ++i)   {
    // Serial.print(OregonMessageBuffer[i] >> 4, HEX);
    // Serial.print(OregonMessageBuffer[i] & 0x0F, HEX);
  }


  // Send the Message over RF
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
  // Send a "pause"
  SEND_LOW();
  delayMicroseconds(TWOTIME * 8);
  // Send a copie of the first message. The v2.1 protocol send the
  // message two time
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));

  SEND_LOW();

  // ------------- FIN Emission de la température en RF433 --------------


}


// ***************************************************************************
// ***************************************************************************
// ***************************************************************************










void setup()
{

  // On déclare le caractère 0 personnalisé
  // le symbole degré, initialisé au début
  lcd.createChar(0, degreesymbol);

  // Initialize LCD to 16 characters by 2 lines
  lcd.begin(16, 2);
  lcd.clear();

  // Start up the library
  sensors.begin();

  // On déclare la pin du capteur
  pinMode(TX_PIN, OUTPUT);

  SEND_LOW();

  // Create the Oregon message for a temperature only sensor (THN132N)
  byte ID[] = {0xEA, 0x4C};

  setType(OregonMessageBuffer, ID);
  setChannel(OregonMessageBuffer, canal);
  setId(OregonMessageBuffer, ID_SONDE);



  // ************** DEBUT Initialisation de l'affichage LCD ***************
  lcd.begin(16, 2);
  lcd.print("Glaciere Choco");

  lcd.setCursor(0, 1);
  lcd.print("Jojo 06/19 V2.1");

  // lcd.print("Consigne: ");
  // lcd.print(CONSIGNE, 1);
  // On affiche le caractère spécial degré définit en haut du programme
  // lcd.write(byte(0));

  delay(4000);
  lcd.clear();

  // On déclare le caractère 0 personnalisé
  // le symbole degré, initialisé au début
  lcd.createChar(0, degreesymbol);

  // ************** FIN Initialisation de l'affichage LCD ***************


  // si la broche analogique 0 n'est pas connectée, le bruit analogique aléatoire
  // provoquera l'appel de l'instruction randomSeed() pour générer
  // différent nombre de départ à chaque exécution du programme.
  // randomSeed() brouillera alors la fonction aléatoire

  randomSeed(analogRead(0));

  // on initialise le temps
  // ce qui permettra d'envoyer dès le démarrage une emission rf433
  time_rf = 0;

  // Initialisation du potentiomètre electropnique MCP 41050
  pinMode (slaveSelectPin, OUTPUT);
  SPI.begin();

  // On déclare la pin OnOff en sortie
  pinMode(OnOff, OUTPUT);

  // On allume la glacière
  digitalWrite(OnOff, HIGH);

  // Extinction de la glacière si besoin:
  // digitalWrite(OnOff, LOW);

  // Flag température pour l'initialisation de l'hysteresis
  flag_temp = 0;


  // Pour debugger si besoin

  // Serial.begin(9600);
  // Serial.println(POT_MOYEN);
  // Serial.println(Pot_Elec);

}





void loop()
{

  // On mesure la température et on l'affiche sur le LCD
  capteur_et_lcd();

  // On calcule si on refroidit ou pas et de combien
  // puis on l'envoie à la glacière
  thermostat();

  // Il faut minimum 2 secondes entre 2 interrogations du capteur
  // pour avoir une mesure fiable
  delay(2000);


  // Debug: Affichage température de consige et Pot_Elec
  lcd.setCursor(0, 1);
  lcd.print("C:");
  lcd.print(CONSIGNE, 1);

  lcd.print(" P:");
  lcd.print(int(Pot_Elec), 1);

  lcd.print(" F:");
  switch (flag_temp) {
    case 1:
      lcd.print("+");
      break;
    case -1:
      lcd.print("-");
      break;
    case 0:
      lcd.print("0");
      break;
  }
  lcd.print("   ");



  // ------------- Emission de la température en RF433 --------------

  // On envoie par radio RF433 toutes les minutes
  // (définit dans les premières lignes du script tout en haut
  // avec la variable "tempsRF433")
  // pour éviter que tous les thermomètres n'envoient leurs
  // données en même temps, on ajoute quelques secondes aléatoires

  // Le temps en secondes depuis le démarrage est calculé
  // grace à la fonction secondMillis() et on à pas le problème du débordement (rollover)
  // de la fonction millis au bout de 50 jours.

  // On regarde si le temps arduino est superieur au temps calculé précédemment
  // (temps arduino + temps défini pour l'émission + quelques secondes aléatoires)
  if (secondMillis() > time_rf ) {

    // On est > au nombre de secondes choisies
    // On peut donc émettre la température en RF433
    emission_433();

    // On calcule une nouvelle valeur aléatoire pour ne pas
    // emettre toujours exactement au meme moment
    // et ne pas risquer de brouiller un autre thermometre
    // Nombre aléatoire de 0 à 5: random(6)
    // que l'on rajoute au temps choisi (tempsRF433)
    // et que l'on stocke dans time_rf pour la prochaine fois

    time_rf = secondMillis() + tempsRF433 + random(6);
  }

}
