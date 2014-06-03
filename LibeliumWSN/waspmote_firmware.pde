/* +---------------------------------------------------------------------------+
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Manuel López Antequera  <mlopezantequera@gmail.com>           |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/*
 *  ----- waspmote_mapir------
 *
 *  Explanation: 
 *  Sends the MAC address, accelerometer data, temperature and battery level of the board to the ZigBee network.
 *  This is done every second, no battery 	
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version:                0.3
 *  Contact:                Manuel López Antequera <mlopezantequera@gmail.com>
 */


/* CHANGELOG:

v0.1 --------------------------------------------
initial version, sends default Waspmote example string.

v0.2 --------------------------------------------

new function: build_message 
  builds the string to send based on which 'SEND_XXX' flags are enabled. 
  It runs at the start and should run after any changes in these flags (in the future they will be configured over the air).

new function: power_management 
  enables/disables devices and sets pins as input/output depending on which 'SEND_XXX' flags are enabled.

Currently available variables: Digital inputs 1-8, Analog inputs 1-7, Temperature, Accelerometer axes, Battery level.

Discovered that there is a maximum payload of 100 bytes (Which translates to 80 useful ascii characters after removal of the headers)

v0.3 --------------------------------------------

Fixed the payload problem: The message is cut into different parts if the payload limit is reached, these parts are sent as different messages.
The ID of the device is mandatory and will appear on all of the parts of the message.
The MAC is not mandatory anymore because it is very long. It is now an option to send the MAC.

v0.4 --------------------------------------------


/* TO-DO / GOALS
*
*	Allow for live board configuration through the ZigBee betwork:
*	    Accept configuration messages to change the frequency of the messages.
*	    Accept configuration messages to change which variables are actually sent.
*	Improve battery management (go to deep sleep, hibernate (jumper)   , disable leds..)
*/

/* IMPROVEMENT OPTIONS
*       Battery level warning (it's better to enable the digipot for a warning instead of sampling battery level all the time: enable both options!
*       Some warning variable that the board has been reset could be useful.
*       Digital outputs (1-8)
*       Digital1 can be PWM output (1-255)
*       RTC read / write
*/

// Identifier for this Waspmote (To avoid sending the MAC every time)
#define WID 1
// Maximum number of characters we can send in a single frame.
#define MAX_PAYLOAD 75

packetXBee* paq_sent; 
int8_t state=0; 
long previous=0; 
char aux[MAX_PAYLOAD];
char aux2[MAX_PAYLOAD];
char* macHigh="          ";
char* macLow="           ";
int aux_1 = 0;
int aux_2 = 0;
char WaspmoteID = WID;

// "Send" flags
boolean SEND_D1 = true;
boolean SEND_D2 = true;
boolean SEND_D3 = true;
boolean SEND_D4 = true;
boolean SEND_D5 = true;
boolean SEND_D6 = true;
boolean SEND_D7 = true;
boolean SEND_D8 = true;
boolean SEND_A1 = true;
boolean SEND_A2 = true;
boolean SEND_A3 = true;
boolean SEND_A4 = true;
boolean SEND_A5 = true;
boolean SEND_A6 = true;
boolean SEND_A7 = true;
boolean SEND_BATT = true;
boolean SEND_ACC = true;
boolean SEND_TEMP = true;
boolean SEND_MAC = true;

// Other configuration flags and settings
unsigned long int sample_period = 2000; //ms
boolean power_5v = true;
boolean power_3v3 = true;

// "Appended" flags
boolean D1_appended = false;
boolean D2_appended = false;
boolean D3_appended = false;
boolean D4_appended = false;
boolean D5_appended = false;
boolean D6_appended = false;
boolean D7_appended = false;
boolean D8_appended = false;

boolean A1_appended = false;
boolean A2_appended = false;
boolean A3_appended = false;
boolean A4_appended = false;
boolean A5_appended = false;
boolean A6_appended = false;
boolean A7_appended = false;

boolean MAC_appended = false;
boolean ACC_appended = false;
boolean TEMP_appended = false;
boolean BATT_appended = false;

uint8_t direccion[8]={0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF};

unsigned int append_strings (char *aux, char *aux2, unsigned int max_size)
{
  if(strlen(aux) + strlen(aux2) > max_size)
    return 1;
  
  strcat (aux,aux2);
  return 0;
}


//build_message appends all the variables that are requested by the "SEND_XXX" flags into
//the string variable 'aux'. 
//There is a maximum payload of MAX_PAYLOAD characters, counting the device ID and \r\n chars. 
//If the requested payload is more than MAX_PAYLOAD characters, it will stop before filling it and will return 1.
//It will only append variables not previously appended (XXX_APPENDED flag)
unsigned int build_message(char *aux)
{
  char *ptr;
  ptr = aux;
  unsigned int does_not_fit = 0;

  ptr += sprintf(ptr,"ID=%d",WaspmoteID);
  
  if(SEND_MAC && !MAC_appended) 
  {
    sprintf(aux2,",MAC=%s%s",macHigh,macLow);
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      MAC_appended = true;
  }
  
  if (SEND_ACC && !ACC_appended)
  {
    sprintf(aux2,",X=%d,Y=%d,Z=%d",ACC.getX(),ACC.getY(),ACC.getZ());
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      ACC_appended = true;
  }

  if (SEND_TEMP && !TEMP_appended)
  {
    sprintf(aux2,",TEMP=%d",(int)RTC.getTemperature());
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      TEMP_appended = true;
  }
  
  if (SEND_BATT && !BATT_appended)
  {
    sprintf(aux2,",BAT=%d%%",PWR.getBatteryLevel());
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      BATT_appended = true;
  }

  if (SEND_D1 && !D1_appended)
  {
    sprintf(aux2,",D1=%d",digitalRead(DIGITAL1));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D1_appended = true;
  }

  if (SEND_D2 && !D2_appended)
  {
    sprintf(aux2,",D2=%d",digitalRead(DIGITAL2));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D2_appended = true;
  }
  
  if (SEND_D3 && !D3_appended)
  {
    sprintf(aux2,",D3=%d",digitalRead(DIGITAL3));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D3_appended = true;
  }
  
  if (SEND_D4 && !D4_appended)
  {
    sprintf(aux2,",D4=%d",digitalRead(DIGITAL4));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D4_appended = true;
  }
  
  if (SEND_D5 && !D5_appended)
  {
    sprintf(aux2,",D5=%d",digitalRead(DIGITAL5));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D5_appended = true;
  }
  
  if (SEND_D6 && !D6_appended)
  {
    sprintf(aux2,",D6=%d",digitalRead(DIGITAL6));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D6_appended = true;
  }
  
  if (SEND_D7 && !D7_appended)
  {
    sprintf(aux2,",D7=%d",digitalRead(DIGITAL7));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D7_appended = true;
  }
  
  if (SEND_D8 && !D8_appended)
  {
    sprintf(aux2,",D8=%d",digitalRead(DIGITAL8));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      D8_appended = true;
  }
  
  if (SEND_A1 && !A1_appended)
  {
    sprintf(aux2,",A1=%d",analogRead(ANALOG1));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      A1_appended = true;
  }
  
  if (SEND_A2 && !A2_appended)
  {
    sprintf(aux2,",A2=%d",analogRead(ANALOG2));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      A2_appended = true;
  }
  
  if (SEND_A3 && !A3_appended)
  {
    sprintf(aux2,",A3=%d",analogRead(ANALOG3));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      A3_appended = true;
  }
  
  if (SEND_A4 && !A4_appended)
  {
    sprintf(aux2,",A4=%d",analogRead(ANALOG4));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      A4_appended = true;
  }
  
  if (SEND_A5 && !A5_appended)
  {
    sprintf(aux2,",A5=%d",analogRead(ANALOG5));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      A5_appended = true;
  }
  
  if (SEND_A6 && !A6_appended)
  {
    sprintf(aux2,",A6=%d",analogRead(ANALOG6));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      A6_appended = true;
  }
  
  if (SEND_A7 && !A7_appended)
  {
    sprintf(aux2,",A7=%d",analogRead(ANALOG7));
    if (append_strings(aux,aux2,MAX_PAYLOAD))
      does_not_fit++;
    else
      A7_appended = true;
  }

  strcat (aux,"\r\n");
  
  return does_not_fit;
}

// power_management
// it should run after any change in the variables which are to be sent to preserve battery or enable the required peripherals
// it also manages direction control for I/O pins
void power_management()
{
  // Digital IO direction control
  if (SEND_D1) pinMode(DIGITAL1,INPUT);
  if (SEND_D2) pinMode(DIGITAL2,INPUT);
  if (SEND_D3) pinMode(DIGITAL3,INPUT);
  if (SEND_D4) pinMode(DIGITAL4,INPUT);
  if (SEND_D5) pinMode(DIGITAL5,INPUT);
  if (SEND_D6) pinMode(DIGITAL6,INPUT);
  if (SEND_D7) pinMode(DIGITAL7,INPUT);
  if (SEND_D8) pinMode(DIGITAL8,INPUT);
  
  // This will turn on the ACC all the time, regardless of the selected sampling frequency
  // This will have to be improved in future versions so that the ACC is turned on only to measure and then back to off.
  if (SEND_ACC) ACC.ON();
  else ACC.setMode(ACC_HIBERNATE);
  //
  
  if (power_5v == true)
    PWR.setSensorPower(SENS_5V,SENS_ON);
  else
    PWR.setSensorPower(SENS_5V,SENS_OFF);
    
  if (power_3v3 == true)
    PWR.setSensorPower(SENS_3V3,SENS_ON);
  else
    PWR.setSensorPower(SENS_3V3,SENS_OFF);
    
    
        
// Todo:
// Disable RTC if none of the devices 
//that need it are enabled 
//(¿i2c needs RTC?) (TEMP,DIGITPOT, ACC)
  
  // Blink + wait before leaving.
  for (int i=0;i<8;i++){
    Utils.blinkLEDs(125);
  }
}

void setup(){
  
  RTC.ON();
  power_management();

  XBee.setMode(XBEE_ON);
  XBee.begin(9600);
  delay(1000);
  XBee.print("+++");
  delay(2000);
  XBee.println("ATBD5,AP2,WR,CN");
  delay(150);
  
  XBee.setMode(XBEE_OFF);
  XBee.close();
     
  // Inits the XBee XSC library 
  xbeeZB.init(ZIGBEE,FREQ2_4G,NORMAL);
 
  // Powers XBee 
  xbeeZB.ON();
  
  // Get the XBee MAC address
  delay(500);
  int counter = 0;  
  while(xbeeZB.getOwnMac()==1&&counter<4){
    xbeeZB.getOwnMac();
    counter++;
  }
  
  Utils.hex2str(xbeeZB.sourceMacHigh,macHigh,4);
  Utils.hex2str(xbeeZB.sourceMacLow,macLow,4);
}

//main loop
void loop(){
  bool more_to_send;
 
  paq_sent=(packetXBee*) calloc(1,sizeof(packetXBee)); 
  
  //Reset the "appended" flags
  D1_appended = false;
  D2_appended = false;
  D3_appended = false;
  D4_appended = false;
  D5_appended = false;
  D6_appended = false;
  D7_appended = false;
  D8_appended = false;
  
  A1_appended = false;
  A2_appended = false;
  A3_appended = false;
  A4_appended = false;
  A5_appended = false;
  A6_appended = false;
  A7_appended = false;
  
  MAC_appended = false;
  ACC_appended = false;
  TEMP_appended = false;
  BATT_appended = false;

  do{
    more_to_send = build_message(aux);
    paq_sent->mode=BROADCAST; 
    paq_sent->MY_known=0; 
    paq_sent->packetID=0x52; 
    paq_sent->opt=0; 
    xbeeZB.hops=0; 
    xbeeZB.setOriginParams(paq_sent,MAC_TYPE); 
    xbeeZB.setDestinationParams(paq_sent, direccion, aux, MAC_TYPE, DATA_ABSOLUTE);
    xbeeZB.sendXBee(paq_sent); 
  } while (more_to_send);
  
  free(paq_sent);
  paq_sent = NULL;
  
  delay(sample_period);
  // Change this to a proper sleep. (sleep function or deepSleep function)
}
