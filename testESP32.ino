/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// Import required libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "web_page.h"
#include <WiFiUdp.h>
#include <NTPClient.h>
#include "EEPROM.h"
#define DBG



/*typedefs*/
typedef struct
{
  uint8_t num;
  uint8_t last_state;  // posledni stav/zmena
  uint8_t id;
} Input;

typedef struct
{
  signed char num;
  uint8_t last_state;  // posledni stav/zmena
  uint8_t id;
} Output;

typedef struct
{
  uint16_t teplota;
  uint16_t cas;
} TermostatStruct;

typedef struct
{
  uint16_t casOn;
  uint16_t casOff;
} SpinackyStruct;



// typedef enum
// {
//   none = -1,
//   trvale = 0,
//   puls = 1,
//   casovac = 2
// } IoModeEnum;

typedef struct
{
  int8_t mode;
  Output *assoc_out;
  uint16_t time;
} IOctrlStruct;

/*variables*/
Input Inputs[4];
uint8_t inps[4] = { 33, 4, 26, 27 };// 4 jen na test / vratit25;

Output Outputs[6];
uint8_t outs[6] = { 23, 19,18, 5, 17, 16 };
int dataLenght;
TermostatStruct termostatTable[4];
Output termostatOut;
SpinackyStruct spinackyTable[4];
Output spinackyOut;
IOctrlStruct IO_controls[4];
uint16_t old_adc_val;
uint16_t adc_val;
uint16_t old_temperature;
uint16_t temperature;
uint16_t minutes;
uint64_t rtc_counter;
uint64_t out_timer;
uint64_t counter, counter_input, _millis;
boolean outSpinIsSetOn = false, outSpinIsSetOff = false;
boolean outTermIsSetOn = false, outTermIsSetOff = false;

uint64_t email_blocking_time[3] = { 0, 0, 0 };
bool enable_send_email[4] = { true, true, true, true };
bool client_connected=false;


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200);

/*deklarace funkci*/

void TestVstupu();
void ReadTemp();
void Spinacky();
void Termostat();
void DataToPage();
void ChangeOutput(Output *_out, uint8_t val);
void SetIO();
void ReadAdc();
void StringDivide(String s[], String s1, char delim, uint8_t cnt);
void PullFromEeprom();
void PushToEeprom();
void GetUdpTime ();


// Replace with your network credentials
const char *ssid = "Net22";
const char *password = "tajneheslo";
String jmeno_site = "";
String heslo = "";

OneWire oneWire(32);
DallasTemperature ds18b20(&oneWire);
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


void notifyClients(String s)
{
  if(client_connected)
  {
    ws.textAll(s);
  }
  
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
 {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  String payload = "";
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
  {
    data[len] = 0;
    String rec_data = "";
    payload = (char *)data;
#ifdef DBG
    Serial.println("ok"); /***/
#endif
    if (payload[0] == '&') 
    {
#ifdef DBG
      Serial.print("message lenght: "); /***/
      Serial.println(len);           /***/
#endif
     // dataLenght = len;
      uint8_t delimiters[19];
      uint8_t index = 0;
      for (int i = 0; i < len; i++)
       {
        rec_data += (char)data[i];
        if (payload[i] == '*')
         {
          delimiters[index++] = i;
#ifdef DBG
          Serial.print(i);
          Serial.print(" ");
#endif
        }
      }
      if (payload[1] == 'S') 
      {
        DataToPage();
        return;
      }

      if (payload[1] == 'T')
      {
        String tstamp = rec_data.substring(delimiters[0] + 1, delimiters[1]);
        minutes = tstamp.toInt();
        rtc_counter = minutes * 60;
#ifdef DBG
        char buf[15];
        String s = itoa(rtc_counter, buf, 10);
        Serial.println(tstamp);
        Serial.println(buf);
#endif
        return;
      }
#ifdef DBG
      Serial.print("data: "); /***/
      Serial.println(rec_data);   /***/
      Serial.println("sub:"); /***/
#endif
      String sub = "";
      // tabulka termostatu
      // termostat table teplota 1
      sub = rec_data.substring(delimiters[0] + 1, delimiters[1]);
#ifdef DBG
      Serial.println(sub); /***/
#endif
      termostatTable[0].teplota = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas 1
      termostatTable[0].cas = sub.substring(sub.indexOf('_') + 1).toInt();
      // termostat table teplota 2
      sub = rec_data.substring(delimiters[1] + 1, delimiters[2]);
#ifdef DBG
      Serial.println(sub); /***/
#endif
      termostatTable[1].teplota = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas 2
      termostatTable[1].cas = sub.substring(sub.indexOf('_') + 1).toInt();
      // termostat table teplota 3
      sub = rec_data.substring(delimiters[2] + 1, delimiters[3]);
#ifdef DBG
      Serial.println(sub); /***/
#endif
      termostatTable[2].teplota = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas 3
      termostatTable[2].cas = sub.substring(sub.indexOf('_') + 1).toInt();
      // termostat table teplota 4
      sub = rec_data.substring(delimiters[3] + 1, delimiters[4]);
#ifdef DBG
      Serial.println(sub); /***/
#endif
      termostatTable[3].teplota = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas 4
      termostatTable[3].cas = sub.substring(sub.indexOf('_') + 1).toInt();
#ifdef DBG
      Serial.println(); /***/
#endif
      // tabulka spinacek
      // spinacky table cas ON 1
      sub = rec_data.substring(delimiters[5] + 1, delimiters[6]);
#ifdef DBG
      Serial.println(sub); /***/
#endif
      spinackyTable[0].casOn = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas OFF 1
      spinackyTable[0].casOff = sub.substring(sub.indexOf('_') + 1).toInt();
      // termostat table cas ON 2
      sub = rec_data.substring(delimiters[6] + 1, delimiters[7]);
#ifdef DBG
      Serial.println(sub);
#endif
      spinackyTable[1].casOn = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas OFF 2
      spinackyTable[1].casOff = sub.substring(sub.indexOf('_') + 1).toInt();
      // termostat table cas ON 3
      sub = rec_data.substring(delimiters[7] + 1, delimiters[8]);
#ifdef DBG
      Serial.println(sub);
#endif
      spinackyTable[2].casOn = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas OFF 3
      spinackyTable[2].casOff = sub.substring(sub.indexOf('_') + 1).toInt();
      // termostat table cas ON 4
      sub = rec_data.substring(delimiters[8] + 1, delimiters[9]);
#ifdef DBG
      Serial.println(sub);
#endif
      spinackyTable[3].casOn = sub.substring(0, sub.indexOf('_')).toInt();
      // termostat table cas OFF 4
      spinackyTable[3].casOff = sub.substring(sub.indexOf('_') + 1).toInt();

      // vystup pro termostat
      sub = rec_data.substring(delimiters[10] + 1, delimiters[11]);
#ifdef DBG
      Serial.println(sub);
#endif
      uint8_t idx = sub.toInt();
      termostatOut.num = outs[idx];
      termostatOut.id = idx + 1;
      // vystup pro spinacky
      sub = rec_data.substring(delimiters[11] + 1, delimiters[12]);
#ifdef DBG
      Serial.println(sub);
#endif
      idx = sub.toInt();
      spinackyOut.num = outs[idx];
      spinackyOut.id = idx + 1;
      // IO nastaveni 1
      sub = rec_data.substring(delimiters[13] + 1, delimiters[14]);
#ifdef DBG
      Serial.println(sub);
#endif
      IO_controls[0].mode = sub.substring(0, sub.indexOf('_')).toInt();
      idx = sub.substring(sub.indexOf('_') + 1, sub.indexOf('|')).toInt();
      IO_controls[0].assoc_out = &Outputs[idx];
      IO_controls[0].time = sub.substring(sub.indexOf('|') + 1).toInt();
      // IO nastaveni 2
      sub = rec_data.substring(delimiters[14] + 1, delimiters[15]);
#ifdef DBG
      Serial.println(sub);
#endif
      IO_controls[1].mode = sub.substring(0, sub.indexOf('_')).toInt();
      idx = sub.substring(sub.indexOf('_') + 1, sub.indexOf('|')).toInt();
      IO_controls[1].assoc_out = &Outputs[idx];
      IO_controls[1].time = sub.substring(sub.indexOf('|') + 1).toInt();
      // IO nastaveni 1
      sub = rec_data.substring(delimiters[15] + 1, delimiters[16]);
#ifdef DBG
      Serial.println(sub);
#endif
      IO_controls[2].mode = sub.substring(0, sub.indexOf('_')).toInt();
      idx = sub.substring(sub.indexOf('_') + 1, sub.indexOf('|')).toInt();
      IO_controls[2].assoc_out = &Outputs[idx];
      IO_controls[2].time = sub.substring(sub.indexOf('|') + 1).toInt();
      // IO nastaveni 1
      sub = rec_data.substring(delimiters[16] + 1, delimiters[17]);
#ifdef DBG
      Serial.println(sub);
#endif
      IO_controls[3].mode = sub.substring(0, sub.indexOf('_')).toInt();
      idx = sub.substring(sub.indexOf('_') + 1, sub.indexOf('|')).toInt();
      IO_controls[3].assoc_out = &Outputs[idx];
      IO_controls[3].time = sub.substring(sub.indexOf('|') + 1).toInt();
      PushToEeprom();
      PullFromEeprom();
    }

    else if (payload[0] == '#') 
    {
#ifdef DBG
      Serial.print("IO ");
      Serial.print(payload[3]);
      Serial.print(payload[5]);
#endif

      switch (payload[3]) 
      {
        case '1':
          if (payload[5] == 'n')
            ChangeOutput(&Outputs[0], HIGH);
          else
            ChangeOutput(&Outputs[0], LOW);
          break;

        case '2':
          if (payload[5] == 'n')
            ChangeOutput(&Outputs[1], HIGH);
          else
            ChangeOutput(&Outputs[1], LOW);
          break;

        case '3':
          if (payload[5] == 'n')
            ChangeOutput(&Outputs[2], HIGH);
          else
            ChangeOutput(&Outputs[2], LOW);
          break;

        case '4':
          if (payload[5] == 'n')
            ChangeOutput(&Outputs[3], HIGH);
          else
            ChangeOutput(&Outputs[3], LOW);
          break;

        case '5':
          if (payload[5] == 'n')
            ChangeOutput(&Outputs[4], HIGH);
          else
            ChangeOutput(&Outputs[4], LOW);
          break;

        case '6':
          if (payload[5] == 'n')
            ChangeOutput(&Outputs[5], HIGH);
          else
            ChangeOutput(&Outputs[5], LOW);
          break;          
        default:
          break;
      }

#ifdef DBG
      Serial.print(Outputs[0].last_state);
      Serial.print(Outputs[1].last_state);
      Serial.print(Outputs[2].last_state);
      Serial.println(Outputs[3].last_state);
#endif
    }
    

  }

 }



void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) 
{
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      client_connected = true;
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      client_connected = false;
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}


int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

void setup() {
  // Serial port for debugging purposes
  SetIO();
  EEPROM.begin(512);
  Serial.begin(115200);
  ds18b20.begin();

  ///////  pinMode(ledPin, OUTPUT);
  ///////digitalWrite(ledPin, LOW);
  // Grab a count of devices on the wire
  numberOfDevices = ds18b20.getDeviceCount();
  
  // locate devices on the bus
  Serial.print("Locating devices...");
  
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (ds18b20.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(ds18b20.getAddress(tempDeviceAddress, i))
	  {
		Serial.print("Found device ");
		Serial.print(i, DEC);
		Serial.print(" with address: ");
		for (uint8_t i = 0; i < 8; i++)
      {
        if (tempDeviceAddress[i] < 16) Serial.print("0");
        Serial.print(tempDeviceAddress[i], HEX);
      }
		Serial.println();
		
		Serial.print("Setting resolution to ");
		Serial.println(9, DEC);
		
		// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
		ds18b20.setResolution(tempDeviceAddress, 9);
		
		Serial.print("Resolution actually set to: ");
		Serial.print(ds18b20.getResolution(tempDeviceAddress), DEC); 
		Serial.println();
	}
  else
  {
		Serial.print("Found ghost device at ");
		Serial.print(i, DEC);
		Serial.print(" but could not detect address. Check power and cabling");
	}
  }
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());

  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) 
  {
    request->send_P(200, "text/html", webpage /*, processor*/);
  });

  // Start server
  server.begin();
  
  PullFromEeprom();
}

void loop() 
{
  ws.cleanupClients();
    //jednou za sekundu
  _millis = millis();
  if (_millis > counter)
  {
    ReadTemp();//
    //ReadAdc(); //cti ADC vstup
    if (termostatOut.num != 0)
      Termostat(); //obsluha termoststu
    if (spinackyOut.num != 0)
      Spinacky(); //obsluha spinacich hodin
    counter = millis() + 1000;
    GetUdpTime(); //aktualizuj cas z NTP serveru
  }
  //jednou za 20 ms cti vstupy
  if (_millis > counter_input)
  {
    TestVstupu();
    counter_input = millis() + 20;
  }
}



/////funcs//////////////////////////////////
void SetIO() 
{
  pinMode(27, INPUT);
  pinMode(33, INPUT);
  pinMode(4, INPUT);//4 jen na test / vratit25;
  pinMode(26, INPUT);
  pinMode(5, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(23, OUTPUT);

  Inputs[0].num = 33;
  Inputs[0].last_state = 0;
  Inputs[0].id = 1;
  Inputs[1].num = 4;//4 jen na test / vratit25;
  Inputs[1].last_state = 0;
  Inputs[1].id = 2;
  Inputs[2].num = 26;
  Inputs[2].last_state = 0;
  Inputs[2].id = 3;
  Inputs[3].num = 27;
  Inputs[3].last_state = 0;
  Inputs[3].id = 4;
  //todo 2x analog input
  Outputs[0].num = 23;//18;
  Outputs[0].id = 1;
  Outputs[0].last_state = 0;
  Outputs[1].num = 19;//5;
  Outputs[1].id = 2;
  Outputs[1].last_state = 0;
  Outputs[2].num = 18;//17;
  Outputs[2].id = 3;
  Outputs[2].last_state = 0;
  Outputs[3].num = 5;//16;
  Outputs[3].id = 4;
  Outputs[3].last_state = 0;
  Outputs[4].num =17;// 23;
  Outputs[4].id = 5;
  Outputs[4].last_state = 0;
  Outputs[5].num = 16;//19;
  Outputs[5].id = 6;
  Outputs[5].last_state = 0;


  IO_controls[0].assoc_out = &Outputs[0];
  IO_controls[0].mode = 1;
  IO_controls[0].time = 0;
  IO_controls[1].assoc_out = &Outputs[1];
  IO_controls[1].mode = 1;
  IO_controls[1].time = 0;
  IO_controls[2].assoc_out = &Outputs[2];
  IO_controls[2].mode = 1;
  IO_controls[2].time = 0;
  IO_controls[3].assoc_out = &Outputs[3];
  IO_controls[3].mode = 1;
  IO_controls[3].time = 0;
}


void StringDivide(String s[], String s1, char delim, uint8_t cnt)
{
  uint8_t index = 0;
  uint8_t delimiters[10];
  for (int i = 0; i < s1.length(); i++) {
    if (s1[i] == delim) {
      delimiters[index++] = i;
    }
  }
  for (int i = 0; i < cnt; i++) {
    s[i] = s1.substring(delimiters[i] + 1, delimiters[i + 1]);
  }
}


/**
 * @brief posle nastavene parametry na stranku
 * 
 */
void DataToPage() {
  //&#T*200_60*200_120*200_180*200_240*#S*60_120*180_240*300_360*420_480*#O*0*1*#IO*0_2|0*1_3|0*-1_-1|0*
  String send_data = "&#T*";
  send_data += String(termostatTable[0].cas) + "_";
  send_data += String(termostatTable[0].teplota) + "*";
  send_data += String(termostatTable[1].cas) + "_";
  send_data += String(termostatTable[1].teplota) + "*";
  send_data += String(termostatTable[2].cas) + "_";
  send_data += String(termostatTable[2].teplota) + "*";
  send_data += String(termostatTable[3].cas) + "_";
  send_data += String(termostatTable[3].teplota) + "*#S*";
  send_data += String(spinackyTable[0].casOn) + "_";
  send_data += String(spinackyTable[0].casOff) + "*";
  send_data += String(spinackyTable[1].casOn) + "_";
  send_data += String(spinackyTable[1].casOff) + "*";
  send_data += String(spinackyTable[2].casOn) + "_";
  send_data += String(spinackyTable[2].casOff) + "*";
  send_data += String(spinackyTable[3].casOn) + "_";
  send_data += String(spinackyTable[3].casOff) + "*#O*";
//vystupy spinacek a termostatu
  signed char j = 0;
  for (size_t i = 0; i < 4; i++){if (outs[i] == termostatOut.num)j = i;}    
  send_data += String(j) + "*";
  j = 0;
  for (size_t i = 0; i < 4; i++){if (outs[i] == spinackyOut.num)j = i;}
   send_data += String(j);     
  //nastaveni funkci vstupu    
  send_data += "*#IO*";
  //vstup 1
  send_data += String(IO_controls[0].mode) + "_";
  j = -1;
  for (size_t i = 0; i < 6; i++){if (IO_controls[0].assoc_out == Outputs + i)j = i;}   
  send_data += String(j) + "|";
  send_data += String(IO_controls[0].time) + "*";
  //vstup 2
  send_data += String(IO_controls[1].mode) + "_";
  j = -1;
  for (size_t i = 0; i < 6; i++){if (IO_controls[1].assoc_out == Outputs + i) j = i;} 
  send_data += String(j) + "|";
  send_data += String(IO_controls[1].time) + "*";
  //vstup 3
  send_data += String(IO_controls[2].mode) + "_";
  j = -1;
  for (size_t i = 0; i < 6; i++){if (IO_controls[2].assoc_out == Outputs + i)j = i;}   
  send_data += String(j) + "|";
  send_data += String(IO_controls[2].time) + "*";
  //vstup 4
  send_data += String(IO_controls[3].mode) + "_";
  j = -1;
  for (size_t i = 0; i < 6; i++){if (IO_controls[3].assoc_out == Outputs + i)j = i;}   
  send_data += String(j) + "|";
  send_data += String(IO_controls[3].time) + "*";

#ifdef DBG
  Serial.println("topage");
#endif
  unsigned int dl = send_data.length();
  char msg[dl];
  send_data.toCharArray(msg, dl);
  Serial.println(msg);
  delay(500);
  notifyClients(String(msg));
  delay(500);
}

void TestVstupu() {
  char msg[6];
  uint64_t local_timer = millis();
  for (size_t i = 0; i < 4; i++) {
    // kontrola jestli je seply casovany vystup
    if (IO_controls[i].mode == 2 && out_timer > 0) {
      if (IO_controls[i].assoc_out->last_state == HIGH) {
        if (millis() > out_timer) {
          ChangeOutput(IO_controls[i].assoc_out, LOW);
          out_timer = 0;
        }
      }
    }

    if (digitalRead(Inputs[i].num)) {
      if (Inputs[i].last_state != 1) {
        sprintf(msg, "#I%don", Inputs[i].id);
#ifdef DBG
        Serial.println(msg);
#endif
        Inputs[i].last_state = 1;
        notifyClients(String(msg));
        if (IO_controls[i].mode == 0)
          ChangeOutput(IO_controls[i].assoc_out, HIGH);
        else if (IO_controls[i].mode == 1) {
#ifdef DBG
          Serial.println("mode1");
#endif
          if (IO_controls[i].assoc_out->last_state == 0) {
            ChangeOutput(IO_controls[i].assoc_out, HIGH);
          } else {
            ChangeOutput(IO_controls[i].assoc_out, LOW);
          }
        } else if (IO_controls[i].mode == 2) {
#ifdef DBG
          Serial.println("mode2");
#endif
          out_timer = local_timer + (IO_controls[i].time * 1000);
          ChangeOutput(IO_controls[i].assoc_out, HIGH);
        } else if (IO_controls[i].mode == 3) {
#ifdef DBG
          Serial.println("mode3");
#endif
          if (local_timer > email_blocking_time[i]) enable_send_email[i] = true;
          if (enable_send_email[i] == true) {
            enable_send_email[i] = false;
            //  SendMail(i);
            email_blocking_time[i] = local_timer + 30000;  //30 sec blokuj dalsi
          }
        }
      }
    }

    else {
      if (Inputs[i].last_state != 0) {
        sprintf(msg, "#I%dof", Inputs[i].id);
#ifdef DBG
        Serial.println(msg);
#endif
        Inputs[i].last_state = 0;
        notifyClients(String(msg));
        if (IO_controls[i].mode == 0)
          ChangeOutput(IO_controls[i].assoc_out, LOW);
      }
      //povol mail
      if (IO_controls[i].mode == 3) {
        //  if(local_timer > email_blocking_time)enable_send_email[i] = true;
      }
    }
  }
}


void ReadTemp() {
  char msg[10];
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  temperature = (uint16_t)(t * 10);
  if (old_temperature != temperature) {
    old_temperature = temperature;
    sprintf(msg, "#T%02u,%1u", temperature / 10, temperature % 10);
    Serial.print("Temp:  ");
    Serial.println(msg);
    notifyClients(String(msg));
#ifdef DBG
    Serial.print(t);
#endif
  }
}

void Spinacky() {
#ifdef DBG
  // Serial.print("min:");Serial.println(minutes);
#endif

  for (uint8_t i = 0; i < 4; i++) {
    if (spinackyTable[i].casOn == minutes) {
      if (outSpinIsSetOn == false) {
        outSpinIsSetOn = true;
        outSpinIsSetOff = false;
        ChangeOutput(&spinackyOut, HIGH);
      }
    } else if (spinackyTable[i].casOff == minutes) {
      if (outSpinIsSetOff == false) {
        outSpinIsSetOff = true;
        outSpinIsSetOn = false;
        ChangeOutput(&spinackyOut, LOW);
      };
    }
  }
}

void Termostat() {
  uint8_t pom_i;
  uint16_t pom_cas_L, pom_cas_H;
  ReadTemp();
  for (size_t i = 0; i < 4; i++) {
    if (termostatTable[i].cas == 0)
      break;
    pom_i = i + 1;
    pom_cas_H = termostatTable[pom_i].cas;
    if (i == 3) {
      pom_i = 0;
      pom_cas_H = 1440 + termostatTable[0].cas;
    }
    pom_cas_L = termostatTable[i].cas;
    // if (i == 0)pom_cas_L = 0;
    if ((minutes >= pom_cas_L) && (minutes < pom_cas_H)) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(pom_cas_L);
      Serial.print("%");
      Serial.print(pom_cas_H);
      Serial.print("-");
      Serial.print(minutes);
      Serial.print("-/-");

      Serial.print(termostatTable[i].teplota);
      Serial.print("-");
      Serial.print(temperature);
      if (temperature < (termostatTable[i].teplota - 5)) {
        if (outTermIsSetOn == false) {
          outTermIsSetOn = true;
          outTermIsSetOff = false;
          ChangeOutput(&termostatOut, HIGH);
#ifdef DBG
          Serial.println(" tstaton");
#endif
        }
      }

      else if (temperature >= termostatTable[i].teplota) {
        if (outTermIsSetOff == false) {
          outTermIsSetOn = false;
          outTermIsSetOff = true;
          ChangeOutput(&termostatOut, LOW);
#ifdef DBG
          Serial.println(" tstatoff");
#endif
        }
      }
      break;
    }
  }
}

void ChangeOutput(Output *_out, uint8_t val)
{
  uint8_t x = _out->id;
  char msg[10];
  sprintf(msg, "#O%d%s", _out->id, val == 1 ? "on" : "of");
#ifdef DBG
  Serial.println(msg);
#endif
  _out->last_state = val;
  digitalWrite(_out->num, val);
#ifdef DBG
  Serial.println(_out->last_state);
#endif
  notifyClients(String(msg));
}

void PushToEeprom()
{
#ifdef DBG
  Serial.println("push EE");
#endif
  int start_point = 0;
  EEPROM.put(start_point, termostatTable);
  start_point += 16; // sizeof(termostatTable);
  delay(10);
#ifdef DBG
  Serial.print(start_point);
  Serial.print(">");
#endif
  EEPROM.put(start_point, spinackyTable);
  delay(10);
  start_point += 16; // sizeof(spinackyTable);
#ifdef DBG
  Serial.print(start_point);
  Serial.print(">");
#endif
  EEPROM.put(start_point, termostatOut);
  delay(10);
  start_point += sizeof(Output);
#ifdef DBG
  Serial.print(start_point);
  Serial.print(">");
#endif
  EEPROM.put(start_point, spinackyOut);
  delay(10);
  start_point += sizeof(Output);
#ifdef DBG
   Serial.print(start_point);
   Serial.print(">");
#endif
  EEPROM.put(start_point, IO_controls);
  delay(10);
  int ff = sizeof(IO_controls);
  Serial.print("#####");
  Serial.println(ff);
  start_point += sizeof(IO_controls);
#ifdef DBG
  Serial.println(start_point);
  Serial.print(">");
#endif
 unsigned int sze = jmeno_site.length();
  char x[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  char y[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 jmeno_site.toCharArray(x, sze + 1);
  EEPROM.put(start_point, x);
  delay(10);
 sze = heslo.length();
 heslo.toCharArray(y, sze + 1);
  start_point += 20;
#ifdef DBG
  Serial.println(start_point);
  Serial.print(">");
#endif
  EEPROM.put(start_point, y);
  delay(10);
  EEPROM.commit();
}

void PullFromEeprom()
{
#ifdef DBG
  Serial.print("pull EE? ");
#endif
  uint8_t c;
  for (int i = 0; i < 114; i++)
  {
    c = EEPROM.read(i);
#ifdef DBG
    if (i == 16 || i == 32 || i == 35 || i == 38 || i == 74 || i == 94)
      Serial.print(">");
    Serial.print(c);
    Serial.print(" ");

#endif
  }
  int start_point = 0;
  EEPROM.get(start_point, termostatTable);
  start_point += sizeof(termostatTable);
  EEPROM.get(start_point, spinackyTable);
  start_point += sizeof(spinackyTable);
  EEPROM.get(start_point, termostatOut);
  start_point += sizeof(Output);
  EEPROM.get(start_point, spinackyOut);
  start_point += sizeof(Output);
  EEPROM.get(start_point, IO_controls);

  start_point += 16;//sizeof(IO_controls);
  char x[20], y[20];
  EEPROM.get(start_point, x);
  jmeno_site = String(x);
  start_point += 20;
  EEPROM.get(start_point, y);
  heslo = String(y);
  Serial.println(jmeno_site);
  Serial.println(heslo);


  #ifdef DBG
    // Serial.print("data");
    // DataToPage();
    Serial.println();
    for (int i = 0; i < 4; i++) {
      Serial.print(termostatTable[i].teplota,10);
      Serial.print("-");
      Serial.println(termostatTable[i].cas,10);
    }
    for (int i = 0; i < 4; i++) {
      Serial.print(spinackyTable[i].casOn,10);
      Serial.print("-");
      Serial.println(spinackyTable[i].casOff,10);
    }
    Serial.print(termostatOut.num,10);
    Serial.print("x");
    Serial.println(spinackyOut.num,10);
    for (int i = 0; i < 4; i++) {
      Serial.print(IO_controls[i].mode,10);
      Serial.print("-");
      Serial.print(IO_controls[i].assoc_out->num,10);
      Serial.print("-");
      Serial.println(IO_controls[i].time,10);
    }
     Serial.print("SSID");
    Serial.println(jmeno_site);
    Serial.print("heslo");
    Serial.println(heslo);
    delay(300);
  #endif
}


void GetUdpTime ()
{
  timeClient.update();
  int hh = timeClient.getHours();
  int mm = timeClient.getMinutes();
  int ss = timeClient.getSeconds();
  rtc_counter = ( hh* 3600) +( mm * 60) + ss;
  minutes = (hh * 60) + mm;             
#ifdef DBG
  char buff[28];
  sprintf(buff, "%02u:%02u:%02u", hh, mm, ss);
  Serial.println(buff);
#endif
}