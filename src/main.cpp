// ESPP1 - P1 DSMR ESP32
// Author  : Romuald Dufour
// License : GPL v3
// Release : 2024.05


#if !defined(ESP8266)
  #error This code is designed to run on ESP8266 and ESP8266-based boards!
#endif

// Include framework & embedded hardware libs
#include <Arduino.h>
#include <string>
#include <Ticker.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <ota.h>

// Include project specific headers
#include "cred.h"


// Hardware I/O
int rx_pin = D2;  // D2
int tx_pin = D1;  // D1
int rx_led = D3;  // D3
//int tx_led = D4;

// Network vars
bool wifi_connected = false;
bool mqtt_connected = false;
bool p1_connected = false;
bool pm1_connected = false;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
WiFiClient p1_client;
WiFiServer p1_server(101);
WiFiClient pm1_client;
WiFiServer pm1_server(102);

#define MQTT_ON 1
#define MQTT_TOPIC "home/smartmeter/"
#define MQTT_TOPIC_SUB "home/smartmeter/cmd/#"
#define MQTT_LWT "home/smartmeter/availability"

// netcli vars
bool netcli_connected = false;
bool netcli_disconnect = false;
WiFiClient cli_client;
WiFiServer cli_server(23);

// cli vars
std::string line = "";
std::string dbgdsp = "";
bool cli_dspOrigP1 = false;
bool cli_dspModP1 = false;
bool cli_dspEnergy = false;
bool cli_dspPower = false;
bool cli_dspPeak = false;


// Configuration vars
struct CFG {
  uint32_t I_Max_meter = 32;
  uint32_t I_Shift = 0;
  bool send_p1 = true;
  bool send_pm1 = true;
  bool send_serial = true;
  //bool Show_Stats = false;
};

CFG cfg;
CFG cfg_old;

uint8_t eeprom_to_save = 0;
uint8_t cmd_clear = 255;


// P1 data
struct DG {
  bool received = false;
  bool decoded = false;
  bool sent = false;
  int idx = 0;
  int idx_crc = 0;
  uint16_t crc;
  char crc_computed[5];
  char crc_received[5];
  bool crc_valid;
  uint32_t E_consumed_1 = 0;
  uint32_t E_consumed_2 = 0;
  uint32_t E_consumed = 0;
  uint32_t E_injected_1 = 0;
  uint32_t E_injected_2 = 0;
  uint32_t E_injected = 0;
  uint32_t P_consumed = 0;
  uint32_t P_injected = 0;
  uint32_t P_max = 0;
  uint32_t P=0;
  uint32_t U_L1 = 0;
  uint32_t U_L2 = 0;
  uint32_t U_L3 = 0;
  uint32_t I_L1 = 0;
  uint32_t I_L2 = 0;
  uint32_t I_L3 = 0;
  uint32_t I_Mod_L1 = 0;
  uint32_t I_Mod_L2 = 0;
  uint32_t I_Mod_L3 = 0;
  uint32_t P_act_L1 = 0;
  uint32_t P_act_L2 = 0;
  uint32_t P_act_L3 = 0;
  uint32_t P_ap_L1 = 0;
  uint32_t P_ap_L2 = 0;
  uint32_t P_ap_L3 = 0;
  uint32_t P_cos_L1 = 0;
  uint32_t P_cos_L2 = 0;
  uint32_t P_cos_L3 = 0;
  uint32_t P_Mod_act_L1 = 0;
  uint32_t P_Mod_act_L2 = 0;
  uint32_t P_Mod_act_L3 = 0;
  uint32_t CurrentPeak = 0;
  uint32_t LastPeak = 0;
  uint32_t CurrentDate = 0;
  uint32_t CurrentTime = 0;
  uint32_t QuarterTime = 0;
  char buf[2048];
  char OrigP1[2048];
  char ModP1[2048];
};

DG dg;
DG dg_old;


// Other vars
Ticker Timer1;
unsigned long currentmillis = millis();
unsigned long lastmillis = currentmillis;
uint32_t uptime = 0;
uint8_t loopidx = 0;
int safecnt = 20;  // Time before start to allow OTA
char uptime_txt[48];
char dttime_txt[48];



void cli_print(String msg, bool ln = false, bool sercli=true, bool netcli=true)
{
  if (sercli) if (ln) Serial.println(msg); else Serial.print(msg);
  if (netcli){
    if (netcli_connected) {
      cli_client.write(msg.c_str());
      if (ln) cli_client.write("\r\n");
    }
  }
}


std::string trim(const std::string& str,
                 const std::string& whitespace = " \t")
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

void check_cfg() {
  if ((cfg.I_Max_meter < 0) || (cfg.I_Max_meter > 32)) cfg.I_Max_meter = 32;
  if ((cfg.I_Shift < 0) || (cfg.I_Shift > 32)) cfg.I_Shift = 32;
}

void print_cfg() {
  cli_print(String("Meter max current    : ") + cfg.I_Max_meter, true, false, true);
  cli_print(String("Station shift current  : ") + cfg.I_Shift, true, false, true);
}

void data_save() {
  check_cfg();
  EEPROM.put(0, cfg);
  EEPROM.commit();
  memcpy(&cfg_old, &cfg, sizeof(cfg));
  dbgdsp = "Data saved";
}

void data_load() {
  EEPROM.get(0, cfg);
  check_cfg();
  memcpy(&cfg_old, &cfg, sizeof(cfg));
  dbgdsp = "Data loaded";
  print_cfg();
}


void uptime_to_text(char const *header, char const *trailer) {
  int d = uptime/86400;
  int h = (uptime - (d*86400)) / 3600;
  int m = (uptime - (d*86400) - (h*3600)) / 60;
  int s = uptime % 60;
  sprintf(uptime_txt, "%s%4id %02ih:%02im:%02is%s", header, d, h, m, s, trailer);
}

void exec_cmd(bool ser = false, bool net = false) {
  std::string cmd = "";
  std::string param1 = "";
  std::string param2 = "";
  std::string param3 = "";
  int p1 = -1;
  int p2 = -1;
  int p3 = -1;
  uint8_t idx = 0;
  line = trim(line);
  while ((idx < line.length()) & (line[idx] != ' ')) {
    cmd += line[idx];
    idx++;
  }
  while ((idx < line.length()) & (line[idx] == ' ')) idx++;
  while ((idx < line.length()) & (line[idx] != ' ')) {
    param1 += line[idx];
    idx++;
  }
  while ((idx < line.length()) & (line[idx] == ' ')) idx++;
  while ((idx < line.length()) & (line[idx] != ' ')) {
    param2 += line[idx];
    idx++;
  }
  while ((idx < line.length()) & (line[idx] == ' ')) idx++;
  while ((idx < line.length()) & (line[idx] != ' ')) {
    param3 += line[idx];
    idx++;
  }
  try { p1 = std::stoi(param1, nullptr, 0); } catch(...) {}
  try { p2 = std::stoi(param2, nullptr, 0); } catch(...) {}
  try { p3 = std::stoi(param3, nullptr, 0); } catch(...) {}
 
  cli_print(cmd.c_str());

  if ((cmd == "show") || (cmd == "sh")) {
    //bool found = false;
    if (param1 == "uptime") {
      uptime_to_text("\n\rUptime         : ", "\n\r");
      cli_print(uptime_txt, true, ser, net);
    }
    if (param1 == "p1") {
      cli_dspOrigP1 = !cli_dspOrigP1;
    }
    if (param1 == "m1") {
      cli_dspModP1 = !cli_dspModP1;
    }
    if (param1 == "power") {
      cli_dspPower = !cli_dspPower;
    }
    if (param1 == "energy") {
      cli_dspEnergy = !cli_dspEnergy;
    }
    if (param1 == "peak") {
      cli_dspPeak = !cli_dspPeak;
    }
    if (param1 == "config") {
      print_cfg();
    }
  }
  if (cmd == "save") data_save();
  if (cmd == "load") data_load();
  
  if (cmd == "setshift") {
    if ((p1 >= 0) && (p1 <= 32)) cfg.I_Shift = p1;
  }

  if (cmd == "serialon") cfg.send_serial = true;
  if (cmd == "serialoff") cfg.send_serial = false;
  if (cmd == "p1on") cfg.send_p1 = true;
  if (cmd == "p1off") cfg.send_p1 = false;
  if (cmd == "pm1on") cfg.send_pm1 = true;
  if (cmd == "pm1off") cfg.send_pm1 = false;
}

void process_cli(bool ser=false, bool net=false) {

  if (dbgdsp.length() > 0) {
      for (unsigned int idx = 0; idx < 2+line.length(); idx++) cli_print("\b \b", false, ser, net);
      uptime_to_text("[", " ] ");
      cli_print(uptime_txt, false, ser, net);
      cli_print(dbgdsp.c_str(), true, ser, net);
      dbgdsp = "";
      cli_print("> ", false, ser, net);
      cli_print(line.c_str(), false, ser, net);
  }

   if (ser && Serial.available() || (net && netcli_connected && cli_client.available())) {
    char ch = 0x00;
    bool fromserial = false;
    bool fromnet = false;
    if (ser && Serial.available()) { ch = Serial.read(); fromserial = true; }
    if (net && netcli_connected && cli_client.available()) { ch = cli_client.read(); fromnet = true; }

    if (fromnet)
    switch (ch)
    {
      case 0xff: // telnet IAC
          break;
      case 0xfb: case 0xfc: case 0xfd: case 0xfe:  // telnet iac cmd
          if (netcli_connected && cli_client.available()) ch = cli_client.read(); // remove IAC command option if any
          break;
    }
    if (fromnet || fromserial) switch (ch)
    {
    case '\r': // CR
    case 0x2d : // Ctrl-M
        cli_print("", true, ser, net);
        exec_cmd();
        cli_print("> ", ser, net);
        line = "";
        break;
    case '\n': break;
    case '\b':
    case 0x7f:
    case 0x28: // Ctrl-H
        if (line.length() > 0) {
            line.pop_back();
            cli_print("\b \b", ser, net);
        }
        break;
    case 0x00:
        break;
    default:
        if (line.length() < 80) {
          line += ch;
          cli_print(String(ch), false, ser, net&fromserial);  // print only to serial, net is already echoed unless coming from serial
        }
        break;
    }
  }
}

void process_mqtt() {
  char topic[80];
  char value[255];
  if (mqtt_client.connected()) {
    if (cmd_clear != 255) {
        sprintf(topic, "%s%i/set", MQTT_TOPIC, cmd_clear);
        mqtt_client.publish(topic, "", true);
        cmd_clear = 255;
        return;
    }
    if (dg.decoded && !dg.sent) {
        if ((dg.E_consumed != dg_old.E_consumed) || (dg.E_injected != dg_old.E_injected)){
          sprintf(topic, "%s%s", MQTT_TOPIC, "Energy");
          sprintf(value, "{\"E_consumed\": %i,\"E_injected\": %i}", dg.E_consumed, dg.E_injected);
          mqtt_client.publish(topic, value, true);
        }
        sprintf(topic, "%s%s", MQTT_TOPIC, "Power");
        sprintf(value, "{\"P_consumed\": %i,\"P_injected\": %i}", dg.P_consumed, dg.P_injected);
        mqtt_client.publish(topic, value, true);
        sprintf(topic, "%s%s", MQTT_TOPIC, "Lines");
        sprintf(value, "{\"U_L1\": %i.%i,\"U_L2\": %i.%i,\"U_L3\": %i.%i}", dg.U_L1/10, dg.U_L1%10, dg.U_L2/10, dg.U_L2%10, dg.U_L3/10, dg.U_L3%10);
        mqtt_client.publish(topic, value, true);
        
        if (dg.P_consumed != dg_old.P_consumed){
          sprintf(topic, "%s%s", MQTT_TOPIC, "P_consumed");
          sprintf(value, "%i", dg.P_consumed);
          mqtt_client.publish(topic, value, true);
        }
        if (dg.P_injected != dg_old.P_injected){
          sprintf(topic, "%s%s", MQTT_TOPIC, "P_injected");
          sprintf(value, "%i", dg.P_injected);
          mqtt_client.publish(topic, value, true);
        }
        if (dg.E_consumed != dg_old.E_consumed){
          sprintf(topic, "%s%s", MQTT_TOPIC, "E_consumed");
          sprintf(value, "%i", dg.E_consumed);
          mqtt_client.publish(topic, value, true);
        }
        if (dg.E_injected != dg_old.E_injected){
          sprintf(topic, "%s%s", MQTT_TOPIC, "E_injected");
          sprintf(value, "%i", dg.E_injected);
          mqtt_client.publish(topic, value, true);
        }
        if (dg.LastPeak != dg_old.LastPeak){
          sprintf(topic, "%s%s", MQTT_TOPIC, "P_QuarterHourPeak");
          sprintf(value, "%i", dg.LastPeak);
          mqtt_client.publish(topic, value, true);
        }
        dg.sent = true;
        memcpy(&dg_old, &dg, sizeof(dg));
    }
  }
}

char *strremove(char *str, const char *sub) {
    char *p, *q, *r;
    if (*sub && (q = r = strstr(str, sub)) != NULL) {
        size_t len = strlen(sub);
        while ((r = strstr(p = r + len, sub)) != NULL) {
            while (p < r)
                *q++ = *p++;
        }
        while ((*q++ = *p++) != '\0')
            continue;
    }
    return str;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  char value[10] = "";
  uint8_t idx = 0;
  while ((idx < length) && (idx < sizeof(value) - 2)) {
    value[idx] = (char)payload[idx];
    idx++;
  }
  value[idx] = '\0';
  dbgdsp = "Message arrived [" + std::string(topic) + "] " + std::string(value);

  strremove(topic, MQTT_TOPIC);
  //int id = 0;
  //try { id = std::stoi(topic, nullptr, 0); } catch(...) {}

  payload[length] = 0;

/*  if (String(topic) == "cmd/mode") {
    if (String((char *) payload) == "transparent") {
      dbgdsp += " - Transparent mode ON";
    }
    if (String((char *)payload) == "") {
    }
  }
*/
  if (String(topic) == "cmd/maxamp") {
    int amp = 0;
    try { amp = std::stoi((char *)payload, nullptr, 0); } catch(...) {}
    dbgdsp += " - Set MAX amp to " + std::to_string(amp) + " A";
    cfg.I_Shift = amp;
  }
}

void crc_compute(DG *dg) {
  unsigned int crc = 0;
  if (dg->idx > 10) {
  for (unsigned int i = 0; i < dg->idx-4; i++) {
    crc ^= (unsigned int)(dg->buf[i]);
    int bit = 0;
    while (bit < 8) {
      if ((crc & 1) != 0) {
        crc = (crc >> 1) ^ 0xA001;
      }
      else crc >>= 1;
      bit++;
    }
  } 
  }else { /*cli_client.write('E');*/}
  sprintf(dg->crc_computed, "%04X", crc);
}

void crc_add(char *buf) {
  unsigned int crc = 0;
  unsigned int idx = 0;
  int16_t l = strlen(buf); 
  for (unsigned int i = 0; i < l; i++) {
    crc ^= (unsigned int)(buf[i]);
    int bit = 0;
    while (bit < 8) {
      if ((crc & 1) != 0) {
        crc = (crc >> 1) ^ 0xA001;
      }
      else crc >>= 1;
      bit++;
    }
    idx = i;
  }
  sprintf(&buf[idx+1], "%04X\n\r", crc);
  buf[idx+7] = '\0';
}

void p1_copytoorig(DG *dg) {
  if (dg->idx > 10) {
    for (unsigned int i = 0; i < dg->idx; i++) {
      dg->OrigP1[i] = dg->buf[i];
      dg->OrigP1[i+1] = '\0';
    }
  }
}

void p1_copytomod(DG *dg) {
  if (dg->idx > 10) {
    for (unsigned int i = 0; i < dg->idx-6; i++) {
      dg->ModP1[i] = dg->buf[i];
      dg->ModP1[i+1] = '\0';
    }
  }
}

uint32_t dg_obis_getdate() {
  char* code = "0-0:1.0.0(";
  char data[7];
  int idx = strstr(dg.buf, code) - dg.buf;
      if (idx > 0){
        memcpy(data, dg.buf + idx + strlen(code), 6);
        data[6]=0;
        uint32_t val = 0;
          try { val = std::stoi(data, nullptr, 10); } catch(...) {}
          return val;
      } else return 0;
}

uint32_t dg_obis_gettime() {
  char* code = "0-0:1.0.0(";
  char data[7];
  int idx = strstr(dg.buf, code) - dg.buf;
      if (idx > 0){
        memcpy(data, dg.buf + idx + strlen(code) + 6, 6);
        data[6]=0;
        uint32_t val = 0;
          try { val = std::stoi(data, nullptr, 10); } catch(...) {}
          return val;
      } else return 0;
}

uint32_t dg_obis_decode(char *code, char *unit) {
  //char st[100];
  int idx = strstr(dg.buf, code) - dg.buf;
      if (idx > 0){
        idx += strlen(code);
        int idx2 = strstr(dg.buf + idx, unit) - dg.buf - idx;
        if ((idx2 > 0) && (idx2 < 100)) {
          char sub[100];
          strncpy(sub, dg.buf+idx, idx2); sub[idx2]='\0';
          int idxComma = strstr(sub, ".") - sub; if (idxComma>0) memmove(&sub[idxComma], &sub[idxComma + 1], strlen(sub) - idxComma);
          uint32_t val = 0;
          try { val = std::stoi(sub, nullptr, 10); } catch(...) {}
          //sprintf(st, "\n%i %i %i %s\n", idx, idx2, val, sub); cli_client.write(st);
          return val;
        } else return 0;
      } else return 0;
}

bool dg_obis_update(char *code, char *unit, uint32_t value, uint8_t int_len, uint8_t dec_len) {
  uint32_t dec = 1;
  for (unsigned int i = 0; i < dec_len; i++) dec *= 10;
  int idx = strstr(dg.ModP1, code) - dg.ModP1;
      if ((idx > 0) && (int_len < 10) && (dec_len < 4)) {
        idx += strlen(code);
        int idx2 = strstr(dg.ModP1 + idx, unit) - dg.ModP1 - idx;
        if (idx2 > 0) {
          char fmt[5];
          char val[10];
          sprintf(fmt, "%%%02ii", int_len);
          sprintf(val, fmt, value / dec);
          memcpy(dg.ModP1+idx, val, int_len);
          sprintf(fmt, "%%%02ii", dec_len);
          sprintf(val, fmt, value % dec);
          memcpy(dg.ModP1+idx+int_len+1, val, dec_len);
          return true;
        } else return false;
      } else return false;
}

void setup() {
  // Init serial port
  Serial.begin(115200);  

  EEPROM.begin(4096);
  data_load();
  
  // Set Digital I/O and Interrupt handlers
  
  // LED
  pinMode(rx_led, OUTPUT);
  digitalWrite(rx_led, HIGH);
  //pinMode(tx_led, OUTPUT);
  //digitalWrite(tx_led, HIGH);

  // Start WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  // Init OTA
  OTAsetup();
  OTAbegin();

}



void loop() {
  int max_read = 10;
  char ch;

  // every seconds processing
  currentmillis = millis();
  if ((lastmillis + 1000 < currentmillis) || (currentmillis + 1000000 < lastmillis)) {

    // time management
    uptime ++;
    lastmillis += 1000;

    if (safecnt > 0) safecnt --;

    // network management
    if (!wifi_connected) {
      if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        dbgdsp = "WiFi connected - IP ";
        dbgdsp.append(WiFi.localIP().toString().c_str());
        // start MQTT client
        mqtt_client.setServer(MQTT_IP, 1883);
        mqtt_client.setCallback(mqtt_callback);
        // start telnet server
        cli_server.begin();
        // start P1 server
        p1_server.begin();
        // start P1 Mod server
        pm1_server.begin();
      }
    }
    else {
      if (MQTT_ON != 0) {
        if (!mqtt_connected){
          if (mqtt_client.connect("ESP8266-P1", MQTT_USER, MQTT_PASS, MQTT_LWT, 1, true, "offline")) {
            mqtt_connected = true;
            dbgdsp = "MQTT connected";
            mqtt_client.publish(MQTT_LWT, "online", true);
            mqtt_client.subscribe(MQTT_TOPIC_SUB);
          } else {
            dbgdsp = "MQTT client connection failed";
          }
        } else if (! mqtt_client.connected()) {
          mqtt_connected = false;
          dbgdsp = "MQTT client disconnected";
        }
      }

      if (WiFi.status() != WL_CONNECTED) {
        wifi_connected = false;
        dbgdsp = "WiFi disconnected";
      }
    }

    // data management
    bool updated = (memcmp(&cfg, &cfg_old, sizeof(cfg)) != 0);
    if (updated) {
      if (eeprom_to_save <= 0) data_save(); else eeprom_to_save--;
    } else eeprom_to_save = 60;
  }

  // every loop processing
  mqtt_client.loop();
  process_cli(false, true);


  // once per loop processing
  switch (loopidx++)
  {
  case 0:  // Process OTA
    OTAhandle();
    break;

  case 1:  // Process telnet clients

    // Management client
    if (!netcli_connected) { cli_client = cli_server.available(); }
    if (cli_client) {
      if (cli_client.connected())
        if (netcli_disconnect == true) {
          netcli_connected = false;
          netcli_disconnect = false;
          dbgdsp = "Network client disconnected";
          cli_client.stop();
        } else
        if (netcli_connected == false) {
          netcli_connected = true;
          netcli_disconnect = false;
          dbgdsp = "Network client connected";
          cli_client.write(0xFF);
          cli_client.write(0xFC);
          cli_client.write(0x22);
        }
      if (!cli_client.connected()) {
        netcli_connected = false;
        dbgdsp = "Network client lost";
      }
    } else if (netcli_connected) {
      netcli_connected = false;
      dbgdsp = "Network client lost";
    }

    // P1 Client cnx
    if (!p1_connected) p1_client = p1_server.available();
    if (p1_client) {
      if (!p1_connected && p1_client.connected()) {
          p1_connected = true;
          dbgdsp = "P1 client connected";
      }
      if (p1_connected && !p1_client.connected()) {
        p1_connected = false;
        dbgdsp = "P1 client disconnected";
      }
    } else if (p1_connected) {
      p1_connected = false;
      dbgdsp = "P1 client lost";
    }

    // P1 Mod client cnx
    if (!pm1_connected) pm1_client = pm1_server.available();
    if (pm1_client) {
      if (!pm1_connected && pm1_client.connected()) {
          pm1_connected = true;
          dbgdsp = "P1 Mod client connected";
      }
      if (pm1_connected && !pm1_client.connected()) {
        pm1_connected = false;
        dbgdsp = "P1 Mod client disconnected";
      }
    } else if (pm1_connected) {
      pm1_connected = false;
      dbgdsp = "P1 Mod client lost";
    }

    break;

  case 2: // Process Serial RX (Incoming P1 port)
    if (safecnt == 0) {
      while ((max_read-- > 0) && Serial.available()) {
          ch = Serial.read();
          //cli_client.write(ch);  // debug
          if (ch == '/') {
            dg.idx=0;
            dg.buf[dg.idx++]=ch;
          } else
          if ((dg.idx > 0) && (dg.idx < 2048)) {
            dg.buf[dg.idx++]=ch;
          }
          if (dg.idx_crc > 0) {
            dg.crc_received[4-dg.idx_crc]=ch;
            dg.crc_received[4] = 0;
            dg.idx_crc--;
            if (dg.idx_crc == 0) {
              dg.buf[dg.idx]=0;
              dg.received = true;
              dg.decoded = false;
              dg.sent = false;
              crc_compute(&dg);
              dg.crc_valid = (strcmp(dg.crc_computed, dg.crc_received) == 0);
              //dbgdsp = dg.crc_computed; dbgdsp += " "; dbgdsp += dg.crc_received; dbgdsp += " "; dbgdsp += dg.crc_valid;
            }
          }
          if (ch == '!') {
            dg.idx_crc = 4;
          }
      }
    }
    break;

  case 3:  // process received datagram
    if (dg.received && dg.crc_valid) {
      dg.P_consumed = dg_obis_decode("1-0:1.7.0(", "*kW)");
      dg.P_injected = dg_obis_decode("1-0:2.7.0(", "*kW)");
      dg.P = dg.P_consumed - dg.P_injected;
      dg.E_consumed_1 = dg_obis_decode("1-0:1.8.1(", "*kWh)");
      dg.E_consumed_2 = dg_obis_decode("1-0:1.8.2(", "*kWh)");
      dg.E_consumed =  dg.E_consumed_1 + dg.E_consumed_2;
      dg.E_injected_1 = dg_obis_decode("1-0:2.8.1(", "*kWh)");
      dg.E_injected_2 = dg_obis_decode("1-0:2.8.2(", "*kWh)");
      dg.E_injected = dg.E_injected_1 + dg.E_injected_2;

      dg.CurrentDate = dg_obis_getdate();
      dg.CurrentTime = dg_obis_gettime();
      dg.QuarterTime = dg.CurrentTime % 100 + 60 * (((dg.CurrentTime / 100) % 100) % 15);
      dg.CurrentPeak = dg_obis_decode("1-0:1.4.0(", "*kW)");
      if (dg.QuarterTime > 0) dg.CurrentPeak = dg.CurrentPeak * 900 / dg.QuarterTime;
      if (dg.QuarterTime < dg_old.QuarterTime) dg.LastPeak = dg_old.CurrentPeak;

      dg.U_L1 = dg_obis_decode("1-0:32.7.0(", "*V)");
      dg.U_L2 = dg_obis_decode("1-0:52.7.0(", "*V)");
      dg.U_L3 = dg_obis_decode("1-0:72.7.0(", "*V)");
      dg.I_L1 = dg_obis_decode("1-0:31.7.0(", "*A)");
      dg.I_L2 = dg_obis_decode("1-0:51.7.0(", "*A)");
      dg.I_L3 = dg_obis_decode("1-0:71.7.0(", "*A)");
      dg.P_act_L1 = dg_obis_decode("1-0:21.7.0(", "*kW)") - dg_obis_decode("1-0:22.7.0(", "*kW)");
      dg.P_act_L2 = dg_obis_decode("1-0:41.7.0(", "*kW)") - dg_obis_decode("1-0:42.7.0(", "*kW)");
      dg.P_act_L3 = dg_obis_decode("1-0:61.7.0(", "*kW)") - dg_obis_decode("1-0:62.7.0(", "*kW)");
      dg.P_ap_L1 = dg.U_L1 * dg.I_L1;
      dg.P_ap_L2 = dg.U_L2 * dg.I_L2;
      dg.P_ap_L3 = dg.U_L3 * dg.I_L3;
      dg.P_cos_L1 = (dg.P_act_L1 != 0 ? (100 * dg.P_ap_L1 / dg.P_act_L1) : 1);
      dg.P_cos_L2 = (dg.P_act_L2 != 0 ? (100 * dg.P_ap_L2 / dg.P_act_L2) : 1);
      dg.P_cos_L3 = (dg.P_act_L3 != 0 ? (100 * dg.P_ap_L3 / dg.P_act_L3) : 1);
      
    //  dg.I_Mod_L1 = dg.I_L1 + cfg.I_Shift*100;  if (dg.I_Mod_L1 > cfg.I_Max_meter*100) dg.I_Mod_L1 = cfg.I_Max_meter*100;
    //  dg.I_Mod_L2 = dg.I_L2 + cfg.I_Shift*100;  if (dg.I_Mod_L2 > cfg.I_Max_meter*100) dg.I_Mod_L2 = cfg.I_Max_meter*100;
    //  dg.I_Mod_L3 = dg.I_L3 + cfg.I_Shift*100;  if (dg.I_Mod_L3 > cfg.I_Max_meter*100) dg.I_Mod_L3 = cfg.I_Max_meter*100;
      
      //dg.P_Mod_act_L1 = dg.U_L1 * dg.I_L1 / 1000;
      //dg.P_Mod_act_L2 = dg.U_L2 * dg.I_L2 / 1000;
      //dg.P_Mod_act_L3 = dg.U_L3 * dg.I_L3 / 1000;

      p1_copytoorig(&dg);
      p1_copytomod(&dg);
    //  dg_obis_update("1-0:31.7.0(", "*A)", dg.I_Mod_L1, 3, 2);
    //  dg_obis_update("1-0:51.7.0(", "*A)", dg.I_Mod_L2, 3, 2);
    //  dg_obis_update("1-0:71.7.0(", "*A)", dg.I_Mod_L3, 3, 2);
      crc_add(dg.ModP1);

      char st[200];
      
      if (cli_dspEnergy) {
        sprintf(st, "\n\rE_Cons:%9i (%i + %i)\n\rE_Inj :%9i (%i + %i)",
                    dg.E_consumed, dg.E_consumed_1, dg.E_consumed_2, dg.E_injected, dg.E_injected_1, dg.E_injected_2
                    ); cli_client.write(st);
        cli_dspEnergy = false;
      }

      if (cli_dspPower) {
        sprintf(st, "\n\rP_Cons:%7i\n\rP_Inj :%7i\n\rU     : %3i %3i %3i\n\rI      : %3i %3i %3i",
                    dg.P_consumed, dg.P_injected,
                    dg.U_L1, dg.U_L2, dg.U_L3, dg.I_L1, dg.I_L2, dg.I_L3
                    ); cli_client.write(st);
        cli_dspPower = false;
      }

      if (cli_dspPeak) {
        sprintf(st, "\r\nDate:%06i\r\nTime:%06i\r\nQuarterTime:%03i\r\nCurrent Peak Pwr :%7i\r\nLast Peak Pwr: %7i",
                    dg.CurrentDate, dg.CurrentTime, dg.QuarterTime,
                    dg.CurrentPeak, dg.LastPeak
                    ); cli_client.write(st);
        cli_dspPeak = false;
      }

      if (cli_dspOrigP1) {
        cli_client.write(dg.OrigP1);
        cli_dspOrigP1 = false;
      }

      if (cli_dspModP1) {
        cli_client.write(dg.ModP1);
        cli_dspModP1 = false;
      }

      if (p1_connected) {
        if (p1_client.connected() && cfg.send_p1) p1_client.write(dg.OrigP1);
      }

      if (pm1_connected) {
        if (pm1_client.connected() && cfg.send_pm1) pm1_client.write(dg.ModP1);
      }

      if (cfg.send_serial) {
        Serial.write(dg.ModP1);
      }

      dg.received = false;
      dg.decoded = true;
    }
    break;

  case 4:  // Process MQTT
    process_mqtt();
    break;
 
  default: // Loop state machine
    loopidx = 0;
    break;
  }

}