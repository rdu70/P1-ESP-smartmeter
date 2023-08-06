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

// Include specific hardware libs
// #include <ELECHOUSE_CC1101_SRC_DRV.h>

// Include project specific headers
#include "cred.h"

// Timer
Ticker Timer1;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
WiFiClient cli_client;
WiFiServer cli_server(23);

// Some constants
#define MAX_REMOTE 40     // Maximum number of remote
#define MQTT_TOPIC "home/smartmeter/"
#define MQTT_TOPIC_SUB "home/smartmeter/cmd/#"
#define MQTT_LWT "home/smartmeter/availability"

// Hardware I/O
int rx_pin = D2;  // D2
int tx_pin = D1;  // D1
int rx_led = D3;
int tx_led = D4;

// Timing
// #define FREQ 433.92

// Receiving vars
#define max_frame_bit 80
volatile bool rx_frame_buffer[max_frame_bit + 1];
volatile uint8_t rx_frame_received[(max_frame_bit / 8) + 1];
volatile bool rx_done = false;
volatile uint8_t rx_bit_cnt = 0;
volatile uint8_t rx_preamble_cnt = 0;
volatile uint8_t rx_state = 0;
volatile bool rx_skip_next_edge = false;
volatile bool rx_in_frame = false;
volatile bool rx_last_bit = false;
volatile unsigned long rx_last_uS = 0;
volatile unsigned long rx_current_uS = 0;
volatile unsigned long rx_elapsed_uS = 0;

// Transmitting vars
volatile uint8_t tx_frame_tosend[(max_frame_bit / 8) + 1];
volatile bool tx_frame_buffer[max_frame_bit];
volatile bool tx_start = false;
volatile bool tx_done = true;
volatile bool tx_protocol = 1;
volatile bool tx_newtiming = false;
volatile uint8_t tx_state = 0;
volatile uint8_t tx_preamb = 0;
volatile uint8_t tx_bit_tosend = 56;
volatile uint8_t tx_bit_cnt = 0;
volatile bool tx_next_bit = 0;
volatile uint8_t tx_repeat = 0;
volatile bool tx_cc1101mode = false;

// cli vars
std::string line = "";
std::string dbgdsp = "";

// netcli vars
bool netcli_connected = false;
bool netcli_disconnect = false;

// Network vars
bool wifi_connected = false;
bool mqtt_connected = false;
uint8_t eeprom_to_save = 0;
uint8_t cmd_clear = 255;

// Remote control storage
struct RC {
  uint8_t protocol = 1;  // default EV protocol
  uint64_t id = 0;
  bool updated = false;
};

RC rc[MAX_REMOTE];
RC rx_rc;
RC tx_rc;
RC tmp_rc;

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
  uint32_t P=0;
  uint32_t U_L1 = 0;
  uint32_t U_L2 = 0;
  uint32_t U_L3 = 0;
  uint32_t I_L1 = 0;
  uint32_t I_L2 = 0;
  uint32_t I_L3 = 0;
  uint32_t P_act_L1 = 0;
  uint32_t P_act_L2 = 0;
  uint32_t P_act_L3 = 0;
  uint32_t P_ap_L1 = 0;
  uint32_t P_ap_L2 = 0;
  uint32_t P_ap_L3 = 0;
  uint32_t P_cos_L1 = 0;
  uint32_t P_cos_L2 = 0;
  uint32_t P_cos_L3 = 0;
  char buf[2048];
};

DG dg;
DG dg_old;

// Other vars
unsigned long currentmillis = millis();
unsigned long lastmillis = currentmillis;
uint32_t uptime = 0;
uint8_t loopidx = 0;
int safecnt = 20;
char uptime_txt[48];



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


void data_save() {
  for (uint8_t idx = 0; idx < MAX_REMOTE; idx++) { rc[idx].updated = false; EEPROM.put(idx*50, rc[idx]); }
  EEPROM.put(2000, rx_rc);
  EEPROM.put(2100, tx_rc);
  EEPROM.commit();
  dbgdsp = "Data saved";
}

void data_load() {
  for (uint8_t idx = 0; idx < MAX_REMOTE; idx++) EEPROM.get(idx*50, rc[idx]);
  EEPROM.get(2000, rx_rc);
  EEPROM.get(2100, tx_rc);
  dbgdsp = "Data loaded";
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
 
  if ((cmd == "show") || (cmd == "sh")) {
    bool found = false;
    if (param1 == "uptime") {
      uptime_to_text("Uptime         : ", "");
      cli_print(uptime_txt, true, ser, net);
    }
  }
  if ((cmd == "copy") || (cmd == "cp")) {
    if ((p1 >= 0) && (p1 < MAX_REMOTE)) tmp_rc = rc[p1];
    else if (param1 == "rx") tmp_rc = rx_rc;
    else if (param1 == "tx") tmp_rc = tx_rc;
    if ((p2 >= 0) && (p2 < MAX_REMOTE)) rc[p2] = tmp_rc;
    else if (param2 == "rx") rx_rc = tmp_rc;
    else if (param2 == "tx") tx_rc = tmp_rc;
  }
  if (cmd == "save") data_save();
  if (cmd == "load") data_load();
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
        dg.sent = true;
        dg_old = dg;
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
  int id = 0;
  try { id = std::stoi(topic, nullptr, 0); } catch(...) {}

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
  }else { cli_client.write('E');}
  sprintf(dg->crc_computed, "%04X", crc);
}

uint32_t dg_decode_obis(char *code, char *unit) {
  int idx = strstr(dg.buf, code) - dg.buf;
      if (idx > 0){
        idx += strlen(code);
        int idx2 = strstr(dg.buf + idx, unit) - dg.buf - idx;
        char st[100];
        if (idx2 > 0) {
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
void setup() {
  // Init serial port
  Serial.begin(115200);  
  Serial.print("\nP1 Smart controller\n");


  Serial.print("> ");

  EEPROM.begin(4096);
  data_load();
  
  // Set Digital I/O and Interrupt handlers
  
  // LED
  pinMode(rx_led, OUTPUT);
  digitalWrite(rx_led, HIGH);
  pinMode(tx_led, OUTPUT);
  digitalWrite(tx_led, HIGH);

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
      }
    }
    else {
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

      if (WiFi.status() != WL_CONNECTED) {
        wifi_connected = false;
        dbgdsp = "WiFi disconnected";
      }
    }

    // data management
    bool updated = false;
    for (uint8_t idx = 0; idx < MAX_REMOTE; idx++) {
      if (rc[idx].updated) updated = true;
    }
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
  case 1:  // Process telnet client
    if (!netcli_connected) cli_client = cli_server.available();
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
    break;
  case 2: // Process RX
    if (safecnt == 0) {
      while ((max_read-- > 0) && Serial.available()) {
          ch = Serial.read();
          //cli_client.write(ch);
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
  case 3:
    // process dg
    if (dg.received && dg.crc_valid) {
      dg.P_consumed = dg_decode_obis("1-0:1.7.0(", "*kW)");
      dg.P_injected = dg_decode_obis("1-0:2.7.0(", "*kW)");
      dg.P = dg.P_consumed - dg.P_injected;
      dg.E_consumed_1 = dg_decode_obis("1-0:1.8.1(", "*kWh)");
      dg.E_consumed_2 = dg_decode_obis("1-0:1.8.2(", "*kWh)");
      dg.E_consumed =  dg.E_consumed_1 + dg.E_consumed_2;
      dg.E_injected_1 = dg_decode_obis("1-0:2.8.1(", "*kWh)");
      dg.E_injected_2 = dg_decode_obis("1-0:2.8.2(", "*kWh)");
      dg.E_injected = dg.E_injected_1 + dg.E_injected_2;
      dg.U_L1 = dg_decode_obis("1-0:32.7.0(", "*V)");
      dg.U_L2 = dg_decode_obis("1-0:52.7.0(", "*V)");
      dg.U_L3 = dg_decode_obis("1-0:72.7.0(", "*V)");
      dg.I_L1 = dg_decode_obis("1-0:31.7.0(", "*A)");
      dg.I_L2 = dg_decode_obis("1-0:51.7.0(", "*A)");
      dg.I_L3 = dg_decode_obis("1-0:71.7.0(", "*A)");
      dg.P_act_L1 = dg_decode_obis("1-0:21.7.0(", "*kW)") - dg_decode_obis("1-0:22.7.0(", "*kW)");
      dg.P_act_L2 = dg_decode_obis("1-0:41.7.0(", "*kW)") - dg_decode_obis("1-0:42.7.0(", "*kW)");
      dg.P_act_L3 = dg_decode_obis("1-0:61.7.0(", "*kW)") - dg_decode_obis("1-0:62.7.0(", "*kW)");
      dg.P_ap_L1 = dg.U_L1 * dg.I_L1;
      dg.P_ap_L2 = dg.U_L2 * dg.I_L2;
      dg.P_ap_L3 = dg.U_L3 * dg.I_L3;
      dg.P_cos_L1 = (dg.P_act_L1 != 0 ? (100 * dg.P_ap_L1 / dg.P_act_L1) : 1);
      dg.P_cos_L2 = (dg.P_act_L2 != 0 ? (100 * dg.P_ap_L2 / dg.P_act_L2) : 1);
      dg.P_cos_L3 = (dg.P_act_L3 != 0 ? (100 * dg.P_ap_L3 / dg.P_act_L3) : 1);
      char st[200];
      sprintf(st, "\n\rP_Cons:%7i\n\rP_Inj :%7i\n\rE_Cons:%9i (%i + %i)\n\rE_Inj :%9i (%i + %i)\n\rU     : %3i %3i %3i\n\rI      : %3i %3i %3i",
                  dg.P_consumed, dg.P_injected, dg.E_consumed, dg.E_consumed_1, dg.E_consumed_2, dg.E_injected, dg.E_injected_1, dg.E_injected_2,
                  dg.U_L1, dg.U_L2, dg.U_L3, dg.I_L1, dg.I_L2, dg.I_L3
                  ); cli_client.write(st);
      
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