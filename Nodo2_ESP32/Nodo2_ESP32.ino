/*
  ACTIVIDAD 7 — Enjambre de 3 carritos con ACO (colonia de hormigas)
  NODO 2 — ESP32 · se conecta a la red del nodo 1

  Cada nodo corre su PROPIA colonia de hormigas (aco.h) y comparte su feromona
  con los demás por WiFi (UDP). El nodo 1 crea la red (modo AP) y reenvía los
  paquetes entre nodos y hacia el gemelo digital (PyBullet en Docker).

    Red WiFi:  ACO_ENJAMBRE   clave: hormigas123   (AP: 192.168.4.1, UDP 4210)

  Botón BOOT: reinicia la búsqueda en TODO el enjambre (para repetir la demo).
  LED: parpadea mientras busca, fijo cuando el nodo convergió a una ruta.
*/

#define NODO_ID 2            // 1 = ESP32 (AP + concentrador) | 2 = ESP32 | 3 = ESP32-C3 Mini

#include <WiFi.h>
#include <WiFiUdp.h>
#include "aco.h"

// ---------------- Red ----------------
const char *SSID  = "ACO_ENJAMBRE";
const char *CLAVE = "hormigas123";
const uint16_t PUERTO = 4210;
const IPAddress IP_AP(192, 168, 4, 1);
const bool ES_AP = (NODO_ID == 1);

// ---------------- Tiempos ----------------
const unsigned long ITER_MS = 600;          // una iteración de la colonia cada 600 ms (visible en el gemelo)
const unsigned long T_VECINO_MS = 5000;     // un vecino sin paquetes por 5 s se considera desconectado
const unsigned long T_PAR_MS = 10000;       // (nodo 1) olvida destinos que no hablan hace 10 s

// ---------------- Pines ----------------
#if CONFIG_IDF_TARGET_ESP32C3
const int PIN_BOOT = 9;
const int PIN_LED  = -1;                    // pon aquí el pin del LED de tu C3 si tiene (p. ej. 8)
#else
const int PIN_BOOT = 0;
const int PIN_LED  = 2;
#endif

aco::Colonia colonia;
WiFiUDP udp;
uint8_t buf[512];

unsigned long tIteracion = 0, tLed = 0, tUltimoDe[4] = {0, 0, 0, 0};
unsigned long recibidosDe[4] = {0, 0, 0, 0};
bool ledEncendido = false, wifiListo = false;

// ---------------- Concentrador (solo nodo 1) ----------------
struct Destino { IPAddress ip; uint16_t puerto; unsigned long visto; bool monitor; bool usado; };
Destino destinos[6];

void registrarDestino(const IPAddress &ip, uint16_t puerto, bool monitor) {
  int libre = -1;
  for (int i = 0; i < 6; i++) {
    if (destinos[i].usado && destinos[i].ip == ip && destinos[i].puerto == puerto) {
      destinos[i].visto = millis();
      destinos[i].monitor |= monitor;
      return;
    }
    if (libre < 0 && (!destinos[i].usado || millis() - destinos[i].visto > T_PAR_MS)) libre = i;
  }
  if (libre >= 0) {
    destinos[libre] = { ip, puerto, millis(), monitor, true };
    Serial.printf("[N1] Nuevo destino %s:%u%s\n", ip.toString().c_str(), puerto, monitor ? " (gemelo digital)" : "");
  }
}

void enviar(const IPAddress &ip, uint16_t puerto, const uint8_t *datos, size_t n) {
  udp.beginPacket(ip, puerto);
  udp.write(datos, n);
  udp.endPacket();
}

// Nodo 1: reenvía a todos los destinos activos menos al que lo mandó
void difundir(const uint8_t *datos, size_t n, const IPAddress &exIp, uint16_t exPuerto) {
  for (int i = 0; i < 6; i++) {
    Destino &d = destinos[i];
    if (!d.usado || millis() - d.visto > T_PAR_MS) continue;
    if (d.ip == exIp && d.puerto == exPuerto) continue;
    enviar(d.ip, d.puerto, datos, n);
  }
}

// ---------------- Lógica del nodo ----------------
void enviarEstado() {
  aco::Paquete p;
  colonia.empaquetar(p);
  if (ES_AP) difundir((uint8_t *)&p, sizeof(p), IPAddress(0, 0, 0, 0), 0);
  else if (wifiListo) enviar(IP_AP, PUERTO, (uint8_t *)&p, sizeof(p));
}

void reiniciarBusqueda(const char *origen) {
  colonia.reiniciar();
  Serial.printf("[N%d] *** Búsqueda reiniciada (%s) ***\n", NODO_ID, origen);
}

void procesarEntrada() {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    int leidos = udp.read(buf, sizeof(buf));
    IPAddress ip = udp.remoteIP();
    uint16_t puerto = udp.remotePort();

    if (leidos >= 3 && memcmp(buf, "SUB", 3) == 0) {               // el gemelo pide los datos
      if (ES_AP) registrarDestino(ip, puerto, true);
    } else if (leidos >= 5 && memcmp(buf, "RESET", 5) == 0) {       // reinicio del enjambre
      if (ES_AP) difundir(buf, leidos, ip, puerto);
      reiniciarBusqueda("orden recibida");
    } else if (leidos == (int)sizeof(aco::Paquete)) {               // feromona de otro nodo
      aco::Paquete p;
      memcpy(&p, buf, sizeof(p));
      if (ES_AP) {
        registrarDestino(ip, puerto, false);
        difundir(buf, leidos, ip, puerto);
      }
      if (colonia.mezclar(p) && p.nodo >= 1 && p.nodo <= 3) {
        tUltimoDe[p.nodo] = millis();
        recibidosDe[p.nodo]++;
      }
    }
  }
}

void revisarBoton() {
  static bool anterior = HIGH;
  static unsigned long tCambio = 0;
  bool lectura = digitalRead(PIN_BOOT);
  if (lectura != anterior && millis() - tCambio > 50) {
    tCambio = millis();
    anterior = lectura;
    if (lectura == LOW) {
      const char *orden = "RESET";
      if (ES_AP) difundir((const uint8_t *)orden, 5, IPAddress(0, 0, 0, 0), 0);
      else if (wifiListo) enviar(IP_AP, PUERTO, (const uint8_t *)orden, 5);
      reiniciarBusqueda("botón BOOT");
    }
  }
}

void imprimirEstado() {
  Serial.printf("[N%d] it %3u | mejor: ", NODO_ID, colonia.iteracion);
  if (colonia.largoMejor) Serial.printf("%2d pasos", colonia.pasosMejor());
  else Serial.print("  -     ");
  Serial.printf(" | preferida: %2d | en la mejor: %3.0f%% | llegan: %3.0f%% | vecinos:",
                colonia.largoPreferida - 1, colonia.enMejor * 100, colonia.exito * 100);
  for (int v = 1; v <= 3; v++) {
    if (v == NODO_ID) continue;
    bool vivo = tUltimoDe[v] && millis() - tUltimoDe[v] < T_VECINO_MS;
    Serial.printf(" N%d %s", v, vivo ? "OK" : "--");
  }
  Serial.println(colonia.convergido ? " | CONVERGIDO" : "");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);

  if (ES_AP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, CLAVE, 6, 0, 4);      // canal 6, visible, hasta 4 clientes (2 nodos + PC)
    wifiListo = true;
    Serial.printf("[N1] Red '%s' creada. IP del AP: %s\n", SSID, WiFi.softAPIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_STA);
#if CONFIG_IDF_TARGET_ESP32C3
    WiFi.setTxPower(WIFI_POWER_8_5dBm);     // muchas placas C3 pequeñas no conectan a potencia máxima
#endif
    WiFi.setAutoReconnect(true);
    WiFi.begin(SSID, CLAVE);
    Serial.printf("[N%d] Conectando a '%s'...\n", NODO_ID, SSID);
  }
  WiFi.setSleep(false);
  udp.begin(PUERTO);

  uint32_t semilla = esp_random() ^ (NODO_ID * 2654435761u);
  colonia.iniciar(NODO_ID, semilla);
  tIteracion = millis() + random(0, ITER_MS);   // desfase para que los nodos no transmitan a la vez
  Serial.printf("[N%d] Colonia lista: %d aristas, %d hormigas por iteración\n",
                NODO_ID, colonia.nAristas, aco::N_HORMIGAS);
}

void loop() {
  if (!ES_AP) {
    bool conectado = WiFi.status() == WL_CONNECTED;
    if (conectado && !wifiListo) Serial.printf("[N%d] Conectado. IP: %s\n", NODO_ID, WiFi.localIP().toString().c_str());
    if (!conectado && wifiListo) Serial.printf("[N%d] Se perdió la red, reintentando...\n", NODO_ID);
    wifiListo = conectado;
  }

  procesarEntrada();
  revisarBoton();

  if ((long)(millis() - tIteracion) >= 0) {   // la colonia corre aunque no haya red
    tIteracion += ITER_MS;
    colonia.iterar();
    enviarEstado();
    imprimirEstado();
  }

  if (PIN_LED >= 0) {                          // LED: fijo si convergió, parpadeo si busca
    if (colonia.convergido) {
      digitalWrite(PIN_LED, HIGH);
    } else if (millis() - tLed > 250) {
      tLed = millis();
      ledEncendido = !ledEncendido;
      digitalWrite(PIN_LED, ledEncendido);
    }
  }
}
