/*
  aco.h — Núcleo del algoritmo de colonia de hormigas (ACO) para el enjambre.
  C++ puro (sin funciones de Arduino) para poder probarlo también en el PC.
  Debe ser IDÉNTICO en los 3 nodos y coincidir con el laberinto de gemelo.py.
*/
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace aco {

// ---------------- Laberinto / almacén ----------------
// '#' pared, '.' pasillo, 'A' inicio, 'M' meta
const int FILAS = 9, COLS = 13;
static const char *LABERINTO[FILAS] = {
  "#############",
  "#A....#.....#",
  "#.###.#.###.#",
  "#.#.......#.#",
  "#.#.###.#.#.#",
  "#...#...#...#",
  "###.#.#####.#",
  "#.....#....M#",
  "#############",
};
const int N_CELDAS = FILAS * COLS;
const int MAX_ARISTAS = 64;
const int MAX_RUTA = 80;
const uint8_t SIN_RUTA = 0xFF;

// ---------------- Parámetros ACO ----------------
const int   N_HORMIGAS = 6;
const float ALFA = 1.0f;          // peso de la feromona
const float BETA = 0.5f;          // peso de la heurística (cercanía a la meta), bajo para que domine la feromona
const float RHO = 0.15f;          // evaporación por iteración
const float Q = 4.0f;             // feromona depositada = Q / longitud
const float ELITE = 2.0f;         // refuerzo extra a la mejor ruta conocida
const float TAU_INI = 1.0f, TAU_MIN = 0.05f, TAU_MAX = 5.0f;
const float W_MEZCLA = 0.25f;     // peso de la feromona recibida de otro nodo
const int   ITER_ESTABLE = 10;    // iteraciones sin mejorar para declarar convergencia
const float FRAC_CONVERGE = 0.75f;// fracción de hormigas que ya siguen la mejor ruta

// ---------------- Paquete de red (303 bytes, little-endian) ----------------
struct __attribute__((packed)) Paquete {
  char     magia[2];              // 'A','C'
  uint8_t  version;               // 1
  uint8_t  tipo;                  // 1 = estado
  uint8_t  nodo;                  // 1..3
  uint8_t  convergido;            // 0 buscando, 1 convergido
  uint16_t iteracion;
  uint16_t mejorLargo;            // pasos de la mejor ruta (0xFFFF = ninguna)
  uint8_t  exitoPct;              // % de hormigas que llegaron a la meta
  uint8_t  enMejorPct;            // % de hormigas que siguieron la mejor ruta
  uint8_t  nAristas;
  uint8_t  largoMejor;            // celdas de la mejor ruta
  uint8_t  largoPreferida;        // celdas de la ruta preferida (máxima feromona)
  uint16_t tau[MAX_ARISTAS];      // feromona × 1000
  uint8_t  mejor[MAX_RUTA];       // índices de celda (fila*COLS + col)
  uint8_t  preferida[MAX_RUTA];    // ruta que hoy seguiría el carrito
};

class Colonia {
public:
  // Grafo
  uint8_t abierta[N_CELDAS];
  uint8_t aristaA[MAX_ARISTAS], aristaB[MAX_ARISTAS];
  int8_t  idxArista[N_CELDAS][4];  // arista hacia derecha, abajo, izquierda, arriba
  int     nAristas = 0;
  int     inicio = 0, meta = 0;
  float   heur[N_CELDAS];

  // Estado
  float   tau[MAX_ARISTAS];
  uint8_t mejor[MAX_RUTA];  int largoMejor = 0;     // celdas
  uint8_t preferida[MAX_RUTA]; int largoPreferida = 0;  // ruta siguiendo la máxima feromona
  uint16_t iteracion = 0;
  int     sinMejora = 0;
  float   exito = 0, enMejor = 0;
  bool    convergido = false;
  uint8_t nodo = 1;

  void iniciar(uint8_t idNodo, uint32_t semilla) {
    nodo = idNodo;
    rng = semilla ? semilla : 0x12345678u;
    nAristas = 0;
    memset(idxArista, -1, sizeof(idxArista));
    int fm = 0, cm = 0;
    for (int f = 0; f < FILAS; f++)
      for (int c = 0; c < COLS; c++) {
        char ch = LABERINTO[f][c];
        abierta[f * COLS + c] = (ch != '#');
        if (ch == 'A') inicio = f * COLS + c;
        if (ch == 'M') { meta = f * COLS + c; fm = f; cm = c; }
      }
    // Aristas en orden fijo: por filas, primero derecha y luego abajo (igual en gemelo.py)
    for (int f = 0; f < FILAS; f++)
      for (int c = 0; c < COLS; c++) {
        int i = f * COLS + c;
        if (!abierta[i]) continue;
        if (c + 1 < COLS && abierta[i + 1]) agregarArista(i, i + 1, 0, 2);
        if (f + 1 < FILAS && abierta[i + COLS]) agregarArista(i, i + COLS, 1, 3);
      }
    for (int f = 0; f < FILAS; f++)
      for (int c = 0; c < COLS; c++)
        heur[f * COLS + c] = 1.0f / (1.0f + fabsf((float)(f - fm)) + fabsf((float)(c - cm)));
    reiniciar();
  }

  void reiniciar() {
    for (int e = 0; e < nAristas; e++) tau[e] = TAU_INI;
    largoMejor = 0; largoPreferida = 0; iteracion = 0; sinMejora = 0;
    exito = 0; enMejor = 0; convergido = false;
  }

  int pasosMejor() const { return largoMejor > 0 ? largoMejor - 1 : 0xFFFF; }

  // Una iteración: N_HORMIGAS buscan, evapora, deposita y refuerza la mejor ruta
  void iterar() {
    static uint8_t rutas[N_HORMIGAS][MAX_RUTA];
    int largos[N_HORMIGAS];
    int exitos = 0;
    bool mejoro = false;
    iteracion++;

    for (int k = 0; k < N_HORMIGAS; k++) {
      largos[k] = caminar(rutas[k]);
      if (largos[k] > 0) {
        exitos++;
        if (largoMejor == 0 || largos[k] < largoMejor) {
          memcpy(mejor, rutas[k], largos[k]);
          largoMejor = largos[k];
          mejoro = true;
        }
      }
    }

    for (int e = 0; e < nAristas; e++) tau[e] *= (1.0f - RHO);
    int enLaMejor = 0;
    for (int k = 0; k < N_HORMIGAS; k++) {
      if (largos[k] <= 0) continue;
      depositar(rutas[k], largos[k], Q / (largos[k] - 1));
      if (largos[k] == largoMejor) enLaMejor++;
    }
    if (largoMejor > 0) depositar(mejor, largoMejor, ELITE * Q / (largoMejor - 1));
    limitar();

    exito = (float)exitos / N_HORMIGAS;
    enMejor = (float)enLaMejor / N_HORMIGAS;
    sinMejora = mejoro ? 0 : sinMejora + 1;
    calcularPreferida();
    // Convergencia "con memoria": una vez alcanzada se mantiene hasta que aparezca una ruta mejor
    if (mejoro) convergido = false;
    if (largoMejor > 0 && sinMejora >= ITER_ESTABLE && enMejor >= FRAC_CONVERGE && largoPreferida == largoMejor)
      convergido = true;
  }

  // Ruta "codiciosa": en cada cruce toma el pasillo con más feromona.
  // Es lo que el carrito seguiría hoy; al converger coincide con la mejor ruta.
  void calcularPreferida() {
    uint8_t visitada[N_CELDAS];
    memset(visitada, 0, sizeof(visitada));
    int pos = inicio, n = 0;
    preferida[n++] = pos; visitada[pos] = 1;
    while (pos != meta && n < MAX_RUTA) {
      int elegido = -1; float mayor = -1;
      for (int d = 0; d < 4; d++) {
        int e = idxArista[pos][d];
        if (e < 0) continue;
        int v = otro(e, pos);
        if (visitada[v]) continue;
        if (tau[e] > mayor) { mayor = tau[e]; elegido = v; }
      }
      if (elegido < 0) break;
      pos = elegido;
      preferida[n++] = pos; visitada[pos] = 1;
    }
    largoPreferida = n;
  }

  // Mezcla la feromona y la mejor ruta recibidas de otro nodo
  bool mezclar(const Paquete &p) {
    if (p.magia[0] != 'A' || p.magia[1] != 'C' || p.version != 1 || p.tipo != 1) return false;
    if (p.nodo == nodo || p.nAristas != nAristas) return false;
    for (int e = 0; e < nAristas; e++) {
      float remoto = p.tau[e] / 1000.0f;
      tau[e] = (1.0f - W_MEZCLA) * tau[e] + W_MEZCLA * remoto;
    }
    limitar();
    if (p.largoMejor > 1 && p.largoMejor <= MAX_RUTA &&
        (largoMejor == 0 || p.largoMejor < largoMejor) && rutaValida(p.mejor, p.largoMejor)) {
      memcpy(mejor, p.mejor, p.largoMejor);   // adopta la mejor ruta del enjambre
      largoMejor = p.largoMejor;
      sinMejora = 0;
    }
    return true;
  }

  void empaquetar(Paquete &p) const {
    memset(&p, 0, sizeof(p));
    p.magia[0] = 'A'; p.magia[1] = 'C';
    p.version = 1; p.tipo = 1; p.nodo = nodo;
    p.convergido = convergido;
    p.iteracion = iteracion;
    p.mejorLargo = pasosMejor();
    p.exitoPct = (uint8_t)(exito * 100 + 0.5f);
    p.enMejorPct = (uint8_t)(enMejor * 100 + 0.5f);
    p.nAristas = nAristas;
    p.largoMejor = largoMejor;
    p.largoPreferida = largoPreferida;
    for (int e = 0; e < nAristas; e++) {
      float v = tau[e] * 1000.0f;
      p.tau[e] = (uint16_t)(v > 65535 ? 65535 : v);
    }
    memcpy(p.mejor, mejor, largoMejor);
    memcpy(p.preferida, preferida, largoPreferida);
  }

private:
  uint32_t rng = 1;

  uint32_t aleatorio() {            // xorshift32
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
  }
  float aleatorio01() { return (aleatorio() >> 8) * (1.0f / 16777216.0f); }

  void agregarArista(int a, int b, int dirA, int dirB) {
    if (nAristas >= MAX_ARISTAS) return;
    aristaA[nAristas] = a; aristaB[nAristas] = b;
    idxArista[a][dirA] = nAristas;
    idxArista[b][dirB] = nAristas;
    nAristas++;
  }

  int otro(int e, int celda) const { return aristaA[e] == celda ? aristaB[e] : aristaA[e]; }

  int aristaEntre(int a, int b) const {
    for (int d = 0; d < 4; d++) {
      int e = idxArista[a][d];
      if (e >= 0 && otro(e, a) == b) return e;
    }
    return -1;
  }

  // Devuelve el número de celdas de la ruta (>0 si llegó a la meta, <0 si quedó atrapada)
  int caminar(uint8_t *ruta) {
    uint8_t visitada[N_CELDAS];
    memset(visitada, 0, sizeof(visitada));
    int pos = inicio, n = 0;
    ruta[n++] = pos; visitada[pos] = 1;
    while (pos != meta && n < MAX_RUTA) {
      float peso[4]; int sig[4]; int m = 0; float suma = 0;
      for (int d = 0; d < 4; d++) {
        int e = idxArista[pos][d];
        if (e < 0) continue;
        int v = otro(e, pos);
        if (visitada[v]) continue;
        float w = powf(tau[e], ALFA) * powf(heur[v], BETA);
        peso[m] = w; sig[m] = v; m++; suma += w;
      }
      if (m == 0) return -n;            // callejón sin salida
      float r = aleatorio01() * suma;
      int elegido = sig[m - 1];
      for (int j = 0; j < m; j++) { r -= peso[j]; if (r <= 0) { elegido = sig[j]; break; } }
      pos = elegido;
      ruta[n++] = pos; visitada[pos] = 1;
    }
    return pos == meta ? n : -n;
  }

  void depositar(const uint8_t *ruta, int n, float cantidad) {
    for (int i = 0; i + 1 < n; i++) {
      int e = aristaEntre(ruta[i], ruta[i + 1]);
      if (e >= 0) tau[e] += cantidad;
    }
  }

  void limitar() {
    for (int e = 0; e < nAristas; e++) {
      if (tau[e] < TAU_MIN) tau[e] = TAU_MIN;
      if (tau[e] > TAU_MAX) tau[e] = TAU_MAX;
    }
  }

  bool rutaValida(const uint8_t *ruta, int n) const {
    if (n < 2 || ruta[0] != inicio || ruta[n - 1] != meta) return false;
    for (int i = 0; i + 1 < n; i++) {
      if (ruta[i] >= N_CELDAS || ruta[i + 1] >= N_CELDAS) return false;
      if (aristaEntre(ruta[i], ruta[i + 1]) < 0) return false;
    }
    return true;
  }
};

}  // namespace aco
