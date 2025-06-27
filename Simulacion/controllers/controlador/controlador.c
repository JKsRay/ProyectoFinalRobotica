#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- PARÁMETROS DE CONFIGURACIÓN ---
#define PASO_TIEMPO 64
#define TAMAÑO_GRILLA 8
#define TAMAÑO_CELDA 0.5
#define LONGITUD_MAXIMA_RUTA 100
#define MAXIMOS_NODOS_ABIERTOS 1000
#define VELOCIDAD 4.0
#define DISTANCIA_MINIMA_LIDAR 0.20  // Aumentado para mejor detección
#define DISTANCIA_MAXIMA_AUTO_DETECCION 0.35  // Mayor margen de seguridad
#define ESPACIO_LIBRE_PARED 0.25  // Distancia mínima segura a paredes
#define TIEMPO_ESCAPE 15  // Tiempo mínimo de maniobra de escape

// --- ESTRUCTURAS DE DATOS ---
typedef struct {
  int x, y;
} Punto;

typedef struct {
  int x, y;
  int g, h, f;
  int indice_padre;
} Nodo;

// --- ALGORITMO A* ---
int heuristica(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int planificar_ruta(int grilla[TAMAÑO_GRILLA][TAMAÑO_GRILLA], Punto inicio, Punto objetivo, Punto ruta[], int longitud_maxima_ruta) {
  Nodo lista_abierta[MAXIMOS_NODOS_ABIERTOS];
  int cantidad_abiertos = 0;
  Nodo lista_cerrada[TAMAÑO_GRILLA * TAMAÑO_GRILLA];
  int cantidad_cerrados = 0;

  Nodo nodo_inicio = {inicio.x, inicio.y, 0, heuristica(inicio.x, inicio.y, objetivo.x, objetivo.y), 0, -1};
  nodo_inicio.f = nodo_inicio.g + nodo_inicio.h;
  lista_abierta[cantidad_abiertos++] = nodo_inicio;

  while (cantidad_abiertos > 0) {
    int mejor_indice = 0;
    for (int i = 1; i < cantidad_abiertos; i++) {
      if (lista_abierta[i].f < lista_abierta[mejor_indice].f) {
        mejor_indice = i;
      }
    }

    Nodo actual = lista_abierta[mejor_indice];
    for (int i = mejor_indice; i < cantidad_abiertos - 1; i++) {
      lista_abierta[i] = lista_abierta[i + 1];
    }
    cantidad_abiertos--;
    lista_cerrada[cantidad_cerrados++] = actual;

    if (actual.x == objetivo.x && actual.y == objetivo.y) {
      int longitud = 0;
      Nodo n = actual;
      while (n.indice_padre != -1 && longitud < longitud_maxima_ruta) {
        ruta[longitud++] = (Punto){n.x, n.y};
        n = lista_cerrada[n.indice_padre];
      }
      ruta[longitud++] = (Punto){inicio.x, inicio.y};
      for (int i = 0; i < longitud / 2; i++) {
        Punto temporal = ruta[i];
        ruta[i] = ruta[longitud - i - 1];
        ruta[longitud - i - 1] = temporal;
      }
      return longitud;
    }

    const int desplazamiento_x[] = {0, 1, 0, -1};
    const int desplazamiento_y[] = {1, 0, -1, 0};
    for (int d = 0; d < 4; d++) {
      int nuevo_x = actual.x + desplazamiento_x[d];
      int nuevo_y = actual.y + desplazamiento_y[d];
      if (nuevo_x < 0 || nuevo_y < 0 || nuevo_x >= TAMAÑO_GRILLA || nuevo_y >= TAMAÑO_GRILLA || grilla[nuevo_x][nuevo_y] == 1) {
        continue;
      }

      bool en_cerrados = false;
      for (int i = 0; i < cantidad_cerrados; i++) {
        if (lista_cerrada[i].x == nuevo_x && lista_cerrada[i].y == nuevo_y) {
          en_cerrados = true;
          break;
        }
      }
      if (en_cerrados) {
        continue;
      }

      int g = actual.g + 1;
      int h = heuristica(nuevo_x, nuevo_y, objetivo.x, objetivo.y);
      int f = g + h;

      int en_abiertos = -1;
      for (int i = 0; i < cantidad_abiertos; i++) {
        if (lista_abierta[i].x == nuevo_x && lista_abierta[i].y == nuevo_y) {
          en_abiertos = i;
          break;
        }
      }

      if (en_abiertos != -1) {
        if (f < lista_abierta[en_abiertos].f) {
          lista_abierta[en_abiertos].f = f;
          lista_abierta[en_abiertos].g = g;
          lista_abierta[en_abiertos].indice_padre = cantidad_cerrados - 1;
        }
      } else if (cantidad_abiertos < MAXIMOS_NODOS_ABIERTOS) {
        Nodo vecino = {nuevo_x, nuevo_y, g, h, f, cantidad_cerrados - 1};
        lista_abierta[cantidad_abiertos++] = vecino;
      }
    }
  }
  return 0;
}

// --- DETECCIÓN DE PROXIMIDAD PELIGROSA ---
bool esta_muy_cerca_de_paredes(const float *rangos, int resolucion, double campo_vision) {
  int cantidad_peligro = 0;
  int total_validos = 0;
  
  for (int i = 0; i < resolucion; i++) {
    double distancia = rangos[i];
    if (!isinf(distancia) && !isnan(distancia) && distancia > DISTANCIA_MINIMA_LIDAR && distancia < 2.0) {
      total_validos++;
      if (distancia < ESPACIO_LIBRE_PARED) {
        cantidad_peligro++;
      }
    }
  }
  
  // Si más del 30% de las lecturas válidas están demasiado cerca
  return (total_validos > 0) && ((double)cantidad_peligro / total_validos > 0.3);
}

// --- PROGRAMA PRINCIPAL ---
int main() {
  wb_robot_init();

  // Inicialización de dispositivos
  WbDeviceTag ruedas[4];
  for (int i = 0; i < 4; i++) {
    char nombre_rueda[8];
    sprintf(nombre_rueda, "wheel%d", i + 1);
    ruedas[i] = wb_robot_get_device(nombre_rueda);
    wb_motor_set_position(ruedas[i], INFINITY);
  }

  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, PASO_TIEMPO);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, PASO_TIEMPO);
  WbDeviceTag brujula = wb_robot_get_device("compass");
  wb_compass_enable(brujula, PASO_TIEMPO);

  // Variables de estado y ruta
  int grilla[TAMAÑO_GRILLA][TAMAÑO_GRILLA] = {0};
  Punto ruta[LONGITUD_MAXIMA_RUTA];
  int longitud_ruta = 0;
  int contador_atascado = 0;
  int temporizador_escape = 0;
  bool objetivo_alcanzado = false;
  bool en_modo_escape = false;
  double x_anterior = 0.0, z_anterior = 0.0;
  int contador_posicion_sin_cambios = 0;

  printf("Controlador Anti-Oscilación v6.0 iniciado.\n");
  wb_robot_step(PASO_TIEMPO);

  // --- BUCLE PRINCIPAL DE CONTROL ---
  while (wb_robot_step(PASO_TIEMPO) != -1) {
    // 1. PERCEPCIÓN
    const double *posicion = wb_gps_get_values(gps);
    const double *valores_brujula = wb_compass_get_values(brujula);
    double x_robot = posicion[0], z_robot = posicion[2];
    double orientacion_robot = atan2(valores_brujula[2], valores_brujula[0]);

    // 2. DETECCIÓN DE MOVIMIENTO ESTANCADO
    double diferencia_posicion = sqrt(pow(x_robot - x_anterior, 2) + pow(z_robot - z_anterior, 2));
    if (diferencia_posicion < 0.02) { // Si no se ha movido significativamente
      contador_posicion_sin_cambios++;
    } else {
      contador_posicion_sin_cambios = 0;
      x_anterior = x_robot;
      z_anterior = z_robot;
    }

    // 3. MAPEO CON FILTRO MEJORADO
    for (int x = 0; x < TAMAÑO_GRILLA; x++) {
      for (int y = 0; y < TAMAÑO_GRILLA; y++) {
        grilla[x][y] = 0;
      }
    }

    const float *rangos = wb_lidar_get_range_image(lidar);
    int resolucion = wb_lidar_get_horizontal_resolution(lidar);
    double campo_vision = wb_lidar_get_fov(lidar);

    // Mapeo con filtros mejorados
    for (int i = 0; i < resolucion; i++) {
      double distancia = rangos[i];

      // FILTRO MEJORADO: Elimina lecturas inválidas Y autodetecciones
      if (isinf(distancia) || isnan(distancia) || distancia < DISTANCIA_MINIMA_LIDAR) {
        continue;
      }
      
      // Filtro adicional: ignorar lecturas demasiado cercanas (autodetección)
      if (distancia < DISTANCIA_MAXIMA_AUTO_DETECCION) {
        continue;
      }

      double angulo_relativo = -campo_vision / 2.0 + (double)i * campo_vision / (resolucion - 1.0);
      double angulo_global = orientacion_robot + angulo_relativo;

      double x_obstaculo = x_robot + distancia * cos(angulo_global);
      double z_obstaculo = z_robot + distancia * sin(angulo_global);

      int celda_x = (int)((x_obstaculo + TAMAÑO_GRILLA * TAMAÑO_CELDA / 2.0) / TAMAÑO_CELDA);
      int celda_y = (int)((z_obstaculo + TAMAÑO_GRILLA * TAMAÑO_CELDA / 2.0) / TAMAÑO_CELDA);

      if (celda_x >= 0 && celda_x < TAMAÑO_GRILLA && celda_y >= 0 && celda_y < TAMAÑO_GRILLA) {
        grilla[celda_x][celda_y] = 1;
      }
    }

    // 4. DETECCIÓN DE PROXIMIDAD PELIGROSA
    bool muy_cerca = esta_muy_cerca_de_paredes(rangos, resolucion, campo_vision);

    // 5. LÓGICA DE ESCAPE MEJORADA
    if (muy_cerca || contador_posicion_sin_cambios > 10) {
      if (!en_modo_escape) {
        en_modo_escape = true;
        temporizador_escape = 0;
        printf("¡ACTIVANDO MODO ESCAPE! Proximidad peligrosa detectada.\n");
      }
    }

    if (en_modo_escape) {
      temporizador_escape++;
      if (temporizador_escape > TIEMPO_ESCAPE && !muy_cerca && contador_posicion_sin_cambios < 5) {
        en_modo_escape = false;
        contador_atascado = 0;
        printf("Modo escape completado. Reanudando navegación normal.\n");
      }
    }

    // 6. PLANIFICACIÓN
    Punto inicio = {(int)((x_robot + TAMAÑO_GRILLA * TAMAÑO_CELDA / 2.0) / TAMAÑO_CELDA), 
                    (int)((z_robot + TAMAÑO_GRILLA * TAMAÑO_CELDA / 2.0) / TAMAÑO_CELDA)};
    Punto objetivo = {0, TAMAÑO_GRILLA - 1}; // Esquina superior izquierda exacta
    
    // VERIFICAR SI HEMOS LLEGADO AL OBJETIVO
    double x_objetivo = (objetivo.x - TAMAÑO_GRILLA / 2.0) * TAMAÑO_CELDA + TAMAÑO_CELDA / 2.0;
    double z_objetivo = (objetivo.y - TAMAÑO_GRILLA / 2.0) * TAMAÑO_CELDA + TAMAÑO_CELDA / 2.0;
    double distancia_al_objetivo = sqrt(pow(x_robot - x_objetivo, 2) + pow(z_robot - z_objetivo, 2));
    
    if (distancia_al_objetivo < 0.3) {
      objetivo_alcanzado = true;
      printf("¡OBJETIVO ALCANZADO! Distancia: %.3f\n", distancia_al_objetivo);
    }

    // ZONA SEGURA: Asegurar que las celdas críticas estén libres
    if (inicio.x >= 0 && inicio.x < TAMAÑO_GRILLA && inicio.y >= 0 && inicio.y < TAMAÑO_GRILLA) {
      grilla[inicio.x][inicio.y] = 0;
      // Liberar área más amplia alrededor del robot
      for (int desplaz_x = -1; desplaz_x <= 1; desplaz_x++) {
        for (int desplaz_y = -1; desplaz_y <= 1; desplaz_y++) {
          int x_adyacente = inicio.x + desplaz_x;
          int y_adyacente = inicio.y + desplaz_y;
          if (x_adyacente >= 0 && x_adyacente < TAMAÑO_GRILLA && y_adyacente >= 0 && y_adyacente < TAMAÑO_GRILLA) {
            grilla[x_adyacente][y_adyacente] = 0;
          }
        }
      }
    }
    grilla[objetivo.x][objetivo.y] = 0;

    longitud_ruta = planificar_ruta(grilla, inicio, objetivo, ruta, LONGITUD_MAXIMA_RUTA);

    // 7. ACCIÓN CON CONTROL ANTI-OSCILACIÓN
    double velocidad_izquierda = 0.0, velocidad_derecha = 0.0;
    
    if (objetivo_alcanzado) {
      // ¡Hemos llegado! Detener el robot
      velocidad_izquierda = 0.0;
      velocidad_derecha = 0.0;
      printf("Misión completada. Robot detenido en el objetivo.\n");
    } else if (en_modo_escape) {
      // MANIOBRA DE ESCAPE MEJORADA
      if (temporizador_escape < 8) {
        // Retroceso más prolongado y decidido
        velocidad_izquierda = -VELOCIDAD * 0.7;
        velocidad_derecha = -VELOCIDAD * 0.7;
        printf("Escape: Retrocediendo... (%d/%d)\n", temporizador_escape, TIEMPO_ESCAPE);
      } else {
        // Giro más amplio para evitar la esquina problemática
        velocidad_izquierda = VELOCIDAD * 0.8;
        velocidad_derecha = -VELOCIDAD * 0.8;
        printf("Escape: Girando para evitar obstáculo... (%d/%d)\n", temporizador_escape, TIEMPO_ESCAPE);
      }
    } else if (longitud_ruta > 1) {
      contador_atascado = 0;
      printf("Navegación normal. Ruta: %d pasos. Pos: (%d,%d) -> Meta: (%d,%d)\n", 
             longitud_ruta, inicio.x, inicio.y, objetivo.x, objetivo.y);

      // Lookahead más conservador cerca del objetivo
      int indice_punto_futuro = (distancia_al_objetivo < 1.0) ? 1 : ((longitud_ruta > 2) ? 2 : 1);
      Punto punto_navegacion = ruta[indice_punto_futuro];

      double x_punto_navegacion = (punto_navegacion.x - TAMAÑO_GRILLA / 2.0) * TAMAÑO_CELDA + TAMAÑO_CELDA / 2.0;
      double z_punto_navegacion = (punto_navegacion.y - TAMAÑO_GRILLA / 2.0) * TAMAÑO_CELDA + TAMAÑO_CELDA / 2.0;

      double angulo_al_punto_navegacion = atan2(z_punto_navegacion - z_robot, x_punto_navegacion - x_robot);
      double diferencia_angulo = angulo_al_punto_navegacion - orientacion_robot;

      while (diferencia_angulo > M_PI) diferencia_angulo -= 2 * M_PI;
      while (diferencia_angulo < -M_PI) diferencia_angulo += 2 * M_PI;

      if (fabs(diferencia_angulo) > 0.08) {
        // Giro más suave y controlado
        double factor_giro = (distancia_al_objetivo < 1.0) ? 3.0 : 4.0;
        velocidad_izquierda = diferencia_angulo * factor_giro;
        velocidad_derecha = -diferencia_angulo * factor_giro;
      } else {
        // Avance adaptativo según la proximidad al objetivo
        double factor_velocidad = (distancia_al_objetivo < 1.0) ? 0.4 : 0.6;
        velocidad_izquierda = VELOCIDAD * factor_velocidad;
        velocidad_derecha = VELOCIDAD * factor_velocidad;
      }
    } else {
      contador_atascado++;
      printf("¡RUTA BLOQUEADA! Activando protocolo de emergencia... (%d)\n", contador_atascado);
      
      // Protocolo de emergencia simplificado
      if (contador_atascado % 20 < 10) {
        velocidad_izquierda = -VELOCIDAD * 0.6;
        velocidad_derecha = -VELOCIDAD * 0.6;
      } else {
        velocidad_izquierda = VELOCIDAD * 0.7;
        velocidad_derecha = -VELOCIDAD * 0.7;
      }
    }

    // Limitación de velocidad
    velocidad_izquierda = fmax(-VELOCIDAD, fmin(VELOCIDAD, velocidad_izquierda));
    velocidad_derecha = fmax(-VELOCIDAD, fmin(VELOCIDAD, velocidad_derecha));

    // Asignar velocidad a las ruedas
    wb_motor_set_velocity(ruedas[0], velocidad_izquierda);
    wb_motor_set_velocity(ruedas[1], velocidad_derecha);
    wb_motor_set_velocity(ruedas[2], velocidad_izquierda);
    wb_motor_set_velocity(ruedas[3], velocidad_derecha);

    fflush(stdout);
  }

  wb_robot_cleanup();
  return 0;
}