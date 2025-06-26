#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/compass.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define TIME_STEP 64
#define GRID_SIZE 8  
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 100
#define MAX_OPEN_NODES 1000
#define SPEED 4.0

typedef struct {
  int x, y;
} Point;

typedef struct {
  int x, y;
  int g, h, f;
  int parent_index;
} Node;

typedef enum {
  PLANNING,
  TURNING,
  MOVING_FORWARD,
  AVOIDING_OBSTACLE,
  IDLE,
  GOAL_REACHED
} RobotState;

int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
  Node open_list[MAX_OPEN_NODES];
  int open_count = 0;
  Node closed_list[GRID_SIZE * GRID_SIZE];
  int closed_count = 0;
  
  Node start_node = {start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1};
  start_node.f = start_node.g + start_node.h;
  open_list[open_count++] = start_node;
  
  while (open_count > 0) {
    int best_index = 0;
    for (int i = 1; i < open_count; i++) {
      if (open_list[i].f < open_list[best_index].f)
        best_index = i;
    }
    
    Node current = open_list[best_index];
    for (int i = best_index; i < open_count - 1; i++)
      open_list[i] = open_list[i + 1];
    open_count--;
    
    closed_list[closed_count++] = current;
    
    if (current.x == goal.x && current.y == goal.y) {
      int length = 0;
      Node n = current;
      while (n.parent_index != -1 && length < max_path_len) {
        path[length++] = (Point){n.x, n.y};
        n = closed_list[n.parent_index];
      }
      path[length++] = (Point){start.x, start.y};
      
      // Reverse path
      for (int i = 0; i < length / 2; i++) {
        Point temp = path[i];
        path[i] = path[length - i - 1];
        path[length - i - 1] = temp;
      }
      return length;
    }
    
    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    
    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
        continue;
      if (grid[nx][ny] == 1)
        continue;
      
      int in_closed = 0;
      for (int i = 0; i < closed_count; i++) {
        if (closed_list[i].x == nx && closed_list[i].y == ny) {
          in_closed = 1;
          break;
        }
      }
      if (in_closed) continue;
      
      int g = current.g + 1;
      int h = heuristic(nx, ny, goal.x, goal.y);
      int f = g + h;
      
      int in_open = -1;
      for (int i = 0; i < open_count; i++) {
        if (open_list[i].x == nx && open_list[i].y == ny) {
          in_open = i;
          break;
        }
      }
      
      if (in_open != -1) {
        if (f < open_list[in_open].f) {
          open_list[in_open].g = g;
          open_list[in_open].h = h;
          open_list[in_open].f = f;
          open_list[in_open].parent_index = closed_count - 1;
        }
      } else if (open_count < MAX_OPEN_NODES) {
        Node neighbor = {nx, ny, g, h, f, closed_count - 1};
        open_list[open_count++] = neighbor;
      }
    }
  }
  return 0;
}

int main() {
  wb_robot_init();
  
  double left_speed = 1.0;
  double right_speed = 1.0;
  int i;
  
  // --- Inicialización de Dispositivos ---
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  // --- Inicialización de Variables de Navegación ---
  int grid[GRID_SIZE][GRID_SIZE] = {0};
  Point path[100];
  int path_length = 0;
  int replan_counter = 0;
  
  RobotState current_state = IDLE;
  
  // Variables declaradas fuera del bucle para evitar problemas de scope
  double robot_bearing = 0.0;
  bool ds_detect_near = false;
  double ds_values[2] = {0.0, 0.0};
  bool goal_reached = false;
  Point goal = {GRID_SIZE - 2, GRID_SIZE - 2};  // Objetivo fijo
  double goal_tolerance = 0.3;  // Tolerancia para considerar que llegó al objetivo
  
  while (wb_robot_step(TIME_STEP) != -1) {
    
    // --- Limpieza y Lectura de Sensores ---
    for (int x = 0; x < GRID_SIZE; x++) {
      for (int y = 0; y < GRID_SIZE; y++) {
        grid[x][y] = 0;
      }
    }
    
    // Reset detection flag
    ds_detect_near = false;
    
    // Lectura de sensores de distancia
    for (i = 0; i < 2; i++) {
      ds_values[i] = wb_distance_sensor_get_value(ds[i]);
      if (ds_values[i] < 950.0) {
        ds_detect_near = true;
      }
    }
    
    // Lectura de GPS
    const double *pose = wb_gps_get_values(gps);
    double robot_x = pose[0];  // Cambiado de float a double
    double robot_y = pose[2];  // Cambiado de float a double
    
    // Lectura de compass (movido aquí para estar siempre disponible)
    const double *compass_vals = wb_compass_get_values(compass);
    robot_bearing = atan2(compass_vals[2], compass_vals[0]);
    
    // --- Verificar si llegó al objetivo ---
    double goal_world_x = (goal.x - GRID_SIZE / 2.0) * CELL_SIZE;
    double goal_world_y = (goal.y - GRID_SIZE / 2.0) * CELL_SIZE;
    double distance_to_goal = sqrt(pow(robot_x - goal_world_x, 2) + pow(robot_y - goal_world_y, 2));
    
    if (distance_to_goal < goal_tolerance && !goal_reached) {
      goal_reached = true;
      current_state = GOAL_REACHED;
      printf("\n🎯 ¡¡¡ OBJETIVO ALCANZADO !!! 🎯\n");
      printf("Distancia al objetivo: %.3f metros\n", distance_to_goal);
      printf("Posición objetivo: (%.2f, %.2f)\n", goal_world_x, goal_world_y);
      printf("Posición robot: (%.2f, %.2f)\n", robot_x, robot_y);
      fflush(stdout);
    }
    
    // Lectura de LIDAR
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    
    bool lidar_detect_near = false;
    for (int i = 0; i < resolution; i++) {
      double angle = -fov / 2 + i * (fov / resolution);
      double dist = ranges[i];
      
      if (isinf(dist)) continue;
      
      if (dist < 1.0) {
        // Usar robot_bearing para orientar correctamente los obstáculos
        double obs_x = robot_x + dist * cos(angle + robot_bearing);
        double obs_y = robot_y + dist * sin(angle + robot_bearing);
        
        int cell_x = (int)((obs_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        int cell_y = (int)((obs_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        
        if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE)
          grid[cell_x][cell_y] = 1;
      }
      
      if (dist < 0.15)
        lidar_detect_near = true;
    }
    
    // --- Planificación de Ruta Periódica ---
    replan_counter++;
    if (replan_counter % 50 == 0) {
      current_state = PLANNING;
      
      int start_cell_x = (int)((robot_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
      int start_cell_y = (int)((robot_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
      Point start = {start_cell_x, start_cell_y};
      
      // Asegurar que start y goal no sean obstáculos
      if (start_cell_x >= 0 && start_cell_x < GRID_SIZE && start_cell_y >= 0 && start_cell_y < GRID_SIZE) {
        grid[start.x][start.y] = 0;
        grid[goal.x][goal.y] = 0;
        path_length = plan_path(grid, start, goal, path, 100);
      } else {
        path_length = 0;
      }
    }
    
    // --- LÓGICA DE CONTROL DE MOVIMIENTO CON ESTADOS ---
    if (goal_reached) {
      // Si ya llegó al objetivo, mantenerlo quieto
      current_state = GOAL_REACHED;
      left_speed = 0.0;
      right_speed = 0.0;
    } else if (lidar_detect_near || ds_detect_near) {
      current_state = AVOIDING_OBSTACLE;
      left_speed = -1.0;
      right_speed = 1.0;
    } else if (path_length > 1) {
      int lookahead_index = 3;
      if (path_length <= lookahead_index) {
        lookahead_index = path_length - 1;
      }
      Point next_waypoint = path[lookahead_index];
      
      double waypoint_x = (next_waypoint.x - GRID_SIZE / 2.0) * CELL_SIZE;
      double waypoint_y = (next_waypoint.y - GRID_SIZE / 2.0) * CELL_SIZE;
      
      double angle_to_goal = atan2(waypoint_y - robot_y, waypoint_x - robot_x);
      
      double angle_diff = angle_to_goal - robot_bearing;
      while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
      
      double tolerance = 0.1;
      if (fabs(angle_diff) > tolerance) {
        current_state = TURNING;
        left_speed = angle_diff * 4.0;
        right_speed = -angle_diff * 4.0;
        
        // Clamp speeds
        if (left_speed > SPEED) left_speed = SPEED;
        if (left_speed < -SPEED) left_speed = -SPEED;
        if (right_speed > SPEED) right_speed = SPEED;
        if (right_speed < -SPEED) right_speed = -SPEED;
      } else {
        current_state = MOVING_FORWARD;
        left_speed = SPEED;
        right_speed = SPEED;
      }
    } else {
      current_state = IDLE;
      left_speed = 0.0;
      right_speed = 0.0;
    }
    
    // --- Aplicar velocidad a los motores ---
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
    
    // --- BLOQUE DE DIAGNÓSTICO E IMPRESIÓN ---
    printf("\n-- STATUS --\n");
    switch (current_state) {
      case PLANNING:
        printf("Estado: PLANIFICANDO NUEVA RUTA\n");
        break;
      case AVOIDING_OBSTACLE:
        printf("Estado: EVASIÓN REACTIVA DE OBSTÁCULO\n");
        break;
      case TURNING:
        printf("Estado: GIRANDO para seguir la ruta\n");
        break;
      case MOVING_FORWARD:
        printf("Estado: AVANZANDO por la ruta\n");
        break;
      case IDLE:
        printf("Estado: EN ESPERA (sin ruta o meta alcanzada)\n");
        break;
      case GOAL_REACHED:
        printf("Estado: 🎯 OBJETIVO ALCANZADO - MISIÓN COMPLETADA 🎯\n");
        break;
    }
    
    // Convertir la orientación a grados
    double robot_bearing_degrees = robot_bearing * 180.0 / M_PI;
    printf("Pos: (%.2f, %.2f) | Orient: %.0f°\n", robot_x, robot_y, robot_bearing_degrees);
    printf("Largo de Ruta Actual: %d\n", path_length);
    printf("Sensor DS Izquierdo: %.2f | DS Derecho: %.2f\n", ds_values[0], ds_values[1]);
    printf("Detección Cercana - DS: %s | LIDAR: %s\n", 
           ds_detect_near ? "SÍ" : "NO", 
           lidar_detect_near ? "SÍ" : "NO");
    printf("------------\n");
    
    fflush(stdout);
  }
  
  wb_robot_cleanup();
  return 0;
}