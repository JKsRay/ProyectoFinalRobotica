#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/compass.h>

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

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
*/
 
int main() {
  wb_robot_init();
  double left_speed = 1.0;
  double right_speed = 1.0;
  int i;

  //motores
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  //sensores 
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  //lidar
  
  WbDeviceTag lidar=wb_robot_get_device("lidar");
  wb_lidar_enable(lidar,TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  
  //GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  // Compass
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

  //ruta grilla
  int grid[GRID_SIZE][GRID_SIZE] = {0};
  Point path[100];
  int path_length = 0;
  int replan_counter = 0;
  
  while (wb_robot_step(TIME_STEP) != -1) {
   
   //Flags
   bool ds_detect_near=false;
   bool lidar_detect_near=false;
   
   // Limpiamos el mapa en cada paso para basar las decisiones solo en la percepción actual.
   // Esto evita la acumulación de errores y resuelve el problema de que el robot se detenga.
   for (int x = 0; x < GRID_SIZE; x++) {
        for (int y = 0; y < GRID_SIZE; y++) {
            grid[x][y] = 0;
        }
    }
   
   left_speed = 1.0;
   right_speed = 1.0;
   

   double ds_values[2];
   for (i = 0; i < 2; i++){
     ds_values[i] = wb_distance_sensor_get_value(ds[i]);
     if (ds_values[i] < 950.0){
       ds_detect_near=true;
       printf("Sensor de distancia %d detectó un obstáculo\n", i);
       break;
     }
   }

        
  
    
    //GPS
    const double *pose = wb_gps_get_values(gps);
    float robot_x = pose[0];
    float robot_y = pose[2]; // Webots usa X-Z como plano horizontal
   
    
    //lidar
    // (1) Obtener datos del LIDAR y construir mapa de obstáculos
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    
    for (int i = 0; i < resolution; i++) {
      double angle = -fov / 2 + i * (fov / resolution);
      double dist = ranges[i];
      
      //printf("Sensor lidar %d: %.2f\n", i, dist);
      
      if(isinf(dist)) continue; //ignorar infinito

      if (dist < 1.0) {
        float obs_x = robot_x + dist * cos(angle);
        float obs_y = robot_y + dist * sin(angle);

        int cell_x = (int)((obs_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        int cell_y = (int)((obs_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);

        if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE)
          grid[cell_x][cell_y] = 1;  // Marcado como ocupado
      }
      
      if(dist < 0.15)
        lidar_detect_near=true; //obstaculo muy cerca girar
    
      }
      
      replan_counter++;
      
      // Condición para replanificar solo cada cierto número de pasos
     if (replan_counter % 50 == 0) {
  
        // --- Planificación de Ruta ---  
        // 1. Convertir la posición GPS del robot a coordenadas de la grilla
        int start_cell_x = (int)((robot_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        int start_cell_y = (int)((robot_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        
        Point start = {start_cell_x, start_cell_y};
        Point goal = {GRID_SIZE - 2, GRID_SIZE - 2};
        
        //Forzamos que la celda de inicio y la de fin estén siempre libres
        grid[start.x][start.y] = 0;
        grid[goal.x][goal.y] = 0;
        
        // 2. Llamar a A* solo si estamos dentro de los límites del mapa
        if (start_cell_x >= 0 && start_cell_x < GRID_SIZE && start_cell_y >= 0 && start_cell_y < GRID_SIZE) {
            path_length = plan_path(grid, start, goal, path, 100);
        } else {
            path_length = 0; // No hay ruta si estamos fuera del mapa
        }
        
      }
              
     // --- LÓGICA DE CONTROL DE MOVIMIENTO ---

      if (lidar_detect_near || ds_detect_near) {
          // MÁXIMA PRIORIDAD: Evasión reactiva de obstáculos
          left_speed = -1.0;
          right_speed = 1.0 ;
      } else if (path_length > 1) {
          // SEGUNDA PRIORIDAD: Seguir la ruta planificada
      
          // 1. Obtener el siguiente punto (waypoint) de la ruta.
          // El path[0] es donde estamos, path[1] es el siguiente.
          int lookahead_index = 3; // experimentar con este valor, 2, 3 o 4 son buenas opciones.
          if (path_length <= lookahead_index) {
              lookahead_index = path_length - 1; // Si la ruta es corta, apunta al final.
          }
          Point next_waypoint = path[lookahead_index];
                
          // Convertir las coordenadas de la grilla del waypoint a coordenadas del mundo
          double waypoint_x = (next_waypoint.x - GRID_SIZE / 2.0) * CELL_SIZE;
          double waypoint_y = (next_waypoint.y - GRID_SIZE / 2.0) * CELL_SIZE;
      
          // 2. Calcular el ángulo hacia el waypoint
          double angle_to_goal = atan2(waypoint_y - robot_y, waypoint_x - robot_x);
      
          // 3. Obtener la orientación actual del robot desde la brújula
          const double *compass_vals = wb_compass_get_values(compass);
          // El norte en Webots está en el eje X, se utiliza atan2(norte, este)
          double robot_bearing = atan2(compass_vals[2], compass_vals[0]);
      
          // 4. Calcular la diferencia de ángulo que necesitamos corregir
          double angle_diff = angle_to_goal - robot_bearing;
      
          // Normalizar el ángulo a un rango de -PI a PI
          while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
          while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
      
          // 5. Decidir si girar o avanzar
          double tolerance = 0.1; // 0.1 radianes (aprox 5.7 grados)
          if (fabs(angle_diff) > tolerance) {
            // Si no estamos orientados, GIRAR.
            left_speed = angle_diff * 4.0; // Aumentamos un poco la reactividad del giro
            right_speed = -angle_diff * 4.0;
        
            // Limitar la velocidad para que no exceda la velocidad máxima.
            if (left_speed > SPEED) left_speed = SPEED;
            if (left_speed < -SPEED) left_speed = -SPEED;
            if (right_speed > SPEED) right_speed = SPEED;
            if (right_speed < -SPEED) right_speed = -SPEED;

          } else {
              // Si ya estamos orientados, AVANZAR.
              left_speed = SPEED;
              right_speed = SPEED;
          }
                
      } else {
          // TERCERA PRIORIDAD: No hay obstáculos y no hay ruta, nos detenemos.
          left_speed = 0.0;
          right_speed = 0.0;
      }
      
      // Aplicar la velocidad calculada a las cuatro ruedas
      wb_motor_set_velocity(wheels[0], left_speed);
      wb_motor_set_velocity(wheels[1], right_speed);
      wb_motor_set_velocity(wheels[2], left_speed);
      wb_motor_set_velocity(wheels[3], right_speed);
        
    
     
   

     // (3) Navegación hacia el siguiente punto (simplificada)
     
    
    fflush(stdout); 
    
  };

  
  wb_robot_cleanup();

  return 0;
}
