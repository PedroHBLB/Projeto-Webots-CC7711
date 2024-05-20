/*
 * File:          Controlador.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>
#include <math.h>

/*
 * You may want to add macros here.
*/
#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10

#define TOLERANCIA 0.001

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
*/

int main(int argc, char **argv){

  int i = 0;
  int counter = 0;
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
  double distancia = 200;

  /* necessary to initialize webots stuff */
  for (i = 0; i < 256; i++)
    texto[i] = '0';
  wb_robot_init();
  
  // Pega o componente da caixa pelo parâmetro DEF = 'Leve'
  WbNodeRef box_node = wb_supervisor_node_get_from_def("Leve");
  
  // Verifica se essa caixa existe
  if (box_node == NULL) {
      printf("Erro: não foi possível encontrar a caixa.\n");
      return 1;
  }

  // configurei MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  // configurei Sensores de Proximidade
  WbDeviceTag SensorProx[QtddSensoresProx];
  SensorProx[0] = wb_robot_get_device("ps0");
  SensorProx[1] = wb_robot_get_device("ps1");
  SensorProx[2] = wb_robot_get_device("ps2");
  SensorProx[3] = wb_robot_get_device("ps3");
  SensorProx[4] = wb_robot_get_device("ps4");
  SensorProx[5] = wb_robot_get_device("ps5");
  SensorProx[6] = wb_robot_get_device("ps6");
  SensorProx[7] = wb_robot_get_device("ps7");

  wb_distance_sensor_enable(SensorProx[0], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[1], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[2], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[3], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[4], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[5], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[6], TIME_STEP);
  wb_distance_sensor_enable(SensorProx[7], TIME_STEP);

  // config leds
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], 0);
  /*
  * You should declare here WbDeviceTag variables for storing
  * robot devices like this:
  *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
  *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
  */

  /* main loop
  * Perform simulation steps of TIME_STEP milliseconds
  * and leave the loop when the simulation is over
  */
  
  // Pega a posição da caixa antes de qualquer tipo de alteração na posição
  const double *box_position_const = wb_supervisor_node_get_position(box_node);
  const double position_1_const = box_position_const[0];
  const double position_2_const = box_position_const[1];
  const double position_3_const = box_position_const[2];
  printf("Posição da caixa CONSTANTE: (%.2f, %.2f, %.2f).", box_position_const[0], box_position_const[1], box_position_const[2]);


  while (wb_robot_step(TIME_STEP) != -1){

    const double *robot_position = wb_supervisor_node_get_position(wb_supervisor_node_get_self());
    
    // Pega a posição da caixa e sempre atualizando para ver se tem alguma alteração na posição
    const double *box_position_var = wb_supervisor_node_get_position(box_node);
   
    const double position_1_var = box_position_var[0];
    const double position_2_var = box_position_var[1];
    const double position_3_var = box_position_var[2];
    printf("Posição da caixa ALTERANDO: (%.2f, %.2f, %.2f).", box_position_var[0], box_position_var[1], box_position_var[2]);
    printf("Posição do robô: (%.2f, %.2f, %.2f)\n", robot_position[0], robot_position[1], robot_position[2]);
                            

    // Se tiver alteração na posição da caixa o robô para e acende os leds       
    if (fabs(position_1_const - position_1_var) > TOLERANCIA || 
        fabs(position_2_const - position_2_var) > TOLERANCIA || 
        fabs(position_3_const - position_3_var) > TOLERANCIA) {
      printf("A posição da caixa mudou. Parando o robô.\n");
      wb_motor_set_velocity(MotorEsquerdo, 0);
      wb_motor_set_velocity(MotorDireito, 0);
      wb_led_set(Leds[0], 1);  // Liga o LED
      break;
    }

    for (i = 0; i < 256; i++)
      texto[i] = 0;

    /*
    * Read the sensors :
    * Enter here functions to read sensor data, like:
    *  double val = wb_distance_sensor_get_value(my_sensor);
    */

    /* Process sensor data here */
    for (i = 0; i < QtddSensoresProx; i++){
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
      sprintf(texto, "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
    }
    printf("%s\n",texto);
    
    wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);

    /*
    * Enter here functions to send actuator commands, like:
    * wb_motor_set_position(my_actuator, 10.0);
    */
                    
    if(LeituraSensorProx[0] > distancia || LeituraSensorProx[1] > distancia){
      AceleradorEsquerdo = -0.5;
      AceleradorDireito = 0.5;
      counter++;
    }else if(LeituraSensorProx[6] > distancia || LeituraSensorProx[7] > distancia){
      AceleradorEsquerdo = 0.5;
      AceleradorDireito = -0.5;
      counter++;
    }else{
      AceleradorEsquerdo = 1;
      AceleradorDireito = 1;
      counter = 0;
    }
    
    if(counter > 10){
      AceleradorEsquerdo = 0.5;
      AceleradorDireito = -1;
    }
    
    int num = rand() % 100;
    if(num > 95){
      AceleradorEsquerdo = -1;
      AceleradorDireito = 1;
    }else if(num < 5){
      AceleradorEsquerdo = -0.5;
      AceleradorDireito = -1;
    }
      
    wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);
    
  };

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  return 0;
}