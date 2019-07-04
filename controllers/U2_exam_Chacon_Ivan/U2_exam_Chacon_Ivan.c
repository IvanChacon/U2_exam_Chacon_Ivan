/*
 * File:          U2_exam_Chacon_Ivan.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define PI 3.14159
#define OBSTACLE_DISTANCE 35


enum {
  GO,
  TURN,
  FREEWAY,
  OBSTACLE
  };

  //Global variables

  double initial_angle_wheel1;

/*
 * Functions
 */

    void goRobot (WbDeviceTag *wheels, float velocity) {
       wb_motor_set_velocity(wheels[0], velocity);
       wb_motor_set_velocity(wheels[1], velocity);
    }

    void stopRobot (WbDeviceTag *wheels, float noVel) {
       wb_motor_set_velocity(wheels[0], noVel);
       wb_motor_set_velocity(wheels[1], noVel);
    }

     int LookForObstacles (WbDeviceTag dist_sensor) {
       double distance = wb_distance_sensor_get_value(dist_sensor);
       printf("Distance: %lf\n", distance);

       if (distance > OBSTACLE_DISTANCE)
 	 return FREEWAY;
 	   else
 	     return OBSTACLE;
     }

      void turn90right (WbDeviceTag *wheels, float velocity) {
          wb_motor_set_velocity(wheels[0], velocity);
          wb_motor_set_velocity(wheels[1], -velocity);
      }

     double getAngleRobot(WbDeviceTag encoder1) {
       printf("Angle\n");
       double angle, angle_wheel1;

       angle_wheel1 = wb_position_sensor_get_value(encoder1);
       printf("Angle wheel1: %lf\n", angle_wheel1);
       angle = fabs(angle_wheel1 - initial_angle_wheel1);
       printf("Angle: %lf\n", angle);

       return angle;
     }

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   // Motor devices
      WbDeviceTag wheels[2];
   wheels[0] = wb_robot_get_device("motor_right");
   wheels[1] = wb_robot_get_device("motor_left");

   wb_motor_set_position (wheels[0], INFINITY);
   wb_motor_set_position (wheels[1], INFINITY);

   // Encoder devices
   WbDeviceTag encoder_right = wb_robot_get_device("encoder1");
   wb_position_sensor_enable(encoder_right, TIME_STEP);
   WbDeviceTag encoder_left = wb_robot_get_device("encoder2");
   wb_position_sensor_enable(encoder_left, TIME_STEP);

   // Distance sensor devices
   WbDeviceTag dist_sensor  = wb_robot_get_device("distance_sensor");
   wb_distance_sensor_enable (dist_sensor, TIME_STEP);

   //variables
   float velocity = -6.333;
   float noVel = 0;
   float angle;
   int ds_state, robot_state = GO;

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    if (robot_state == GO) {
      ds_state = LookForObstacles(dist_sensor);

      if (ds_state == FREEWAY) {
        goRobot(wheels, velocity);
        angle = wb_position_sensor_get_value(encoder_right);
        printf("Angle: %lf\n", angle);
      } else if (ds_state == OBSTACLE) {
        robot_state = TURN;
        stopRobot(wheels, noVel);
        initial_angle_wheel1 = wb_position_sensor_get_value(encoder_right);
      }
    } else if (robot_state == TURN) {
      turn90right(wheels, velocity);
      angle = getAngleRobot(encoder_right);

      if (angle >= 0.59*PI) {
        robot_state = GO;
        stopRobot(wheels, noVel);
      }
    }
	}

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
