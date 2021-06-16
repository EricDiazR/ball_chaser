#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
// ROS::Publisher de los comandos de motor;
ros::Publisher motor_command_publisher;

// funcion callback de nombre "handle_drive_request" que se ejecuta cuando un servicio "drive_bot" 
// es solicitado
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{

    ROS_INFO("DriveToTargetRequest received - linear_x:%0.2f, angular_z:%0.2f", (float)req.linear_x, (float)req.angular_z);

    // Creación de un objeto "motor_command" de tipo "geometry_msgs::Twist"
    geometry_msgs::Twist motor_command;
    // Definir las velocidades de las ruedas
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publicar velocidades para manejar el robot
    motor_command_publisher.publish(motor_command);
 
    // Regresar una respuesta de mensaje
    res.msg_feedback = "Vel linear x set: " + std::to_string(motor_command.linear.x) + " Vel angular z set: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}
// Esta funcion publica las velocidades "linear x" y velocidades angulares solicitadas a las juntas de las ruedas del vehiculo.
// Despues de publicar las velocidades solicitadas, un mensaje de retroalimentacion debe der regresado con las velocidades solicitadas.

int main(int argc, char** argv)
{
    // Se inicia un nodo de ROS de nombre "drive_bot"
    ros::init(argc, argv, "drive_bot");

    // Se crea un objeto NodeHandle de ROS
    ros::NodeHandle n;

    // Se inorma al nodo maestro que se estará publicando un mensaje de tipo "geometry_msgs::Twist" en el tópico actuador del robot con una cola de mensajes de 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Se define un servucio de nombre "/ball_chaser/command_robot"con una funcion handle
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send velocity commands");

    ros::spin();

    return 0;
}
