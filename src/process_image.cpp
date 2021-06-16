#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

//  Definir un cliente global que solicite servicios
ros::ServiceClient client;

//  Esta función manda a llamar al servicio "command_robot" para manejar el robot en una direccion
//  en especifico
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Driving the robot");

    //: Solicita un servicio y pasa las velocidades para manejar el robot 
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Manda a llamar el servicio "command_robot" y pasa los comandos del motor solicitados
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }

}

// Esta funcion se ejecuta continuamente y lee la informacion de la imagen
	
void process_image_callback(const sensor_msgs::Image image)
{

    double velx = 0;
    double angularz =0 ;
    
    // TODO
    // Crear un ciclo para cada pixel en la imagen y checar si existe algun pixel blanco
    // Si es así, identificar en qué sección está el pixel (izquierda, derecha o centrada)
    // Dependiendo de la posición, pasar asignar valores a velx y angularz para mandarlos en 
    // la función drivebot(velx,angularz)
    // En caso de que no haya pixel blanco, el vehiculo deverá detenerse.
    // TODO


   
    drive_robot(velx, angularz);

}


int main(int argc, char** argv)
{
    // Inicializa el nodo "process_image"
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define un cliente de servicio capáz de solicitar servicios de "command_robot"
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Se subscribe al topico "/camera/rgb/image_raw" para leer la informacion de la imagen incluida 
    // dentro de la función "process_image_callback"
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Manipula la cominicacion entre eventos de ROS
    ros::spin();

    return 0;
}
