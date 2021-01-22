#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "planification/ListePoints.h"
#include "planification/Checkpoints.h"

static float K = 0.5;
static float dist_p = 0.5;

geometry_msgs::Pose pos;
planification::ListePoints checkpoints;
float u_des_x;
float u_des_w;
bool end = false;

void positionCallback(const nav_msgs::Odometry &msg) //Callback de récuperation de l'odométrie
{
  pos = msg.pose.pose;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "commande");
  ros::NodeHandle n;
  ros::Publisher commande_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber position_sub = n.subscribe("odom", 1000, positionCallback);
  ros::ServiceClient checkpoints_client = n.serviceClient<planification::Checkpoints>("checkpoints");
  planification::Checkpoints srv;
  ros::Rate loop_rate(10);
  ros::Rate checkpoint_rate(1);

  int indice_point = 0;
  while (!checkpoints_client.call(srv)) //on attend que le rrt renvoie la liste des points de passage
  {
    ros::spinOnce();
    checkpoint_rate.sleep();
  }
  ROS_INFO("Service checkpoints received");
  checkpoints = srv.response.points;

  geometry_msgs::Point point = checkpoints.points[indice_point];
  ROS_INFO("x : %f", point.x);
  ROS_INFO("y : %f", point.y);

  while (ros::ok())
  {

    if (sqrt(pow(point.x - pos.position.x, 2) + pow(point.y - pos.position.y, 2)) <= 0.1 ) //lorsque l'on est assez proche du point de passage actuel
    {
      if (indice_point + 1 < checkpoints.points.size()) //s'il existe, on passe au point suivant
      {
        indice_point++;
        point = checkpoints.points[indice_point];
        ROS_INFO("checkpoint : [%f,%f]", point.x, point.y);
      }
      else //sinon on signale la fin du parcours
      {
        end = true;
      }
    }

    //conversion quaternions -> lacet
    double siny_cosp = 2 * (pos.orientation.w * pos.orientation.z + pos.orientation.x * pos.orientation.y);
    double cosy_cosp = 1 - 2 * (pos.orientation.y * pos.orientation.y + pos.orientation.z * pos.orientation.z);
    float theta = atan2(siny_cosp, cosy_cosp);
    ROS_INFO("x : %f", pos.position.x);
    ROS_INFO("y : %f", pos.position.y);
    ROS_INFO("theta : %f", theta);

    //erreur de position entre le robot et le point de passage actuel
    float err_x = point.x - pos.position.x;
    float err_y = point.y - pos.position.y;

    float dist = sqrt(pow(err_x,2)+pow(err_y,2));
    // vitesses carthésiennes désirées du robot
    float v_des_x = K * err_x;
    float v_des_y = K * err_y;

    if(end) //lorsque l'on est assez proche du dernier point, on met la commande à 0
    {
      u_des_x = 0;
      u_des_w = 0;
    }
    else
    {
      u_des_x = cos(theta) * v_des_x/dist + sin(theta) * v_des_y/dist; //on normalise le vecteur v_des pour avoir une vitesse constante
      u_des_w = -sin(theta) / dist_p  * v_des_x + cos(theta) / dist_p  * v_des_y;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = u_des_x;
    cmd.angular.z = u_des_w;
    commande_pub.publish(cmd);
    ROS_INFO("commande vx : %f", cmd.linear.x);
    ROS_INFO("commande w : %f", cmd.angular.z);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
