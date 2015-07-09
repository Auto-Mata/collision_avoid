
#ifndef AGENTCLASS_H_
#define AGENTCLASS_H_

#include "ros/ros.h"
#include <RVO.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>

	
class AgentClass
{
	private:
		bool addAgentTrue[6];
		int agentNumber[6];
		int activeAgent;
		RVO::RVOSimulator* simulator;
		ros::Publisher pub_;
		std::vector<ros::Subscriber> sub_odom; //Subscrieber za svaki pioneer na njegovu odometriju (trenutna pozicija i brzina)
		ros::Subscriber sub_; //Subscrieber za preferiranu brzinu robota na kojem se program vrti
		float theta; //Kut vektora preferirane brzine, potreban u callback funkciji za pref brzinu
		void callback_function_main(const geometry_msgs::Twist::ConstPtr& data, int i);
		void callback_function_others(const nav_msgs::Odometry::ConstPtr& data, int i);		
		void setupScenario();
	public:
		AgentClass(ros::NodeHandle handle, std::string ns);
		~AgentClass();
		void run();

};
#endif 
