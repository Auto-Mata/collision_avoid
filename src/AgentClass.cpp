#include <AgentClass.h>
#include <boost/bind.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#define R 0.34/2

AgentClass::AgentClass(ros::NodeHandle handle, std::string ns)
/***
Kosntruktor klase
handle - handle za node 
ns - namespace aktivnog robota (npr. pioneer1)
***/
{
	ROS_INFO("Pocetak konstruktora");
	pub_ = handle.advertise<geometry_msgs::Twist>("/"+ns+"/cmd_vel",1,false);
	int i;
	for (i=0;i<=6;i++)
	{
		addAgentTrue[i] = true;
		sub_odom.push_back(handle.subscribe<nav_msgs::Odometry>("/"+ns+"/odom",1,boost::bind(&AgentClass::callback_function_others, this, _1, i)));

	}
	if(ns=="pioneer1"){i = 0;}
	else if(ns=="pioneer2"){i = 1;}
	else if(ns=="pioneer3"){i = 2;}
	else if(ns=="pioneer4"){i = 3;}
	else if(ns=="pioneer5"){i = 4;}
	else if(ns=="pioneer6"){i = 5;}
	activeAgent = i;
	sub_ = handle.subscribe<geometry_msgs::Twist>("/"+ns+"/cmd_vel_pref",1,boost::bind(&AgentClass::callback_function_main,this,_1,i));
	simulator = new RVO::RVOSimulator();
	AgentClass::setupScenario();
	ROS_INFO("Kraj konstruktora");
}

AgentClass::~AgentClass()
/***
Dekonstruktor klase
***/
{
	delete simulator;
}

void AgentClass::setupScenario()
/***
Funkcija za postavljanje osnovnih vrijednosti svakog robota (radius,vremeski korak tau,broj robota itd.)
***/
{
	simulator->setTimeStep(0.1f); 
	simulator->setAgentDefaults(1.0f,5,0.1f,5.0f,0.4f,1.0f);
}
/*
void AgentClass::callback_function_main(const geometry_msgs::Twist::ConstPtr& data, int i)
{
	if(!addAgentTrue[i])	
	{
		if (os>=0)
		{
			RVO::Vector2 v_pref = RVO::Vector2(-1*data->linear.x*cos(theta),data->linear.x*sin(theta));
			simulator->setAgentPrefVelocity(agentNumber[i],v_pref);
		}	
		else
		{
			RVO::Vector2 v_pref = RVO::Vector2(-1*data->linear.x*cos(theta),data->-1*linear.x*sin(theta));
			simulator->setAgentPrefVelocity(agentNumber[i],v_pref);	
		}
	}
}*/
void AgentClass::callback_function_others(const nav_msgs::Odometry::ConstPtr& data, int i)
{
	tf::Quaternion q = tf::Quaternion(data->pose.pose.orientation.x,data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
	tf::Vector3 ax = q.getAxis();
	float X,Y;
	float kut = round(100*q.getAngle())/100.0; 

	if(ax.z()<= 0)
	{
		kut = M_PI - kut;
	}
	else
	{
		kut = -M_PI + kut;
	}

	/*
	if (ax.z()>=0)
	{

		X = round(100*data->pose.pose.position.x)/100.0 - R*cos(kut);
		Y = round(100*data->pose.pose.position.y)/100.0 + R*sin(kut);
		v = RVO::Vector2(-1*cos(kut)*round(100*data->twist.twist.linear.x)/100.0,sin(kut)*round(100*data->twist.twist.linear.x)/100.0);
	}
	else
	{
		X = round(100*data->pose.pose.position.x)/100.0 - R*cos(kut);
		Y = round(100*data->pose.pose.position.y)/100.0 - R*sin(kut);	
		v = RVO::Vector2(-1*cos(kut)*round(100*data->twist.twist.linear.x)/100.0,-1*sin(kut)*round(100*data->twist.twist.linear.x)/100.0);

	}
	*/
	RVO::Vector2 v = RVO::Vector2(cos(kut)*round(100*data->twist.twist.linear.x)/100.0,sin(kut)*round(100*data->twist.twist.linear.x)/100.0);
	X = round(100*data->pose.pose.position.x)/100.0 + R*cos(kut);
	Y = round(100*data->pose.pose.position.y)/100.0 + R*sin(kut);
	if(addAgentTrue[i])
	{
		simulator->addAgent(RVO::Vector2(X,Y));
		agentNumber[i]= simulator->getNumAgents()-1;
		addAgentTrue[i] = false;
		simulator->setAgentVelocity(agentNumber[i],v);
	}
	else
	{
		simulator->setAgentPosition(agentNumber[i],RVO::Vector2(X,Y));
		simulator->setAgentVelocity(agentNumber[i],v);
	}


	if(i == activeAgent)
	{
		theta = kut;  //Spremanje informacije o kutu za aktivnog robota
	}
}

void AgentClass::callback_function_main(const geometry_msgs::Twist::ConstPtr& data, int i)
{
	if(!addAgentTrue[i])
	{
			float w  = data->angular.z;
			theta = theta + w*0.1; //kut se povečava u odnosu na kutnu brzinu
			simulator->setAgentPrefVelocity(agentNumber[i], RVO::Vector2(cos(theta)*data->linear.x,sin(theta)*data->linear.x));		
	}	
}



void AgentClass::run()
/***
Funkcija koja se periodički odvija. Prikuplja informacije o stanjima robota i obavlja izračun potreban
za izbjegavanje sudara
***/
{
	if (!addAgentTrue[activeAgent])
	{
		geometry_msgs::Twist brzina;
		simulator->doStep();
		RVO::Vector2 sim_v = simulator->getAgentVelocity(agentNumber[activeAgent]);
		float kut = atan2(sim_v.y(),sim_v.x());
		brzina.angular.z = (kut - theta)/0.1;
		brzina.linear.x = sqrt(pow(sim_v.x(),2)+ pow(sim_v.y(),2));
		pub_.publish(brzina);
	}
}

