#include <RVO.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
/* Ovaj program predstavlja jedan pioneer robot. On mora poznavati sve ostale
agente u prostoru tj. mora znati njihovu poziciju i brzinu kako bi mogao
obaviti simulaciju za vremenski korak u kojem ne smije doci do sudara*/

/*Potrebno napraviti: 

	- glavna kalsa jednog agenta: mora se povezati sa ROS-om i povući sve potrebne ulaze (topics-e). 
	Prvo ću napraviti za slučaj da su roboti u Gazebo simulatoru i da je unaprijed određen broj vozila (2 vozila za jednostavnost).
	Poslije je potrebno poopćiti program tako da se pri pokretanju ulaznim argumentima zadaje broj vozila i ostali potrebni parametri koje
	korisnik mora unijeti

*/

class AgentClass
{
	private:
		geometry_msgs::Twist poruka;
		ros::Publisher pub;
		ros::Subscriber sub1; 
		ros::ServiceClient pozicija_pioneer1_;
		/*Dinamicko subscribanje jos nije podrzano u ROS-u pa u ovom slucaju uzimam
		6 (treba dodati za koacnu verziju) subscribera kao fiksni broj jer toliko 
		pioneera imamo na raspolaganju*/
		void callback_function(const geometry_msgs::Twist::ConstPtr& data);		
			
	public:

		AgentClass(ros::NodeHandle handle);
		void run();

};

AgentClass::AgentClass(ros::NodeHandle handle)
{
//Definicija konstruktora klase

	
	//const std::string cmd_vel = "/cmd_vel";
	pub = handle.advertise<geometry_msgs::Twist>("/pioneer1/cmd_vel",1000,false); //Postavljanje publishera
	//const std::string cmd_vel_pref = "/cmd_vel_pref";	
	sub1 = handle.subscribe("pioneer1/cmd_vel_pref",1000,&AgentClass::callback_function,this); //Postavljanje subscribera
	//Pracenje pozicije robota iz Gazeba
	pozicija_pioneer1_ = handle.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
}
void AgentClass::run()
{
	//Ova funkcija se poziva periodicki iz main() funkcije
	pub.publish(poruka);

	//Ispis stanja modela na ros_info

	gazebo_msgs::GetModelState pozicija;
	pozicija.request.model_name = "pioneer";	
	pozicija_pioneer1_.call(pozicija);

	//geometry_msgs::Pose a = pozicija.response.pose;
	//ROS_INFO("%f",a.position.x);
	if (pozicija.response.success){	
	ROS_INFO("%f", pozicija.response.pose.position.x);
	}
	
}

void AgentClass::callback_function(const geometry_msgs::Twist::ConstPtr& data)
{
	//definicija callback funkcije

	poruka.linear.x = data->linear.x;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "TestNode"); //Pokretanje noda
	ros::NodeHandle handle; //Handler
	

	AgentClass pioneer1(handle);

	ros::Rate r(10);
while( ros::ok() )
{
	pioneer1.run();
	ros::spinOnce();
	r.sleep();
}
return 0;
}
