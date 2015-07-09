#include <RVO.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
/* Ovaj program predstavlja jedan pioneer robot. On mora poznavati sve ostale
agente u prostoru tj. mora znati njihovu poziciju i brzinu kako bi mogao
obaviti simulaciju za vremenski korak u kojem ne smije doci do sudara*/

/*Potrebno napraviti: 

	- glavna kalsa jednog agenta: mora se povezati sa ROS-om i povući sve potrebne ulaze (topics-e). 
	Prvo ću napraviti za slučaj da su roboti u Gazebo simulatoru i da je unaprijed određen broj vozila (2 vozila za jednostavnost).
	Poslije je potrebno poopćiti program tako da se pri pokretanju ulaznim argumentima zadaje broj vozila i ostali potrebni parametri koje
	korisnik mora unijeti (ovo ipak nemože jer dinamičko subscribanje njije riješeno u rosu)

*/

class AgentClass
{
	private:
		bool addAgent1True_,addAgent2True_;
		int pioneer1_num, pioneer2_num;
		RVO::RVOSimulator* simulator;
		ros::Publisher pub_;
		ros::Subscriber sub[6]; //Subscribere strpam u jedan array da smanjim duljinu deklaracije
		ros::ServiceClient pozicija_; //Dovoljan je jedan service krijent za citanje pozicija iz Gazeba
		void callback_function_main(const geometry_msgs::Twist::ConstPtr& data);
		void callback_function_2(const geometry_msgs::Twist::ConstPtr& data);		
		void setupScenario();	
		float theta_prije;
	public:

		AgentClass(ros::NodeHandle handle);
		~AgentClass();
		void run();

};

AgentClass::AgentClass(ros::NodeHandle handle)
{
//Definicija konstruktora klase

	//const std::string cmd_vel = "/cmd_vel";
	pub_ = handle.advertise<geometry_msgs::Twist>("/pioneer2/cmd_vel",1000,false); //Postavljanje publishera
	//const std::string cmd_vel_pref = "/cmd_vel_pref";	
	sub[0] = handle.subscribe("/pioneer1/cmd_vel_pref",1000,&AgentClass::callback_function_main,this); //Postavljanje glavnog subscribera
	sub[1] = handle.subscribe("/pioneer2/cmd_vel_pref",1000,&AgentClass::callback_function_2,this); //Preferirana brzina drugog pioneera
	//Pracenje pozicije robota iz Gazeba
	pozicija_ = handle.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	//Rezervacija memorije za novu instancu simulatora
	simulator = new RVO::RVOSimulator();
	setupScenario();
	addAgent1True_ = true;
	addAgent2True_ = true;
	theta_prije = 0.0; //Primjena u AgentClas::Run() za izračun kutne brzine
}

AgentClass::~AgentClass()
{
	//Oslobađanje memorije 
	delete simulator;
}

void AgentClass::run()
{	
	//Funkcija se periodicki poziva i radi korak simulacije u RVO simulatoru
	if (!(addAgent1True_))
	{
		gazebo_msgs::GetModelState pozicija;
		pozicija.request.model_name = "pioneer1";
		pozicija_.call(pozicija);						//Ocitavanje pozicije pioneera1 u Gazebu

		//Stvaranje objekta kvaterniona klase tf::Quaternion kako bi mogao koristiti funkcije za dobivanje kuta rotacije  
		tf::Quaternion q = tf::Quaternion(pozicija.response.pose.orientation.x,pozicija.response.pose.orientation.y,pozicija.response.pose.orientation.z,pozicija.response.pose.orientation.w);

		float theta = round(100*q.getAngleShortestPath())/100.0; //Kut rotacije najkraćim putem
		tf::Vector3 os = q.getAxis();
		float X,Y,V;
		RVO::Vector2 trenutna_brzina;
		if (os.z() >= 0.0)
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 + (0.34/2)*sin(theta);

			V = round(100*pozicija.response.twist.linear.x)/100.0;
			trenutna_brzina = RVO::Vector2(-1*V*cos(theta),V*sin(theta));
		}
		else
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 - (0.34/2)*sin(theta);

			V = round(100*pozicija.response.twist.linear.x)/100.0;
			trenutna_brzina = RVO::Vector2(-1*V*cos(theta),-1*V*sin(theta));
		}
		simulator->setAgentVelocity(pioneer1_num,trenutna_brzina);	//Trenutna brzina iz Gazeba u RVO simulator
		simulator->setAgentPosition(pioneer1_num,RVO::Vector2(X,Y)); //Trenutna pozicija iz Gazeba u RVO simulator
	}

	if (!(addAgent2True_))
	{
		gazebo_msgs::GetModelState pozicija;
		pozicija.request.model_name = "pioneer2";
		pozicija_.call(pozicija);						//Ocitavanje pozicije pioneera2 u Gazebu
		
		//Stvaranje objekta kvaterniona klase tf::Quaternion kako bi mogao koristiti funkcije za dobivanje kuta rotacije  
		tf::Quaternion q = tf::Quaternion(pozicija.response.pose.orientation.x,pozicija.response.pose.orientation.y,pozicija.response.pose.orientation.z,pozicija.response.pose.orientation.w);

		float theta = round(100*q.getAngleShortestPath())/100.0; //Kut rotacije najkraćim putem
		tf::Vector3 os = q.getAxis();
		float X,Y,V;
		RVO::Vector2 trenutna_brzina;
		if (os.z() >= 0.0)
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 + (0.34/2)*sin(theta);

			V = round(100*pozicija.response.twist.linear.x)/100.0;
			trenutna_brzina = RVO::Vector2(-1*V*cos(theta),V*sin(theta));
		}
		else
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 - (0.34/2)*sin(theta);

			V = round(100*pozicija.response.twist.linear.x)/100.0;
			trenutna_brzina = RVO::Vector2(-1*V*cos(theta),-1*V*sin(theta));
		}
		simulator->setAgentVelocity(pioneer1_num,trenutna_brzina);	//Trenutna brzina iz Gazeba u RVO simulator
		simulator->setAgentPosition(pioneer1_num,RVO::Vector2(X,Y)); //Trenutna pozicija iz Gazeba u RVO simulator
	}

	
	if (!addAgent2True_)
	{
		simulator->doStep(); //Provođenje koraka simulacije

		geometry_msgs::Twist brzina_twist;
		RVO::Vector2 brzina = simulator->getAgentVelocity(pioneer2_num);
		float theta_sad = atan2(brzina.y(),brzina.x()); //Kut vektora brzine u ovom koraku simulacije
		brzina_twist.linear.x = sqrt(pow(brzina.x(),2)+ pow(brzina.y(),2));
		brzina_twist.angular.z = (theta_sad-theta_prije)/simulator->getTimeStep();	
		theta_prije = theta_sad; //Spremanje kuta za sljedeći korak simulacije
		pub_.publish(brzina_twist);
	}
}

void AgentClass::callback_function_main(const geometry_msgs::Twist::ConstPtr& data)
{
	//definicija callback funkcije
	
	if (addAgent1True_)
		{
		gazebo_msgs::GetModelState pozicija;
		pozicija.request.model_name = "pioneer1";
		pozicija_.call(pozicija);

		tf::Quaternion q = tf::Quaternion(pozicija.response.pose.orientation.x,pozicija.response.pose.orientation.y,pozicija.response.pose.orientation.z,pozicija.response.pose.orientation.w);
		float theta = round(100*q.getAngleShortestPath())/100.0; //Kut rotacije najkraćim putem
		tf::Vector3 os = q.getAxis();
		float X,Y;
		if(os.z() >= 0.0)
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 + (0.34/2)*sin(theta);
		}
		else
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 - (0.34/2)*sin(theta);
		}
		simulator->addAgent(RVO::Vector2(X,Y)); //Prvotno dodavanje pioneera u RVO simulator
		pioneer1_num = simulator->getNumAgents()-1; //Pamćenje rednog broja robota u simulatoru za povrat informacije iz simulatora
		addAgent1True_ = false; //Zastavica da je robot dodan u simulator 
		//Postavljanje preferirane brzine 
		if (os.z() >= 0.0)
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer1_num, brzina_pref); //Postavljanje prefrerirane brzine 1. pioneera
		}
		else
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),-1*data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer1_num, brzina_pref); //Postavljanje prefrerirane brzine 1. pioneera
		}
		}
	else
		{
		gazebo_msgs::GetModelState pozicija;
		pozicija.request.model_name = "pioneer1";
		pozicija_.call(pozicija);

		tf::Quaternion q = tf::Quaternion(pozicija.response.pose.orientation.x,pozicija.response.pose.orientation.y,pozicija.response.pose.orientation.z,pozicija.response.pose.orientation.w);
		float theta = round(100*q.getAngleShortestPath())/100.0; //Kut rotacije najkraćim putem
		tf::Vector3 os = q.getAxis();
		if (os.z() >= 0.0)
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer1_num, brzina_pref); //Postavljanje prefrerirane brzine 1. pioneera
		}
		else
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),-1*data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer1_num, brzina_pref); //Postavljanje prefrerirane brzine 1. pioneera
		}
		}
		
}
void AgentClass::callback_function_2(const geometry_msgs::Twist::ConstPtr& data)
{
	if (addAgent2True_)
		{
		gazebo_msgs::GetModelState pozicija;
		pozicija.request.model_name = "pioneer2";
		pozicija_.call(pozicija);
		
		tf::Quaternion q = tf::Quaternion(pozicija.response.pose.orientation.x,pozicija.response.pose.orientation.y,pozicija.response.pose.orientation.z,pozicija.response.pose.orientation.w);
		float theta = round(100*q.getAngleShortestPath())/100.0; //Kut rotacije najkraćim putem
		theta_prije = theta; //Orjentacija pri prvom pojavljivanju
		tf::Vector3 os = q.getAxis();
		float X,Y;
		if(os.z() >= 0.0)
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 + (0.34/2)*sin(theta);
		}
		else
		{
			X = round(100*pozicija.response.pose.position.x)/100.0 - (0.34/2)*cos(theta);
			Y = round(100*pozicija.response.pose.position.y)/100.0 - (0.34/2)*sin(theta);
		}
		simulator->addAgent(RVO::Vector2(X,Y)); //Prvotno dodavanje pioneera u RVO simulator
		pioneer2_num = simulator->getNumAgents()-1; //Pamćenje rednog broja robota u simulatoru za povrat informacije iz simulatora
		addAgent2True_ = false; //Zastavica da je robot dodan u simulator 
		//Postavljanje preferirane brzine 2. pioneera
		if (os.z() >= 0.0)
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer2_num, brzina_pref); //Postavljanje prefrerirane brzine 2. pioneera
		}
		else
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),-1*data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer2_num, brzina_pref); //Postavljanje prefrerirane brzine 2. pioneera
		}

		}
	else
		{
		gazebo_msgs::GetModelState pozicija;
		pozicija.request.model_name = "pioneer2";
		pozicija_.call(pozicija);
		
		tf::Quaternion q = tf::Quaternion(pozicija.response.pose.orientation.x,pozicija.response.pose.orientation.y,pozicija.response.pose.orientation.z,pozicija.response.pose.orientation.w);
		float theta = round(100*q.getAngleShortestPath())/100.0; //Kut rotacije najkraćim putem
		tf::Vector3 os = q.getAxis();
		if (os.z() >= 0.0)
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer2_num, brzina_pref); //Postavljanje prefrerirane brzine 2. pioneera
		}
		else
		{
			RVO::Vector2 brzina_pref = RVO::Vector2(-1*data->linear.x*cos(theta),-1*data->linear.x*sin(theta)); //Vektor preferirane brzine
			simulator->setAgentPrefVelocity(pioneer2_num, brzina_pref); //Postavljanje prefrerirane brzine 2. pioneera
		}
		}
	


}
void AgentClass::setupScenario()
{
	simulator->setTimeStep(0.1f);  //Postavljanje vremena koraka simulacije 	
	
	simulator->setAgentDefaults(1.0f,5,0.5f,5.0f,0.4f,1.0f); //Osnovni parametri za svakog novog robota dodanog u simulaciju
}





int main(int argc, char **argv)
{
	ros::init(argc, argv, "TestNode2"); //Pokretanje noda
	ros::NodeHandle handle; //Handler
	
{
	AgentClass pioneer1(handle);

	ros::Rate r(10);
while( ros::ok() )
{
	pioneer1.run();
	ros::spinOnce();
	r.sleep();
}
}
return 0;
}
