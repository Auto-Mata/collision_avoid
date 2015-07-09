#include <AgentClass.h>

int main(int argc, char **argv)
{
	std::string name = argv[1];
	ros::init(argc, argv, name+"_node"); //Pokretanje noda
	ros::NodeHandle handle; //Handler

	AgentClass pioneer(handle,name);

	ros::Rate r(10);
while( ros::ok() )
{
	pioneer.run();
	ros::spinOnce();
	r.sleep();
}
return 0;
}