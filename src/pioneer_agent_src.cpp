#include <RVO.h>
#include <ROS.h>
#include <geometry_msgs.h>

/* Ovaj program predstavlja jedan pioneer robot. On mora poznavati sve ostale
agente u prostoru tj. mora znati njihovu poziciju i brzinu kako bi mogao
obaviti simulaciju za vremenski korak u kojem ne smije doci do sudara*/

/*Potrebno napraviti: 

	- glavna kalsa jednog agenta: mora se povezati sa ROS-om i povući sve potrebne ulaze (topics-e). 
	Prvo ću napraviti za slučaj da su roboti u Gazebo simulatoru i da je unaprijed određen broj vozila (2 vozila za jednostavnost).
	Poslije je potrebno poopćiti program tako da se pri pokretanju ulaznim argumentima zadaje broj vozila i ostali potrebni parametri koje
	korisnik mora unijeti

*/