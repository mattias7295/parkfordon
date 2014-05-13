/*
 * autodrive.c
 *
 * Created: 2014-05-13 11:27:46
 *  Author: johe0179
 */ 

#include "autodrive.h"
#define R 6371
#define TO_RAD (3.1415926536 / 180)

void calcHeading(double latPerson, double lonPerson) {
	
	
	// Läs av position
	double latVehicle;
	double lonVehicle;
	// Räkna ut vilken riktning fordonet ska vända sig åt av argumenten
	int angle = atan2( lonPerson-lonVehicle, latPerson-latVehicle);
	
	// Om cos av vinkeln mellan riktningarna är positiv: vrid åt höger, Annars vänster
	if(cos(angle)<0) {	
		// Läs av riktning
		uint16_t dir = compas_update();
		//Vänd åt vänster	
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			dir = compas_update();
			
			initEngineLeftBackward(64);
			initEngineRightForward(64);
			_delay_ms(500);	
		}
		// Om vinkeln stämmer ungefär, kör framåt tills koordinaterna överenstämmer
		while(!checkDistance(latPerson,lonPerson)){
			initEngineLeftForward(128);
			initEngineRightForward(128);
			_delay_ms(500);
		}
		
	} else {
		uint16_t dir = compas_update();
		//Vänd åt höger
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			dir = compas_update();
			
			initEngineRightBackward(64);
			initEngineLeftForward(64);
			_delay_ms(500);
		}
		// Om vinkeln stämmer ungefär, kör framåt tills koordinaterna överenstämmer
		while(!checkDistance(latPerson,lonPerson)){
			initEngineLeftForward(128);
			initEngineRightForward(128);
			_delay_ms(500);
		}
	}		
	
}

//Haversine formula
int checkDistance(double latPerson, double lonPerson) {
	double latVehicle;
	double lonVehicle;
	double dx, dy, dz;
	latPerson -= lonVehicle;
	latPerson *= TO_RAD, lonPerson *= TO_RAD, lonVehicle *= TO_RAD;
	
	dz = sin(lonPerson) - sin(lonVehicle);
	dx = cos(latPerson) * cos(lonPerson) - cos(lonVehicle);
	dy = sin(latPerson) * cos(lonPerson);
	return (asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R)<0.00002;
}