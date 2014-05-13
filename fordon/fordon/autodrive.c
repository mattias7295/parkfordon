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
	
	
	// L�s av position
	double latVehicle;
	double lonVehicle;
	// R�kna ut vilken riktning fordonet ska v�nda sig �t av argumenten
	int angle = atan2( lonPerson-lonVehicle, latPerson-latVehicle);
	
	// Om cos av vinkeln mellan riktningarna �r positiv: vrid �t h�ger, Annars v�nster
	if(cos(angle)<0) {	
		// L�s av riktning
		uint16_t dir = compas_update();
		//V�nd �t v�nster	
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			dir = compas_update();
			
			initEngineLeftBackward(64);
			initEngineRightForward(64);
			_delay_ms(500);	
		}
		// Om vinkeln st�mmer ungef�r, k�r fram�t tills koordinaterna �verenst�mmer
		while(!checkDistance(latPerson,lonPerson)){
			initEngineLeftForward(128);
			initEngineRightForward(128);
			_delay_ms(500);
		}
		
	} else {
		uint16_t dir = compas_update();
		//V�nd �t h�ger
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			dir = compas_update();
			
			initEngineRightBackward(64);
			initEngineLeftForward(64);
			_delay_ms(500);
		}
		// Om vinkeln st�mmer ungef�r, k�r fram�t tills koordinaterna �verenst�mmer
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