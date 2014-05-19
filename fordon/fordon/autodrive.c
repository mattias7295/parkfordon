/*
 * autodrive.c
 *
 * Created: 2014-05-13 11:27:46
 *  Author: johe0179
 */ 

#include "autodrive.h"
#define R 6371
#define TO_RAD (3.1415926536 / 180)

void calcHeading() {
	double latPerson;
	double lonPerson;
	char latitude[10];
	char longitude[11];
	USART_Transmit(0xFF);
	for(int i = 0;i<10;i++) {
		latitude[i] = USART_Receive();
	}
	USART_Transmit(0xFF);
	for(int i = 0;i<11;i++) {
		longitude[i] = USART_Receive();
	}
	
	char degA[3];
	degA[0] = latitude[0];
	degA[1] = latitude[1];
	degA[2] = '\0';
	
	double deg = atof(degA);
	
	char minA[8];
	minA[0] = latitude[2];
	minA[1] = latitude[3];
	minA[2] = latitude[4];
	minA[3] = latitude[5];
	minA[4] = latitude[6];
	minA[5] = latitude[7];
	minA[6] = latitude[8];
	minA[7] = '\0';
	
	double minutes = atof(minA);
	latPerson = deg + minutes/60;
	
	char degO[4];
	degO[0] = longitude[0];
	degO[1] = longitude[1];
	degO[2] = longitude[2];
	degO[3] = '\0';
	
	deg  = atof(degO);
	
	char minO[8];
	minO[0] = longitude[3];
	minO[1] = longitude[4];
	minO[2] = longitude[5];
	minO[3] = longitude[6];
	minO[4] = longitude[7];
	minO[5] = longitude[8];
	minO[6] = longitude[9];
	minO[7] = '\0';
	
	minutes = atof(minO);
	lonPerson = deg + minutes/60;
	
	// L�s av position
	double latVehicle;
	double lonVehicle;
	// R�kna ut vilken riktning fordonet ska v�nda sig �t av argumenten
	int angle = atan2( lonPerson-lonVehicle, latPerson-latVehicle);
	
	// Om cos av vinkeln mellan riktningarna �r positiv: vrid �t h�ger, Annars v�nster
	if(cos(angle)<0) {	
		// L�s av riktning
		double dir = (double)compas_update();
		dir = dir/10;
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

/*
Bluetooth kommunikation
Automatisk k�rning

Transmit
Receive 10 bytes
Transmit
Receive 11 bytes

 
 
Manuell k�rning
*/