/*
 * autodrive.c
 *
 * Created: 2014-05-13 11:27:46
 *  Author: johe0179
 */ 

#include "autodrive.h"
#define R 6371
#define TO_RAD (3.1415926536 / 180)

static void put_char(uint8_t c, FILE* stream);



/*int calcHeading() {
	double latPerson;
	double lonPerson;
	char latitude[10];
	char longitude[11];*/
	/*USART_Transmit(0xFF);
	for(int i = 0;i<10;i++) {
		latitude[i] = USART_Receive();
		if(latitude[i] == 0x00) {
			return 1;
		}
	}
	USART_Transmit(0xFF);
	for(int i = 0;i<11;i++) {
		longitude[i] = USART_Receive();
		if(longitude[i] == 0x00) {
			return 1;
		}
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
	lonPerson = deg + minutes/60;*/

	
	/*parseGPS();
	latPerson = 63.821367;
	lonPerson = 20.309551;

	printf("Lat: %lf \nLon: %lf \n", lat, lon);
	// R�kna ut vilken riktning fordonet ska v�nda sig �t av argumenten
	int angle = atan2( lonPerson-lon, latPerson-lat);

	// Om cos av vinkeln mellan riktningarna �r positiv: vrid �t h�ger, Annars v�nster
	if(cos(angle)<0) {	
		// L�s av riktning
		double dir = (double)compas_update();
		dir = dir/10;
		//V�nd �t v�nster	
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			printf("calcheading\n");
			dir = (double)compas_update();
			dir = dir/10;
			initEngineLeftBackward(64);
			initEngineRightForward(64);
			_delay_ms(500);	
		}
		
	} else {
		double dir = (double)compas_update();
		dir = dir/10;
		//V�nd �t h�ger
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			dir = (double)compas_update();
			dir = dir/10;
			initEngineRightBackward(64);
			initEngineLeftForward(64);
			_delay_ms(500);
		}
		
	}
	// Om vinkeln st�mmer ungef�r, k�r fram�t tills koordinaterna �verenst�mmer
	while(!checkDistance(latPerson,lonPerson)){
		//initEngineLeftForward(128);
		//initEngineRightForward(128);
		prevSpeedL = 120;
		prevSpeedR = 120;
		_delay_ms(500);
		
	}
	return 0;		
	
}

//Haversine formula
int checkDistance(double latPerson, double lonPerson) {

	parseGPS();
	
	double dx, dy, dz;
	latPerson -= lon;
	latPerson *= TO_RAD, lonPerson *= TO_RAD, lon *= TO_RAD;
	
	dz = sin(lonPerson) - sin(lon);
	dx = cos(latPerson) * cos(lonPerson) - cos(lon);
	dy = sin(latPerson) * cos(lonPerson);
	return (asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R)<0.00002;
}

static void put_char(uint8_t c, FILE* stream)
{
	if (c == '\n') put_char('\r', stream);
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
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