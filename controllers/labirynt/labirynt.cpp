// File:          labirynt.cpp
// Date:
// Description:
// Author:    Szymon Kowalski
// Modifications:


#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include<webots/Gyro.hpp>
#include<webots/PositionSensor.hpp>
#include<webots/LightSensor.hpp>
#include <iostream>

#define PI 3.141592

using namespace webots;
const double WIELKOSC_KRATKI = 0.2;
const double PROMIEN_KOLA = 0.02;

struct Sensory
{
	Motor* koloL = NULL;
	Motor* koloP = NULL;
	PositionSensor* enkoderL = NULL;
	PositionSensor* enkoderP = NULL;
	DistanceSensor* czujniki[8];
	Gyro* Zyro = NULL;
	LightSensor* fotodioda = NULL;
};

struct Wierzcholek {
	int id =-1;									//id wierzcho³ka
	int kierunki[4] = {-1, -1, -1, -1};		//w kierunkach zapisujemy wolne wierzcho³ki i ich id
											// -1 -> nieodwiedzone
											// -2 -> zablokowane (niedostêpne)
											// -3 -> dostêpne, nieodwiedzone
											// kierunki[0] = przód, [1]-> lewo , [2]-> ty³, [3]-> prawo
	unsigned int koszt;						//do dijksrtry
	bool odwiedzone = false;
	bool wszystkoObczajone = false;

};

void ruch(Motor* koloL, Motor* koloP, double predkosc, char ruch = 'n') {
	//'n' -> naprzod
	// 't' -> tyl

	switch (ruch) {

	case 'n':
	{
		koloL->setVelocity(predkosc);
		koloP->setVelocity(predkosc);
		break;
	}
	case 't':
	{
		koloL->setVelocity(-predkosc);
		koloP->setVelocity(-predkosc);
		break;
	}

	case 'r':
	{
		koloL->setVelocity(predkosc);
		koloP->setVelocity(-predkosc);
		break;
	}
	case 'l':
	{
		koloL->setVelocity(-predkosc);
		koloP->setVelocity(predkosc);
		break;
	}

	}
	return;
}

double czytajZyro(int os, const double* osie) {

	// 0 ->  oœ x
	// 1 ->  oœ y
	// 2 ->  oœ z


	double wartosc = osie[os] + 100000;   //wartosc w zakresie 0 200000
	wartosc = 26.6 * (wartosc / 200000) - 13.3;

	return wartosc;
}

double czytajAkc(int os, const double* osie) {
	// 0 ->  oœ x
	// 1 ->  oœ y
	// 2 ->  oœ z

	double wartosc = osie[os] / 50 - 100;   //wartosc w zakresie 0 200000
	//wartosc = (wartosc * 2) - 1000000;

	return wartosc;
}

double czytajOdl(DistanceSensor* czujnik) {

	double wynik = czujnik->getValue();
	wynik = ((1000 - wynik)/1000) * 0.2;
	return wynik;

}

double deltaRot(double rotStara, double rotNowa, bool kierunek = true) {
	//kierunek true = przod

	if (rotNowa >= rotStara && kierunek)
		return rotNowa - rotStara;

	if (rotNowa < rotStara && kierunek)
		return rotStara - rotNowa ;

	if (!kierunek && rotStara >= rotNowa)
		return rotStara - rotNowa;

	if (!kierunek && rotStara < rotNowa)
		return rotNowa - rotStara;

};


int wybierzKier(Wierzcholek* graf, int& aktID, int& rot) {
	int kier = -1;
	int cel = 0;

	for (int i = 0; i < 4; i++) {

		cel = graf[aktID].kierunki[i];
		std::cout << i << "  graf[aktID].kierunki[i]  " << cel <<"  odwidzoen:"<<graf[cel].odwiedzone<< std::endl;

		if (cel == -2 || graf[cel].wszystkoObczajone) //zablokowane
			continue;

		if (!graf[cel].odwiedzone) {
			aktID = graf[aktID].kierunki[i];
			std::cout << "aktID " << aktID  << std::endl;
			kier = (i ) % 4;
			std::cout<<"kier " << kier << " rot: " << rot << std::endl;
			return kier;
		}
	}

	graf[aktID].wszystkoObczajone = true;
	std::cout << "Obczajone? " << graf[aktID].wszystkoObczajone << std::endl;

	for (int i = 0; i < 4; i++) {
		cel = graf[aktID].kierunki[i];
		std::cout << "ObczajoneHHH? " << graf[cel].wszystkoObczajone << std::endl;

		if (!graf[cel].wszystkoObczajone) {
			aktID = graf[aktID].kierunki[i];
			std::cout << "aktID " << aktID << std::endl;
			return i;
		}
	}
	//wszystkie albo zablokowane albo odwiedzone, w takim wypadku sie cofamy
	std::cout << "GUGHUGUGUGUGUGG " << aktID << std::endl;
	// nie powinno siê wydarzyæ
	return 2;
}

void rozeznanie(Wierzcholek* graf, Sensory sensory, int &aktID, int& maxID, int rot ) {
	//ps5 -> lewy czujnik
	//ps2 -> prawy
	//ps4 -> lewo tyl
	//ps3 -> prawo tyl
	//ps0 -> prawo przód
	//ps7 -> lewo przód

	graf[aktID].odwiedzone = true;


	double odczyty[8] = { NULL };
	for (int i = 0; i < 8; i++) {
		odczyty[i] = czytajOdl(sensory.czujniki[i]);
		std::cout << "odczyt:  " <<i<<"  "<< odczyty[i] << std::endl;
	}

	bool kierunkiOdw[4] = { false };
	kierunkiOdw[0] = odczyty[0] >= 0.16 && odczyty[7] >= 0.16;  //przód
	kierunkiOdw[1] = odczyty[5] >= 0.16;						// lewo
	kierunkiOdw[2] = odczyty[4] >= 0.16 && odczyty[3] >= 0.16;	//ty³
	kierunkiOdw[3] = odczyty[2] >= 0.16;						//prawo

	std::cout << "przód: " << kierunkiOdw[0] << "  lewo: " << kierunkiOdw[1] << "  ty³: " << kierunkiOdw[2] << "  prawo: " << kierunkiOdw[3] << std::endl;

	
	for (int i = 0; i < 4; i++) {
		std::cout<<"rot: "<<rot << "  i: " << i << "  " << graf[aktID].kierunki[(i + rot) % 4] << std::endl;
		
		if (graf[aktID].kierunki[(i + rot) % 4] != -1)
			continue;

		if (kierunkiOdw[i]) {
			
			graf[aktID].kierunki[(i + rot) % 4] = ++maxID; // nieodwiedzone, dostêpne;  od razu nadajemy ID!!			
			graf[maxID].id = maxID;
			graf[maxID].kierunki[(2 + i + rot) %4] = aktID;
			std::cout << "i: " <<i<< std::endl;
		}
		else
			graf[aktID].kierunki[(i + rot) % 4] = -2; //nieodwiedzone, niedostêpne
	}

	bool obczajone = false;
	int cel = 0;
	int trzyRogi = 0;

	for (int i = 0; i < 4; i++)
	{
		cel = graf[aktID].kierunki[i];
		if (cel == -2)
			trzyRogi++;

		obczajone *= (cel == -2) || graf[cel].wszystkoObczajone;
	}
	obczajone += trzyRogi >= 3;

	graf[aktID].wszystkoObczajone = obczajone;
	

	std::cout << "Obczajone? " << obczajone << std::endl;

	std::cout<< "aktID"<<aktID << "  Kierunki aktID: " << graf[aktID].kierunki[0] << " " << graf[aktID].kierunki[1] << " "
		<< graf[aktID].kierunki[2] << " " << graf[aktID].kierunki[3] << " " << std::endl;

	std::cout<<"maxID: "<<maxID << "  Kierunki maxID: " << graf[maxID].kierunki[0] << " " << graf[maxID].kierunki[1] << " "
		<< graf[maxID].kierunki[2] << " " << graf[maxID].kierunki[3] << " " << std::endl;

}


bool koniec(Sensory sensory) {
	double val = sensory.fotodioda->getValue()/1000;

	std::cout<<"val   " << val << std::endl;

	return val > 0.5;

}

bool kratka(Motor* koloL, Motor* koloP, Sensory sensory, int krok, 
	Wierzcholek* graf, int aktID, int polecenie, int &rot) {
	bool koniec = false;
	double predkosc = 2;

	double rotKolNowa = 0;
	static double rotKolStara;
	static double rotKol;

	double omegaY = 0;
	static double rotY;

	double odczyty[8] = { NULL };
	for (int i = 0; i < 8; i++) {
		odczyty[i] = czytajOdl(sensory.czujniki[i]);
		//std::cout << "odczyt:  " << i << "  " << odczyty[i] << std::endl;
	}
	
	bool korekta = abs((odczyty[5] - odczyty[2])) > 0.001;
	korekta = odczyty[5] == 0.2 || odczyty[2] == 0.2 ? false: korekta;
	
	//std::cout << "Polecenie - rot   " << (4 + polecenie - rot)%4 << std::endl;
	/*
		0 -> Przód
		1 -> Jazda w lewo
		2 -> Jazda do ty³u
		3 -> Jazda w prawo		
	*/
	switch ((4 + polecenie -rot) %4) {
	case 0:  //przod

		ruch(koloL, koloP, predkosc);
		rotKolNowa = sensory.enkoderL->getValue();
		rotKol += deltaRot(rotKolStara, rotKolNowa) * PROMIEN_KOLA;
		rotKolStara = rotKolNowa;
		
		koniec = rotKol >= WIELKOSC_KRATKI;
		if (koniec) {
			ruch(koloL, koloP, 0);
			rotKol = 0;
			rot = polecenie;
			rot %= 4;

		}
		break;


	case 1: // lewo 

		if (rotY < PI / 2) {
			ruch(koloL, koloP, 0.2, 'l');
			omegaY = czytajZyro(2, sensory.Zyro->getValues());
			rotY += omegaY * krok / 1000;
			rotKolStara = sensory.enkoderL->getValue();
			break;
		}

		while (korekta && abs((odczyty[5] - odczyty[2])) > 0.001) {
			break;
		}
		
		ruch(koloL, koloP, predkosc);
		rotKolNowa = sensory.enkoderL->getValue();
		rotKol += deltaRot(rotKolStara, rotKolNowa) * PROMIEN_KOLA;
		rotKolStara = rotKolNowa;
		
		koniec = rotKol >= WIELKOSC_KRATKI;
		if (koniec) {
			ruch(koloL, koloP, 0);
			rotKol = 0;
			rotY = 0;
			rot = polecenie;
			rot %= 4;
		}
		break;

	case 2: //tyl
		ruch(koloL, koloP, -predkosc);
		rotKolNowa = sensory.enkoderL->getValue();
		rotKol += deltaRot(rotKolStara, rotKolNowa, false) * PROMIEN_KOLA;
		rotKolStara = rotKolNowa;
		koniec = rotKol >= WIELKOSC_KRATKI;
		if (koniec) {
			ruch(koloL, koloP, 0);
			rotKol = 0;
			//rot = polecenie;
			//rot %= 4;
		}
		break;

	case 3: //prawo
		if (rotY > -PI / 2) {
			ruch(koloL, koloP, 0.2, 'r');
			omegaY = czytajZyro(2, sensory.Zyro->getValues());
			rotY += omegaY * krok / 1000;
			rotKolStara = sensory.enkoderL->getValue();
			break;
		}

		while (korekta && abs((odczyty[5] - odczyty[2])) > 0.001) {
			break;
		}
		ruch(koloL, koloP, predkosc);
		rotKolNowa = sensory.enkoderL->getValue();
		rotKol += deltaRot(rotKolStara, rotKolNowa) * PROMIEN_KOLA;
		rotKolStara = rotKolNowa;

		koniec = rotKol >= WIELKOSC_KRATKI;

		if (koniec) {
			ruch(koloL, koloP, 0);
			rotKol = 0;
			rotY = 0;
			rot = polecenie;
			rot %= 4;
		}
		break;
	default:
		ruch(koloL, koloP, 0);
		break;
	}

	return koniec;
}


void dijkstra(Wierzcholek* graf) {

}


int main(int argc, char** argv) {


	Robot* robot = new Robot();

	int krok = (int)robot->getBasicTimeStep();

	std::cout << "Krok symulacji: " << krok;

	Motor* koloL = robot->getMotor("left wheel motor");
	Motor* koloP = robot->getMotor("right wheel motor");

	PositionSensor* enkoderL = robot->getPositionSensor("left wheel sensor");
	PositionSensor* enkoderP = robot->getPositionSensor("right wheel sensor");

	enkoderL->enable(krok);
	enkoderP->enable(krok);


	Gyro* zyro = robot->getGyro("gyro");
	zyro->enable(krok);
	//const double* LUTZyro = zyro->getLookupTable();
	//const int LUTZyroRozm = zyro->getLookupTableSize();

	const double* osieZyro = NULL;
	
	LightSensor* fotodioda = robot->getLightSensor("light sensor");
	fotodioda->enable(krok);
	
	Sensory sensory;
	{
		sensory.czujniki[0] = robot->getDistanceSensor("ps0");
		sensory.czujniki[1] = robot->getDistanceSensor("ps1");
		sensory.czujniki[2] = robot->getDistanceSensor("ps2");
		sensory.czujniki[3] = robot->getDistanceSensor("ps3");
		sensory.czujniki[4] = robot->getDistanceSensor("ps4");
		sensory.czujniki[5] = robot->getDistanceSensor("ps5");
		sensory.czujniki[6] = robot->getDistanceSensor("ps6");
		sensory.czujniki[7] = robot->getDistanceSensor("ps7");
		sensory.enkoderL = enkoderL;
		sensory.enkoderP = enkoderP;
		sensory.Zyro = zyro;
		sensory.fotodioda = fotodioda;
	}

	Wierzcholek temp;
	temp.id = -1;
	temp.koszt = INFINITY;
	temp.kierunki[0] = -1;
	temp.kierunki[1] = -1;
	temp.kierunki[2] = -1;
	temp.kierunki[3] = -1;

	Wierzcholek graf[64] = { temp };
	graf[0].id = 0;
	graf[0].koszt = 0;
	graf[0].odwiedzone = true;

	for (int i = 0; i < 8; i++)
		sensory.czujniki[i]->enable(krok);


	koloL->setPosition(INFINITY);
	koloP->setPosition(INFINITY);

	 
	int aktID = 0;
	int maxID = 0;

	double obrotY = 0;
	double odczytY = 0;

	bool czyRoz = true;
	bool czyKoniec = false;

	int IDKoniec = -1;
	int kier = 0; //0 > prosto

	int rot = 0; //0 -> prosto na zachód,

	while (robot->step(krok) != -1) {

		
		if (czyRoz) {

			rozeznanie(graf, sensory, aktID, maxID, rot);
			kier = wybierzKier(graf, aktID, rot);
			czyRoz = false;
			std::cout << "Rozeznanie" << std::endl;
			std::cout << "AktID:" << aktID << std::endl;
			std::cout<<"kier:  " << kier << "  " << rot << std::endl;
			czyKoniec += koniec(sensory);
			if (czyKoniec) {
				std::cout << "KONIEC!" << std::endl;
				ruch(koloL, koloP, 0);			
			}
		}

		if(!czyKoniec)
			if (kratka(koloL, koloP, sensory, krok, graf, aktID, kier, rot) ) 
				czyRoz = true;
		
		
	};

	// Enter here exit cleanup code.

	delete robot;
	return 0;
}