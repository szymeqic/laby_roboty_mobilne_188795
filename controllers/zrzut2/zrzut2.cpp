// File:          zrzut2.cpp
// Date:
// Description:
// Author:		Szymon Kowalski 188795
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <iostream>


using namespace webots;

void obrot(Motor*  kola[], double predkosc) {
	kola[0]->setVelocity(predkosc);
	kola[1]->setVelocity(predkosc);
	kola[2]->setVelocity(-predkosc);
	kola[3]->setVelocity(-predkosc);
}

void jazdaPrzod(Motor* kola[], double predkosc) {
	
	kola[0]->setVelocity(predkosc);
	kola[1]->setVelocity(predkosc);
	kola[2]->setVelocity(predkosc);
	kola[3]->setVelocity(predkosc);
}

int znajdzNajmniejszy(const float* obraz, int rozdzielczosc) {
	
	int idxMin = 0;
	for (int i = 1; i < rozdzielczosc; i++)
		if (obraz[i] < obraz[idxMin])
			idxMin = i;

	return idxMin;
}

int main(int argc, char** argv) {

	Robot* robot = new Robot();
	int krok = (int)robot->getBasicTimeStep();

	Motor* kolo1 = robot->getMotor("kolo1"); // Przednie z lewej
	Motor* kolo2 = robot->getMotor("kolo2"); // Tylne z lewej
	Motor* kolo3 = robot->getMotor("kolo3"); // Przednie z prawej
	Motor* kolo4 = robot->getMotor("kolo4"); // Tylne z prawej

	Motor* kola[4] = {kolo1, kolo2, kolo3, kolo4};

	kolo1->setPosition(INFINITY);
	kolo2->setPosition(INFINITY);
	kolo3->setPosition(INFINITY);
	kolo4->setPosition(INFINITY);

	obrot(kola, 5);


	Lidar* lidar = robot->getLidar("lidar");

	const float* obraz = NULL;
	
	int liczbaWarstw = lidar->getNumberOfLayers();
	const int rozdzielczosc = lidar->getHorizontalResolution();
	const int rozmObraz = liczbaWarstw * rozdzielczosc;	// liczba Punktów
	

	int lidarKrok = 200;
	lidar->enable(lidarKrok);

	if(lidar->isPointCloudEnabled())
		lidar->disablePointCloud();

	int idxBlisko = 0;


	while (robot->step(krok) != -1) {

		obraz = lidar->getRangeImage();

		idxBlisko = znajdzNajmniejszy(obraz, rozdzielczosc);
		if (idxBlisko <= rozdzielczosc / 2 -5)
			obrot(kola, 1);
		else if( idxBlisko >= rozdzielczosc/2 + 5)
			obrot(kola, -1);
		else
			jazdaPrzod(kola, 3);

		std::cout<< obraz[idxBlisko]<<"  "<<idxBlisko << std::endl;

	};

	delete robot;
	return 0;
}
