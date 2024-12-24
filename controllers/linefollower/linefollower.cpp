// File:          linefollower.cpp
// Date:
// Description:
// Author:			Szymon Kowalski
// Modifications:


#include <webots/Robot.hpp>
#include <webots/motor.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;

bool jasno(DistanceSensor* czujkiIR);
void kontrola(Motor** kola, DistanceSensor** czujkiIR, double predkosc);


void kontrola(Motor** kola, DistanceSensor** czujkiIR, double predkosc) {

	double deltaV = 1.2;

	bool lewo = jasno(czujkiIR[0]);
	bool prawo = jasno(czujkiIR[1]);

	if ((lewo && prawo) || (!lewo && !prawo)) { //skrzy¿owanie lub jazda do przodu
		for (int i = 0; i < 4; i++)
			kola[i]->setVelocity(predkosc);
		return;
	}
	if (prawo) {
		kola[0]->setVelocity(predkosc * (1-deltaV));
		kola[1]->setVelocity(predkosc* (1 - deltaV));
		kola[2]->setVelocity(predkosc * (1 + deltaV));
		kola[3]->setVelocity(predkosc * (1 + deltaV));
		return;
	}

	if (lewo) {
		kola[0]->setVelocity(predkosc * (1 + deltaV));
		kola[1]->setVelocity(predkosc * (1 + deltaV));
		kola[2]->setVelocity(predkosc * (1 - deltaV));
		kola[3]->setVelocity(predkosc * (1 - deltaV));
		return;
	}
	
}

bool jasno(DistanceSensor* czujkiIR) {
	return czujkiIR->getValue() < 100;
}

int main(int argc, char** argv) {

	Robot* robot = new Robot();
	int krok = (int)robot->getBasicTimeStep();

	Motor* kola[4];

	kola[0] = robot->getMotor("kolo1"); //lewe przód
	kola[1] = robot->getMotor("kolo2"); //lewe ty³
	kola[2] = robot->getMotor("kolo3"); //prawe pzród 
	kola[3] = robot->getMotor("kolo4"); //prawe ty³

	for (int i = 0; i < 4; i++) {
		kola[i]->setPosition(INFINITY);
	}

	DistanceSensor* czujkiIR[2];

	czujkiIR[0] = robot->getDistanceSensor("czujkaIRlewo");
	czujkiIR[1] = robot->getDistanceSensor("czujkaIRprawo");

	czujkiIR[0]->enable(krok);
	czujkiIR[1]->enable(krok);

	double predkosc = 4;

	while (robot->step(krok) != -1) {

		kontrola(kola, czujkiIR, 1);
	
	}
		delete robot;
		return 0;	
}
