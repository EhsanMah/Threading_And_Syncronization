#ifndef DETECTBOGIES_H
#define DETECTBOGIES_H
#include "Surveillance.h"

class DetectBogies: public Surveillance{
public:
DetectBogies(const std::shared_ptr<Simulator> &sim);

void processThreads(void);
private:
void getBogiePositions();
void getOrientationofBogies(void);
double calculateAngle(GlobalOrd &point1,GlobalOrd &point2, double angle);
std::vector<double>bogieOrientation_;
};



#endif // DETECTBOGIES_H
