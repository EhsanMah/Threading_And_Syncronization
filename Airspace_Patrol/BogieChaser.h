#ifndef BOGIECHASER_H
#define BOGIECHASER_H
#include "Surveillance.h"


class BogieChaser: public Surveillance{


public:
    BogieChaser(const std::shared_ptr<Simulator> &sim);
    void setBogies(const std::vector<Pose>& bogies);
    void feedWatchDog();
    void processThreads(void);
private:


std::vector<double>calculateBogieVelocities();
std::vector<GlobalOrd> projectNextPoint();
unsigned int getClosestBogie();
std::vector<GlobalOrd>projectTrajectory(Pose currentPose,unsigned int time,double vel);
void controlFriendly();

void keepAircraftInAirspace();
void calculateBogieTrajectories();
 double calcNormal(unsigned int bogieNo);


 std::vector<GlobalOrd>calculatePath(GlobalOrd goalPose);



 std::vector<std::vector<GlobalOrd>>bogieTrajectories_;
 GlobalOrd pivotPoint;

velocity purePursuit(GlobalOrd goalPose,unsigned int time);
std::vector<double>bogieOrientation_,calcVelocities;
velocity friendlyVel;
unsigned int closestBogie;
std::vector<GlobalOrd>pointAheadContainer_;
std::mutex trajLock;
std::vector<unsigned int> timeOfImpact_;
std::vector<GlobalOrd>path_;
bool goalReached=true;
bool bogiesSet=false;
bool visited=true;
};



#endif // BOGIECHASER_H
