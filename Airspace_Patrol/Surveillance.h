#ifndef SURVEILLANCE_H
#define SURVEILLANCE_H
#include "simulator.h"
#include <thread>
#include <vector>
#include <iostream>
#include <math.h>
#include <mutex>
struct LockedPose{
    Pose pose;
    std::mutex poseMutex;
};

struct LockedRangeBearingStamped{
    std::vector<RangeBearingStamped> container;
    std::mutex dataMutex;
};

struct LockedRangeVelocityStamped{

    std::vector<RangeVelocityStamped> container;
    std::mutex dataMutex;
};

struct velocity{
    double angular;
    double linear;

};

struct velocityLocked{
    velocity vel;
    std::mutex velMutex;
};

class Surveillance{
public:
Surveillance(const std::shared_ptr<Simulator> &sim);

std::vector<Pose>getBogiePoses(void);

virtual void processThreads(void)=0;
protected:
Pose getFriendlyPose(void);
std::vector<double>getBogieVelocities(void);
std::vector<RangeBearingStamped>getBogieDataFromFriendly(void);
std::vector<RangeVelocityStamped>getBogieDataFromBase(void);
GlobalOrd transformGlobal(GlobalOrd origin,double angle,double distance);

std::shared_ptr<Simulator> simulator_;
LockedPose friendlyPose_;
std::vector<Pose>bogiePoses_;
LockedRangeBearingStamped dataFromFriendly_;
LockedRangeVelocityStamped dataFromBase_;
std::vector<double>bogieVelocities_;
};



#endif // SURVEILLANCE_H
