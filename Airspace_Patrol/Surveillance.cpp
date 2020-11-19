#include "Surveillance.h"

Surveillance::Surveillance(const std::shared_ptr<Simulator> &sim){
    simulator_ = sim;
    friendlyPose_.pose=simulator_->getFriendlyPose();
    dataFromBase_.container=simulator_->rangeVelocityToBogiesFromBase();
    dataFromFriendly_.container=simulator_->rangeBearingToBogiesFromFriendly();
    bogiePoses_.reserve(dataFromFriendly_.container.size());
    bogiePoses_.assign(dataFromFriendly_.container.size(),{{0,0},0});
    bogieVelocities_.reserve(dataFromFriendly_.container.size());
    bogieVelocities_.assign(dataFromFriendly_.container.size(),0);

}

Pose Surveillance::getFriendlyPose(void){
    friendlyPose_.poseMutex.lock();
    friendlyPose_.pose=simulator_->getFriendlyPose();
    friendlyPose_.poseMutex.unlock();
    return friendlyPose_.pose;
}


std::vector<Pose>Surveillance::getBogiePoses(void){

    return bogiePoses_;
}
std::vector<double>Surveillance::getBogieVelocities(void){
    bogieVelocities_.reserve(dataFromBase_.container.size());
    bogieVelocities_.assign(dataFromBase_.container.size(),0);

dataFromBase_.dataMutex.lock();
    for(unsigned int v=0;v<dataFromBase_.container.size();v++){
    bogieVelocities_.at(v)=dataFromBase_.container.at(v).velocity;
    }
    dataFromBase_.dataMutex.unlock();
    return bogieVelocities_;
}

std::vector<RangeBearingStamped>Surveillance::getBogieDataFromFriendly(void){
    dataFromFriendly_.dataMutex.lock();
    dataFromFriendly_.container=simulator_->rangeBearingToBogiesFromFriendly();
     dataFromFriendly_.dataMutex.unlock();
    return dataFromFriendly_.container;
}
std::vector<RangeVelocityStamped>Surveillance::getBogieDataFromBase(void){
    dataFromBase_.dataMutex.lock();
    dataFromBase_.container=simulator_->rangeVelocityToBogiesFromBase();
    dataFromBase_.dataMutex.unlock();
    return dataFromBase_.container;
}

GlobalOrd Surveillance::transformGlobal(GlobalOrd origin,double angle,double distance){
    GlobalOrd calculatedPoint;

    calculatedPoint.x=origin.x+(distance*cos(angle));
    calculatedPoint.y=origin.y+(distance*sin(angle));
    return calculatedPoint;
}
