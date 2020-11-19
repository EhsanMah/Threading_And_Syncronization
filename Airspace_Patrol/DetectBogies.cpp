#include "DetectBogies.h"
DetectBogies::DetectBogies(const std::shared_ptr<Simulator> &sim)
    :Surveillance(sim) {

}

void DetectBogies::getBogiePositions(){

    getBogieDataFromFriendly();
    getFriendlyPose();
    for(unsigned int i=0;i<dataFromFriendly_.container.size();i++){
        dataFromFriendly_.dataMutex.lock();
        RangeBearingStamped fromFriendly = dataFromFriendly_.container.at(i);
        dataFromFriendly_.dataMutex.unlock();
        friendlyPose_.poseMutex.lock();
        Pose friendlyPose= friendlyPose_.pose;
        friendlyPose_.poseMutex.unlock();

    double thetaF=friendlyPose_.pose.orientation;
    double thetaFb =fromFriendly.bearing;
    double thetaR=thetaF+thetaFb;

    bogiePoses_.at(i).position=transformGlobal(friendlyPose.position,thetaR,fromFriendly.range);

    }
}

void DetectBogies::getOrientationofBogies(void){
while(true){
    for(unsigned int i=0;i<dataFromBase_.container.size();i++){
        GlobalOrd prevPos,currPos;


        prevPos=bogiePoses_.at(i).position;



        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        getBogiePositions();

        currPos=bogiePoses_.at(i).position;


        double ang = (prevPos.y-currPos.y)/(prevPos.x-currPos.x);
       double orient=atan(ang);

        bogiePoses_.at(i).orientation=(calculateAngle(prevPos,currPos,orient));

       }
}

}

double DetectBogies::calculateAngle(GlobalOrd &point1,GlobalOrd &point2, double angle){
    double precised_angle=0;
      if(point2.x>point1.x){
              precised_angle=angle;

      }else{
              precised_angle=angle+M_PI;

      }

    return precised_angle;
}



void DetectBogies::processThreads(void){
    std::thread orientations(&DetectBogies::getOrientationofBogies,this);

    orientations.join();
}
