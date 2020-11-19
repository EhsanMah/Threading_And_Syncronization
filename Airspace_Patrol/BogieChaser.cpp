#include"BogieChaser.h"
#include <queue>
#include <stack>
BogieChaser::BogieChaser(const std::shared_ptr<Simulator> &sim)
    :Surveillance(sim) {
    pointAheadContainer_.reserve(this->dataFromFriendly_.container.size());
    pointAheadContainer_.assign(this->dataFromFriendly_.container.size(),{0.0,0.0});
    bogieTrajectories_.reserve(this->dataFromFriendly_.container.size());
    bogieTrajectories_.assign(this->dataFromFriendly_.container.size(),{{0,0}});
    timeOfImpact_.reserve(this->dataFromFriendly_.container.size());
    timeOfImpact_.assign(this->dataFromFriendly_.container.size(),0);

    friendlyVel.linear=100;
    friendlyVel.angular = 0;

}
void BogieChaser::setBogies(const std::vector<Pose> &bogies){

    this->bogiePoses_=bogies;
    bogiesSet=true;

 }





std::vector<double>BogieChaser::calculateBogieVelocities(){


    return calcVelocities;
}

std::vector<GlobalOrd> BogieChaser::projectNextPoint(){
    if(bogiesSet==false){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }else{

    for(unsigned int b=0;b<bogiePoses_.size();b++){

    double distance=getBogieVelocities().at(b);
    Pose bogiePose = getBogiePoses().at(b);
    pointAheadContainer_.at(b)=transformGlobal(bogiePose.position,bogiePose.orientation,distance);
}
    }

    return pointAheadContainer_;
}

double BogieChaser::calcNormal(unsigned int bogieNo){

    friendlyPose_.poseMutex.lock();
    Pose friendlyPose=friendlyPose_.pose;
    friendlyPose_.poseMutex.unlock();
    double normal=2000;
    unsigned int time=0;
    double assumedFriendlyVel=bogieVelocities_.at(bogieNo)+20;
    trajLock.lock();
    std::vector<GlobalOrd>traj=bogieTrajectories_.at(bogieNo);
    trajLock.unlock();
    std::vector<GlobalOrd> friendlyTrajectory= projectTrajectory(friendlyPose,traj.size(),assumedFriendlyVel);
    for(unsigned int b=0; b<traj.size();b++){
        double newNormal=simulator_->distance(friendlyTrajectory.at(b),traj.at(b));

        if(newNormal<normal){
            normal=newNormal;

            time=b;
            if(newNormal==0){
                break;
            }
        }
    }
    timeOfImpact_.at(bogieNo)=time;
    return normal;

}

unsigned int BogieChaser::getClosestBogie(){
    if(goalReached==true){
    getBogieDataFromFriendly();
    getBogieVelocities();
    getBogieDataFromBase();
    getBogieDataFromFriendly();
    friendlyPose_.poseMutex.lock();
    double normal=3000;

    Pose friendlyPose=friendlyPose_.pose;
    friendlyPose_.poseMutex.unlock();
    for(unsigned int b=0;b<bogiePoses_.size();b++){
    double assumedFriendlyVel=bogieVelocities_.at(b)+20;
    trajLock.lock();
    std::vector<GlobalOrd>traj=bogieTrajectories_.at(b);
    trajLock.unlock();
    std::vector<GlobalOrd> friendlyTrajectory= projectTrajectory(friendlyPose,traj.size(),assumedFriendlyVel);
    double newNormal=simulator_->distance(traj.at(5),friendlyTrajectory.at(5));
    if(newNormal<normal){
        normal=newNormal;
        closestBogie=b;
    }
    }
    }

    return closestBogie;
}

velocity BogieChaser::purePursuit(GlobalOrd goalPose,unsigned int time){
    visited=false;
    friendlyPose_.poseMutex.lock();
    Pose friendlyPose=friendlyPose_.pose;
    double distance=simulator_->distance(goalPose,friendlyPose_.pose.position);
    double deltaY=goalPose.y-friendlyPose_.pose.position.y;
    friendlyPose_.poseMutex.unlock();
    if(friendlyPose_.pose.position.x==goalPose.x || friendlyPose_.pose.position.y==goalPose.y ){
         visited=false;
    }
    double vel= distance/time;

    friendlyVel.linear=200;

    friendlyVel.angular=((2*deltaY)/(distance*distance))*friendlyVel.linear;


    return friendlyVel;
}

std::vector<GlobalOrd>BogieChaser::calculatePath(GlobalOrd goalPose){
//    if(goalReached==false){
    friendlyPose_.poseMutex.lock();
    Pose friendlyPose=friendlyPose_.pose;
    friendlyPose_.poseMutex.unlock();
    path_.reserve(10);
    path_.assign(10,{0,0});
    double l=simulator_->distance(friendlyPose.position,goalPose);
    double deltaY=goalPose.y-friendlyPose.position.y;
    double deltaX=(goalPose.x-friendlyPose.position.x);
    double rad=(l*l)/(2*deltaY);
    double d=(rad-deltaY);
    double pivotAngle=fabs(atan(deltaX/d)-M_PI);


    pivotPoint.x=(friendlyPose.position.x);
    pivotPoint.y=(friendlyPose.position.y+deltaY+d);
    double it=(M_PI-pivotAngle)/10;

    for(unsigned int a=0;a<10;a++){
    double angle=(a*it);
    GlobalOrd nextPoint=transformGlobal(pivotPoint,a,rad);

    path_.at(a)=transformGlobal(pivotPoint,angle,rad);
    }
//    }
    return path_;
}
void BogieChaser::controlFriendly(){

    while(true){
        if(bogiesSet==false){

        }else{

        double range = simulator_->distance(getFriendlyPose().position,getBogiePoses().at(closestBogie).position);
        if( range<100 ){
            goalReached=true;
            getClosestBogie();
        }

        if(simulator_->elapsed()<5000){
            goalReached=true;
            getClosestBogie();
        }

    getFriendlyPose();

     std::vector<GlobalOrd> a=projectNextPoint();


     GlobalOrd bogiePose= bogiePoses_.at(getClosestBogie()).position;
    if(bogiePose.x>3000||bogiePose.x<-3000){
        goalReached=true;
        getClosestBogie();
    }
    if(bogiePose.y>3000||bogiePose.y<-3000){
        goalReached=true;
       getClosestBogie();
    }
    friendlyPose_.poseMutex.lock();
    Pose friendlyPose = friendlyPose_.pose;
    friendlyPose_.poseMutex.unlock();
    double bogieBearing=fabs(atan(fabs(bogiePose.y-friendlyPose.position.y)/fabs(bogiePose.x-friendlyPose.position.x))-friendlyPose.orientation);

    unsigned int p=0;
std::vector<GlobalOrd>path=calculatePath(bogiePose);

//

    friendlyVel=purePursuit(path.at(p),5);

    if(visited==false){
    p++;

    friendlyVel=purePursuit(path.at(p),1);


    }


        }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    }

}
std::vector<GlobalOrd>BogieChaser::projectTrajectory(Pose currentPose,unsigned int time,double vel){
    std::vector<GlobalOrd> trajectory;
    trajectory.reserve(time);
    trajectory.assign(time,{0,0});
    for(unsigned int t=1;t<time;t++){
        double dist=vel*t;
        currentPose.position=transformGlobal(currentPose.position,currentPose.orientation,dist);
        trajectory.at(t-1)=currentPose.position;
    }
    return trajectory;
}

void BogieChaser::keepAircraftInAirspace(){
    while(true){
//        if(bogiesSet==true){
    friendlyPose_.poseMutex.lock();
    GlobalOrd friendlyPos=friendlyPose_.pose.position;
    friendlyPose_.poseMutex.unlock();
    if(friendlyPos.x>3500||friendlyPos.x<-3500){

        friendlyVel.linear=simulator_->V_TERM;
        friendlyVel.angular=(6*9.81)/friendlyVel.linear;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        friendlyVel.linear=80;
        friendlyVel.angular=(6*9.81)/friendlyVel.linear;
    }
    if(friendlyPos.y>3500||friendlyPos.y<-3500){

        friendlyVel.linear=simulator_->V_TERM;
        friendlyVel.angular=-((6*9.81)/friendlyVel.linear);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        friendlyVel.linear=80;
        friendlyVel.angular=-((6*9.81)/friendlyVel.linear);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
void BogieChaser::calculateBogieTrajectories(){

    while (true) {
        if(bogiesSet==true){
            trajLock.lock();
        for(unsigned int b=0;b<bogiePoses_.size();b++){
            bogieTrajectories_.at(b)=projectTrajectory(bogiePoses_.at(b),8,bogieVelocities_.at(b));
        }
        trajLock.unlock();


        }else{
             std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


}

void BogieChaser::feedWatchDog(){
    while (true) {


    simulator_->controlFriendly(friendlyVel.linear,friendlyVel.angular);

    if(bogiesSet==true){
    std::vector<Pose>poses;
    for(auto p:path_){
    Pose pose;
    pose.position=bogiePoses_.at(closestBogie).position;
    pose.orientation=bogiePoses_.at(closestBogie).orientation;
    poses.push_back(pose);
    simulator_->testPose(poses);
    }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }


}

void BogieChaser::processThreads(void){
std::thread t1(&BogieChaser::keepAircraftInAirspace,this);
std::thread t2(&BogieChaser::calculateBogieTrajectories,this);
std::thread t3(&BogieChaser::controlFriendly,this);
t1.join();
t2.join();
t3.join();
}
