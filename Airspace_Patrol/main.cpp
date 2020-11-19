/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  TODO: Add information here
 *
 *  @author {TODO: Your student name + id}
 *  @date {TODO}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h" //Note, we don't need to specify the absolute path of this file (Cmake can find i due to include_directoriest)
#include "DetectBogies.h"
#include "BogieChaser.h"
//For example purposes only, this thread attmps to get the friendly aircraft's
//(red triangle) pose every 4 seconds. It plots this pose on the
//simulation (blue triangle) which stays on the image for 1 second, as per the
//'testPose()' documentation in the simualtor class.
void exampleThread(const std::shared_ptr<Simulator> & sim,const std::shared_ptr<DetectBogies> & object,const std::shared_ptr<BogieChaser> & chaser) {
  while(true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Pose pose;
    std::vector<Pose> poses;


     chaser->setBogies(object->getBogiePoses());


  }
}



int main(void)
{
  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());

  std::shared_ptr<DetectBogies> detectBogie(new DetectBogies(sim));
  std::shared_ptr<BogieChaser> chaseBogie(new BogieChaser(sim));
  DetectBogies detect(sim);
  threads.push_back(sim->spawn());

   threads.push_back(std::thread(exampleThread, sim,detectBogie,chaseBogie));
  threads.push_back(std::thread(&DetectBogies::processThreads,detectBogie));

 threads.push_back(std::thread(&BogieChaser::processThreads,chaseBogie));
  threads.push_back(std::thread(&BogieChaser::feedWatchDog,chaseBogie));


  //Join threads and begin!
  for(auto & t: threads){
    t.join();
  }

  return 0;
}
