//
// Created by Jemin Hwangbo on 2022/03/17.
//


#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

#include "raisim/RaisimServer.hpp"
#include "exercise2_STUDENTID.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world); // visualization server
  world.addGround();
  world.setTimeStep(0.0001);

  // a1
  auto a1 = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/a1/urdf/a1.urdf");
  a1->setName("a1");
  server.focusOn(a1);

  // a1 configuration
  Eigen::VectorXd gc(a1->getGeneralizedCoordinateDim());
  Eigen::VectorXd gv(a1->getDOF());

  gc << 0, 0, 10.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  gv << 0.1, 0.2, 0.3, 0.1, 0.4, 0.3, 0.1,0.1,0.1, 0.2,0.2,0.2, 0.3,0.3,0.3, 0.4,0.4,0.4;
  a1->setState(gc, gv);

  // visualization
  server.launchServer();
  raisim::Vec<3> footVel, footAngVel;
  bool answerCorrect = true;

  for (int i=0; i<20000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    world.integrate1();

    a1->getFrameVelocity("RL_foot_fixed", footVel);
    a1->getFrameAngularVelocity("RL_foot_fixed", footAngVel);

    if((footVel.e() - getFootLinearVelocity(gc, gv)).norm() < 1e-8) {
      std::cout<<"the linear velocity is correct "<<std::endl;
    } else {
      std::cout<<"the linear velocity is not correct "<<std::endl;
      answerCorrect = false;
    }

    if((footAngVel.e() - getFootAngularVelocity(gc, gv)).norm() < 1e-8) {
      std::cout<<"the angular velocity is correct "<<std::endl;
    } else {
      std::cout<<"the angular velocity is not correct "<<std::endl;
      answerCorrect = false;
    }

    world.integrate2();
    a1->getState(gc, gv);
  }

  server.killServer();

  if(answerCorrect) {
    std::cout<<"The solution is correct "<<std::endl;
  } else {
    std::cout<<"The solution is not correct "<<std::endl;
    answerCorrect = false;
  }

  return 0;
}
