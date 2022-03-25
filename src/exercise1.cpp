//
// Created by Jemin Hwangbo on 2022/03/17.
//

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

#include "exercise1_STUDENTID.hpp"
#include "raisim/RaisimServer.hpp"


int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world); // visualization server
  world.addGround();

  // a1
  auto a1 = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/a1/urdf/a1.urdf");
  a1->setName("a1");
  server.focusOn(a1);

  // a1 configuration
  Eigen::VectorXd jointNominalConfig(a1->getGeneralizedCoordinateDim());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  a1->setGeneralizedCoordinate(jointNominalConfig);

  // debug sphere
  auto debugSphere = server.addVisualSphere("debug_sphere", 0.05);
  debugSphere->setColor(1,0,0,1);
  debugSphere->setPosition(getEndEffectorPosition(jointNominalConfig));

  // visualization
  server.launchServer();
  for (int i=0; i<2000000; i++)
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

  server.killServer();
}
