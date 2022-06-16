//
// Created by jemin on 6/15/22.
//


#include "raisim/World.hpp"
#include "final_exam_STUDENTID.hpp"
#include "raisim/RaisimServer.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);
  world.setTimeStep(0.002);
  auto cartpole_double = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/cartPole/cartpole_double.urdf");

  // a1_simplified configuration
  Eigen::VectorXd gc(cartpole_double->getGeneralizedCoordinateDim()), gv(cartpole_double->getDOF());
  gc << 0, 1, 2;
  gv << 0.1, 0.2, 0.3;
  cartpole_double->setState(gc, gv);

  /// this function updates internal variables in raisim (such as the mass matrix and nonlinearities)
  world.integrate1();
  cartpole_double->getMassMatrix(); /// due to the current bug in RaiSim, we have to compute the mass matrix first to get the bias term

  if((getNonlinearities(gc, gv) - cartpole_double->getNonlinearities({0,0,-9.81}).e()).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed "<<std::endl;


  // uncomment these lines if you want to visualize the system
//  server.launchServer();
//
//  while(true) {
//    server.integrateWorldThreadSafe();
//    std::this_thread::sleep_for(std::chrono::microseconds(int(world.getTimeStep() * 1e6)));
//  }

  return 0;
}
