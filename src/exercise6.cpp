//
// Created by Jemin Hwangbo on 2022/04/08.
//

#include "raisim/World.hpp"
#include "exercise6_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  world.setTimeStep(0.002);
  auto a1 = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/a1/urdf/a1_simplified.urdf");

  // a1_simplified configuration
  Eigen::VectorXd gc(a1->getGeneralizedCoordinateDim()), gv(a1->getDOF());
  gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8;
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  a1->setState(gc, gv);

  /// this function updates internal variables in raisim (such as the mass matrix and nonlinearities)
  world.integrate1();
  a1->getMassMatrix(); /// due to the current bug in RaiSim, we have to compute the mass matrix first to get the bias term

  if((getNonlinearities(gc, gv) - a1->getNonlinearities({0,0,-9.81}).e()).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed "<<std::endl;

  std::cout<<"the solution is\n"<<a1->getNonlinearities({0,0,-9.81})<<std::endl;

  return 0;
}
