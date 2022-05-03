//
// Created by Jemin Hwangbo on 2022/04/08.
//

#include "raisim/World.hpp"
#include "exercise5_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  world.setTimeStep(0.002);
  auto a1 = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/a1/urdf/a1_simplified.urdf");

  // a1_simplified configuration
  Eigen::VectorXd gc(a1->getGeneralizedCoordinateDim());
  gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8;
  a1->setGeneralizedCoordinate(gc);

  /// this function updates internal variables in raisim (such as the mass matrix and nonlinearities)
  world.integrate1();

  std::cout<<"mass matrix should be \n"<< a1->getMassMatrix().e()<<std::endl;

  if((getMassMatrix(gc) - a1->getMassMatrix().e()).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed "<<std::endl;


  return 0;
}
