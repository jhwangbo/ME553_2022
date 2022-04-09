//
// Created by Jemin Hwangbo on 2022/04/08.
//

#include "raisim/RaisimServer.hpp"
#include "exercise3_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world

  // kinova
  auto a1 = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/a1/urdf/a1.urdf");

  // kinova configuration
  Eigen::VectorXd gc(a1->getGeneralizedCoordinateDim()), gv(a1->getDOF());
  gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;
  a1->setState(gc, gv);

  /// this function updates internal variables in raisim (such as the mass matrix and nonlinearities)
  world.integrate1();

//  std::cout<<"mass matrix should be \n"<< a1->getMassMatrix().e()<<std::endl;
//  std::cout<<"nonlinearities should be \n"<< a1->getNonlinearities({0,0,-9.81}).e()<<std::endl;

  if((getMassMatrix(gc) - a1->getMassMatrix().e()).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed "<<std::endl;

  if((getNonlinearities(gc, gv) - a1->getNonlinearities({0,0,-9.81}).e()).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed "<<std::endl;

  return 0;
}
