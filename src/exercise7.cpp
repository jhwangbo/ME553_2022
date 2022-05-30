//
// Created by jemin on 5/30/22.
//


#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "exercise7_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);
  world.setTimeStep(0.002);
  auto kinova = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/kinova/robot.urdf");

  // a1_simplified configuration
  Eigen::VectorXd gc(kinova->getGeneralizedCoordinateDim()), gv(kinova->getDOF());
  gc << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  kinova->setState(gc, gv);

  /// this function updates internal variables in raisim (such as the mass matrix and nonlinearities)
  world.integrate1();
  kinova->getMassMatrix(); /// due to the current bug in RaiSim, we have to compute the mass matrix first to get the bias term
  kinova->articulatedBodyAlgorithm({0,0,-9.81}, 0.002);
  Eigen::MatrixXd massMatrixInv = kinova->getInverseMassMatrix().e();
  Eigen::VectorXd nonlin = kinova->getNonlinearities(raisim::Vec<3>{0,0,-9.81}).e();

  if((computeGeneralizedAcceleration(gc, gv) -  massMatrixInv * (-nonlin)).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed "<<std::endl;

//  server.launchServer();
//
//  while (true) {
//    server.integrateWorldThreadSafe();
//    usleep(10000);
//  }
//  server.killServer();
  return 0;
}
