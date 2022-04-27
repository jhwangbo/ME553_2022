//
// Created by Jemin Hwangbo on 2022/04/08.
//

#include "raisim/RaisimServer.hpp"
#include "exercise4_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  world.setTimeStep(0.002);
  raisim::RaisimServer server(&world);

  // kinova
  auto cart_pole = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/cartPole/cartpole.urdf");

  // kinova configuration
  Eigen::VectorXd gc(cart_pole->getGeneralizedCoordinateDim()), gv(cart_pole->getDOF());
  gc << 0, 1.7; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.0, 0.0;
  cart_pole->setState(gc, gv);

  server.launchServer();
  for (int i=0; i<20000; i++) {
    // world.integrate(); use this line for debugging. If you use this line, comment the following 4 lines
    auto ga = getGeneralizedAcceleration(gc, gv);
    gv = gv + ga * world.getTimeStep();
    gc = gc + gv * world.getTimeStep(); // this is called semi-implicit integration. it uses updated velocities
    cart_pole->setState(gc, gv);

    std::this_thread::sleep_for(std::chrono::microseconds(2000));
  }
  server.killServer();

  return 0;
}
