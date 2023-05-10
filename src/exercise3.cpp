//
// Created by Jemin Hwangbo on 2022/04/08.
//

#include "raisim/RaisimServer.hpp"
#include "exercise3_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char *argv[])
{
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);
  world.addGround();
  world.setTimeStep(0.001);

  // kinova
  auto aliengo = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/aliengo/aliengo_modified.urdf");
  // auto aliengo = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/cartPole/cartpole.urdf");
  aliengo->printOutMovableJointNamesInOrder();
  // kinova configuration
  Eigen::VectorXd gc(aliengo->getGeneralizedCoordinateDim()), gv(aliengo->getDOF());
  gc << 0, 23, 23, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;
  std::cout << "gc dim: " << aliengo->getGeneralizedCoordinateDim() << std::endl;
  std::cout << "gv dim: " << aliengo->getDOF() << std::endl;
  // gc << 0, 3, 2, -0.3, -0.4, 1, 0, 0, 0;
  // gv << 3, 1, -0.2, 0, 0, 0, 1, 0;

  aliengo->setState(gc, gv);
  server.focusOn(aliengo);

  /// if you are using an old version of Raisim, you need this line
  world.integrate1();

  std::cout << "mass matrix should be \n"
            << aliengo->getMassMatrix().e() << std::endl;

  server.launchServer();

  while (true)
  {
    RS_TIMED_LOOP(0.1)
    server.integrateWorldThreadSafe();
    // if ((getMassMatrix(gc) - aliengo->getMassMatrix().e()).norm() < 1e-8)
    //   std::cout << "passed " << std::endl;
    // else
    //   std::cout << "failed " << std::endl;
  }

  server.closeConnection();

  return 0;
}
