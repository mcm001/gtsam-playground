/**
 * @file ISAM2Example_SmartFactor.cpp
 * @brief test of iSAM with smart factors, led to bitbucket issue #367
 * @author Alexander (pumaking on BitBucket)
 */

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <iostream>
#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::P;
using symbol_shorthand::X;

// Make the typename short so it looks much cleaner
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

/**
 * This test does a couple things:
 * - has a camera located at a known pose relative to the body
 * - camera has known calibration and pixel noise
 * - we start at a known pose (if i don't i get an indeterminant system; 
 *   need more than one pixel observation on first loop to fully determine our pose3)
 * - between states we use odometry BetweenFactors (which I -think- act similar to twists)
 * - at each timestamp we observe a fixed point in space through our camera
*/
int main(int argc, char* argv[]) {
  Cal3_S2::shared_ptr K(new Cal3_S2(1000, 1000, 0.0, 320, 240));

  auto measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  Vector6 sigmas;
  sigmas << Vector3::Constant(0.1), Vector3::Constant(0.3);
  auto posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);

  // same odometry noise 
  Vector6 odomSigma;
  odomSigma << Vector3::Constant(0.01), Vector3::Constant(0.05);
  auto odometryNoise = noiseModel::Diagonal::Sigmas(odomSigma);

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  parameters.cacheLinearizedFactors = false;
  parameters.enableDetailedResults = true;
  parameters.print();
  ISAM2 isam(parameters);

  // Create a factor graph
  NonlinearFactorGraph graph;
  Values initialEstimate;

  Point3 point(0.0, 0.0, 1.0);

  // Intentionally initialize the variables off from the ground truth
  Pose3 delta(Rot3::Rodrigues(0.0, 0.0, 0.0), Point3(0.05, -0.10, 0.20));

  // Ground truth poses for our robot's trajectory
  Pose3 pose1(Rot3(), Point3(0.0, 0.0, 0.0));
  Pose3 pose2(Rot3(), Point3(0.0, 0.2, 0.0));
  Pose3 pose3(Rot3(), Point3(0.0, 0.4, 0.0));
  Pose3 pose4(Rot3(), Point3(0.0, 0.5, 0.0));
  Pose3 pose5(Rot3(), Point3(0.0, 0.6, 0.0));

  vector<Pose3> poses = {pose1, pose2, pose3, pose4, pose5};

  // Add first pose
  graph.addPrior(X(0), poses[0], posePriorNoise);
  initialEstimate.insert(X(0), poses[0].compose(delta));

  // Create smart factor with measurement from first pose only
  Pose3 body_P_sensor {Rot3{}, Point3{}};
  SmartFactor::shared_ptr smartFactor(new SmartFactor(measurementNoise, K, body_P_sensor));
  smartFactor->add(PinholePose<Cal3_S2>(poses[0], K).project(point), X(0));
  graph.push_back(smartFactor);

  // loop over remaining poses
  for (size_t i = 1; i < 5; i++) {
    cout << "****************************************************" << endl;
    cout << "i = " << i << endl;

    // Add odometry factor connecting us to the previous node

    Pose3 twist = poses[i-1].transformPoseTo(poses[i]);
    cout << "Twist= " << twist << endl;
    graph.emplace_shared<BetweenFactor<Pose3>>(X(i-1), X(i), twist, odometryNoise);
    initialEstimate.insert(X(i), poses[i].compose(delta));


    // "Simulate" measurement from this pose
    PinholePose<Cal3_S2> camera(poses[i], K);
    Point2 measurement = camera.project(point);
    cout << "Measurement " << i << ": [" << measurement << "]" << endl;

    // Add measurement to smart factor
    smartFactor->add(measurement, X(i));

    // Update iSAM2
    ISAM2Result result = isam.update(graph, initialEstimate);
    cout << "Isam2 results: ";
    result.print();

    cout << "> Detailed results:" << endl;
    for (auto keyedStatus : result.detail->variableStatus) {
      const auto& status = keyedStatus.second;
      PrintKey(keyedStatus.first);
      cout << " {" << endl;
      cout << "reeliminated: " << status.isReeliminated << endl;
      cout << "relinearized above thresh: " << status.isAboveRelinThreshold
           << endl;
      cout << "relinearized involved: " << status.isRelinearizeInvolved << endl;
      cout << "relinearized: " << status.isRelinearized << endl;
      cout << "observed: " << status.isObserved << endl;
      cout << "new: " << status.isNew << endl;
      cout << "in the root clique: " << status.inRootClique << endl;
      cout << "}" << endl;
    }

    Values currentEstimate = isam.calculateEstimate();
    currentEstimate.print("Current estimate: ");

    boost::optional<Point3> pointEstimate = smartFactor->point(currentEstimate);
    if (pointEstimate) {
      cout << *pointEstimate << endl;
    } else {
      cout << "Point degenerate." << endl;
    }

    // Reset graph and initial estimate for next iteration
    graph.resize(0);
    initialEstimate.clear();
  }

  return 0;
}
