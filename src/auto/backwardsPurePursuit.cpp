/*
 * @author D Michael Jones, 914M - acetousk
 *
 * The ideas portrayed in this code are mainly from the following
 * documents:
 *      Pure Pursuit            : https://github.com/team914/autolib-pdfs/blob/master/pure-pursuit.pdf
 *      Adaptive Pure Pursuit   : https://github.com/team914/autolib-pdfs/blob/master/adaptive-pure-pursuit.pdf
 *      Path Tracking           : https://github.com/team914/autolib-pdfs/blob/master/path-tracking.pdf
 *      PiLons Position Tracking: https://github.com/team914/autolib-pdfs/blob/master/pilons-position-tracking.pdf
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "auto/backwardsPurePursuit.hpp"
#include "okapi/api.hpp"
#include "api.hpp"

namespace autolib{

BackwardsPurePursuit::BackwardsPurePursuit( const std::vector<IndexedDistancePosePath> &ipaths, const okapi::QLength &ilookaheadDistance, double ispeed, bool iSlowDown ): paths(ipaths){
    #ifdef DEBUG
        printf( "\tInheriting Points from Path\n" );
    #endif
    lookaheadDistance = ilookaheadDistance.convert(okapi::meter);
    speed = ispeed;
    slowDown = iSlowDown;
}

// bool PurePursuit::run( double x, double y, double yaw, const std::string &iid, const std::shared_ptr<okapi::OdomChassisController> &controller  ){
//     //okapi::OdomState pose = std::move(ipose);
//     //std::cout << "run 1\n" << std::flush;
//     return run( x, y, yaw, iid, controller ); //Pose{ pose.x, pose.y, pose.theta }
// }

bool BackwardsPurePursuit::run( double x, double y, double yaw, const std::string &iid, const std::shared_ptr<okapi::OdomChassisController> &controller  ){
    if( path.path.empty() || path.id != iid )
            findPath( iid );
    int index = findNearestPose( Pose{ okapi::QLength{x}, okapi::QLength{y}, okapi::QAngle{yaw} } );
    int goalIndex = findGoalPose( index );

    //flip yaw for backwards pursuit
    yaw = yaw + okapi::pi;
    double yawp = yaw;


    double pgXDiff = path.path[goalIndex].pose.x - x;
    double pgYDiff = path.path[goalIndex].pose.y - y;
    double goalYaw = path.path[goalIndex].pose.yaw;
    double pathYaw = path.path[index].pose.yaw;
    double yawpg = std::atan2(pgYDiff, pgXDiff);

    double distance = findDistanceBetweenPoses(path.path[index].pose,path.path[path.path.size() - 1].pose);
    std::cout << "\n" << std::flush << "distance: " << distance << "\n" << std::flush;

//    double speed = 0.9;
    double left = 0;
    double right = 0;

    double reqVelocity = .9;

    double turnpower = 0;

    double slow = (distance / 1.25) / speed;
    if (slow < 0.7) {
      slow = 0.7;
    } if (slow > 1 || slowDown == 0) {
      slow = 1;
    }

    //double goaldirection = goalYaw - pathYaw;

    double goaldirection = yawpg - yawp;
    while (goaldirection < okapi::pi) {
      goaldirection = goaldirection + (2 * okapi::pi);
    }
    while (goaldirection > okapi::pi) {
      goaldirection = goaldirection - (2 * okapi::pi);
    }
    //std::cout << "turn: " << goaldirection << std::flush;
    if (goalIndex - index < 10) {
      //std::cout << "done  ";
      //goaldirection = path.path[index].pose.yaw;
      left = 0;
      right = 0;
    } else {
      if ((goaldirection < 0.002 && goaldirection > -0.002)) {
        //std::cout << "straight  ";
        left = -speed;
        right = -speed;
      } else {
        turnpower = (std::abs(goaldirection) * 6 / okapi::pi) * speed;
        if (turnpower > speed) turnpower = speed;
        //std::cout << goaldirection << " ";

        // Make the robot slow down and turn gently if reaching the final pose
        /*if (distance < 0.5) {
          turnpower = turnpower * (distance);
        }*/
        if (goaldirection > 0 ) {
          std::cout << "left  ";
          left = -((speed - turnpower) * slow);
          right = -(speed * slow);
        } else {
          std::cout << "right  ";
          left = -(speed * slow);
          right = -((speed - turnpower) * slow);
        }
      }
    }
//    std::cout << currentPosition.x.getValue() << "," << currentPosition.y.getValue() << "\n" << std::flush;
    std::cout << index << "/" << path.path.size() << " " << goalIndex << ":" << goaldirection << " "
    << x << "," << y
    /*<< " yawp: " << yaw
    << " yawpg: " << yawpg
    << "  d: " << goaldirection << " xdiff: " << pgXDiff << " ydiff: " << pgYDiff
    << " left: " << left << " right: " << right
    */<< "\n" << std::flush;
    controller->getModel()->left( left );
    controller->getModel()->right( right );
    //std::cout << "test: " << goalIndex - index << std::flush;
    return goalIndex - index > 10;
}

void BackwardsPurePursuit::findPath( const std::string &iid ){
    for( const auto &ipath: paths ){
        //std::cout << ipath.id << "=" << iid << "\n" << std::flush;
        if( ipath.id == iid ){
          //std::cout << "path found" << std::flush;
            avgDistanceBetweenPoses = ipath.averagePathLength;
            path.id = iid;
            path.averagePathLength = avgDistanceBetweenPoses;
            for( const auto &pose: ipath.path ){
                path.path.emplace_back( pose );
            }
        }
    }
}

double BackwardsPurePursuit::findDistanceBetweenPoses( const InternalPose &P1, const InternalPose &P2 ){
    return std::sqrt( okapi::ipow( P2.x - P1.x, 2 ) + okapi::ipow( P2.y - P1.y, 2 ) );
}

int BackwardsPurePursuit::findNearestPose( const Pose iipose ) {
  InternalPose ipose{ iipose.x.convert(okapi::meter), iipose.y.convert(okapi::meter), iipose.yaw.convert(okapi::radian) };
  int foundIndex = 0;
  double mindistance = 100;
  //std::cout << "path size " << path.path.size() << "\n" <<std::flush;
  for(int i=0; i<path.path.size(); i++) {
    InternalPose pose = path.path.at(i).pose;
    double distance = findDistanceBetweenPoses( pose, ipose );
    if (distance < mindistance) {
      mindistance = distance;
      foundIndex = i;
    }
  }
  return foundIndex;
}


int BackwardsPurePursuit::findGoalPose(int index) {
  for (int i = index+1; i < path.path.size() - 1; i++) {
    if (findDistanceBetweenPoses(path.path[index].pose, path.path[i].pose) >= lookaheadDistance) {
      return i;
    }
  }
  return path.path.size()-1;
}


}//autolib
