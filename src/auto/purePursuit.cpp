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
#include "auto/purePursuit.hpp"
#include "okapi/api.hpp"
#include "api.hpp"

namespace autolib{

PurePursuit::PurePursuit( const std::vector<IndexedDistancePosePath> &ipaths, const okapi::QLength &ilookaheadDistance, bool iSlowDown): paths(ipaths){
    #ifdef DEBUG
        printf( "\tInheriting Points from Path\n" );
    #endif
    lookaheadDistance = ilookaheadDistance.convert(okapi::meter);
    slowDown = iSlowDown;
}

// bool PurePursuit::run( double x, double y, double yaw, const std::string &iid, const std::shared_ptr<okapi::OdomChassisController> &controller  ){
//     //okapi::OdomState pose = std::move(ipose);
//     //std::cout << "run 1\n" << std::flush;
//     return run( x, y, yaw, iid, controller ); //Pose{ pose.x, pose.y, pose.theta }
// }

okapi::Motor left1(15);
okapi::Motor left2(16);
okapi::Motor left3(17);

okapi::Motor right1(-18);
okapi::Motor right2(-19);
okapi::Motor right3(-20);

bool PurePursuit::run( double x, double y, double yaw, const std::string &iid, const std::shared_ptr<okapi::OdomChassisController> &controller, double ispeed  ){
    if( path.path.empty() || path.id != iid )
            findPath( iid );
    int index = findNearestPose( Pose{ okapi::QLength{x}, okapi::QLength{y}, okapi::QAngle{yaw} } );
    int goalIndex = findGoalPose( index );
    double yawp = yaw;
    double pgXDiff = path.path[goalIndex].pose.x - x;
    double pgYDiff = path.path[goalIndex].pose.y - y;
    double goalYaw = path.path[goalIndex].pose.yaw;
    double pathYaw = path.path[index].pose.yaw;
    double yawpg = std::atan2(pgYDiff, pgXDiff);
    double speed = ispeed;

    double distance = findDistanceBetweenPoses(path.path[index].pose,path.path[path.path.size() - 1].pose);
    //std::cout << "\n" << std::flush << "distance: " << distance << "\n" << std::flush;

//    double speed = 0.9;
    double left = 0;
    double right = 0;

    double reqVelocity = .9;

    double turnpower = 0;

    double slow = (distance / 1.25) / speed;
    if (slow < 0.3) {
      slow = 0.3;
    } if (slow > 1 || slowDown == 0) {
      slow = 1;
    }
    if (slow * speed < 0.15) {
      slow = 0.15;
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
        left = speed;
        right = speed;
      } else {
        turnpower = (std::abs(goaldirection) * 4 / okapi::pi) * speed;
        if (turnpower > speed) turnpower = speed;
        //std::cout << goaldirection << " ";
        if (goaldirection < 0 ) {
          //std::cout << "left  ";
          left = (speed - turnpower) * slow;
          right = speed * slow;
        } else {
          //std::cout << "right  ";
          left = speed * slow;
          right = (speed - turnpower) * slow;
        }
      }
    }
    //std::cout << currentPosition.x.getValue() << "," << currentPosition.y.getValue() << "\n" << std::flush;
    std::cout << index << "/" << path.path.size() << " " << goalIndex << ":" << goaldirection << " "
    << x << "," << y
    << " yawp: " << yaw
    << " yawpg: " << yawpg
    << "  d: " << goaldirection << " yawpg: " << yawpg << " xdiff: " << pgXDiff << " ydiff: " << pgYDiff
    << " left: " << left << " right: " << right
    << "\n" << std::flush;
    if (left == 0) {
      left1.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
      left2.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
      left3.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    } else {
      left1.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      left2.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      left3.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      controller->getModel()->left( left );
    }
    if (right == 0) {
      right1.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
      right2.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
      right3.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    } else {
      right1.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      right2.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      right3.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      controller->getModel()->right( right );
    }

    //std::cout << "test: " << goalIndex - index << std::flush;
    return goalIndex - index > 10;
}

void PurePursuit::findPath( const std::string &iid ){
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

double PurePursuit::findDistanceBetweenPoses( const InternalPose &P1, const InternalPose &P2 ){
    return std::sqrt( okapi::ipow( P2.x - P1.x, 2 ) + okapi::ipow( P2.y - P1.y, 2 ) );
}

int PurePursuit::findNearestPose( const Pose iipose ) {
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


int PurePursuit::findGoalPose(int index) {
  for (int i = index+1; i < path.path.size() - 1; i++) {
    if (findDistanceBetweenPoses(path.path[index].pose, path.path[i].pose) >= lookaheadDistance) {
      return i;
    }
  }
  return path.path.size()-1;
}

}//autolib
