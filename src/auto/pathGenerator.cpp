/*
 * @author D Michael Jones, 914M - acetousk
 * @author Ryan Benasutti, WPI - Octogonapus
 *
 * The ideas portrayed in this code are mainly from the following
 * documents:
 *      Pure Pursuit: https://github.com/team914/autolib-pdfs/blob/master/pure-pursuit.pdf
 *      Pathfinder  : https://github.com/JacisNonsense/Pathfinder/issues
 *      Desmos      : https://www.desmos.com/calculator/bulcgjwydy
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "auto/pathGenerator.hpp"
#include <initializer_list>
#include <vector>
#include <limits>

namespace autolib{

PathGenerator::PathGenerator( const okapi::PathfinderLimits &ilimits ){
    limits = ilimits;

    const std::shared_ptr<okapi::Logger> &ilogger = okapi::Logger::getDefaultLogger();
    logger = ilogger;
}

void PathGenerator::generatePath(const std::initializer_list<Pose> &iwaypoints, const std::string &iid){
    generatePath( iwaypoints, iid, limits );
}

/**
 * This code was taken from OkapiLib most credit regarding this code goes to the authors of the file linked
 * below:
 *      https://github.com/OkapiLib/OkapiLib/blob/f0da55095b128fdfed18fab68c232569e2a69d06/src/api/control/async/asyncMotionProfileController.cpp#L67
 */
void PathGenerator::generatePath(   const std::initializer_list<Pose> &iwaypoints,
                                    const std::string &iid,
                                    const okapi::PathfinderLimits &ilimits ){
    if (iwaypoints.size() == 0) {
        // No point in generating a path
        std::cout <<
        "AsyncMotionProfileController: Not generating a path because no waypoints were given.\n" << std::flush;
        return;
    }

    std::vector<Waypoint> points;
    points.reserve(iwaypoints.size());
    for (auto &point : iwaypoints) {
        points.push_back(
        Waypoint{point.x.getValue(), point.y.getValue(), point.yaw.convert(okapi::radian)});
        std::cout << "Waypoint: " << point.x.getValue() << "," << point.y.getValue() << std::flush;
    }

    std::cout << "AsyncMotionProfileController: Preparing trajectory\n" << std::flush;

    TrajectoryCandidate candidate;
    pathfinder_prepare(points.data(),
                        static_cast<int>(points.size()),
                        FIT_HERMITE_CUBIC,
                        PATHFINDER_SAMPLES_FAST,
                        0.010,
                        ilimits.maxVel,
                        ilimits.maxAccel,
                        ilimits.maxJerk,
                        &candidate);

    const int length = candidate.length;
    auto *trajectory = new Segment[length];
    pathfinder_generate(&candidate, trajectory);



    // Free the old path before overwriting it
    //forceRemovePath(iid);

    std::vector<InternalDistancePoseIndexed> poses;
    double totalPathLengths;
    for( int i = 0; i < candidate.length; i++ ){
        std::cout.precision(4);
        std::cout << "PathGenerator: Pose Generated: " << std::fixed << trajectory[i].x << ", " << trajectory[i].y << ", " << trajectory[i].heading << "\n" << std::flush;
        totalPathLengths += candidate.totalLength;
        poses.emplace_back( InternalDistancePoseIndexed{ InternalPose{trajectory[i].x, trajectory[i].y, trajectory[i].heading}, candidate.path_length, candidate.totalLength } );
    }
    std::cout << "Total Length: " << totalPathLengths << "  Avg Length: " << candidate.totalLength / candidate.length << std::flush;

    free(trajectory);
    IndexedDistancePosePath path{ iid,  poses, candidate.totalLength / candidate.length };

    paths.emplace_back( path );

    std::cout << "AsyncMotionProfileController: Completely done generating path " + iid+ "\n" << std::flush;
    std::cout << "AsyncMotionProfileController: Path length: " + std::to_string(length) +"\n" << std::flush;
}

void PathGenerator::showPath(const std::string &iid){
  //std::cout<< "start\n" << std::flush;
    for( const auto &ipath: paths ) {
      if( ipath.id == iid ) {
        for(auto & p : ipath.path) {
          //std::cout << p.index << ": " << p.pose.x << ": \n" << std::flush;
        }
      }
    }
  //std::cout<< "finish\n" << std::flush;
}

std::vector<IndexedDistancePosePath> &PathGenerator::getPaths(){
    return paths;
}

}//autolib
