#include "SpeedController.h"
#include <cstdio>
#include <cmath>

using namespace std;

SpeedController::SpeedController()
: dummy(10000)
, accel(0)
, jerk(0)
, prevSpeedDiff(0)
, lane(1)
, Kp(0.80)
, Kd(2.00)
{

}

void SpeedController::setSpeedControlParams(double const kp, double const kd, double const ki)
{
  Kp = kp;
  Kd = kd;
}

void SpeedController::initializeLane(int const laneInit)
{
  lane = laneInit;
}

double SpeedController::getSpeed(double const currSpeed)
{
  double targetSpeed = currSpeed;

  double const carAheadSpeed = carAhead.speed;
  double const carAheadDist = carAhead.distance;

  targetSpeed = max_speed;
  if (carAheadDist < maxSafetyDist)
  {
    targetSpeed -= (max_speed - carAheadSpeed) * ((maxSafetyDist - carAheadDist) / safetyRange);
  }

  double const speedDiff = targetSpeed - currSpeed;
  double const speedDiffDiff = speedDiff - prevSpeedDiff;


  double newAccel = (Kp * speedDiff + Kd * speedDiffDiff); // + Ki * speedDiffInt);
  if (newAccel > max_accel)
  {
    newAccel = max_accel;
  }
  else if (newAccel < -max_accel)
  {
    newAccel =-max_accel;
  }
  if ((newAccel - accel) > max_jerk)
  {
    newAccel = accel + max_jerk;
  }
  else if ((newAccel - accel) < -max_jerk)
  {
    newAccel = accel - max_jerk;
  }
  accel = newAccel;

  // Calculate new speed
  targetSpeed = currSpeed + accel*0.02;


  // Limit the new speed to the maximum speed
  if (targetSpeed > max_speed)
  {
    accel = (targetSpeed - max_speed)/0.02;
    targetSpeed = max_speed;
  }

//  std::printf("getSpeed: curr:%4.1f    carAhead:%4.1f/%6.1f    sDiff:%4.1f   sDiffDiff:%4.1f  ->  acc:%4.1f targetSpeed:%4.1f\n",
//              currSpeed, carAheadSpeed, carAheadDist, speedDiff, speedDiffDiff, accel, targetSpeed);

  prevSpeedDiff = speedDiff;

  return targetSpeed;
}


int SpeedController::getLane(std::vector<LaneOccupancy> const & laneOcc)
{
  carAhead = laneOcc[lane].ahead;
  if (maxSafetyDist > carAhead.distance)
  {
    double const leftLaneCarAheadDistance = (lane>0) ? laneOcc[lane-1].ahead.distance : 0;
    double const leftLaneCarBehindDistance = (lane>0) ? laneOcc[lane-1].behind.distance : 0;
    double const rightLaneCarAheadDistance = (lane<2) ? laneOcc[lane+1].ahead.distance : 0;
    double const rightLaneCarBehindDistance = (lane<2) ? laneOcc[lane+1].behind.distance : 0;

    if ((leftLaneCarAheadDistance > carAhead.distance) &&
        (leftLaneCarBehindDistance > minSafetyDist))
    {
//      printf("Change to left lane\n");
      lane--;
    }
    else if ((rightLaneCarAheadDistance > carAhead.distance) &&
             (rightLaneCarBehindDistance > minSafetyDist))
    {
//      printf("Change to right lane\n");
      lane++;
    }
    else
    {
//      printf("Cannot change lane\n");
    }
  }
  return lane;
}
