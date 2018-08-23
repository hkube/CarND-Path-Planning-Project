#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H
#include <vector>

class SpeedController
{
 public:
  typedef struct CarDistance
  {
    CarDistance() : id(-1), distance(999), speed(99) {}

    int id;
    double distance;
    double speed;
  } CarDistance;

  typedef struct
  {
    CarDistance ahead;
    CarDistance behind;
  } LaneOccupancy;

  SpeedController();

  void setSpeedControlParams(double const kp, double const kd, double const ki);
  void initializeLane(int const laneInit);
  double getSpeed(double const currSpeed);
  int getLane(std::vector<LaneOccupancy> const & laneOcc);

 private:
  constexpr static double miles2km = 1.609344;
  constexpr static double km2miles = 1.0 / miles2km;

  constexpr inline static double mph2kmh(double mph) { return mph * miles2km; }
  constexpr inline static double kmh2mph(double kmh) { return kmh * km2miles; }
  constexpr inline static double kmh2mps(double kmh) { return kmh * (1000.0 / 3600.0); }
  constexpr inline static double mps2kmh(double mps) { return mps * 3.6; }

//  constexpr static double max_speed = kmh2mps(mph2kmh(49.0 /* miles/h */));
  constexpr static double max_speed = 0.4470361111 * 49.0 /* m/s */;
  constexpr static double max_accel = 8.0  /* m / s^2 */;
  constexpr static double max_jerk  = 8.0  /* m / s^3 */;
  constexpr static double timeStep  = 0.02 /* seconds */;

  constexpr static double maxSafetyDist = 50;
  constexpr static double minSafetyDist = 20;
  constexpr static double safetyRange = maxSafetyDist - minSafetyDist;


  std::vector<double> dummy;
  double accel;
  double jerk;
  double prevSpeedDiff;
  int lane;
  double Kp;
  double Kd;
  CarDistance carAhead;
};

#endif
