#pragma once

#include <math.h>
#include <vector>

using namespace std;

struct WayPoint{
  double x;
  double y;
  double s;
  double dx;
  double dy;
};

struct Car {
	int id;
  	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
};

struct PrePath{
	double x;
	double y;
	double s;
	double d;
};

struct OtherCar {
	int carid;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
};

struct CarStatus{
	bool carAhead;
	bool carLeft;
	bool carRight;
	CarStatus(bool _carAhead=false, bool _carLeft=false, bool _carRight=false):
	 carAhead(_carAhead), carLeft(_carLeft), carRight(_carRight){}
};

struct Decision {
	int lane;
	double speed;
};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mph_to_ms(double mile) {return mile / 2.24;}

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<WayPoint> &maps);

// double mile_to_meters(double mile);
