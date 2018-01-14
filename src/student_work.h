#pragma once

#include "util.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "spline.h"
#include <iostream>

using namespace std;
//not used
vector<double> solve(int t0, int tf, vector<double>& ts) {
	
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(6,6);

	for (int i = 0; i < 6; i++) {
		matrix(0, i) = pow(t0, i);
		matrix(3, i) = pow(tf, i);
	}

	matrix(1, 0) = 0; matrix(1, 1) = 1; matrix(1, 2) = 2*t0; matrix(1,3) = 3*pow(t0,2);
	matrix(1, 4) = 4*pow(t0,3); matrix(1,5) = 5*pow(t0,4); 

	matrix(2, 0) = 0; matrix(2,1) = 0; matrix(2,2) = 2; matrix(2,3) = 6*t0; matrix(2,4) = 12*t0*t0;
	matrix(2, 5) = 20*t0*t0*t0;

	matrix(4,0)  = 0; matrix(4, 1) = 1; matrix(4, 2) = 2*tf; matrix(4,3) = 3*pow(tf,2);
	matrix(4, 4) = 4*pow(tf,3); matrix(4,5) = 5*pow(tf,4); 

	matrix(5, 0) = 0; matrix(5,1) = 0; matrix(5,2) = 2; matrix(5,3) = 6*tf; matrix(5,4) = 12*tf*tf;
	matrix(5, 5) = 20*tf*tf*tf;

	Eigen::VectorXd right(6);
	for (int i = 0; i < 6; i++) {
		right(i) = ts[i];
	}

	Eigen::VectorXd ans = matrix.inverse() * right;

	vector<double> coefficients(6, 0);

	for (int i = 0; i < 6; i++)
		coefficients[i] = ans(i);
	return coefficients;
}

int getLaneID(double d) {
	return int(d/4);
}

//not used
bool isTooCloseAhead(OtherCar& other, Car& car) {
	double threshhold = 30;
	return (other.s - car.s) < threshhold && (other.s - car.s) > 0 
			&& getLaneID(car.d) == getLaneID(other.d);
}
//not used
bool isTooCloseBehind(OtherCar& other, Car& car) {
	double threshhold = 10;
	return (car.s - other.s) < threshhold && (other.s - car.s) > 0 && 
			getLaneID(car.d) == getLaneID(other.d); 
}
//not used
vector<int> laneOptions(Car& car) {
	vector<int> options;
	int laneID = getLaneID(car.d);
	options.push_back(laneID);
	if (laneID < 2) {
		options.push_back(laneID + 1);
	}
	if (laneID > 0) {
		options.push_back(laneID - 1);
	}
	return options;
}
//not used
vector<int> getPossibleCars(vector<OtherCar>& otherCars, Car& car) {

	double horizon = 60;

	vector<int> indexes;

	for (int i = 0; i < otherCars.size(); i++) {
		auto& item = otherCars[i];
		int laneID = getLaneID(item.d);
		int mylaneID = getLaneID(car.d);
		if (laneID < 0 || laneID > 2) continue;
		if (fabs(item.s - car.s) < horizon && fabs(laneID-mylaneID)<2) {
			indexes.push_back(i);
		}
	}

	return indexes;
}


// not used get the current speed at thims time
double  get_ref_speed(vector<OtherCar> &otherCars, Car &car) {
	
	static double ref_speed = 0.0;
	vector<int> indexes = getPossibleCars(otherCars, car);

	for (auto index : indexes) {
		auto& othercar = otherCars[index];
		if (isTooCloseAhead(othercar, car)) {
			ref_speed -= 0.2;
			return ref_speed;
		} 
	}
	if (ref_speed < 45) ref_speed += 0.5;
	return ref_speed;
}

//predict the surrounding movement in the next 0.02 seconds
CarStatus Prediction (vector<OtherCar>& otherCars, Car& mycar, vector<PrePath>& Pre_paths) {

	CarStatus carStatus;
	
	int myLane = getLaneID(mycar.d);

	for (auto& otherCar : otherCars) {
		int carLane = getLaneID(otherCar.d);
		if (carLane < 0 || carLane > 2) continue;

		double vx = otherCar.vx;
		double vy = otherCar.vy;
		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = otherCar.s;

		//prediction for the next 0.02 second
		check_car_s += ((double)Pre_paths.size() * 0.02 * check_speed);
		
		if (carLane == myLane) {
			carStatus.carAhead |= (check_car_s > mycar.s && check_car_s - mycar.s < 30);
		} else if (carLane - myLane == -1) {
			carStatus.carLeft |= ((mycar.s - 30 < check_car_s) && (mycar.s + 30 > check_car_s));
		} else {
			carStatus.carRight |= ((mycar.s - 30 < check_car_s) && (mycar.s + 30 > check_car_s));
		}
	}

	return carStatus;
}

Decision DecisionMaking(CarStatus& carStatus, Car& mycar) {

	Decision decision;
	double speed_diff = 0.0;
	const double MAX_SPEED = 47.5;
	const double MAX_ACC = .224;
	static double ref_speed = 0.0;
	
	int myLane = getLaneID(mycar.d);
	decision.lane = myLane;

	if (carStatus.carAhead) {
		if (!carStatus.carLeft && myLane > 0) {
			decision.lane = myLane - 1;
		} else if (!carStatus.carRight && myLane !=2) {
			decision.lane = myLane + 1;
		} else {
			speed_diff -= MAX_ACC;
		}
	} else {
		if (myLane != 1) {
			if ( (myLane == 0 && !carStatus.carRight) || (myLane == 2 && !carStatus.carLeft)) {
				decision.lane = 1;
			}
		} 
		if (ref_speed < MAX_SPEED) {
			speed_diff += MAX_ACC;
		}
	}

	ref_speed += speed_diff;
	decision.speed = ref_speed;
	return decision;
}

//main function to get path;
void generatePath(vector<double>& pathX, vector<double>& pathY, Car& car, vector<WayPoint>& maps,
	vector<OtherCar>& otherCars, vector<PrePath>& Pre_paths) {

	//get ref_speed 
	// double ref_speed = get_ref_speed(otherCars, car);
	auto carStatus =  Prediction(otherCars, car, Pre_paths);
	auto decision  =  DecisionMaking(carStatus, car); 
	// int lane = 1;

	//find the lane to drive and reference speed to drive on
	int lane = decision.lane;
	double ref_speed = decision.speed;


	int prev_path_size = Pre_paths.size();

	vector<double> ptsx, ptsy;

	double ref_x   = car.x;
	double ref_y   = car.y;
	double ref_yaw = deg2rad(car.yaw);

	if (prev_path_size < 2) {
		double prev_car_x = car.x - cos(car.yaw);
		double prev_car_y = car.y - sin(car.yaw);

		ptsx.push_back(prev_car_x);
		ptsy.push_back(prev_car_y);

		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y);

	} else {
		ref_x = Pre_paths[prev_path_size-1].x;
		ref_y = Pre_paths[prev_path_size-1].y;

		double ref_x_prev = Pre_paths[prev_path_size-2].x;
		double ref_y_prev = Pre_paths[prev_path_size-2].y;

		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
		ptsx.push_back(ref_x_prev);
		ptsy.push_back(ref_y_prev);

		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y);
	}

	double spacing = 30.0;

	vector<double> next_wp0 = getXY(car.s + spacing, (2 + 4*lane), maps);
	vector<double> next_wp1 = getXY(car.s + 2 * spacing, (2 + 4 * lane), maps);
	vector<double> next_wp2 = getXY(car.s + 3 * spacing, (2 + 4 * lane), maps);	

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	/*
	 * Convert points to frame of ref relative to car
	*/

	for (int i = 0; i < ptsx.size(); i++) {
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}

	tk::spline s;
	s.set_points(ptsx, ptsy);

	for (int i = 0; i < prev_path_size; i++) {
		pathX.push_back(Pre_paths[i].x);
		pathY.push_back(Pre_paths[i].y);
	}

	//set horizon
	double target_x = 30;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);

	int NUM_POINTS = 50;
	double x_start = 0.0;
	double delta_t = 0.02;
	double N = (target_dist)/(delta_t * mph_to_ms(ref_speed));

	for (int i = 0; i < NUM_POINTS - prev_path_size; i++) {

		double x_point = x_start + target_x / N;
		double y_point = s(x_point);

		x_start = x_point;

		//convert back from car's frame of ref
		double x_ref = x_point;
		double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        pathX.push_back(x_point);
        pathY.push_back(y_point);
	}

}

// void generatePath(vector<double>& pathX, vector<double>& pathY, Car& car, vector<WayPoint>& maps,
// 	vector<SensorInfo>& Sensors, vector<PrePath>& Pre_paths) {

// 	int last_point = Pre_paths.size() - 1;
// 	double T = 2;

// 	double targetSpeed = 50;
// 	if (car.speed < 10) {
// 		targetSpeed = 10;
// 	} else if (car.speed < 20) {
// 		targetSpeed = 20;
// 	} else if (car.speed < 30) {
// 		targetSpeed = 30;
// 	}else if (car.speed < 40) {
// 		targetSpeed = 40;
// 	} else {
// 		targetSpeed = 50;
// 	}

// 	double si = 0, si_dot = 0, si_ddot = 0;
// 	double di = 0, di_dot = 0, di_ddot = 0;
// 	double sf = 0, sf_dot = 0, sf_ddot = 0;
// 	double df = 0, df_dot = 0, df_ddot = 0;

// 	if (last_point == -1) {
// 		si = car.s;
// 		si_dot = 0;
// 		si_ddot = 0;

// 		di = car.d;
// 		di_dot = 0;
// 		di_ddot = 0;
// 	} else {

// 		si = Pre_paths[last_point].s;
// 		si_dot = 0;
// 		si_ddot = 0;

// 		di = Pre_paths[last_point].d;
// 		di_dot = 0;
// 		di_ddot = 0;
// 	}

// 	df       = car.d;
// 	df_dot   = 0;
// 	df_ddot  = 0;

// 	sf_dot   = mph_to_ms(targetSpeed); 
// 	sf_ddot  = 0;
// 	sf       = si + sf_dot * T;

// 	cout << "si: " << si << "sf: " << sf << endl;

// 	vector<double> s_conditions = {si, si_dot, si_ddot, sf, sf_dot, sf_ddot};

// 	vector<double> s_coefficients = solve(0, T, s_conditions);

// 	vector<double> d_conditions = {di, di_dot, di_ddot, df, df_dot, df_ddot};

// 	vector<double> d_coefficients = solve(0, T, d_conditions);

// 	vector<double> trajectory_s, trajectory_d;

// 	Pre_paths 

// 	double t = 0;
// 	for (int i = ) {
// 		double s, d;
// 		s = s_coefficients[0] + s_coefficients[1] * t + s_coefficients[2] * pow(t, 2) 
// 				+ s_coefficients[3] * pow(t, 3) + s_coefficients[4] * pow(t, 4) + s_coefficients[5] * pow(t, 5);
// 		d = d_coefficients[0] + d_coefficients[1] * t + d_coefficients[2] * pow(t, 2) 
// 			+ d_coefficients[3] * pow(t, 3) + d_coefficients[4] * pow(t, 4) + d_coefficients[5] * pow(t, 5);

// 	}
// }
 