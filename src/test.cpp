#include "student_work.h"
#include <iostream>
#define EPS 1e-6

bool test_solver() {

	Eigen::VectorXd coefficients = Eigen::VectorXd::Random(6);

	int t1 = 1;
	int t2 = 2;

	double res1 = coefficients(0) + coefficients(1) * t1 + coefficients(2) * pow(t1, 2) 
				+ coefficients(3) * pow(t1, 3) + coefficients(4) * pow(t1, 4) + coefficients(5) * pow(t1, 5);
    
    double res2 = coefficients(1) + 2*coefficients(2)*t1 + 3*coefficients(3)*pow(t1,2)
    			+ 4*coefficients(4)*pow(t1,3) + 5*coefficients(5)*pow(t1,4);

    double res3 = 2*coefficients(2) + 6 * coefficients(3) * t1 + 12 * coefficients(4) * pow(t1, 2)
    			+ 20 * coefficients(5) * pow(t1, 3);

   	double res4 = coefficients(0) + coefficients(1) * t2 + coefficients(2) * pow(t2, 2) 
				+ coefficients(3) * pow(t2, 3) + coefficients(4) * pow(t2, 4) + coefficients(5) * pow(t2, 5);
    double res5 = coefficients(1) + 2*coefficients(2) * t2 + 3*coefficients(3) * pow(t2, 2)
    			+ 4*coefficients(4)*pow(t2,3) + 5*coefficients(5)*pow(t2,4);

    double res6 = 2*coefficients(2) + 6 * coefficients(3) * t2 + 12 * coefficients(4) * pow(t2, 2)
    			+ 20 * coefficients(5) * pow(t2, 3);

   vector<double> res = {res1, res2, res3, res4, res5, res6};

   vector<double> ans = solve(t1, t2, res);

   // cout << ans << endl;
    for (int i = 0; i < 6; i++) {
		if (fabs(coefficients(i) - ans[i]) > EPS)
			return false;
	}

	return true;
    
}

int main () {
	
	if(!test_solver()) {
		std::cerr << "Failure in test!" << std::endl; 
	} else {
		std::cout << "You have passed the test_solver test" << endl;
	}
}