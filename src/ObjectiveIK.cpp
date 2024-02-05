#include "ObjectiveIK.h"
#include <sstream>
#include <cmath>

using namespace Eigen;

ObjectiveIK::ObjectiveIK()
{

}

ObjectiveIK::~ObjectiveIK()
{

}

double ObjectiveIK::evalObjective(const VectorXd &x) const
{
	/*
	double wtar = pow(10, 3);
	double wreg = pow(10, 0);
	VectorXd r(3);
	r << 1, 0, 1;
	std::vector<MatrixXd> R;
	std::vector<MatrixXd> T;

	// T1 which is the root
	Matrix3d T1;
	T1 << 1, 0, 0,
		  0, 1, 0,
		  0, 0, 1;

	//R1
	Matrix3d R1;
	R1 << cos(x(0)), -sin(x(0)), 0, //we can use the first value of x because x rn is a scalar
		  sin(x(0)),  cos(x(0)), 0,
			      0,          0, 1;
	std::stringstream check;
	check << R1;

	Matrix3d R1_prime;
	R1_prime << -sin(x(0)), -cos(x(0)), 0,
				 cos(x(0)), -sin(x(0)), 0,
						0,			0, 0;
	std::stringstream().swap(check);
	check << R1_prime;

	MatrixXd temp = T1 * R1 * r; //for now, fix later
	MatrixXd p(2, 1);
	p << temp(0), temp(1);
	std::stringstream().swap(check);
	check << p;

	MatrixXd temp2 = T1 * R1_prime * r;
	MatrixXd p_prime(2, 1);
	p_prime << temp2(0), temp2(1);
	std::stringstream().swap(check);
	check << p_prime;

	//MatrixXd ptarget = p;`
	//ptarget << 0, 1, 1;
	std::stringstream().swap(check);
	check << ptarget;
	
	//MatrixXd ptarget = p;
	MatrixXd deltap = p - (this->ptarget);
	std::stringstream().swap(check);
	check << deltap;

	double f = 0.5 * wtar * ((deltap.transpose() * deltap)(0,0)) + 0.5 * wreg * pow(x(0), 2);
	*/

	int n = x.size(); //number of links
	VectorXd X = x;

	double wtar = pow(10, 3);
	MatrixXd Wreg(n, n);
	Wreg.setZero();
	for (int i = 0; i < n; i++) {
		Wreg(i, i) = wreg[i];
	}
	VectorXd r(3); 
	r << 1, 0, 1; 
	std::vector<MatrixXd> R;
	std::vector<MatrixXd> R_prime;
	std::vector<MatrixXd> T;

	for (int i = 0; i < n; i++) {
		Matrix3d Ttemp;
		if (i == 0) {//root
			Ttemp << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;
		}
		else {
			Ttemp << 1, 0, 1,
				0, 1, 0,
				0, 0, 1;
		}
		T.push_back(Ttemp);
	}

	for (int i = 0; i < n; i++) {
		Matrix3d Rtemp;
		Rtemp << cos(X(i)), -sin(X(i)), 0,
			sin(X(i)), cos(X(i)), 0,
			0, 0, 1;
		R.push_back(Rtemp);
	}

	/*for (int i = 0; i < n; i++) {
		Matrix3d R_primetemp;
		R_primetemp << -sin(X(i)), -cos(X(i)), 0,
			cos(X(i)), -sin(X(i)), 0,
			0, 0, 0;
		R_prime.push_back(R_primetemp);
	}*/

	std::stringstream check;

	MatrixXd temp(3, 1); 
	temp = r; //start with r since it's at the end
	for (int i = n - 1; i >= 0; i--) {
		temp = T[i] * R[i] * temp; //(TxRx) * (Tx+1)(Rx+1)r
	}
	MatrixXd p(2, 1);
	p << temp(0), temp(1); //getting rid of the homogenous coord
	std::stringstream().swap(check);
	check << p;


	//MatrixXd P_prime(2, n);
	//for (int i = 0; i < n; i++) {
	//	//MatrixXd temp2 = T1 * R1_prime * r;
	//	MatrixXd temp2(3, 1);
	//	temp2 = r;
	//	for (int j = n - 1; j >= 0; j--) { // j is decreasing
	//		//p'_i = 
	//		if (j == i) {
	//			temp2 = T[j] * R_prime[j] * temp2;
	//		}
	//		else {
	//			temp2 = T[j] * R[j] * temp2;
	//		}
	//	}
	//	MatrixXd p_prime(2, 1); //p'_i;
	//	p_prime << temp2(0), temp2(1);
	//	P_prime.block<2, 1>(0, i) = p_prime;
	//}

	/*std::stringstream().swap(check);
	check << P_prime;*/

	std::stringstream().swap(check);
	check << ptarget;

	MatrixXd deltap = p - this->ptarget;
	std::stringstream().swap(check);
	check << deltap;

	double f = 0.5 * wtar * ((deltap.transpose() * deltap)(0, 0)) + 0.5 * ((X.transpose() * Wreg * X)(0, 0));
	std::stringstream().swap(check);
	check << f;


	return f;
}

double ObjectiveIK::evalObjective(const VectorXd &x, VectorXd &g) const
{
	int n = x.size(); //number of links
	VectorXd X = x;

	double wtar = pow(10, 3);
	MatrixXd Wreg(n, n);
	Wreg.setZero();
	for (int i = 0; i < n; i++) {
		Wreg(i, i) = wreg[i];
	}
	VectorXd r(3);
	r << 1, 0, 1;
	std::vector<MatrixXd> R;
	std::vector<MatrixXd> R_prime;
	std::vector<MatrixXd> T;

	for (int i = 0; i < n; i++) {
		Matrix3d Ttemp;
		if (i == 0) {//root
			Ttemp << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;
		}
		else {
			Ttemp << 1, 0, 1,
				0, 1, 0,
				0, 0, 1;
		}
		T.push_back(Ttemp);
	}

	for (int i = 0; i < n; i++) {
		Matrix3d Rtemp;
		Rtemp << cos(X(i)), -sin(X(i)), 0,
			sin(X(i)), cos(X(i)), 0,
			0, 0, 1;
		R.push_back(Rtemp);
	}

	for (int i = 0; i < n; i++) {
		Matrix3d R_primetemp;
		R_primetemp << -sin(X(i)), -cos(X(i)), 0,
			cos(X(i)), -sin(X(i)), 0,
			0, 0, 0;
		R_prime.push_back(R_primetemp);
	}

	std::stringstream check;

	MatrixXd temp(3, 1);
	temp = r; //start with r since it's at the end
	for (int i = n - 1; i >= 0; i--) {
		temp = T[i] * R[i] * temp; //(TxRx) * (Tx+1)(Rx+1)r
	}
	MatrixXd p(2, 1);
	p << temp(0), temp(1); //getting rid of the homogenous coord
	std::stringstream().swap(check);
	check << p;


	MatrixXd P_prime(2, n);
	for (int i = 0; i < n; i++) {
		//MatrixXd temp2 = T1 * R1_prime * r;
		MatrixXd temp2(3, 1);
		temp2 = r;
		for (int j = n - 1; j >= 0; j--) { // j is decreasing
			//p'_i = 
			if (j == i) {
				temp2 = T[j] * R_prime[j] * temp2;
			}
			else {
				temp2 = T[j] * R[j] * temp2;
			}
		}
		MatrixXd p_prime(2, 1); //p'_i;
		p_prime << temp2(0), temp2(1);
		P_prime.block<2, 1>(0, i) = p_prime;
	}

	std::stringstream().swap(check);
	check << P_prime;

	std::stringstream().swap(check);
	check << ptarget;

	MatrixXd deltap = p - this->ptarget;
	std::stringstream().swap(check);
	check << deltap;

	double f = 0.5 * wtar * ((deltap.transpose() * deltap)(0, 0)) + 0.5 * ((X.transpose() * Wreg * X)(0, 0));

	VectorXd gradient(n); 

	VectorXd delta_times_prime(n);
	for (int i = 0; i < n; i++) {
		Vector2d p_prime_i = P_prime.block<2,1>(0,i);
		delta_times_prime(i) = (deltap.transpose() * p_prime_i)(0,0);
	}
	gradient = wtar * ((deltap.transpose() * P_prime).transpose()) + (Wreg * X);
	//gradient = (wtar * delta_times_prime) + (Wreg * X);

	std::stringstream().swap(check);
	check << gradient;
	
	g = gradient;

	return f;
}

double ObjectiveIK::evalObjective(const VectorXd &x, VectorXd &g, MatrixXd &H) const
{
	return 0.0;
}
