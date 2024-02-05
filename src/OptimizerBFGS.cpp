#include "OptimizerBFGS.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerBFGS::OptimizerBFGS()
{
	
}

OptimizerBFGS::~OptimizerBFGS()
{
	
}

VectorXd OptimizerBFGS::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{
	VectorXd x = xInit;
	double alpha = pow(10, -1);
	double epsilon = tol;
	double alpha0 = alphaInit;
	//double gamma = 0.5; //0.8;//
	//int iterMaxLS = 20;
	//int iter = 0;
	//int iterMax = 5;//5;//50;//
	
	int n = xInit.size();
	MatrixXd I(n, n);
	I.setIdentity();
	MatrixXd A = I;
	VectorXd x0(n);
	x0.setZero();
	VectorXd g0(n);
	g0.setZero();


	for (iter = 0; iter < iterMax; iter++) {
		//search direction
		VectorXd grad(n);
		double fx = objective->evalObjective(x, grad); //eval f and g at x
		if (iter > 0) {
			MatrixXd s = x - x0;
			MatrixXd y = grad - g0;
			double rho = 1 / ((y.transpose() * s)(0, 0));
			//A = (Matrix2d::Identity() - (rho * (s * y.transpose()))) * A * (Matrix2d::Identity() - (rho * (y - s.transpose()))) + (rho * (s * s.transpose()));
			A = (I - rho * (s * y.transpose())) * A * (I - rho * (y * s.transpose())) + rho * (s * s.transpose());

		}
		VectorXd p = (- 1 * A) * grad;

		//line search
		VectorXd deltax;
		alpha = alpha0;
		for (iterLS = 0; iterLS < iterMaxLS; iterLS++) {
			deltax = alpha * p;
			double f1 = objective->evalObjective(x + deltax);
			if (f1 < fx) {
				break;
			}
			alpha *= gamma;
		}

		//step
		x0 = x;
		g0 = grad;
		x += deltax;
		double length = grad.norm();
		if (length < epsilon) {
			break;
		}

	}

	iterationsUsed = iter;
	return x;
}
