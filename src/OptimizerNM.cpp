#include "OptimizerNM.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerNM::OptimizerNM()
{
	
}

OptimizerNM::~OptimizerNM()
{
	
}

VectorXd OptimizerNM::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{
	VectorXd x = xInit;
	double alpha = pow(10, -1);
	double epsilon = tol;
	double alpha0 = alphaInit;
	//double gamma = 0.8;
	//int iterMaxLS = 20;
	//int iter = 0;
	//int iterMax = 50;
	
	for (iter = 0; iter < iterMax; iter++) {
		//search direction
		VectorXd grad;
		MatrixXd hessian;
		double fx = objective->evalObjective(x, grad, hessian); //eval f, g, and H at x
		//x = A\b is MATLAB notation for solving the linear system Ax = b
		//Eigen's LDLT solver: x = A.ldlt().solve(b);
		VectorXd p = (-1*hessian).ldlt().solve(grad);

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
		x += deltax;
		double length = grad.norm();
		if (length < epsilon) {
			break;
		}

	}

	iterationsUsed = iter;
	return x;
}
