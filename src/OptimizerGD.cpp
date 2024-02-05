#include "OptimizerGD.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerGD::OptimizerGD()
{
	
}

OptimizerGD::~OptimizerGD()
{
	
}

VectorXd OptimizerGD::optimize(const shared_ptr<Objective> objective, const VectorXd& xInit) {
	
	if (useLineSearch) {
		return optimizewithLS(objective, xInit);
	}
	else {
		VectorXd x = xInit;
		double alpha = pow(10, -1);
		double epsilon = tol;/*pow(10, -6);
		int iter = 0;
		int iterMax = 50;*/
		iter = 0;
		for (iter; iter < iterMax; iter++) {
			VectorXd grad;
			double fx = objective->evalObjective(x, grad); //eval f and g at x
			VectorXd p = -grad;

			VectorXd deltax = alpha * p;
			x += deltax;
			double length = grad.norm();
			if (length < epsilon) {
				break;
			}

		}
		iterationsUsed = iter;
		return x;
	}
	
}

VectorXd OptimizerGD::optimizewithLS(const std::shared_ptr<Objective> objective, const Eigen::VectorXd& xInit) {
	VectorXd x = xInit;
	double alpha = pow(10, -1);
	double epsilon = tol;//pow(10,-6)
	double alpha0 = alphaInit;
	//double gamma = 0.8;//0.8;//0.5;//
	//int interMaxLS = 20;
	//int iter = 0;
	//int iterMax = 50;//50;//5;//
	int n = xInit.size();

	for (iter = 0; iter < iterMax; iter++) {
		VectorXd grad;
		double fx = objective->evalObjective(x, grad); //eval f and g at x
		VectorXd p = -grad;

		/*double e = pow(10, -7);
		VectorXd g_(n); 
		g_.setZero();
		for (int i = 0; i < n; i++) {
			VectorXd theta_ = x;
			theta_(i) += e;
			double f_ = objective->evalObjective(theta_);
			g_(i) = (f_ - fx) / e;
		}
		cout << "g: " << grad << "\n g_: " << g_ << endl << endl;*/

		VectorXd deltax;
		alpha = alpha0;
		for (iterLS = 0; iterLS < iterMaxLS; iterLS++) {
			deltax = alpha * p;
			//VectorXd newx = x + deltax;
			double f1 = objective->evalObjective(x + deltax);
			if (f1 < fx) {
				break;
			}
			alpha *= gamma;
		}


		x += deltax;
		double length = grad.norm();
		if (length < epsilon) {
			break;
		}

	}
	
	iterationsUsed = iter;
	return x;
}

/*VectorXd OptimizerGD::optimize(const shared_ptr<Objective> objective, const VectorXd& xInit)
{
	int iterMax = 50;
	VectorXd x = xInit;
	double alpha = pow(10, -1);
	double epsilon = pow(10, -6);
	int iter = 0;
	double alpha0 = 1.0f;
	double gamma = 0.8;
	int interMaxLS = 20;

	Matrix2d A = Matrix2d::Identity();
	Vector2d x0(0, 0), g0(0,0);

	for (iter; iter < iterMax; iter++) {
		//search direction
		VectorXd grad;
		double fx = objective->evalObjective(x, grad); //eval f and g at x
		if (iter > 1) {
			MatrixXd s = x - x0;
			MatrixXd y = grad - g0;
			double rho = 1 / (y.transpose() * s)(0,0);
			A = (Matrix2d::Identity() - rho * (s * y.transpose())) * A * (Matrix2d::Identity() - rho * (y * s.transpose())) + rho * (s * s.transpose());
		}

		//MatrixXd hessian; 
		//double fx = objective->evalObjective(x, grad, hessian); //eval f, g, and H at x
		//VectorXd p = -grad;
		//x = A\b is MATLAB notation for solving the linear system Ax = b
		//Eigen's LDLT solver: x = A.ldlt().solve(b);
		//VectorXd p = (-1*hessian).ldlt().solve(grad);
		VectorXd p = -A * grad; //2x1 vector
		VectorXd deltax;
		
		//line search
		alpha = alpha0;
		for (int interLS = 0; interLS < interMaxLS; interLS++) {
			deltax = alpha * p;
			//VectorXd newx = x + deltax;
			double f1 = objective->evalObjective(x + deltax);
			if (f1 < fx) {
				break;
			}
			alpha *= gamma;
		}

		//step
		//deltax = alpha * p;
		x0 = x;
		g0 = grad;
		x += deltax;
		double length = grad.norm();
		if (length < epsilon) {
			break;
		}
	}
	iterationsUsed = iter;
	return x;//xInit;
}

*/
