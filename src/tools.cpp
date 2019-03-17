#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
	  VectorXd rmse(4);
	  rmse << 0, 0, 0, 0;

	  // check the validity of the following inputs:
	  //  * the estimation vector size should not be zero
	  //  * the estimation vector size should equal ground truth vector size
	  if (estimations.size() != ground_truth.size()
	      || estimations.size() == 0) {
	      std::cout << "ERROR: Invalid estimation or ground_truth data" << std::endl;
	      return rmse;
	  }

	  //accumulate squared residuals
	  for (unsigned int i = 0; i < estimations.size(); ++i){

	      VectorXd residual = estimations[i] - ground_truth[i];

	      //coefficient-wise multiplication
	      residual = residual.array()*residual.array();
	      rmse += residual;
	  }

	  //calculate the mean
	  rmse = rmse / estimations.size();

	  //calculate the squared root
	  rmse = rmse.array().sqrt();

	  //return the result
	  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
	  MatrixXd Hj(3, 4);
	  //recover state parameters

	  // position x,y
	  float px = x_state(0);
	  float py = x_state(1);
	  // velocity x,y
	  float vx = x_state(2);
	  float vy = x_state(3);


	  float pxSq = px*px;
	  float pySq = py*py;
	  float pxSqApySq = pxSq + pySq;
	  float sqrtpxSqApySq = sqrt(pxSqApySq);
	  float c3 = (pxSqApySq*sqrtpxSqApySq);

	  // division by zero
	  if (fabs(pxSqApySq) < 0.0001){
	      std::cout << "ERROR: Division by Zero" << std::endl;
	      return Hj;
	  }

	  //compute the Jacobian matrix
	  Hj << (px / sqrtpxSqApySq), (py / sqrtpxSqApySq), 0, 0,
	      -(py / pxSqApySq), (px / pxSqApySq), 0, 0,
	      py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / sqrtpxSqApySq, py / sqrtpxSqApySq;

	  return Hj;
}
