#include <iostream>
#include <eigen3/Eigen/Dense>
#include "include/ukf.h"

using Eigen::MatrixXd;

int main(int argc, char *argv[])
{

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */
  MatrixXd Xsig = MatrixXd(5, 11);
  ukf.GenerateSigmaPoints(&Xsig);

  // print result
  std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  MatrixXd Xsig_aug = MatrixXd(7, 15);
  ukf.AugmentedSigmaPoints(&Xsig_aug);

  // print result
  std::cout << "XsigAug = " << std::endl << Xsig_aug << std::endl;

  return 0;
}