#pragma once

#include <Eigen/Dense>

Eigen::Matrix<double, 4, 1> rot2quat(const Eigen::Matrix3d R_)
{
	double d11 = R_(0, 0);
	double d21 = R_(1, 0);
	double d31 = R_(2, 0);
	double d12 = R_(0, 1);
	double d22 = R_(1, 1);
	double d32 = R_(2, 1);
	double d13 = R_(0, 2);
	double d23 = R_(1, 2);
	double d33 = R_(2, 2);
	Eigen::Matrix<double, 4, 4> K3;
	K3.setZero();
	K3(0, 0) = d11 - d22 - d33;
	K3(0, 1) = d21 + d12;
	K3(0, 2) = d31 + d13;
	K3(0, 3) = d23 - d32;

	K3(1, 0) = K3(0, 1);
	K3(1, 1) = d22 - d11 - d33;
	K3(1, 2) = d32 + d23;
	K3(1, 3) = d31 - d13;

	K3(2, 0) = K3(0, 2);
	K3(2, 1) = K3(1, 2);
	K3(2, 2) = d33 - d11 - d22;
	K3(2, 3) = d12 - d21;

	K3(3, 0) = K3(0, 3);
	K3(3, 1) = K3(1, 3);
	K3(3, 2) = K3(2, 3);
	K3(3, 3) = d11 + d22 + d33;

	K3 = K3 / 3;

	Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> es(K3);
	int index = 0;
	double max_eig = es.pseudoEigenvalueMatrix()(0, 0);
	for (int i = 1; i < 4; i++)
	{
		if (es.pseudoEigenvalueMatrix()(i, i) > max_eig)
		{
			index = i;
			max_eig = es.pseudoEigenvalueMatrix()(i, i);
		}
	}

	Eigen::Matrix<double, 4, 1> quat = es.pseudoEigenvectors().col(index);
	double w = quat(3);
	double x = quat(0);
	double y = quat(1);
	double z = quat(2);
	if (w < 0)
		quat << -x, -y, -z, -w;
	else
		quat << x, y, z, w;
	return quat;
}
Eigen::Matrix3d skew_matrix(Eigen::Vector3d x)
{
	Eigen::Matrix3d skew_x;
	skew_x.setZero();
	skew_x(0, 1) = -x(2);
	skew_x(0, 2) = x(1);

	skew_x(1, 0) = x(2);
	skew_x(1, 2) = -x(0);

	skew_x(2, 0) = -x(1);
	skew_x(2, 1) = x(0);

	return skew_x;
}