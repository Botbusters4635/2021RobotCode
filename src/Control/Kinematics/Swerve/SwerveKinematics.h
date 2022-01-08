//
// Created by alberto on 15/08/19.
//

#ifndef ECTOCONTROL_SWERVEKINEMATICS_H
#define ECTOCONTROL_SWERVEKINEMATICS_H

#include "Control/Kinematics/Swerve/SwerveState.h"
#include "Math/DataTypes/Twist2D.h"
#include <cmath>
#include <chrono>
#include <memory>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/SVD>

class SwerveKinematics {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	SwerveKinematics(double length, double width);
	
	SwerveState calculateInverseKinematics(const Twist2D &targetVelocity, const SwerveState &currentState,
	                                       const Point2D &centerOfRotation);
	
	SwerveState calculateInverseKinematics(const Twist2D &targetVelocity, const SwerveState &currentState);
	
	SwerveState OLDcalculateInverseKinematics(const Twist2D &targetVelocity, const SwerveState &currentState);
	
	//Returns delta in x, y, theta
	Twist2D calculateForwardKinematics(const SwerveState &currentValues) const;

private:
	void calculateWheelValue(double x, double y, SwerveWheel &wheel, const SwerveWheel &lastValue) const;
	
	Point2D previousCenterOfRotation;
	
	Eigen::Matrix<double, 8, 3> inverseKinematics;
	Eigen::MatrixXd forwardKinematics;
	
	double length, width;
	
	double radius;
	
	double lengthComponent, widthComponent;
	
	SwerveState lastState, lastSet;
	
	std::chrono::high_resolution_clock::time_point lastRunTime;
	
	//Magic
	template<class MatT>
	static Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
	pseudoinverse(const MatT &mat,
	              typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
	{
		typedef typename MatT::Scalar Scalar;
		auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
		const auto &singularValues = svd.singularValues();
		Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(),
		                                                                                          mat.rows());
		singularValuesInv.setZero();
		for (int i = 0; i < singularValues.size(); ++i) {
			if (singularValues(i) > tolerance) {
				singularValuesInv(i, i) = Scalar{1} / singularValues(i);
			} else {
				singularValuesInv(i, i) = Scalar{0};
			}
		}
		return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
	}
};

#endif //ECTOCONTROL_SWERVEKINEMATICS_H
