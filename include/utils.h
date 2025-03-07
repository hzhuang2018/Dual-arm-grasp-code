/*
 * @Author: yipeng_li 
 * @Date: 2018-03-23 09:32:50 
 * @Last Modified by:   yipeng_li 
 * @Last Modified time: 2018-03-23 09:32:50 
 */
#ifndef UTILS_H
#define UTILS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

namespace Robot
{
    // define rotation matrix 
    typedef Eigen::Matrix<double, 3, 3> Rot;
    // define translation vector
    typedef Eigen::Matrix<double, 3, 1> Vec;
    // define 4x4 tranformation matrix
    typedef Eigen::Matrix<double, 4, 4> Trans;
    

    // cv::Mat to Eigen
    inline Rot Mat2Rot(const cv::Mat& R_)
    {
        if (R_.cols !=3 || R_.rows != 3)
        {
            std::cout << "Error occurs when trans Mat to Rot!" << std::endl;
            exit(1);
        }
        Rot R;
        cv::cv2eigen(R_, R);
        return R;
    }
    //  Eigen to cv::Mat
    inline cv::Mat Rot2Mat(const Rot& R_)
    {
        cv::Mat R(3, 3, CV_64FC1);
        cv::eigen2cv(R_, R);
        return R;
    }
    // cv::Mat to Vec
    inline Vec Mat2Vec(const cv::Mat& t_)
    {
        if (t_.cols !=1 || t_.rows != 3)
        {
            std::cout << "Error occurs when trans Mat to Rot!" << std::endl;
            exit(1);
        }
        Vec t;
        cv::cv2eigen(t_, t);
        return t;
    }
    // define skew funtion
    inline Rot skew(const Vec& t)
    {
        Rot skew_x = Eigen::MatrixXd::Zero(3,3);
        skew_x(0,1) = -t(2,0);
        skew_x(0,2) = t(1,0);
        skew_x(1,0) = t(2,0);
        skew_x(1,2) = -t(0,0);
        skew_x(2,0) = -t(1,0);
        skew_x(2,1) = t(0,0);

        return skew_x;
    }
	inline Rot vec2rot(double x_, double y_, double z_)
	{
		cv::Mat theta(3, 1, CV_64FC1);
		cv::Mat Rc(3, 3, CV_64FC1);
		theta.at<double>(0, 0) = x_;
		theta.at<double>(1, 0) = y_;
		theta.at<double>(2, 0) = z_;

		cv::Rodrigues(theta, Rc);
		return Mat2Rot(Rc);
	}
	/*inline Vec rot2vec(Eigen::Matrix3f rot_)
	{
		cv::Mat theta(3, 1, CV_32FC1);
		cv::Mat Rc;
		Rc = Rot2Mat(rot_);

		Vec ret;
		cv::Rodrigues(Rc, theta);
		ret << theta.at<float>(0, 0), theta.at<float>(1, 0), theta.at<float>(2, 0);
		return ret;
	}*/
	inline Vec rot2vec(const Eigen::Matrix3d& rot_)
	{
		Rot rot;
		rot(0, 0) = rot_(0, 0); rot(0, 1) = rot_(0, 1); rot(0, 2) = rot_(0, 2);
		rot(1, 0) = rot_(1, 0); rot(1, 1) = rot_(1, 1); rot(1, 2) = rot_(1, 2);
		rot(2, 0) = rot_(2, 0); rot(2, 1) = rot_(2, 1); rot(2, 2) = rot_(2, 2);
		cv::Mat theta(3, 1, CV_64FC1);
		cv::Mat Rc;
		Rc = Rot2Mat(rot);

		Vec ret;
		cv::Rodrigues(Rc, theta);
		ret << theta.at<double>(0, 0), theta.at<double>(1, 0), theta.at<double>(2, 0);
		return ret;
	}


	inline double sign_fun(double x)
	{
		if (x <= 0)
			return -1.0;
		else
			return 1.0;
	}

    // define class of quaternion 
    class Quat
    {
    public:
        Quat()
        {
            q(0,0) = 0.0;
            q(1,0) = 0.0;
            q(2,0) = 0.0;
            q(3,0) = 1.0;
        }
        Quat(double x, double y, double z, double w)
        {
            setValue(x,y,z,w);
        }
        Quat(const Eigen::Matrix<double, 4, 1>& q_)
        {
            q = q_;
            normalize();
        }
        ~Quat(){}


        inline void setValue(double x, double y, double z, double w)
        {
            q(0,0) = x;
            q(1,0) = y;
            q(2,0) = z;
            q(3,0) = w;
            normalize();
        }

        inline Quat& operator= (const Quat& other_)
        {
            q = other_.q;
            return *this;
        }
        inline Quat& inverse()
        {
            q = q_inv();
            return *this;
        }
        // define q_x
        //inline Quat operator* (const Quat)
        
        inline Rot R()
        {
            Rot R_;
            R_ = (2*eta()*eta()-1)*Eigen::MatrixXd::Identity(3,3) + 2*eta()*skew(epsilon()) + 2*epsilon()*epsilon().transpose();
            return R_;
        }
        inline void normalize()
        {
            q.normalize();
        } 
        // quaternion operation
        inline Eigen::Matrix<double, 4, 4> q_x()
        {
            Eigen::Matrix<double, 4, 4> qx;
            qx = Eigen::MatrixXd::Zero(4,4);
            qx.block<3,3>(0,0) = eta()*Eigen::MatrixXd::Identity(3,3) - skew(epsilon());
            qx.block<3,1>(0,3) = epsilon();
            qx.block<1,3>(3,0) = -epsilon().transpose();
            qx(3,3) = eta();

            return qx;
        }
        inline Eigen::Matrix<double, 4, 4> q_ox()
        {
            Eigen::Matrix<double, 4, 4> qx;
            qx = Eigen::MatrixXd::Zero(4,4);
            qx.block<3,3>(0,0) = eta()*Eigen::MatrixXd::Identity(3,3) + skew(epsilon());
            qx.block<3,1>(0,3) = epsilon();
            qx.block<1,3>(3,0) = -epsilon().transpose();
            qx(3,3) = eta();

            return qx;
        }
    private:
        Eigen::Matrix<double, 4, 1> q; // q = [\epsilon \eta]

        Eigen::Matrix<double, 3, 1> epsilon() {return q.block<3,1>(0,0);}
		double eta() {return q(3,0);}

        inline Eigen::Matrix<double, 4, 1> q_inv()
        {
            Eigen::Matrix<double, 4, 1> qinv;
            qinv.block<3,1>(0,0) = -q.block<3,1>(0,0);
            return qinv;
        }
    };
} // end of Robot

#endif