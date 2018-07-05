#pragma once
#include <Eigen/Eigen>

class CCamera
{
public:
	CCamera();
	~CCamera();

public:
	CCamera() : K(Eigen::Matrix3d::Identity(3, 3)), R(Eigen::Matrix3d::Identity(3, 3)), t(Eigen::Vector3d(0, 0, 0)), P(Eigen::Matrix3Xd::Identity(3, 4)) {}
	CCamera(Eigen::Matrix3d _K) : K(_K), R(Eigen::Matrix3d::Identity(3, 3)), t(Eigen::Vector3d(0, 0, 0)), P(Eigen::Matrix3Xd::Identity(3, 4)) {}

	Eigen::Vector3d getC() { return C; }
	Eigen::Matrix3Xd getP() { return P; }
	Eigen::Matrix3d getKinv() { return K_inv; }
	Eigen::Matrix3d getM() { return M; }
	Eigen::Matrix3d getMinv() { return M_inv; }

	void set() {
		setC();
		setP();
		setKinv();
		setMMinv();
	}

	Eigen::Matrix3d R;
	Eigen::Vector3d t;
private:
	Eigen::Matrix3d K;

	Eigen::Matrix3d K_inv;
	Eigen::Vector3d C;
	Eigen::Matrix3Xd P;

	Eigen::Matrix3d M;
	Eigen::Matrix3d M_inv;

	void setC() { C = -R*t; }
	void setP() {
		for (int x = 0; x < 3; x++) {
			for (int y = 0; y < 3; y++) {
				P(x, y) = R(x, y);
			}
			P(x, 3) = t(x);
		}

		P = K * P;
	}
	void setKinv() { K_inv = K.inverse(); }
	void setMMinv() {
		M = P.block(0, 0, 3, 3);
		M_inv = M.inverse();
	}
};

