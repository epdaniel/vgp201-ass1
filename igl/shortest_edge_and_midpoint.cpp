// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2016 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "shortest_edge_and_midpoint.h"
#include "circulation.h"
#include <iostream>
#include <Eigen/LU>

Eigen::MatrixXd face_normals_dec;

IGL_INLINE void igl::shortest_edge_and_midpoint(
  const int e,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & /*F*/,
  const Eigen::MatrixXi & E,
  const Eigen::VectorXi & /*EMAP*/,
  const Eigen::MatrixXi & /*EF*/,
  const Eigen::MatrixXi & /*EI*/,
  double & cost,
  Eigen::RowVectorXd & p)
{
  cost = (V.row(E(e,0))-V.row(E(e,1))).norm();
  p = 0.5*(V.row(E(e,0))+V.row(E(e,1)));
}

IGL_INLINE void igl::edgeErrorAndOptimalPlacement(
	const int e,
	const Eigen::MatrixXd & V,
	const Eigen::MatrixXi & F,
	const Eigen::MatrixXi & E,
	const Eigen::VectorXi & EMAP,
	const Eigen::MatrixXi & EF,
	const Eigen::MatrixXi & EI,
	double & cost,
	Eigen::RowVectorXd & p)
{
	int v1 = E(e, 0);
	int v2 = E(e, 1);
	Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();

	std::vector<int> N = circulation(e, true, EMAP, EF, EI);
	std::vector<int> Nd = circulation(e, false, EMAP, EF, EI);
	N.insert(N.begin(), Nd.begin(), Nd.end());

	for (auto i : N)
	{
		Eigen::Vector3d normal = face_normals_dec.row(i).normalized();
		//std::cout << "face " << i << " normal: " << normal << std::endl;
		double d = -V.row(F.row(i)[0]) * normal;
		Eigen::Vector4d p = Eigen::Vector4d(normal[0], normal[1], normal[2], d).transpose();
		Eigen::Matrix4d Kp = p * (p.transpose());
		Q += Kp;
	}
	Eigen::Matrix4d Qtag = Q;
	Qtag(3, 0) = 0;
	Qtag(3, 1) = 0;
	Qtag(3, 2) = 0;
	Qtag(3, 3) = 1;

	Eigen::Vector4d vtag = Qtag.inverse() * Eigen::Vector4d(0, 0, 0, 1); 
	p = Eigen::Vector3d(vtag[0], vtag[1], vtag[2]);
	//std::cout << "p: " << p << std::endl;
	//p = 0.5*(V.row(v1) + V.row(v2)); 

	cost = vtag.transpose() *  Q * vtag;
	std::cout << "edge " << e << ", cost = " << cost << ", new v position (" << p << ")" << std::endl;
}
