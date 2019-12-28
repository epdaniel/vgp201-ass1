#include "Movable.h"
#include <iostream>
using namespace std;
Movable::Movable()
{
	T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}


Eigen::Matrix4f Movable::MakeTrans()
{
	return (Tout * Tin).matrix();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.pretranslate(amt);
}

//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle, bool yAxis)
{	
	Eigen::Matrix3f mat = Tout.rotation().matrix();
	mat.transposeInPlace();
	if(yAxis)
		Tout.rotate(Eigen::AngleAxisf(angle, mat * rotAxis));
	else
		Tout.rotate(Eigen::AngleAxisf(angle, rotAxis));
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	Tout.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt)
{
	Tin.translate(-amt);
	Tout.translate(amt);
}
