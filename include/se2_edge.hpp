/*
 * se2_edge.hpp
 *
 *  Created on: Sep 30, 2011
 *      Author: yasir
 */

#ifndef SE2_EDGE_HPP_
#define SE2_EDGE_HPP_

#include <se2.hpp>
#include <Eigen/Core>
#include <iostream>
namespace Olson{
	using namespace Eigen;
	struct VERTEX_SE2;
	class EDGE_SE2 : public SE2
	{
	public:
		VERTEX_SE2* _to;
		Matrix3d _information;
		Matrix3d _covariance;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EDGE_SE2(): SE2(){}
		EDGE_SE2(double x, double y, double theta, const Matrix3d& information,VERTEX_SE2* to = NULL)
			:SE2(x,y,theta){
			_to = to;
			_information = information;
			_covariance = _information.inverse();
		}
		EDGE_SE2(VERTEX_SE2* to, const EDGE_SE2& tr2)
		{
			*this  = tr2;
			_to = to;
		}
		EDGE_SE2(VERTEX_SE2* to,const SE2& Transf, const Matrix3d& information)
					:SE2(Transf){
					_to = to;
					_information = information;
					_covariance = _information.inverse();
				}

		EDGE_SE2 operator * (const EDGE_SE2& tr2) const{
		  EDGE_SE2 result(*this);
		  result._t += _R*tr2._t;
		  result._R.angle()+= tr2._R.angle();
		  result._R.angle()=normalize_theta(result._R.angle());

		  Matrix3d J1, J2;

		  double ct1 = cos(_R.angle());
		  double st1 = sin(_R.angle());

		  double x2 = tr2.translation()[0];
		  double y2 = tr2.translation()[1];

		  J1<< 	  1., 0., -x2*st1-y2*ct1,
				  0., 1.,  x2*ct1-y2*st1,
				  0., 0.,  1.;

		  J2 <<   ct1,	-st1, 	0.,
				  st1,	 ct1,	0.,
				  0.,	  0.,   1.;

		  result._covariance = J1*_covariance*J1.transpose()+J2*tr2._covariance*J2.transpose();
		  result._information = result._covariance.inverse();
		  result._to = tr2._to;
		  return result;
		}
		 EDGE_SE2 inverse() const{
			  EDGE_SE2 ret;
			  ret._R=_R.inverse();
			  ret._R.angle()=normalize_theta(ret._R.angle());
			  ret._t=ret._R*(_t*-1.);

			  Matrix3d J;
			  double st = sin(_R.angle());
			  double ct = cos(_R.angle());
			  double x = _t(0);
			  double y = _t(1);

			  J << 	 -ct,	-st,	x*st-y*ct,
					  st,	-ct,	x*ct-y*st,
					  0., 	 0., 		  -1.;

			  ret._covariance = J*_covariance*J.transpose();
			  ret._information = ret._covariance.inverse();

		      return ret;
		}
		bool operator <(const EDGE_SE2& b) const
		{
			return _covariance.determinant()<b._covariance.determinant();
		}
		VERTEX_SE2* to()
		{
			return _to;
		}

	};

};

#endif /* SE2_EDGE_HPP_ */
