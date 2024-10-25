#include"plan.hpp"
#include<cmath>
#include<iostream>
#include <aris.hpp>


using namespace std;



auto cosCurve::getCurve(int count)->double
{
    int t = count + 1;
    double s = 0;
    s = a_ * std::cos(w_ * t / 1000.0 + p_);

    return s;
}





auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //三角形曲线  正弦波
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//计算梯形曲线的参数，由成员函数初始化，对应输入参数由构造函数初始化
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1)
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//安速度计算，此时给定的加速度不起作用
		this->Tc_ = 2.0 / v_;
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}



auto TCurve2::getCurveParam()->void
{
	this->ta_ = v_ / a_;
	this->pa_ = 0.5 * a_ * ta_ * ta_;
	this->tm_ = (p_ - 2 * pa_) / v_;
	this->tc_ = tm_ + 2 * ta_;
}



auto TCurve2::getTCurve(int count)->double
{
	int t = count + 1;
	double s = 0;

	if (t < tc_ * 1000 + 1)
	{
		if (tc_ - 2 * ta_ > 0)
		{
			if (t < ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * (t / 1000.0) * (t / 1000.0);
			}
			else if (ta_ * 1000 < t && t < tc_ * 1000 - ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * ta_ * ta_ + a_ * ta_ * (t / 1000.0 - ta_);
			}
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 1000.0) * (tc_ - t / 1000.0);
			}
		}

		else
		{
			ta_ = sqrt(p_ / a_);
			tc_ = 2 * ta_;

			if (t < ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * (t / 1000.0) * (t / 1000.0);
			}
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 1000.0) * (tc_ - t / 1000.0);
			}
		}


		return s;
	}
	else
	{
		return p_;
	}

}


