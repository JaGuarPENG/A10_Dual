
const double PI = 3.141592653589793;


class cosCurve
{
private:
    double a_;
    double w_;
    double p_;

public:
    auto getCurve(int count) -> double;  
    cosCurve(double a, double w, double p)
    {
        a_ = a;
        w_ = w;
        p_ = p;
    }  
    ~cosCurve() {} 
};



///功能：生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
//   ##参数定义##
//  Tc:生成曲线总共需要的时间，由输入的加速度和速度计算
//   v:速度，由用户输入，构造函数初始化
//   a:加速度，由用户输入，构造函数初始化
//  ta:加速段所需的时间，由输入的速度和加速度计算得到
class TCurve
{
private:
    double Tc_;
    double v_;
    double a_;
    double ta_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return Tc_; };
    TCurve(double a, double v) { a_ = a; v_ = v; }
    ~TCurve() {}
};


//T形曲线改，输入值为（最大加速度，最大速度，目标位置）
//tc_指运动到目标完成时间；
//tm_指以最大速度运动所需时间；
//v_为最大运动速度；
//a_为最大运动加速度；
//ta_指加速时间；
//p_指最终运动位置；
//pa_指加减速所需要运动的位置。
class TCurve2
{
private:
    double tc_;
    double tm_;
    double v_;
    double a_;
    double ta_;
    double p_;
    double pa_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return tc_; };
    TCurve2(double a, double v, double p) { a_ = a; v_ = v; p_ = p; }
    ~TCurve2() {}
};







