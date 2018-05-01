#ifndef BATTERY_H
#define BATTERY_H

#include "Simulator.hpp"

using namespace std;

class Battery : public Device
{
    public:
        Battery(int positive, int negative, double wh = 8.1, double soc0 = 1.0);
        
        void Init();
        void Step(double t, double dt);
        
        double GetSOC();
        double GetTerminalVoltage();
        double GetTerminalCurrent();
        
    // private:
        double updateSOC(const double dt);
        double Vin();
        double Rin();
        double Rt1();
        double Ct1();
        double Rt2();
        double Ct2();
        
        void setJacobianBlock(int i, int j, double g);
    
        int _pos;
        int _neg;
        int _nodep;//node 2 from model
        int _nodeq;//node 3 from model
        double _wh;
        double _soc;
        
    
};

Battery::Battery(int positive, int negative, double wh, double soc0)
    :_pos(positive), _neg(negative), _wh(wh), _soc(soc0)
{}

double Battery::GetSOC()
{
    return _soc;
}

double Battery::GetTerminalVoltage()
{
    return GetStateDifference(_pos, _neg);
}

double Battery::GetTerminalCurrent()
{
    return (Vin() - GetStateDifference(_nodep, _neg))/Rin();
}

double Battery::updateSOC(const double dt)
{
    _soc += (GetTerminalVoltage()*-1*GetTerminalCurrent()*dt)/(_wh*3600);
}

double Battery::Vin()
{
    return -1.031*exp(-35.0*_soc) + 3.685 + 0.2156*_soc 
        - 0.1178*pow(_soc, 2) + 0.3201*pow(_soc, 3);
}

double Battery::Rin()
{
    return 0.1562*exp(-24.37*_soc) + 0.07446;
}

double Battery::Rt1()
{
    return 0.3208*exp(-29.14*_soc) + 0.04669;
}

double Battery::Ct1()
{
    return -752.9*exp(-13.51*_soc) + 703.6;
}

double Battery::Rt2()
{
    return 6.603*exp(-155.2*_soc) + 0.04984;
}

double Battery::Ct2()
{
    return -6056.0*exp(-27.12*_soc) + 4475.0;
}

void Battery::setJacobianBlock(int i, int j, double g)
{
    AddJacobian(i, i, g);
    AddJacobian(i, j, -g);
    AddJacobian(j, i, -g);
    AddJacobian(j, j, g);
}

void Battery::Init()
{
    _nodep = GetNextNode();
    _nodeq = GetNextNode();
}
void Battery::Step(double t, double dt)
{
    
    double grin = 1/Rin();
    double gr1 = 1/Rt1();
    double gr2 = 1/Rt2();
    double gc1 = Ct1()/dt;
    double gc2 = Ct2()/dt;
    
    setJacobianBlock(_neg, _nodep, grin);
    setJacobianBlock(_nodep, _nodeq, gr1);
    setJacobianBlock(_nodep, _nodeq, gc1);
    setJacobianBlock(_nodeq, _pos, gr2);
    setJacobianBlock(_nodeq, _pos, gc2);
    
    AddBEquivalent(_neg, -Vin()/Rin());
    AddBEquivalent(_nodep, Vin()/Rin() - gc1*GetStateDifference(_nodeq, _nodep));
    AddBEquivalent(_nodeq, gc1*GetStateDifference(_nodeq, _nodep) - gc2*GetStateDifference(_pos, _nodeq));
    AddBEquivalent(_pos, gc2*GetStateDifference(_pos, _nodeq));
    updateSOC(dt);
}
#endif 