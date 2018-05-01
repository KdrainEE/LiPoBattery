#include "battery.hpp"
#include "Resistor.hpp"
#include "Plotter.hpp"

int main(){
    //dummy test that does nothing right now
    Resistor rload(1,0, 10.0);
    Battery myBat(1,0);
    int nNodes = 1;
    
    Simulator sim(nNodes);
    
    sim.Add(myBat);
    sim.Add(rload);
    
    
    
    double tf = 500;
    sim.Init(0.1, tf);
    
    
    Plotter voltageplot("Battery Voltage");
    Plotter currentplot("Battery current");
    Plotter soc("State of Charge");
    Plotter resistances("Equivalent Resistances");
    Plotter capacitances("Equivalent Capacitances");
    resistances.SetLabels("Rin","Rt1", "Rt2");
    soc.SetLabels("SOC");
    capacitances.SetLabels("Ct1", "Ct2");
    voltageplot.SetLabels("Terminal Voltage (V)");
    currentplot.SetLabels("Terminal Current (A)");
    while(sim.IsRunning() && myBat.GetSOC() > 0.0)
    {
        
        voltageplot.AddRow(sim.GetTime(), myBat.GetTerminalVoltage());
        currentplot.AddRow(sim.GetTime(), myBat.GetTerminalCurrent());
        soc.AddRow(sim.GetTime(), myBat.GetSOC());
        resistances.AddRow(sim.GetTime(), myBat.Rin(), myBat.Rt1(), myBat.Rt2());
        capacitances.AddRow(sim.GetTime(), myBat.Ct1(), myBat.Ct2());
        sim.Step();
    }
    if(myBat.GetSOC() < 0.0)
    {
        voltageplot.AddRow(sim.GetTime(), 0.0);
    }
    voltageplot.Plot();
    currentplot.Plot();
    soc.Plot();
    resistances.Plot();
    capacitances.Plot();
    return 0;
}