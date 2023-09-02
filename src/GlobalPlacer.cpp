#include "GlobalPlacer.h"
#include "ExampleFunction.h"
#include "NumericalOptimizer.h"
#include <cmath>

GlobalPlacer::GlobalPlacer(Placement &placement)
    : _placement(placement)
{
}

double GlobalPlacer::netWL(Module &m)
{
    double HPWL = 0;
    for (int i = 0; i < m.numPins(); i++)
    {
        // initialize the max and min value
        double maxX = -999999999;
        double maxY = -999999999;
        double minX = 999999999;
        double minY = 999999999;

        // get the net max and min value
        Net &n = _placement.net(m.pin(i).netId());
        for (int i = 0; i < n.numPins(); i++)
        {
            maxX = max(maxX, n.pin(i).x());
            maxY = max(maxY, n.pin(i).y());
            minX = min(minX, n.pin(i).x());
            minY = min(minY, n.pin(i).y());
        }

        // accumulate the HPWL
        HPWL += (maxX - minX) + (maxY - minY);
    }
    return HPWL;
}

void GlobalPlacer::place()
{
    // set the random seed
    srand(777);

    // get the initial solution in random
    for (int i = 0; i < _placement.numModules(); i++)
    {
        double rx = _placement.boundryLeft() + ((double)rand() / (RAND_MAX + 1.0)) * (_placement.boundryRight() - _placement.boundryLeft());   // random coorinate x
        double ry = _placement.boundryBottom() + ((double)rand() / (RAND_MAX + 1.0)) * (_placement.boundryTop() - _placement.boundryBottom()); // random coordinate y
        _placement.module(i).setPosition(rx, ry);                                                                                              // set the random position
    }

    // Simulated Annealing parameters
    double T = 1000000000000000000;                                                                                                      // temperature
    const double cool = 0.0000000000000000000000000000000000000000000000000000000000000000000000000001;                                  // the stop temperature
    const double fastT = 0.0001;                                                                                                         // fast bound
    const double refineT = 0.0000001;                                                                                                    // refinement bound
    double gamma = 0.99989;                                                                                                              // cooling rate
    const double gamma_refine = 0.99995;                                                                                                 // refinement cooling rate
    const double init = (_placement.boundryRight() - _placement.boundryLeft()) + (_placement.boundryTop() - _placement.boundryBottom()); // outline HPWL (for normalization)
    double curCost, nextCost;                                                                                                            // cost of each operation
    double delta, p, rng;                                                                                                                // the check parameter of accept or not
    int s1, s2;                                                                                                                          // module index
    double s1x, s1y, s2x, s2y;                                                                                                           // coordinate of module
    // int counter = 0;

    // Do the Simulated Annealing
    while (T >= cool)
    {
        // cout << "****iteration: " << counter++ << "****" << endl;

        // Get the to be swapped module by random picking
        s1 = rand() % _placement.numModules();
        s2 = rand() % _placement.numModules();
        // cout << "s1:" << s1 << " s2:" << s2 << endl;

        Module &m1 = _placement.module(s1);
        Module &m2 = _placement.module(s2);
        s1x = m1.x();
        s1y = m1.y();
        s2x = m2.x();
        s2y = m2.y();

        // current cost
        curCost = (netWL(m1) + netWL(m2)) / init;
        // cout << "curCost: " << curCost << endl;

        // swap the module
        m1.setPosition(s2x, s2y);
        m2.setPosition(s1x, s1y);

        // next cost
        nextCost = (netWL(m1) + netWL(m2)) / init;
        // cout << "nextCost:" << nextCost << endl;

        // check accept or not
        delta = nextCost - curCost;
        p = exp(-delta / T);
        rng = (double)rand() / (RAND_MAX + 1.0);

        if (delta <= 0) // directly accept (cost is lower)
        {
            // cout << "Accepth with curCost: " << curCost << " nextCost: " << nextCost << endl;
        }
        else if (p >= rng && T < fastT) // accept with probabiltiy (cost is higher but accept)
        {
            // cout << "Accepth with curCost: " << curCost << " nextCost: " << nextCost << endl;
            // cout << "With P: " << p << " RNG: " << rng << endl;
        }
        else // reject (cost is higher and reject)
        {
            m1.setPosition(s1x, s1y);
            m2.setPosition(s2x, s2y);
            // cout << "Reject with curCost: " << curCost << " nextCost: " << nextCost << endl;
        }

        // cooling
        T *= gamma;

        // slow down the cooling rate to do the refinement
        if (T < refineT)
        {
            gamma = gamma_refine;
        }
    }
}

void GlobalPlacer::plotPlacementResult(const string outfilename, bool isPrompt)
{
    ofstream outfile(outfilename.c_str(), ios::out);
    outfile << " " << endl;
    outfile << "set title \"wirelength = " << _placement.computeHpwl() << "\"" << endl;
    outfile << "set size ratio 1" << endl;
    outfile << "set nokey" << endl
            << endl;
    outfile << "plot[:][:] '-' w l lt 3 lw 2, '-' w l lt 1" << endl
            << endl;
    outfile << "# bounding box" << endl;
    plotBoxPLT(outfile, _placement.boundryLeft(), _placement.boundryBottom(), _placement.boundryRight(), _placement.boundryTop());
    outfile << "EOF" << endl;
    outfile << "# modules" << endl
            << "0.00, 0.00" << endl
            << endl;
    for (size_t i = 0; i < _placement.numModules(); ++i)
    {
        Module &module = _placement.module(i);
        plotBoxPLT(outfile, module.x(), module.y(), module.x() + module.width(), module.y() + module.height());
    }
    outfile << "EOF" << endl;
    outfile << "pause -1 'Press any key to close.'" << endl;
    outfile.close();

    if (isPrompt)
    {
        char cmd[200];
        sprintf(cmd, "gnuplot %s", outfilename.c_str());
        if (!system(cmd))
        {
            cout << "Fail to execute: \"" << cmd << "\"." << endl;
        }
    }
}

void GlobalPlacer::plotBoxPLT(ofstream &stream, double x1, double y1, double x2, double y2)
{
    stream << x1 << ", " << y1 << endl
           << x2 << ", " << y1 << endl
           << x2 << ", " << y2 << endl
           << x1 << ", " << y2 << endl
           << x1 << ", " << y1 << endl
           << endl;
}
