#define _GLIBCXX_USE_CXX11_ABI 0
#ifndef GLOBALPLACER_H
#define GLOBALPLACER_H

#include "Placement.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>

class GlobalPlacer
{
public:
    GlobalPlacer(Placement &placement);
    double netWL(Module &m);
    void place();
    void plotPlacementResult(const string outfilename, bool isPrompt = false);

private:
    Placement &_placement;
    void plotBoxPLT(ofstream &stream, double x1, double y1, double x2, double y2);
};

#endif // GLOBALPLACER_H
