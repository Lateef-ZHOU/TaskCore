#include "DAQFamily/DAQFrames.h"
#include "GMBFamily/GimbalModel.h"
using PolicyRange = LinearMapping<0.0, 6.0, 3>;
using PolicyFrequency = LinearMapping<0.0, 1.0, 4>;
static_assert(StationaryObject<Grid2D<PolicyRange, PolicyFrequency>> && StationaryObject<Array1D<PolicyRange>>);
static_assert(StationaryObject<WindProfileFrame<PolicyRange>>);
int main() { return 0; }
