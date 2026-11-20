#include "DAQFamily/DAQChannelSpec.h"
#include "GMBFamily/GimbalModel.h"
using PolicyRange = UniformMapping<0.0, 6.0, 3>;
using PolicyFrequency = UniformMapping<0.0, 1.0, 4>;
static_assert(StationaryObject<Spectrum<PolicyRange, PolicyFrequency>> && StationaryObject<RadialWind<PolicyRange>>);
static_assert(StationaryObject<VerticalWindProfileUnit<PolicyRange>>);
int main() { return 0; }
