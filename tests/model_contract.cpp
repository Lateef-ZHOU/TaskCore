#include "DAQFamily/DAQChannelSpec.h"
#include "GMBFamily/GimbalModel.h"
using PolicyRange = UniformMapping<0.0, 6.0, 3>;
using PolicyFrequency = DopplerMapping<UniformMapping<0.0, 1.0, 4>, 1.55e-6, 80.0>;
static_assert(StationaryObject<Spectrum<PolicyRange, PolicyFrequency>> && StationaryObject<RadialWind<PolicyRange>>);
static_assert(StationaryObject<VerticalWindProfileUnit<PolicyRange>>);
int main() { return 0; }
