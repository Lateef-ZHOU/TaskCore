#include "DAQFamily/DAQChannelSpec.h"
#include "GMBFamily/GimbalModel.h"
inline constexpr DAQParams<UniformMapping<0.0, 6.0, 3>> policyParams{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 80,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .frequencyBinCount = 4,
};
static_assert(StationaryObject<Spectrum<policyParams>> && StationaryObject<RadialWind<policyParams>>);
static_assert(StationaryObject<VerticalWindProfileUnit<policyParams>>);
int main() { return 0; }
