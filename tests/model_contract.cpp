#include "DAQFamily/DAQChannelSpec.h"
#include "GMBFamily/GimbalModel.h"
inline constexpr DAQParams policyParams{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 80,
    .rangeStartM = 0, .rangeStepM = 6,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .rangeGateCount = 3, .frequencyBinCount = 4,
};
static_assert(StationaryObject<Spectrum<policyParams>> && StationaryObject<RadialWind<policyParams>>);
static_assert(StationaryObject<VerticalWindProfileUnit<policyParams>>);
int main() { return 0; }
