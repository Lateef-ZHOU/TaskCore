#include "DAQFamily/DAQChannelSpec.h"
#include "GMBFamily/GimbalModel.h"
struct Spec {
    static constexpr const char *name = "policy";
    static constexpr int rangeGateCount = 3, frequencyBinCount = 4;
    static constexpr double rangeStartM = 0, rangeStepM = 6;
    static constexpr double frequencyStartMHz = 0, frequencyStepMHz = 1;
    static constexpr double wavelengthM = 1.55e-6, zeroWindMHz = 80;
};
static_assert(ValueObject<Spec>);
static_assert(StationaryObject<Spectrum<Spec>> && StationaryObject<RadialWind<Spec>>);
static_assert(StationaryObject<VerticalWindProfileUnit<Spec>>);

int main() { return 0; }
