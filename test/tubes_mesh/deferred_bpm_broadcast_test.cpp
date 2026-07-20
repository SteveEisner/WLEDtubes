#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "deferred_bpm_broadcast.h"

namespace {

constexpr uint32_t PHRASE_FIVE = 5UL << 12;
constexpr uint32_t PHRASE_SIX = 6UL << 12;

void expect(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

// A scheduled BPM remains pending throughout its starting phrase, then becomes
// available exactly once after the local beat frame enters the next phrase.
void bpm_is_taken_after_phrase_boundary() {
    DeferredBpmBroadcast deferred;
    uint16_t emittedBpm = 0;

    deferred.schedule(120U << 8, PHRASE_FIVE + 100);
    bool emittedBeforeBoundary = deferred.takeAtPhraseBoundary(PHRASE_SIX - 1, emittedBpm);
    bool emittedAtBoundary = deferred.takeAtPhraseBoundary(PHRASE_SIX, emittedBpm);
    bool emittedTwice = deferred.takeAtPhraseBoundary(PHRASE_SIX + 1, emittedBpm);

    expect(!emittedBeforeBoundary, "BPM became available inside its starting phrase");
    expect(emittedAtBoundary, "BPM did not become available at the phrase boundary");
    expect(emittedBpm == (120U << 8), "phrase boundary returned the wrong BPM");
    expect(!emittedTwice, "one scheduled BPM was emitted more than once");
    expect(!deferred.active(), "emitted BPM remained pending");
}

// Multiple local changes in one phrase collapse into one boundary send carrying the
// most recent BPM, so obsolete intermediate values never enter the mesh.
void latest_bpm_replaces_pending_value() {
    DeferredBpmBroadcast deferred;
    uint16_t emittedBpm = 0;

    deferred.schedule(120U << 8, PHRASE_FIVE + 100);
    deferred.schedule(125U << 8, PHRASE_FIVE + 200);
    bool emitted = deferred.takeAtPhraseBoundary(PHRASE_SIX, emittedBpm);

    expect(emitted, "replacement BPM did not become available at the boundary");
    expect(emittedBpm == (125U << 8), "obsolete BPM was emitted instead of the latest value");
}

// An authoritative declaration cancels a pending local change, preventing that stale
// value from overriding the declared BPM at a later phrase boundary.
void cancellation_discards_pending_bpm() {
    DeferredBpmBroadcast deferred;
    uint16_t emittedBpm = 0;
    deferred.schedule(125U << 8, PHRASE_FIVE + 100);

    deferred.cancel();
    bool emitted = deferred.takeAtPhraseBoundary(PHRASE_SIX, emittedBpm);

    expect(!emitted, "cancelled BPM was emitted at the phrase boundary");
    expect(!deferred.active(), "cancelled BPM remained pending");
}

} // namespace

int main() {
    const std::array<std::pair<const char*, void (*)()>, 3> tests = {{
        {"BPM is taken after phrase boundary", bpm_is_taken_after_phrase_boundary},
        {"latest BPM replaces pending value", latest_bpm_replaces_pending_value},
        {"cancellation discards pending BPM", cancellation_discards_pending_bpm},
    }};

    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "PASS: " << test.first << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
            return 1;
        }
    }
    return 0;
}
