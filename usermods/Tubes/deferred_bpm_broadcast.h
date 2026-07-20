#pragma once

#include <stdint.h>

// Holds the latest local BPM change until that clock crosses a phrase boundary.
// Scheduling again within the phrase replaces the value without adding another send.
class DeferredBpmBroadcast {
  public:
    void schedule(uint16_t bpm, uint32_t beatFrame) {
        pendingBpm = bpm;
        scheduledPhrase = phraseOf(beatFrame);
        pending = true;
    }

    bool active() const {
        return pending;
    }

    void cancel() {
        pending = false;
    }

    bool takeAtPhraseBoundary(uint32_t beatFrame, uint16_t& bpm) {
        if (!pending || phraseOf(beatFrame) == scheduledPhrase)
            return false;

        bpm = pendingBpm;
        pending = false;
        return true;
    }

  private:
    static uint32_t phraseOf(uint32_t beatFrame) {
        return beatFrame >> 12;
    }

    bool pending = false;
    uint16_t pendingBpm = 0;
    uint32_t scheduledPhrase = 0;
};
