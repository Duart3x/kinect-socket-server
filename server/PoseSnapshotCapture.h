// Licensed under the MIT License.

#pragma once

#include <k4abt.h>
#include <chrono>
#include <string>

class PoseSnapshotCapture
{
public:
    PoseSnapshotCapture(std::chrono::milliseconds captureDelay = std::chrono::milliseconds(3000));

    // Update with the current body data and timestamp
    // Returns true if a snapshot was just captured
    bool UpdateData(k4abt_body_t selectedBody, uint64_t currentTimestampUsec);

    // Get the remaining seconds until snapshot (for display)
    float GetRemainingSeconds() const;

    // Get countdown display text
    std::string GetCountdownText() const;

    bool IsBothHandsRaised() const;
    bool IsSnapshotTaken() const;
    bool IsCountdownStarted() const;

    // Reset the snapshot state to allow capturing again
    void Reset();

    // Manually trigger a snapshot capture (mapped to 'r' key)
    void TriggerManualCapture(const k4abt_body_t& body);

private:
    void SaveSkeletonSnapshot(const k4abt_body_t& body);
    const char* GetJointName(int jointId) const;

    std::chrono::milliseconds m_captureDelay;
    std::chrono::microseconds m_previousTimestamp;
    std::chrono::microseconds m_handsRaisedTimeSpan;
    bool m_bothHandsAreRaised;
    bool m_snapshotTaken;
    bool m_countdownStarted;
};
