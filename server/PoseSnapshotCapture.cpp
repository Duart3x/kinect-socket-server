// Licensed under the MIT License.

#include "PoseSnapshotCapture.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
#include <ctime>

using json = nlohmann::json;
using namespace std::chrono;

PoseSnapshotCapture::PoseSnapshotCapture(milliseconds captureDelay)
	: m_captureDelay(captureDelay)
	, m_previousTimestamp(microseconds::zero())
	, m_handsRaisedTimeSpan(microseconds::zero())
	, m_bothHandsAreRaised(false)
	, m_snapshotTaken(false)
	, m_countdownStarted(false)
{
}

bool PoseSnapshotCapture::UpdateData(k4abt_body_t selectedBody, uint64_t currentTimestampUsec)
{
	k4a_float3_t leftWristJoint = selectedBody.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position;
	k4a_float3_t rightWristJoint = selectedBody.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position;
	k4a_float3_t headJoint = selectedBody.skeleton.joints[K4ABT_JOINT_HEAD].position;

	// Notice: y direction is pointing towards the ground! So jointA.y < jointB.y means jointA is higher than jointB
	bool bothHandsAreRaised = leftWristJoint.xyz.y < headJoint.xyz.y &&
		rightWristJoint.xyz.y < headJoint.xyz.y;

	microseconds currentTimestamp(currentTimestampUsec);
	if (m_previousTimestamp == microseconds::zero())
	{
		m_previousTimestamp = currentTimestamp;
		m_handsRaisedTimeSpan = microseconds::zero();
	}

	microseconds elapsedTime = currentTimestamp - m_previousTimestamp;

	// Start countdown when hands are raised for the first time
	if (bothHandsAreRaised && !m_countdownStarted && !m_snapshotTaken)
	{
		m_countdownStarted = true;
		m_handsRaisedTimeSpan = microseconds::zero();
		m_bothHandsAreRaised = true;
	}

	// Continue countdown once started, regardless of hand position
	if (m_countdownStarted && !m_snapshotTaken)
	{
		m_handsRaisedTimeSpan += elapsedTime;

		if (m_handsRaisedTimeSpan >= m_captureDelay)
		{
			// Capture snapshot!
			SaveSkeletonSnapshot(selectedBody);
			m_snapshotTaken = true;

			// Auto-reset after capture
			Reset();

			m_previousTimestamp = currentTimestamp;
			return true;
		}
	}

	// Update hand raised status for display purposes
	m_bothHandsAreRaised = bothHandsAreRaised;

	m_previousTimestamp = currentTimestamp;
	return false;
}

float PoseSnapshotCapture::GetRemainingSeconds() const
{
	if (!m_countdownStarted || m_snapshotTaken)
		return 0.0f;

	auto remaining = duration_cast<milliseconds>(m_captureDelay - m_handsRaisedTimeSpan);
	return remaining.count() / 1000.0f;
}

std::string PoseSnapshotCapture::GetCountdownText() const
{
	if (!m_countdownStarted)
		return "";

	if (m_snapshotTaken)
		return "SNAPSHOT CAPTURED!";

	float remaining = GetRemainingSeconds();
	if (remaining > 0.0f)
	{
		char buffer[64];
		snprintf(buffer, sizeof(buffer), "Snapshot in: %.1f", remaining);
		return std::string(buffer);
	}

	return "";
}

bool PoseSnapshotCapture::IsBothHandsRaised() const
{
	return m_bothHandsAreRaised;
}

bool PoseSnapshotCapture::IsSnapshotTaken() const
{
	return m_snapshotTaken;
}

bool PoseSnapshotCapture::IsCountdownStarted() const
{
	return m_countdownStarted;
}

void PoseSnapshotCapture::Reset()
{
	m_snapshotTaken = false;
	m_bothHandsAreRaised = false;
	m_countdownStarted = false;
	m_handsRaisedTimeSpan = microseconds::zero();
	m_previousTimestamp = microseconds::zero();
}

void PoseSnapshotCapture::SaveSkeletonSnapshot(const k4abt_body_t& body)
{
	// Generate filename with timestamp
	auto now = system_clock::now();
	auto time = system_clock::to_time_t(now);

	char filename[256];
	std::tm timeinfo;
	localtime_s(&timeinfo, &time);
	strftime(filename, sizeof(filename), "pose_snapshot_%Y%m%d_%H%M%S.json", &timeinfo);

	// Create JSON object
	json jsonData;
	jsonData["body_id"] = body.id;
	jsonData["timestamp"] = filename;

	// Add all joints
	json jointsArray = json::array();
	for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
	{
		const k4a_float3_t& pos = body.skeleton.joints[joint].position;
		const k4a_quaternion_t& ori = body.skeleton.joints[joint].orientation;
		k4abt_joint_confidence_level_t confidence = body.skeleton.joints[joint].confidence_level;

		json jointObj;
		jointObj["joint_id"] = joint;
		jointObj["joint_name"] = GetJointName(joint);

		jointObj["position"] = {
			   {"x", pos.xyz.x},
		 {"y", pos.xyz.y},
			   {"z", pos.xyz.z}
		};

		jointObj["orientation"] = {
			{"w", ori.wxyz.w},
 {"x", ori.wxyz.x},
			{"y", ori.wxyz.y},
			{"z", ori.wxyz.z}
		};

		jointObj["confidence_level"] = static_cast<int>(confidence);

		jointsArray.push_back(jointObj);
	}

	jsonData["joints"] = jointsArray;

	// Write to file with pretty print
	std::ofstream file(filename);
	if (!file.is_open())
	{
		printf("Error: Could not create snapshot file: %s\n", filename);
		return;
	}

	file << std::setw(2) << jsonData << std::endl;
	file.close();

	printf("Pose snapshot saved to: %s\n", filename);
}

const char* PoseSnapshotCapture::GetJointName(int jointId) const
{
	switch (jointId)
	{
	case K4ABT_JOINT_PELVIS: return "PELVIS";
	case K4ABT_JOINT_SPINE_NAVEL: return "SPINE_NAVEL";
	case K4ABT_JOINT_SPINE_CHEST: return "SPINE_CHEST";
	case K4ABT_JOINT_NECK: return "NECK";
	case K4ABT_JOINT_CLAVICLE_LEFT: return "CLAVICLE_LEFT";
	case K4ABT_JOINT_SHOULDER_LEFT: return "SHOULDER_LEFT";
	case K4ABT_JOINT_ELBOW_LEFT: return "ELBOW_LEFT";
	case K4ABT_JOINT_WRIST_LEFT: return "WRIST_LEFT";
	case K4ABT_JOINT_HAND_LEFT: return "HAND_LEFT";
	case K4ABT_JOINT_HANDTIP_LEFT: return "HANDTIP_LEFT";
	case K4ABT_JOINT_THUMB_LEFT: return "THUMB_LEFT";
	case K4ABT_JOINT_CLAVICLE_RIGHT: return "CLAVICLE_RIGHT";
	case K4ABT_JOINT_SHOULDER_RIGHT: return "SHOULDER_RIGHT";
	case K4ABT_JOINT_ELBOW_RIGHT: return "ELBOW_RIGHT";
	case K4ABT_JOINT_WRIST_RIGHT: return "WRIST_RIGHT";
	case K4ABT_JOINT_HAND_RIGHT: return "HAND_RIGHT";
	case K4ABT_JOINT_HANDTIP_RIGHT: return "HANDTIP_RIGHT";
	case K4ABT_JOINT_THUMB_RIGHT: return "THUMB_RIGHT";
	case K4ABT_JOINT_HIP_LEFT: return "HIP_LEFT";
	case K4ABT_JOINT_KNEE_LEFT: return "KNEE_LEFT";
	case K4ABT_JOINT_ANKLE_LEFT: return "ANKLE_LEFT";
	case K4ABT_JOINT_FOOT_LEFT: return "FOOT_LEFT";
	case K4ABT_JOINT_HIP_RIGHT: return "HIP_RIGHT";
	case K4ABT_JOINT_KNEE_RIGHT: return "KNEE_RIGHT";
	case K4ABT_JOINT_ANKLE_RIGHT: return "ANKLE_RIGHT";
	case K4ABT_JOINT_FOOT_RIGHT: return "FOOT_RIGHT";
	case K4ABT_JOINT_HEAD: return "HEAD";
	case K4ABT_JOINT_NOSE: return "NOSE";
	case K4ABT_JOINT_EYE_LEFT: return "EYE_LEFT";
	case K4ABT_JOINT_EAR_LEFT: return "EAR_LEFT";
	case K4ABT_JOINT_EYE_RIGHT: return "EYE_RIGHT";
	case K4ABT_JOINT_EAR_RIGHT: return "EAR_RIGHT";
	default: return "UNKNOWN";
	}
}
