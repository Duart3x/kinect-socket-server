// Licensed under the MIT License.

#include "SkeletonSocketSender.h"
#include <nlohmann/json.hpp>
#include <iostream>

using json = nlohmann::json;

SkeletonSocketSender::SkeletonSocketSender(const std::string& host, int port)
	: m_host(host)
	, m_port(port)
	, m_socket(INVALID_SOCKET)
	, m_initialized(false)
	, m_connected(false)
{
}

SkeletonSocketSender::~SkeletonSocketSender()
{
	Close();
}

bool SkeletonSocketSender::Initialize()
{
	if (m_initialized)
	{
		return true;
	}

	// Initialize Winsock
	WSADATA wsaData;
	int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (result != 0)
	{
		printf("WSAStartup failed with error: %d\n", result);
		return false;
	}

	m_initialized = true;

	// Create socket
	m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_socket == INVALID_SOCKET)
	{
		printf("Socket creation failed with error: %ld\n", WSAGetLastError());
		WSACleanup();
		m_initialized = false;
		return false;
	}

	// Setup server address structure
	sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(m_port);

	// Convert IP address from string to binary form
	if (inet_pton(AF_INET, m_host.c_str(), &serverAddr.sin_addr) <= 0)
	{
		printf("Invalid address / Address not supported\n");
		closesocket(m_socket);
		m_socket = INVALID_SOCKET;
		WSACleanup();
		m_initialized = false;
		return false;
	}

	// Connect to server
	result = connect(m_socket, (sockaddr*)&serverAddr, sizeof(serverAddr));
	if (result == SOCKET_ERROR)
	{
		printf("Connection failed with error: %ld\n", WSAGetLastError());
		printf("Make sure the server is running at %s:%d\n", m_host.c_str(), m_port);
		closesocket(m_socket);
		m_socket = INVALID_SOCKET;
		WSACleanup();
		m_initialized = false;
		return false;
	}

	m_connected = true;
	printf("Connected to server at %s:%d\n", m_host.c_str(), m_port);
	return true;
}

bool SkeletonSocketSender::SendSkeletonData(const k4abt_body_t& body, uint64_t timestamp)
{
	if (!m_connected || m_socket == INVALID_SOCKET)
	{
		return false;
	}

	// Create JSON from skeleton
	std::string jsonData = CreateJsonFromSkeleton(body, timestamp);

	// Add newline delimiter for easier parsing on receiver side
	jsonData += "\n";

	// Send data
	int result = send(m_socket, jsonData.c_str(), (int)jsonData.length(), 0);
	if (result == SOCKET_ERROR)
	{
		printf("Send failed with error: %ld\n", WSAGetLastError());
		m_connected = false;
		return false;
	}

	return true;
}

void SkeletonSocketSender::Close()
{
	if (m_socket != INVALID_SOCKET)
	{
		closesocket(m_socket);
		m_socket = INVALID_SOCKET;
	}

	if (m_initialized)
	{
		WSACleanup();
		m_initialized = false;
	}

	m_connected = false;
}

bool SkeletonSocketSender::IsConnected() const
{
	return m_connected;
}

std::string SkeletonSocketSender::CreateJsonFromSkeleton(const k4abt_body_t& body, uint64_t timestamp)
{
	json jsonData;

	jsonData["body_id"] = body.id;
	jsonData["timestamp"] = timestamp;

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

	// Return compact JSON (no indentation for faster transmission)
	return jsonData.dump();
}

const char* SkeletonSocketSender::GetJointName(int jointId) const
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
