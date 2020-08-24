#pragma once

#ifdef _WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
#elif
#include <netdb.h>
#include <netinet/ip.h>
#include <sys/ioctl.h>
#endif

#include <climits>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "../../../common/viewer.h"

#include "glm/glm.hpp"

enum INCOMING_MESSAGE_TYPE {
	MSG_CAPTURE_FRAME,
	MSG_CALIBRATE,
	MSG_RECEIVE_SETTINGS,
	MSG_REQUEST_STORED_FRAME,
	MSG_REQUEST_LAST_FRAME,
	MSG_RECEIVE_CALIBRATION,
	MSG_CLEAR_STORED_FRAMES
};

enum OUTGOING_MESSAGE_TYPE {
	MSG_CONFIRM_CAPTURED,
	MSG_CONFIRM_CALIBRATED,
	MSG_STORED_FRAME,
	MSG_LAST_FRAME
};

typedef struct Point3s {
	Point3s() {
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
	}
	Point3s(short X, short Y, short Z) {
		this->X = X;
		this->Y = Y;
		this->Z = Z;
	}
	short X;
	short Y;
	short Z;
} Point3s;

typedef struct Point3f {
	Point3f() {
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
	}
	Point3f(float X, float Y, float Z) {
		this->X = X;
		this->Y = Y;
		this->Z = Z;
	}
	float X;
	float Y;
	float Z;
} Point3f;

typedef struct Point2f {
	Point2f() {
		this->X = 0;
		this->Y = 0;
	}
	Point2f(float X, float Y) {
		this->X = X;
		this->Y = Y;
	}
	float X;
	float Y;
} Point2f;

typedef struct Color {
	Color() {
		this->rgbRed = UCHAR_MAX;
		this->rgbGreen = UCHAR_MAX;
		this->rgbBlue = UCHAR_MAX;
	}
	Color(uint8_t rgbRed, uint8_t rgbGreen, uint8_t rgbBlue) {
		this->rgbRed = rgbRed;
		this->rgbGreen = rgbGreen;
		this->rgbBlue = rgbBlue;
	}
	uint8_t rgbRed;
	uint8_t rgbGreen;
	uint8_t rgbBlue;
} Color;

Point3f RotatePoint(Point3f &point, std::vector<std::vector<float>> &R);

Point3f InverseRotatePoint(Point3f &point, std::vector<std::vector<float>> &R);

glm::dvec3 rotate_vector(const glm::dvec3& v, const glm::dvec3& k, double theta);

Color coord_to_rgb(rs2::video_frame texture, rs2::texture_coordinate coord,
		int tex_width = -1, int tex_height = -1,
		int tex_bytes_per_pixel = -1, int tex_stride = -1);

Color get_pixel_color(rs2::video_frame texture, int x, int y,
	int tex_width, int tex_height, int tex_bytes_per_pixel,
	int tex_stride);

std::string receive_bytes(int sockfd);

void send_bytes(int sockfd, const char *buf, int len);

/////////////////////////////////////////////////////////////////
// Parallel for from:
// https://ideone.com/Z7zldb
////////////////////////////////////////////////////////////////

class ThreadPool {
	public:
		template <typename Index, typename Callable>
			static void ParallelFor(Index start, Index end, Callable func) {
				// Estimate number of threads in the pool
				const static unsigned nb_threads_hint =
					std::thread::hardware_concurrency();
				const static unsigned nb_threads =
					(nb_threads_hint == 0u ? 8u : nb_threads_hint);

				// Size of a slice for the range functions
				Index n = end - start + 1;
				Index slice = (Index)std::round(n / static_cast<double>(nb_threads));
				slice = std::max(slice, Index(1));

				// [Helper] Inner loop
				auto launchRange = [&func](int k1, int k2) {
					for (Index k = k1; k < k2; k++) {
						func(k);
					}
				};

				// Create pool and launch jobs
				std::vector<std::thread> pool;
				pool.reserve(nb_threads);
				Index i1 = start;
				Index i2 = std::min(start + slice, end);
				for (unsigned i = 0; i + 1 < nb_threads && i1 < end; ++i) {
					pool.emplace_back(launchRange, i1, i2);
					i1 = i2;
					i2 = std::min(i2 + slice, end);
				}
				if (i1 < end) {
					pool.emplace_back(launchRange, i1, end);
				}

				// Wait for jobs to finish
				for (std::thread &t : pool) {
					if (t.joinable()) {
						t.join();
					}
				}
			}
};
