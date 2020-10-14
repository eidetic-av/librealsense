#pragma once

#ifdef _WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
#elif
#include <netdb.h>
#include <netinet/ip.h>
#include <unistd.h>
#endif

#include <string.h>
#include <thread>
#include <climits>

#include "../../common/viewer.h"
#include "livescan/calibration.h"
#include "livescan/utils.h"
#include "livescan/zstd.h"

#include "livescan/glm/glm.hpp"
#include <omp.h>

#ifdef _WIN32
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)
#define bcopy(s1, s2, n) memmove((s2), (s1), (n))
#endif

struct LiveScanSocket {
	std::shared_ptr<rs2::notifications_model> not_model = NULL;

	int sockfd;

	char server_ip[16] = "127.0.0.1";
	int server_port = 48001;
	bool connected = false;
	bool connecting = false;

	int device_count = 0;

	Calibration calibration;
	Point3f camera_offset;

	float manual_position_x = 0;
	float manual_position_y = 0;
	float manual_position_z = 0;
	float manual_rotation_x = 0;
	float manual_rotation_y = 0;
	float manual_rotation_z = 0;

	float cutoff_min_z = 0;
	float cutoff_max_z = 10;
	float cutoff_min_x = -10;
	float cutoff_max_x = 10;
	float cutoff_min_y = -10;
	float cutoff_max_y = 10;

	bool capture_frame;
	bool calibrate = false;
	bool confirm_calibrated = false;
	bool stream_only_bodies = false;
	bool confirm_captured;
	std::vector<float> bounds;
	bool filter;
	int filter_neighbors;
	float filter_threshold;
	bool enable_compression;
	int compression_level;

	std::mutex update_mutex;

	static void listener(LiveScanSocket* liveScanSocket);
	static void update(LiveScanSocket* liveScanSocket);

#ifdef _WIN32
	WSADATA wsa_data;
#endif

	void connect() {

		std::string connect_msg =
			"Attempting to connect to LiveScan server at ";
		connect_msg += server_ip;
		connect_msg += ":";
		connect_msg += std::to_string(server_port);
		not_model->add_log(connect_msg);
		connecting = true;

		// create server socket
#ifdef _WIN32
		int win_result;

		// Initialize Winsock
		win_result = WSAStartup(MAKEWORD(2, 2), &wsa_data);
		if (win_result != 0) {
			not_model->add_log("WSAStartup failed: " + std::to_string(win_result));
			return;
		}
		not_model->add_log("WSAStartup returned: " + std::to_string(win_result));
#elif
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0) {
			not_model->add_log("Failed create socket.");
			connecting = false;
			return;
		}
#endif

#ifdef _WIN32
		// convert host ip and port to struct
		struct addrinfo* result = NULL,
			* ptr = NULL,
			hints;

		SecureZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;

		// Resolve the server address and port
		win_result = getaddrinfo(server_ip, std::to_string(server_port).c_str(), &hints, &result);
		if (win_result != 0) {
			not_model->add_log("gettaddrinfo failed: " + std::to_string(win_result));
			WSACleanup();
			return;
		}

		// Attempt to connect to the first address returned by
		// the call to getaddrinfo
		ptr = result;

		// Create a SOCKET for connecting to server
		sockfd = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
		if (sockfd == INVALID_SOCKET) {
			not_model->add_log("Failed to connect:" + WSAGetLastError());
			connecting = false;
			freeaddrinfo(result);
			WSACleanup();
			return;
		}

		// try to connect
		win_result = ::connect(sockfd, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (win_result == SOCKET_ERROR) {
			closesocket(sockfd);
			sockfd = INVALID_SOCKET;
		}

		freeaddrinfo(result);

		if (sockfd == INVALID_SOCKET) {
			not_model->add_log("Failed to connect.");
			WSACleanup();
			return;
		}
#elif
		// try to reach server
		struct hostent* server = gethostbyname(server_ip);
		if (server == NULL) {
			not_model->add_log("Failed to connect. Is the server running?");
			connecting = false;
			return;
		}

		// convert host ip and port to C struct
		struct sockaddr_in serv_addr;
		bzero((char*)&serv_addr, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		bcopy((char*)server->h_addr, (char*)&serv_addr.sin_addr.s_addr,
			server->h_length);
		serv_addr.sin_port = htons(server_port);

		// try to connect
		if (::connect(sockfd, (struct sockaddr*)&serv_addr,
			sizeof(serv_addr)) < 0) {
			not_model->add_log("Failed to connect.");
			connecting = false;
			return;
		}
#endif
		// successfully connected
		not_model->add_log("Successfully connected to LiveScan server!");
		connecting = false;
		connected = true;

		// start listener thread
		std::thread t_listener(listener, this);
		t_listener.detach();

		// start frame send thread
		std::thread t_updater(update, this);
		t_updater.detach();
	}

	void disconnect() {
#ifdef _WIN32
		closesocket(sockfd);
#elif
		close(sockfd);
#endif
		not_model->add_log("Disconnected from LiveScan server.");
		connected = false;
	}

	bool new_frames = false;
	rs2::frame latest_depth_frame;
	rs2::frame latest_color_frame;

	void send_frame(rs2::frame f) {
		auto frames = f.as<rs2::frameset>();
		if (frames.size() >= 2)
		{
			latest_depth_frame = frames.get_depth_frame();
			latest_color_frame = frames.get_color_frame();
			new_frames = true;
		}
	}
};

void LiveScanSocket::listener(LiveScanSocket* liveScanSocket) {
	char byte_to_send;

	while (liveScanSocket->connected) {

		// throttle updates to 1000hz
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		std::string received = receive_bytes(liveScanSocket->sockfd);

		for (unsigned int i = 0; i < received.length(); i++) {
			if (received[i] == MSG_CAPTURE_FRAME) {
				liveScanSocket->capture_frame = true;

			}
			else if (received[i] == MSG_CALIBRATE) {
				liveScanSocket->calibration.bCalibrated = false;
				liveScanSocket->calibrate = true;

			}
			else if (received[i] == MSG_RECEIVE_SETTINGS) {

				std::vector<float> bounds(6);
				i++;
				int nBytes = *(int*)(received.c_str() + i);
				i += sizeof(int);

				for (int j = 0; j < 6; j++) {
					bounds[j] = *(float*)(received.c_str() + i);
					i += sizeof(float);
				}

				liveScanSocket->filter = (received[i] != 0);
				i++;

				liveScanSocket->filter_neighbors =
					*(int*)(received.c_str() + i);
				i += sizeof(int);

				liveScanSocket->filter_threshold =
					*(float*)(received.c_str() + i);
				i += sizeof(float);

				liveScanSocket->bounds = bounds;

				int num_markers = *(int*)(received.c_str() + i);
				i += sizeof(int);

				liveScanSocket->calibration.markerPoses.resize(num_markers);

				for (int j = 0; j < num_markers; j++) {
					for (int k = 0; k < 3; k++) {
						for (int l = 0; l < 3; l++) {
							liveScanSocket->calibration.markerPoses[j].R[k][l] = *(float*)(received.c_str() + i);
							i += sizeof(float);
						}
					}

					for (int k = 0; k < 3; k++) {
						liveScanSocket->calibration.markerPoses[j].t[k] = *(float*)(received.c_str() + i);
						i += sizeof(float);
					}

					liveScanSocket->calibration.markerPoses[j].markerId = *(int*)(received.c_str() + i);
					i += sizeof(int);
				}

				liveScanSocket->stream_only_bodies = (received[i] != 0);
				i += 1;

				liveScanSocket->compression_level =
					*(int*)(received.c_str() + i);
				i += sizeof(int);
				if (liveScanSocket->compression_level > 0)
					liveScanSocket->enable_compression = true;
				else
					liveScanSocket->enable_compression = false;

				// so that we do not lose the next character in the stream
				i--;
			}
			// send stored frame
			else if (received[i] == MSG_REQUEST_STORED_FRAME) {
				/* byteToSend = MSG_STORED_FRAME; */
				/* m_pClientSocket->SendBytes(&byteToSend, 1); */

				/* vector<Point3s> points; */
				/* vector<RGB> colors; */
				/* bool res = m_framesFileWriterReader.readFrame(points,
				 * colors); */
				 /* if (res == false) */
				 /* { */
				 /* 	int size = -1; */
				 /* 	m_pClientSocket->SendBytes((char*)&size, 4); */
				 /* } else */
				 /* 	SendFrame(points, colors, m_vLastFrameBody); */
			}
			// send last frame
			else if (received[i] == MSG_REQUEST_LAST_FRAME) {
				/* byteToSend = MSG_LAST_FRAME; */
				/* m_pClientSocket->SendBytes(&byteToSend, 1); */

				/* SendFrame(m_vLastFrameVertices, m_vLastFrameRGB,
				 * m_vLastFrameBody); */
			}
			// receive calibration data
			else if (received[i] == MSG_RECEIVE_CALIBRATION) {
				liveScanSocket->not_model->add_log("received cal");
				/* i++; */
				/* for (int j = 0; j < 3; j++) */
				/* { */
				/* 	for (int k = 0; k < 3; k++) */
				/* 	{ */
				/* 		calibration.worldR[j][k] =
				 * *(float*)(received.c_str() + i); */
				 /* 		i += sizeof(float); */
				 /* 	} */
				 /* } */
				 /* for (int j = 0; j < 3; j++) */
				 /* { */
				 /* 	calibration.worldT[j] = *(float*)(received.c_str() +
				  * i); */
				  /* 	i += sizeof(float); */
				  /* } */

				  /* //so that we do not lose the next character in the stream */
				  /* i--; */
			}
			else if (received[i] == MSG_CLEAR_STORED_FRAMES) {
				/* m_framesFileWriterReader.closeFileIfOpened(); */
			}
		}


		if (liveScanSocket->confirm_captured) {
			byte_to_send = MSG_CONFIRM_CAPTURED;
			send_bytes(liveScanSocket->sockfd, &byte_to_send, 1);
			liveScanSocket->confirm_captured = false;
		}

		if (liveScanSocket->confirm_calibrated) {

			std::lock_guard<std::mutex> lock(liveScanSocket->update_mutex);

			liveScanSocket->not_model->add_log("Successfully calibrated -- sending results.");

			int size = (9 + 3) * sizeof(float) + sizeof(int) + 1;
			char* buffer = new char[size];
			buffer[0] = MSG_CONFIRM_CALIBRATED;
			int i = 1;

			memcpy(buffer + i, &liveScanSocket->calibration.iUsedMarkerId, 1 * sizeof(int));
			i += 1 * sizeof(int);
			memcpy(buffer + i, liveScanSocket->calibration.worldR[0].data(), 3 * sizeof(float));
			i += 3 * sizeof(float);
			memcpy(buffer + i, liveScanSocket->calibration.worldR[1].data(), 3 * sizeof(float));
			i += 3 * sizeof(float);
			memcpy(buffer + i, liveScanSocket->calibration.worldR[2].data(), 3 * sizeof(float));
			i += 3 * sizeof(float);
			memcpy(buffer + i, liveScanSocket->calibration.worldT.data(), 3 * sizeof(float));
			i += 3 * sizeof(float);

			send_bytes(liveScanSocket->sockfd, buffer, size);
			liveScanSocket->confirm_calibrated = false;

			liveScanSocket->not_model->add_log("Sent.");
		}
	}
}

void LiveScanSocket::update(LiveScanSocket* liveScanSocket) {

	while (liveScanSocket->connected) {

		// wait until we have a valid frameset...
		// throttle updates to 100hz
		while (!liveScanSocket->new_frames)
			std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// map the point cloud
		rs2::pointcloud pointcloud;
		pointcloud.map_to(liveScanSocket->latest_color_frame);

		// generate the vertices
		rs2::points points = pointcloud.calculate(liveScanSocket->latest_depth_frame);
		auto vertices = points.get_vertices();

		/*liveScanSocket->
			spout_sender->SendImage((unsigned char const*)&vertices, 640, 480, GL_RGBA);*/

			// get the texture data to map coords to pixels
		auto tex_coords = points.get_texture_coordinates();
		auto video_frame = liveScanSocket->latest_color_frame.as<rs2::video_frame>();
		const int tex_width = video_frame.get_width();
		const int tex_height = video_frame.get_height();
		const int tex_bytes_per_pixel =
			video_frame.get_bytes_per_pixel();
		const int tex_stride = video_frame.get_stride_in_bytes();

		// create LiveScan data vectors to send
		std::vector<Point3s> vertex_data;
		std::vector<Color> color_data;

		std::vector<float> manual_position;
		std::vector<float> manual_rotation;

		manual_position.push_back(liveScanSocket->manual_position_x);
		manual_position.push_back(liveScanSocket->manual_position_y);
		manual_position.push_back(liveScanSocket->manual_position_z);

		manual_rotation.push_back(liveScanSocket->manual_rotation_x);
		manual_rotation.push_back(liveScanSocket->manual_rotation_y);
		manual_rotation.push_back(liveScanSocket->manual_rotation_z);

		for (int i = 0; i < points.size(); i++) {
			if (vertices[i].z) {


				bool discard = false;

				glm::dvec3 dvec_point(vertices[i].x, vertices[i].y, vertices[i].z);
				glm::dvec3 axis_one(0.f, 0.f, 1.f);
				glm::dvec3 axis_two(0.f, 1.f, 0.f);
				glm::dvec3 axis_three(1.f, 0.f, 0.f);

				glm::dvec3 rotated_once = rotate_vector(dvec_point, axis_one, manual_rotation[0]);
				glm::dvec3 rotated_twice = rotate_vector(rotated_once, axis_two, manual_rotation[1]);
				glm::dvec3 rotated_thrice = rotate_vector(rotated_twice, axis_three, manual_rotation[2]);

				float final_x = rotated_thrice[0] + manual_position[0];
				float final_y = rotated_thrice[1] + manual_position[1];
				float final_z = rotated_thrice[2] + manual_position[2];

				auto point = Point3f(final_x, final_y, final_z);

				discard = discard || (point.Z < liveScanSocket->cutoff_min_z);
				discard = discard || (point.Z > liveScanSocket->cutoff_max_z);
				discard = discard || (point.X < liveScanSocket->cutoff_min_x);
				discard = discard || (point.X > liveScanSocket->cutoff_max_x);
				discard = discard || (point.Y < liveScanSocket->cutoff_min_y);
				discard = discard || (point.Y > liveScanSocket->cutoff_max_y);

				if (!discard) {
					// position data is sent as short values in
					// millimetres
					short X = static_cast<short>(point.X * 1000);
					short Y = static_cast<short>(point.Y * 1000);
					short Z = static_cast<short>(point.Z * 1000);
					vertex_data.push_back(Point3s(X, Y, Z));

					// get the corresponding pixel's rgb value to send
					auto rgb = coord_to_rgb(
						liveScanSocket->latest_color_frame, tex_coords[i], tex_width,
						tex_height, tex_bytes_per_pixel, tex_stride);
					color_data.push_back(rgb);
				}
			}
		}

		if (liveScanSocket->calibrate) {

			liveScanSocket->not_model->add_log("Attempting to calibrate.");

			Point3f* camera_coordinates = new Point3f[tex_width * tex_height];

			std::vector<Color> tex_data;
			auto color_ptr = (uint8_t*)liveScanSocket->latest_color_frame.get_data();

			for (unsigned int y = 0; y < tex_height; y++) {
				for (unsigned int x = 0; x < tex_width; x++) {
					Color c = get_pixel_color(liveScanSocket->latest_color_frame, x, y, tex_width, tex_height, tex_bytes_per_pixel, tex_stride);
					tex_data.push_back(c);
				}
			}

			bool completed_calibration =
				liveScanSocket->calibration.Calibrate((Color*)tex_data.data(), camera_coordinates, tex_width, tex_height);

			liveScanSocket->camera_offset = Point3f(camera_coordinates->X, camera_coordinates->Y, camera_coordinates->Z);

			delete[] camera_coordinates;

			if (completed_calibration)
			{
				liveScanSocket->confirm_calibrated = true;
				liveScanSocket->calibrate = false;
			}

		}

		if (!liveScanSocket->confirm_calibrated) {

			std::lock_guard<std::mutex> lock(liveScanSocket->update_mutex);

			char byte_to_send = MSG_LAST_FRAME;
			send_bytes(liveScanSocket->sockfd, &byte_to_send, 1);

			int point_count = vertex_data.size();

			// create the buffer
			// buffer size is equal to the (the total number of vertices
			// sent * (r,g,b + x,y,z as shorts)) + sizeof (the point count)
			int buf_size = point_count * ((3 * sizeof(uint8_t)) +
				(3 * sizeof(short))) +
				sizeof(int);
			std::vector<char> buffer(buf_size);
			int pos = 0;

			// add the point count to start of the buffer
			memcpy(buffer.data(), &point_count, sizeof(point_count));
			pos += sizeof(point_count);

			char* vertex_data_ptr = (char*)vertex_data.data();
			char* color_data_ptr = (char*)color_data.data();

			for (unsigned int i = 0; i < point_count; i++) {
				// add the rgb values
				memcpy(buffer.data() + pos, color_data_ptr,
					sizeof(uint8_t) * 3);
				color_data_ptr += sizeof(uint8_t) * 3;
				pos += sizeof(uint8_t) * 3;
				// add the xyz values
				memcpy(buffer.data() + pos, vertex_data_ptr,
					sizeof(short) * 3);
				vertex_data_ptr += sizeof(short) * 3;
				pos += sizeof(short) * 3;
			}

			// add bodies
			int num_bodies = 0;
			buf_size += sizeof(num_bodies);
			buffer.resize(buf_size);
			memcpy(buffer.data() + pos, &num_bodies, sizeof(num_bodies));
			pos += sizeof(num_bodies);

			// create the TCP header with compression value 0 (for now)
			int compression = static_cast<int>(false);

			char header[8];
			memcpy(header, (char*)&buf_size, sizeof(buf_size));
			memcpy(header + 4, (char*)&compression, sizeof(compression));

			// send the header + buffer
			send_bytes(liveScanSocket->sockfd, (char*)&header,
				sizeof(int) * 2);
			send_bytes(liveScanSocket->sockfd, buffer.data(), buf_size);

		}
		liveScanSocket->new_frames = false;
	}
}

