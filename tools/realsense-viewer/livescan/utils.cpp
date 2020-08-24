#include "utils.h"

Point3f RotatePoint(Point3f &point, std::vector<std::vector<float>> &R) {
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res.Y = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res.Z = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	return res;
}

Point3f InverseRotatePoint(Point3f &point, std::vector<std::vector<float>> &R) {
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[1][0] + point.Z * R[2][0];
	res.Y = point.X * R[0][1] + point.Y * R[1][1] + point.Z * R[2][1];
	res.Z = point.X * R[0][2] + point.Y * R[1][2] + point.Z * R[2][2];

	return res;
}

Color coord_to_rgb(rs2::video_frame texture, rs2::texture_coordinate coord,
		int tex_width, int tex_height, int tex_bytes_per_pixel,
		int tex_stride) {
	// get width and height if not specified
	if (tex_width == -1) tex_width = texture.get_width();
	if (tex_height == -1) tex_height = texture.get_height();

	// normals to texture Coordinates conversion
	int x_value =
		(std::min)((std::max)(int(coord.u * tex_width + .5f), 0), tex_width - 1);
	int y_value =
		(std::min)((std::max)(int(coord.v * tex_height + .5f), 0), tex_height - 1);

	// get pixel size and stride if not specified
	if (tex_bytes_per_pixel == -1)
		tex_bytes_per_pixel = texture.get_bytes_per_pixel();
	int bytes = x_value * tex_bytes_per_pixel;

	if (tex_stride == -1) tex_stride = texture.get_stride_in_bytes();
	int strides = y_value * tex_stride;

	int texture_index = (bytes + strides);

	const auto new_texture =
		reinterpret_cast<const uint8_t *>(texture.get_data());

	// RGB components to save in tuple
	int NT1 = new_texture[texture_index];
	int NT2 = new_texture[texture_index + 1];
	int NT3 = new_texture[texture_index + 2];

	return Color(NT1, NT2, NT3);
}

Color get_pixel_color(rs2::video_frame texture, int x, int y,
	int tex_width, int tex_height, int tex_bytes_per_pixel,
	int tex_stride) {
	// get width and height if not specified
	if (tex_width == -1) tex_width = texture.get_width();
	if (tex_height == -1) tex_height = texture.get_height();

	// get pixel size and stride if not specified
	if (tex_bytes_per_pixel == -1)
		tex_bytes_per_pixel = texture.get_bytes_per_pixel();
	int bytes = x * tex_bytes_per_pixel;

	if (tex_stride == -1) tex_stride = texture.get_stride_in_bytes();
	int strides = y * tex_stride;

	int texture_index = (bytes + strides);

	const auto new_texture =
		reinterpret_cast<const uint8_t *>(texture.get_data());

	// RGB components to save in tuple
	int NT1 = new_texture[texture_index];
	int NT2 = new_texture[texture_index + 1];
	int NT3 = new_texture[texture_index + 2];

	return Color(NT1, NT2, NT3);
}

glm::dvec3 rotate_vector(const glm::dvec3& v, const glm::dvec3& k, double theta) {
	double cos_theta = cos(theta);
	double sin_theta = sin(theta);
	glm::dvec3 rotated = (v * cos_theta) + (glm::cross(k, v) * sin_theta) + (k * glm::dot(k, v)) * (1 - cos_theta);
	return rotated;
}

std::string receive_bytes(int sockfd) {
	std::string ret;
	char buf[1024];

	while (1) {
		u_long arg = 0;
#ifdef _WIN32
		if (ioctlsocket(sockfd, FIONREAD, &arg) != 0) break;
#elif
		if (ioctl(sockfd, FIONREAD, &arg) != 0) break;
#endif

		if (arg == 0) break;

		if (arg > 1024) arg = 1024;

		int rv = recv(sockfd, buf, arg, 0);
		if (rv <= 0) break;

		std::string t;

		t.assign(buf, rv);
		ret += t;
	}

	return ret;
}

void send_bytes(int sockfd, const char *buf, int len) {
	send(sockfd, buf, len, 0);
}
