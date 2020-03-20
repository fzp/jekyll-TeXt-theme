#include <vector>
#include <cmath>
#include <algorithm>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 0, 255, 255);

Model *model = NULL;
int *zbuffer = NULL;
const int width = 800;
const int height = 800;
const int depth = 255;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
	bool isSteep = false;
	if (std::abs(x1 - x0) < std::abs(y1 - y0))
	{
		std::swap(x0, y0);
		std::swap(x1, y1);
		isSteep = true;
	}

	if (x0 > x1)
	{
		std::swap(x0, x1);
		std::swap(y0, y1);
	}

	int y = y0;
	int derror = 2 * std::abs(y1 - y0);
	int error = 0;
	for (int x = x0; x <= x1; x++)
	{
		if (isSteep)
		{
			image.set(y, x, color);
		}
		else
		{
			image.set(x, y, color);
		}
		error += derror;
		if (error + x0 >= x1)
		{
			y += y1 > y0 ? 1 : -1;
			error -= 2 * (x1 - x0);
		}
	}
}

bool inside(Vec2i *pts, Vec2i p)
{
	Vec3i w = Vec3i(pts[2].x - pts[0].x, pts[1].x - pts[0].x, pts[0].x - p.x) ^
			  Vec3i(pts[2].y - pts[0].y, pts[1].y - pts[0].y, pts[0].y - p.y);
	if (std::abs(w.z) < 1)
		return false;
	if (w.z < 0)
	{
		w = w * (-1);
	}
	return w.x > 0 && w.y > 0 && (w.z - w.x - w.y) > 0;
}

bool inside(Vec3i *pts, Vec2i p)
{
	Vec3i w = Vec3i(pts[2].x - pts[0].x, pts[1].x - pts[0].x, pts[0].x - p.x) ^
			  Vec3i(pts[2].y - pts[0].y, pts[1].y - pts[0].y, pts[0].y - p.y);
	if (std::abs(w.z) < 1)
		return false;
	if (w.z < 0)
	{
		w = w * (-1);
	}
	return w.x > 0 && w.y > 0 && (w.z - w.x - w.y) > 0;
}

void triangle(Vec2i *pts, TGAImage &image, TGAColor color)
{
	std::sort(pts, pts + 3, [](Vec2i a, Vec2i b) -> bool { return a.x < b.x; });
	int smallx = pts[0].x;
	int largex = pts[2].x;
	std::sort(pts, pts + 3, [](Vec2i a, Vec2i b) -> bool { return a.y < b.y; });
	int smally = pts[0].y;
	int largey = pts[2].y;
	for (int x = smallx; x <= largex; x++)
		for (int y = smally; y <= largey; y++)
		{
			if (inside(pts, Vec2i(x, y)))
			{
				image.set(x, y, color);
			}
		}
}

Vec3f barycentric(Vec3i *pts, Vec2i p)
{
	Vec3f w = Vec3f(pts[2].x - pts[0].x, pts[1].x - pts[0].x, pts[0].x - p.x) ^
			  Vec3f(pts[2].y - pts[0].y, pts[1].y - pts[0].y, pts[0].y - p.y);
	if (std::abs(w.z) < 1)
		return Vec3f(-1, 1, 1);
	return Vec3f(1.f - (w.x + w.y) / w.z, w.y / w.z, w.x / w.z);
}

void triangle(Vec3i *pts, TGAImage &image, TGAColor color, int *zbuffer)
{
	std::sort(pts, pts + 3, [](Vec3i a, Vec3i b) -> bool { return a.x < b.x; });
	int smallx = pts[0].x;
	int largex = pts[2].x;
	std::sort(pts, pts + 3, [](Vec3i a, Vec3i b) -> bool { return a.y < b.y; });
	int smally = pts[0].y;
	int largey = pts[2].y;
	for (int x = smallx; x <= largex; x++)
		for (int y = smally; y <= largey; y++)
		{
			Vec3f bc_screen = barycentric(pts, Vec2i(x, y));
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0)
				continue;
			float z = bc_screen * Vec3f(pts[0].z, pts[1].z, pts[2].z);
			if (zbuffer[x + width * y] < z)
			{
				zbuffer[x + width * y] = z;
				image.set(x, y, color);
			}
		}
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
{
	if (t0.y == t1.y && t0.y == t2.y)
		return; // I dont care about degenerate triangles
	// sort the vertices, t0, t1, t2 lower−to−upper (bubblesort yay!)
	if (t0.y > t1.y)
		std::swap(t0, t1);
	if (t0.y > t2.y)
		std::swap(t0, t2);
	if (t1.y > t2.y)
		std::swap(t1, t2);
	int total_height = t2.y - t0.y;
	for (int i = 0; i < total_height; i++)
	{
		bool second_half = i > t1.y - t0.y || t1.y == t0.y;
		int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
		float alpha = (float)i / total_height;
		float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height; // be careful: with above conditions no division by zero here
		Vec2i A = t0 + (t2 - t0) * alpha;
		Vec2i B = second_half ? t1 + (t2 - t1) * beta : t0 + (t1 - t0) * beta;
		if (A.x > B.x)
			std::swap(A, B);
		for (int j = A.x; j <= B.x; j++)
		{
			image.set(j, t0.y + i, color); // attention, due to int casts t0.y+i != A.y
		}
	}
}

int testTriangle(int argc, char **argv)
{
	TGAImage frame(200, 200, TGAImage::RGB);
	Vec2i pts[3] = {Vec2i(10, 10), Vec2i(100, 30), Vec2i(190, 160)};
	triangle(pts, frame, TGAColor(255, 0, 0, 255));
	frame.flip_vertically(); // to place the origin in the bottom left corner of the image
	frame.write_tga_file("framebuffer.tga");
	return 0;
}

int testTriangle1(int argc, char **argv)
{
	TGAImage image(200, 200, TGAImage::RGB);
	Vec2i t0[3] = {Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80)};
	Vec2i t1[3] = {Vec2i(180, 50), Vec2i(150, 1), Vec2i(70, 180)};
	Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};
	triangle(t0, image, red);
	triangle(t1, image, white);
	triangle(t2, image, green);
	image.flip_vertically(); // to place the origin in the bottom left corner of the image
	image.write_tga_file("framebuffer.tga");
	return 0;
}

int testTriangle2(int argc, char **argv)
{
	if (2 == argc)
	{
		model = new Model(argv[1]);
	}
	else
	{
		model = new Model("obj/african_head.obj");
	}

	zbuffer = new int[width * height];
	for (int i = 0; i < width * height; i++)
	{
		zbuffer[i] = std::numeric_limits<int>::min();
	}

	TGAImage image(width, height, TGAImage::RGB);
	Vec3f light = Vec3f(0, 0, -1);
	for (int i = 0; i < model->nfaces(); i++)
	{

		std::vector<int> face = model->face(i);
		Vec3i screen_coords[3];
		Vec3f world_coords[3];
		for (int j = 0; j < 3; j++)
		{
			world_coords[j] = model->vert(face[j]);
			screen_coords[j] = Vec3i((world_coords[j].x + 1.) * width / 2.,
									 (world_coords[j].y + 1.) * height / 2.,
									 (world_coords[j].z + 1.) * depth / 2.);
		}
		Vec3f n = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
		n.normalize();
		float intensity = n * light;
		if (intensity > 0)
			triangle(screen_coords, image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255), zbuffer);
	}

	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	delete model;
	return 0;
}

int testLine(int argc, char **argv)
{
	if (2 == argc)
	{
		model = new Model(argv[1]);
	}
	else
	{
		model = new Model("obj/african_head.obj");
	}

	TGAImage image(width, height, TGAImage::RGB);
	for (int i = 0; i < model->nfaces(); i++)
	{
		std::vector<int> face = model->face(i);
		for (int j = 0; j < 3; j++)
		{
			Vec3f v0 = model->vert(face[j]);
			Vec3f v1 = model->vert(face[(j + 1) % 3]);
			int x0 = (v0.x + 1.) * width / 2.;
			int y0 = (v0.y + 1.) * height / 2.;
			int x1 = (v1.x + 1.) * width / 2.;
			int y1 = (v1.y + 1.) * height / 2.;
			line(x0, y0, x1, y1, image, white);
		}
	}

	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	delete model;
	return 0;
}

int main(int argc, char **argv)
{
	return testTriangle2(argc, argv);
}
