#include <models/line_segment.hpp>
#include <opencv2/imgproc.hpp>

namespace math_helpers {

static float eucl2D(const cv::Mat& a, const cv::Mat& b)
{
	const float *ptr1 = a.ptr<float> ();
	const float *ptr2 = b.ptr<float> ();

	return sqrt(pow(*ptr2 - *ptr1, 2) + pow(*(ptr2 + 1) - *(ptr1 + 1), 2));
}
 
static float eucl2D(const Eigen::Vector2f& pt1, const Eigen::Vector2f& pt2)
{
	return sqrt(pow(pt2(0) - pt1(0), 2) + pow(pt2(1) - pt1(1), 2));
}

static int cross2D(const Eigen::Vector2f& u, const Eigen::Vector2f& v)
{
	Eigen::Matrix2f A;
	A << u(0), v(0),
		u(1), v(1);
	return A.determinant();
}

//TODO: improve by accounting for parallel lines
static bool intersection_line_segment(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2f& b2, Eigen::Vector2f& intersection)
{
	Eigen::Vector2f pq = a2 - a1,
		r = b1 - a1,
		s = b2 - a2;
	float a = cross2D(pq, r),
		b = cross2D(pq, s),
		c = cross2D(r, s);

	if (c != 0)
	{
		float t = b / c,
			u = a / c;

		if (0 <= t && t <= 1 &&
			0 <= u && u <= 1)
		{
			intersection = a1 + t * r;
			return true;
		}
	}

	return false;
}

//TODO: improve by accounting for parallel lines
static bool intersects_line_segment(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2f& b2)
{
	Eigen::Vector2f pq = a2 - a1,
		r = b1 - a1,
		s = b2 - a2;
	float a = cross2D(pq, r),
		b = cross2D(pq, s),
		c = cross2D(r, s);

	if (c != 0)
	{
		float t = b / c,
			u = a / c;

		if (0 <= t && t <= 1 &&
			0 <= u && u <= 1)
		{
			return true;
		}
	}

	return false;
}

static bool intersection_line(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2f& b2, Eigen::Vector2f& intersection)
{
	if(cross2D(b1 - a1, b2 - a2) == 0) return false;

	Eigen::Hyperplane<float, 2> u = Eigen::Hyperplane<float, 2>::Through(a1, b1),
		v = Eigen::Hyperplane<float, 2>::Through(a2, b2);

	intersection = u.intersection(v);

	return true;
}


static Eigen::Vector3f proj_a2b3D(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float b_squared_norm) {
	return (a.dot(b) / b_squared_norm) * b;
}

static Eigen::Vector3f proj_a2b3D(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
	return proj_a2b(a, b, b.squaredNorm());
}

static float cross_ratio(float A, float B, float C,
	float a, float b, float c, float q)
{
	float AC = C - A, BC = C - B,
		ac = c - a, bc = c - b,
		bq = q - b, aq = q - a;
	float alpha = (ac * bq * BC) / (bc * aq * AC);

	return (-alpha * A + B) / (1 - alpha);
}

static bool inside_polygon(const Eigen::Vector2f& a, const std::vector<line_segment>& sides) {
			int intersections = 0;
			Eigen::Vector2f O(0, a(1));
			bool on_line = false;

			for(int i = 0; i < sides.size(); i++) {
				Eigen::Vector2f pq = O - sides[i].a,
				r = sides[i].b - sides[i].a,
				s = a - O;
				float e = cross2D(pq, r),
				b = cross2D(pq, s),
				c = cross2D(r, s);

				if (c != 0)
				{
					float t = b / c,
					u = e / c;

					if (0 <= t && t <= 1 &&
					0 <= u && u <= 1)
					{
						intersections++;

						if(u == 1)  {
							on_line = true;
							break;
						}
					}
				}
			}	

			if(intersections % 2 > 0 && !on_line) return true;
			return false;
}

}