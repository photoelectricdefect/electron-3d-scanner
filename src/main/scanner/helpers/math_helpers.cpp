#include <models/line_segment.hpp>
#include <opencv2/imgproc.hpp>

namespace math_helpers {

// double eucl2D(const cv::Mat& a, const cv::Mat& b)
// {
// 	const double *ptr1 = a.ptr<double> ();
// 	const double *ptr2 = b.ptr<double> ();

// 	return sqrt(pow(*ptr2 - *ptr1, 2) + pow(*(ptr2 + 1) - *(ptr1 + 1), 2));
// }
 
double eucl2D(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
	return sqrt(pow(b(0) - a(0), 2) + pow(b(1) - a(1), 2));
}

double cross2D(const Eigen::Vector2d& u, const Eigen::Vector2d& v)
{
	return u(0) * v(1) - v(0) * u(1);
}

//TODO: improve by accounting for parallel lines
bool intersection_line_segment(const Eigen::Vector2d& a1, const Eigen::Vector2d& b1, const Eigen::Vector2d& a2, const Eigen::Vector2d& b2, Eigen::Vector2d& intersection)
{
	Eigen::Vector2d pq = a2 - a1,
		r = b1 - a1,
		s = b2 - a2;
	double a = cross2D(pq, r),
		b = cross2D(pq, s),
		c = cross2D(r, s);

	if (c != 0)
	{
		double t = b / c,
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

// //TODO: improve by accounting for parallel lines
// bool intersects_line_segment(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2f& b2)
// {
// 	Eigen::Vector2f pq = a2 - a1,
// 		r = b1 - a1,
// 		s = b2 - a2;
// 	float a = cross2D(pq, r),
// 		b = cross2D(pq, s),
// 		c = cross2D(r, s);

// 	if (c != 0)
// 	{
// 		float t = b / c,
// 			u = a / c;

// 		if (0 <= t && t <= 1 &&
// 			0 <= u && u <= 1)
// 		{
// 			return true;
// 		}
// 	}

// 	return false;
// }

bool intersection_line(const Eigen::Vector2d& a1, const Eigen::Vector2d& b1, const Eigen::Vector2d& a2, const Eigen::Vector2d& b2, Eigen::Vector2d& intersection)
{
	if(cross2D(b1 - a1, b2 - a2) == 0) return false;

	Eigen::Hyperplane<double, 2> u = Eigen::Hyperplane<double, 2>::Through(a1, b1),
		v = Eigen::Hyperplane<double, 2>::Through(a2, b2);

	intersection = u.intersection(v);

	return true;
}


Eigen::Vector3d proj_a2b3D(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double b_squared_norm) {
	return (a.dot(b) / b_squared_norm) * b;
}

Eigen::Vector3d proj_a2b3D(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	return proj_a2b3D(a, b, b.squaredNorm());
}

double cross_ratio(double A, double B, double C,
	double a, double b, double c, double q)
{
	double AC = C - A, BC = C - B,
		ac = c - a, bc = c - b,
		bq = q - b, aq = q - a;
	double alpha = (ac * bq * BC) / (bc * aq * AC);

	return (-alpha * A + B) / (1 - alpha);
}

bool inside_polygon(const Eigen::Vector2d& a, const std::vector<line_segment>& sides) {
			int intersections = 0;
			Eigen::Vector2d O(0, a(1));
			bool on_line = false;

			for(int i = 0; i < sides.size(); i++) {
				Eigen::Vector2d pq = O - sides[i].a,
				r = sides[i].b - sides[i].a,
				s = a - O;
				double e = cross2D(pq, r),
				b = cross2D(pq, s),
				c = cross2D(r, s);

				if (c != 0)
				{
					double t = b / c,
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