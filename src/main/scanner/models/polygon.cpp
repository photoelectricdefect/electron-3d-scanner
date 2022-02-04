#include <models/polygon.hpp>
#include <cfloat>
#include <iostream>

        polygon::polygon(const std::vector<Eigen::Vector2d>& vertices_) : vertices(vertices_) {
            for(int i = 1; i < vertices.size(); i++) {
                sides.push_back(line_segment(vertices[i - 1], vertices[i]));
            }

            sides.push_back(line_segment(vertices[vertices.size() - 1], vertices[0]));
        }

        rectangle polygon::frame() {
			double min_x = DBL_MAX , max_x = -DBL_MAX,
			 min_y = DBL_MAX, max_y = -DBL_MAX;
				
					for(int i = 0; i < vertices.size(); i++) {
						if(vertices[i](0) < min_x) {
							min_x = vertices[i](0);
						}
						if(vertices[i](0) > max_x) {
							max_x = vertices[i](0);
						}
						if(vertices[i](1) < min_y) {
							min_y = vertices[i](1);
						}
						if(vertices[i](1) > max_y) {
							max_y = vertices[i](1);
						}
					}		
			
			return rectangle(Eigen::Vector2d(min_x, min_y), Eigen::Vector2d(max_x, max_y));
        }

		polygon polygon::translate(const Eigen::Vector2d& r0) {		
				std::vector<Eigen::Vector2d> vertices_tr = vertices;

				for(auto& v : vertices_tr) {
					v += r0;
				}


	        return polygon(vertices_tr);
		}

		void polygon::print_sides() {		
            for(int i = 0; i < sides.size(); i++) {
				std::cout<<"a: "<<sides[i].a<<std::endl;
				std::cout<<"b: "<<sides[i].b<<std::endl;
            }
		}