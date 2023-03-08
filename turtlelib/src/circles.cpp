#include "turtlelib/circles.hpp"

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Polar & p){
        os << "Angle: " << p.theta << ",\tR: " << p.r;
        return os;
    }

    double polarDistance(const Polar p1, const Polar p2){
      const auto x1 = p1.r*cos(p1.theta);
      const auto y1 = p1.r*sin(p1.theta);
      const auto x2 = p2.r*cos(p2.theta);
      const auto y2 = p2.r*sin(p2.theta);
      const auto dx = x2 - x1;
      const auto dy = y2 - y1;
      return sqrt(dx * dx + dy * dy);
    }

    bool atOrigin(const Polar p){
      return ((p.r == 0) && (p.theta == 0));
    }

    Vector2D toVector(const Polar p){
      return Vector2D{p.r*cos(p.theta), p.r*sin(p.theta)};
    }

    Circle detectCircle(const std::vector<Vector2D> cluster){
      Circle result;
      // compute hatx and haty, the mean of x, y coordinates
      double hatx = 0, haty = 0;
      int npoints = static_cast<int>(cluster.size());
      for (int i = 0; i < npoints; i++){
        hatx += cluster.at(i).x;
        haty += cluster.at(i).y;
      }
      hatx /= npoints;
      haty /= npoints;
      // shift the coordinates so that centroid is at origin
      std::vector<Vector2D> shifted_cluster;
      // compute zi = xi^2 + yi^2 for shifted
      std::vector<double> z;
      // and mean of z: zbar
      double zbar = 0;
      // fill in data matrix:
      // zi & xi & yi & 1 
      arma::mat data_matrix(npoints, 4);
      for (int i = 0; i < npoints; i++){
        // shift the cluster over
        Vector2D shifted{cluster.at(i).x - hatx, cluster.at(i).y - haty};
        shifted_cluster.push_back(shifted);
        // compute z of shifted
        const double zi = shifted.x * shifted.x + shifted.y * shifted.y;
        z.push_back(zi);
        zbar += zi;
        // fill data matrix
        data_matrix.at(i, 0) = zi;
        data_matrix.at(i, 1) = shifted.x;
        data_matrix.at(i, 2) = shifted.y;
        data_matrix.at(i, 3) = 1;
      }
      zbar /= npoints;
      // fill in moment matrix
      arma::mat moment_mat = (1.0/npoints)*data_matrix.t()*data_matrix;
      
      return result;
    }
}