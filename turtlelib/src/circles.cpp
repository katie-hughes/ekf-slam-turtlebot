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

      arma::mat constraint_mat(4, 4, arma::fill::zeros);
      constraint_mat.at(0,0) = 8*zbar;
      constraint_mat.at(1,1) = 1;
      constraint_mat.at(2,2) = 1;
      constraint_mat.at(3,0) = 2;
      constraint_mat.at(0,3) = 2;

      arma::mat constraint_mat_inv = constraint_mat.i();

      // 9. Compute Singular Value Decomposition of Z (data matrix)
      arma::mat U;
      arma::vec sigma;
      arma::mat V;
      arma::svd(U, sigma, V, data_matrix);

      // if smallest singular value is less than 10^-12 then A is 4th col of V
      arma::vec A;
      // else let Y = 
      // finally convert to circle
      A = V.col(3);
      // do this for ease of matching notation
      double A1 = A.at(0);
      double A2 = A.at(1);
      double A3 = A.at(2);
      double A4 = A.at(3);
      // calculate coordinates
      double a = -0.5*A2/A1;
      double b = -0.5*A3/A1;
      double R2 = (A2*A2 + A3*A3 - 4*A1*A4)/(4*A1*A1);

      Circle result;
      result.r = sqrt(R2);
      result.x = a + hatx;
      result.y = b + haty;
      return result;
    }
}