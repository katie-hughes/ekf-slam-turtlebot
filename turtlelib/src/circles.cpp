#include "turtlelib/circles.hpp"

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Polar & p){
        os << "Angle: " << p.theta << ",\tR: " << p.r;
        return os;
    }

    std::ostream & operator<<(std::ostream & os, const Circle & c){
      os << "Center: (" << c.x << ", " << c.y << ") Rad: " << c.r;
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
      // std::cout << "Hello??" << std::endl;
      // Length of cluster must be at least 4
      if (cluster.size() <= 3){
        throw std::logic_error("Must have at least 4 points in cluster!");
      }
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

      // std::cout << "Data Mat\n" << data_matrix << std::endl;
      // std::cout << "Moment Mat\n" << moment_mat << std::endl;

      arma::mat constraint_mat(4, 4, arma::fill::zeros);
      constraint_mat.at(0,0) = 8*zbar;
      constraint_mat.at(1,1) = 1;
      constraint_mat.at(2,2) = 1;
      constraint_mat.at(3,0) = 2;
      constraint_mat.at(0,3) = 2;

      // std::cout << "Constraint mat\n" << constraint_mat << std::endl;
      arma::mat constraint_mat_inv = constraint_mat.i();

      // 9. Compute Singular Value Decomposition of Z (data matrix)
      arma::mat U;
      arma::vec s;
      arma::mat V;
      arma::svd(U, s, V, data_matrix);

      // std::cout << "U\n" << U << std::endl;
      // std::cout << "S\n" << s << std::endl;
      // std::cout << "V\n" << V << std::endl;

      // if smallest singular value is less than 10^-12 then A is 4th col of V
      // how do I determine this??
      // the elements of S are "singular values"
      double smallest_singular_value = s(3);
      // std::cout << "Smallest singular value " << smallest_singular_value << std::endl;
      arma::vec A;
      if (smallest_singular_value < 1e-12){
        // std::cout << "Take A straight from V" << std::endl;
        // A is 4th column of V matrix
        A = V.col(3);
      } else {
        // std::cout << "Do more calculations" << std::endl;
        // arma::mat sigma(npoints, 4, arma::fill::zeros);
        // sigma should NOT be a square matrix. but it might be for this calculation LOL
        arma::mat sigma(4, 4, arma::fill::zeros);
        for (int i = 0; i < 4; i++){
          sigma.at(i,i) = s.at(i);
        }
        // std::cout << "Sigma\n" << sigma << std::endl;
        // std::cout << "should be data mat\n" << U*sigma*V.t() << std::endl;
        // it fails on this line :( V * sigma is incompatible
        arma::mat Y = V * sigma * V.t();
        // std::cout << "Y\n" << Y << std::endl;
        arma::mat Q = Y * constraint_mat_inv * Y;
        // std::cout << "Q\n" << Q << std::endl;
        // Find the eigenvectors and values of Q
        arma::cx_vec eigenvalues;
        arma::cx_mat eigenvectors;
        arma::eig_gen(eigenvalues, eigenvectors, Q);
        // std::cout << "vec\n" << eigenvectors << std::endl;
        // std::cout << "val\n" << eigenvalues << std::endl;
        // Find the smallest positive eigenvalue of Q
        // the eigenvectors are stored as column vectors
        double smallest_positive_eigenvalue = -1.0;
        double smallest_positive_eigenvalue_index = -1.0;
        for (int i = 0; i < 4; i++){
          const auto curr_eigenvalue = eigenvalues.at(i).real();
          // std::cout << "Eig " << curr_eigenvalue << std::endl;
          if ((curr_eigenvalue > 0) && ((curr_eigenvalue < smallest_positive_eigenvalue) || 
                                        (smallest_positive_eigenvalue == -1.0))){
            smallest_positive_eigenvalue = curr_eigenvalue;
            smallest_positive_eigenvalue_index = i;
          }
        }
        // std::cout << "Smallest " << smallest_positive_eigenvalue << " idx " << smallest_positive_eigenvalue_index << std::endl;
        arma::cx_vec Astar = eigenvectors.col(smallest_positive_eigenvalue_index);
        // std::cout << "Astar\n" << Astar << std::endl;
        // Solve YA = Astar
        // A = Yinv Astar
        arma::cx_mat A_cx = Y.i() * Astar;
        // std::cout << "A_complex\n" << A_cx << std::endl;
        A = arma::vec(4);
        for (int i = 0; i < 4; i++){
          A.at(i) = A_cx.at(i).real();
        }
        // std::cout << "A\n" << A << std::endl;
      }
      // finally convert to circle
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