#ifndef COLLISIONLOOKUP
#define COLLISIONLOOKUP

#include "dubins.h"
#include "constants.h"
#include "constants_aws.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <string>
namespace plt =matplotlibcpp;
#include <Python.h>
#include"matplotlibcpp.h"

namespace HybridAStar {
namespace Lookup {

inline void dubinsLookup(float* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the Dubin's lookup table...";

  DubinsPath path;

  int width = Constants::dubinsWidth / Constants::cellSize;

  const int headings = Constants::headings;

  double start[3];
  double goal[] = {0, 0, 0};

  for (int X = 0; X < width; ++X) {
    start[0] = X;

    for (int Y = 0; Y < width; ++Y) {
      start[1] = Y;

      for (int h0 = 0; h0 < headings; ++h0) {
        start[2] = Constants::deltaHeadingRad * h0;

        for (int h1 = 0; h1 < headings; ++h1) {
          goal[2] = Constants::deltaHeadingRad * h1;

          dubins_init(start, goal, Constants::r, &path);
          lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] = dubins_path_length(&path);

          if (DEBUG && lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] < sqrt(X * X + Y * Y) * 1.000001) {
            std::cout << X << " | " << Y << " | "
                      << Constants::deltaHeadingDeg* h0 << " | "
                      << Constants::deltaHeadingDeg* h1 << " length: "
                      << lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] << "\n";

          }
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

inline int sign(double x) {
  if (x >= 0) { return 1; }
  else { return -1; }
}

inline void collisionLookup(Constants::config* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the collision lookup table...";

  const float cSize = Constants::cellSize;

  const int size = Constants::bbSize;

  struct point {
    double x;
    double y;
  };

  point c;
  point temp;

  point p[4];
  point nP[4];

  double theta;

  point t;
  point start;
  point end;

  int X;
  int Y;

  double tMaxX;
  double tMaxY;

  double tDeltaX;
  double tDeltaY;

  int stepX;
  int stepY;

  bool cSpace[size * size];
  bool inside = false;
  int hcross1 = 0;
  int hcross2 = 0;

  int count = 0;
  const int positionResolution = Constants::positionResolution;
  const int positions = Constants::positions;
  point points[positions];

  for (int i = 0; i < positionResolution; ++i) {
    for (int j = 0; j < positionResolution; ++j) {
      points[positionResolution * i + j].x = 1.f / positionResolution * j;
      points[positionResolution * i + j].y = 1.f / positionResolution * i;
    }
  }

  for (int q = 0; q < positions; ++q) {

    theta = 0;

    c.x = (double)size / 2 + points[q].x;
    c.y = (double)size / 2 + points[q].y;

    p[0].x = c.x - Constants::length() / 2.0 / cSize;
    p[0].y = c.y - Constants::width() / 2.0 / cSize;

    p[1].x = c.x - Constants::length() / 2.0 / cSize;
    p[1].y = c.y + Constants::width() / 2.0 / cSize;

    p[2].x = c.x + Constants::length() / 2.0 / cSize;
    p[2].y = c.y + Constants::width() / 2.0 / cSize;

    p[3].x = c.x + Constants::length() / 2.0 / cSize;
    p[3].y = c.y - Constants::width() / 2.0 / cSize;

    for (int o = 0; o < Constants::headings; ++o) {
      if (DEBUG) { std::cout << "\ndegrees: " << theta * 180.f / M_PI << std::endl; }

      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          cSpace[i * size + j] = false;
        }
      }

      for (int j = 0; j < 4; ++j) {

        temp.x = p[j].x - c.x;
        temp.y = p[j].y - c.y;

        nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
        nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
      }

      theta += Constants::deltaHeadingRad;

      for (int k = 0; k < 4; ++k) {

        if (k < 3) {
          start = nP[k];
          end = nP[k + 1];
        } else {
          start = nP[k];
          end = nP[0];
        }

        X = (int)start.x;
        Y = (int)start.y;

        cSpace[Y * size + X] = true;
        t.x = end.x - start.x;
        t.y = end.y - start.y;
        stepX = sign(t.x);
        stepY = sign(t.y);

        if (t.x != 0) {
          tDeltaX = 1.f / std::abs(t.x);
        } else {
          tDeltaX = 1000;
        }

        if (t.y != 0) {
          tDeltaY = 1.f / std::abs(t.y);
        } else {
          tDeltaY = 1000;
        }

        if (stepX > 0) {
          tMaxX = tDeltaX * (1 - (start.x - (long)start.x));
        } else {
          tMaxX = tDeltaX * (start.x - (long)start.x);
        }

        if (stepY > 0) {
          tMaxY = tDeltaY * (1 - (start.y - (long)start.y));
        } else {
          tMaxY = tDeltaY * (start.y - (long)start.y);
        }

        while ((int)end.x != X || (int)end.y != Y) {

          if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {

            tMaxX = tMaxX + tDeltaX;
            X = X + stepX;
            cSpace[Y * size + X] = true;

          }

          else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
            tMaxY = tMaxY + tDeltaY;
            Y = Y + stepY;
            cSpace[Y * size + X] = true;
          }

          else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {

            if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {

              X = X + stepX;
              cSpace[Y * size + X] = true;
            } else {
              Y = Y + stepY;
              cSpace[Y * size + X] = true;
            }
          } else {

            std::cout << "\n--->tie occured, please check for error in script\n";
            break;
          }
        }
      }

      for (int i = 0; i < size; ++i) {

        inside = false;
        hcross1 = -1;
        hcross2 = -1;

        for (int k = 0; k < size; ++k) {
          if (cSpace[i * size + k]) {
            if (!inside) {
              hcross1 = k;
              inside = true;
            }
            hcross2 = k;
          }
        }

        for (int j = 0; j < size; ++j) {

          if (j > hcross1 && j < hcross2 && inside) {
            cSpace[i * size + j] = true;
          }
        }
      }

      count = 0;

      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          if (cSpace[i * size + j]) {

            lookup[q * Constants::headings + o].pos[count].x = j - (int)c.x;
            lookup[q * Constants::headings + o].pos[count].y = i - (int)c.y;

            count++;
          }
        }
      }

      lookup[q * Constants::headings + o].length = count;

      if (DEBUG) {

        for (int i = 0; i < size; ++i) {
          std::cout << "\n";

          for (int j = 0; j < size; ++j) {
            if (cSpace[i * size + j]) {
              std::cout << "#";
            } else {
              std::cout << ".";
            }
          }
        }

        std::cout << "\n\nthe center of " << q* Constants::headings + o << " is at " << c.x << " | " << c.y << std::endl;

        for (int i = 0; i < lookup[q * Constants::headings + o].length; ++i) {
          std::cout << "[" << i << "]\t" << lookup[q * Constants::headings + o].pos[i].x << " | " << lookup[q * Constants::headings + o].pos[i].y << std::endl;
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

inline void collisionLookup_polygon_Ellipse(Constants::config* lookup, bool enable_visualization = false){
  printf("使用椭圆包络避障方法\r\n");

    if (enable_visualization) {
        Py_Initialize();
        if (!Py_IsInitialized()) {
            std::cerr << "Failed to initialize Python interpreter!" << std::endl;
        }
        PyRun_SimpleString("import matplotlib; matplotlib.use('TkAgg')");
    }

    const std::vector<Eigen::Vector2d> car_shape_ellipse = HybridAStar::Constants::my_car_shape_for_collision();

    int positionResolution;
    int headings;
    double delta_heading_rad;
    double cell_size;
    if(Constants::Algorithm_Framework=="AWS_from_others"){
      positionResolution = Constants::positionResolution;
      headings = Constants::headings;
      delta_heading_rad = Constants::deltaHeadingRad;
      cell_size = Constants::cellSize;
    }else{
      positionResolution = Constants_aws::positionResolution;
      headings = Constants_aws::headings;
      delta_heading_rad = Constants_aws::deltaHeadingRad;
      cell_size = Constants_aws::cellSize;
    }

    struct Point { double x; double y; };

    double min_x = car_shape_ellipse[0].x()-std::sqrt(2)*std::abs(car_shape_ellipse[0].x()-car_shape_ellipse[1].x());
    double max_x = car_shape_ellipse[0].x()+std::sqrt(2)*std::abs(car_shape_ellipse[0].x()-car_shape_ellipse[1].x());
    double min_y = car_shape_ellipse[0].y()-std::sqrt(2)*std::abs(car_shape_ellipse[0].y()-car_shape_ellipse[1].y());
    double max_y = car_shape_ellipse[0].y()+std::sqrt(2)*std::abs(car_shape_ellipse[0].y()-car_shape_ellipse[1].y());

    std::vector<std::vector<Point>> p_on_ellipse;

    std::vector<std::vector<Point>> p_on_ellipse_after;

    for (int i=0 ; i< (car_shape_ellipse.size())/2.0 ; i++) {
        int first_tip=2*i;
        double x0=car_shape_ellipse[first_tip].x();
        double y0=car_shape_ellipse[first_tip].y();
        double xp=car_shape_ellipse[first_tip+1].x();
        double yp=car_shape_ellipse[first_tip+1].y();

        min_x = std::min(min_x,x0-std::sqrt(2)*std::abs(x0-xp));
        max_x = std::max(max_x,x0+std::sqrt(2)*std::abs(x0-xp));
        min_y = std::min(min_y,y0-std::sqrt(2)*std::abs(y0-yp));
        max_y = std::max(max_y,y0+std::sqrt(2)*std::abs(y0-yp));
        std::vector<Point>  p_on_ellipse_i;

        for(int j=0;j<45;j++){
          double p_on_ellipse_x=std::sqrt(2)*(xp-x0)*std::sin(j*8/180.0*M_PI)+x0;
          double p_on_ellipse_y=std::sqrt(2)*(yp-y0)*std::cos(j*8/180.0*M_PI)+y0;
          Point p_on_ellipse_ij={p_on_ellipse_x,p_on_ellipse_y};
          p_on_ellipse_i.push_back(p_on_ellipse_ij);
        }
          p_on_ellipse.push_back(p_on_ellipse_i);
      }

    double shape_width = max_x - min_x;
    double shape_height = max_y - min_y;

    double diagonal = sqrt(shape_width * shape_width + shape_height * shape_height);

    int bb_size = std::ceil((diagonal + 4) / cell_size);

    if (bb_size & 1) {

      bb_size++;
    }

    const int num_ellipses = car_shape_ellipse.size() / 2;
    if (num_ellipses < 1) {
        std::cerr << "Error: At least one ellipse is required." << std::endl;
        return;
    }

    std::cout << "Ellipse Shape: " << num_ellipses << " ellipses, width=" << shape_width
              << ", height=" << shape_height << ", bb_size=" << bb_size << std::endl;

    bool DEBUG = false;
    std::cout << "I am building the collision lookup table for ellipse-based shape...";

    std::vector<Point> p(car_shape_ellipse.size());
    std::vector<Point> nP(car_shape_ellipse.size());

    Point c;
    Point temp;
    double theta;

    Point t, start, end;
    int X, Y;
    double tMaxX, tMaxY;
    double tDeltaX, tDeltaY;
    int stepX, stepY;

    std::vector<bool> cSpace(bb_size * bb_size, false);
    bool inside = false;
    int hcross1 = 0;
    int hcross2 = 0;

    int count = 0;
    const int positions = positionResolution * positionResolution;
    std::vector<Point> points(positions);

    auto setCellSafe = [&](int x, int y) {

        if (x >= 0 && x < bb_size && y >= 0 && y < bb_size) {
            cSpace[y * bb_size + x] = true;
        }

    };

    auto rasterizeLineBresenham = [&](Point start, Point end, auto setCellFunc, int gridSize) {

        int X = (int)start.x;
        int Y = (int)start.y;
        int endX = (int)end.x;
        int endY = (int)end.y;

        Point delta;
        delta.x = end.x - start.x;
        delta.y = end.y - start.y;

        int stepX = (delta.x > 0) ? 1 : ((delta.x < 0) ? -1 : 0);
        int stepY = (delta.y > 0) ? 1 : ((delta.y < 0) ? -1 : 0);

        double tDeltaX = (delta.x != 0) ? 1.0 / std::abs(delta.x) : 1e8;
        double tDeltaY = (delta.y != 0) ? 1.0 / std::abs(delta.y) : 1e8;

        double tMaxX = tDeltaX * ((stepX > 0) ? (ceil(start.x) - start.x) : (start.x - floor(start.x)));
        double tMaxY = tDeltaY * ((stepY > 0) ? (ceil(start.y) - start.y) : (start.y - floor(start.y)));

        setCellFunc(X, Y);

        while (X != endX || Y != endY) {
            if (abs(tMaxX) < abs(tMaxY)) {

                tMaxX += tDeltaX;
                X += stepX;
            } else if (abs(tMaxX) > abs(tMaxY)) {

                tMaxY += tDeltaY;
                Y += stepY;
            } else {
                tMaxX += tDeltaX;
                tMaxY += tDeltaY;
                X += stepX;
                Y += stepY;
            }

            setCellFunc(X, Y);
            if(enable_visualization)
            {
              printf("new p=[%d ,%d ]\r\n",X,Y);
            }
        }
    };

    for (int i = 0; i < positionResolution; ++i) {
        for (int j = 0; j < positionResolution; ++j) {
            points[positionResolution * i + j].x = 1.0 / positionResolution * j;
            points[positionResolution * i + j].y = 1.0 / positionResolution * i;
        }
    }
    if (enable_visualization) {
        plt::figure();
    }

    for (int q = 0; q < positions; ++q) {
        theta = 0;

        c.x = (double)bb_size / 2 + points[q].x;
        c.y = (double)bb_size / 2 + points[q].y;

        for (int i = 0; i < (int)car_shape_ellipse.size(); ++i) {

            p[i].x = c.x + car_shape_ellipse[i].x() / cell_size;
            p[i].y = c.y + car_shape_ellipse[i].y() / cell_size;
        }

        for (int o = 0; o < headings; ++o) {
            if (DEBUG) { std::cout << "\ndegrees: " << theta * 180.0 / M_PI << std::endl; }

            std::fill(cSpace.begin(), cSpace.end(), false);
              std::vector<double> xxx;
              std::vector<double> yyy;

            for (int j = 0; j < (int)car_shape_ellipse.size(); ++j) {
                temp.x = p[j].x - c.x;
                temp.y = p[j].y - c.y;
                nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
                nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
            }

            for(int i = 0; i < (int)car_shape_ellipse.size()/2; i++){
                std::vector<Point> p_on_ellipse_after_j;
                for(int j = 0; j < 45; j++){

                    p_on_ellipse_after_j.push_back({
                        p_on_ellipse[i][j].x * cos(theta) - p_on_ellipse[i][j].y * sin(theta) + c.x,
                        p_on_ellipse[i][j].x * sin(theta) + p_on_ellipse[i][j].y * cos(theta) + c.y});

                    xxx.push_back(p_on_ellipse_after_j[j].x);
                    yyy.push_back(p_on_ellipse_after_j[j].y);
                }
                p_on_ellipse_after.push_back(p_on_ellipse_after_j);
            }

            for (int j = 0; j < ((int)car_shape_ellipse.size())/2; ++j) {
                Eigen::Vector2d center={car_shape_ellipse[2*j].x(),car_shape_ellipse[2*j].y()};
                Eigen::Vector2d p_p   ={car_shape_ellipse[2*j-1].x(),car_shape_ellipse[2*j-1].y()};

                xxx.push_back(nP[j].x );
                yyy.push_back(nP[j].y );
            }

            if(enable_visualization

            ){

              plt::scatter(xxx,yyy,1);

              for (int i = -10; i <= 20; ++i) {
                std::vector<double> x_line = {double(i), double(i)};
                std::vector<double> y_line = {-5.0, 20.0};
                plt::plot(x_line, y_line, "g--");
              }

              for (int ii = -5; ii <= 20; ++ii) {
                std::vector<double> x_line = {-10.0, 20.0};
                std::vector<double> y_line = {double(ii), double(ii)};
                plt::plot(x_line, y_line, "g--");
              }

            }

            theta += delta_heading_rad;

            auto isInsideEllipse = [&](Point o,Point p, double theta_rad, Point c, Point p_) -> bool {
                double x=(p_.x-c.x)*std::cos(theta_rad)+(p_.y-c.y)*std::sin(theta_rad);
                double y=(p_.y-c.y)*std::cos(theta_rad)-(p_.x-c.x)*std::sin(theta_rad);
                double result = std::pow((x-o.x)/(p.x-o.x),2)+std::pow((y-o.y)/(p.y-o.y),2)-2;
                return result <= 0;
            };

            for (int e = 0; e < num_ellipses; ++e) {

                for (int y = 0; y <= bb_size; ++y) {
                    for (int x = 0; x <= bb_size; ++x) {
                        Point test_point = {(double)x, (double)y };
                        Point test_point1 = {(double)x+1.0, (double)y };
                        Point test_point2 = {(double)x, (double)y+1.0 };
                        Point test_point3 = {(double)x+1.0, (double)y+1.0 };

                        double theta_rad=o/(headings*1.00)*2*M_PI;
                        Point ellipse_o={car_shape_ellipse[2*e].x(),car_shape_ellipse[2*e].y()};
                        Point ellipse_p={car_shape_ellipse[2*e+1].x(),car_shape_ellipse[2*e+1].y()};

                        if (isInsideEllipse(ellipse_o,ellipse_p,theta_rad,c,test_point)
                        ||
                            isInsideEllipse(ellipse_o,ellipse_p,theta_rad,c,test_point1)||
                            isInsideEllipse(ellipse_o,ellipse_p,theta_rad,c,test_point2)||
                            isInsideEllipse(ellipse_o,ellipse_p,theta_rad,c,test_point3)
                      ) {
                            setCellSafe(x, y);
                            if(enable_visualization)
                            {
                              plt::plot({double(x)}, {double(y)}, "ko");
                            }

                        }
                    }
                }
            }
            if (enable_visualization) {
              plt::draw();
              plt::pause(10);
              plt::clf();
            }

            if(o==22&&q==24){

            }
            count = 0;
            for (int i = 0; i < bb_size; ++i) {
                for (int j = 0; j < bb_size; ++j) {
                    if (cSpace[i * bb_size + j]) {
                        lookup[q * headings + o].pos[count].x = j - (int)c.x;
                        lookup[q * headings + o].pos[count].y = i - (int)c.y;

                        if (enable_visualization && q==24 && o==22) {
                          printf("pos[%d].[x,y] =[%d , %d] \r\n",count,lookup[q * headings + o].pos[count].x,lookup[q * headings + o].pos[count].y);
                          plt::plot({double(j - (int)c.x)}, {double(i - (int)c.y)}, "bo");
                        }
                        count++;
                    }
                }
            }

            lookup[q * headings + o].length = count;
            if (DEBUG) {  }
        }
      }

    std::cout << " done!" << std::endl;
}

inline void collisionLookup_polygon(Constants::config* lookup,
                                    bool enable_visualization = false,
                                    const std::vector<std::vector<int>>& body_triagle = {},
                                    bool beisen = true) {
  printf("使用三角避障方法\r\n");

    if (enable_visualization) {
        Py_Initialize();
        if (!Py_IsInitialized()) {
            std::cerr << "Failed to initialize Python interpreter!" << std::endl;
        }
        PyRun_SimpleString("import matplotlib; matplotlib.use('TkAgg')");
    }
    const std::vector<std::vector<int>>& polygon_triangles = body_triagle;

    const std::vector<Eigen::Vector2d> polygon_vertices = Constants::my_car_shape_for_collision();

    std::cout << "Original polygon shape (" << polygon_vertices.size() << " vertices):" << std::endl;
    for (size_t i = 0; i < polygon_vertices.size(); ++i) {
        std::cout << "  Vertex " << i << ": (" << polygon_vertices[i].x()
                  << ", " << polygon_vertices[i].y() << ")" << std::endl;
    }
    int positionResolution;
    int headings;
    double delta_heading_rad;
    double cell_size;

    if(Constants::Algorithm_Framework=="AWS_from_others"){
      positionResolution = Constants::positionResolution;
      headings = Constants::headings;
      delta_heading_rad = Constants::deltaHeadingRad;
      cell_size = Constants::cellSize;
    }else{
      positionResolution = Constants_aws::positionResolution;
      headings = Constants_aws::headings;
      delta_heading_rad = Constants_aws::deltaHeadingRad;
      cell_size = Constants_aws::cellSize;
    }

    double min_x = polygon_vertices[0].x();
    double max_x = polygon_vertices[0].x();
    double min_y = polygon_vertices[0].y();
    double max_y = polygon_vertices[0].y();

    for (const auto& vertex : polygon_vertices) {
        min_x = std::min(min_x, vertex.x());
        max_x = std::max(max_x, vertex.x());
        min_y = std::min(min_y, vertex.y());
        max_y = std::max(max_y, vertex.y());
    }

    double polygon_width = max_x - min_x;
    double polygon_height = max_y - min_y;

    double diagonal = sqrt(polygon_width * polygon_width + polygon_height * polygon_height);

    int bb_size = std::ceil((diagonal + 4) / cell_size);

    if (bb_size & 1) {

      bb_size++;
    }

    struct Point { double x; double y; };

    const int num_vertices = polygon_vertices.size();
    if (num_vertices < 3) {
        std::cerr << "Error: A polygon must have at least 3 vertices." << std::endl;
        return;
    }

    std::cout << "Polygon: " << num_vertices << " vertices, width=" << polygon_width
              << ", height=" << polygon_height << ", bb_size=" << bb_size << std::endl;

    bool DEBUG = false;
    std::cout << "I am building the collision lookup table for a custom polygon...";

    std::vector<Point> p(num_vertices);
    std::vector<Point> nP(num_vertices);

    Point c;
    Point temp;
    double theta;

    Point t, start, end;
    int X, Y;
    double tMaxX, tMaxY;
    double tDeltaX, tDeltaY;
    int stepX, stepY;

    std::vector<bool> cSpace(bb_size * bb_size, false);
    bool inside = false;
    int hcross1 = 0;
    int hcross2 = 0;

    int count = 0;
    const int positions = positionResolution * positionResolution;
    std::vector<Point> points(positions);

    auto setCellSafe = [&](int x, int y) {

        if (x >= 0 && x < bb_size && y >= 0 && y < bb_size) {
            cSpace[y * bb_size + x] = true;
        }

    };

    auto rasterizeLineBresenham = [&](Point start, Point end, auto setCellFunc, int gridSize) {
        if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
            !std::isfinite(end.x) || !std::isfinite(end.y)) {
            std::cerr << "rasterizeLineBresenham received non-finite input." << std::endl;
            return;
        }

        int X = static_cast<int>(std::floor(start.x));
        int Y = static_cast<int>(std::floor(start.y));
        int endX = static_cast<int>(std::floor(end.x));
        int endY = static_cast<int>(std::floor(end.y));

        Point delta;
        delta.x = end.x - start.x;
        delta.y = end.y - start.y;

        constexpr double kEps = 1e-12;
        int stepX = (delta.x > kEps) ? 1 : ((delta.x < -kEps) ? -1 : 0);
        int stepY = (delta.y > kEps) ? 1 : ((delta.y < -kEps) ? -1 : 0);

        const double inf = std::numeric_limits<double>::infinity();
        double tDeltaX = (stepX != 0) ? 1.0 / std::abs(delta.x) : inf;
        double tDeltaY = (stepY != 0) ? 1.0 / std::abs(delta.y) : inf;

        double tMaxX = inf;
        double tMaxY = inf;
        if (stepX > 0) {
            tMaxX = tDeltaX * (std::floor(start.x) + 1.0 - start.x);
        } else if (stepX < 0) {
            tMaxX = tDeltaX * (start.x - std::floor(start.x));
        }
        if (stepY > 0) {
            tMaxY = tDeltaY * (std::floor(start.y) + 1.0 - start.y);
        } else if (stepY < 0) {
            tMaxY = tDeltaY * (start.y - std::floor(start.y));
        }

        setCellFunc(X, Y);
        const int maxSteps = std::abs(endX - X) + std::abs(endY - Y) + std::max(4, gridSize * 2);
        int steps = 0;

        while (X != endX || Y != endY) {
            const int prevX = X;
            const int prevY = Y;

            if (tMaxX < tMaxY) {

                tMaxX += tDeltaX;
                X += stepX;
            } else if (tMaxX > tMaxY) {

                tMaxY += tDeltaY;
                Y += stepY;
            } else {
                tMaxX += tDeltaX;
                tMaxY += tDeltaY;
                X += stepX;
                Y += stepY;
            }

            if (X == prevX && Y == prevY) {
                std::cerr << "rasterizeLineBresenham made no progress: start=("
                          << start.x << "," << start.y << "), end=("
                          << end.x << "," << end.y << "), delta=("
                          << delta.x << "," << delta.y << ")" << std::endl;
                break;
            }

            if (++steps > maxSteps) {
                std::cerr << "rasterizeLineBresenham exceeded step guard: start=("
                          << start.x << "," << start.y << "), end=("
                          << end.x << "," << end.y << "), current=("
                          << X << "," << Y << "), target=("
                          << endX << "," << endY << ")" << std::endl;
                break;
            }

            setCellFunc(X, Y);
            if(enable_visualization)
            {
              printf("new p=[%d ,%d ]\r\n",X,Y);
            }
        }
    };

    for (int i = 0; i < positionResolution; ++i) {
        for (int j = 0; j < positionResolution; ++j) {
            points[positionResolution * i + j].x = 1.0 / positionResolution * j;
            points[positionResolution * i + j].y = 1.0 / positionResolution * i;
        }
    }
    if (enable_visualization) {
        plt::figure();
    }

    for (int q = 0; q < positions; ++q) {
        theta = 0;

        c.x = (double)bb_size / 2 + points[q].x;
        c.y = (double)bb_size / 2 + points[q].y;

        for (int i = 0; i < num_vertices; ++i) {

            p[i].x = c.x + polygon_vertices[i].x() / cell_size;
            p[i].y = c.y + polygon_vertices[i].y() / cell_size;
        }

        for (int o = 0; o < headings; ++o) {
            if (DEBUG) { std::cout << "\ndegrees: " << theta * 180.0 / M_PI << std::endl; }

            std::fill(cSpace.begin(), cSpace.end(), false);

              std::vector<double> xxx;
              std::vector<double> yyy;

            for (int j = 0; j < num_vertices; ++j) {
                temp.x = p[j].x - c.x;
                temp.y = p[j].y - c.y;
                nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
                nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;

                  xxx.push_back(nP[j].x );
                  yyy.push_back(nP[j].y );

            }
                  xxx.push_back(xxx[0]);
                  yyy.push_back(yyy[0]);

            if(enable_visualization&& q==24 && o==22){

              plt::plot(xxx,yyy,"r-");

              for (int i = -10; i <= 20; ++i) {
                std::vector<double> x_line = {double(i), double(i)};
                std::vector<double> y_line = {-5.0, 20.0};
                plt::plot(x_line, y_line, "g--");
              }

              for (int ii = -5; ii <= 20; ++ii) {
                std::vector<double> x_line = {-10.0, 20.0};
                std::vector<double> y_line = {double(ii), double(ii)};
                plt::plot(x_line, y_line, "g--");
              }
            }

            theta += delta_heading_rad;

            if (polygon_triangles.empty()) {
                std::cerr << "Error: No triangles provided for collision detection!" << std::endl;
                continue;
            }

            for (const auto& tri : polygon_triangles) {
                if (tri.size() != 3) {
                    std::cerr << "Warning: Invalid triangle with " << tri.size() << " vertices" << std::endl;
                    continue;
                }

                if (tri[0] < 0 || tri[0] >= (int)nP.size() ||
                    tri[1] < 0 || tri[1] >= (int)nP.size() ||
                    tri[2] < 0 || tri[2] >= (int)nP.size()) {
                    std::cerr << "Warning: Invalid triangle indices [" << tri[0] << "," << tri[1] << "," << tri[2]
                              << "] for nP.size() = " << nP.size() << std::endl;
                    continue;
                }

                Point v0 = nP[tri[0]];
                Point v1 = nP[tri[1]];
                Point v2 = nP[tri[2]];
                if (beisen) {

                  rasterizeLineBresenham(v0, v1, setCellSafe, bb_size);
                  rasterizeLineBresenham(v1, v2, setCellSafe, bb_size);
                  rasterizeLineBresenham(v2, v0, setCellSafe, bb_size);
                }

                double min_x = std::min({v0.x, v1.x, v2.x});
                double max_x = std::max({v0.x, v1.x, v2.x});
                double min_y = std::min({v0.y, v1.y, v2.y});
                double max_y = std::max({v0.y, v1.y, v2.y});

                int start_x = std::max(0, (int)floor(min_x));
                int end_x = std::min(bb_size - 1, (int)ceil(max_x));
                int start_y = std::max(0, (int)floor(min_y));
                int end_y = std::min(bb_size - 1, (int)ceil(max_y));

                for (int y = start_y; y <= end_y; ++y) {
                    for (int x = start_x; x <= end_x; ++x) {

                        Point test_point = {(double)x , (double)y};

                        double denominator = ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
                        if (std::abs(denominator) < 1e-10) continue;
                        double a = ((v1.y - v2.y) * (test_point.x - v2.x) + (v2.x - v1.x) * (test_point.y - v2.y)) / denominator;
                        double b = ((v2.y - v0.y) * (test_point.x - v2.x) + (v0.x - v2.x) * (test_point.y - v2.y)) / denominator;
                        double c = 1 - a - b;

                        if (a >= 0 && b >= 0 && c >= 0) {
                            setCellSafe(x, y);
                        }
                    }
                }
            }

            if (enable_visualization) {
                for (int i = 0; i < bb_size; ++i) {
                    for (int j = 0; j < bb_size; ++j) {
                        if (cSpace[i * bb_size + j]) {

                            int world_x = (c.x - bb_size/2 + j) * cell_size;
                            int world_y = (c.y - bb_size/2 + i) * cell_size;
                           if (enable_visualization&& q==24 && o==22) {
                            plt::plot({double(world_x)}, {double(world_y)}, "ko");
                           }
                        }
                    }
                }

            }

            if(o==22&&q==24){

            }
            count = 0;
            for (int i = 0; i < bb_size; ++i) {
                for (int j = 0; j < bb_size; ++j) {
                    if (cSpace[i * bb_size + j]) {
                        lookup[q * headings + o].pos[count].x = j - (int)c.x;
                        lookup[q * headings + o].pos[count].y = i - (int)c.y;

                        if (enable_visualization && q==24 && o==22) {
                          printf("pos[%d].[x,y] =[%d , %d] \r\n",count,lookup[q * headings + o].pos[count].x,lookup[q * headings + o].pos[count].y);

                        }
                        count++;
                    }
                }
            }
            if (enable_visualization && q==24 && o==22) {
              plt::show();
              plt::pause(1);
              plt::clf();
            }
            lookup[q * headings + o].length = count;
            if(q * headings + o==78){

            }

            if (DEBUG) {  }
        }
    }
    std::cout << " done!" << std::endl;
}

}
}
#endif
