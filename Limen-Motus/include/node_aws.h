#ifndef NodeAWS_H
#define NodeAWS_H

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

#include "node3d.h"

namespace HybridAStar {

class NodeAWS : public Node3D{
 public:
  NodeAWS(): NodeAWS(0, 0, 0, 0, 0, 0, 0, 0, 0, nullptr, nullptr) {}
  NodeAWS(float x, float y, float t, float g, float h,
          float vx, float vy, float vyaw, int prim = 0,
          const NodeAWS* pred=nullptr,
          const std::vector<std::vector<float>> *sampling_table=nullptr
          )
          :Node3D(x, y, t, g, h, pred, prim) {

    _vx = vx;
    _vy = vy;
    _vyaw = vyaw;
    this->pred = pred;
    this->sampling_table = sampling_table;
  }
  NodeAWS(const Node3D& node3d, float vx=0, float vy=0, float vyaw=0,
          const NodeAWS* pred=nullptr,
          const std::vector<std::vector<float>> *sampling_table=nullptr)
          :Node3D(node3d) {

    _vx = vx;
    _vy = vy;
    _vyaw = vyaw;
    this->pred = pred;
    this->sampling_table = sampling_table;
  }
  ~NodeAWS() {}

  const float getVX() const { return _vx; }
  const float getVY() const { return _vy; }
  const float getVYaw() const { return _vyaw; }

  const NodeAWS* getPred() const { return pred; }
  void setPred(const NodeAWS* pred) { this->pred = pred; }

  int setIdxM(float realX, float realY, float realT, int width, int height) {
    const int resolution = Constants_aws::positionResolution;
    const int headings = Constants_aws::headings;
    const int x = static_cast<int>(realX);
    const int y = static_cast<int>(realY);
    const int iX = std::max(0, std::min(static_cast<int>((realX - x) * resolution), resolution - 1));
    const int iY = std::max(0, std::min(static_cast<int>((realY - y) * resolution), resolution - 1));
    const int iT = std::max(
        0,
        std::min(
            static_cast<int>(Helper::normalizeHeadingRad(realT) / Constants_aws::deltaHeadingRad),
            headings - 1));
    return setIdxValue(
        y * width * Constants_aws::positions * headings +
        x * Constants_aws::positions * headings +
        iY * resolution * headings +
        iX * headings +
        iT);
  }

  bool isInRange(const NodeAWS& goal) const;
  NodeAWS* createSuccessor(const int i);
  typedef std::pair<std::vector<double>,std::vector<double>> WheelCtrlCmd;
  WheelCtrlCmd RelativeVAndOmegaControl(double vx, double vy, double omega,
                                        double lim_angle = M_PI_2,
                                        bool wheel_rotate_velocity = false);

  void updateG();

  static const float aws_dx[];

  static const float aws_dy[];

  static const float aws_dt[];

  const std::vector<std::vector<float>> *sampling_table;

 private:

    float _vx;
    float _vy;
    float _vyaw;
    const NodeAWS* pred;

};

}
#endif
