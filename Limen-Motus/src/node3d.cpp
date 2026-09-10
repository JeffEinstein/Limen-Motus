#include "node3d.h"

using namespace HybridAStar;

const int Node3D::dir = 3;

const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / Constants::deltaHeadingRad) >= 0 && (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}

bool Node3D::isInRange(const Node3D& goal) const {
  int random = rand() % 10 + 1;
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
}

Node3D* Node3D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  if (i < 3) {
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }

  else {
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}
Node3D* Node3D::createSuccessor_DSU(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  float step = Constants::cellSize / Constants::positionResolution;
  float angleStep = Constants::deltaHeadingRad;

  switch(i) {
    case 0:
      xSucc = x + step;
      ySucc = y;
      tSucc = t;
      break;
    case 1:
      xSucc = x - step;
      ySucc = y;
      tSucc = t;
      break;
    case 2:
      xSucc = x;
      ySucc = y + step;
      tSucc = t;
      break;
    case 3:
      xSucc = x;
      ySucc = y - step;
      tSucc = t;
      break;
    case 4:
      xSucc = x;
      ySucc = y;
      tSucc = Helper::normalizeHeadingRad(t + angleStep);
      break;
    case 5:
      xSucc = x;
      ySucc = y;
      tSucc = Helper::normalizeHeadingRad(t - angleStep);
      break;
    default:
      xSucc = x;
      ySucc = y;
      tSucc = t;
      break;
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

void Node3D::updateG() {

  if (prim < 3) {

    if (pred->prim != prim) {

      if (pred->prim > 2) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning;
      }
    } else {
      g += dx[0];
    }
  }

  else {

    if (pred->prim != prim) {

      if (pred->prim < 3) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } else {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}

bool Node3D::operator == (const Node3D& rhs) const {
  return std::abs(x-rhs.x)<=1 &&
         std::abs(y-rhs.y)<=1 &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
