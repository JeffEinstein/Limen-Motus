#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>
#include "Sampling_M.h"

#include "constants.h"
#include "constants_aws.h"
#include "helper.h"
namespace HybridAStar {

class Node3D {
 public:

  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}

  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
  }

  float getX() const { return x; }

  float getY() const { return y; }

  float getT() const { return t; }

  float getG() const { return g; }

  float getH() const { return h; }

  float getC() const { return g + h; }

  int getIdx() const { return idx; }

  int getPrim() const { return prim; }

  bool isOpen() const { return o; }

  bool isClosed() const { return c; }

  const Node3D* getPred() const { return pred; }

  bool getIsOk() const { return isok; }

  void setIsOk(const bool& ok) { this->isok = ok; }

  int get_index_neighbor()  const{ return index_neighbor; }
  int get_index_neighbor_simple()  const{ return index_neighbor_simple; }

  float &getG_mutable() { return g; }

  void setX(const float& x) { this->x = x; }

  void setY(const float& y) { this->y = y; }

  void setT(const float& t) { this->t = t; }

  void setG(const float& g) { this->g = g; }

  void setH(const float& h) { this->h = h; }

  int setIdx(int width, int height) { this->idx = (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return idx;}

  int setIdxM(float realX, float realY, float realT,int width, int height) {

    int x = (int)realX;
    int y = (int)realY;

    int iX = (int)((realX - x) * Constants::positionResolution);
    int iY = (int)((realY - y) * Constants::positionResolution);

    int iT = (int)(realT / Constants::deltaHeadingRad);

    this->idx =  y * width * Constants::positionResolution * Constants::positionResolution * Constants::headings +
           x * Constants::positionResolution * Constants::positionResolution * Constants::headings +
           iY * Constants::positionResolution * Constants::headings +
           iX * Constants::headings +
           iT;
    return idx;
}

  void set_index_neighbor(int index) {index_neighbor=index;}
  void set_index_neighbor_simple(int index) {index_neighbor_simple=index;}

  void open() { o = true; c = false;}

  void close() { c = true; o = false; }

  void reset() { c = false; o = false;index_neighbor_simple=-1;index_neighbor=-1; }

  void setPred(const Node3D* pred) { this->pred = pred; }

  void updateG();

  double res_xy=1.0f/Constants::positionResolution;
  double res_t=Constants::deltaHeadingRad;
  bool operator == (const Node3D& rhs) const;

  bool isInRange(const Node3D& goal) const;

  bool isOnGrid(const int width, const int height) const;

  Node3D* createSuccessor(const int i);
  Node3D* createSuccessor_DSU(const int i);

  static const int dir;

  static const float dx[];

  static const float dy[];

  static const float dt[];

 protected:
  int setIdxValue(int value) { idx = value; return idx; }

 private:

  float x;

  float y;

  float t;

  float g;

  float h;

  int idx;

  bool o;

  bool c;

  int prim;

  const Node3D* pred;

  bool isok ;

  int index_neighbor;
  int index_neighbor_simple;

};
}
#endif
