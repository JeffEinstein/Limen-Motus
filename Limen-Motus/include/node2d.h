#ifndef NODE2D_H
#define NODE2D_H

#include <cmath>

#include "constants.h"
namespace HybridAStar {

class Node2D {
 public:

  Node2D(): Node2D(0, 0, 0, 0, nullptr) {}

  Node2D(int x, int y, float g, float h, Node2D* pred) {
    this->x = x;
    this->y = y;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->d = false;
    this->idx = -1;
  }

  int getX() const { return x; }

  int getY() const { return y; }

  float getG() const { return g; }

  float getH() const { return h; }

  float getC() const { return g + h; }

  int getIdx() const { return idx; }

  bool  isOpen() const { return o; }

  bool  isClosed() const { return c; }

  bool  isDiscovered() const { return d; }

  Node2D* getPred() const { return pred; }

  void setX(const int& x) { this->x = x; }

  void setY(const int& y) { this->y = y; }

  void setG(const float& g) { this->g = g; }

  void setH(const float& h) { this->h = h; }

  int setIdx(int width) { this->idx = y * width + x; return idx;}

  void open() { o = true; c = false; }

  void close() { c = true; o = false; }

  void reset() { c = false; o = false; }

  void discover() { d = true; }

  void setPred(Node2D* pred) { this->pred = pred; }

  void updateG() { g += movementCost(*pred); d = true; }

  void updateH(const Node2D& goal) { h = movementCost(goal); }

  float movementCost(const Node2D& pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }

  bool operator == (const Node2D& rhs) const;

  bool isOnGrid(const int width, const int height) const;

  Node2D* createSuccessor(const int i);

  static const int dir;

  static const int dx[];

  static const int dy[];

 private:

  int x;

  int y;

  float g;

  float h;

  int idx;

  bool o;

  bool c;

  bool d;

  Node2D* pred;
};
}
#endif
