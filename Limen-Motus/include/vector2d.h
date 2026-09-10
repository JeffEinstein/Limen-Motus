#ifndef VECTOR2D
#define VECTOR2D

#include <cmath>
#include <iostream>
namespace HybridAStar {

class Vector2D {
 public:

  inline Vector2D(const float x = 0, const float y = 0) { this->x = x; this->y = y; }

  inline Vector2D operator * (const float k) const { return Vector2D(x * k, y * k); }

  inline Vector2D operator / (const float k) const { return Vector2D(x / k, y / k); }

  inline Vector2D operator + (const Vector2D& b) const { return Vector2D(x + b.x, y + b.y); }

  inline Vector2D operator - (const Vector2D& b) const { return Vector2D(x - b.x, y - b.y); }

  inline Vector2D operator - () const  {return Vector2D(-x, -y);}

  friend std::ostream& operator<<(std::ostream& os, const Vector2D& b) {os << "(" << b.x << "|" << b.y << ")"; return os; }

  float length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }

  float sqlength() const { return x*x + y*y; }

  float dot(Vector2D b) const { return x * b.x + y * b.y; }

  inline Vector2D ort(Vector2D b) const {
    Vector2D a(this->x, this->y);
    Vector2D c;

    c = a - b * a.dot(b) / b.sqlength();
    return c;
  }
  inline float getX() { return x; }
  inline float getY() { return y; }

 private:

  float x;

  float y;

};
inline Vector2D operator * (double k, const Vector2D& b) {
  return (b * k);
}
}
#endif
