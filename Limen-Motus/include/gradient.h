#ifndef GRADIENT
#define GRADIENT

#include <vector>

using namespace std;

namespace HybridAStar {

class ColorGradient {
 private:
  struct ColorPoint {
    float r, g, b;
    float val;
    ColorPoint(float red, float green, float blue, float value)
      : r(red), g(green), b(blue), val(value) {}
  };
  vector<ColorPoint> color;

 public:

  ColorGradient()  {  createDefaultHeatMapGradient();  }

  void addColorPoint(float red, float green, float blue, float value) {
    for (unsigned int i = 0; i < color.size(); i++)  {
      if (value < color[i].val) {
        color.insert(color.begin() + i, ColorPoint(red, green, blue, value));
        return;
      }
    }

    color.push_back(ColorPoint(red, green, blue, value));
  }

  void clearGradient() { color.clear(); }

  void createDefaultHeatMapGradient() {
    color.clear();
    color.push_back(ColorPoint(0, 0, 1,   0.0f));
    color.push_back(ColorPoint(0, 1, 1,   0.25f));
    color.push_back(ColorPoint(0, 1, 0,   0.5f));
    color.push_back(ColorPoint(1, 1, 0,   0.75f));
    color.push_back(ColorPoint(1, 0, 0,   1.0f));
  }

  void getColorAtValue(const float value, float& red, float& green, float& blue) {
    if (color.size() == 0)
    { return; }

    for (unsigned int i = 0; i < color.size(); i++) {
      ColorPoint& currC = color[i];

      if (value < currC.val) {
        ColorPoint& prevC  = color[ max(0, (int)i - 1) ];
        float valueDiff    = (prevC.val - currC.val);
        float fractBetween = (valueDiff == 0) ? 0 : (value - currC.val) / valueDiff;
        red   = (prevC.r - currC.r) * fractBetween + currC.r;
        green = (prevC.g - currC.g) * fractBetween + currC.g;
        blue  = (prevC.b - currC.b) * fractBetween + currC.b;
        return;
      }
    }

    red   = color.back().r;
    green = color.back().g;
    blue  = color.back().b;
    return;
  }
};
}
#endif
