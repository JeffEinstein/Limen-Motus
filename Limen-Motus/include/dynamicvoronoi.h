#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

#include "bucketedqueue.h"

namespace HybridAStar {

class DynamicVoronoi {

 public:

  DynamicVoronoi();
  ~DynamicVoronoi();

  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);

  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  void occupyCell(int x, int y);

  void clearCell(int x, int y);

  void exchangeObstacles(const std::vector<IntPoint> &newObstacles);

  void update(bool updateRealDist = true);

  void prune();

  float getDistance(int x, int y) const;

  bool isVoronoi(int x, int y) const;

  bool isOccupied(int x, int y) const;

  void visualize(const char* filename = "result.ppm");

  unsigned int getSizeX() const {return sizeX;}

  unsigned int getSizeY() const {return sizeY;}

 public:
  struct dataCell {
    float dist;
    char voronoi;
    char queueing;
    int obstX;
    int obstY;
    bool needsRaise;
    int sqdist;
  };

  typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
  typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;

  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist = true);
  inline void reviveVoroNeighbors(int& x, int& y);

  inline bool isOccupied(int& x, int& y, dataCell& c);
  inline markerMatchResult markerMatch(int x, int y);

  BucketPrioQueue open;
  std::queue<INTPOINT> pruneQueue;

  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;

  int sizeY;
  int sizeX;
  dataCell** data;
  bool** gridMap;

  int padding;
  double doubleThreshold;

  double sqrt2;

};
}

#endif
