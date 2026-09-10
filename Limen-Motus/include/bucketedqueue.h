#ifndef _PRIORITYQUEUE2_H_
#define _PRIORITYQUEUE2_H_

#define MAXDIST 1000
#define RESERVE 64

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include "point.h"

namespace HybridAStar {

class BucketPrioQueue {

 public:

  BucketPrioQueue();

  bool empty() const;

  void push(int prio, INTPOINT t);

  INTPOINT pop();

 private:

  static void initSqrIndices();
  static std::vector<int> sqrIndices;
  static int numBuckets;
  int count;
  int nextBucket;

  std::vector<std::queue<INTPOINT> > buckets;
};
}
#endif
