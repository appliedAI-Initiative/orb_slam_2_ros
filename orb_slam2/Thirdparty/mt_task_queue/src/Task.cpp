#include "Task.h"

namespace TaskQueue {

bool operator < (const Task& left, const Task& right) {
  return left.GetPriority() < right.GetPriority();
}


bool operator > (const Task& left, const Task& right) {
  return left.GetPriority() > right.GetPriority();
}

} // namespace TaskQueue
