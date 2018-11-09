#include "Task.h"

namespace TaskQueue {

bool operator < (const Task& left, const Task& right) {
  return left.GetPriority() < right.GetPriority();
}


bool operator > (const Task& left, const Task& right) {
  return left.GetPriority() > right.GetPriority();
}


void Task::RunTask () {
  if () {
    result_ = task_function_ ();
  } else {
    task_function_ ();
  }

}

} // namespace TaskQueue
