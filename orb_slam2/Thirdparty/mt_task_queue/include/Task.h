/*
Small and simple task queue which supports mulithreading, result buffering and task priorization.
For more information see the README.
*/


#ifndef MT_TASK_QUEUE_TASK_H_
#define MT_TASK_QUEUE_TASK_H_

#include <functional>

namespace TaskQueue {

template <typename ReturnType, typename... Args>
class Task {
public:
  Task (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function);

  friend bool operator < (const Task& left, const Task& right) {
    return left.GetPriority() < right.GetPriority();
  }
  friend bool operator > (const Task& left, const Task& right) {
    return left.GetPriority() > right.GetPriority();
  }

  unsigned int GetPriority () {return priority_;}
  unsigned int GetId () {return task_id_;}
  bool HasReturnValue () {return has_return_value_;}
  ReturnType GetResult () {return result_;}
  void RunTask ();

private:
  unsigned int task_id_;
  unsigned int priority_;
  std::function<ReturnType(Args...)> task_function_;
  ReturnType result_;
  bool has_return_value_;
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_TASK_H_
