#ifndef MT_TASK_QUEUE_TASK_H_
#define MT_TASK_QUEUE_TASK_H_

#include <functional>

namespace TaskQueue {

template <typename ReturnType, typename... Args>
class Task {
public:
  Task (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function)
   : task_id_(task_id), priority_(priority), task_function_(task_function) {}

  friend bool operator < (const Task& left, const Task& right);
  friend bool operator > (const Task& left, const Task& right);

  unsigned int GetPriority () {return priority_;}
  ReturnType GetResult () {return result_;}
  void RunTask ();

private:
  unsigned int task_id_;
  unsigned int priority_;
  std::function<ReturnType(Args...)> task_function_;
  ReturnType result_;
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_TASK_H_
