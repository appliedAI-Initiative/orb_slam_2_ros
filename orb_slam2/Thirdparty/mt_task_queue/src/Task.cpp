#include "Task.h"

namespace TaskQueue {

template <typename ReturnType, typename... Args>
Task<ReturnType, Args...>::Task (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function)
: task_id_(task_id), priority_(priority), task_function_(task_function) {
  if (task_function_.target_type() != typeid(void(*)(Args...))) { //right? TODO, WARNING, fuck, i've no idea what im doing
    has_return_value_ = true;
  } else {
    has_return_value_ = false;
  }
}


template <typename ReturnType, typename... Args>
void Task<ReturnType, Args...>::Task::RunTask () {
  if (has_return_value_) {
    result_ = task_function_ ();
  } else {
    task_function_ ();
  }

}

} // namespace TaskQueue
