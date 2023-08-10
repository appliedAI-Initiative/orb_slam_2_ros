/*
Small and simple task queue which supports mulithreading, result buffering and task priorization.
For more information see the README.
*/


#ifndef MT_TASK_QUEUE_TASK_H_
#define MT_TASK_QUEUE_TASK_H_

#include <functional>

// required for explicit template instantiation in Task.cpp
#include <boost/any.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<opencv2/core/core.hpp>

namespace TaskQueue {

template <typename ReturnType, typename... Args>
class Task {
public:
  /*
   * Constructor will require task_id, priority, the task function to run and of course the arguments to pass to that function.
   */
  Task (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function, Args... function_args);

  /*
   * This operator cannot be "friend bool operator < ..." because then we can't define it as a const function.
   * And we need this operator to be const to tell the compiler that it will not change the class members- this
   * is required when Task objects are copied or moved (e.g. pushed or popped from lists or passed by reference I think).
   */
  bool operator < (const Task& other) const {
    return GetPriority() < other.GetPriority();
  }

  /*
   * This operator cannot be "friend bool operator > ..." because then we can't define it as a const function.
   * And we need this operator to be const to tell the compiler that it will not change the class members- this
   * is required when Task objects are copied or moved (e.g. pushed or popped from lists or passed by reference I think).
   */
  bool operator > (const Task& other) const {
    return GetPriority() > other.GetPriority();
  }

  /*
   * This function has to be const to allow the > and < operators to be const.
   */
  unsigned int GetPriority () const {return priority_;}
  unsigned int GetId () {return task_id_;}
  bool HasReturnValue () {return has_return_value_;}
  ReturnType GetResult () {return result_;}
  void RunTask ();

private:
  unsigned int task_id_;
  unsigned int priority_;
  std::function<ReturnType(Args...)> task_function_;
  /*
   * Alongside the task function and its returned result, we also need its parameters.
   */
  std::tuple<Args...> function_args_;
  ReturnType result_;
  bool has_return_value_;
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_TASK_H_
