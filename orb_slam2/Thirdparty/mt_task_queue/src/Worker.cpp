#include "Worker.h"

namespace TaskQueue {

template <typename ReturnType, typename... Args>
Worker<ReturnType, Args...>::Worker (std::priority_queue<Task<ReturnType, Args...>,
                                     std::vector<Task<ReturnType, Args...>>,
                                     std::greater<Task<ReturnType, Args...>>> &task_queue,
                std::map<unsigned int, Task<ReturnType, Args...>> &results, std::mutex &queue_mutex, std::mutex &map_mutex,
                std::condition_variable &condition_var)
                : task_queue_(task_queue), results_(results), queue_mutex_(queue_mutex), map_mutex_(map_mutex), condition_var_(condition_var) {
  end_operator_flag_ = false;
  Operator ();
}


template <typename ReturnType, typename... Args>
void Worker<ReturnType, Args...>::Operator () {
  std::unique_lock<std::mutex> lock (thread_mutex_);
  while (!end_operator_flag_) {
    condition_var_.wait(lock);

    queue_mutex_.lock();
    if (task_queue_.empty()) {
      continue;
    }

    queue_mutex_.lock();
    Task<ReturnType, Args...> top_task = task_queue_.top();
    task_queue_.pop();
    queue_mutex_.unlock();

    top_task.RunTask ();

    if (top_task.HasReturnValue()) {
      map_mutex_.lock();
      results_[top_task.GetId()] = top_task;
      map_mutex_.unlock();
    }
  }

}

} // namespace TaskQueue
