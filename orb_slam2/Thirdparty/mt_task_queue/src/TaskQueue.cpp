#include "TaskQueue.h"

namespace TaskQueue {

template <typename ReturnType, typename... Args>
TaskQueue<ReturnType, Args...>::TaskQueue (unsigned int num_worker_threads) {
  if (num_worker_threads == 0) {
    throw std::invalid_argument ("Cannot launch threat queue with 0 workers");
  }

  for (int i : num_worker_threads) {
    workers_.push_back (Worker<ReturnType, Args...> (task_queue_, results_, queue_mutex_, map_mutex_, condition_var_));
  }
}


template <typename ReturnType, typename... Args>
TaskQueue<ReturnType, Args...>::~TaskQueue () {
  for (auto worker : workers_) {
    worker.EndWorker();
  }
}


template <typename ReturnType, typename... Args>
void TaskQueue<ReturnType, Args...>::AddTask (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function) {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  task_queue_.push(Task<ReturnType, Args...> (task_id, priority, task_function));
  condition_var_.notify_one();
}


template <typename ReturnType, typename... Args>
Task<ReturnType, Args...> TaskQueue<ReturnType, Args...>::GetTask (unsigned int task_id) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  auto it = results_.find (task_id);
  if (it != results_.end()) {
    return results_[task_id];
  } else {
    throw std::bad_exception ("Result not (yet?) available");
  }
}


template <typename ReturnType, typename... Args>
bool TaskQueue<ReturnType, Args...>::ResultAvailable (unsigned int task_id) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  auto it = results_.find (task_id);
  if (it != results_.end()) {
    return true;
  } else {
    return false;
  }
}


template <typename ReturnType, typename... Args>
bool TaskQueue<ReturnType, Args...>::QueueIsEmpty () {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  return task_queue_.empty();
}


template <typename ReturnType, typename... Args>
unsigned int TaskQueue<ReturnType, Args...>::NumJobsCurrentlyRunning () {
  unsigned int workers_working = 0;
  for (auto worker : workers_) {
    if (worker.IsIdeling()) {
      workers_working ++;
    }
  }
  return workers_working;
}

} // namespace TaskQueue
