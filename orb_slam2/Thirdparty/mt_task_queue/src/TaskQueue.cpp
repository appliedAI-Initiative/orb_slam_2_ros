#include "TaskQueue.h"

namespace TaskQueue {

TaskQueue::TaskQueue (unsigned int num_worker_threads) {
  if (num_worker_threads == 0) {
    throw std::invalid_argument ("Cannot launch threat queue with 0 workers");
  }

  end_runners_flag_ = false;

  for (int i : num_worker_threads) {
    worker_threads_.push_back (Worker (task_queue_, results_, queue_mutex_, map_mutex_, condition_var_));
  }
}


TaskQueue::~TaskQueue () {
  end_runners_flag_ = true;
  for (int i : num_worker_threads) {
    worker_threads_.EndWorker();
  }
}


void TaskQueue::AddTask (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function) {
  unique_lock<mutex> lock(queue_mutex_);
  task_queue_.push(Task (task_id, priority, task_function));
  condition_var_.notify_one();
}


Task TaskQueue::GetTask (unsigned int task_id) {
  unique_lock<mutex> lock(map_mutex_);
  std::map<unsigned int, Task>::const_iterator it = results_.find (task_id);
  if (it != results_.end()) {
    return results_[task_id];
  } else {
    throw std::bad_exception ("Result not (yet?) available");
  }
}


bool TaskQueue::ResultAvailable (unsigned int task_id) {
  unique_lock<mutex> lock(map_mutex_);
  std::map<unsigned int, Task>::const_iterator it = results_.find (task_id);
  if (it != results_.end()) {
    return true;
  } else {
    return false;
  }
}


bool TaskQueue::QueueIsEmpty () {
  unique_lock<mutex> lock(queue_mutex_);
  return task_queue_.empty();
}

} // namespace TaskQueue
