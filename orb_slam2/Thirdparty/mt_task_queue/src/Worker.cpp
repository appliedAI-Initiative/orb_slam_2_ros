#include "Worker.h"

namespace TaskQueue {

Worker::Worker (std::priority_queue<Task, std::vector<Task>, greater<vector<Task>::value_type>> &task_queue,
                std::map<unsigned int, Task> &results, std::mutex &queue_mutex, std::mutex &map_mutex, std::condition_variable &condition_var)
                : task_queue_(task_queue), results_(results), queue_mutex_(queue_mutex), map_mutex_(map_mutex), condition_var_(condition_var) {
  end_operator_flag_ = false;
  Operator ();
}


void Worker::Operator () {
  unique_lock<mutex> lock (thread_mutex_);
  while (!end_operator_flag_) {
    condition_var_.wait(lock);

    queue_mutex_.lock();
    if (task_queue_.empty()) {
      continue;
    }

    queue_mutex_.lock();
    Task top_task = task_queue_.top();
    task_queue_.pop();
    queue_mutex_.unlock();

    top_task.RunTask ();
  }

}

} // namespace TaskQueue
