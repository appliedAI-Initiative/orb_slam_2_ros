#include "Worker.h"

namespace TaskQueue {

template <typename ReturnType, typename... Args>
/*
 * Arguments to constructor now contain mutex pointers because mutexes are problematic to copy. results map also
 * now contains pointers to Task objects rather than objects themselves- to avoid unnecessary copy-construction.
 */
Worker<ReturnType, Args...>::Worker (std::priority_queue<Task<ReturnType, Args...>,
                                     std::vector<Task<ReturnType, Args...>>,
                                     std::greater<Task<ReturnType, Args...>>> &task_queue,
                std::map<unsigned int, Task<ReturnType, Args...>*> &results, std::mutex* queue_mutex, std::mutex* map_mutex,
                std::condition_variable &condition_var)
                : task_queue_(task_queue), results_(results), condition_var_(condition_var) {
  end_operator_flag_ = false;
  idle_ = true;
  queue_mutex_ = queue_mutex;
  map_mutex_ = map_mutex;
  Operator ();
}

/**
 * Move constructor to deal with the mutexes that can't be copied. 
 *
 * Really we should not execute this until we're sure that Operator () has completed execution. So really either
 * throw an exception or block if other.idle_ == false or if the mutexes are locked. But again, since we are only
 * creating 1 Worker for now, I'll let this slide.
 */
template <typename ReturnType, typename... Args>
Worker<ReturnType, Args...>::Worker(Worker<ReturnType, Args...>&& other) : task_queue_(other.task_queue_), results_(other.results_), condition_var_(other.condition_var_) {
    // mutexes can be just assigned as they are only pointers
    queue_mutex_ = other.queue_mutex_;
    map_mutex_ = other.map_mutex_;
    // thread_mutex_ will be created brand new just as in the normal constructor. That one is only used to acquire unique_lock in the Operator ()
    
    /*
     * atomic<bool> fields cannot be assigned from existing atomic<bool> field because that would cause copy-construction and atomic types do not like
     * to be copy-constructed. Therefore we use the load() function to get their bool value.
     */
    end_operator_flag_ = other.end_operator_flag_.load();
    idle_ = other.idle_.load();
    // worker_thread_ will also be created brand new- not sure what it is used for, I think it is not used, but we'll deal with that later. That happens automatically.
}

template <typename ReturnType, typename... Args>
void Worker<ReturnType, Args...>::Operator () {
  std::unique_lock<std::mutex> lock (thread_mutex_);
  while (!end_operator_flag_) {
    idle_ = true;
    condition_var_.wait(lock);

    /*
     * Since mutexes are now passed as pointers, we need pointers' access operator -> .
     */
    queue_mutex_->lock();
    if (task_queue_.empty()) {
      queue_mutex_->unlock();
      continue;
    }
    idle_ = false;
    
    Task<ReturnType, Args...> top_task = task_queue_.top();
    task_queue_.pop();

    queue_mutex_->unlock();

    top_task.RunTask ();

    if (top_task.HasReturnValue()) {
      /*
       * Since mutexes are now passed as pointers, we need pointers' access operator -> .
       */
      map_mutex_->lock();
      results_[top_task.GetId()] = &top_task;
      map_mutex_->unlock();
    }
  }

}

// Explicit template instantiation for the supported types. This is needed so that the rest of the project can link to this module. If this is not here, then support for the
// required types will not be generated by the compiler. This is what template <typename ReturnType, typename... Args> does for the Worker object.
template class Worker<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>, cv::Mat>;

} // namespace TaskQueue
