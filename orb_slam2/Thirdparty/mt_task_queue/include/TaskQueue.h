/*
Small and simple task queue which supports mulithreading, result buffering and task priorization.
For more information see the README.
*/


#ifndef MT_TASK_QUEUE_TASKQUEUE_H_
#define MT_TASK_QUEUE_TASKQUEUE_H_

#include <deque>
#include <functional>
#include <vector>
#include <map>
#include <thread>
#include <stdexcept>
#include <mutex>
#include <condition_variable>

#include "Task.h"
#include "Worker.h"

// required for explicit template instantiation in TaskQueue.cpp
#include <boost/any.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<opencv2/core/core.hpp>

namespace TaskQueue {

template <typename ReturnType, typename... Args>
class TaskQueue {
  public:
    TaskQueue (unsigned int num_worker_threads);
    ~TaskQueue ();

    /*
     * AddTask function as a bare minimum requires the task function alongside its arguments. Priority and task_id need to be given
     * or can be defaulted.
     */
    void AddTask (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function, Args... function_args);
    void AddTask (unsigned int priority, std::function<ReturnType(Args...)> task_function, Args... function_args) {
        AddTask(0, priority, task_function, function_args...);
    }
    void AddTask (std::function<ReturnType(Args...)> task_function, Args... function_args) {
        AddTask(1, 0, task_function, function_args...);
    }

    /*
     * The task that we return has to be a pointer to the Task instance because otherwise a copy construction of Task  is required
     * when we call this function and copy construction of task can be problematic due to atomic functions and thread locks inside it,
     * which can't be easily copied.
     */
    Task<ReturnType, Args...>* GetTask (unsigned int task_id);
    bool ResultAvailable (unsigned int task_id);
    unsigned int NumJobsCurrentlyRunning ();

  private:
    std::priority_queue <Task<ReturnType, Args...>, std::vector<Task<ReturnType, Args...>>, std::greater<Task<ReturnType, Args...>>> task_queue_;
    std::vector<Worker<ReturnType, Args...>*> workers_;
    /*
     * The results_ buffer has to contain pointers to Task objects because our GetTask function now returns a pointer rather than a Task object.
     */
    std::map<unsigned int, Task<ReturnType, Args...>*> results_;
    std::mutex queue_mutex_;
    std::mutex map_mutex_;
    std::condition_variable condition_var_;

    bool QueueIsEmpty ();
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_TASKQUEUE_H_
