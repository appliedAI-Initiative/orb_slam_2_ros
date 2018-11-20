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

namespace TaskQueue {

template <typename ReturnType, typename... Args>
class TaskQueue {
  public:
    TaskQueue (unsigned int num_worker_threads);
    ~TaskQueue ();

    void AddTask (std::function<ReturnType(Args...)> task_function) {AddTask (1, task_function);}
    void AddTask (unsigned int priority, std::function<ReturnType(Args...)> task_function) {AddTask (0, 1, task_function);}
    void AddTask (unsigned int task_id, unsigned int priority, std::function<ReturnType(Args...)> task_function);
    Task<ReturnType, Args...> GetTask (unsigned int task_id);
    bool ResultAvailable (unsigned int task_id);

  private:
    std::priority_queue <Task<ReturnType, Args...>, std::vector<Task<ReturnType, Args...>>, std::greater<Task<ReturnType, Args...>>> task_queue_;
    std::vector<Worker<ReturnType, Args...>> workers_;
    std::map<unsigned int, Task<ReturnType, Args...>> results_;
    std::mutex queue_mutex_;
    std::mutex map_mutex_;
    std::condition_variable condition_var_;

    bool QueueIsEmpty ();
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_TASKQUEUE_H_
