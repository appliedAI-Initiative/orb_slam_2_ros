/*
Small and simple task queue which supports mulithreading, result buffering and task priorization.
If you want to get a result from a task, you need to set a task id.
*/


#ifndef MT_TASK_QUEUE_TASKQUEUE_H_
#define MT_TASK_QUEUE_TASKQUEUE_H_

#include <dequeue>
#include <functional>
#include <vector>
#include <map>
#include <threat>
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
    Task GetTask (unsigned int task_id);
    bool ResultAvailable (unsigned int task_id);

  private:
    std::priority_queue<Task, std::vector<Task>, greater<vector<Task>::value_type>> task_queue_;
    std::vector<Worker> workers_;
    std::map<unsigned int, Task> results_;
    std::mutex queue_mutex_;
    std::mutex map_mutex_;
    std::condition_variable condition_var_;

    bool QueueIsEmpty ();
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_TASKQUEUE_H_
