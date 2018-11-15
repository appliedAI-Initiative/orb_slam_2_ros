#ifndef MT_TASK_QUEUE_WORKER_H_
#define MT_TASK_QUEUE_WORKER_H_

#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>
#include <atomic>
#include <queue>
#include <vector>

#include "Task.h"

namespace TaskQueue {

template <typename ReturnType, typename... Args>
class Worker {
  public:
    Worker (std::priority_queue<Task<ReturnType, Args...>, std::vector<Task<ReturnType, Args...>>,
                                std::greater<Task<ReturnType, Args...>>> &task_queue,
            std::map<unsigned int, Task<ReturnType, Args...>> &results, std::mutex &queue_mutex, std::mutex &map_mutex, std::condition_variable &condition_var);

    void EndWorker() {end_operator_flag_ = true;}

  private:
    std::priority_queue <Task<ReturnType, Args...>, std::vector<Task<ReturnType, Args...>>, std::greater<Task<ReturnType, Args...>>> &task_queue_;
    std::map<unsigned int, Task<ReturnType, Args...>> &results_;
    std::mutex &queue_mutex_;
    std::mutex &map_mutex_;
    std::mutex thread_mutex_;
    std::condition_variable &condition_var_;
    std::atomic<bool> end_operator_flag_;
    std::thread worker_thread_;

    void Operator ();
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_WORKER_H_
