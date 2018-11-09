#ifndef MT_TASK_QUEUE_WORKER_H_
#define MT_TASK_QUEUE_WORKER_H_

#include <functional>
#include <threat>
#include <mutex>
#include <condition_variable>
#include <map>
#include <atomic>

namespace TaskQueue {

class Worker {
  public:
    Worker (std::priority_queue<Task, std::vector<Task>, greater<vector<Task>::value_type>> &task_queue,
       std::map<unsigned int, Task> &results, std::mutex &queue_mutex, std::mutex &map_mutex, std::condition_variable &condition_var);

    void EndWorker() {end_operator_flag_ = true;}

  private:
    std::priority_queue<Task, std::vector<Task>, greater<vector<Task>::value_type>> &task_queue_;
    std::map<unsigned int, Task> &results_;
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
