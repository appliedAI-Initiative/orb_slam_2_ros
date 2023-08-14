/*
Small and simple task queue which supports mulithreading, result buffering and task priorization.
For more information see the README.
*/


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
    /*
     * In the constructor to Worker we must pass pointers to map_mutex, queue_mutex and the results buffer must contain pointers
     * to Task objects because otherwise these mutexes and Tasks would have to be copy-constructed, which is problematic.
     */
    Worker (std::priority_queue<Task<ReturnType, Args...>, std::vector<Task<ReturnType, Args...>>,
                                std::greater<Task<ReturnType, Args...>>> &task_queue,
            std::map<unsigned int, Task<ReturnType, Args...>*> &results, std::mutex* queue_mutex, std::mutex* map_mutex, std::condition_variable &condition_var);

    /*
     * We need a move constructor, because somewhere in our code we are moving Worker objects (returning them from some function probably).
     */
    Worker(Worker<ReturnType, Args...>&&);

    void EndWorker() {end_operator_flag_ = true;}
    bool IsIdeling () {return idle_;}

    ~Worker();


  private:
    std::priority_queue <Task<ReturnType, Args...>, std::vector<Task<ReturnType, Args...>>, std::greater<Task<ReturnType, Args...>>> &task_queue_;
    /*
     * The results_ buffer must contain pointers to Task objects.
     */
    std::map<unsigned int, Task<ReturnType, Args...>*> &results_;
    /*
     * Mutex objects need to be pointed at instead of referred to because otherwise we need to copy-construct them in this class constructor.
     */
    std::mutex* queue_mutex_;
    std::mutex* map_mutex_;
    std::mutex thread_mutex_;
    std::condition_variable &condition_var_;
    std::atomic<bool> end_operator_flag_;
    std::atomic<bool> idle_;
    std::thread worker_thread_;

    void Operator ();
};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_WORKER_H_
