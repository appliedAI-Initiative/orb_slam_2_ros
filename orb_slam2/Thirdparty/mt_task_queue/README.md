# Multithreaded task queue (mt_task_queue)
## About
mt_task_queue is a small and simple task queue which supports mulithreading, result buffering and task priorization.
If you want to have the result stored for later use you need to set a task id.
With this id you can querry for the result, which will return a task if its done which holds the result.

## TODO
- Give the possibility to change priority of task once it is in the queue
- Add function to remove or add workers at runtime
- Make the length of the queue accessible
