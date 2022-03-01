#ifndef SHAREDQ_H_
#define SHAREDQ_H_

#include <boost/thread.hpp>
#include <queue>

namespace scanner {
    template<typename T>
    class shared_queue {
        public:
            boost::mutex mtx;
            std::queue<T> q;
            boost::condition_variable condition;


    void enqueue(T t) {
        boost::unique_lock<boost::mutex> lock(mtx);
        q.push(t);
        condition.notify_one();
    }

    T dequeue() {
        boost::unique_lock<boost::mutex> lock(mtx);

        while(q.size() <= 0) 
            condition.wait(lock);

        T t = q.front();
        q.pop();
        return t;
    }

    // bool dequeue_timed(T& t, int timeout) {
    //     boost::unique_lock<boost::mutex> lock(mtx);
    //     auto duration = boost::chrono::system_clock::now() + timeout;

    //     while(q.size() <= 0) {
    //     if (condition_.wait_until(lock, duration) == 
    //         boost::condition_variable::cv_status::timeout) return false;
    //     } 

    //     t = q.front();
    //     q.pop();
    //     return true;        
    // }

    void try_enqueue(T t) {
        boost::unique_lock<boost::mutex> lock(mtx, boost::try_to_lock);
        
        if(!lock.owns_lock()) return;

        q.push(t);
        condition.notify_one();
    }

    void try_dequeue(T& t) {
        boost::unique_lock<boost::mutex> lock(mtx, boost::try_to_lock);

        if(!lock.owns_lock() || q.size() <= 0) return;

        t = q.front();
        q.pop();
    }

    // bool try_dequeue(T& t, int timeout) {
    //     auto time=boost::get_system_time()+boost::posix_time::milliseconds(timeout);
    //     boost::mutex mtxx;
    //     boost::unique_lock<boost::mutex> lock(mtxx,time);

    //     if(!lock.owns_lock()) return false;

    //     while(q.size()<=0) if(!condition.timed_wait(lock,time)) return false;

    //     t = q.front();
    //     q.pop();

    //     return true;
    // }

    void clear() {
        boost::unique_lock<boost::mutex> lock(mtx);
    
        std::queue<T> empty;
        std::swap(q, empty);
    }

    // template<typename F>
    // void lock(F& fn) {
    //     boost::unique_lock<boost::mutex> lock(mtx);
    //     fn();
    //     condition.notify_one();
    // }

    };
}

#endif