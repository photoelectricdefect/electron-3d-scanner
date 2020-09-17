#ifndef THREADSAFEQ_H_
#define THREADSAFEQ_H_

#include <boost/thread.hpp>
#include <queue>

namespace scanner {
    template<typename T>
    class threadsafe_queue {
        private:
            std::queue<T> _q;
            boost::mutex _mutex;
            boost::condition_variable _condition;
        public:
    void enqueue(T t) {
        boost::unique_lock<boost::mutex> lock(_mutex);
        _q.push(t);
        _condition.notify_one();
    }

    T dequeue() {
        boost::unique_lock<boost::mutex> lock(_mutex);
        std::cout << "dequeing..." << std::endl;
        while(_q.size() <= 0) _condition.wait(lock);

        T t = _q.front();
        _q.pop();
        return t;
    }

    void try_enqueue(T t) {
        boost::unique_lock<boost::mutex> lock(_mutex, boost::try_to_lock);
        
        if(!lock.owns_lock()) return;

        _q.push(t);
        _condition.notify_one();
    }

    void try_dequeue(T& t) {
        boost::unique_lock<boost::mutex> lock(_mutex, boost::try_to_lock);

        if(!lock.owns_lock() || _q.size() <= 0) return;

        t = _q.front();
        _q.pop();
        return t;
    }

    void clear() {
        boost::unique_lock<boost::mutex> lock(_mutex);
    
        std::queue<T> empty;
        std::swap(_q, empty);
    }

    void lockq() {
        _mutex.lock();       
    }

    void unlockq() {
        _mutex.unlock();       
    }
    };
}

#endif