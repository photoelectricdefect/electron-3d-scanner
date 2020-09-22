#ifndef EVENT_H_
#define EVENT_H_

namespace scanner {
    template<typename T>
    class event {
        public:
            T t;
            event(T _t = T()) : t(_t) {}
    };
}

#endif