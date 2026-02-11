#pragma once

#include <coroutine>
#include <queue>
#include <optional>
#include <chrono>
#include <thread>
#include <utility>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>


namespace kmx::aether::v0_1
{
    // Forward declaration
    class scheduler;

    template<typename T>
    struct task;

    namespace detail
    {
        struct promise_base
        {
            std::coroutine_handle<> continuation;
        };

        template<typename T>
        struct promise_type_t : promise_base
        {
            T value;

            task<T> get_return_object();

            std::suspend_always initial_suspend() { return {}; }

            struct final_awaitable
            {
                bool await_ready() const noexcept { return false; }
                std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type_t> h) noexcept
                {
                    auto& promise = h.promise();
                    if (promise.continuation)
                        return promise.continuation;
                    return std::noop_coroutine();
                }
                void await_resume() const noexcept {}
            };

            final_awaitable final_suspend() noexcept { return {}; }

            void unhandled_exception() { std::terminate(); }

            template<typename U>
            requires std::convertible_to<U, T>
            void return_value(U&& v) { value = std::forward<U>(v); }
        };

        template<>
        struct promise_type_t<void> : promise_base
        {
            task<void> get_return_object();

            std::suspend_always initial_suspend() { return {}; }

            struct final_awaitable
            {
                bool await_ready() const noexcept { return false; }
                std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type_t> h) noexcept
                {
                    auto& promise = h.promise();
                    if (promise.continuation)
                        return promise.continuation;
                    return std::noop_coroutine();
                }
                void await_resume() const noexcept {}
            };

            final_awaitable final_suspend() noexcept { return {}; }

            void unhandled_exception() { std::terminate(); }

            void return_void() {}
        };
    }

    template<typename T = void>
    struct task
    {
        using promise_type = detail::promise_type_t<T>;
        using handle_type = std::coroutine_handle<promise_type>;

        handle_type handle;

        task(handle_type h) : handle(h) {}
        task(task&& other) noexcept : handle(other.handle) { other.handle = nullptr; }
        task(const task&) = delete;
        ~task() { if (handle) handle.destroy(); }

        bool await_ready() { return false; }

        std::coroutine_handle<> await_suspend(std::coroutine_handle<> h)
        {
            handle.promise().continuation = h;
            return handle;
        }

        T await_resume()
        {
            if constexpr (!std::is_void_v<T>)
                return std::move(handle.promise().value);
        }
    };

    namespace detail
    {
        template<typename T>
        task<T> promise_type_t<T>::get_return_object()
        {
            return task<T>{std::coroutine_handle<promise_type_t<T>>::from_promise(*this)};
        }

        inline task<void> promise_type_t<void>::get_return_object()
        {
            return task<void>{std::coroutine_handle<promise_type_t<void>>::from_promise(*this)};
        }
    }

    class scheduler
    {
    public:
        using service_tag = void;

    private:
        std::mutex _q_mutex;
        std::condition_variable _cv;
        std::queue<std::coroutine_handle<>> _tasks;
        std::vector<std::thread> _workers;
        std::atomic<bool> _stop = false;

    public:
        void schedule(std::coroutine_handle<> h)
        {
            {
                std::lock_guard<std::mutex> lock(_q_mutex);
                _tasks.push(h);
            }
            _cv.notify_one();
        }

        void run(size_t threads = 4)
        {
            for (size_t i = 0; i < threads; ++i)
            {
                _workers.emplace_back([this]
                {
                    while (true)
                    {
                        std::unique_lock<std::mutex> lock(_q_mutex);
                        _cv.wait(lock, [this] { return _stop || !_tasks.empty(); });

                        if (_stop && _tasks.empty())
                            return;

                        if (_tasks.empty())
                            continue;

                        auto h = _tasks.front();
                        _tasks.pop();
                        lock.unlock();

                        if (h && !h.done())
                            h.resume();
                    }
                });
            }

            for (auto& t : _workers)
            {
                if (t.joinable())
                    t.join();
            }
        }

        void stop()
        {
            _stop = true;
            _cv.notify_all();
        }
    };

    struct sleep_stub
    {
        scheduler* _sched;

        sleep_stub(scheduler& s) : _sched(&s) {}

        bool await_ready() const { return false; }

        void await_suspend(std::coroutine_handle<> h)
        {
             _sched->schedule(h);
        }

        void await_resume() const {}
    };
}
