#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <vector>
#include <iostream>
#include <mutex>
#include <thread>

template<class T>
class RingBuffer
{
    public:
        RingBuffer(size_t size)
        : max_size_(size),
          current_size_(0),
          full_(false)
        {
        }

        // Copy constructor
        RingBuffer(const RingBuffer& r)
        {
            max_size_ = r.max_size_;
            current_size_ = r.current_size_;
            full_ = r.full_;
            data_ = r.data_;
        }



        //Add newest value to the buffer
        void push_back(T data)
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if(!full_)
            {
                data_.push_back(data);
                current_size_++;
            }
            else
            {
                pop_front();
                data_.push_back(data);
                current_size_ = max_size_;
            }

            full_ = current_size_ == max_size_;
        }

        // Read oldest value
        T get()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            T data;

            if(current_size_ != 0)
            {
                data = data_.front();
                pop_front();
            }    

            full_ = false;
            return data;
        }

        T& end()
        {
            return *(data_.end() - 1);
        }

        std::size_t size() const
        {
            return data_.size();
        }

        bool empty() const
        {
            //if head and tail are equal, we are empty
	        return (!full_ && (current_size_ == max_size_));
        }

        bool full() const
        {
            return full_;
        }

        void reset()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_size_ = 0;
            full_ = false;
            data_.clear();
        }

    private:

        void pop_front()
        {
            data_.erase(data_.begin());
        }

        std::mutex mutex_;
        std::vector<T> data_;
        size_t current_size_;
        size_t max_size_;
        bool full_;
};



#endif