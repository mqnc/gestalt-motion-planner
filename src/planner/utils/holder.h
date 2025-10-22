#pragma once

// We cannot store abstract objects by value (e.g. in containers),
// so we have to use pointers. We can almost use std::unique_ptr,
// except sometimes we also want to deep-copy the objects around.
// Holder provides this functionality on top of std::unique_ptr.
// The held object must have a clone() method returning a holder
// of its own type.

// I was apparently not the first to have this problem:
// https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2012/n3339.pdf
// simbody implementation: https://github.com/simbody/simbody/blob/c8e14d2708c1be3fdc7c4a7e67ad66d615bd4906/SimTKcommon/include/SimTKcommon/internal/ClonePtr.h
// drake implementation: https://github.com/RobotLocomotion/drake/blob/f3c36f7e8a5012cd2adfab0a3d9760144d6dde7c/common/copyable_unique_ptr.h
// https://github.com/LoopPerfect/valuable
// https://github.com/martinmoene/value-ptr-lite

#include <memory>
#include <type_traits>

// https://stackoverflow.com/a/23537056/3825996

template <typename T>
struct Holder : public std::unique_ptr<T, std::default_delete<T>> {
	
	// THIS CLASS MUST NOT HAVE FIELDS
	// otherwise we risk slicing

	// inherit constructors
    using std::unique_ptr<T, std::default_delete<T>>::unique_ptr;

    Holder() {}

    // Copy constructor (deep copy)
    Holder(const Holder<T>& other) {
        this->reset(other ? other->clone().release() : nullptr);
    }

	// Copy assignment (deep copy) from derived class
	template <typename U, typename = std::enable_if_t<std::is_base_of<T, U>::value>>
    Holder<T>& operator=(const Holder<U>& other) {
        if (this->get() != other.get()) {
            this->reset(other ? other->clone().release() : nullptr);
        }
        return *this;
    }

    // Move constructor
    Holder(Holder<T>&& other) noexcept
        : std::unique_ptr<T, std::default_delete<T>>(std::move(other)) {}

    // Move assignment
    Holder<T>& operator=(Holder<T>&& other) noexcept {
        std::unique_ptr<T, std::default_delete<T>>::operator=(std::move(other));
        return *this;
    }
};


template<typename T, typename... Args>
Holder<T> hold(Args&&... args) {
	return Holder<T>(new T(std::forward<Args>(args)...));
}


// we can't use the copy constructor or operator= because we would slice the object
// we must call a method on the cloned object so v-table magic can happen

// // https://codereview.stackexchange.com/a/103804l

// template <typename T>
// struct Holder : public std::unique_ptr<T> {
// 	using std::unique_ptr<T>::unique_ptr;
// 	Holder() {};
// 	Holder(Holder<T> const& other) {
// 		auto value = *other.get();
// 		this->reset(new T(value));
// 	}
// 	Holder<T>& operator=(Holder<T> const& other) {
// 		auto value = *other.get();
// 		this->reset(new T(value));
// 		return *this;
// 	}
// };