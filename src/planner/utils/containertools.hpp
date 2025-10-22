
#pragma once

#include <array>
#include <deque>
#include <forward_list>
#include <list>
#include <valarray>
#include <vector>

template<typename S>
struct container_adaptor_for;

template<class S> // wrapped source type
class Container {
	typedef typename container_adaptor_for<S>::type Adaptor;
	typename Adaptor::iterator b;
	typename Adaptor::iterator e;

public:

	explicit Container(const S& source) {
		b = Adaptor::begin(source);
		e = Adaptor::end(source);
	}

	typename Adaptor::iterator begin() const { return b; }
	typename Adaptor::iterator end() const { return e; }
	size_t size() const { return e - b; }

	template<typename T> // casting target type
	operator T() {
		typedef typename container_adaptor_for<T>::type TargetAdaptor;
		return TargetAdaptor::fromContainer(*this);
	}
};




// struct ContainerAdaptor_Eigen_Vector4f {
// 	typedef Eigen::Vector4f type;
// 	typedef const float* iterator;
// 	static iterator begin(const Eigen::Vector4f& v) { return &v[0]; }
// 	static iterator end(const Eigen::Vector4f& v) { return &v[4]; }
// 	template<typename S>
// 	static Eigen::Vector4f fromContainer(const Container<S>& container) {
// 		Eigen::Vector4f result;
// 		size_t i = 0;
// 		for (const auto& val : container) {
// 			result[i++] = val;
// 		}
// 		return result;
// 	}
// };
// template<>
// struct container_adaptor_for<Eigen::Vector4f> {
// 	typedef ContainerAdaptor_Eigen_Vector4f type;
// };

// struct ContainerAdaptor_btVector4 {
// 	typedef btVector4 type;
// 	typedef const btScalar* iterator;
// 	static iterator begin(const btVector4& v) { return &v[0]; }
// 	static iterator end(const btVector4& v) { return &v[4]; }

// 	template<typename T>
// 	static btVector4 fromContainer(const Container<T>& container) {
// 		btVector4 result;
// 		size_t i = 0;
// 		for (const auto& val : container) {
// 			result[i++] = val;
// 		}
// 		return result;
// 	}
// };
// template<> struct container_adaptor_for<btVector4> { typedef ContainerAdaptor_btVector4 type; };
