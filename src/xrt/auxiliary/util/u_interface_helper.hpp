// Copyright 2019-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Helpers for implementing Monado's "C-style classes/interfaces" using C++
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup aux_util
 */

#pragma once

#include <memory>
#include <type_traits>
#include <utility>

namespace xrt::auxiliary::util {


/*!
 * This is a struct template that help you implement Monado interfaces with C++ classes.
 *
 * @tparam T Your implementation
 *
 * This type exists to:
 * - contain an initial member of the interface structure type (required by Monado)
 * - own your heap-allocated C++ implementation object
 * - be a standard-layout type (required by the casting needed to make use of the first item)
 *
 * Unfortunately we cannot ergonomically just inherit from the Monado interface types directly, or include it as our
 * first element because of the third job of the wrapper type: standard layout types can't have non-static data members
 * as well as a base with non-static data members, as well a number of other widely-useful C++ features.
 *
 * Your type T must have `using interface_type = xyz;` as a public type alias for some interface structure type `xyz`.
 *
 */
template <typename T> struct InterfaceImplWrapper
{
	using interface_type = typename T::interface_type;
	interface_type base;
	T *impl;

	using impl_obj_type = T;
	using wrapper_type = InterfaceImplWrapper<T>;

	static_assert(!std::is_pointer<T>::value, "Implementation type T should be a class/struct type, not a pointer");
	static_assert(!std::is_reference<T>::value,
	              "Implementation type T should be a class/struct type, not a reference");
	static_assert(!std::is_const<T>::value, "Implementation type T should not be const-qualified");
	static_assert(!std::is_volatile<T>::value, "Implementation type T should not be volatile-qualified");
	/*!
	 * Get the implementation object from an interface pointer that is known to refer to an interface implementation
	 * created with these helpers and with these types.
	 *
	 * This is an unsafe call in general: make sure you only call this when you know the "real" type, as this
	 * performs `reinterpret_cast`! Use in implementing interface function pointer trampolines is safe, however.
	 */
	static inline impl_obj_type &
	getObj(interface_type *base_ptr) noexcept
	{
		return getTypedWrapper(base_ptr)->getObj();
	}

	/*!
	 * Get the implementation object from a wrapper reference.
	 *
	 * This is a safe non-static member call because the wrapper object is fully typed.
	 */
	impl_obj_type &
	getObj() const noexcept
	{
		return *impl;
	}

	/*!
	 * Get the interface pointer from a wrapper reference.
	 *
	 * This is a safe non-static member call because the wrapper object is fully typed.
	 */
	interface_type *
	getInterface() noexcept
	{
		return &base;
	}

	/**
	 * Destroys a wrapper (owned via an interface pointer), checking for nulls.
	 *
	 * @param base_ptr An interface pointer. If not null, it will be
	 * treated as a pointer to wrapper and correctly destroyed.
	 *
	 * Generally you will set the base (interface) `destroy` function pointer to this function or to @ref
	 * destroyClearing depending on the interface.
	 *
	 * You might use this function on its own if you happen to own a wrapper by raw interface pointer, but this is
	 * usually a code smell unless a C interface expects it. Use @ref unique_impl_wrapper instead, preferably.
	 *
	 * This is an unsafe call in general: make sure you only call this when you know the "real" type, as this
	 * performs `reinterpret_cast`! Setting an interface function pointer to this is safe, however.
	 */
	static void
	destroy(interface_type *base_ptr)
	{
		if (base_ptr == nullptr) {
			return;
		}
		auto *wrapper = getTypedWrapper(base_ptr);
		if (wrapper->impl != nullptr) {
			delete wrapper->impl;
			wrapper->impl = nullptr;
		}
		delete wrapper;
	}

	/**
	 * Destroys a wrapper (owned via an interface pointer), checking for nulls and clearing the pointer storage for
	 * the caller.
	 *
	 * @param base_ptr_storage The address of an interface pointer. If the pointer is not null, it will be
	 * destroyed, and the pointer will be set to null.
	 *
	 * Generally you will set the base (interface) `destroy` function pointer to this function or to @ref
	 * destroy depending on the interface.
	 *
	 * This is an unsafe call in general: make sure you only call this when you know the "real" type, as this
	 * performs `reinterpret_cast`! Setting an interface function pointer to this is safe, however.
	 */
	static void
	destroyClearing(interface_type **base_ptr_storage)
	{
		if (base_ptr_storage == nullptr) {
			return;
		}
		interface_type *base_ptr = *base_ptr_storage;
		if (base_ptr == nullptr) {
			return;
		}
		destroy(base_ptr);
		*base_ptr_storage = nullptr;
	}

private:
	/**
	 * Helper to do the `reinterpret_cast` right.
	 *
	 * Anything that's calling this is inherently unsafe by default!
	 *
	 * @param base_ptr
	 * @return wrapper_type*
	 */
	static inline wrapper_type *
	getTypedWrapper(interface_type *base_ptr) noexcept
	{
		return reinterpret_cast<wrapper_type *>(base_ptr);
	}
};

namespace deleters {
	/**
	 * @brief Custom deleter for @ref InterfaceImplWrapper
	 */
	template <typename T> struct WrapperDeleter
	{
		void
		operator()(InterfaceImplWrapper<T> *wrapper) const
		{
			InterfaceImplWrapper<T>::destroy(wrapper->getInterface());
		}
	};
} // namespace deleters

/**
 * The type of a unique_ptr that can be used to uniquely own a @ref InterfaceImplWrapper type.
 *
 * Needs the custom deleter because the wrapper type owns a resource, but can't have a destructor that releases
 * the resource or it would no longer be a standard-layout type.
 */
template <typename T> using unique_impl_wrapper = std::unique_ptr<InterfaceImplWrapper<T>, deleters::WrapperDeleter<T>>;

/**
 * Takes ownership of a C++-owned unique_impl_wrapper, and returns an interface pointer that owns it instead, for use
 * passing to C code that expects to take ownership.
 *
 * @param wrapper Uniqely-owned wrapper
 * @return interface_type*
 *
 * Will return null if you pass in null.
 */
template <typename T>
static typename T::interface_type *
releaseWrapperAsInterface(unique_impl_wrapper<T> &&wrapper)
{
	static_assert(std::is_standard_layout<InterfaceImplWrapper<T>>::value,
	              "Must be standard layout for use with Monado's C-style inheritance");
	if (!wrapper) {
		return nullptr;
	}
	return wrapper.release()->getInterface();
}
/*!
 * Helper for creating implementations of Monado interfaces using C++ classes: wraps your object in a wrapper containing
 * the base interface structure.
 *
 * @note Your implementation type T must define a public type alias `using interface_type = xyz;` for some Monado
 * interface type `xyz`, so that this function and @ref InterfaceImplWrapper knows the right interface type to use.
 * @param obj A unique_ptr to the implementation object type.
 *
 * Typically this function will be wrapped by one specific to an interface type. It will probably look something like
 * this:
 *
 * ```
 * static_assert(std::is_same<typename T::interface_type, InterfaceType>::value,
 *               "T must have interface_type alias for InterfaceType.");
 * using wrapper = InterfaceImplWrapper<T>;
 * auto ret = makeUniqueImplWrapper(std::move(implementation));
 * auto &base = ret->base;
 *
 * base.destroy = wrapper::destroy; // or wrapper::destroyClearing
 *
 * // and lots of lines like this
 * base.method_name = [](InterfaceType* ptr, ...){ wrapper::getObj(ptr).methodName(...); };
 *
 * return ret;
 * ```
 */
template <typename T>
static unique_impl_wrapper<T>
makeUniqueImplWrapper(std::unique_ptr<T> &&obj)
{
	static_assert(std::is_standard_layout<InterfaceImplWrapper<T>>::value,
	              "Must be standard layout for use with Monado's C-style inheritance");
	unique_impl_wrapper<T> ret{new InterfaceImplWrapper<T>{}};
	// U_ZERO(ret.get());
	ret->base = {};
	ret->impl = obj.release();

	return ret;
}


} // namespace xrt::auxiliary::util
