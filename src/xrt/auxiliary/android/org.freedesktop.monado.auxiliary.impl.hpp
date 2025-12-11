// Copyright 2020-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Inline implementations for partially-generated wrapper for the
 * `org.freedesktop.monado.auxiliary` Java package - do not include on its own!
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup aux_android
 */

#pragma once

#include "math/m_api.h"

#include "wrap/android.app.h"
#include "wrap/android.content.h"
#include "wrap/android.view.h"
#include <vector>


namespace wrap {
namespace org::freedesktop::monado::auxiliary {
	inline MonadoView
	MonadoView::attachToWindow(android::content::Context const &displayContext,
	                           void *nativePointer,
	                           wrap::android::view::WindowManager_LayoutParams const &lp)
	{
		return MonadoView(Meta::data().clazz().call<jni::Object>(
		    Meta::data().attachToWindow, displayContext.object(),
		    static_cast<long long>(reinterpret_cast<uintptr_t>(nativePointer)), lp.object()));
	}

	inline void
	MonadoView::removeFromWindow(const MonadoView &view)
	{
		Meta::data().clazz().call<void>(Meta::data().removeFromWindow, view.object());
	}

	inline jni::Object
	MonadoView::getDisplayMetrics(android::content::Context const &context)
	{
		return Meta::data().clazz().call<jni::Object>(Meta::data().getDisplayMetrics, context.object());
	}

	inline float
	MonadoView::getDisplayRefreshRate(android::content::Context const &context)
	{
		return Meta::data().clazz().call<float>(Meta::data().getDisplayRefreshRate, context.object());
	}

	inline int32_t
	MonadoView::getDisplayModeIdWidth(const android::content::Context &displayContext,
	                                  int32_t displayId,
	                                  int32_t displayModeId)
	{
		return Meta::data().clazz().call<int32_t>(Meta::data().getDisplayModeIdWidth, displayContext.object(),
		                                          displayId, displayModeId);
	}

	inline int32_t
	MonadoView::getDisplayModeIdHeight(const android::content::Context &displayContext,
	                                   int32_t displayId,
	                                   int32_t displayModeId)
	{
		return Meta::data().clazz().call<int32_t>(Meta::data().getDisplayModeIdHeight, displayContext.object(),
		                                          displayId, displayModeId);
	}

	inline std::vector<float>
	MonadoView::getSupportedRefreshRates(android::content::Context const &context)
	{
		jni::Object refreshRateArray =
		    Meta::data().clazz().call<jni::Object>(Meta::data().getSupportedRefreshRates, context.object());
		jfloat *refreshRates =
		    (jfloat *)jni::env()->GetFloatArrayElements((jfloatArray)refreshRateArray.getHandle(), 0);
		jsize length = jni::env()->GetArrayLength((jfloatArray)refreshRateArray.getHandle());
		std::vector<float> refreshRateVector;
		for (int i = 0; i < length; i++) {
			refreshRateVector.push_back(refreshRates[i]);
		}
		return refreshRateVector;
	}

	inline void *
	MonadoView::getNativePointer()
	{
		assert(!isNull());
		return reinterpret_cast<void *>(
		    static_cast<intptr_t>(object().call<long long>(Meta::data().getNativePointer)));
	}

	inline void
	MonadoView::markAsDiscardedByNative()
	{
		assert(!isNull());
		return object().call<void>(Meta::data().markAsDiscardedByNative);
	}

	inline android::view::SurfaceHolder
	MonadoView::waitGetSurfaceHolder(int32_t wait_ms)
	{
		assert(!isNull());
		return android::view::SurfaceHolder(
		    object().call<jni::Object>(Meta::data().waitGetSurfaceHolder, wait_ms));
	}

	inline ActivityLifecycleListener
	ActivityLifecycleListener::construct(void *nativePointer)
	{
		return ActivityLifecycleListener(Meta::data().clazz().newInstance(
		    Meta::data().init, static_cast<long long>(reinterpret_cast<uintptr_t>(nativePointer))));
	}

	inline void
	ActivityLifecycleListener::registerCallback(android::app::Activity const &activity)
	{
		object().call<void>(Meta::data().registerCallback, activity.object());
	}

	inline void
	ActivityLifecycleListener::unregisterCallback(android::app::Activity const &activity)
	{
		object().call<void>(Meta::data().unregisterCallback, activity.object());
	}

	inline int32_t
	ReadResult::getStatus() const
	{
		assert(!isNull());
		return java::lang::Integer(object().get<jni::Object>(Meta::data().second)).intValue();
	}

	inline bool
	ReadResult::getData(std::span<uint8_t> outData) const
	{
		assert(!isNull());
		if (outData.empty()) {
			return false;
		}

		const auto byteArrayObj = object().get<jni::Object>(Meta::data().first);
		if (byteArrayObj.isNull()) {
			return false;
		}

		jni::Array<jni::byte_t> byteArray((jarray)byteArrayObj.getHandle());
		if (byteArray.isNull() || byteArray.getLength() < 1) {
			return false;
		}

		const long len = MIN(outData.size(), static_cast<size_t>(byteArray.getLength()));
		jni::env()->GetByteArrayRegion(jbyteArray(byteArray.getHandle()), 0, len,
		                               reinterpret_cast<jbyte *>(outData.data()));
		return true;
	}

	inline AndroidBleDevice
	AndroidBleDevice::openDevice(android::content::Context const &context,
	                             const std::string &serviceUuid,
	                             const std::string &notifyCharUuid,
	                             const std::string &writeCharUuid,
	                             const std::string &deviceName,
	                             int majorDeviceClass)
	{
		return AndroidBleDevice(Meta::data().clazz().call<jni::Object>(
		    Meta::data().openDevice, context.object(), serviceUuid, notifyCharUuid, writeCharUuid, deviceName,
		    majorDeviceClass));
	}

	inline AndroidBleDevice::ConnectionState
	AndroidBleDevice::getConnectionState() const
	{
		assert(!isNull());
		return static_cast<ConnectionState>(object().call<int32_t>(Meta::data().getConnectionStateOrdinal));
	}

	inline bool
	AndroidBleDevice::isConnected() const
	{
		assert(!isNull());
		return object().call<bool>(Meta::data().isConnected);
	}

	inline void
	AndroidBleDevice::connect()
	{
		assert(!isNull());
		object().call<void>(Meta::data().connect);
	}

	inline void
	AndroidBleDevice::disconnect()
	{
		assert(!isNull());
		object().call<void>(Meta::data().disconnect);
	}

	inline void
	AndroidBleDevice::destroy()
	{
		assert(!isNull());
		object().call<void>(Meta::data().destroy);
	}

	inline std::string
	AndroidBleDevice::getAddress() const
	{
		assert(!isNull());
		return object().call<std::string>(Meta::data().getAddress);
	}

	inline std::string
	AndroidBleDevice::getName() const
	{
		assert(!isNull());
		return object().call<std::string>(Meta::data().getName);
	}

	inline ReadResult
	AndroidBleDevice::read(int timeoutMs) const
	{
		assert(!isNull());
		return ReadResult(object().call<jni::Object>(Meta::data().read, timeoutMs));
	}

	inline bool
	AndroidBleDevice::write(std::span<const uint8_t> data)
	{
		assert(!isNull());
		if (data.empty()) {
			return false;
		}

		jni::Array<jni::byte_t> arr(static_cast<long>(data.size()));
		jni::env()->SetByteArrayRegion(jbyteArray(arr.getHandle()), 0, static_cast<jsize>(data.size()),
		                               reinterpret_cast<const jbyte *>(data.data()));
		return object().call<bool>(Meta::data().write, arr);
	}

	inline bool
	AndroidBleDevice::writeToCharacteristic(const std::string &charUuid, std::span<const uint8_t> data)
	{
		assert(!isNull());
		if (data.empty()) {
			return false;
		}

		jni::Array<jni::byte_t> arr(static_cast<long>(data.size()));
		jni::env()->SetByteArrayRegion(jbyteArray(arr.getHandle()), 0, static_cast<jsize>(data.size()),
		                               reinterpret_cast<const jbyte *>(data.data()));
		return object().call<bool>(Meta::data().writeToCharacteristic, charUuid, arr);
	}
} // namespace org::freedesktop::monado::auxiliary
} // namespace wrap
