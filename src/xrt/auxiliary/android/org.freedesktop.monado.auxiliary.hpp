// Copyright 2020-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Partially-generated wrapper for the
 * `org.freedesktop.monado.auxiliary` Java package.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup aux_android
 */

#pragma once

#include <span>
#include <vector>
#include <string>

#include "wrap/ObjectWrapperBase.h"

namespace wrap {
namespace android::app {
	class Activity;
} // namespace android::app

namespace android::content {
	class Context;
} // namespace android::content

namespace android::view {
	class SurfaceHolder;
	class WindowManager_LayoutParams;
} // namespace android::view

namespace org::freedesktop::monado::auxiliary {
	class MonadoView;
} // namespace org::freedesktop::monado::auxiliary

} // namespace wrap


namespace wrap {
namespace org::freedesktop::monado::auxiliary {
	/*!
	 * Wrapper for org.freedesktop.monado.auxiliary.MonadoView objects.
	 */
	class MonadoView : public ObjectWrapperBase
	{
	public:
		using ObjectWrapperBase::ObjectWrapperBase;
		static constexpr const char *
		getTypeName() noexcept
		{
			return "org/freedesktop/monado/auxiliary/MonadoView";
		}

		static constexpr const char *
		getFullyQualifiedTypeName() noexcept
		{
			return "org.freedesktop.monado.auxiliary.MonadoView";
		}

		/*!
		 * Wrapper for the attachToWindow static method
		 *
		 * Java prototype:
		 * `public static org.freedesktop.monado.auxiliary.MonadoView attachToActivity(android.content.Context,
		 * long, android.view.WindowManager.LayoutParams);`
		 *
		 * JNI signature:
		 * (Landroid/content/Context;JLandroid/view/WindowManager$LayoutParams;)Lorg/freedesktop/monado/auxiliary/MonadoView;
		 *
		 */
		static MonadoView
		attachToWindow(android::content::Context const &displayContext,
		               void *nativePointer,
		               android::view::WindowManager_LayoutParams const &lp);

		/*!
		 * Wrapper for the removeFromWindow static method
		 *
		 * Java prototype:
		 * `public static void removeFromWindow(android.content.Context,
		 * org.freedesktop.monado.auxiliary.MonadoView, int);`
		 *
		 * JNI signature: (Landroid/content/Context;Lorg/freedesktop/monado/auxiliary/MonadoView;I)V
		 *
		 */
		static void
		removeFromWindow(MonadoView const &view);

		/*!
		 * Wrapper for the getDisplayMetrics static method
		 *
		 * Java prototype:
		 * `public static android.util.DisplayMetrics getDisplayMetrics(android.content.Context);`
		 *
		 * JNI signature: (Landroid/content/Context;)Landroid/util/DisplayMetrics;
		 *
		 */
		static jni::Object
		getDisplayMetrics(android::content::Context const &context);

		/*!
		 * Wrapper for the getDisplayRefreshRate static method
		 *
		 * Java prototype:
		 * `public static float getDisplayRefreshRate(android.content.Context);`
		 *
		 * JNI signature: (Landroid/content/Context;)F;
		 *
		 */
		static float
		getDisplayRefreshRate(android::content::Context const &context);

		/*!
		 * Wrapper for the getDisplayModeIdWidth static method
		 *
		 * Java prototype:
		 * `public static int getDisplayModeIdWidth(@NonNull final Context context, int display,
		 * int displayModeId);`
		 *
		 * JNI signature: (Landroid/content/Context;II)I;
		 *
		 */
		static int32_t
		getDisplayModeIdWidth(android::content::Context const &context,
		                      int32_t displayId,
		                      int32_t displayModeId);

		/*!
		 * Wrapper for the getDisplayModeIdHeight static method
		 *
		 * Java prototype:
		 * `public static int getDisplayModeIdHeight(@NonNull final Context context, int display,
		 *  int displayModeId);`
		 *
		 * JNI signature: (Landroid/content/Context;II)I;
		 *
		 */
		static int32_t
		getDisplayModeIdHeight(android::content::Context const &context,
		                       int32_t displayId,
		                       int32_t displayModeId);

		/*!
		 * Wrapper for the getSupportedRefreshRates static method
		 *
		 * Java prototype:
		 * `public static float[] getSupportedRefreshRates(android.content.Context);`
		 *
		 * JNI signature: (Landroid/content/Context;)[F;
		 *
		 */
		static std::vector<float>
		getSupportedRefreshRates(android::content::Context const &context);

		/*!
		 * Wrapper for the getNativePointer method
		 *
		 * Java prototype:
		 * `public long getNativePointer();`
		 *
		 * JNI signature: ()J
		 *
		 */
		void *
		getNativePointer();

		/*!
		 * Wrapper for the markAsDiscardedByNative method
		 *
		 * Java prototype:
		 * `public void markAsDiscardedByNative();`
		 *
		 * JNI signature: ()V
		 *
		 */
		void
		markAsDiscardedByNative();

		/*!
		 * Wrapper for the waitGetSurfaceHolder method
		 *
		 * Java prototype:
		 * `public android.view.SurfaceHolder waitGetSurfaceHolder(int);`
		 *
		 * JNI signature: (I)Landroid/view/SurfaceHolder;
		 *
		 */
		android::view::SurfaceHolder
		waitGetSurfaceHolder(int32_t wait_ms);

		/*!
		 * Initialize the static metadata of this wrapper with a known
		 * (non-null) Java class.
		 */
		static void
		staticInitClass(jni::jclass clazz)
		{
			Meta::data(clazz);
		}

		/*!
		 * Class metadata
		 */
		struct Meta : public MetaBase
		{
			jni::method_t attachToWindow;
			jni::method_t removeFromWindow;
			jni::method_t getDisplayMetrics;
			jni::method_t getDisplayRefreshRate;
			jni::method_t getSupportedRefreshRates;
			jni::method_t getNativePointer;
			jni::method_t markAsDiscardedByNative;
			jni::method_t waitGetSurfaceHolder;
			jni::method_t getDisplayModeIdWidth;
			jni::method_t getDisplayModeIdHeight;

			/*!
			 * Singleton accessor
			 */
			static Meta &
			data(jni::jclass clazz = nullptr)
			{
				static Meta instance{clazz};
				return instance;
			}

		private:
			explicit Meta(jni::jclass clazz);
		};
	};

	class ActivityLifecycleListener : public ObjectWrapperBase
	{
	public:
		using ObjectWrapperBase::ObjectWrapperBase;
		static constexpr const char *
		getTypeName() noexcept
		{
			return "org/freedesktop/monado/auxiliary/ActivityLifecycleListener";
		}

		static constexpr const char *
		getFullyQualifiedTypeName() noexcept
		{
			return "org.freedesktop.monado.auxiliary.ActivityLifecycleListener";
		}

		/*!
		 * Initialize the static metadata of this wrapper with a known
		 * (non-null) Java class.
		 */
		static void
		staticInitClass(jni::jclass clazz)
		{
			Meta::data(clazz);
		}

		/*!
		 * Wrapper for a constructor
		 *
		 * Java prototype:
		 * `public org.freedesktop.monado.auxiliary.ActivityLifecycleListener(long);`
		 *
		 * JNI signature: (J)V
		 *
		 */
		static ActivityLifecycleListener
		construct(void *nativePointer);

		/*!
		 * Wrapper for the registerCallback method
		 *
		 * Java prototype:
		 * `public void registerCallback(android.app.Activity);`
		 *
		 * JNI signature: (Landroid/app/Activity;)V
		 *
		 */
		void
		registerCallback(android::app::Activity const &activity);

		/*!
		 * Wrapper for the unregisterCallback method
		 *
		 * Java prototype:
		 * `public void unregisterCallback(android.app.Activity);`
		 *
		 * JNI signature: (Landroid/app/Activity;)V
		 *
		 */
		void
		unregisterCallback(android::app::Activity const &activity);

		/*!
		 * Class metadata
		 */
		struct Meta : public MetaBase
		{
			jni::method_t init;
			jni::method_t registerCallback;
			jni::method_t unregisterCallback;

			/*!
			 * Singleton accessor
			 */
			static Meta &
			data(jni::jclass clazz = nullptr)
			{
				static Meta instance{clazz};
				return instance;
			}

		private:
			explicit Meta(jni::jclass clazz);
		};
	};

	class ReadResult : public ObjectWrapperBase
	{
	public:
		using ObjectWrapperBase::ObjectWrapperBase;

		static constexpr const char *
		getTypeName() noexcept
		{
			return "android/util/Pair";
		}

		int32_t
		getStatus() const;

		bool
		getData(std::span<uint8_t> outData) const;

		struct Meta : public MetaBaseDroppable
		{
			jni::field_t first;
			jni::field_t second;

			/*!
			 * Singleton accessor
			 */
			static Meta &
			data()
			{
				static Meta instance{};
				return instance;
			}

		private:
			Meta();
		};
	};

	/*!
	 * Wrapper for org.freedesktop.monado.auxiliary.AndroidBleDevice objects.
	 */
	class AndroidBleDevice : public ObjectWrapperBase
	{
	public:
		using ObjectWrapperBase::ObjectWrapperBase;

		enum class ConnectionState : int32_t
		{
			DISCONNECTED,
			CONNECTING,
			CONNECTED,
			READY,
			RECEIVING,
			CONNECTION_LOST
		};

		static constexpr const char *
		getTypeName() noexcept
		{
			return "org/freedesktop/monado/auxiliary/AndroidBleDevice";
		}

		static constexpr const char *
		getFullyQualifiedTypeName() noexcept
		{
			return "org.freedesktop.monado.auxiliary.AndroidBleDevice";
		}

		static AndroidBleDevice
		openDevice(android::content::Context const &context,
		           const std::string &serviceUuid,
		           const std::string &notifyCharUuid,
		           const std::string &writeCharUuid,
		           const std::string &deviceName,
		           int majorDeviceClass);

		ConnectionState
		getConnectionState() const;

		bool
		isConnected() const;

		void
		connect();

		void
		disconnect();

		void
		destroy();

		std::string
		getAddress() const;

		std::string
		getName() const;

		ReadResult
		read(int timeoutMs) const;

		bool
		write(std::span<const uint8_t> data);

		bool
		writeToCharacteristic(const std::string &charUuid, std::span<const uint8_t> data);

		/*!
		 * Initialize the static metadata of this wrapper with a known
		 * (non-null) Java class.
		 */
		static void
		staticInitClass(jni::jclass clazz)
		{
			Meta::data(clazz);
		}

		/*!
		 * Class metadata
		 */
		struct Meta : public MetaBase
		{
			jni::method_t openDevice;
			jni::method_t getConnectionStateOrdinal;
			jni::method_t isConnected;
			jni::method_t connect;
			jni::method_t disconnect;
			jni::method_t destroy;
			jni::method_t getAddress;
			jni::method_t getName;
			jni::method_t read;
			jni::method_t write;
			jni::method_t writeToCharacteristic;

			/*!
			 * Singleton accessor
			 */
			static Meta &
			data(jni::jclass clazz = nullptr)
			{
				static Meta instance{clazz};
				return instance;
			}

		private:
			explicit Meta(jni::jclass clazz);
		};
	};
} // namespace org::freedesktop::monado::auxiliary
} // namespace wrap

#include "org.freedesktop.monado.auxiliary.impl.hpp"
