// Copyright 2020-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Out-of-line implementations for partially-generated wrapper for the
 * `org.freedesktop.monado.auxiliary` Java package.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup aux_android
 */

#include "org.freedesktop.monado.auxiliary.hpp"

namespace wrap {
namespace org::freedesktop::monado::auxiliary {
	MonadoView::Meta::Meta(jni::jclass clazz)
	    : MetaBase(MonadoView::getTypeName(), clazz),
	      attachToWindow(classRef().getStaticMethod(
	          "attachToWindow",
	          "(Landroid/content/Context;JLandroid/view/WindowManager$LayoutParams;)Lorg/freedesktop/monado/"
	          "auxiliary/MonadoView;")),
	      removeFromWindow(
	          classRef().getStaticMethod("removeFromWindow", "(Lorg/freedesktop/monado/auxiliary/MonadoView;)V")),
	      getDisplayMetrics(classRef().getStaticMethod("getDisplayMetrics",
	                                                   "(Landroid/content/Context;)Landroid/util/DisplayMetrics;")),
	      getDisplayRefreshRate(
	          classRef().getStaticMethod("getDisplayRefreshRate", "(Landroid/content/Context;)F")),
	      getSupportedRefreshRates(
	          classRef().getStaticMethod("getSupportedRefreshRates", "(Landroid/content/Context;)[F")),
	      getNativePointer(classRef().getMethod("getNativePointer", "()J")),
	      markAsDiscardedByNative(classRef().getMethod("markAsDiscardedByNative", "()V")),
	      waitGetSurfaceHolder(classRef().getMethod("waitGetSurfaceHolder", "(I)Landroid/view/SurfaceHolder;")),
	      getDisplayModeIdWidth(
	          classRef().getStaticMethod("getDisplayModeIdWidth", "(Landroid/content/Context;II)I")),
	      getDisplayModeIdHeight(
	          classRef().getStaticMethod("getDisplayModeIdHeight", "(Landroid/content/Context;II)I"))
	{}

	ActivityLifecycleListener::Meta::Meta(jni::jclass clazz)
	    : MetaBase(ActivityLifecycleListener::getTypeName(), clazz), init(classRef().getMethod("<init>", "(J)V")),
	      registerCallback(classRef().getMethod("registerCallback", "(Landroid/app/Activity;)V")),
	      unregisterCallback(classRef().getMethod("unregisterCallback", "(Landroid/app/Activity;)V"))
	{}

	ReadResult::Meta::Meta()
	    : MetaBaseDroppable(ReadResult::getTypeName()), first(classRef().getField("first", "Ljava/lang/Object;")),
	      second(classRef().getField("second", "Ljava/lang/Object;"))
	{
		MetaBaseDroppable::dropClassRef();
	}

	AndroidBleDevice::Meta::Meta(jni::jclass clazz)
	    : MetaBase(AndroidBleDevice::getTypeName(), clazz),
	      openDevice(classRef().getStaticMethod("openDevice",
	                                            "(Landroid/content/Context;Ljava/lang/String;Ljava/lang/"
	                                            "String;Ljava/lang/String;Ljava/lang/String;I)"
	                                            "Lorg/freedesktop/monado/auxiliary/AndroidBleDevice;")),
	      getConnectionStateOrdinal(classRef().getMethod("getConnectionStateOrdinal", "()I")),
	      isConnected(classRef().getMethod("isConnected", "()Z")), connect(classRef().getMethod("connect", "()V")),
	      disconnect(classRef().getMethod("disconnect", "()V")), destroy(classRef().getMethod("destroy", "()V")),
	      getAddress(classRef().getMethod("getAddress", "()Ljava/lang/String;")),
	      getName(classRef().getMethod("getName", "()Ljava/lang/String;")),
	      read(classRef().getMethod("read", "(I)Landroid/util/Pair;")),
	      write(classRef().getMethod("write", "([B)Z")),
	      writeToCharacteristic(classRef().getMethod("writeToCharacteristic", "(Ljava/lang/String;[B)Z"))
	{}
} // namespace org::freedesktop::monado::auxiliary
} // namespace wrap
