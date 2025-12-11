// Copyright 2025-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Global state for finding and connecting to BLE devices on Android
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 *
 * @ingroup aux_android
 *
 */
package org.freedesktop.monado.auxiliary;

import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import java.util.HashSet;
import java.util.Set;

/**
 * Singleton that manages BLE device connections globally.
 *
 * <p>This class: - Tracks claimed device addresses to prevent multiple connections to the same
 * device - Provides a single connector thread that serializes all find-and-connect requests
 */
@Keep
public final class AndroidBleGlobals {

    private final Object lock = new Object();
    private final Set<String> connectedAddresses = new HashSet<>();
    private final BleConnectorThread connectorThread;

    private AndroidBleGlobals() {
        connectorThread = new BleConnectorThread(this::tryClaimAddress);
    }

    /*!
     * Convenience type alias for device connection requests.
     */
    public static class DeviceConnectRequest extends BleConnectorThread.Request {
        /*!
         * @brief Create a request with no timeout (searches indefinitely).
         */
        public DeviceConnectRequest(
                @androidx.annotation.Nullable java.util.UUID serviceUuid,
                @androidx.annotation.Nullable String deviceName,
                int majorDeviceClass,
                @NonNull Callback callback) {
            super(serviceUuid, deviceName, majorDeviceClass, callback);
        }

        /*!
         * @brief Create a request with a maximum search timeout.
         *
         * @param serviceUuid      Service UUID to filter by (can be null)
         * @param deviceName       Device name pattern to match (can be null)
         * @param majorDeviceClass Major device class to filter by, or -1 to disable
         * @param timeoutMs        Maximum time to search in milliseconds, or TIMEOUT_INFINITE for no limit
         * @param callback         Callback to invoke when device is found or timeout occurs
         */
        public DeviceConnectRequest(
                @androidx.annotation.Nullable java.util.UUID serviceUuid,
                @androidx.annotation.Nullable String deviceName,
                int majorDeviceClass,
                long timeoutMs,
                @NonNull Callback callback) {
            super(serviceUuid, deviceName, majorDeviceClass, timeoutMs, callback);
        }
    }

    /*!
     * @brief Atomically check if an address is available and claim it if so.
     *
     * This method combines the check and record operations atomically to prevent
     * race conditions where multiple threads could both see an address as available
     * and then both try to connect to it.
     *
     * @param address The Bluetooth device address to claim
     * @return true if the address was successfully claimed (was not already connected),
     *         false if the address is empty or already claimed by another device
     */
    public boolean tryClaimAddress(@NonNull String address) {
        if (address.isEmpty()) {
            return false;
        }
        synchronized (lock) {
            if (connectedAddresses.contains(address)) {
                return false;
            }
            connectedAddresses.add(address);
            return true;
        }
    }

    /*!
     * @brief Release a previously claimed address.
     *
     * Call this when a device disconnects to allow the address to be
     * claimed again for reconnection.
     *
     * @param address The Bluetooth device address to release
     */
    public void releaseAddress(@NonNull String address) {
        if (address.isEmpty()) {
            return;
        }
        synchronized (lock) {
            connectedAddresses.remove(address);
        }
    }

    /*!
     * @brief Submit a request to find and connect to a BLE device.
     *
     * The request will be queued and processed by the connector thread.
     * Requests are processed serially to avoid race conditions when
     * multiple devices are searching for the same type of hardware.
     *
     * @param request The connection request
     */
    public void submitConnectRequest(@NonNull DeviceConnectRequest request) {
        connectorThread.submitRequest(request);
    }

    /*!
     * @brief Shutdown the connector thread.
     *
     * This method stops the connector thread and cancels all pending requests.
     * The thread can be restarted by submitting a new request.
     */
    public void shutdown() {
        connectorThread.shutdown();
    }

    private static class Holder {
        private static final AndroidBleGlobals INSTANCE = new AndroidBleGlobals();
    }

    public static AndroidBleGlobals getInstance() {
        return Holder.INSTANCE;
    }
}
