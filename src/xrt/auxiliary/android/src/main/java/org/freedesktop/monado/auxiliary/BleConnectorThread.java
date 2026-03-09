// Copyright 2023-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Background thread for finding and connecting to BLE devices
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 *
 * @ingroup aux_android
 *
 */
package org.freedesktop.monado.auxiliary;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothClass;
import android.bluetooth.BluetoothDevice;
import android.os.ParcelUuid;
import android.util.Log;
import androidx.annotation.GuardedBy;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/*!
 * @brief Background thread that processes BLE device connection requests.
 *
 * This class manages a single thread that serializes find-and-connect requests,
 * ensuring that multiple callers don't race to claim the same device.
 */
@Keep
public final class BleConnectorThread {
    private static final String TAG = "Monado BleConnectorThread";

    // ! Retry interval when searching for devices
    private static final long CONNECT_RETRY_INTERVAL_MS = 1000;

    // ! Timeout for polling the request queue
    private static final long QUEUE_POLL_TIMEOUT_MS = 1000;

    // ! Callback interface for claiming addresses
    public interface AddressClaimCallback {
        /*!
         * @brief Attempt to claim an address, must be atomic!
         * @param address The address to claim
         * @return true if successfully claimed, false if already claimed
         */
        boolean tryClaimAddress(@NonNull String address);
    }

    /*!
     * @brief Request to find and connect to a BLE device.
     */
    public static class Request {
        // ! No timeout - search indefinitely until device is found or request is cancelled
        public static final long TIMEOUT_INFINITE = -1;

        public final UUID serviceUuid;
        public final String deviceName;
        public final int majorDeviceClass;
        public final long timeoutMs;
        public final Callback callback;
        private volatile boolean cancelled = false;

        /*!
         * @brief Callback interface for connection requests.
         */
        public interface Callback {
            /*!
             * @brief Called when a device has been found and claimed.
             * @param address The claimed device address, or empty string if timed out
             */
            void onDeviceFound(@NonNull String address);
        }

        /*!
         * @brief Create a request with no timeout (searches indefinitely).
         */
        public Request(
                @Nullable UUID serviceUuid,
                @Nullable String deviceName,
                int majorDeviceClass,
                @NonNull Callback callback) {
            this(serviceUuid, deviceName, majorDeviceClass, TIMEOUT_INFINITE, callback);
        }

        /*!
         * @brief Create a request with a maximum search timeout.
         *
         * @param serviceUuid     Service UUID to filter by (can be null)
         * @param deviceName      Device name pattern to match (can be null)
         * @param majorDeviceClass Major device class to filter by, or -1 to disable
         * @param timeoutMs       Maximum time to search in milliseconds, or TIMEOUT_INFINITE for no limit
         * @param callback        Callback to invoke when device is found or timeout occurs
         */
        public Request(
                @Nullable UUID serviceUuid,
                @Nullable String deviceName,
                int majorDeviceClass,
                long timeoutMs,
                @NonNull Callback callback) {
            this.serviceUuid = serviceUuid;
            this.deviceName = deviceName;
            this.majorDeviceClass = majorDeviceClass;
            this.timeoutMs = timeoutMs;
            this.callback = callback;
        }

        /*!
         * @brief Cancel this request.
         */
        public void cancel() {
            cancelled = true;
        }

        public boolean isCancelled() {
            return cancelled;
        }
    }

    private final Object lock = new Object();
    private final BlockingQueue<Request> requestQueue = new LinkedBlockingQueue<>();
    private final AddressClaimCallback addressClaimCallback;

    private volatile Thread thread = null;
    private volatile boolean isRunning = false;

    /*!
     * @brief Create a new connector thread.
     * @param addressClaimCallback Callback used to claim device addresses
     */
    public BleConnectorThread(@NonNull AddressClaimCallback addressClaimCallback) {
        this.addressClaimCallback = addressClaimCallback;
    }

    /*!
     * @brief Submit a request to find and connect to a BLE device.
     *
     * The request will be queued and processed by the connector thread.
     * Requests are processed serially to avoid race conditions.
     *
     * @param request The connection request
     */
    public void submitRequest(@NonNull Request request) {
        synchronized (lock) {
            requestQueue.add(request);
            ensureRunning();
        }
    }

    /*!
     * @brief Ensure the connector thread is running.
     */
    @GuardedBy("lock") private void ensureRunning() {
        if (isRunning) {
            return;
        }
        isRunning = true;
        thread = new Thread(this::runLoop, "BLE-Connector");
        thread.start();
    }

    /*!
     * @brief Shutdown the connector thread.
     *
     * This method stops the thread and cancels all pending requests.
     * The thread can be restarted by submitting a new request.
     */
    public void shutdown() {
        Thread threadToJoin = null;
        synchronized (lock) {
            if (!isRunning) {
                return;
            }

            Log.v(TAG, "Shutdown requested");

            for (Request request : requestQueue) {
                request.cancel();
            }
            isRunning = false;
            threadToJoin = thread;
            thread = null;

            requestQueue.clear();
        }

        if (threadToJoin != null) {
            threadToJoin.interrupt();
            try {
                threadToJoin.join(2000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /*!
     * @brief Main loop for the connector thread.
     */
    private void runLoop() {
        Log.i(TAG, "Thread started");

        while (isRunning) {
            Request request = null;
            try {
                // Poll with timeout to allow periodic check of isRunning flag
                request = requestQueue.poll(QUEUE_POLL_TIMEOUT_MS, TimeUnit.MILLISECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }

            // No request received, loop back to check isRunning flag
            if (request == null || request.callback == null) {
                continue;
            }

            // Skip cancelled requests
            if (request.isCancelled()) {
                Log.d(TAG, "Skipping cancelled request");
                continue;
            }

            Log.d(
                    TAG,
                    "Processing connect request for device: "
                            + (request.deviceName != null ? request.deviceName : "any"));

            String address = "";
            boolean timedOut = false;
            long startTime = System.currentTimeMillis();

            // ! Keep trying until we find and claim a device, request is cancelled, or timeout
            while (isRunning && !timedOut && !request.isCancelled() && address.isEmpty()) {
                address = findAndClaimDevice(request);
                if (!address.isEmpty()) {
                    break;
                }

                /*!
                 * A device address wasn't found yet, possibly not powered or bonded yet
                 * sleep some period of time before re-attempting again or time out with not
                 * found if the request has a max time limit to search.
                 */

                long sleepTime = CONNECT_RETRY_INTERVAL_MS;
                // Check timeout before sleeping
                if (request.timeoutMs != Request.TIMEOUT_INFINITE) {
                    long elapsed = System.currentTimeMillis() - startTime;
                    if (elapsed >= request.timeoutMs) {
                        timedOut = true;
                        Log.w(TAG, "Search timed out after " + elapsed + "ms");
                        break;
                    }
                    long remaining = request.timeoutMs - elapsed;
                    sleepTime = Math.min(CONNECT_RETRY_INTERVAL_MS, remaining);
                }

                try {
                    Thread.sleep(sleepTime);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            if (!request.isCancelled()) {
                try {
                    if (!address.isEmpty()) {
                        Log.i(TAG, "Device found, invoking callback for address: " + address);
                    } else if (timedOut) {
                        Log.w(TAG, "Search timed out, invoking callback with empty address");
                    } else {
                        Log.v(TAG, "No device found, invoking callback with empty address");
                    }
                    request.callback.onDeviceFound(address);
                } catch (Exception e) {
                    Log.e(TAG, "Exception in connect callback", e);
                }
            }
        }

        Log.i(TAG, "Thread stopped");
    }

    @NonNull @SuppressLint("MissingPermission")
    private String findAndClaimDevice(Request request) {
        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
        if (adapter == null || !adapter.isEnabled()) {
            Log.w(TAG, "Bluetooth is disabled! Enable it in the system settings.");
            return "";
        }

        Set<BluetoothDevice> bondedDevices = adapter.getBondedDevices();
        if (bondedDevices == null || bondedDevices.isEmpty()) {
            Log.w(TAG, "No devices paired, retrying...");
            return "";
        }

        String address = findAndClaimDeviceAddress(bondedDevices, request);
        if (address.isEmpty()) {
            Log.d(TAG, "Could not find available device in bonded devices, retrying...");
            return "";
        }

        return address;
    }

    @NonNull @SuppressLint("MissingPermission")
    private String findAndClaimDeviceAddress(Set<BluetoothDevice> bondedDevices, Request request) {
        for (BluetoothDevice device : bondedDevices) {
            // Check device type - accept both LE-only and dual-mode (BR/EDR + LE) devices
            int deviceType = device.getType();
            if (deviceType != BluetoothDevice.DEVICE_TYPE_LE
                    && deviceType != BluetoothDevice.DEVICE_TYPE_DUAL) {
                continue;
            }

            // Check device class
            if (request.majorDeviceClass != -1) {
                BluetoothClass bluetoothClass = device.getBluetoothClass();
                if (bluetoothClass == null
                        || bluetoothClass.getMajorDeviceClass() != request.majorDeviceClass) {
                    continue;
                }
            }

            // Check device name
            String name = device.getName();
            if (request.deviceName != null
                    && (name == null || !name.contains(request.deviceName))) {
                continue;
            }

            String address = device.getAddress();

            // Atomically try to claim this address
            if (address == null || !addressClaimCallback.tryClaimAddress(address)) {
                continue;
            }

            Log.i(TAG, String.format("Found and claimed device at address %s: %s", address, name));

            return address;
        }

        return "";
    }

    /*!
     * WARNING: This is not reliable as getUuids returns a cached list that won't be updated
     * until a service discovery is performed!
     */
    @SuppressWarnings("unused")
    @SuppressLint("MissingPermission")
    private boolean checkUuids(ParcelUuid[] uuids, UUID serviceUuid) {
        if (uuids == null || serviceUuid == null) {
            return true; // No UUID filter specified
        }
        for (ParcelUuid uuid : uuids) {
            if (uuid.getUuid().equals(serviceUuid)) {
                return true;
            }
        }
        return false;
    }
}
