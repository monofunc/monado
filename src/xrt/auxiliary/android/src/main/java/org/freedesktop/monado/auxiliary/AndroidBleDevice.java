// Copyright 2023-2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Android BLE device wrapper for os_ble.h abstraction.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 *
 * @ingroup aux_android
 *
 * This class provides a generic BLE device implementation that can be used
 * as the Android backend for the os_ble.h abstraction layer. It supports:
 * - Device discovery by name pattern or UUID
 * - Reading notification data from a characteristic
 * - Writing data to a characteristic
 * - Connection state management
 *
 */
package org.freedesktop.monado.auxiliary;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothProfile;
import android.content.Context;
import android.os.Build;
import android.util.Log;
import android.util.Pair;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Android BLE device wrapper that implements the functionality needed for the os_ble.h abstraction
 * layer.
 *
 * <p>This class handles: * Device discovery by name pattern or service UUID * GATT connection
 * management * Notification subscriptions for reading data * Characteristic writes for sending
 * commands * Connection state monitoring and reconnection
 */
@Keep
public class AndroidBleDevice {
    private static final String TAG = "Monado AndroidBleDevice";

    /*!
     * Standard Client Characteristic Configuration Descriptor (CCCD) UUID.
     * This is the Bluetooth SIG-defined UUID for the descriptor that must be
     * written to enable notifications on a characteristic.
     */
    private static final UUID UUID_CONFIG_DESCRIPTOR =
            UUID.fromString("00002902-0000-1000-8000-00805F9B34FB");

    // ! Default connection timeout check interval in milliseconds
    private static final long CONNECTION_CHECK_INTERVAL_MS = 500;

    /*!
     * Timeout for considering connection lost (no data received).
     * If no notification data is received within this period while in RECEIVING
     * state, the connection is considered lost. This detects device sleep or
     * out-of-range conditions that don't trigger a proper disconnect.
     */
    private static final long CONNECTION_TIMEOUT_MS = 2000;

    /*!
     * Maximum number of packets to buffer.
     * If the consumer (native code calling read()) is slower than the producer
     * (BLE notifications), older packets are dropped to prevent memory growth.
     */
    private static final int PACKET_QUEUE_SIZE = 1;

    /*!
     * Connection state enumeration.
     *
     * State transitions:
     *   * DISCONNECTED -> CONNECTING: When connect() is called and device is found
     *   * CONNECTING -> CONNECTED: When GATT connection succeeds and discovering services.
     *   * CONNECTED -> READY: Gatt connected and service was successfully discovered
     *   * READY -> RECEIVING: When first notification data arrives
     *   * RECEIVING -> CONNECTION_LOST: When no data received for CONNECTION_TIMEOUT_MS
     *   * Any state -> DISCONNECTED: On explicit disconnect or GATT error
     *
     */
    public enum ConnectionState {
        // ! Device is disconnected
        DISCONNECTED,
        // ! Connection attempt in progress
        CONNECTING,
        // ! Connected but not yet receiving data (service discovery or notification setup)
        CONNECTED,
        // ! Connected and service has successfully been discovered and fully ready to write/notify
        READY,
        // ! Connected and actively receiving notification data
        RECEIVING,
        // ! Connection was lost (timeout or disconnect)
        CONNECTION_LOST
    }

    /*!
     * Listener interface for connection state changes.
     * Can be used to notify native code of state transitions.
     */
    public interface ConnectionStateListener {
        /*!
         * @brief Called when the connection state changes.
         *
         * @param oldState Previous connection state
         * @param newState New connection state
         */
        void onConnectionStateChanged(ConnectionState oldState, ConnectionState newState);
    }

    // ! Configuration (immutable after construction)
    private final Context context;
    private final UUID serviceUuid;
    private final UUID notifyCharUuid;
    private final UUID writeCharUuid;
    private final String deviceName;
    private final int majorDeviceClass;

    /*! Connection state
     * volatile ensures visibility across threads, but state transitions
     * must still be synchronized to prevent race conditions
     */
    private volatile ConnectionState connectionState = ConnectionState.DISCONNECTED;
    private final Object connectionStateLock = new Object();

    // ! Timers
    private Timer checkConnectionTimer;

    // ! Connection request (cached for cancellation)
    private AndroidBleGlobals.DeviceConnectRequest connectRequest;

    // ! GATT objects
    private BluetoothGattCallback gattCallback;
    private BluetoothGatt gatt;
    private final Object gattLock = new Object();

    // Device tracking
    private volatile String deviceAddress = "";
    private volatile boolean firstTimeConnection = true;

    // Data reception
    private volatile long lastDataTime = -1;
    private final Object dataTimeLock = new Object();

    /*!
     * @brief Packet queue for read operations.
     * Uses a bounded blocking queue instead of a simple "lastPacket" variable to:
     *   1. Allow blocking reads with timeout (for os_ble_read semantics)
     *   2. Buffer multiple packets if consumer is slower than producer
     *   3. Provide thread-safe producer/consumer access without explicit locking
     */
    private final BlockingQueue<byte[]> packetQueue = new ArrayBlockingQueue<>(PACKET_QUEUE_SIZE);

    // ! Connection state listener (optional, for native callbacks)
    private volatile ConnectionStateListener connectionStateListener;

    // ! Builder class for creating AndroidBleDevice instances.
    public static class Builder {
        private final Context context;
        private UUID serviceUuid;
        private UUID notifyCharUuid;
        private UUID writeCharUuid = null;
        private String deviceName = null;
        private int majorDeviceClass = -1;

        /*!
         * @brief Create a new builder.
         * @param context Android context (application or activity context)
         */
        public Builder(@NonNull Context context) {
            this.context = context.getApplicationContext();
        }

        /*!
         * @brief Set the service UUID to connect to.
         * @param uuid Service UUID string (e.g., "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
         * @return This builder
         */
        public Builder serviceUuid(@NonNull String uuid) {
            this.serviceUuid = UUID.fromString(uuid);
            return this;
        }

        /*!
         * @brief Set the characteristic UUID to subscribe to for notifications (read).
         * @param uuid Characteristic UUID string
         * @return This builder
         */
        public Builder notifyCharacteristicUuid(@NonNull String uuid) {
            this.notifyCharUuid = UUID.fromString(uuid);
            return this;
        }

        /*!
         * @brief Set the characteristic UUID to write to.
         * @param uuid Characteristic UUID string
         * @return This builder
         */
        public Builder writeCharacteristicUuid(@NonNull String uuid) {
            this.writeCharUuid = UUID.fromString(uuid);
            return this;
        }

        /*!
         * @brief Set the device name pattern to search for in bonded devices.
         *
         * The device name must contain this pattern (case-sensitive).
         *
         * @param  pattern Name pattern to match
         * @return This builder
         */
        public Builder deviceName(@NonNull String pattern) {
            this.deviceName = pattern;
            return this;
        }

        /*!
         * @brief Set the required major device class for filtering.
         *
         * Only devices with this major class will be considered.
         * Use constants from {@ref BluetoothClass.Device.Major}, e.g.,
         * {@ref BluetoothClass.Device.Major#UNCATEGORIZED}.
         *
         * @param majorClass Major device class constant
         * @return This builder
         */
        public Builder majorDeviceClass(int majorClass) {
            this.majorDeviceClass = majorClass;
            return this;
        }

        /*!
         * @brief Build the AndroidBleDevice instance.
         * @return New AndroidBleDevice instance
         * @throws IllegalStateException if required fields are not set
         */
        public AndroidBleDevice build() {
            if (context == null) {
                throw new IllegalStateException("Context is required");
            }
            if (serviceUuid == null) {
                throw new IllegalStateException("Service UUID is required");
            }
            if (notifyCharUuid == null) {
                throw new IllegalStateException("Notify characteristic UUID is required");
            }
            return new AndroidBleDevice(this);
        }
    }

    // ! Private constructor - use Builder to create instances.
    private AndroidBleDevice(Builder builder) {
        this.context = builder.context;
        this.serviceUuid = builder.serviceUuid;
        this.notifyCharUuid = builder.notifyCharUuid;
        this.writeCharUuid = builder.writeCharUuid;
        this.deviceName = builder.deviceName;
        this.majorDeviceClass = builder.majorDeviceClass;
    }

    /*!
     * @brief Create and connect to a BLE device by device name & service UUID.
     *
     * This factory method searches for a device advertising the specified service UUID.
     *
     * @param context           Android context
     * @param serviceUuid       Service UUID string
     * @param notifyCharUuid    Notification characteristic UUID string
     * @param writeCharUuid     Write characteristic UUID string (can be null)
     * @param deviceName        Device name pattern to match (can be null)
     * @param majorDeviceClass  Major device class filter, or -1 to disable filtering
     * @return New connected    AndroidBleDevice, or null on error
     */
    @Keep
    @Nullable public static AndroidBleDevice openDevice(
            @NonNull Context context,
            @NonNull String serviceUuid,
            @NonNull String notifyCharUuid,
            String writeCharUuid,
            String deviceName,
            int majorDeviceClass) {
        try {
            Builder builder =
                    new Builder(context)
                            .serviceUuid(serviceUuid)
                            .notifyCharacteristicUuid(notifyCharUuid)
                            .majorDeviceClass(majorDeviceClass);

            if (writeCharUuid != null && !writeCharUuid.isEmpty()) {
                builder.writeCharacteristicUuid(writeCharUuid);
            }

            if (deviceName != null && !deviceName.isEmpty()) {
                builder.deviceName(deviceName);
            }

            AndroidBleDevice device = builder.build();
            device.connect();
            return device;
        } catch (Exception e) {
            Log.e(TAG, "Failed to create device", e);
            return null;
        }
    }

    /*!
     * @brief Set a listener for connection state changes.
     * @param listener The listener, or null to remove
     */
    @Keep
    public void setConnectionStateListener(@Nullable ConnectionStateListener listener) {
        this.connectionStateListener = listener;
    }

    /*!
     * @brief Get the current connection state.
     * @return Current connection state
     */
    @Keep
    public ConnectionState getConnectionState() {
        synchronized (connectionStateLock) {
            return connectionState;
        }
    }

    @Keep
    public int getConnectionStateOrdinal() {
        return getConnectionState().ordinal();
    }

    @Keep
    public boolean isConnectionState(@NonNull ConnectionState... states) {
        if (states.length == 0) {
            return false;
        }
        synchronized (connectionStateLock) {
            for (ConnectionState s : states) {
                if (connectionState == s) {
                    return true;
                }
            }
            return false;
        }
    }

    /*!
     * @brief Check if the device is connected and receiving data.
     * @return true if connected and receiving
     */
    @Keep
    public boolean isConnected() {
        return isConnectionState(ConnectionState.READY, ConnectionState.RECEIVING);
    }

    /*!
     * Start connecting to the BLE device.
     * This will search for a bonded device matching the configured criteria
     * and establish a GATT connection.
     */
    @Keep
    @SuppressLint("MissingPermission")
    public void connect() {

        if (!isConnectionState(ConnectionState.DISCONNECTED, ConnectionState.CONNECTION_LOST)) {
            return;
        }

        Log.i(TAG, "Starting connection...");

        if (checkConnectionTimer != null) {
            checkConnectionTimer.cancel();
            checkConnectionTimer = null;
        }

        checkConnectionTimer = new Timer("BLE-Check", true);
        checkConnectionTimer.scheduleAtFixedRate(
                new CheckConnectionTask(),
                CONNECTION_CHECK_INTERVAL_MS,
                CONNECTION_CHECK_INTERVAL_MS);

        // Submit a connection request to the global connector thread
        connectRequest =
                new AndroidBleGlobals.DeviceConnectRequest(
                        serviceUuid, deviceName, majorDeviceClass, this::onDeviceFound);
        AndroidBleGlobals.getInstance().submitConnectRequest(connectRequest);
    }

    /*!
     * @brief Callback invoked when the global connector thread finds and claims a device.
     *
     * @param address The claimed device address
     */
    @SuppressLint("MissingPermission")
    private void onDeviceFound(@NonNull String address) {
        if (address.isEmpty()) {
            Log.i(TAG, "Device not found");
            setConnectionState(ConnectionState.DISCONNECTED);
            return;
        }

        deviceAddress = address;
        Log.i(TAG, "Device found, connecting to address: " + address);
        setConnectionState(ConnectionState.CONNECTING);

        if (!gattConnect(address)) {
            Log.e(TAG, "GATT connection failed");
            setConnectionState(ConnectionState.DISCONNECTED);
        }
    }

    @SuppressLint("MissingPermission")
    private boolean gattConnect(String address) {
        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
        if (adapter == null || !adapter.isEnabled()) {
            Log.e(
                    TAG,
                    String.format(
                            "Cannot connect to %s, bluetooth adapter is null or disabled",
                            address));
            return false;
        }

        BluetoothDevice device = adapter.getRemoteDevice(address);
        if (device == null) {
            Log.e(TAG, String.format("Could not get remote device with address %s!", address));
            return false;
        }

        gattCallback = new BleGattCallback();

        // TRANSPORT_LE forces BLE connection even for dual-mode devices.
        // autoConnect=true means the system will automatically connect when
        // the device becomes available (useful for devices that sleep).
        BluetoothGatt newGatt =
                device.connectGatt(context, true, gattCallback, BluetoothDevice.TRANSPORT_LE);
        if (newGatt == null) {
            Log.e(TAG, String.format("Could not connect device %s over GATT!", address));
            return false;
        }

        synchronized (gattLock) {
            gatt = newGatt;
        }
        return true;
    }

    /*!
     * Disconnect from the device and release resources.
     */
    @Keep
    public void disconnect() {
        Log.i(TAG, "Disconnecting...");

        // Cancel connection timer
        if (checkConnectionTimer != null) {
            checkConnectionTimer.cancel();
            checkConnectionTimer = null;
        }

        // Cancel any pending connect request
        if (connectRequest != null) {
            connectRequest.cancel();
            connectRequest = null;
        }

        // Disconnect GATT
        synchronized (gattLock) {
            if (gatt != null) {
                try {
                    gatt.disconnect();
                    gatt.close();
                } catch (SecurityException e) {
                    Log.e(TAG, "SecurityException during disconnect", e);
                }
                gatt = null;
            }
        }

        // Release the claimed address so it can be used again
        if (!deviceAddress.isEmpty()) {
            AndroidBleGlobals.getInstance().releaseAddress(deviceAddress);
            deviceAddress = "";
        }

        // Clear packet queue
        packetQueue.clear();

        setConnectionState(ConnectionState.DISCONNECTED);
    }

    /*!
     * Destroy the device and release all resources.
     * Alias for disconnect() for compatibility with os_ble.h naming.
     */
    @Keep
    public void destroy() {
        disconnect();
    }

    /*!
     * @brief Read notification data from the device.
     *
     * This method blocks until data is available or the timeout expires.
     * It corresponds to os_ble_read() in the C interface.
     *
     * @param timeoutMs  Timeout in milliseconds. Negative value blocks indefinitely,
     *                   0 polls without blocking, positive value blocks for that duration.
     * @return           Pair<byte[], Integer> containing data and status:
     *                   - data: byte array of received data, or null on timeout/error
     *                   - status: >0 on Success, 0 on timeout, -1 on error
     */
    @Keep
    @NonNull public Pair<byte[], Integer> read(int timeoutMs) {
        try {
            if (timeoutMs < 0) {
                // Block indefinitely
                return new Pair<>(packetQueue.take(), 1);
            } else if (timeoutMs == 0) {
                // Poll without blocking
                byte[] data = packetQueue.poll();
                return new Pair<>(data, data != null ? 1 : 0);
            } else {
                // Block with timeout
                byte[] data = packetQueue.poll(timeoutMs, TimeUnit.MILLISECONDS);
                return new Pair<>(data, data != null ? 1 : 0);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return new Pair<>(null, -1);
        }
    }

    /*!
     * @brief Write data to the write characteristic.
     *
     * This method corresponds to os_ble_write() in the C interface.
     *
     * @param data Data to write
     * @return true if the write was initiated successfully
     */
    @Keep
    @SuppressLint("MissingPermission")
    public boolean write(@NonNull byte[] data) {
        if (writeCharUuid == null) {
            Log.e(TAG, "Write characteristic UUID not configured");
            return false;
        }
        return writeToCharacteristic(writeCharUuid, data);
    }

    /*!
     * @brief Write data to a specific characteristic UUID.
     *
     * @param charUuid The characteristic UUID string to write to
     * @param data Data to write
     * @return true if the write was initiated successfully
     */
    @Keep
    @SuppressLint("MissingPermission")
    public boolean writeToCharacteristic(@NonNull String charUuid, @NonNull byte[] data) {
        try {
            UUID uuid = UUID.fromString(charUuid);
            return writeToCharacteristic(uuid, data);
        } catch (IllegalArgumentException e) {
            Log.e(TAG, "Invalid characteristic UUID: " + charUuid, e);
            return false;
        }
    }

    /*!
     * @brief Write data to a specific characteristic UUID.
     *
     * @param charUuid The characteristic UUID to write to
     * @param data Data to write
     * @return true if the write was initiated successfully
     */
    @Keep
    @SuppressLint("MissingPermission")
    public boolean writeToCharacteristic(@NonNull UUID charUuid, @NonNull byte[] data) {
        synchronized (gattLock) {
            if (gatt == null) {
                Log.e(TAG, "Cannot write: not connected");
                return false;
            }

            BluetoothGattService service = gatt.getService(serviceUuid);
            if (service == null) {
                Log.e(TAG, "Cannot write: service not found");
                return false;
            }

            BluetoothGattCharacteristic characteristic = service.getCharacteristic(charUuid);
            if (characteristic == null) {
                Log.e(TAG, "Cannot write: characteristic " + charUuid + " not found");
                return false;
            }

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                int writeType = getPreferredWriteType(characteristic);
                int result = gatt.writeCharacteristic(characteristic, data, writeType);
                return result == BluetoothGatt.GATT_SUCCESS;
            } else {
                characteristic.setValue(data);
                return gatt.writeCharacteristic(characteristic);
            }
        }
    }

    @Keep
    private static int getPreferredWriteType(BluetoothGattCharacteristic characteristic) {
        int properties = characteristic.getProperties();

        if ((properties & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0) {
            // Prefer no-response for lower latency (common for XR controllers)
            return BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE;
        }
        if ((properties & BluetoothGattCharacteristic.PROPERTY_WRITE) == 0) {
            Log.w(
                    TAG,
                    "Characteristic does not support any write type! Defaulting to"
                            + " WRITE_TYPE_DEFAULT.");
        }
        return BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT;
    }

    /*!
     * @brief Get the device address (MAC address) of the connected device.
     * @return Device address, or empty string if not connected
     */
    @Keep
    @NonNull @SuppressLint("MissingPermission")
    public String getAddress() {
        synchronized (gattLock) {
            if (gatt == null) {
                return "";
            }
            BluetoothDevice device = gatt.getDevice();
            if (device == null) {
                return "";
            }
            String address = device.getAddress();
            return address == null ? "" : address;
        }
    }

    @Keep
    @NonNull @SuppressLint("MissingPermission")
    public String getName() {
        synchronized (gattLock) {
            if (gatt == null) {
                return "";
            }
            BluetoothDevice device = gatt.getDevice();
            if (device == null) {
                return "";
            }
            String name = device.getName();
            return name == null ? "" : name;
        }
    }

    /*!
     * @brief Set the connection state and notify listener if changed.
     * @param state New connection state
     */
    private void setConnectionState(ConnectionState state) {
        ConnectionState oldState;
        ConnectionStateListener listener = null;
        synchronized (connectionStateLock) {
            if (state == connectionState) {
                return;
            }
            listener = connectionStateListener;
            oldState = connectionState;
            connectionState = state;
            Log.i(TAG, "Connection state: " + oldState + " -> " + state);
        }

        /*!
         * Notify listener outside of lock to prevent deadlocks if the listener
         * calls back into this object. We capture the listener reference to avoid
         * races if setConnectionStateListener() is called concurrently.
         */
        if (listener != null) {
            try {
                listener.onConnectionStateChanged(oldState, state);
            } catch (Exception e) {
                Log.e(TAG, "Exception in connection state listener", e);
            }
        }
    }

    /*!
     * @brief Attempt to reconnect after connection loss.
     *
     * Note: This method is intentionally NOT called automatically when
     * CONNECTION_LOST is detected. Different devices require different recovery
     * strategies (e.g., device needs a RESUME command, not a reconnect).
     * The driver layer should handle recovery via ConnectionStateListener.
     *
     * This method is kept for potential future use or manual reconnection.
     */
    @SuppressWarnings("unused")
    @SuppressLint("MissingPermission")
    private void reconnect() {
        Log.i(TAG, "Attempting reconnection...");

        // ! Try to use existing GATT connection first
        synchronized (gattLock) {
            if (gatt != null) {
                try {
                    if (gatt.connect()) {
                        Log.i(TAG, "Reconnect initiated via existing GATT");
                        setConnectionState(ConnectionState.CONNECTING);
                        return;
                    }
                } catch (SecurityException e) {
                    Log.e(TAG, "SecurityException during reconnect", e);
                }
            }
        }

        // ! Fall back to full reconnection via the global connector thread
        if (connectRequest != null) {
            connectRequest.cancel();
        }
        connectRequest =
                new AndroidBleGlobals.DeviceConnectRequest(
                        serviceUuid, deviceName, majorDeviceClass, this::onDeviceFound);
        AndroidBleGlobals.getInstance().submitConnectRequest(connectRequest);
    }

    // ! Timer task that monitors connection health by checking data reception timing.
    private class CheckConnectionTask extends TimerTask {

        @Override
        public void run() {

            if (!isConnectionState(ConnectionState.RECEIVING)) {
                return;
            }

            long lastTime = -1;
            synchronized (dataTimeLock) {
                lastTime = lastDataTime;
            }

            if (lastTime == -1) {
                return;
            }

            long msSinceLastData = System.currentTimeMillis() - lastTime;
            if (msSinceLastData > CONNECTION_TIMEOUT_MS) {
                Log.w(TAG, "Connection timeout - no data for " + msSinceLastData + "ms");
                synchronized (dataTimeLock) {
                    lastDataTime = -1;
                }
                setConnectionState(ConnectionState.CONNECTION_LOST);

                /*!
                 * Note: We do NOT automatically reconnect here.
                 * a specific device could send a device-specific RESUME command,
                 * but for a generic abstraction, the driver layer should handle
                 * recovery via the ConnectionStateListener or by checking isConnected().
                 */
            }
        }
    }

    // ! GATT callback that handles all BLE events.
    private final class BleGattCallback extends BluetoothGattCallback {

        @SuppressLint("MissingPermission")
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            Log.d(
                    TAG,
                    String.format(
                            "onConnectionStateChange: status=%d newState=%d", status, newState));

            /*!
             * Per Android API docs: status indicates success/failure of the operation,
             * while newState indicates the resulting connection state.
             * Must check status first - newState may be invalid on error.
             */
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "Connection state change failed with status: " + status);
                // On error, treat as disconnected regardless of newState
                packetQueue.clear();
                setConnectionState(ConnectionState.DISCONNECTED);
                return;
            }

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                setConnectionState(ConnectionState.CONNECTED);
                if (!gatt.discoverServices()) {
                    Log.e(TAG, "Could not start service discovery");
                    setConnectionState(ConnectionState.DISCONNECTED);
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                packetQueue.clear();
                setConnectionState(ConnectionState.DISCONNECTED);
            }
        }

        @SuppressLint("MissingPermission")
        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "Service discovery failed with status: " + status);
                setConnectionState(ConnectionState.DISCONNECTED);
                return;
            }

            if (firstTimeConnection) {
                logServices(gatt);
                firstTimeConnection = false;
            }

            // Enable notifications on the configured characteristic.
            // State will transition to READY in onDescriptorWrite() when CCCD write completes.
            if (!enableNotifications(gatt)) {
                Log.e(TAG, "Failed to enable notifications, disconnecting");
                setConnectionState(ConnectionState.DISCONNECTED);
            }
        }

        /*!
         * @brief Enable notifications on the configured characteristic.
         *
         * This initiates the async GATT operation. The state will transition to READY
         * in onDescriptorWrite() when the CCCD write completes successfully.
         *
         * @param gatt The GATT connection
         * @return true if notification setup was initiated successfully, false on error
         */
        @SuppressLint("MissingPermission")
        private boolean enableNotifications(@NonNull BluetoothGatt gatt) {
            BluetoothGattService service = gatt.getService(serviceUuid);
            if (service == null) {
                Log.e(TAG, "Service not found: " + serviceUuid);
                return false;
            }

            BluetoothGattCharacteristic characteristic = service.getCharacteristic(notifyCharUuid);
            if (characteristic == null) {
                Log.e(TAG, "Notify characteristic not found: " + notifyCharUuid);
                return false;
            }

            if ((characteristic.getProperties() & BluetoothGattCharacteristic.PROPERTY_NOTIFY)
                    == 0) {
                Log.e(
                        TAG,
                        "Characteristic does not support notifications: "
                                + notifyCharUuid.toString());
                return false;
            }

            /*!
             * BLE notifications require TWO steps:
             * 1. Enable local notifications (tells Android to deliver callbacks)
             */
            if (!gatt.setCharacteristicNotification(characteristic, true)) {
                Log.e(TAG, "Failed to enable local notifications");
                return false;
            }

            /*!
             * 2. Write to the CCCD (Client Characteristic Configuration Descriptor)
             * to tell the remote device to actually send notifications.
             * Without this, the device won't transmit even though we're listening.
             */
            BluetoothGattDescriptor descriptor =
                    characteristic.getDescriptor(UUID_CONFIG_DESCRIPTOR);
            if (descriptor == null) {
                Log.e(TAG, "CCCD descriptor not found");
                return false;
            }

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                int result =
                        gatt.writeDescriptor(
                                descriptor, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                if (result != BluetoothGatt.GATT_SUCCESS) {
                    Log.e(TAG, "Failed to write CCCD descriptor: " + result);
                    return false;
                }
            } else {
                descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                if (!gatt.writeDescriptor(descriptor)) {
                    Log.e(TAG, "Failed to write CCCD descriptor");
                    return false;
                }
            }

            Log.i(TAG, "Notification setup initiated for characteristic: " + notifyCharUuid);
            return true;
        }

        @Override
        public void onDescriptorWrite(
                BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "Descriptor write failed: " + status);
                // If CCCD write failed, we can't receive notifications - disconnect
                if (UUID_CONFIG_DESCRIPTOR.equals(descriptor.getUuid())) {
                    setConnectionState(ConnectionState.DISCONNECTED);
                }
                return;
            }

            // CCCD write succeeded - device is now ready to receive notifications
            if (UUID_CONFIG_DESCRIPTOR.equals(descriptor.getUuid())) {
                Log.i(TAG, "CCCD write complete, device is ready");
                setConnectionState(ConnectionState.READY);
            }
        }

        /*!
         * API 33+ introduced a new callback signature with the value
         * passed directly, avoiding the thread-safety issue of getValue() on
         * a mutable characteristic object. We implement both for compatibility.
         */
        @Override
        public void onCharacteristicChanged(
                @NonNull BluetoothGatt gatt,
                @NonNull BluetoothGattCharacteristic characteristic,
                @NonNull byte[] value) {
            handleCharacteristicChanged(characteristic.getUuid(), value);
        }

        /*!
         * Legacy callback for API < 33. The characteristic.getValue() call here
         * is technically not thread-safe, but in practice the GATT callback thread
         * is single-threaded so it's safe.
         */
        @Override
        public void onCharacteristicChanged(
                BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            handleCharacteristicChanged(characteristic.getUuid(), characteristic.getValue());
        }

        private void handleCharacteristicChanged(UUID charUuid, byte[] value) {
            if (value == null || value.length == 0) {
                Log.w(TAG, "Received empty notification");
                return;
            }

            if (!notifyCharUuid.equals(charUuid)) {
                Log.w(TAG, "Unexpected characteristic changed: " + charUuid);
                return;
            }

            setConnectionState(ConnectionState.RECEIVING);

            synchronized (dataTimeLock) {
                lastDataTime = System.currentTimeMillis();
            }

            // Add to queue, dropping oldest if full.
            // Using while loop instead of if to handle the rare case where another
            // thread fills the queue between poll() and offer(). Both operations
            // are atomic but not combined atomically.
            while (!packetQueue.offer(value)) {
                packetQueue.poll();
            }
        }

        @Override
        public void onCharacteristicWrite(
                BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "Characteristic write failed: " + status);
            }
        }

        private void logServices(BluetoothGatt gatt) {
            List<BluetoothGattService> services = gatt.getServices();
            Log.i(TAG, String.format("Discovered %d services:", services.size()));

            for (BluetoothGattService service : services) {
                List<BluetoothGattCharacteristic> characteristics = service.getCharacteristics();
                Log.i(
                        TAG,
                        String.format(
                                "  Service %s (%d characteristics)",
                                service.getUuid(), characteristics.size()));

                for (BluetoothGattCharacteristic characteristic : characteristics) {
                    List<BluetoothGattDescriptor> descriptors = characteristic.getDescriptors();
                    Log.i(
                            TAG,
                            String.format(
                                    "    Characteristic %s (%d descriptors)",
                                    characteristic.getUuid(), descriptors.size()));
                }
            }
        }
    }
}
