package com.app.musicbike.services // Ensure this package declaration matches file location

// Android & System Imports
import android.Manifest
import android.annotation.SuppressLint // Needed for suppressing permission warnings on BLE calls
import android.app.Notification // Added
import android.app.NotificationChannel // Added
import android.app.NotificationManager // Added
import android.app.PendingIntent // Added (though not fully used in this version's notification tap)
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.content.pm.ServiceInfo // Added for foregroundServiceType on Android 14+
import android.os.Binder // For binding mechanism
import android.os.Build // For checking Android version (SDK_INT)
import android.os.Handler // For delayed tasks (like stopping scan)
import android.os.IBinder // Interface for Binder
import android.os.Looper // For Handler on main thread
import android.os.ParcelUuid // For UUID filtering
import android.util.Log // For logging

// Bluetooth Specific Imports
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor // Needed for enabling notifications (CCCD)
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings

// AndroidX Lifecycle Imports (for LiveData)
import androidx.core.app.NotificationCompat // Added
import androidx.core.content.ContextCompat
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData

// Java Imports
import java.nio.ByteBuffer // Required for binary parsing
import java.nio.ByteOrder // Required for specifying endianness
import java.util.UUID

// If your MainActivity (or other activity to open on notification tap) is in a different package,
// you might need to import it explicitly.
// import com.app.musicbike.MainActivity // Example: Your main activity

// Make sure this R class import matches YOUR app's package name if it's not resolving automatically.
// For example, if your package is com.example.myapp, it would be import com.example.myapp.R
import com.app.musicbike.R


/**
 * A background Service to manage Bluetooth Low Energy (BLE) communication.
 * Handles scanning for devices, connecting via GATT, discovering services/characteristics,
 * and communicating status/data back to the UI using LiveData.
 * Runs as a Foreground Service to ensure reliability when the app is in the background.
 */
@SuppressLint("MissingPermission") // Suppress warnings: Permissions checked in UI before calling methods
class BleService : Service() {

    // --- Constants ---
    companion object {
        private const val SCAN_PERIOD: Long = 10000 // Stops scanning after 10 seconds.
        private val TAG = "BleService" // Tag for Logcat messages
        private val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        // Service & Characteristic UUIDs (ensure these match your ESP32 definitions)
        private val MUSIC_BIKE_SERVICE_UUID: UUID = UUID.fromString("0fb899fa-2b3a-4e11-911d-4fa05d130dc1")
        private val SPEED_CHARACTERISTIC_UUID: UUID = UUID.fromString("a635fed5-9a19-4e31-8091-84d020481329")
        private val PITCH_CHARACTERISTIC_UUID: UUID = UUID.fromString("726c4b96-bc56-47d2-95a1-a6c49cce3a1f")
        private val ROLL_CHARACTERISTIC_UUID: UUID = UUID.fromString("a1e929e3-5a2e-4418-806a-c50ab877d126")
        private val YAW_CHARACTERISTIC_UUID: UUID = UUID.fromString("cd6fc0f8-089a-490e-8e36-74af84977c7b")
        private val GFORCE_CHARACTERISTIC_UUID: UUID = UUID.fromString("a6210f30-654f-32ea-9e37-432a639fb38e")
        private val EVENT_CHARACTERISTIC_UUID: UUID = UUID.fromString("26205d71-58d1-45e6-9ad1-1931cd7343c3")
        private val IMU_DIRECTION_CHARACTERISTIC_UUID: UUID = UUID.fromString("ceb04cf6-0555-4243-a27b-c85986ab4bd7")
        private val HALL_DIRECTION_CHARACTERISTIC_UUID: UUID = UUID.fromString("f231de63-475c-463d-9b3f-f338d7458bb9")
        private val IMU_SPEED_STATE_CHARACTERISTIC_UUID: UUID = UUID.fromString("738f5e54-5479-4941-ae13-caf4a9b07b2e")
        private val ACCELEROMETER_ZERO_CHARACTERISTIC_UUID: UUID = UUID.fromString("a29ff0d6-5bf9-4878-83f0-9f66a7e35a15")

        // Constants for Foreground Service Notification
        private const val NOTIFICATION_CHANNEL_ID = "BleServiceChannel"
        private const val NOTIFICATION_CHANNEL_NAME = "BLE Background Service"
        private const val ONGOING_NOTIFICATION_ID = 101 // Must be > 0

        // Optional: Action for Intent to stop the service from notification
        // const val ACTION_STOP_SERVICE = "com.app.musicbike.services.STOP_SERVICE"
    }

    // --- Bluetooth & State Variables ---
    private lateinit var bluetoothManager: BluetoothManager
    private var bluetoothAdapter: BluetoothAdapter? = null
    private val bluetoothLeScanner by lazy { bluetoothAdapter?.bluetoothLeScanner }
    private var scanning = false
    private val handler = Handler(Looper.getMainLooper())
    private val foundDevices = mutableMapOf<String, BluetoothDevice>()
    private var bluetoothGatt: BluetoothGatt? = null
    private var connectionAttemptAddress: String? = null
    private val notificationQueue = ArrayDeque<BluetoothGattCharacteristic>()
    @Volatile
    private var isProcessingQueue = false

    // For Foreground Service Notification
    private lateinit var notificationManager: NotificationManager


    // --- LiveData for UI Communication ---
    private val _connectionStatus = MutableLiveData("Idle").apply {
        // Observe changes to update notification content.
        // For a long-running service, ensure this observer doesn't cause issues if the
        // service is ever fully destroyed and recreated in complex scenarios.
        // However, for a persistent foreground service, this is generally acceptable.
        observeForever { status ->
            if(this@BleService::notificationManager.isInitialized) { // Check if service is ready to update notification
                updateNotificationContent("Status: $status")
            }
        }
    }
    private val _scanResults = MutableLiveData<List<BluetoothDevice>>(emptyList())
    private val _speed = MutableLiveData(0.0f)
    private val _pitch = MutableLiveData(0.0f)
    private val _roll = MutableLiveData(0.0f)
    private val _yaw = MutableLiveData(0.0f)
    private val _lastEvent = MutableLiveData("NONE")
    private val _imuDirection = MutableLiveData(1)
    private val _hallDirection = MutableLiveData(1)
    private val _imuSpeedState = MutableLiveData(0)
    private val _gForce = MutableLiveData(0.0f)

    private val eventTimeoutHandler = Handler(Looper.getMainLooper())
    private val EVENT_TIMEOUT_MS = 3000L
    private val resetEventRunnable = Runnable { _lastEvent.postValue("NONE") }

    val connectionStatus: LiveData<String> get() = _connectionStatus
    val scanResults: LiveData<List<BluetoothDevice>> get() = _scanResults
    val speed: LiveData<Float> get() = _speed
    val pitch: LiveData<Float> get() = _pitch
    val roll: LiveData<Float> get() = _roll
    val yaw: LiveData<Float> get() = _yaw
    val lastEvent: LiveData<String> get() = _lastEvent
    val imuDirection: LiveData<Int> get() = _imuDirection
    val hallDirection: LiveData<Int> get() = _hallDirection
    val imuSpeedState: LiveData<Int> get() = _imuSpeedState
    val gForce: LiveData<Float> get() = _gForce

    // --- Binder ---
    private val binder = LocalBinder()
    inner class LocalBinder : Binder() {
        fun getService(): BleService = this@BleService
    }

    // --- Service Lifecycle Methods ---
    override fun onCreate() {
        super.onCreate()
        Log.d(TAG, "onCreate: Service instance created.")
        notificationManager = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        createNotificationChannel()
        if (!initializeBluetooth()) {
            Log.e(TAG, "onCreate: Failed to initialize Bluetooth. Stopping service.")
            stopSelf()
        }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        Log.d(TAG, "onStartCommand received.")

        // if (intent?.action == ACTION_STOP_SERVICE) {
        //     Log.d(TAG, "Received stop service action from notification.")
        //     disconnectAndStopService()
        //     return START_NOT_STICKY
        // }

        val initialNotificationText = _connectionStatus.value ?: "BLE service starting..."
        val notification = buildNotification(initialNotificationText)

        // Check if we have ALL required permissions for FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE
        val hasConnectPermission = hasConnectPermission()
        val hasScanPermission = hasScanPermission()
        val hasAdvertisePermission = hasPermission(Manifest.permission.BLUETOOTH_ADVERTISE)
        val hasForegroundServicePermission = hasPermission(Manifest.permission.FOREGROUND_SERVICE_CONNECTED_DEVICE)
        
        Log.d(TAG, "Permission check - Connect: $hasConnectPermission, Scan: $hasScanPermission, Advertise: $hasAdvertisePermission, ForegroundService: $hasForegroundServicePermission")
        
        // For CONNECTED_DEVICE type, we need:
        // 1. FOREGROUND_SERVICE_CONNECTED_DEVICE (required)
        // 2. At least one of: BLUETOOTH_ADVERTISE, BLUETOOTH_CONNECT, BLUETOOTH_SCAN, etc.
        val hasRequiredPermissions = hasForegroundServicePermission && 
            (hasConnectPermission || hasScanPermission || hasAdvertisePermission)

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q && hasRequiredPermissions) {
            try {
                Log.d(TAG, "Attempting to start foreground service with CONNECTED_DEVICE type")
                startForeground(
                    ONGOING_NOTIFICATION_ID,
                    notification,
                    ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE
                )
                Log.i(TAG, "Service started in foreground with CONNECTED_DEVICE type.")
            } catch (e: SecurityException) {
                Log.w(TAG, "Failed to start with CONNECTED_DEVICE type, falling back to default", e)
                try {
                    startForeground(ONGOING_NOTIFICATION_ID, notification)
                    Log.i(TAG, "Service started in foreground with default type.")
                } catch (e2: SecurityException) {
                    Log.e(TAG, "Failed to start foreground service even with default type", e2)
                    // If we can't start as foreground service at all, stop the service
                    stopSelf()
                    return START_NOT_STICKY
                }
            }
        } else {
            Log.d(TAG, "Starting foreground service with default type (missing permissions or Android < Q)")
            try {
                startForeground(ONGOING_NOTIFICATION_ID, notification)
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                    Log.i(TAG, "Service started in foreground with default type (missing permissions).")
                } else {
                    Log.i(TAG, "Service started in foreground (Android < Q).")
                }
            } catch (e: SecurityException) {
                Log.e(TAG, "Failed to start foreground service with default type", e)
                stopSelf()
                return START_NOT_STICKY
            }
        }
        return START_STICKY
    }

    private fun initializeBluetooth(): Boolean {
        bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager
            ?: run {
                Log.e(TAG, "initializeBluetooth: Unable to initialize BluetoothManager.")
                return false
            }
        bluetoothAdapter = bluetoothManager.adapter
        if (bluetoothAdapter == null) {
            Log.e(TAG, "initializeBluetooth: Bluetooth not supported on this device.")
            return false
        }
        return true
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val serviceChannel = NotificationChannel(
                NOTIFICATION_CHANNEL_ID,
                NOTIFICATION_CHANNEL_NAME,
                NotificationManager.IMPORTANCE_LOW
            ).apply {
                description = "Channel for Music Bike BLE Service"
            }
            notificationManager.createNotificationChannel(serviceChannel)
            Log.d(TAG, "Notification channel created.")
        }
    }

    private fun buildNotification(contentText: String = "BLE service running"): Notification {
        // val notificationIntent = Intent(this, MainActivity::class.java) // Replace MainActivity if needed
        // val pendingIntent = PendingIntent.getActivity(
        //     this,
        //     0,
        //     notificationIntent,
        //     PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
        // )

        // IMPORTANT: Ensure 'ic_ble_notification' exists in your res/drawable folder
        // And that 'com.app.musicbike.R' correctly resolves to your app's R class.
        val iconResId = R.drawable.ic_ble_notification

        return NotificationCompat.Builder(this, NOTIFICATION_CHANNEL_ID)
            .setContentTitle("Music Bike Service")
            .setContentText(contentText)
            .setSmallIcon(iconResId)
            // .setContentIntent(pendingIntent) // Uncomment to open app on notification tap
            .setOngoing(true)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .build()
    }

    private fun updateNotificationContent(contentText: String) {
        if (::notificationManager.isInitialized) {
            val notification = buildNotification(contentText)
            notificationManager.notify(ONGOING_NOTIFICATION_ID, notification)
            Log.d(TAG, "Notification updated: $contentText")
        } else {
            Log.w(TAG, "NotificationManager not initialized, cannot update notification.")
        }
    }

    override fun onBind(intent: Intent): IBinder {
        Log.d(TAG, "onBind: Service binding requested. Foreground service may be running.")
        if (bluetoothAdapter == null) {
            initializeBluetooth()
        }
        return binder
    }

    override fun onUnbind(intent: Intent?): Boolean {
        Log.w(TAG, "onUnbind: Service unbound.")
        // For a foreground service designed to run independently,
        // do not stop it or close GATT here unless specifically intended
        // (e.g., if no clients are bound AND no active BLE connection is desired to persist).
        return super.onUnbind(intent)
    }

    override fun onDestroy() {
        super.onDestroy()
        Log.w(TAG, "onDestroy: Service destroyed.")
        // Clean up the observer from LiveData if it was added with observeForever
        // _connectionStatus.removeObserver { status -> ... } // Requires holding a reference to the observer lambda

        stopForeground(true) // true = remove notification
        handler.removeCallbacksAndMessages(null)
        stopScan()
        closeGatt()
    }

    override fun onTaskRemoved(rootIntent: Intent?) {
        Log.d(TAG, "onTaskRemoved called for BleService - app task swiped away.")

        // Gracefully shut down BLE operations
        Log.i(TAG, "Stopping scan, disconnecting, and closing GATT due to task removal.")
        stopScan()   // Call your existing stopScan() method
        disconnect() // Call your existing disconnect() method
        closeGatt()  // Call your existing closeGatt() method

        // Stop foreground state and remove notification
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) { // Android 7.0 (Nougat) / API 24
            stopForeground(Service.STOP_FOREGROUND_REMOVE)
        } else {
            @Suppress("DEPRECATION")
            stopForeground(true)
        }

        stopSelf() // Stop the service itself
        Log.i(TAG, "BleService stopped due to task removal.")

        super.onTaskRemoved(rootIntent)
    }

    fun disconnectAndStopService() {
        Log.i(TAG, "disconnectAndStopService called.")
        disconnect()
        closeGatt() // is usually called by disconnect's onConnectionStateChange,
        // but ensure resources are released if disconnect() doesn't complete fully.
        // If closeGatt also handles _connectionStatus.postValue("Disconnected"), it's fine.
        // Explicitly calling closeGatt here ensures cleanup if disconnect() itself doesn't fully manage it.
        // However, be mindful of redundant calls or state changes if closeGatt is robustly called
        // from onConnectionStateChange when a disconnect occurs.
        // For simplicity now, we assume disconnect() leads to closeGatt() or handles status.
        // The critical parts are stopping foreground and self.
        stopForeground(true)
        stopSelf()
    }

    // --- Permission Check Helpers --- (Original - Unchanged)
    private fun hasPermission(permission: String): Boolean {
        return ContextCompat.checkSelfPermission(this, permission) == PackageManager.PERMISSION_GRANTED
    }

    private fun hasScanPermission(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            hasPermission(Manifest.permission.BLUETOOTH_SCAN)
        } else {
            hasPermission(Manifest.permission.ACCESS_FINE_LOCATION)
        }
    }

    private fun hasConnectPermission(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            hasPermission(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            true
        }
    }

    // --- BLE Scan Operations --- (Original - Unchanged, _connectionStatus will trigger notification updates)
    fun startScan() {
        Log.d(TAG, "startScan requested.")
        if (!hasScanPermission()) {
            Log.e(TAG, "startScan: Scan permissions missing.")
            _connectionStatus.postValue("Error: Permissions Missing")
            return
        }
        if (bluetoothAdapter?.isEnabled != true) {
            Log.e(TAG, "startScan: Bluetooth not enabled or adapter unavailable.")
            _connectionStatus.postValue("Error: Bluetooth not enabled")
            return
        }
        if (scanning) {
            Log.d(TAG, "startScan: Scan already in progress.")
            return
        }
        Log.d(TAG, "startScan: Starting BLE scan...")
        scanning = true
        foundDevices.clear()
        _scanResults.postValue(emptyList())
        _connectionStatus.postValue("Scanning for Music Bike...")

        handler.removeCallbacksAndMessages(null)
        handler.postDelayed({ if (scanning) stopScan() }, SCAN_PERIOD)

        val scanFilter = ScanFilter.Builder()
            .setServiceUuid(ParcelUuid(MUSIC_BIKE_SERVICE_UUID))
            .build()
        val scanFilters: List<ScanFilter> = listOf(scanFilter)
        val scanSettings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        try {
            bluetoothLeScanner?.startScan(scanFilters, scanSettings, leScanCallback)
                ?: Log.e(TAG, "startScan: bluetoothLeScanner is null, cannot start scan.")
        } catch (e: SecurityException) {
            Log.e(TAG, "startScan: SecurityException.", e)
            _connectionStatus.postValue("Error: Scan Permission Denied")
            scanning = false
        } catch (e: IllegalStateException) {
            Log.e(TAG, "startScan: IllegalStateException.", e)
            _connectionStatus.postValue("Error: Bluetooth Off?")
            scanning = false
        }
    }

    fun stopScan() {
        if (!scanning) return
        Log.d(TAG, "stopScan: Stopping BLE Scan...")
        scanning = false
        if (_connectionStatus.value == "Scanning for Music Bike...") {
            _connectionStatus.postValue("Scan Finished")
        }

        if (!hasScanPermission()) {
            Log.e(TAG, "stopScan: Missing required permissions to stop scan.")
            _connectionStatus.postValue("Error: Scan Stop Permission Missing")
            return
        }
        try {
            bluetoothLeScanner?.stopScan(leScanCallback)
                ?: Log.e(TAG, "stopScan: bluetoothLeScanner is null, cannot stop scan.")
        } catch (e: SecurityException) {
            Log.e(TAG, "stopScan: SecurityException.", e)
            _connectionStatus.postValue("Error: Scan Permission Denied")
        } catch (e: IllegalStateException) {
            Log.e(TAG, "stopScan: IllegalStateException.", e)
            _connectionStatus.postValue("Error: Bluetooth Off?")
        } finally {
            handler.removeCallbacksAndMessages(null)
        }
    }

    private val leScanCallback = object : ScanCallback() { // (Original - Unchanged)
        override fun onScanResult(callbackType: Int, result: ScanResult?) {
            val device = result?.device ?: return
            try {
                val deviceAddress = device.address
                if (!foundDevices.containsKey(deviceAddress)) {
                    val deviceName = try { device.name ?: "Unknown Device" } catch (se: SecurityException) { "No Permission" }
                    Log.i(TAG, "Device found: $deviceName ($deviceAddress)")
                    foundDevices[deviceAddress] = device
                    _scanResults.postValue(foundDevices.values.toList())
                }
            } catch (e: Exception) {
                Log.e(TAG, "onScanResult: Error processing scan result for ${device.address}", e)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "onScanFailed: BLE Scan Failed with error code: $errorCode")
            scanning = false
            _connectionStatus.postValue("Error: Scan Failed ($errorCode)")
        }
    }

    // --- GATT Connection Operations ---
    // (connect, disconnect, closeGatt updated to ensure _connectionStatus.postValue triggers notification updates)
    fun connect(address: String): Boolean {
        Log.d(TAG, "connect: Attempting connection to address: $address")
        if (bluetoothAdapter?.isEnabled != true) {
            Log.e(TAG, "connect: Bluetooth not enabled or adapter unavailable.")
            _connectionStatus.postValue("Error: Bluetooth not enabled")
            return false
        }
        if (!hasConnectPermission()) {
            Log.e(TAG, "connect: Missing BLUETOOTH_CONNECT permission.")
            _connectionStatus.postValue("Error: Connect Permission Missing")
            return false
        }

        bluetoothGatt?.device?.address?.let { existingAddress ->
            if (existingAddress != address) {
                Log.w(TAG, "connect: Disconnecting from $existingAddress before connecting to $address.")
                closeGatt()
            } else {
                Log.w(TAG, "connect: Already connected or attempting connection to $address.")
                // If already connected or attempting to this device, reflect current status or allow attempt to proceed.
                // This might need more nuanced logic based on exact state. For now, assume a new attempt is okay if not connected.
                if (bluetoothGatt != null && bluetoothGatt?.device?.address == address && _connectionStatus.value?.startsWith("Connected") == true) {
                    return false // Already connected
                }
            }
        }

        val device: BluetoothDevice = try {
            bluetoothAdapter!!.getRemoteDevice(address)
        } catch (e: IllegalArgumentException) {
            Log.e(TAG, "connect: Invalid Bluetooth address format: $address", e)
            _connectionStatus.postValue("Error: Invalid address")
            return false
        } catch (e: Exception) {
            Log.e(TAG, "connect: Failed to get remote device $address", e)
            _connectionStatus.postValue("Error: Device retrieval failed")
            return false
        }

        stopScan()

        val deviceName = try { device.name ?: address } catch (se: SecurityException) { address }
        _connectionStatus.postValue("Connecting to $deviceName...") // Update status & notification
        connectionAttemptAddress = address

        bluetoothGatt = try {
            device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        } catch (e: SecurityException) {
            Log.e(TAG, "connect: SecurityException on connectGatt.", e)
            _connectionStatus.postValue("Error: Connect Permission Denied")
            connectionAttemptAddress = null
            null
        } catch (e: Exception) {
            Log.e(TAG, "connect: Exception on connectGatt.", e)
            _connectionStatus.postValue("Error: Connection Failed")
            connectionAttemptAddress = null
            null
        }

        if (bluetoothGatt == null) {
            Log.e(TAG, "connect: connectGatt failed to return a BluetoothGatt instance.")
            connectionAttemptAddress = null
            // Status likely set in catch blocks
            return false
        }

        Log.d(TAG, "connect: connectGatt call initiated.")
        return true
    }

    fun disconnect() {
        val gatt = bluetoothGatt ?: run {
            Log.w(TAG, "disconnect: Not connected (bluetoothGatt is null).")
            if (_connectionStatus.value != "Idle" && _connectionStatus.value != "Disconnected" && !_connectionStatus.value.orEmpty().startsWith("Error:")) {
                _connectionStatus.postValue("Disconnected")
            }
            return
        }
        val address = gatt.device?.address ?: "unknown address"
        Log.d(TAG, "disconnect: Requesting disconnection from $address")

        if (!hasConnectPermission()) {
            Log.e(TAG, "disconnect: Missing BLUETOOTH_CONNECT permission.")
            _connectionStatus.postValue("Error: Disconnect Permission Missing")
            return
        }

        try {
            gatt.disconnect() // Status update will happen in onConnectionStateChange
            Log.d(TAG, "disconnect: disconnect() called for $address.")
        } catch (e: SecurityException) {
            Log.e(TAG, "disconnect: SecurityException.", e)
            _connectionStatus.postValue("Error: Disconnect Permission Denied")
        } catch (e: Exception) {
            Log.e(TAG, "disconnect: Exception.", e)
            _connectionStatus.postValue("Error: Disconnect Failed")
        }
    }

    private fun closeGatt() {
        val gatt = bluetoothGatt ?: return
        val address = gatt.device?.address ?: "unknown address"
        Log.d(TAG, "closeGatt: Closing GATT client for $address")

        if (!hasConnectPermission()) {
            Log.e(TAG, "closeGatt: Missing BLUETOOTH_CONNECT permission.")
            //_connectionStatus.postValue("Error: Disconnect Permission Missing") // Avoid overwriting specific disconnect error
            bluetoothGatt = null // Still clear the reference
            return
        }

        try {
            gatt.close()
            Log.d(TAG, "closeGatt: GATT client closed for $address.")
        } catch (e: SecurityException) {
            Log.e(TAG, "closeGatt: SecurityException.", e)
            // _connectionStatus.postValue("Error: Disconnect Permission Denied")
        } catch (e: Exception) {
            Log.e(TAG, "closeGatt: Exception.", e)
            // _connectionStatus.postValue("Error: GATT Close Failed")
        } finally {
            bluetoothGatt = null
            // Only update status if not already an error or explicitly disconnected by callback
            val currentStatus = _connectionStatus.value
            if (currentStatus != null && currentStatus != "Disconnected" && !currentStatus.startsWith("Error:")) {
                //If onConnectionStateChange didn't set it to "Disconnected" (e.g. direct closeGatt call)
                if (currentStatus != "Idle") _connectionStatus.postValue("Disconnected")
            }
            // Consider if stopForeground/stopSelf should be called here based on app logic
            // For example, if the service's only purpose is an active connection:
            // Log.d(TAG, "closeGatt: Connection closed. Stopping foreground service.")
            // stopForeground(true)
            // stopSelf()
        }
    }


    // --- GATT Callback Implementation ---
    // (onConnectionStateChange, onServicesDiscovered, onCharacteristicChanged will trigger _connectionStatus or data LiveData updates,
    // which in turn can update notification via the _connectionStatus observer)
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt?, status: Int, newState: Int) {
            val deviceAddress = gatt?.device?.address
            Log.d(TAG, "onConnectionStateChange: Address=$deviceAddress, Status=$status, NewState=$newState")

            if (connectionAttemptAddress != null && deviceAddress != connectionAttemptAddress) {
                Log.w(TAG, "onConnectionStateChange: Ignoring callback for unexpected address $deviceAddress (expecting $connectionAttemptAddress)")
                // gatt?.close() // Optionally close unexpected connections
                return
            }

            if (status == BluetoothGatt.GATT_SUCCESS) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    val connectedDeviceName = try { gatt?.device?.name ?: deviceAddress ?: "Device" } catch (se: SecurityException) { deviceAddress ?: "Device" }
                    Log.i(TAG, "onConnectionStateChange: Successfully connected to $connectedDeviceName")
                    _connectionStatus.postValue("Connected to $connectedDeviceName")
                    bluetoothGatt = gatt
                    connectionAttemptAddress = null

                    Handler(Looper.getMainLooper()).postDelayed({
                        bluetoothGatt?.let { currentGatt ->
                            if (hasConnectPermission()) {
                                Log.d(TAG, "onConnectionStateChange: Initiating service discovery...")
                                if (currentGatt.discoverServices()) {
                                    _connectionStatus.postValue("Discovering Services...")
                                } else {
                                    Log.e(TAG, "onConnectionStateChange: discoverServices() failed to initiate.")
                                    _connectionStatus.postValue("Error: Discovery Failed Start")
                                    disconnect()
                                }
                            } else {
                                Log.e(TAG, "onConnectionStateChange: Missing Connect permission for service discovery.")
                                _connectionStatus.postValue("Error: Discovery Permission")
                                disconnect()
                            }
                        } ?: Log.e(TAG, "onConnectionStateChange: bluetoothGatt became null before discovery.")
                    }, 100)

                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    Log.i(TAG, "onConnectionStateChange: Disconnected from $deviceAddress (Status: GATT_SUCCESS)")
                    _connectionStatus.postValue("Disconnected")
                    closeGatt() // Clean up resources
                    connectionAttemptAddress = null
                }
            } else {
                Log.e(TAG, "onConnectionStateChange: GATT Error! Status: $status, Addr: $deviceAddress, NewState: $newState")
                _connectionStatus.postValue("Error: Connection Failed ($status)")
                closeGatt()
                connectionAttemptAddress = null
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt?, status: Int) {
            // (Original logic - _connectionStatus changes will update notification)
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val address = gatt?.device?.address
                Log.i(TAG, "onServicesDiscovered: Services discovered successfully for $address")
                _connectionStatus.postValue("Services Discovered")

                val service = gatt?.getService(MUSIC_BIKE_SERVICE_UUID)
                if (service == null) {
                    Log.e(TAG, "onServicesDiscovered: Music Bike Service UUID $MUSIC_BIKE_SERVICE_UUID not found!")
                    _connectionStatus.postValue("Error: Service Not Found")
                    disconnect()
                    return
                }

                Log.d(TAG, "onServicesDiscovered: Queueing characteristics for notifications...")
                notificationQueue.clear()
                isProcessingQueue = false

                service.getCharacteristic(SPEED_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(PITCH_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(ROLL_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(YAW_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(EVENT_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(IMU_DIRECTION_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(HALL_DIRECTION_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(IMU_SPEED_STATE_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }
                service.getCharacteristic(GFORCE_CHARACTERISTIC_UUID)?.also { notificationQueue.addLast(it) }

                service.getCharacteristic(ACCELEROMETER_ZERO_CHARACTERISTIC_UUID)?.also {
                    Log.d(TAG, "onServicesDiscovered: Found accelerometer zero characteristic")
                } ?: Log.w(TAG, "onServicesDiscovered: Accelerometer zero characteristic not found")

                if (notificationQueue.isNotEmpty()) {
                    Log.d(TAG, "onServicesDiscovered: Starting notification queue processing.")
                    processNotificationQueue()
                } else {
                    Log.w(TAG, "onServicesDiscovered: Notification queue is empty.")
                    _connectionStatus.postValue("Ready (No Notifiable Characteristics)")
                }

            } else {
                Log.e(TAG, "onServicesDiscovered failed with status: $status")
                _connectionStatus.postValue("Error: Service Discovery Failed ($status)")
                disconnect()
            }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt?, descriptor: BluetoothGattDescriptor?, status: Int) {
            // (Original logic)
            Log.d(TAG, "onDescriptorWrite ----> START <---- Char: ${descriptor?.characteristic?.uuid}, Status: $status")
            val characteristicUUID = descriptor?.characteristic?.uuid ?: "unknown characteristic"
            if (descriptor?.uuid == CCCD_UUID) {
                if (status == BluetoothGatt.GATT_SUCCESS) {
                    Log.d(TAG, "onDescriptorWrite: CCCD write success for $characteristicUUID")
                } else {
                    Log.e(TAG, "onDescriptorWrite: CCCD write FAILED for $characteristicUUID, Status: $status")
                }
            } else {
                Log.d(TAG, "onDescriptorWrite: Callback for non-CCCD descriptor ${descriptor?.uuid}, Status: $status")
            }
            isProcessingQueue = false
            processNotificationQueue()
            Log.d(TAG, "onDescriptorWrite ----> END <---- Char: $characteristicUUID")
        }


        override fun onCharacteristicChanged(
            gatt: BluetoothGatt, // Use this signature (gatt is non-null here)
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray // Use this signature available from API 33+, or the deprecated one for older
        ) {
            // Fallback for older Android versions might be needed if you use the deprecated onCharacteristicChanged
            // This signature is for Android 13 (API 33) and above.
            // If targeting lower, ensure your gattCallback's onCharacteristicChanged matches the framework's.
            // The framework calls the correct one based on API level.
            // For compatibility, it's often easier to use the deprecated version:
            // override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic)
            // and then access characteristic.value
            // Let's assume the `value` parameter is correctly populated.

            val uuid = characteristic.uuid
            // val hexString = value.joinToString("") { "%02X".format(it) }
            // Log.d(TAG, "onCharacteristicChanged UUID: $uuid, Size: ${value.size}, Raw Hex: $hexString, Value: ${characteristic.value.contentToString()}")

            // Use characteristic.value if the 'value' param isn't working as expected or for wider compatibility
            val currentBytes = characteristic.value ?: value // Prefer characteristic.value if robust

            try {
                when (uuid) {
                    SPEED_CHARACTERISTIC_UUID -> {
                        if (currentBytes.size == 4) {
                            val buffer = ByteBuffer.wrap(currentBytes).order(ByteOrder.LITTLE_ENDIAN)
                            _speed.postValue(buffer.float)
                        } else Log.w(TAG, "Incorrect byte count for Speed: ${currentBytes.size}")
                    }
                    PITCH_CHARACTERISTIC_UUID -> {
                        if (currentBytes.size == 4) {
                            val buffer = ByteBuffer.wrap(currentBytes).order(ByteOrder.LITTLE_ENDIAN)
                            _pitch.postValue(buffer.float)
                        } else Log.w(TAG, "Incorrect byte count for Pitch: ${currentBytes.size}")
                    }
                    ROLL_CHARACTERISTIC_UUID -> {
                        if (currentBytes.size == 4) {
                            val buffer = ByteBuffer.wrap(currentBytes).order(ByteOrder.LITTLE_ENDIAN)
                            _roll.postValue(buffer.float)
                        } else Log.w(TAG, "Incorrect byte count for Roll: ${currentBytes.size}")
                    }
                    YAW_CHARACTERISTIC_UUID -> {
                        if (currentBytes.size == 4) {
                            val buffer = ByteBuffer.wrap(currentBytes).order(ByteOrder.LITTLE_ENDIAN)
                            _yaw.postValue(buffer.float)
                        } else Log.w(TAG, "Incorrect byte count for Yaw: ${currentBytes.size}")
                    }
                    GFORCE_CHARACTERISTIC_UUID -> {
                        if (currentBytes.size == 4) {
                            val buffer = ByteBuffer.wrap(currentBytes).order(ByteOrder.LITTLE_ENDIAN)
                            _gForce.postValue(buffer.float)
                        } else Log.w(TAG, "Incorrect byte count for G-Force: ${currentBytes.size}")
                    }
                    EVENT_CHARACTERISTIC_UUID -> {
                        if (currentBytes.isNotEmpty()) {
                            val eventCode = currentBytes[0].toInt() and 0xFF
                            val eventString = when(eventCode) {
                                1 -> "JUMP"
                                2 -> "DROP"
                                3 -> "180"
                                else -> null
                            }
                            eventString?.let { _lastEvent.postValue(it) }
                        } else Log.w(TAG, "Empty data for Event")
                    }
                    IMU_DIRECTION_CHARACTERISTIC_UUID -> {
                        if (currentBytes.isNotEmpty()) _imuDirection.postValue(currentBytes[0].toInt() and 0xFF)
                        else Log.w(TAG, "Empty data for IMU Direction")
                    }
                    HALL_DIRECTION_CHARACTERISTIC_UUID -> {
                        if (currentBytes.isNotEmpty()) _hallDirection.postValue(currentBytes[0].toInt() and 0xFF)
                        else Log.w(TAG, "Empty data for Hall Direction")
                    }
                    IMU_SPEED_STATE_CHARACTERISTIC_UUID -> {
                        if (currentBytes.isNotEmpty()) _imuSpeedState.postValue(currentBytes[0].toInt() and 0xFF)
                        else Log.w(TAG, "Empty data for IMU Speed State")
                    }
                    else -> Log.w(TAG, "Unhandled characteristic: $uuid")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error parsing characteristic $uuid", e)
            }
        }
    }

    // --- Helper Function to Enable Notifications (Queue-based) --- (Original - Unchanged)
    private fun processNotificationQueue() {
        val gatt = bluetoothGatt
        if (gatt == null) {
            Log.e(TAG, "processNotificationQueue: GATT is null! Cannot process queue.")
            synchronized(notificationQueue) { isProcessingQueue = false }
            return
        }

        var characteristicToProcess: BluetoothGattCharacteristic? = null
        var isQueueEmpty = false

        synchronized(notificationQueue) {
            if (isProcessingQueue) {
                Log.d(TAG, "processNotificationQueue: Exiting - Already processing.")
                return
            }
            if (notificationQueue.isEmpty()) {
                isQueueEmpty = true
            } else {
                characteristicToProcess = notificationQueue.removeFirst()
                isProcessingQueue = true
                Log.d(TAG, "processNotificationQueue: Processing ${characteristicToProcess?.uuid}")
            }
        }

        if (isQueueEmpty) {
            if (!isProcessingQueue) { // Double check flag outside sync, should be false if queue was empty
                Log.i(TAG, "processNotificationQueue: Queue empty. Setting status to Ready.")
                // Set a more specific ready status if some notifications were enabled vs none
                if (_connectionStatus.value == "Discovering Services..." || _connectionStatus.value == "Services Discovered") {
                    _connectionStatus.postValue("Ready")
                } else if (_connectionStatus.value?.startsWith("Connected") == true && notificationQueue.isEmpty()) {
                    // If already connected and queue emptied, assume ready
                    _connectionStatus.postValue("Ready")
                }
            }
            return
        }

        val characteristic = characteristicToProcess ?: run {
            Log.w(TAG, "processNotificationQueue: Dequeued characteristic was null.")
            isProcessingQueue = false
            processNotificationQueue()
            return
        }

        if (!hasConnectPermission()) {
            Log.e(TAG, "processNotificationQueue: Missing CONNECT permission for ${characteristic.uuid}.")
            isProcessingQueue = false; return
        }
        if ((characteristic.properties and BluetoothGattCharacteristic.PROPERTY_NOTIFY) == 0) {
            Log.w(TAG, "processNotificationQueue: ${characteristic.uuid} does not support NOTIFY. Skipping.")
            isProcessingQueue = false; processNotificationQueue(); return
        }
        if (!gatt.setCharacteristicNotification(characteristic, true)) {
            Log.e(TAG, "processNotificationQueue: setCharacteristicNotification(true) failed for ${characteristic.uuid}")
            isProcessingQueue = false; processNotificationQueue(); return
        }
        val descriptor = characteristic.getDescriptor(CCCD_UUID)
        if (descriptor == null) {
            Log.e(TAG, "processNotificationQueue: CCCD not found for ${characteristic.uuid}.")
            isProcessingQueue = false; processNotificationQueue(); return
        }

        Log.d(TAG, "processNotificationQueue: Writing ENABLE_NOTIFICATION_VALUE to CCCD for ${characteristic.uuid}")
        if (!writeCccd(gatt, descriptor, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)) {
            Log.e(TAG, "processNotificationQueue: Failed to initiate CCCD write for ${characteristic.uuid}")
            isProcessingQueue = false
            processNotificationQueue()
        } else {
            Log.d(TAG, "processNotificationQueue: CCCD write initiated. Waiting for onDescriptorWrite.")
        }
    }

    private fun writeCccd(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, value: ByteArray): Boolean {
        // (Original logic)
        if (!hasConnectPermission()) {
            Log.e(TAG, "writeCccd: Missing Connect permission for ${descriptor.characteristic?.uuid}")
            return false
        }
        return try {
            val initiated: Boolean
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                val status = gatt.writeDescriptor(descriptor, value)
                initiated = status == BluetoothGatt.GATT_SUCCESS
                if (!initiated) Log.e(TAG, "writeCccd (API 33+): writeDescriptor failed with status $status")
            } else {
                @Suppress("DEPRECATION")
                descriptor.value = value
                @Suppress("DEPRECATION")
                initiated = gatt.writeDescriptor(descriptor)
            }
            initiated
        } catch (e: SecurityException) {
            Log.e(TAG, "writeCccd: SecurityException for ${descriptor.characteristic?.uuid}", e); false
        } catch (e: Exception) {
            Log.e(TAG, "writeCccd: Exception for ${descriptor.characteristic?.uuid}", e); false
        }
    }

    // --- Add new function to zero the accelerometer --- (Original - Unchanged)
    fun zeroAccelerometer(): Boolean {
        val gatt = bluetoothGatt ?: run {
            Log.e(TAG, "zeroAccelerometer: Not connected")
            return false
        }
        if (!hasConnectPermission()) {
            Log.e(TAG, "zeroAccelerometer: Missing BLUETOOTH_CONNECT permission")
            return false
        }
        val service = gatt.getService(MUSIC_BIKE_SERVICE_UUID)
        val characteristic = service?.getCharacteristic(ACCELEROMETER_ZERO_CHARACTERISTIC_UUID)
        if (characteristic == null) {
            Log.e(TAG, "zeroAccelerometer: Characteristic not found")
            return false
        }
        try {
            characteristic.value = byteArrayOf(1) // Write a non-zero value
            characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            val writeSuccess = gatt.writeCharacteristic(characteristic)
            if (writeSuccess) Log.i(TAG, "zeroAccelerometer: Write request sent")
            else Log.e(TAG, "zeroAccelerometer: Write request failed")
            return writeSuccess
        } catch (e: SecurityException) {
            Log.e(TAG, "zeroAccelerometer: SecurityException", e); return false
        } catch (e: Exception) {
            Log.e(TAG, "zeroAccelerometer: Exception", e); return false
        }
    }
} // End of BleService class