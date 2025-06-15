package com.app.musicbike.services

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service // Make sure Service is imported for STOP_FOREGROUND_REMOVE
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.SharedPreferences
import android.content.pm.ServiceInfo
import android.os.Binder
import android.os.Build
import android.os.IBinder
import android.os.Looper
import android.os.Handler
import android.util.Log
import androidx.core.app.NotificationCompat
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.Observer
import com.app.musicbike.R // Make sure R is imported correctly

class MusicService : Service() {

    private val TAG = "MusicService"
    private val binder = LocalBinder()
    private lateinit var notificationManager: NotificationManager
    private lateinit var prefs: SharedPreferences
    private var isInferenceActive = false // Added flag

    // --- FMOD Native Interface ---
    companion object {
        private const val NOTIFICATION_CHANNEL_ID = "MusicServiceChannel"
        private const val NOTIFICATION_CHANNEL_NAME = "Music Playback Service"
        private const val ONGOING_NOTIFICATION_ID = 102

        // Actions for controlling InferenceService
        const val ACTION_START_INFERENCE = "com.app.musicbike.services.action.START_INFERENCE" // Added
        const val ACTION_STOP_INFERENCE = "com.app.musicbike.services.action.STOP_INFERENCE"   // Added

        // SharedPreferences Keys for Stats
        private const val PREFS_NAME = "MusicBikeRideStats"
        private const val KEY_MAX_SPEED = "maxSpeed"
        private const val KEY_MAX_POSITIVE_PITCH = "maxPositivePitch"
        private const val KEY_MIN_NEGATIVE_PITCH = "minNegativePitch" // Will store as positive value
        private const val KEY_JUMP_COUNT = "jumpCount"
        private const val KEY_DROP_COUNT = "dropCount"
        private const val KEY_180_COUNT = "oneEightyCount"

        init {
            try {
                Log.d("FMOD_MusicService", "Attempting to load FMOD libraries from MusicService...")
                System.loadLibrary("fmod")
                System.loadLibrary("fmodstudio")
                System.loadLibrary("musicbike")
                Log.d("FMOD_MusicService", "FMOD libraries loaded successfully by MusicService.")
            } catch (e: UnsatisfiedLinkError) {
                Log.e("FMOD_MusicService", "Failed to load FMOD libraries in MusicService.", e)
            }
        }
    }

    private external fun nativeStartFMODPlayback(masterBankPath: String, stringsBankPath: String)
    private external fun nativeSetFMODParameter(paramName: String, value: Float)
    private external fun nativeToggleFMODPlayback()
    private external fun nativePlayFMODEvent()
    private external fun nativeIsFMODPaused(): Boolean
    private external fun nativeStopFMODUpdateThread()
    private external fun nativeStopFMODPlayback()

    // --- End FMOD Native Interface ---

    // --- BleService Connection ---
    private var bleService: BleService? = null
    private var bleServiceBound = false
    private val bleServiceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            Log.d(TAG, "onServiceConnected: BleService connection established in MusicService.")
            val binder = service as BleService.LocalBinder
            bleService = binder.getService()
            bleServiceBound = true

            bleService?.speed?.observeForever(speedObserver)
            bleService?.pitch?.observeForever(pitchObserver)
            bleService?.lastEvent?.observeForever(eventObserver)
            bleService?.hallDirection?.observeForever(hallDirectionObserver)
            Log.d(TAG, "MusicService now observing BleService LiveData (including Hall Direction).")
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            Log.w(TAG, "onServiceDisconnected: BleService connection lost in MusicService.")
            bleService?.speed?.removeObserver(speedObserver)
            bleService?.pitch?.removeObserver(pitchObserver)
            bleService?.lastEvent?.removeObserver(eventObserver)
            bleService?.hallDirection?.removeObserver(hallDirectionObserver)
            bleService = null
            bleServiceBound = false
        }
    }
    // --- End BleService Connection ---

    // --- Auto Mode State Variables ---
    private var isSpeedInAutoMode = false
    private var isPitchInAutoMode = false
    private var isEventInAutoMode = false
    private var isHallDirectionInAutoMode = false
    private var isPitchSignalReversed = false

    // --- LiveData for current FMOD parameter values (exposed to ViewModel) ---
    private val _currentFmodSpeed = MutableLiveData<Float>(0.0f)
    val currentFmodSpeed: LiveData<Float> get() = _currentFmodSpeed
    // ... (other _currentFmod... LiveData)
    private val _currentFmodPitch = MutableLiveData<Float>(0.0f)
    val currentFmodPitch: LiveData<Float> get() = _currentFmodPitch
    private val _currentFmodEventParameter = MutableLiveData<Float>(0.0f)
    val currentFmodEventParameter: LiveData<Float> get() = _currentFmodEventParameter
    private val _currentFmodHallDirection = MutableLiveData<Float>(1.0f)
    val currentFmodHallDirection: LiveData<Float> get() = _currentFmodHallDirection

    // LiveData for RIDE STATS (to be observed by ViewModel)
    private val _rideMaxSpeed = MutableLiveData<Float>(0f)
    val rideMaxSpeed: LiveData<Float> get() = _rideMaxSpeed

    private val _rideMaxPositivePitch = MutableLiveData<Float>(0f)
    val rideMaxPositivePitch: LiveData<Float> get() = _rideMaxPositivePitch

    private val _rideMinNegativePitch = MutableLiveData<Float>(0f) // Stores absolute value, display will negate
    val rideMinNegativePitch: LiveData<Float> get() = _rideMinNegativePitch

    private val _rideJumpCount = MutableLiveData<Int>(0)
    val rideJumpCount: LiveData<Int> get() = _rideJumpCount

    private val _rideDropCount = MutableLiveData<Int>(0)
    val rideDropCount: LiveData<Int> get() = _rideDropCount

    private val _ride180Count = MutableLiveData<Int>(0)
    val ride180Count: LiveData<Int> get() = _ride180Count
    // --- END RIDE STATS LiveData ---

    // Observers for BleService LiveData - UPDATED to also update stats
    private val speedObserver = Observer<Float> { speed ->
        updateMaxSpeedStat(speed) // Update stat
        if (isSpeedInAutoMode) {
            val clampedSpeed = speed.coerceIn(0f, 25f)
            setFmodParameterInternal("Wheel Speed", clampedSpeed)
        }
    }

    private val pitchObserver = Observer<Float> { rawPitch ->
        updatePitchStats(rawPitch) // Update stat
        if (isPitchInAutoMode) {
            val finalPitch = if (isPitchSignalReversed) -rawPitch else rawPitch
            val clampedPitch = finalPitch.coerceIn(-45f, 45f)
            setFmodParameterInternal("Pitch", clampedPitch)
        }
    }

    private val eventObserver = Observer<String> { event ->
        // Note: FMOD "Event" parameter is set when an event occurs.
        // Stat counting should happen based on the incoming 'event' string.
        when (event.uppercase(java.util.Locale.US)) {
            "JUMP" -> incrementJumpCountStat()
            "DROP" -> incrementDropCountStat()
            "180" -> increment180CountStat()
        }

        if (isEventInAutoMode && event != "NONE") {
            val eventValue = when (event.uppercase(java.util.Locale.US)) {
                "JUMP" -> 1.0f
                "DROP" -> 2.0f
                "180"  -> 3.0f
                else -> -1.0f
            }
            if (eventValue != -1.0f) {
                setFmodParameterInternal("Event", eventValue)
                Handler(Looper.getMainLooper()).postDelayed({
                    if (isEventInAutoMode && _currentFmodEventParameter.value == eventValue) {
                        setFmodParameterInternal("Event", 0.0f)
                    }
                }, 500)
            }
        }
    }

    private val hallDirectionObserver = Observer<Int> { direction ->
        // This observer handles Hall Direction for FMOD auto mode
        // It does not directly contribute to the stats we defined (max speed, pitch, jump/drop counts)
        if (isHallDirectionInAutoMode) {
            setFmodParameterInternal("Hall Direction", direction.toFloat())
        }
    }

    inner class LocalBinder : Binder() {
        fun getService(): MusicService = this@MusicService
    }

    override fun onCreate() {
        super.onCreate()
        notificationManager = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        prefs = applicationContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE) // Initialize SharedPreferences
        createNotificationChannel()
        Log.d(TAG, "onCreate: MusicService created.")

        loadRideStats() // Load persisted stats

        org.fmod.FMOD.init(this)
        val fmodIsInitialized: Boolean = org.fmod.FMOD.checkInit()
        Log.d(TAG, "FMOD.checkInit() result in MusicService: $fmodIsInitialized")
        if (!fmodIsInitialized) {
            Log.e(TAG, "FMOD initialization failed in MusicService! Stopping service.")
            stopSelf()
            return
        } else {
            Log.i(TAG, "FMOD initialized successfully in MusicService.")
        }

        val bleServiceIntent = Intent(this, BleService::class.java)
        bindService(bleServiceIntent, bleServiceConnection, Context.BIND_AUTO_CREATE)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        Log.d(TAG, "onStartCommand: MusicService action: ${intent?.action}")

        when (intent?.action) {
            ACTION_START_INFERENCE -> {
                if (!isInferenceActive) {
                    val inferenceIntent = Intent(this, InferenceService::class.java)
                    startService(inferenceIntent)
                    isInferenceActive = true
                    Log.d(TAG, "InferenceService started by MusicService.")
                    updateNotificationWithCurrentState()
                }
                return START_STICKY // Stay running for this command
            }
            ACTION_STOP_INFERENCE -> {
                if (isInferenceActive) {
                    val inferenceIntent = Intent(this, InferenceService::class.java)
                    stopService(inferenceIntent)
                    isInferenceActive = false
                    Log.d(TAG, "InferenceService stopped by MusicService.")
                    updateNotificationWithCurrentState()
                }
                return START_STICKY // Stay running for this command
            }
        }

        // Default behavior for starting the service (e.g., from app launch or system restart)
        val notificationText = if (isPlaying()) {
            if (isInferenceActive) "Playing Music - Analyzing Data..." else "Playing Music..."
        } else {
            if (isInferenceActive) "Music Service Active - Analyzing Data" else "Music Service Active"
        }
        val notification = buildNotification(notificationText)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(ONGOING_NOTIFICATION_ID, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_MEDIA_PLAYBACK)
        } else {
            startForeground(ONGOING_NOTIFICATION_ID, notification)
        }
        Log.i(TAG, "MusicService started/restarted in foreground.")
        return START_STICKY
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(NOTIFICATION_CHANNEL_ID, NOTIFICATION_CHANNEL_NAME, NotificationManager.IMPORTANCE_LOW)
            notificationManager.createNotificationChannel(channel)
        }
    }

    private fun buildNotification(contentText: String): Notification {
        val iconResId = R.drawable.ic_music_notification // Ensure this drawable exists
        // Construct the final content text based on playback and inference state
        val finalContentText = when {
            isPlaying() && isInferenceActive -> "Playing - Analyzing Data..."
            isPlaying() && !isInferenceActive -> "Playing Music..."
            !isPlaying() && isInferenceActive -> "Paused - Analyzing Data" // Or "Music Ready - Analyzing Data"
            else -> "Music Service Ready" // Or use the passed contentText if more generic states are needed
        }

        return NotificationCompat.Builder(this, NOTIFICATION_CHANNEL_ID)
            .setContentTitle("Music Bike Playback")
            .setContentText(finalContentText) // Use the dynamically constructed text
            .setSmallIcon(iconResId)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .setOngoing(true) // Good practice for foreground service notifications
            .build()
    }

    fun updateNotification(newContentText: String) { // This might be less used now if buildNotification is smart
        if (::notificationManager.isInitialized) {
            // Instead of newContentText, always build based on current state
            updateNotificationWithCurrentState()
        }
    }

    // New helper function to update notification based on all current states
    private fun updateNotificationWithCurrentState() {
        if (::notificationManager.isInitialized) {
            val contentText = when {
                isPlaying() && isInferenceActive -> "Playing - Analyzing Data..."
                isPlaying() && !isInferenceActive -> "Playing Music..."
                !isPlaying() && isInferenceActive -> "Paused - Analyzing Data"
                else -> "Music Paused" // Or "Music Ready" or similar
            }
            notificationManager.notify(ONGOING_NOTIFICATION_ID, buildNotification(contentText)) // Pass a representative base text, buildNotification will refine it
        }
    }

    // Make sure to call updateNotificationWithCurrentState() when isPlaying() state changes too.
    // For example, in nativeToggleFMODPlayback() or wherever playback state is directly altered.
    // This is already implicitly handled if onStartCommand is re-triggered for general startup,
    // but direct calls after state changes are more robust.

    // Example of where to call it (you'll need to integrate this into your FMOD control logic):
    fun togglePlaybackAndNotify() { // This is a new suggested method
        nativeToggleFMODPlayback() // Your existing toggle function
        updateNotificationWithCurrentState() // Update notification after toggling
    }

    override fun onBind(intent: Intent): IBinder {
        Log.d(TAG, "onBind: MusicService bound by a client.")
        return binder
    }

    override fun onTaskRemoved(rootIntent: Intent?) {
        Log.d(TAG, "onTaskRemoved called - app task swiped away by user.")

        if (isPlaying()) { // isPlaying() uses nativeIsFMODPaused(), which should be true if actually playing
            Log.i(TAG, "onTaskRemoved: Music was playing, attempting to pause it via nativeToggleFMODPlayback.")
            togglePlaybackAndNotify() // This should toggle a playing event to paused
        } else {
            Log.i(TAG, "onTaskRemoved: Music was already not playing (paused or stopped). No playback action taken.")
            // If it was stopped, we do nothing that would start it.
            // If it was paused, it remains paused (which is fine for stopping the service).
        }

        updateNotification("Music Service Shutting Down"); // Update notification appropriately

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            stopForeground(Service.STOP_FOREGROUND_REMOVE)
        } else {
            @Suppress("DEPRECATION")
            stopForeground(true)
        }

        stopSelf()
        Log.i(TAG, "MusicService stopped due to task removal.")

        super.onTaskRemoved(rootIntent)
    }

    override fun onDestroy() {
        super.onDestroy()
        Log.d(TAG, "onDestroy: MusicService being destroyed.")

        if (bleServiceBound) {
            bleService?.speed?.removeObserver(speedObserver)
            bleService?.pitch?.removeObserver(pitchObserver)
            bleService?.lastEvent?.removeObserver(eventObserver)
            bleService?.hallDirection?.removeObserver(hallDirectionObserver)
            unbindService(bleServiceConnection)
            bleServiceBound = false
            bleService = null
        }

        nativeStopFMODUpdateThread()
        nativeStopFMODPlayback()
        // stopForeground(true) is implicitly handled by stopSelf() if service is stopped,
        // but explicitly calling it in onTaskRemoved is good for immediate notification removal.
        // If onDestroy is called for other reasons while foregrounded, it might also be good here,
        // but often stopSelf() is enough to trigger system cleanup of foreground state.
        // For safety, if not already stopped via onTaskRemoved:
        // stopForeground(Service.STOP_FOREGROUND_REMOVE) // Or stopForeground(true) for older

        org.fmod.FMOD.close()
        Log.d(TAG, "FMOD closed in MusicService.")
        Log.d(TAG, "MusicService onDestroy completed.")
    }

    // --- Public Control Methods ---
    fun play() {
        Log.d(TAG, "play() called.")
        if (nativeIsFMODPaused()) {
            togglePlaybackAndNotify()        
        }
        updateNotification(if (isPlaying()) "Playing Music..." else "Playback Error")
    }

    fun pause() {
        Log.d(TAG, "pause() called.")
        if (!nativeIsFMODPaused()) {
            togglePlaybackAndNotify()
        }
        updateNotification("Music Paused")
    }

    fun togglePlayback() {
        Log.d(TAG, "togglePlayback() called.")
        togglePlaybackAndNotify()
        updateNotification(if (isPlaying()) "Playing Music..." else "Music Paused")
    }

    fun loadBank(masterBankPath: String, stringsBankPath: String) {
        Log.d(TAG, "loadBank() called with master: $masterBankPath, strings: $stringsBankPath")
        nativeStartFMODPlayback(masterBankPath, stringsBankPath)
        updateNotification("Bank Loaded: ${masterBankPath.substringAfterLast('/')}")
    }

    fun setFmodParameter(name: String, value: Float) {
        Log.d(TAG, "setFmodParameter (public): $name to $value")
        setFmodParameterInternal(name, value)
    }

    private fun setFmodParameterInternal(name: String, value: Float) {
        nativeSetFMODParameter(name, value)
        when (name) {
            "Wheel Speed" -> _currentFmodSpeed.postValue(value)
            "Pitch" -> _currentFmodPitch.postValue(value)
            "Event" -> _currentFmodEventParameter.postValue(value)
            "Hall Direction" -> _currentFmodHallDirection.postValue(value)
        }
    }

    fun playFmodEvent() {
        Log.d(TAG, "playFmodEvent() called.")
        nativePlayFMODEvent()
    }

    fun isPlaying(): Boolean {
        val isPaused = nativeIsFMODPaused()
        return !isPaused
    }

    // --- Public Methods to Control Auto Modes ---
    fun setAutoSpeedMode(enabled: Boolean) {
        isSpeedInAutoMode = enabled
        Log.d(TAG, "Auto Speed Mode set to: $enabled")
        if (enabled) {
            bleService?.speed?.value?.let { currentSpeed ->
                val clampedSpeed = currentSpeed.coerceIn(0f, 25f)
                Log.d(TAG, "AUTO MODE ON: Initializing Wheel Speed to $clampedSpeed")
                setFmodParameterInternal("Wheel Speed", clampedSpeed)
            }
        }
    }

    fun setAutoPitchMode(enabled: Boolean) {
        isPitchInAutoMode = enabled
        Log.d(TAG, "Auto Pitch Mode set to: $enabled")
        if (enabled) {
            bleService?.pitch?.value?.let { currentPitch ->
                val clampedPitch = currentPitch.coerceIn(-45f, 45f)
                Log.d(TAG, "AUTO MODE ON: Initializing Pitch to $clampedPitch")
                setFmodParameterInternal("Pitch", clampedPitch)
            }
        }
    }

    fun setAutoEventMode(enabled: Boolean) {
        isEventInAutoMode = enabled
        Log.d(TAG, "Auto Event Mode set to: $enabled")
        if (!enabled) {
            setFmodParameterInternal("Event", 0.0f)
        }
    }

    fun setAutoHallDirectionMode(enabled: Boolean) {
        isHallDirectionInAutoMode = enabled
        Log.d(TAG, "Auto Hall Direction Mode set to: $enabled")
        if (enabled) {
            bleService?.hallDirection?.value?.let { currentDirection ->
                Log.d(TAG, "AUTO MODE ON: Initializing Hall Direction to $currentDirection")
                setFmodParameterInternal("Hall Direction", currentDirection.toFloat())
            }
        }
    }

    fun setPitchSignalReversal(reversed: Boolean) {
        isPitchSignalReversed = reversed
        Log.d(TAG, "Pitch Signal Reversal set to: $reversed")
        // If pitch is in auto mode, immediately apply the new reversal state to the current FMOD parameter
        if (isPitchInAutoMode) {
            bleService?.pitch?.value?.let { currentRawPitch ->
                val finalPitch = if (isPitchSignalReversed) -currentRawPitch else currentRawPitch
                val clampedPitch = finalPitch.coerceIn(-45f, 45f)
                Log.d(TAG, "AUTO MODE (Reversal Changed): Updating Pitch to $clampedPitch")
                setFmodParameterInternal("Pitch", clampedPitch)
            }
        }
    }
    // Stat Update, Persistence, and Reset Logic for MusicService
    private fun updateMaxSpeedStat(currentSpeed: Float) {
        val currentMax = _rideMaxSpeed.value ?: 0f
        if (currentSpeed > currentMax) {
            _rideMaxSpeed.postValue(currentSpeed)
            prefs.edit().putFloat(KEY_MAX_SPEED, currentSpeed).apply()
            Log.d(TAG, "New Max Speed: $currentSpeed")
        }
    }

    private fun updatePitchStats(currentPitch: Float) {
        val currentMaxPos = _rideMaxPositivePitch.value ?: 0f
        val currentMinNegStored = _rideMinNegativePitch.value ?: 0f // This is stored as positive

        if (currentPitch > 0 && currentPitch > currentMaxPos) {
            _rideMaxPositivePitch.postValue(currentPitch)
            prefs.edit().putFloat(KEY_MAX_POSITIVE_PITCH, currentPitch).apply()
            Log.d(TAG, "New Max Positive Pitch: $currentPitch")
        } else if (currentPitch < 0 && -currentPitch > currentMinNegStored) {
            // store absolute value for minNegativePitch (which is a max of the negative magnitude)
            val newMinNegMagnitude = -currentPitch
            _rideMinNegativePitch.postValue(newMinNegMagnitude)
            prefs.edit().putFloat(KEY_MIN_NEGATIVE_PITCH, newMinNegMagnitude).apply()
            Log.d(TAG, "New Min Negative Pitch (stored as positive): $newMinNegMagnitude (actual: $currentPitch)")
        }
    }

    private fun incrementJumpCountStat() {
        val newCount = (_rideJumpCount.value ?: 0) + 1
        _rideJumpCount.postValue(newCount)
        prefs.edit().putInt(KEY_JUMP_COUNT, newCount).apply()
        Log.d(TAG, "Jump count: $newCount")
    }

    private fun incrementDropCountStat() {
        val newCount = (_rideDropCount.value ?: 0) + 1
        _rideDropCount.postValue(newCount)
        prefs.edit().putInt(KEY_DROP_COUNT, newCount).apply()
        Log.d(TAG, "Drop count: $newCount")
    }

    private fun increment180CountStat() {
        val newCount = (_ride180Count.value ?: 0) + 1
        _ride180Count.postValue(newCount)
        prefs.edit().putInt(KEY_180_COUNT, newCount).apply()
        Log.d(TAG, "180 count: $newCount")
    }

    fun resetRideStats() {
        Log.i(TAG, "Resetting ride stats in MusicService.")
        _rideMaxSpeed.postValue(0f)
        _rideMaxPositivePitch.postValue(0f)
        _rideMinNegativePitch.postValue(0f)
        _rideJumpCount.postValue(0)
        _rideDropCount.postValue(0)
        _ride180Count.postValue(0)

        prefs.edit()
            .putFloat(KEY_MAX_SPEED, 0f)
            .putFloat(KEY_MAX_POSITIVE_PITCH, 0f)
            .putFloat(KEY_MIN_NEGATIVE_PITCH, 0f)
            .putInt(KEY_JUMP_COUNT, 0)
            .putInt(KEY_DROP_COUNT, 0)
            .putInt(KEY_180_COUNT, 0)
            .apply()
    }

    private fun loadRideStats() {
        _rideMaxSpeed.value = prefs.getFloat(KEY_MAX_SPEED, 0f)
        _rideMaxPositivePitch.value = prefs.getFloat(KEY_MAX_POSITIVE_PITCH, 0f)
        _rideMinNegativePitch.value = prefs.getFloat(KEY_MIN_NEGATIVE_PITCH, 0f)
        _rideJumpCount.value = prefs.getInt(KEY_JUMP_COUNT, 0)
        _rideDropCount.value = prefs.getInt(KEY_DROP_COUNT, 0)
        _ride180Count.value = prefs.getInt(KEY_180_COUNT, 0)
        Log.i(TAG, "Ride stats loaded from SharedPreferences in MusicService.")
    }
}