package com.app.musicbike.ui.viewmodels

import android.app.Application
import android.content.Context
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.LifecycleOwner
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.Observer
import com.app.musicbike.services.MusicService

class MusicViewModel(application: Application) : AndroidViewModel(application) {

    private val TAG = "MusicViewModel"
    // SharedPreferences for UI settings like auto-modes and pitch reversal
    private val uiPrefs = application.getSharedPreferences("MusicBikeUISettings", Context.MODE_PRIVATE)

    private var musicService: MusicService? = null

    // --- LiveData for UI state observed by MusicFragment ---
    private val _isPlaying = MutableLiveData<Boolean>(false)
    val isPlaying: LiveData<Boolean> get() = _isPlaying

    private val _currentBankName = MutableLiveData<String>("None")
    val currentBankName: LiveData<String> get() = _currentBankName

    // These LiveData are for displaying FMOD parameters (updated by service's LiveData)
    private val _wheelSpeedDisplay = MutableLiveData<Float>(0f)
    val wheelSpeedDisplay: LiveData<Float> get() = _wheelSpeedDisplay

    private val _pitchDisplay = MutableLiveData<Float>(0f)
    val pitchDisplay: LiveData<Float> get() = _pitchDisplay

    private val _eventDisplayValue = MutableLiveData<Float>(0f)
    val eventDisplayValue: LiveData<Float> get() = _eventDisplayValue

    private val _hallDirectionDisplayValue = MutableLiveData<Float>(1f)
    val hallDirectionDisplayValue: LiveData<Float> get() = _hallDirectionDisplayValue

    // --- LiveData for Ride Stats (now populated by observing MusicService) ---
    private val _maxSpeed = MutableLiveData<Float>(0f)
    val maxSpeed: LiveData<Float> get() = _maxSpeed

    private val _maxPositivePitch = MutableLiveData<Float>(0f)
    val maxPositivePitch: LiveData<Float> get() = _maxPositivePitch

    private val _minNegativePitch = MutableLiveData<Float>(0f)
    val minNegativePitch: LiveData<Float> get() = _minNegativePitch

    private val _jumpCount = MutableLiveData<Int>(0)
    val jumpCount: LiveData<Int> get() = _jumpCount

    private val _dropCount = MutableLiveData<Int>(0)
    val dropCount: LiveData<Int> get() = _dropCount

    private val _oneEightyCount = MutableLiveData<Int>(0)
    val oneEightyCount: LiveData<Int> get() = _oneEightyCount
    // --- END Ride Stats LiveData ---

    // Observers for MusicService's FMOD parameter LiveData
    private var serviceLifecycleOwner: LifecycleOwner? = null
    private val fmodSpeedObserver = Observer<Float> { speed -> _wheelSpeedDisplay.postValue(speed) }
    private val fmodPitchObserver = Observer<Float> { pitch -> _pitchDisplay.postValue(pitch) }
    private val fmodEventObserver = Observer<Float> { eventParam -> _eventDisplayValue.postValue(eventParam) }
    private val fmodHallDirObserver = Observer<Float> { directionParam -> _hallDirectionDisplayValue.postValue(directionParam) }

    // Observers for MusicService's RIDE STATS LiveData
    private val rideMaxSpeedObserver = Observer<Float> { value -> _maxSpeed.postValue(value) }
    private val rideMaxPosPitchObserver = Observer<Float> { value -> _maxPositivePitch.postValue(value) }
    private val rideMinNegPitchObserver = Observer<Float> { value -> _minNegativePitch.postValue(value) }
    private val rideJumpCountObserver = Observer<Int> { value -> _jumpCount.postValue(value) }
    private val rideDropCountObserver = Observer<Int> { value -> _dropCount.postValue(value) }
    private val ride180CountObserver = Observer<Int> { value -> _oneEightyCount.postValue(value) }
    // RIDE STATS Observers


    // Removed init block that loaded stats directly; stats will come from service.

    fun setMusicService(service: MusicService?, owner: LifecycleOwner) {
        removeServiceObservers() // Remove from old instance if any
        this.musicService = service
        this.serviceLifecycleOwner = owner

        if (this.musicService != null) {
            _isPlaying.postValue(this.musicService!!.isPlaying()) // Initial playback state

            // Observe FMOD parameter LiveData from MusicService
            this.musicService!!.currentFmodSpeed.observe(owner, fmodSpeedObserver)
            this.musicService!!.currentFmodPitch.observe(owner, fmodPitchObserver)
            this.musicService!!.currentFmodEventParameter.observe(owner, fmodEventObserver)
            this.musicService!!.currentFmodHallDirection.observe(owner, fmodHallDirObserver)

            // Observe RIDE STATS LiveData from MusicService
            this.musicService!!.rideMaxSpeed.observe(owner, rideMaxSpeedObserver)
            this.musicService!!.rideMaxPositivePitch.observe(owner, rideMaxPosPitchObserver)
            this.musicService!!.rideMinNegativePitch.observe(owner, rideMinNegPitchObserver)
            this.musicService!!.rideJumpCount.observe(owner, rideJumpCountObserver)
            this.musicService!!.rideDropCount.observe(owner, rideDropCountObserver)
            this.musicService!!.ride180Count.observe(owner, ride180CountObserver)
            // STATS Observation ---

            Log.d(TAG, "MusicService instance set and ALL observers attached.")
        } else {
            Log.w(TAG, "MusicService instance is null in ViewModel. Observers not attached.")
        }
    }

    private fun removeServiceObservers() {
        serviceLifecycleOwner?.let { owner ->
            musicService?.currentFmodSpeed?.removeObserver(fmodSpeedObserver)
            musicService?.currentFmodPitch?.removeObserver(fmodPitchObserver)
            musicService?.currentFmodEventParameter?.removeObserver(fmodEventObserver)
            musicService?.currentFmodHallDirection?.removeObserver(fmodHallDirObserver)

            // Remove RIDE STATS Observers
            musicService?.rideMaxSpeed?.removeObserver(rideMaxSpeedObserver)
            musicService?.rideMaxPositivePitch?.removeObserver(rideMaxPosPitchObserver)
            musicService?.rideMinNegativePitch?.removeObserver(rideMinNegPitchObserver)
            musicService?.rideJumpCount?.removeObserver(rideJumpCountObserver)
            musicService?.rideDropCount?.removeObserver(rideDropCountObserver)
            musicService?.ride180Count?.removeObserver(ride180CountObserver)
        }
        Log.d(TAG, "Removed observers from previous MusicService instance.")
    }

    fun togglePlayback() {
        Log.d(TAG, "ViewModel togglePlayback() ENTERED.")
        if (musicService == null) {
            Log.e(TAG, "ViewModel togglePlayback: musicService is NULL! Cannot send command.")
            return
        }
        Log.d(TAG, "ViewModel togglePlayback: musicService is available. Calling service.togglePlayback().")
        musicService?.togglePlayback()
        // Update LiveData based on the new state from the service
        _isPlaying.postValue(musicService?.isPlaying() ?: false) // isPlaying should also be non-null if service is non-null
        Log.d(TAG, "ViewModel togglePlayback() EXITED. New isPlaying LiveData: ${isPlaying.value}")
    }

    fun loadBank(masterBankPath: String, stringsBankPath: String, bankDisplayName: String) {
        musicService?.loadBank(masterBankPath, stringsBankPath)
        _currentBankName.postValue(bankDisplayName)
        _isPlaying.postValue(false)
        Log.d(TAG, "loadBank called for: $bankDisplayName")
    }

    fun setFmodParameter(name: String, value: Float) {
        if (_isWheelSpeedAuto.value == true && name == "Wheel Speed") return
        if (_isPitchAuto.value == true && name == "Pitch") return
        if (_isEventAuto.value == true && name == "Event") return
        if (_isHallDirectionAuto.value == true && name == "Hall Direction") return

        musicService?.setFmodParameter(name, value)
        Log.d(TAG, "setFmodParameter (manual command): $name to $value")
    }

    fun playFmodEvent() {
        musicService?.playFmodEvent()
        Log.d(TAG, "playFmodEvent (generic) called from ViewModel")
    }


    // Auto Mode LiveData (loaded from uiPrefs)
    private val _isWheelSpeedAuto = MutableLiveData<Boolean>(loadBooleanPreference("isWheelSpeedAuto", false))
    val isWheelSpeedAuto: LiveData<Boolean> get() = _isWheelSpeedAuto
    private val _isPitchAuto = MutableLiveData<Boolean>(loadBooleanPreference("isPitchAuto", false))
    val isPitchAuto: LiveData<Boolean> get() = _isPitchAuto
    private val _isEventAuto = MutableLiveData<Boolean>(loadBooleanPreference("isEventAuto", false))
    val isEventAuto: LiveData<Boolean> get() = _isEventAuto
    private val _isHallDirectionAuto = MutableLiveData<Boolean>(loadBooleanPreference("isHallDirectionAuto", false))
    val isHallDirectionAuto: LiveData<Boolean> get() = _isHallDirectionAuto

    // Pitch Reversal LiveData (loaded from uiPrefs)
    private val _isPitchSignalReversed = MutableLiveData<Boolean>(loadBooleanPreference("isPitchSignalReversed", false))
    val isPitchSignalReversed: LiveData<Boolean> get() = _isPitchSignalReversed

    fun setWheelSpeedAuto(isAuto: Boolean) {
        _isWheelSpeedAuto.postValue(isAuto)
        musicService?.setAutoSpeedMode(isAuto)
        saveBooleanPreference("isWheelSpeedAuto", isAuto)
        Log.d(TAG, "setWheelSpeedAuto: $isAuto")
    }
    fun setPitchAuto(isAuto: Boolean) {
        _isPitchAuto.postValue(isAuto)
        musicService?.setAutoPitchMode(isAuto)
        saveBooleanPreference("isPitchAuto", isAuto)
        Log.d(TAG, "setPitchAuto: $isAuto")
    }
    fun setEventAuto(isAuto: Boolean) {
        _isEventAuto.postValue(isAuto)
        musicService?.setAutoEventMode(isAuto)
        saveBooleanPreference("isEventAuto", isAuto)
        Log.d(TAG, "setEventAuto: $isAuto")
    }
    fun setHallDirectionAuto(isAuto: Boolean) {
        _isHallDirectionAuto.postValue(isAuto)
        musicService?.setAutoHallDirectionMode(isAuto)
        saveBooleanPreference("isHallDirectionAuto", isAuto)
        Log.d(TAG, "setHallDirectionAuto: $isAuto")
    }
    fun togglePitchSignalReversal() {
        val newState = !(_isPitchSignalReversed.value ?: false)
        _isPitchSignalReversed.postValue(newState)
        musicService?.setPitchSignalReversal(newState)
        saveBooleanPreference("isPitchSignalReversed", newState)
        Log.d(TAG, "Pitch signal reversal toggled to: $newState")
    }

    private fun savePitchSignalReversalPreference(isReversed: Boolean) {
        // TODO: Implement SharedPreferences saving for pitch reversal
        // Example:
        // val prefs = applicationContext.getSharedPreferences("MusicBikePrefs", Context.MODE_PRIVATE)
        // prefs.edit().putBoolean("pitch_signal_reversed", isReversed).apply()
        Log.d(TAG, "Pitch reversal preference saved: $isReversed (Implement SharedPreferences)")
    }

    private fun loadPitchSignalReversalPreference(): Boolean {
        // TODO: Implement SharedPreferences loading for pitch reversal
        // Example:
        // val prefs = applicationContext.getSharedPreferences("MusicBikePrefs", Context.MODE_PRIVATE)
        // return prefs.getBoolean("pitch_signal_reversed", false) // Default to false
        Log.d(TAG, "Pitch reversal preference loaded (Implement SharedPreferences)")
        return false // Default placeholder
    }

    // Ride Stats Logic

    // resetStats now calls the method in MusicService
    fun resetStats() {
        Log.i(TAG, "ViewModel requesting MusicService to reset ride stats.")
        musicService?.resetRideStats()
        // The LiveData in this ViewModel (_maxSpeed, etc.) will automatically update
        // because they are observing the LiveData from MusicService, which will be reset.
    }

    // Helper for boolean UI preferences
    private fun saveBooleanPreference(key: String, value: Boolean) {
        uiPrefs.edit().putBoolean(key, value).apply()
    }

    private fun loadBooleanPreference(key: String, defaultValue: Boolean): Boolean {
        return uiPrefs.getBoolean(key, defaultValue)
    }

    override fun onCleared() {
        super.onCleared()
        removeServiceObservers()
        this.musicService = null
        Log.d(TAG, "MusicViewModel cleared.")
    }
}