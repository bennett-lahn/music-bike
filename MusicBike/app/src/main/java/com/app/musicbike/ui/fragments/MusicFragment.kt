/*
 * MusicFragment.kt - Music Control User Interface
 * 
 * This fragment provides the main user interface for controlling music playback
 * and managing FMOD parameter settings. It allows users to switch between manual
 * and automatic modes for various audio parameters.
 * 
 * Key Features:
 * - Music playback controls (play/pause/bank selection)
 * - Manual parameter adjustment via sliders and controls
 * - Automatic mode integration with BLE sensor data
 * - Real-time display of current parameter values
 * - Ride statistics display and reset functionality
 * - BLE connection status monitoring
 * 
 * UI Components:
 * - Bank selector for choosing audio banks
 * - Parameter controls (speed, pitch, events, direction)
 * - Auto mode switches for BLE integration
 * - Statistics display for ride tracking
 * - Connection status indicators
 */
package com.app.musicbike.ui.fragments

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.*
import android.widget.*
import androidx.fragment.app.Fragment
import androidx.fragment.app.viewModels
import androidx.lifecycle.Observer
import com.app.musicbike.R
import com.app.musicbike.databinding.FragmentMusicBinding
import com.app.musicbike.services.BleService
import com.app.musicbike.services.MusicService
import com.app.musicbike.ui.activities.MainActivity
import com.app.musicbike.ui.viewmodels.MusicViewModel
import java.io.File
import java.util.*

/**
 * MusicFragment - Main UI for music control and parameter management
 * 
 * Provides controls for FMOD playback and real-time parameter adjustment.
 * Integrates with MusicService and BleService for audio and sensor functionality.
 */
class MusicFragment : Fragment() {

    private val TAG = "MusicFragment"
    private var _binding: FragmentMusicBinding? = null
    private val binding get() = _binding!!
    private val handler = Handler(Looper.getMainLooper())

    private val musicViewModel: MusicViewModel by viewModels()
    private var bleService: BleService? = null

    // UI state tracking for auto mode switches
    private var isWheelSpeedAutoUi = false
    private var isPitchAutoUi = false
    private var isEventAutoUi = false
    private var isHallDirectionAutoUi = false

    /**
     * Check if BLE service is connected and ready for data transmission
     * @return true if BLE is connected, false otherwise
     */
    private fun isBleConnected(): Boolean {
        val mainActivity = activity as? MainActivity
        this.bleService = mainActivity?.getBleServiceInstance()
        val status = bleService?.connectionStatus?.value ?: return false
        return status.contains("connected", true) ||
                status.contains("ready", true) ||
                status.contains("discovered", true)
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View {
        _binding = FragmentMusicBinding.inflate(inflater, container, false)
        Log.d(TAG, "onCreateView")
        return binding.root
    }

    /**
     * Called by MainActivity when BleService is ready for use
     * Sets up BLE service reference and updates UI accordingly
     */
    fun onServiceReady() {
        Log.d(TAG, "onServiceReady (for BleService) called.")
        this.bleService = (activity as? MainActivity)?.getBleServiceInstance()
        updateUiBasedOnBleConnection()
    }

    /**
     * Called by MainActivity when MusicService is ready for use
     * Initializes music service connection and loads default bank
     */
    fun onMusicServiceReady(service: MusicService?) {
        Log.d(TAG, "onMusicServiceReady called with service: $service")
        musicViewModel.setMusicService(service, viewLifecycleOwner)

        // Ensure UI is fully initialized (spinner populated)
        val selectedBankName = binding.bankSelector.selectedItem?.toString() ?: "Master"
        val allBanks = requireContext().assets.list("")?.filter {
            it.endsWith(".bank") && !it.endsWith("strings.bank")
        } ?: emptyList()

        val selectedFileName = allBanks.firstOrNull { it.removeSuffix(".bank") == selectedBankName }
            ?: allBanks.firstOrNull() ?: return

        val masterBankPath = copyAssetToInternalStorage(selectedFileName)
        val stringsBankPath = copyAssetToInternalStorage("Master.strings.bank")
        musicViewModel.loadBank(masterBankPath, stringsBankPath, selectedBankName)

        observeViewModel()
    }


    /**
     * Copy an asset file to internal storage for FMOD access
     * @param assetName Name of the asset file to copy
     * @return Absolute path to the copied file
     */
    private fun copyAssetToInternalStorage(assetName: String): String {
        val file = File(requireContext().filesDir, assetName)
        if (file.exists()) file.delete()
        try {
            requireContext().assets.open(assetName).use { input ->
                file.outputStream().use { output -> input.copyTo(output) }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to copy asset $assetName", e)
            return ""
        }
        return file.absolutePath
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        Log.d(TAG, "onViewCreated")

        (activity as? MainActivity)?.let { mainAct ->
            if (mainAct.isBleServiceConnected) {
                this.bleService = mainAct.getBleServiceInstance()
            }
            if (mainAct.isMusicServiceConnected) {
                // Pass viewLifecycleOwner here as well
                musicViewModel.setMusicService(mainAct.getMusicServiceInstance(), viewLifecycleOwner)
            }
        }
        observeViewModel() // Call this to set up observers
        setupUI()
        updateUiBasedOnBleConnection()
    }

    /**
     * Initialize all UI components and set up event listeners
     * Configures controls for music playback and parameter adjustment
     */
    private fun setupUI() {
        Log.d(TAG, "setupUI called")

        // Music playback toggle button
        binding.toggleButton.setOnClickListener {
            Log.d(TAG, "ToggleButton in MusicFragment - CLICKED!")
            musicViewModel.togglePlayback()
        }

        // === BANK SELECTOR SETUP ===
        // Dropdown for selecting different FMOD audio banks
        val bankSpinner = binding.bankSelector
        val allBanks = requireContext().assets.list("")?.filter {
            it.endsWith(".bank") && !it.endsWith("strings.bank")
        } ?: emptyList()
        val bankNames = allBanks.map { it.removeSuffix(".bank") }
        val bankAdapter = ArrayAdapter(requireContext(), android.R.layout.simple_spinner_item, bankNames)
        bankAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        bankSpinner.adapter = bankAdapter

        val defaultBankIndex = bankNames.indexOf("Master").takeIf { it >= 0 } ?: 0
        bankSpinner.setSelection(defaultBankIndex)

        bankSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>, view: View?, position: Int, id: Long) {
                if (allBanks.isNotEmpty() && position < allBanks.size) {
                    val selectedBankName = allBanks[position]
                    val masterBankPath = copyAssetToInternalStorage(selectedBankName)
                    val stringsBankPath = copyAssetToInternalStorage("Master.strings.bank")
                    musicViewModel.loadBank(masterBankPath, stringsBankPath, bankNames[position])
                }
            }
            override fun onNothingSelected(parent: AdapterView<*>?) {}
        }

        // === WHEEL SPEED PARAMETER CONTROL ===
        setupSeekBar(binding.wheelSpeedSeekBar, binding.wheelSpeedLabel, "Wheel Speed", 25, 0, 0) { value ->
            if (musicViewModel.isWheelSpeedAuto.value != true) {
                musicViewModel.setFmodParameter("Wheel Speed", value)
            }
        }
        binding.wheelSpeedModeSwitch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked && !isBleConnected()) {
                binding.wheelSpeedModeSwitch.isChecked = false
                showToast("BLE not connected. Auto mode cancelled.")
                return@setOnCheckedChangeListener
            }
            musicViewModel.setWheelSpeedAuto(isChecked)
        }

        // === PITCH PARAMETER CONTROL ===
        setupSeekBar(binding.pitchSeekBar, binding.pitchLabel, "Pitch", 90, -45, 45) { value ->
            if (musicViewModel.isPitchAuto.value != true) {
                musicViewModel.setFmodParameter("Pitch", value)
            }
        }
        binding.pitchModeSwitch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked && !isBleConnected()) {
                binding.pitchModeSwitch.isChecked = false
                showToast("BLE not connected. Auto mode cancelled.")
                return@setOnCheckedChangeListener
            }
            musicViewModel.setPitchAuto(isChecked)
        }
        binding.pitchReverseSwitch.setOnCheckedChangeListener { _, isChecked ->
            if (musicViewModel.isPitchSignalReversed.value != isChecked) {
                musicViewModel.togglePitchSignalReversal()
                Log.d(TAG, "Pitch Reverse Switch toggled to: $isChecked")
            } else {
                Log.d(TAG, "Pitch Reverse Switch state unchanged, ignoring toggle.")
            }
        }

        // Set initial enabled state for pitchReverseSwitch based on current "Auto Pitch" mode state
        binding.pitchReverseSwitch.isEnabled = musicViewModel.isPitchAuto.value ?: false


        // === EVENT PARAMETER CONTROL ===
        // Dropdown for manually triggering bike events
        val events = arrayOf("None", "Jump", "Drop", "180")
        val eventAdapter = ArrayAdapter(requireContext(), android.R.layout.simple_spinner_item, events)
        binding.eventSpinner.adapter = eventAdapter
        binding.eventSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>, view: View?, position: Int, id: Long) {
                if (musicViewModel.isEventAuto.value != true) {
                    musicViewModel.setFmodParameter("Event", position.toFloat())
                    if (position in 1..3) { // Jump, Drop, or 180
                        // The service now handles auto-resetting the "Event" parameter if needed.
                        // No need for handler here if service does it.
                    }
                }
            }
            override fun onNothingSelected(parent: AdapterView<*>?) {}
        }
        binding.eventModeSwitch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked && !isBleConnected()) {
                binding.eventModeSwitch.isChecked = false
                showToast("BLE not connected. Auto mode cancelled.")
                return@setOnCheckedChangeListener
            }
            musicViewModel.setEventAuto(isChecked)
        }

        // === HALL DIRECTION PARAMETER CONTROL ===
        // Dropdown for setting bike direction (forward/reverse)
        val hallDirectionOptions = arrayOf("Forward", "Reverse")
        val hallAdapter = ArrayAdapter(requireContext(), android.R.layout.simple_spinner_item, hallDirectionOptions)
        binding.hallDirectionSpinner.adapter = hallAdapter
        // Initial selection will be handled by observing ViewModel

        binding.hallDirectionSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>, view: View?, position: Int, id: Long) {
                if (musicViewModel.isHallDirectionAuto.value != true) {
                    val value = if (position == 0) 1f else 0f // UI "Forward" (pos 0) = FMOD 1f
                    musicViewModel.setFmodParameter("Hall Direction", value)
                }
            }
            override fun onNothingSelected(parent: AdapterView<*>?) {}
        }
        binding.hallDirectionModeSwitch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked && !isBleConnected()) {
                binding.hallDirectionModeSwitch.isChecked = false
                showToast("BLE not connected. Auto mode cancelled.")
                return@setOnCheckedChangeListener
            }
            musicViewModel.setHallDirectionAuto(isChecked)
        }

        // === AUTO ALL SWITCH ===
        // Master switch to enable/disable all auto modes simultaneously
        binding.autoAllSwitch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked && !isBleConnected()) {
                showToast("BLE not connected. Auto All cancelled.")
                handler.postDelayed({
                    if(!isBleConnected()) binding.autoAllSwitch.isChecked = false
                }, 100)
                return@setOnCheckedChangeListener
            }
            musicViewModel.setWheelSpeedAuto(isChecked)
            musicViewModel.setPitchAuto(isChecked)
            musicViewModel.setEventAuto(isChecked)
            musicViewModel.setHallDirectionAuto(isChecked)
        }

        // === STATISTICS RESET BUTTON ===
        binding.resetStatsButton.setOnClickListener {
            Log.d(TAG, "Reset Stats button clicked.")
            musicViewModel.resetStats()
            // Optionally show a Toast or confirmation
            showToast("Ride stats have been reset.")
        }
    }

    /**
     * Set up observers for ViewModel LiveData to update UI in real-time
     * Observes playback state, parameter values, auto mode states, and statistics
     */
    private fun observeViewModel() {
        Log.d(TAG, "Setting up ViewModel observers.")

        // === PLAYBACK STATE OBSERVER ===
        musicViewModel.isPlaying.observe(viewLifecycleOwner, Observer { isPlaying ->
            Log.d(TAG, "isPlaying changed: $isPlaying")
            binding.toggleButton.text = if (isPlaying) "Pause" else "Play"

            /*
            val serviceIntent = Intent(requireActivity(), MusicService::class.java)
            if (isPlaying) {
                serviceIntent.action = MusicService.ACTION_START_INFERENCE
                Log.d(TAG, "Requesting MusicService to START inference.")
            } else {
                serviceIntent.action = MusicService.ACTION_STOP_INFERENCE
                Log.d(TAG, "Requesting MusicService to STOP inference.")
            }
            requireActivity().startService(serviceIntent) // Send command to MusicService
            */
        })

        musicViewModel.currentBankName.observe(viewLifecycleOwner) { bankName ->
            Log.d(TAG, "Observed currentBankName: $bankName")
            val bankNamesAdapter = (binding.bankSelector.adapter as? ArrayAdapter<String>)
            bankNamesAdapter?.let { adapter ->
                val position = (0 until adapter.count).firstOrNull { adapter.getItem(it) == bankName }
                if (position != null && binding.bankSelector.selectedItemPosition != position) {
                    binding.bankSelector.setSelection(position)
                }
            }
        }

        musicViewModel.wheelSpeedDisplay.observe(viewLifecycleOwner) { speed ->
            binding.wheelSpeedLabel.text = "Wheel Speed: ${speed.toInt()}" // Always update label
            if (musicViewModel.isWheelSpeedAuto.value == true) {
                binding.wheelSpeedSeekBar.progress = speed.coerceIn(0f, 25f).toInt()
            }
        }
        musicViewModel.pitchDisplay.observe(viewLifecycleOwner) { pitch ->
            binding.pitchLabel.text = "Pitch: ${pitch.toInt()}" // Always update label
            if (musicViewModel.isPitchAuto.value == true) {
                binding.pitchSeekBar.progress = (pitch.coerceIn(-45f, 45f) + 45).toInt()
            }
        }
        musicViewModel.eventDisplayValue.observe(viewLifecycleOwner) { eventParam ->
            // Update UI for event if needed, e.g., if you have a text display for last auto-triggered event
            // For the spinner, it's mostly for manual selection or reflecting auto-selection
            if (musicViewModel.isEventAuto.value == true) {
                val selection = when(eventParam.toInt()) {
                    1 -> 1 // Jump
                    2 -> 2 // Drop
                    3 -> 3 // 180
                    else -> 0 // None
                }
                if (binding.eventSpinner.selectedItemPosition != selection) {
                    binding.eventSpinner.setSelection(selection)
                }
            }
        }
        musicViewModel.hallDirectionDisplayValue.observe(viewLifecycleOwner) { directionParam ->
            // Update UI for hall direction if needed
            if (musicViewModel.isHallDirectionAuto.value == true) {
                val selection = if (directionParam == 1f) 0 else 1 // UI "Forward" (pos 0) = FMOD 1f
                if (binding.hallDirectionSpinner.selectedItemPosition != selection) {
                    binding.hallDirectionSpinner.setSelection(selection)
                }
            }
        }

        // === AUTO MODE STATE OBSERVERS ===
        // Update UI controls based on auto mode states
        musicViewModel.isWheelSpeedAuto.observe(viewLifecycleOwner) { isAuto ->
            if (binding.wheelSpeedModeSwitch.isChecked != isAuto) binding.wheelSpeedModeSwitch.isChecked = isAuto
            binding.wheelSpeedSeekBar.isEnabled = !isAuto
        }
        musicViewModel.isPitchAuto.observe(viewLifecycleOwner) { isAuto ->
            if (binding.pitchModeSwitch.isChecked != isAuto) binding.pitchModeSwitch.isChecked = isAuto
            binding.pitchSeekBar.isEnabled = !isAuto
            binding.pitchReverseSwitch.isEnabled = isAuto
        }
        musicViewModel.isEventAuto.observe(viewLifecycleOwner) { isAuto ->
            if (binding.eventModeSwitch.isChecked != isAuto) binding.eventModeSwitch.isChecked = isAuto
            binding.eventSpinner.isEnabled = !isAuto
        }
        musicViewModel.isHallDirectionAuto.observe(viewLifecycleOwner) { isAuto ->
            if (binding.hallDirectionModeSwitch.isChecked != isAuto) binding.hallDirectionModeSwitch.isChecked = isAuto
            binding.hallDirectionSpinner.isEnabled = !isAuto
        }

        // Observer for Pitch Reversal Switch State
        musicViewModel.isPitchSignalReversed.observe(viewLifecycleOwner) { isReversed ->
            if (binding.pitchReverseSwitch.isChecked != isReversed) {
                binding.pitchReverseSwitch.isChecked = isReversed
            }
            Log.d(TAG, "Observed pitchReversalEnabled from ViewModel: $isReversed, UI Switch updated.")
        }

        // === RIDE STATISTICS OBSERVERS ===
        // Update statistics display in real-time
        musicViewModel.maxSpeed.observe(viewLifecycleOwner) { maxSpeed ->
            binding.maxSpeedStatText.text = String.format(Locale.US, "Max Speed: %.1f km/h", maxSpeed)
        }
        musicViewModel.maxPositivePitch.observe(viewLifecycleOwner) { maxPitchUp ->
            binding.maxPositivePitchStatText.text = String.format(Locale.US, "Max Pitch (Up): %.1f°", maxPitchUp)
        }
        musicViewModel.minNegativePitch.observe(viewLifecycleOwner) { maxPitchDown -> // Value is positive
            binding.minNegativePitchStatText.text = String.format(Locale.US, "Max Pitch (Down): -%.1f°", maxPitchDown)
        }
        musicViewModel.jumpCount.observe(viewLifecycleOwner) { count ->
            binding.jumpCountStatText.text = "Jumps: $count"
        }
        musicViewModel.dropCount.observe(viewLifecycleOwner) { count ->
            binding.dropCountStatText.text = "Drops: $count"
        }

        musicViewModel.oneEightyCount.observe(viewLifecycleOwner) { count ->
            binding.oneEightyCountStatText.text = "180s: $count"
        }
    }

    /**
     * Update UI state based on BLE connection status
     * Disables auto mode switches when BLE is not connected
     */
    private fun updateUiBasedOnBleConnection() {
        val connected = isBleConnected()
        Log.d(TAG, "updateUiBasedOnBleConnection: BLE connected = $connected")
        if (!connected) {
            if (binding.wheelSpeedModeSwitch.isChecked) binding.wheelSpeedModeSwitch.isChecked = false
            if (binding.pitchModeSwitch.isChecked) binding.pitchModeSwitch.isChecked = false
            if (binding.eventModeSwitch.isChecked) binding.eventModeSwitch.isChecked = false
            if (binding.hallDirectionModeSwitch.isChecked) binding.hallDirectionModeSwitch.isChecked = false
            if (binding.autoAllSwitch.isChecked) binding.autoAllSwitch.isChecked = false
        }
    }

    /**
     * Configure a SeekBar with label and change listener
     * @param seekBar The SeekBar to configure
     * @param label TextView to display current value
     * @param labelText Base text for the label
     * @param max Maximum value for the SeekBar
     * @param offset Value offset for calculations
     * @param initialProgress Initial progress value
     * @param onChange Callback for value changes
     */
    private fun setupSeekBar(
        seekBar: SeekBar, label: TextView, labelText: String,
        max: Int, offset: Int, initialProgress: Int,
        onChange: (Float) -> Unit
    ) {
        seekBar.max = max
        val isAssociatedAutoModeActive = when (seekBar.id) {
            R.id.wheelSpeedSeekBar -> musicViewModel.isWheelSpeedAuto.value ?: false
            R.id.pitchSeekBar -> musicViewModel.isPitchAuto.value ?: false
            else -> false
        }

        if (!isAssociatedAutoModeActive) {
            seekBar.progress = initialProgress
            val initialValue = initialProgress + offset
            label.text = "$labelText: $initialValue"
            // onChange(initialValue.toFloat()) // ViewModel will set initial FMOD params if needed
        } else {
            // Value will be set by observing ViewModel's display LiveData
        }

        seekBar.setOnSeekBarChangeListener(object : SimpleSeekBarChangeListener() {
            override fun onProgressChanged(sBar: SeekBar?, progress: Int, fromUser: Boolean) {
                if (fromUser) {
                    val value = progress + offset
                    label.text = "$labelText: $value"
                    onChange(value.toFloat())
                }
            }
        })
    }

    private fun showToast(msg: String) {
        if (isAdded) {
            Toast.makeText(requireContext(), msg, Toast.LENGTH_SHORT).show()
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        Log.d(TAG, "onDestroyView")
        _binding = null
        handler.removeCallbacksAndMessages(null)
    }

    /**
     * Base implementation of SeekBarChangeListener with empty methods
     * Allows subclasses to override only needed methods
     */
    open class SimpleSeekBarChangeListener : SeekBar.OnSeekBarChangeListener {
        override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {}
        override fun onStartTrackingTouch(seekBar: SeekBar?) {}
        override fun onStopTrackingTouch(seekBar: SeekBar?) {}
    }
}