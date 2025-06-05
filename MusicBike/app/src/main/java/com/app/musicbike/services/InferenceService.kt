package com.app.musicbike.services

import android.app.Service
import android.content.Intent
import android.os.Binder
import android.os.Handler
import android.os.HandlerThread
import android.os.IBinder
import android.os.Looper
import android.os.Message
import android.os.Process
import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData

// Google Play services TensorFlow Lite API
import com.google.android.gms.tflite.java.TfLite
import com.google.android.gms.tasks.Task
import org.tensorflow.lite.InterpreterApi
import org.tensorflow.lite.InterpreterApi.Options.TfLiteRuntime
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer
import org.tensorflow.lite.DataType
import org.tensorflow.lite.support.metadata.MetadataExtractor

// Standard Java/Kotlin imports
import java.io.IOException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

// Add imports for foreground service
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.os.Build
import androidx.core.app.NotificationCompat
import com.app.musicbike.R
import com.app.musicbike.ui.activities.MainActivity
import java.io.BufferedReader
import java.io.FileInputStream
import java.io.InputStreamReader

class InferenceService : Service() {

    private var interpreter: InterpreterApi? = null
    private var modelByteBuffer: MappedByteBuffer? = null
    private var metadataExtractor: MetadataExtractor? = null
    private val initializeTask: Task<Void> by lazy { TfLite.initialize(this) }

    private lateinit var inputShape: IntArray
    private lateinit var outputShape: IntArray

    private val TFLITE_MODEL_FILENAME = "trick_detector_compatible_with_metadata.tflite" // FP32 model with metadata
    private val TAG = "InferenceService"

    private var serviceLooper: Looper? = null
    private var serviceHandler: ServiceHandler? = null
    private val bufferLock = ReentrantLock()

    // Foreground service constants
    private val NOTIFICATION_CHANNEL_ID = "inference_service_channel"
    private val NOTIFICATION_ID = 1
    private val CHANNEL_NAME = "ML Inference Service"
    private val BUFFER_SIZE = 800
    private val INFERENCE_TRIGGER_COUNT = 100
    private val sensorBuffer = ArrayList<SensorReading>(BUFFER_SIZE)
    private var writeCount = 0
    private var classNames: List<String> = emptyList()
    private val binder = LocalBinder()
    private val _inferenceResult = MutableLiveData<String>()
    val inferenceResult: LiveData<String> get() = _inferenceResult
    var isModelReady = false

    // Data class for sensor readings
    data class SensorReading(
        val pitch: Float,
        val roll: Float,
        val yaw: Float,
        val gforce: Float,
        val timestamp: Long = System.currentTimeMillis()
    )

    // Handler that receives messages from the thread
    private inner class ServiceHandler(looper: Looper) : Handler(looper) {
        override fun handleMessage(msg: Message) {
            try {
                when (msg.what) {
                    MSG_ADD_SENSOR_DATA -> {
                        val sensorReading = msg.obj as? SensorReading
                        sensorReading?.let { reading -> addSensorDataToBuffer(reading) }
                    }
                    MSG_RUN_INFERENCE -> {
                        Log.d(TAG, "Running inference on current buffer...")
                        performInference()
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error in ServiceHandler: ${e.message}", e)
            }
        }
    }

    companion object {
        private const val MSG_ADD_SENSOR_DATA = 1
        private const val MSG_RUN_INFERENCE = 2
    }

    inner class LocalBinder : Binder() {
        fun getService(): InferenceService = this@InferenceService
    }

    override fun onCreate() {
        super.onCreate()
        Log.d(TAG, "onCreate")
        HandlerThread("InferenceServiceThread", Process.THREAD_PRIORITY_BACKGROUND).apply {
            start()
            serviceLooper = looper
            serviceHandler = ServiceHandler(looper)
        }
        initializeSensorBuffer()
        loadLiteRTModel()
        try {
            createNotificationChannel()
            startForeground(NOTIFICATION_ID, createNotification())
            Log.d(TAG, "Foreground service started successfully")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start foreground service: ${e.message}", e)
        }
    }

    private fun initializeSensorBuffer() {
        bufferLock.withLock {
            sensorBuffer.clear()
            writeCount = 0
        }
        Log.d(TAG, "Sensor buffer initialized with size $BUFFER_SIZE")
    }

    fun addSensorData(pitch: Float, roll: Float, yaw: Float, gforce: Float) {
        val sensorReading = SensorReading(pitch, roll, yaw, gforce)
        serviceHandler?.obtainMessage(MSG_ADD_SENSOR_DATA, sensorReading)?.sendToTarget()
    }

    private fun addSensorDataToBuffer(sensorReading: SensorReading) {
        bufferLock.withLock {
            if (sensorBuffer.size >= BUFFER_SIZE) {
                sensorBuffer.removeAt(0)
            }
            sensorBuffer.add(sensorReading)
            writeCount++
            Log.v(TAG, "Added sensor data: ${sensorReading}, Buffer size: ${sensorBuffer.size}, Write count: $writeCount")

            val modelSeqLen = if (::inputShape.isInitialized && inputShape.size > 1) inputShape[1] else -1
            if (modelSeqLen > 0 && writeCount % INFERENCE_TRIGGER_COUNT == 0 && sensorBuffer.size >= modelSeqLen) {
                 Log.d(TAG, "Triggering inference after $writeCount writes, buffer has ${sensorBuffer.size} items, model needs $modelSeqLen")
                serviceHandler?.obtainMessage(MSG_RUN_INFERENCE)?.sendToTarget()
            } else if (modelSeqLen <= 0 && writeCount % INFERENCE_TRIGGER_COUNT == 0) {
                Log.d(TAG, "Triggering inference after $writeCount writes, but model input shape not ready yet.")
            }
        }
    }
    
    @Throws(IOException::class)
    private fun loadModelFile(modelFilename: String): MappedByteBuffer {
        assets.openFd(modelFilename).use { fileDescriptor ->
            FileInputStream(fileDescriptor.fileDescriptor).use { inputStream ->
                val fileChannel = inputStream.channel
                val startOffset = fileDescriptor.startOffset
                val declaredLength = fileDescriptor.declaredLength
                return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength)
            }
        }
    }

    private fun loadLiteRTModel() {
        initializeTask.addOnSuccessListener {
            try {
                modelByteBuffer = loadModelFile(TFLITE_MODEL_FILENAME)
                modelByteBuffer?.let { buffer ->
                    metadataExtractor = MetadataExtractor(buffer.duplicate())
                    
                    inputShape = metadataExtractor!!.getInputTensorShape(0)
                    outputShape = metadataExtractor!!.getOutputTensorShape(0)

                    try {
                        metadataExtractor!!.getAssociatedFile("trick_labels.txt").use { inputStream ->
                            BufferedReader(InputStreamReader(inputStream)).use { reader ->
                                classNames = reader.readLines().filter { it.isNotBlank() }.map { it.trim() }
                            }
                        }
                         Log.i(TAG, "Loaded ${classNames.size} class names from metadata: ${classNames.joinToString()}")
                    } catch (e: Exception) {
                        Log.e(TAG, "Failed to load 'trick_labels.txt' from metadata: ${e.message}", e)
                        classNames = listOf("ErrorLoadingLabels")
                    }
                    
                    val interpreterOptions = InterpreterApi.Options().setRuntime(TfLiteRuntime.FROM_SYSTEM_ONLY)
                    interpreter = InterpreterApi.create(buffer, interpreterOptions)

                    Log.i(TAG, "Input Tensor (from metadata): shape=${inputShape.joinToString()}, type=FLOAT32")
                    Log.i(TAG, "Output Tensor (from metadata): shape=${outputShape.joinToString()}, type=FLOAT32")
                    Log.d(TAG, "Google Play services TensorFlow Lite FP32 interpreter initialized successfully with metadata.")
                    isModelReady = true
                } ?: run {
                    throw IOException("Model byte buffer is null after loading.")
                }

            } catch (e: Exception) {
                Log.e(TAG, "Error loading model with Google Play services TF Lite: ${e.message}", e)
                sendResultToClient("Error: Failed to load model - ${e.message}")
                interpreter = null 
                metadataExtractor = null
                modelByteBuffer = null
                isModelReady = false
            }
        }.addOnFailureListener { e ->
            Log.e(TAG, "Cannot initialize Google Play services TensorFlow Lite", e)
            sendResultToClient("Error: Failed to initialize TF Lite - ${e.message}")
            isModelReady = false
        }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        Log.d(TAG, "onStartCommand, startId: $startId")
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            try {
                startForeground(NOTIFICATION_ID, createNotification())
            } catch (e: Exception) {
                Log.e(TAG, "Failed to start foreground in onStartCommand: ${e.message}", e)
            }
        }
        return START_STICKY
    }

    override fun onBind(intent: Intent): IBinder? {
        Log.d(TAG, "onBind")
        return binder
    }

    override fun onDestroy() {
        super.onDestroy()
        Log.d(TAG, "onDestroy")
        interpreter?.close()
        Log.d(TAG, "LiteRT interpreter closed.")
        serviceLooper?.quitSafely()
    }

    fun getClassNames(): List<String> = classNames
    fun getClassName(index: Int): String = classNames.getOrNull(index) ?: "Class $index (OOB)"
    fun getClassInfo(): String = if (classNames.isNotEmpty() && classNames.first() != "ErrorLoadingLabels") {
        "Model loaded with ${classNames.size} classes: ${classNames.joinToString(", ")}"
    } else {
        "Class names not loaded from metadata or error occurred."
    }
    fun hasCustomClassNames(): Boolean = classNames.isNotEmpty() && classNames.first() != "ErrorLoadingLabels"


    private fun prepareInputData(): TensorBuffer? {
        if (metadataExtractor == null || !::inputShape.isInitialized || inputShape.isEmpty()) {
            Log.e(TAG, "MetadataExtractor or inputShape not initialized.")
            return null
        }

        val modelSeqLen = inputShape[1]
        val numFeatures = inputShape[2]
        val currentBufferCopy: MutableList<SensorReading> = mutableListOf()

        val canProceed = bufferLock.withLock {
            if (sensorBuffer.size < modelSeqLen) {
                Log.w(TAG, "Buffer not full enough for sequence (${sensorBuffer.size}/$modelSeqLen), skipping inference")
                false
            } else {
                val startIdx = sensorBuffer.size - modelSeqLen
                for (i in startIdx until sensorBuffer.size) {
                    currentBufferCopy.add(sensorBuffer[i])
                }
                true
            }
        }

        if (!canProceed) return null

        // Create input buffer for FP32 model - no quantization needed
        val inputTensorBuffer = TensorBuffer.createFixedSize(inputShape, DataType.FLOAT32)
        
        // Prepare flat float array for the TensorBuffer
        // FP32 model expects [1, modelSeqLen, numFeatures] in float32 format
        val floatInputArray = FloatArray(1 * modelSeqLen * numFeatures)
        var dstIdx = 0
        for (i in 0 until modelSeqLen) {
            val reading = currentBufferCopy[i]
            val features = floatArrayOf(reading.pitch, reading.roll, reading.yaw, reading.gforce)
            for (j in 0 until numFeatures) {
                if (j < features.size) {
                    floatInputArray[dstIdx++] = features[j]
                } else {
                    floatInputArray[dstIdx++] = 0.0f
                }
            }
        }
        
        inputTensorBuffer.loadArray(floatInputArray)
        Log.d(TAG, "FP32 Input TensorBuffer prepared. Shape: ${inputTensorBuffer.shape.joinToString()}, DataType: ${inputTensorBuffer.dataType}")
        return inputTensorBuffer
    }
    
    private fun performInference() {
        if (interpreter == null || metadataExtractor == null || !::inputShape.isInitialized || !::outputShape.isInitialized || !isModelReady) {
            Log.e(TAG, "Interpreter, metadata, or shapes not initialized properly.")
            sendResultToClient("Error: Model components not ready")
            return
        }

        val inputTensorBuffer = prepareInputData()
        if (inputTensorBuffer == null) {
            sendResultToClient("Error: Failed to prepare input data")
            return
        }

        try {
            // Create output buffer for FP32 model - no dequantization needed
            val outputTensorBuffer = TensorBuffer.createFixedSize(outputShape, DataType.FLOAT32)

            Log.d(TAG, "FP32 Input TensorBuffer ready. Shape: ${inputTensorBuffer.shape.joinToString()}, Type: ${inputTensorBuffer.dataType}")
            Log.d(TAG, "FP32 Output TensorBuffer created. Shape: ${outputTensorBuffer.shape.joinToString()}, Type: ${outputTensorBuffer.dataType}")

            Log.d(TAG, "Running inference with interpreter.run(inputBuffer.buffer, outputBuffer.buffer)...")
            val startTime = System.currentTimeMillis()
            
            interpreter!!.run(inputTensorBuffer.buffer, outputTensorBuffer.buffer)
            
            val inferenceTime = System.currentTimeMillis() - startTime
            Log.i(TAG, "FP32 Inference completed in ${inferenceTime}ms")
            
            // Get results directly as float32 - no dequantization needed for FP32 model
            val probabilities = outputTensorBuffer.floatArray
            Log.d(TAG, "FP32 Output retrieved. Probabilities size: ${probabilities.size}")
             if (probabilities.isNotEmpty()) {
                val firstFew = probabilities.take(minOf(5, probabilities.size))
                                      .joinToString { value -> String.format("%.4f", value) }
                Log.d(TAG, "FP32 Output (first ${firstFew.split(',').size}): $firstFew")
            }
            
            processOutput(probabilities, inferenceTime)
            
        } catch (e: Exception) {
            Log.e(TAG, "Error running inference: ${e.message}", e)
            sendResultToClient("Error in inference: ${e.message}")
        }
    }

    private fun processOutput(outputData: FloatArray, inferenceTime: Long) {
        Log.d(TAG, "Processing output...")
        if (outputData.isEmpty()) {
            sendResultToClient("Result: No output data, Time: ${inferenceTime}ms")
            return
        }

        Log.d(TAG, "Output (probabilities): ${outputData.joinToString(limit = 10) { value -> String.format("%.4f", value) }}")

        val maxIndex = outputData.indices.maxByOrNull { outputData[it] } ?: -1
        val confidence = if (maxIndex >= 0 && maxIndex < outputData.size) outputData[maxIndex] else 0f
        val className = getClassName(maxIndex)
        val confidencePercent = (confidence * 100).coerceIn(0f, 100f)
        sendResultToClient("Prediction: $className (${confidencePercent.toInt()}%), Time: ${inferenceTime}ms")
    }

    private fun sendResultToClient(result: String) {
        val intent = Intent("com.app.musicbike.INFERENCE_RESULT")
        intent.putExtra("result_data", result)
        androidx.localbroadcastmanager.content.LocalBroadcastManager.getInstance(this).sendBroadcast(intent)
        Log.d(TAG, "Result broadcasted: $result")
        _inferenceResult.postValue(result)
    }

    fun triggerInference() = serviceHandler?.obtainMessage(MSG_RUN_INFERENCE)?.sendToTarget()
    
    fun retryLoadModel() { 
        Log.d(TAG, "Retrying model load...")
        isModelReady = false
        interpreter = null
        metadataExtractor = null
        modelByteBuffer = null
        loadLiteRTModel() 
    }
    
    fun checkModelReady(): Boolean = isModelReady && interpreter != null && metadataExtractor != null && ::inputShape.isInitialized && ::outputShape.isInitialized && classNames.isNotEmpty() && classNames.first() != "ErrorLoadingLabels"
    
    fun getBufferStatus(): String {
        return bufferLock.withLock {
            val modelStatus = if (checkModelReady()) "Model: Ready" else "Model: Not loaded/Ready"
            val modelSeqLen = if (::inputShape.isInitialized && inputShape.size > 1) inputShape[1] else "N/A"
            "Buffer: ${sensorBuffer.size}/$BUFFER_SIZE (Model needs $modelSeqLen), Writes: $writeCount, $modelStatus"
        } 
    }
    
    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(NOTIFICATION_CHANNEL_ID, CHANNEL_NAME, NotificationManager.IMPORTANCE_LOW).apply {
                description = "Handles ML inference for bike trick recognition"
                setShowBadge(false); enableVibration(false); setSound(null, null)
            }
            (getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager).createNotificationChannel(channel)
        }
    }

    private fun createNotification(): Notification {
        val notificationIntent = Intent(this, MainActivity::class.java)
        val pendingIntentFlags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        } else { PendingIntent.FLAG_UPDATE_CURRENT }
        val pendingIntent = PendingIntent.getActivity(this, 0, notificationIntent, pendingIntentFlags)
        return NotificationCompat.Builder(this, NOTIFICATION_CHANNEL_ID)
            .setSmallIcon(R.drawable.ic_music_notification)
            .setContentTitle("ML Inference Service")
            .setContentText("Processing sensor data for bike trick recognition")
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .setContentIntent(pendingIntent)
            .setOngoing(true).setAutoCancel(false).build()
    }
}
