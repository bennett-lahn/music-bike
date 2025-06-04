import tensorflow as tf
import numpy as np
import os
import glob
from sklearn.model_selection import train_test_split
from tflite_support import flatbuffers
from tflite_support import metadata as _metadata
from tflite_support import metadata_schema_py_generated as _metadata_fb
from tflite_support.metadata_writers import metadata_info
from tflite_support.metadata_writers import writer_utils

TIMESTEPS = 220  # 10 seconds at 50Hz
NUM_FEATURES = 4  # pitch, roll, yaw, gForce
NUM_CLASSES = 3
BATCH_SIZE = 32
EPOCHS = 50

# Custom Dense layer that uses older TensorFlow operations for better TFLite compatibility
class CompatibleDense(tf.keras.layers.Layer):
    def __init__(self, units, activation=None, use_bias=True, **kwargs):
        super(CompatibleDense, self).__init__(**kwargs)
        self.units = units
        self.activation = tf.keras.activations.get(activation)
        self.use_bias = use_bias

    def build(self, input_shape):
        self.w = self.add_weight(
            shape=(input_shape[-1], self.units),
            initializer='glorot_uniform',
            trainable=True,
            name='kernel'
        )
        if self.use_bias:
            self.b = self.add_weight(
                shape=(self.units,),
                initializer='zeros',
                trainable=True,
                name='bias'
            )
        super(CompatibleDense, self).build(input_shape)

    def call(self, inputs):
        # Use MatMul instead of Dense/FullyConnected
        output = tf.linalg.matmul(inputs, self.w)
        if self.use_bias:
            output = tf.add(output, self.b)
        if self.activation is not None:
            output = self.activation(output)
        return output

def load_real_data(timesteps=150):
    X = []
    y = []
    label_map = {}
    
    # Find all data files in current directory
    files = glob.glob('./*.txt')
    
    if not files:
        raise FileNotFoundError("No .txt files found in directory")
    
    for file_path in files:
        # Extract trick name from filename - handle both patterns:
        # Pattern 1: [trick]_[num].txt -> extract [trick]
        # Pattern 2: [trick].txt -> extract [trick]
        filename = os.path.basename(file_path)
        if '_' in filename:
            # Pattern 1: trick_num.txt
            trick_name = filename.split('_')[0]
        else:
            # Pattern 2: trick.txt
            trick_name = filename.replace('.txt', '')
        
        # Read IMU data from file
        sequence = []
        with open(file_path, 'r') as f:
            for line in f:
                if line.strip():  # Skip empty lines
                    try:
                        # Parse: timestamp, pitch, roll, yaw, gForce, hallDir, speed
                        values = line.strip().split(",")
                        if len(values) >= 5:  # Ensure we have at least timestamp, pitch, roll, yaw, gForce
                            pitch, roll, yaw, gForce = float(values[1]), float(values[2]), float(values[3]), float(values[4])
                            sequence.append([pitch, roll, yaw, gForce])
                    except:
                        continue  # Skip malformed lines
        
        # Pad/truncate to fixed timesteps
        if len(sequence) < timesteps:
            # Pad with last value repeated
            pad_needed = timesteps - len(sequence)
            sequence += [sequence[-1]] * pad_needed if sequence else [[0.0, 0.0, 0.0, 0.0]] * pad_needed
        else:
            sequence = sequence[:timesteps]
        
        X.append(sequence)
        y.append(trick_name)
    
    # Create label mapping
    unique_labels = sorted(list(set(y)))
    if len(unique_labels) != 3:
        raise ValueError(f"Exactly 3 classes required. Found: {unique_labels}")
    label_map = {label: idx for idx, label in enumerate(unique_labels)}
    print(f"Class labels: {unique_labels}")
    
    # Save labels to file for metadata
    with open('trick_labels.txt', 'w') as f:
        for label in unique_labels:
            f.write(f"{label}\n")
    
    return np.array(X, dtype=np.float32), np.array([label_map[label] for label in y]), unique_labels

def augment_imu_data(X, y, augmentation_factor=8):
    """
    Augment IMU data with realistic transformations for bike trick detection.
    
    Args:
        X: Input sequences (samples, timesteps, features)
        y: Labels
        augmentation_factor: How many augmented samples per original sample
    
    Returns:
        Augmented X and y arrays
    """
    augmented_X = []
    augmented_y = []
    
    # Keep original data
    augmented_X.extend(X)
    augmented_y.extend(y)
    
    for i in range(len(X)):
        original_sequence = X[i]
        original_label = y[i]
        
        for aug_idx in range(augmentation_factor):
            augmented_sequence = original_sequence.copy()
            
            # 1. Add Gaussian noise (sensor noise simulation)
            noise_std = 0.02  # Small noise
            noise = np.random.normal(0, noise_std, augmented_sequence.shape)
            augmented_sequence += noise
            
            # 2. Scale variations (different g-force ranges)
            scale_factor = np.random.uniform(0.9, 1.1)
            augmented_sequence[:, 3] *= scale_factor  # Scale gForce only
            
            # 3. Time shifting (slight timing variations)
            shift_amount = np.random.randint(-10, 11)  # ±10 timesteps
            if shift_amount != 0:
                if shift_amount > 0:
                    # Shift right, pad with first values
                    augmented_sequence = np.concatenate([
                        np.repeat([augmented_sequence[0]], shift_amount, axis=0),
                        augmented_sequence[:-shift_amount]
                    ])
                else:
                    # Shift left, pad with last values
                    augmented_sequence = np.concatenate([
                        augmented_sequence[-shift_amount:],
                        np.repeat([augmented_sequence[-1]], -shift_amount, axis=0)
                    ])
            
            # 4. Rotation noise (IMU orientation variations)
            rotation_noise = np.random.normal(0, 1.0, (augmented_sequence.shape[0], 3))  # pitch, roll, yaw
            augmented_sequence[:, :3] += rotation_noise
            
            # 5. Random dropout (simulate sensor glitches)
            if np.random.random() < 0.3:  # 30% chance
                dropout_start = np.random.randint(0, len(augmented_sequence) - 5)
                dropout_length = np.random.randint(1, 6)  # 1-5 timesteps
                dropout_end = min(dropout_start + dropout_length, len(augmented_sequence))
                # Replace with interpolated values
                if dropout_start > 0 and dropout_end < len(augmented_sequence):
                    for col in range(augmented_sequence.shape[1]):
                        start_val = augmented_sequence[dropout_start - 1, col]
                        end_val = augmented_sequence[dropout_end, col]
                        interpolated = np.linspace(start_val, end_val, dropout_end - dropout_start)
                        augmented_sequence[dropout_start:dropout_end, col] = interpolated
            
            augmented_X.append(augmented_sequence)
            augmented_y.append(original_label)
    
    return np.array(augmented_X, dtype=np.float32), np.array(augmented_y)

def create_compatible_imu_model(input_shape, num_classes):
    """
    TFLite-compatible CNN model using only standard builtin operations.
    Replaces LSTM with CNN layers for better TFLite compatibility.
    """    
    inputs = tf.keras.layers.Input(shape=input_shape)
    
    # Multi-scale feature extraction
    x1 = tf.keras.layers.Conv1D(16, 3, activation='relu', padding='same')(inputs)
    x2 = tf.keras.layers.Conv1D(16, 7, activation='relu', padding='same')(inputs)
    x3 = tf.keras.layers.Conv1D(16, 11, activation='relu', padding='same')(inputs)
    
    # Combine features
    x = tf.keras.layers.Concatenate()([x1, x2, x3])  # 48 features
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.MaxPooling1D(2)(x)  # Reduce from 220 to 110
    
    # Second convolution layer
    x = tf.keras.layers.Conv1D(64, 5, activation='relu', padding='same')(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.MaxPooling1D(2)(x)  # Reduce from 110 to 55
    
    # Third convolution layer for temporal patterns (replaces LSTM)
    x = tf.keras.layers.Conv1D(32, 3, activation='relu', padding='same')(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.MaxPooling1D(2)(x)  # Reduce from 55 to 27
    
    # Global pooling to aggregate temporal information
    x = tf.keras.layers.GlobalAveragePooling1D()(x)
    
    # Classification layers
    x = tf.keras.layers.Dense(64, activation='relu')(x)
    x = tf.keras.layers.Dropout(0.5)(x)
    x = tf.keras.layers.Dense(32, activation='relu')(x)
    x = tf.keras.layers.Dropout(0.3)(x)
    outputs = tf.keras.layers.Dense(num_classes, activation='softmax')(x)
    
    model = tf.keras.Model(inputs=inputs, outputs=outputs, name='simple_trick_detector')
    
    # Compile with slightly higher learning rate for faster convergence
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.002),
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    
    return model

def add_comprehensive_metadata(model_path, output_path, labels_file, class_names):
    """
    Add comprehensive metadata including quantization parameters and class information.
    """
    # Load the model to get quantization parameters
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    
    input_details = interpreter.get_input_details()[0]
    output_details = interpreter.get_output_details()[0]
    
    # Create model metadata
    model_meta = _metadata_fb.ModelMetadataT()
    model_meta.name = "Bike Trick Detector"
    model_meta.description = ("IMU-based bike trick classification model. "
                             "Detects bike tricks from pitch, roll, yaw, and gForce sensor data.")
    model_meta.version = "v1.0.0"
    model_meta.author = "Music Bike Team"
    
    # Input metadata with quantization parameters
    input_meta = _metadata_fb.TensorMetadataT()
    input_meta.name = "imu_sequence"
    input_meta.description = (
        f"Input IMU sequence with {TIMESTEPS} timesteps and {NUM_FEATURES} features. "
        "Features are: [pitch, roll, yaw, gForce]. Expected data range varies by sensor."
    )
    
    # Add quantization parameters for input if quantized
    if input_details['dtype'] == np.uint8 or input_details['dtype'] == np.int8:
        input_quantization = input_details['quantization_parameters']
        if input_quantization['scales'].size > 0:
            # Input quantization metadata
            input_quant_meta = _metadata_fb.QuantizationParametersT()
            input_quant_meta.scale = input_quantization['scales'].tolist()
            input_quant_meta.zeroPoint = input_quantization['zero_points'].tolist()
            input_meta.quantization = input_quant_meta
            
            print(f"Input quantization - Scale: {input_quantization['scales']}, "
                  f"Zero Point: {input_quantization['zero_points']}")
    
    # Input statistics
    input_stats = _metadata_fb.StatsT()
    input_stats.max = [180.0, 180.0, 180.0, 10.0]  # Max expected values for pitch, roll, yaw, gForce
    input_stats.min = [-180.0, -180.0, -180.0, -10.0]  # Min expected values
    input_meta.stats = input_stats
    
    # Output metadata with quantization parameters
    output_meta = _metadata_fb.TensorMetadataT()
    output_meta.name = "trick_probabilities"
    output_meta.description = f"Probability distribution over {NUM_CLASSES} bike trick classes."
    
    # Add quantization parameters for output if quantized
    if output_details['dtype'] == np.uint8 or output_details['dtype'] == np.int8:
        output_quantization = output_details['quantization_parameters']
        if output_quantization['scales'].size > 0:
            # Output quantization metadata
            output_quant_meta = _metadata_fb.QuantizationParametersT()
            output_quant_meta.scale = output_quantization['scales'].tolist()
            output_quant_meta.zeroPoint = output_quantization['zero_points'].tolist()
            output_meta.quantization = output_quant_meta
            
            print(f"Output quantization - Scale: {output_quantization['scales']}, "
                  f"Zero Point: {output_quantization['zero_points']}")
    
    # Output statistics
    output_stats = _metadata_fb.StatsT()
    output_stats.max = [1.0]
    output_stats.min = [0.0]
    output_meta.stats = output_stats
    
    # Associate label file with output
    label_file = _metadata_fb.AssociatedFileT()
    label_file.name = os.path.basename(labels_file)
    label_file.description = f"Labels for {NUM_CLASSES} bike trick classes: {', '.join(class_names)}"
    label_file.type = _metadata_fb.AssociatedFileType.TENSOR_AXIS_LABELS
    output_meta.associatedFiles = [label_file]
    
    # Create subgraph metadata
    subgraph = _metadata_fb.SubGraphMetadataT()
    subgraph.inputTensorMetadata = [input_meta]
    subgraph.outputTensorMetadata = [output_meta]
    model_meta.subgraphMetadata = [subgraph]
    
    # Build and populate metadata
    b = flatbuffers.Builder(0)
    b.Finish(
        model_meta.Pack(b),
        _metadata.MetadataPopulator.METADATA_FILE_IDENTIFIER)
    metadata_buf = b.Output()
    
    # Create populator and add metadata
    populator = _metadata.MetadataPopulator.with_model_file(model_path)
    populator.load_metadata_buffer(metadata_buf)
    populator.load_associated_files([labels_file])
    populator.populate()
    
    # Save the model with metadata
    with open(output_path, 'wb') as f:
        f.write(populator.get_model_buffer())
    
    print(f"✓ Metadata added successfully to {output_path}")
    return True

def convert_to_compatible_tflite(model, model_name='compatible_model'):
    """
    Convert model to TFLite with only builtin operations for maximum compatibility.
    """
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    
    # Use ONLY builtin operations - no SELECT_TF_OPS
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
    
    # Disable custom ops to force builtin-only conversion
    converter.allow_custom_ops = False
    
    try:
        tflite_model = converter.convert()
        with open(f'{model_name}.tflite', 'wb') as f:
            f.write(tflite_model)
        print(f"✓ Builtin-only model saved as {model_name}.tflite")
        return True
    except Exception as e:
        print(f"✗ Builtin-only conversion failed: {e}")
        return False

def convert_with_quantization(model, model_name='compatible_model_quant', X_train=None):
    """
    Convert with quantization using only builtin operations and representative dataset.
    """
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    
    # Enable quantization
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    
    # Use ONLY builtin operations
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
    
    # Disable custom ops to force builtin-only conversion
    converter.allow_custom_ops = False
    
    # Provide representative dataset for quantization
    if X_train is not None:
        def representative_dataset():
            # Use a subset of training data as representative dataset
            sample_size = min(100, len(X_train))
            indices = np.random.choice(len(X_train), sample_size, replace=False)
            for i in indices:
                # Convert to float32 and add batch dimension
                yield [X_train[i:i+1].astype(np.float32)]
        
        converter.representative_dataset = representative_dataset
        
        # Set inference input/output types to int8 for full integer quantization
        converter.target_spec.supported_types = [tf.int8]
        converter.inference_input_type = tf.int8
        converter.inference_output_type = tf.int8
    
    try:
        quantized_model = converter.convert()
        with open(f'{model_name}.tflite', 'wb') as f:
            f.write(quantized_model)
        print(f"✓ Quantized builtin-only model saved as {model_name}.tflite")
        return True
    except Exception as e:
        print(f"✗ Quantized builtin-only conversion failed: {e}")
        return False

# Main execution
if __name__ == "__main__":
    # Load and augment data
    print("Loading original data...")
    X_original, y_original, class_names = load_real_data(TIMESTEPS)
    print(f"Original data: {len(X_original)} samples")
    print(f"Classes: {class_names}")

    print("Augmenting data...")
    X_augmented, y_augmented = augment_imu_data(X_original, y_original, augmentation_factor=8)
    print(f"Augmented data: {len(X_augmented)} samples")

    X_train, X_test, y_train, y_test = train_test_split(X_augmented, y_augmented, test_size=0.2, stratify=y_augmented)

    print(f"Training samples: {len(X_train)}")
    print(f"Test samples: {len(X_test)}")

    # Create COMPATIBLE model instead of the original
    print("\nCreating TFLite-compatible model...")
    model = create_compatible_imu_model((TIMESTEPS, NUM_FEATURES), NUM_CLASSES)
    model.summary()

    # Training callbacks
    checkpoint = tf.keras.callbacks.ModelCheckpoint(
        'best_compatible_model.h5',
        monitor='val_accuracy',
        save_best_only=True,
        mode='max'
    )

    early_stop = tf.keras.callbacks.EarlyStopping(
        patience=15,
        restore_best_weights=True
    )

    reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(
        monitor='val_loss',
        factor=0.5,
        patience=8,
        min_lr=0.0001
    )

    # Train model
    print("\nTraining compatible model...")
    history = model.fit(
        X_train, y_train,
        validation_split=0.15,
        epochs=EPOCHS,
        batch_size=BATCH_SIZE,
        callbacks=[checkpoint, early_stop, reduce_lr],
        verbose=1
    )

    # Evaluate
    test_loss, test_acc = model.evaluate(X_test, y_test)
    print(f"\nTest accuracy: {test_acc:.2f}")

    # Convert with multiple compatibility approaches
    print("\nConverting to TFLite with compatibility optimizations...")

    # Try standard conversion
    success1 = convert_to_compatible_tflite(model, 'trick_detector_compatible')

    # Try quantized conversion with representative dataset
    success2 = convert_with_quantization(model, 'trick_detector_compatible_quant', X_train)

    # Add metadata to models
    print("\nAdding comprehensive metadata...")
    labels_file = 'trick_labels.txt'
    
    if success1:
        try:
            add_comprehensive_metadata(
                'trick_detector_compatible.tflite',
                'trick_detector_compatible_with_metadata.tflite',
                labels_file,
                class_names
            )
        except Exception as e:
            print(f"✗ Failed to add metadata to float model: {e}")
    
    if success2:
        try:
            add_comprehensive_metadata(
                'trick_detector_compatible_quant.tflite',
                'trick_detector_compatible_quant_with_metadata.tflite',
                labels_file,
                class_names
            )
        except Exception as e:
            print(f"✗ Failed to add metadata to quantized model: {e}")

    if success1 or success2:
        print("\n✓ Models created successfully!")
        print("\nFiles created:")
        if success1:
            print("- trick_detector_compatible.tflite (FP32, compatible)")
            print("- trick_detector_compatible_with_metadata.tflite (FP32, with metadata)")
        if success2:
            print("- trick_detector_compatible_quant.tflite (INT8, compatible)")
            print("- trick_detector_compatible_quant_with_metadata.tflite (INT8, with metadata)")
        print("- trick_labels.txt (class labels)")
        
        print(f"\nPlace the .tflite file and labels.txt in: app/src/main/assets/")
        print("Use the *_with_metadata.tflite versions for automatic quantization handling in Android.")
    else:
        print("\n✗ All conversion attempts failed. The model may need further simplification.")
