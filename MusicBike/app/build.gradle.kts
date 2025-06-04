plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.app.musicbike"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.app.musicbike"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        externalNativeBuild {
            cmake {
                cppFlags += ""
            }
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    buildFeatures {
        viewBinding = true
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    kotlinOptions {
        jvmTarget = "11"
    }
    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }
}

dependencies {

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)
    implementation(libs.androidx.activity)
    implementation(libs.androidx.constraintlayout)
    implementation(files("libs/fmod.jar"))
    
    // LocalBroadcastManager for result broadcasting
    implementation("androidx.localbroadcastmanager:localbroadcastmanager:1.1.0")
    implementation(libs.litert.metadata)

    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    implementation(libs.androidx.viewpager2)
    // Lifecycle components including LiveData
    implementation(libs.androidx.lifecycle.livedata.ktx) // Use the latest stable version
    implementation(libs.androidx.lifecycle.viewmodel.ktx) // Often used with LiveData
    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.fragment.ktx) // Provides viewModels delegate
    implementation(libs.androidx.swiperefreshlayout) // Used for ML file refresh

    // Google Play services TensorFlow Lite dependencies (using version catalog)
    implementation(libs.play.services.tflite.java)
    // Optional: include TensorFlow Lite Support Library
    implementation(libs.play.services.tflite.support)

}