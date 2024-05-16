import org.jetbrains.kotlin.storage.CacheResetOnProcessCanceled.enabled


plugins {
    alias(libs.plugins.androidApplication)
    alias(libs.plugins.jetbrainsKotlinAndroid)
    id("com.google.gms.google-services")
    id("androidx.navigation.safeargs")
}

android {
    namespace = "com.example.isegyeboard"
    compileSdk = 34

    buildFeatures {
        dataBinding = true
    }

    defaultConfig {
        applicationId = "com.example.isegyeboard"
        minSdk = 24
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
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
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }

    buildFeatures {
        viewBinding = true
    }
}

dependencies {

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)
    implementation(libs.androidx.activity)
    implementation(libs.androidx.constraintlayout)

    // 레트로핏
    implementation(libs.converter.gson)
    implementation(libs.retrofit)

    //glide
    implementation (libs.glide)
    implementation(libs.androidx.activity.v182)
    implementation(libs.androidx.work.runtime.ktx)
    annotationProcessor (libs.glide.compiler)

    // Firebase
    implementation(platform(libs.firebase.bom))
//    implementation("com.google.firebase:firebase-analytics")
    implementation(libs.firebase.messaging)

    // safe args
    implementation(libs.androidx.navigation.fragment.ktx)
    implementation(libs.androidx.navigation.ui.ktx)

    //cameraX
    implementation(libs.androidx.camera.core.v133)
    implementation(libs.androidx.camera.camera2.v133)
    implementation(libs.androidx.camera.lifecycle.v133)
    implementation(libs.androidx.camera.view.v133)

    // okhttp3
    implementation("com.squareup.okhttp3:okhttp:4.9.0")

    // qr code
    implementation("com.google.zxing:core:3.4.1")

    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
}