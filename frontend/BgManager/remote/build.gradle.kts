plugins {
    id("java-library")
    alias(libs.plugins.jetbrainsKotlinJvm)
    id("kotlin-kapt")
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

dependencies {
    // 레트로핏
    implementation(libs.retrofit)
    implementation(libs.converter.gson)
    implementation(libs.converter.scalars)

    //okHttp
    implementation(libs.okhttp)
    implementation(libs.logging.interceptor)

    //Gson
    implementation(libs.gson)

    // 코루틴
    implementation(libs.kotlinx.coroutines.android)
    implementation(libs.kotlinx.coroutines.core)

    //hilt
    kapt(libs.hilt.android.compiler)
    implementation(libs.hilt.core)

    implementation(project(":data"))
}