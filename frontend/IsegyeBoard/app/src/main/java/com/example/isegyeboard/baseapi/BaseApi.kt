package com.example.isegyeboard.baseapi

import com.google.gson.GsonBuilder
import okhttp3.OkHttpClient
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import java.util.concurrent.TimeUnit

object BaseApi {
    private const val BASE_URL = "https://k10a706.p.ssafy.io/api/"

    private val okHttpClient = OkHttpClient.Builder()
        .connectTimeout(30, TimeUnit.SECONDS) // 연결 제한 시간 (초)
        .callTimeout(30, TimeUnit.SECONDS)
        .readTimeout(30, TimeUnit.SECONDS) // 읽기 제한 시간 (초)
        .writeTimeout(30, TimeUnit.SECONDS) // 쓰기 제한 시간 (초)
        .build()

    private val client = Retrofit
        .Builder()
        .baseUrl(BASE_URL)
        .client(okHttpClient)
        .addConverterFactory(GsonConverterFactory.create(GsonBuilder().setLenient().create()))
        .build()

    fun getInstance() : Retrofit {
        return client
    }
}