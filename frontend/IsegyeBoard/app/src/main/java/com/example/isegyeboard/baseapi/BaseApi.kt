package com.example.isegyeboard.baseapi

import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory

object BaseApi {
    private val BASE_URL = "https://j10a706.p.ssafy.io/api/"

    private val client = Retrofit
        .Builder()
        .baseUrl(BASE_URL)
        .addConverterFactory(GsonConverterFactory.create())
        .build()

    fun getInstance() : Retrofit {
        return client
    }
}