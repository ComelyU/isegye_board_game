package com.example.isegyeboard.recommend

import com.example.isegyeboard.baseapi.BasicResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

interface RecommendApi {
    @POST("recommend")
    fun sendRecommendData(@Body requestBody: Map<String, String?>): Call<BasicResponse>
}