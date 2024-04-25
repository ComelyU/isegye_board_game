package com.example.isegyeboard.room_login

import com.example.isegyeboard.baseapi.BasicResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

interface LoginApi {
    @POST("room/login")
    fun sendStoreInfo(@Body requestBody: Map<String, String?>): Call<BasicResponse>
}