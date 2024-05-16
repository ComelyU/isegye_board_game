package com.example.isegyeboard.photo

import okhttp3.MultipartBody
import okhttp3.ResponseBody
import retrofit2.Response
import retrofit2.http.Multipart
import retrofit2.http.POST
import retrofit2.http.Part
import retrofit2.http.Path
import retrofit2.http.Streaming

interface SendPhotoApi {
    @Multipart
    @POST("customer/{customerId}/swapface")
    @Streaming
    suspend fun sendPhoto(
        @Path("customerId") customerId: String,
        @Part sourceImg: MultipartBody.Part
    ): ResponseBody
}