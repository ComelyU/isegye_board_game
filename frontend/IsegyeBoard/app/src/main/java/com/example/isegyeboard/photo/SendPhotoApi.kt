package com.example.isegyeboard.photo

import com.example.isegyeboard.baseapi.BasicResponse
import okhttp3.MultipartBody
import retrofit2.http.Multipart
import retrofit2.http.POST
import retrofit2.http.Part

interface SendPhotoApi {
    @Multipart
    @POST("send_photo")
    suspend fun sendPhoto(
        @Part photo: MultipartBody.Part
    ): BasicResponse
}