package com.example.isegyeboard.room_login

import com.example.isegyeboard.baseapi.BasicResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path
import retrofit2.http.Query

interface LoginApi {
    @GET("stores/rooms/valid/{storeId}")
    fun sendStoreInfo(
        @Path("storeId") storeId: String ,
        @Query("roomNumber") roomNumber: String
    ): Call<Int>
}