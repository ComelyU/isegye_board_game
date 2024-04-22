package com.example.isegyeboard.login

import com.example.isegyeboard.baseapi.BasicResponse
import com.example.isegyeboard.room_history.RoomStartResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST
import retrofit2.http.PUT

interface StartApi {
    @POST("room")
    fun sendRoomInfo(@Body requestBody: Map<String, String?>): Call<RoomStartResponse>

    @PUT("room")
    fun deleteRoomInfo(@Body requestBody: Map<String, String?>): Call<BasicResponse>
}