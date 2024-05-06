package com.example.isegyeboard.login

import com.example.isegyeboard.baseapi.BasicResponse
import com.example.isegyeboard.room_history.OrderMenuResponse
import com.example.isegyeboard.room_history.RoomStartResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.PUT
import retrofit2.http.Path

interface RoomApi {
    @POST("room")
    fun sendRoomInfo(@Body requestBody: Map<String, String?>): Call<RoomStartResponse>

    @PUT("room")
    fun deleteRoomInfo(@Body requestBody: Map<String, String?>): Call<BasicResponse>

    @GET("menu/order/{customerId}")
    suspend fun getHistoryList(
        @Path("customerId") customerId: String,
    ) : List<OrderMenuResponse>

}