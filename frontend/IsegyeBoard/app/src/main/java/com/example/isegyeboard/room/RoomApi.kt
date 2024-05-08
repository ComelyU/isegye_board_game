package com.example.isegyeboard.room

import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.PATCH
import retrofit2.http.POST
import retrofit2.http.Path

interface RoomApi {
    @POST("customer/{roomId}")
    fun sendRoomInfo(
        @Path("roomId") roomId: String,
        @Body requestBody: Map<String, Int>
    ): Call<RoomStartResponse>

    @PATCH("customer/{customerId}")
    fun deleteCustomerInfo(
        @Path("customerId") customerId: String
    ): Call<Int>

}